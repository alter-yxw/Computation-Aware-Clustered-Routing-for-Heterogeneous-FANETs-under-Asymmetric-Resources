/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef CAFBCR_PHY_HEADER_BOUNDARY_H
#define CAFBCR_PHY_HEADER_BOUNDARY_H

#include "ns3/dsss-phy.h"
#include "ns3/erp-ofdm-phy.h"
#include "ns3/simulator.h"
#include "ns3/wifi-phy-state-helper.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/yans-wifi-phy.h"

namespace ns3
{
class CafbcrHeaderBoundaryErpPhy : public ErpOfdmPhy
{
  public:
    void StartReceivePreamble(Ptr<const WifiPpdu> ppdu,
                             RxPowerWattPerChannelBand& powers,
                             Time duration) override;
};

class CafbcrHeaderBoundaryDsssPhy : public DsssPhy
{
  public:
    void StartReceivePreamble(Ptr<const WifiPpdu> ppdu,
                             RxPowerWattPerChannelBand& powers,
                             Time duration) override;
};

// Local to a binary that selects this helper. The shared Wi-Fi library and
// ordinary YansWifiPhyHelper retain their original implementation.
class CafbcrHeaderBoundaryYansWifiPhy : public YansWifiPhy
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::CafbcrHeaderBoundaryYansWifiPhy")
                                .SetParent<YansWifiPhy>()
                                .SetGroupName("Wifi")
                                .AddConstructor<CafbcrHeaderBoundaryYansWifiPhy>();
        return tid;
    }

    void ConfigureStandard(WifiStandard standard) override
    {
        YansWifiPhy::ConfigureStandard(standard);
        auto it = m_phyEntities.find(WIFI_MOD_CLASS_ERP_OFDM);
        // ConfigureStandard can recurse while setting the operating channel.
        if (it != m_phyEntities.end() && !DynamicCast<CafbcrHeaderBoundaryErpPhy>(it->second))
        {
            auto replacement = Create<CafbcrHeaderBoundaryErpPhy>();
            replacement->SetOwner(this);
            it->second = replacement;
        }
        // 802.11g also receives DSSS/HR-DSSS frames. Dispatch is by the
        // incoming PPDU's modulation, even when the pending header is ERP.
        // Preserve the native shared entity for the two DSSS map entries.
        auto dsss = m_phyEntities.find(WIFI_MOD_CLASS_DSSS);
        if (dsss != m_phyEntities.end() &&
            !DynamicCast<CafbcrHeaderBoundaryDsssPhy>(dsss->second))
        {
            auto replacement = Create<CafbcrHeaderBoundaryDsssPhy>();
            replacement->SetOwner(this);
            dsss->second = replacement;
            auto hrDsss = m_phyEntities.find(WIFI_MOD_CLASS_HR_DSSS);
            if (hrDsss != m_phyEntities.end())
            {
                hrDsss->second = replacement;
            }
        }
    }

    bool DeferAtDueHeader(Ptr<PhyEntity> entity,
                         Ptr<const WifiPpdu> ppdu,
                         const RxPowerWattPerChannelBand& powers,
                         Time duration)
    {
        if (!m_currentEvent || !m_state->IsStateIdle() || m_endPhyRxEvent.IsExpired() ||
            m_endPhyRxEvent.GetTs() != static_cast<uint64_t>(Simulator::Now().GetTimeStep()))
        {
            return false;
        }
        // The field-end event is already queued at Now with an older UID than
        // this new callback. Let that native decision run first. Do not add the
        // incoming signal twice, advance time, or clear reception state.
        ++m_deferrals;
        Simulator::ScheduleNow([entity, ppdu, powersCopy = RxPowerWattPerChannelBand(powers), duration]() mutable {
            entity->StartReceivePreamble(ppdu, powersCopy, duration);
        });
        return true;
    }

    uint64_t GetHeaderBoundaryDeferrals() const
    {
        return m_deferrals;
    }

  private:
    uint64_t m_deferrals{0};
};

inline void
CafbcrHeaderBoundaryErpPhy::StartReceivePreamble(Ptr<const WifiPpdu> ppdu,
                                               RxPowerWattPerChannelBand& powers,
                                               Time duration)
{
    auto owner = DynamicCast<CafbcrHeaderBoundaryYansWifiPhy>(m_wifiPhy);
    if (owner && owner->DeferAtDueHeader(this, ppdu, powers, duration))
    {
        return;
    }
    ErpOfdmPhy::StartReceivePreamble(ppdu, powers, duration);
}

inline void
CafbcrHeaderBoundaryDsssPhy::StartReceivePreamble(Ptr<const WifiPpdu> ppdu,
                                                RxPowerWattPerChannelBand& powers,
                                                Time duration)
{
    auto owner = DynamicCast<CafbcrHeaderBoundaryYansWifiPhy>(m_wifiPhy);
    if (owner && owner->DeferAtDueHeader(this, ppdu, powers, duration))
    {
        return;
    }
    DsssPhy::StartReceivePreamble(ppdu, powers, duration);
}

class CafbcrHeaderBoundaryYansWifiPhyHelper : public YansWifiPhyHelper
{
  public:
    explicit CafbcrHeaderBoundaryYansWifiPhyHelper(bool enabled = true)
    {
        if (enabled)
        {
            m_phys.at(0).SetTypeId(CafbcrHeaderBoundaryYansWifiPhy::GetTypeId());
        }
    }
};
} // namespace ns3
#endif
