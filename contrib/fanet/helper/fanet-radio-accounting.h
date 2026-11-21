/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef FANET_RADIO_ACCOUNTING_H
#define FANET_RADIO_ACCOUNTING_H
#include "ns3/net-device-container.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-phy.h"
#include "ns3/wifi-phy-state-helper.h"
#include <cstdint>
#include <ostream>
#include <vector>

// Common physical accounting across routing protocols. Includes retransmissions,
// ACKs and protocol packets; excludes PacketTag metadata and ideal controller calls.
namespace fanet_common {

class FidelityRadioAccounting
{
public:
 void Attach(const ns3::NetDeviceContainer& devices)
 {
  for(auto it=devices.Begin();it!=devices.End();++it)
  {
   auto wifi=ns3::DynamicCast<ns3::WifiNetDevice>(*it);
   if(!wifi) continue;
   for(auto phy:wifi->GetPhys())
   {
    phy->TraceConnectWithoutContext("PhyTxBegin",ns3::MakeCallback(&FidelityRadioAccounting::Tx,this));
    phy->GetState()->TraceConnectWithoutContext("State",ns3::MakeCallback(&FidelityRadioAccounting::State,this));
    m_phys.push_back(phy);
   }
  }
 }
 void Print(std::ostream& os) const
 {
  os<<"RADIO_ACCOUNTING phys="<<m_phys.size()<<" tx_frames="<<m_frames
    <<" tx_packet_bytes="<<m_bytes<<" summed_tx_airtime_s="<<m_airtime
    <<" scope=all_wifi_transmissions_including_retries ideal_control_bytes=unmodeled\n";
 }
 void Disconnect()
 {
  for(auto phy:m_phys)
  {
   phy->TraceDisconnectWithoutContext("PhyTxBegin",ns3::MakeCallback(&FidelityRadioAccounting::Tx,this));
   phy->GetState()->TraceDisconnectWithoutContext("State",ns3::MakeCallback(&FidelityRadioAccounting::State,this));
  }
  m_phys.clear();
 }
private:
 void Tx(ns3::Ptr<const ns3::Packet> p,double) { ++m_frames; m_bytes+=p->GetSize(); }
 void State(ns3::Time,ns3::Time duration,WifiPhyState state)
 { if(state==WifiPhyState::TX) m_airtime+=duration.GetSeconds(); }
 std::vector<ns3::Ptr<ns3::WifiPhy>> m_phys;
 uint64_t m_frames=0,m_bytes=0;
 double m_airtime=0;
};
} // namespace fanet_common

#endif // FANET_RADIO_ACCOUNTING_H
