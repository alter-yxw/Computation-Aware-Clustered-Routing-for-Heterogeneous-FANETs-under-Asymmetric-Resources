/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef FANET_ATTACK_LAYER_H
#define FANET_ATTACK_LAYER_H
//
// Protocol-agnostic packet-drop layer (AttackLayer).
//
// Purpose: exercise every routing protocol under the same drop attack without
// writing protocol-specific injection code. The layer intercepts the Wi-Fi
// device receive callback and drops selected packets before they reach the
// upper layers; accepted IPv4/ARP/IPv6 packets go to TrafficControlLayer.
// Install it after InternetStackHelper and IP interface setup.
//
// Scope: AttackLayer implements packet dropping only. Wormhole, routing
// forgery and selective forwarding remain protocol-specific and are not
// implemented here.
//
// Usage:
//   AttackLayer al;
//   al.Install(devices, attackerIds, AttackLayer::DROP_ONLY, 1.0, 30.0);
//   Simulator::Run();
//   uint64_t dropped = al.GetDropped(nodeId);
//
#include "ns3/core-module.h"
#include "ns3/net-device-container.h"
#include "ns3/net-device.h"
#include "ns3/node.h"
#include "ns3/packet.h"
#include "ns3/address.h"
#include "ns3/ipv4.h"
#include "ns3/ipv6.h"
#include "ns3/traffic-control-layer.h"
#include "ns3/wifi-net-device.h"

#include <cstdint>
#include <map>
#include <random>
#include <set>
#include <utility>

namespace fanet_common {

using NetDeviceContainer = ns3::NetDeviceContainer;
using NetDevice = ns3::NetDevice;
template <typename T>
using Ptr = ns3::Ptr<T>;
using Node = ns3::Node;
using Address = ns3::Address;
using Simulator = ns3::Simulator;

class AttackLayer {
public:
  enum Profile {
    NONE,        // No callback installed; all packets pass through.
    DROP_ONLY    // Malicious nodes drop packets with a given probability.
  };

  AttackLayer() = default;

  // Install AttackLayer on the Wi-Fi receive callbacks of the attacker nodes.
  // Supports the standard InternetStackHelper receive chain only; custom
  // receive callbacks are not covered. The object must outlive
  // Simulator::Destroy().
  // devices: all NetDevices.
  // attackers: set of attacker node IDs.
  // profile: see the enum above.
  // dropProb: per-packet drop probability in [0, 1]; DROP_ONLY only.
  // attackStart: attack start time in seconds; earlier packets pass through.
  void Install(NetDeviceContainer devices,
               std::set<uint32_t> attackers,
               Profile profile,
               double dropProb,
               double attackStart);

  uint64_t GetDropped(uint32_t nodeId) const
  {
    auto it = m_dropped.find(nodeId);
    return it == m_dropped.end() ? 0 : it->second;
  }

  uint64_t GetForwarded(uint32_t nodeId) const
  {
    auto it = m_forwarded.find(nodeId);
    return it == m_forwarded.end() ? 0 : it->second;
  }

  uint32_t GetAttackerCount() const { return static_cast<uint32_t>(m_attackers.size()); }

  Profile GetProfile() const { return m_profile; }
  double GetDropProb() const { return m_dropProb; }
  double GetAttackStart() const { return m_attackStart; }

private:
  // A separate lambda is built per attacker during Install, capturing self and
  // nodeId, instead of a static callback with a complex MakeBoundCallback.
  // MAC receive callback signature:
  // bool(Ptr<NetDevice>, Ptr<const Packet>, uint16_t, const Address&).

  Profile m_profile{NONE};
  double m_dropProb{1.0};
  double m_attackStart{0.0};
  std::set<uint32_t> m_attackers;
  std::map<uint32_t, uint64_t> m_dropped;
  std::map<uint32_t, uint64_t> m_forwarded;
  std::mt19937_64 m_rng;
};

// Out-of-class definition so all ns3 types are declared before use.
inline void AttackLayer::Install(NetDeviceContainer devices,
                                 std::set<uint32_t> attackers,
                                 Profile profile,
                                 double dropProb,
                                 double attackStart)
{
  m_profile = profile;
  m_dropProb = dropProb;
  m_attackStart = attackStart;
  m_attackers = std::move(attackers);

  if (m_profile == NONE || m_attackers.empty())
  {
    // NONE profile or no attackers: install no callback.
    return;
  }

  // Initialize counters.
  for (uint32_t nodeId : m_attackers)
  {
    m_dropped[nodeId] = 0;
    m_forwarded[nodeId] = 0;
  }

  // Shared RNG for all attackers, seeded deterministically.
  m_rng.seed(0xC0FFEEu);

  // Install a callback on each attacker device.
  for (uint32_t i = 0; i < devices.GetN(); ++i)
  {
    auto dev = devices.Get(i);
    uint32_t nodeId = dev->GetNode()->GetId();
    if (m_attackers.find(nodeId) == m_attackers.end())
    {
      continue;
    }

    // Stock ns-3.40 has no WifiNetDevice::GetReceiveCallback(). For standard
    // InternetStackHelper devices, Node dispatches IPv4/ARP/IPv6 to this public
    // TrafficControlLayer entry point. Use the same destination and packet type
    // as Node::NonPromiscReceiveFromDevice; do not swallow accepted packets.
    auto wifi = ns3::DynamicCast<ns3::WifiNetDevice>(dev);
    NS_ABORT_MSG_IF(!wifi, "AttackLayer requires a WifiNetDevice receive callback");
    auto node = dev->GetNode();
    auto tc = node->GetObject<ns3::TrafficControlLayer>();
    auto ipv4 = node->GetObject<ns3::Ipv4>();
    auto ipv6 = node->GetObject<ns3::Ipv6>();
    const bool hasIpv4 = ipv4 && ipv4->GetInterfaceForDevice(dev) >= 0;
    const bool hasIpv6 = ipv6 && ipv6->GetInterfaceForDevice(dev) >= 0;
    NS_ABORT_MSG_IF(!tc || (!hasIpv4 && !hasIpv6),
                    "Install AttackLayer after InternetStackHelper and IP interface setup");
    auto upstream = [tc, hasIpv4, hasIpv6](ns3::Ptr<NetDevice> device,
                                         ns3::Ptr<const ns3::Packet> packet,
                                         uint16_t protocol, const Address& sender) -> bool {
      // EtherTypes handled by the standard Internet stack. Node also ignores
      // protocols for which no handler was installed on this device.
      if ((hasIpv4 && (protocol == 0x0800 || protocol == 0x0806)) ||
          (hasIpv6 && protocol == 0x86dd))
      {
        tc->Receive(device, packet, protocol, sender, device->GetAddress(),
                    NetDevice::PacketType(0));
        return true;
      }
      return false;
    };
    auto self = this;
    auto cb = ns3::Callback<bool, ns3::Ptr<NetDevice>, ns3::Ptr<const ns3::Packet>,
                            uint16_t, const Address&>(
        [self, nodeId, upstream](ns3::Ptr<NetDevice> device, ns3::Ptr<const ns3::Packet> packet,
                       uint16_t protocol, const Address& sender) -> bool {
          if (self->m_profile == NONE)
          {
            return upstream(device, packet, protocol, sender);
          }
          if (Simulator::Now().GetSeconds() < self->m_attackStart)
          {
            return upstream(device, packet, protocol, sender);
          }
          if (self->m_profile == DROP_ONLY)
          {
            std::uniform_real_distribution<double> dist(0.0, 1.0);
            double r = dist(self->m_rng);
            if (r < self->m_dropProb)
            {
              ++self->m_dropped[nodeId];
              return false;
            }
          }
          ++self->m_forwarded[nodeId];
          return upstream(device, packet, protocol, sender);
        });
    (void)dev->SetReceiveCallback(cb);
  }
}

} // namespace fanet_common

#endif /* FANET_ATTACK_LAYER_H */
