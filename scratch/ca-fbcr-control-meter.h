/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef CA_FBCR_CONTROL_METER_H
#define CA_FBCR_CONTROL_METER_H

#include "ns3/abort.h"
#include "ns3/callback.h"
#include "ns3/ipv4-l3-protocol.h"
#include "ns3/loopback-net-device.h"
#include "ns3/packet.h"
#include <algorithm>
#include <cstdint>

namespace ns3::cafbcr {

// CA-entrypoint-only common control meter. One event is a non-loopback IPv4 Tx
// packet whose UDP source port is the routing protocol's control port; bytes
// are the IPv4 total length (IPv4 + UDP + protocol payload), excluding L2.
// Forwarded control is excluded, so each node reports only locally originated
// control. The IPv4 Tx trace fires before ARP/MAC transmission, so a counted
// packet may still be dropped by ARP or MAC/PHY; MAC retries are not counted.
class ControlCostMeter
{
public:
  explicit ControlCostMeter(uint16_t controlPort) : m_controlPort(controlPort) {}

  void Attach(Ptr<Node> node)
  {
    Ptr<Ipv4L3Protocol> ip = node->GetObject<Ipv4L3Protocol>();
    NS_ABORT_MSG_IF(!ip, "ControlCostMeter requires Ipv4L3Protocol");
    const bool connected =
      ip->TraceConnectWithoutContext("Tx", MakeBoundCallback(&ControlCostMeter::Tx, this));
    NS_ABORT_MSG_IF(!connected, "ControlCostMeter could not connect the IPv4 Tx trace");
  }

  uint64_t Packets() const { return m_packets; }
  uint64_t Bytes() const { return m_bytes; }

private:
  static void Tx(ControlCostMeter* self, Ptr<const Packet> packet, Ptr<Ipv4> ip, uint32_t interface)
  {
    if (DynamicCast<LoopbackNetDevice>(ip->GetNetDevice(interface))) return;
    uint8_t header[28] = {};
    const uint32_t copied =
      packet->CopyData(header, std::min<uint32_t>(sizeof(header), packet->GetSize()));
    if (copied < sizeof(header) || (header[0] >> 4) != 4) return;
    const uint32_t ihl = (header[0] & 0x0f) * 4;
    if (ihl < 20 || copied < ihl + 8 || header[9] != 17) return;
    const uint16_t sourcePort = (static_cast<uint16_t>(header[ihl]) << 8) | header[ihl + 1];
    if (sourcePort != self->m_controlPort) return;
    const uint32_t sourceValue = (static_cast<uint32_t>(header[12]) << 24) |
                                 (static_cast<uint32_t>(header[13]) << 16) |
                                 (static_cast<uint32_t>(header[14]) << 8) | header[15];
    if (ip->GetInterfaceForAddress(Ipv4Address(sourceValue)) < 0) return;
    ++self->m_packets;
    self->m_bytes += packet->GetSize();
  }

  uint16_t m_controlPort;
  uint64_t m_packets = 0;
  uint64_t m_bytes = 0;
};
} // namespace ns3::cafbcr
#endif
