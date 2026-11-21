/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef CAFBCR_CLUSTER_HOP_TAG_H
#define CAFBCR_CLUSTER_HOP_TAG_H

#include "ns3/tag.h"

namespace ns3 {

/**
 * \brief Inter-cluster forwarding loop-prevention tag.
 *
 * A packet forwarded by a ClusterHead through greedy/perimeter forwarding is
 * stamped with that CH's node ID. Before applying forwarding strategy A3, a
 * downstream member checks the tag: if the previous CH is its own CH, sending
 * the packet back to A3 would create a ping-pong loop. The member then skips
 * A3 and lets greedy/perimeter forwarding terminate the detour.
 *
 * Tag size: 2 bytes (uint16_t lastClusterHeadId).
 */
class CafbcrClusterHopTag : public Tag
{
public:
  static TypeId GetTypeId();
  TypeId GetInstanceTypeId() const override;

  CafbcrClusterHopTag();

  void Serialize(TagBuffer i) const override;
  void Deserialize(TagBuffer i) override;
  uint32_t GetSerializedSize() const override;
  void Print(std::ostream& os) const override;

  void     SetClusterHeadId(uint16_t id) { m_chId = id; }
  uint16_t GetClusterHeadId() const      { return m_chId; }

private:
  uint16_t m_chId;  ///< Node ID of the last CH that forwarded this packet (0xFFFF = unstamped).
};

} // namespace ns3

#endif // CAFBCR_CLUSTER_HOP_TAG_H
