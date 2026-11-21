/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "cafbcr-cluster-hop-tag.h"
#include "ns3/log.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("CafbcrClusterHopTag");
NS_OBJECT_ENSURE_REGISTERED(CafbcrClusterHopTag);

TypeId CafbcrClusterHopTag::GetTypeId()
{
  static TypeId tid = TypeId("ns3::CafbcrClusterHopTag")
    .SetParent<Tag>()
    .SetGroupName("Cafbcr")
    .AddConstructor<CafbcrClusterHopTag>();
  return tid;
}

TypeId CafbcrClusterHopTag::GetInstanceTypeId() const
{
  return GetTypeId();
}

CafbcrClusterHopTag::CafbcrClusterHopTag()
  : m_chId(0xFFFF)
{
}

uint32_t CafbcrClusterHopTag::GetSerializedSize() const
{
  return 2;
}

void CafbcrClusterHopTag::Serialize(TagBuffer i) const
{
  i.WriteU16(m_chId);
}

void CafbcrClusterHopTag::Deserialize(TagBuffer i)
{
  m_chId = i.ReadU16();
}

void CafbcrClusterHopTag::Print(std::ostream& os) const
{
  os << "ClusterHop[lastCH=" << m_chId << "]";
}

} // namespace ns3
