/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "cafbcr-compute-task-tag.h"
#include "ns3/log.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("CafbcrComputeTaskTag");
NS_OBJECT_ENSURE_REGISTERED(CafbcrComputeTaskTag);

TypeId CafbcrComputeTaskTag::GetTypeId()
{
  static TypeId tid = TypeId("ns3::CafbcrComputeTaskTag")
    .SetParent<Tag>()
    .SetGroupName("Cafbcr")
    .AddConstructor<CafbcrComputeTaskTag>();
  return tid;
}

TypeId CafbcrComputeTaskTag::GetInstanceTypeId() const
{
  return GetTypeId();
}

CafbcrComputeTaskTag::CafbcrComputeTaskTag()
  : m_workload(1.0),
    m_dataSize(1024),
    m_deadline(5.0),
    m_sourceNodeId(0xFFFF),
    m_taskId(0)
{
}

CafbcrComputeTaskTag::~CafbcrComputeTaskTag()
{
}

uint32_t CafbcrComputeTaskTag::GetSerializedSize() const
{
  // double(8) + uint32(4) + double(8) + uint16(2) + uint32(4) = 26 bytes
  return 26;
}

void CafbcrComputeTaskTag::Serialize(TagBuffer i) const
{
  // 使用两个 uint32 拼接来存储 double (防止某些版本 TagBuffer 不支持 WriteU64)
  union { double d; uint64_t u; } workloadConv;
  workloadConv.d = m_workload;
  i.WriteU32(static_cast<uint32_t>(workloadConv.u & 0xFFFFFFFF));
  i.WriteU32(static_cast<uint32_t>(workloadConv.u >> 32));

  i.WriteU32(m_dataSize);

  union { double d; uint64_t u; } deadlineConv;
  deadlineConv.d = m_deadline;
  i.WriteU32(static_cast<uint32_t>(deadlineConv.u & 0xFFFFFFFF));
  i.WriteU32(static_cast<uint32_t>(deadlineConv.u >> 32));

  i.WriteU16(m_sourceNodeId);
  i.WriteU32(m_taskId);
}

void CafbcrComputeTaskTag::Deserialize(TagBuffer i)
{
  union { double d; uint64_t u; } workloadConv;
  uint32_t workloadLo = i.ReadU32();
  uint32_t workloadHi = i.ReadU32();
  workloadConv.u = workloadLo | (static_cast<uint64_t>(workloadHi) << 32);
  m_workload = workloadConv.d;

  m_dataSize = i.ReadU32();

  union { double d; uint64_t u; } deadlineConv;
  uint32_t deadlineLo = i.ReadU32();
  uint32_t deadlineHi = i.ReadU32();
  deadlineConv.u = deadlineLo | (static_cast<uint64_t>(deadlineHi) << 32);
  m_deadline = deadlineConv.d;

  m_sourceNodeId = i.ReadU16();
  m_taskId = i.ReadU32();
}

void CafbcrComputeTaskTag::Print(std::ostream& os) const
{
  os << "ComputeTask[wl=" << m_workload << " GF, data=" << m_dataSize
     << " B, ddl=" << m_deadline << " s, src=" << m_sourceNodeId
     << ", id=" << m_taskId << "]";
}

// ========== Getter/Setter 实现 ==========

void CafbcrComputeTaskTag::SetWorkload(double gflops)
{
  m_workload = gflops;
}

double CafbcrComputeTaskTag::GetWorkload() const
{
  return m_workload;
}

void CafbcrComputeTaskTag::SetDataSize(uint32_t bytes)
{
  m_dataSize = bytes;
}

uint32_t CafbcrComputeTaskTag::GetDataSize() const
{
  return m_dataSize;
}

void CafbcrComputeTaskTag::SetDeadline(double seconds)
{
  m_deadline = seconds;
}

double CafbcrComputeTaskTag::GetDeadline() const
{
  return m_deadline;
}

void CafbcrComputeTaskTag::SetSourceNodeId(uint16_t id)
{
  m_sourceNodeId = id;
}

uint16_t CafbcrComputeTaskTag::GetSourceNodeId() const
{
  return m_sourceNodeId;
}

void CafbcrComputeTaskTag::SetTaskId(uint32_t id)
{
  m_taskId = id;
}

uint32_t CafbcrComputeTaskTag::GetTaskId() const
{
  return m_taskId;
}

} // namespace ns3
