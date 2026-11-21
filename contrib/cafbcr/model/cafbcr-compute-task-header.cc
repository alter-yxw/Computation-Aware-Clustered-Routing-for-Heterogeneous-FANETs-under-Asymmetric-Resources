/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "cafbcr-compute-task-header.h"
#include <cmath>
#include <cstring>
#include <limits>

namespace ns3::cafbcr {
NS_OBJECT_ENSURE_REGISTERED(ComputeTaskHeader);
TypeId ComputeTaskHeader::GetTypeId()
{
  static TypeId tid = TypeId("ns3::cafbcr::ComputeTaskHeader")
    .SetParent<Header>().SetGroupName("Cafbcr").AddConstructor<ComputeTaskHeader>();
  return tid;
}
TypeId ComputeTaskHeader::GetInstanceTypeId() const { return GetTypeId(); }
uint32_t ComputeTaskHeader::GetSerializedSize() const { return SIZE; }
void ComputeTaskHeader::SetTask(const CafbcrComputeTaskTag& task, Time deadline, Kind kind)
{
  m_source = task.GetSourceNodeId(); m_task = task.GetTaskId(); m_kind = kind;
  m_payload = kind == REQUEST ? task.GetDataSize() : 8;
  m_workload = kind == REQUEST ? task.GetWorkload() : 0;
  m_deadlineNs = static_cast<uint64_t>(deadline.GetNanoSeconds());
}
void ComputeTaskHeader::Serialize(Buffer::Iterator i) const
{
  static_assert(sizeof(double) == 8 && std::numeric_limits<double>::is_iec559);
  uint64_t workload;
  std::memcpy(&workload, &m_workload, sizeof(workload));
  i.WriteHtonU16(m_magic); i.WriteU8(m_version); i.WriteU8(m_kind);
  i.WriteHtonU16(m_length); i.WriteHtonU16(m_source);
  i.WriteHtonU32(m_task); i.WriteHtonU32(m_payload);
  i.WriteHtonU64(workload); i.WriteHtonU64(m_deadlineNs);
}
uint32_t ComputeTaskHeader::Deserialize(Buffer::Iterator i)
{
  m_complete = i.GetRemainingSize() >= SIZE;
  if (!m_complete) return 0;
  m_magic = i.ReadNtohU16(); m_version = i.ReadU8(); m_kind = i.ReadU8();
  m_length = i.ReadNtohU16(); m_source = i.ReadNtohU16();
  m_task = i.ReadNtohU32(); m_payload = i.ReadNtohU32();
  uint64_t workload = i.ReadNtohU64();
  std::memcpy(&m_workload, &workload, sizeof(workload));
  m_deadlineNs = i.ReadNtohU64();
  return IsValid() ? SIZE : 0;
}
bool ComputeTaskHeader::IsValid() const
{
  return m_complete && m_magic == 0xcafc && m_version == 1 && m_length == SIZE &&
    m_source != 0xffff && m_deadlineNs <= static_cast<uint64_t>(std::numeric_limits<int64_t>::max()) &&
    std::isfinite(m_workload) &&
    ((m_kind == REQUEST && m_workload > 0 && m_payload > 0) ||
     (m_kind == COMPLETION && m_workload == 0 && m_payload == 8));
}
CafbcrComputeTaskTag ComputeTaskHeader::ToTag() const
{
  CafbcrComputeTaskTag task;
  task.SetSourceNodeId(m_source); task.SetTaskId(m_task);
  task.SetWorkload(m_workload); task.SetDataSize(m_kind == REQUEST ? m_payload : 0);
  task.SetDeadline(GetDeadline().GetSeconds());
  return task;
}
void ComputeTaskHeader::Print(std::ostream& os) const
{
  os << "TaskWire[v=" << unsigned(m_version) << ",kind=" << unsigned(m_kind)
     << ",src=" << m_source << ",id=" << m_task << ",payload=" << m_payload << ']';
}
} // namespace ns3::cafbcr
