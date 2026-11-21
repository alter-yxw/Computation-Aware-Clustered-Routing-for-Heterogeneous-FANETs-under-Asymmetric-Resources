/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef CAFBCR_COMPUTE_TASK_HEADER_H
#define CAFBCR_COMPUTE_TASK_HEADER_H
#include "cafbcr-compute-task-tag.h"
#include "ns3/header.h"
#include "ns3/nstime.h"

namespace ns3::cafbcr {
// Real application bytes, independent of the simulator's routing-tag cache.
class ComputeTaskHeader : public Header
{
public:
  static constexpr uint32_t SIZE = 32;
  static constexpr const char* SCHEMA = "cafbcr-task-wire-v1";
  enum Kind : uint8_t { REQUEST = 1, COMPLETION = 2 };
  static TypeId GetTypeId();
  TypeId GetInstanceTypeId() const override;
  uint32_t GetSerializedSize() const override;
  void Serialize(Buffer::Iterator start) const override;
  uint32_t Deserialize(Buffer::Iterator start) override;
  void Print(std::ostream& os) const override;
  void SetTask(const CafbcrComputeTaskTag& task, Time deadline, Kind kind);
  CafbcrComputeTaskTag ToTag() const;
  bool IsValid() const;
  Kind GetKind() const { return static_cast<Kind>(m_kind); }
  uint32_t GetPayloadBytes() const { return m_payload; }
  Time GetDeadline() const { return NanoSeconds(static_cast<int64_t>(m_deadlineNs)); }
private:
  uint16_t m_magic = 0xcafc, m_length = SIZE, m_source = 0xffff;
  uint8_t m_version = 1, m_kind = REQUEST;
  uint32_t m_task = 0, m_payload = 0;
  double m_workload = 0;
  uint64_t m_deadlineNs = 0;
  bool m_complete = true;
};
} // namespace ns3::cafbcr
#endif
