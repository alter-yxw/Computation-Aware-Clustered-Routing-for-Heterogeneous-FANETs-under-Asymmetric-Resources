/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef CAFBCR_COMPUTE_TASK_TAG_H
#define CAFBCR_COMPUTE_TASK_TAG_H

#include "ns3/tag.h"

namespace ns3 {

/**
 * \brief Compute-task tag.
 *
 * Attached to task packets so the routing protocol can make
 * compute-aware forwarding decisions (CAF).
 *
 * Tag size: 26 bytes
 * - double  workload   (8) - Workload in GFLOP.
 * - uint32  dataSize   (4) - Data size in bytes.
 * - double  deadline   (8) - Relative deadline in seconds.
 * - uint16  sourceId   (2) - Source node ID.
 * - uint32  taskId     (4) - Task identifier.
 */
class CafbcrComputeTaskTag : public Tag
{
public:
  static TypeId GetTypeId();
  TypeId GetInstanceTypeId() const override;

  CafbcrComputeTaskTag();
  virtual ~CafbcrComputeTaskTag();

  void Serialize(TagBuffer i) const override;
  void Deserialize(TagBuffer i) override;
  uint32_t GetSerializedSize() const override;
  void Print(std::ostream& os) const override;

  // ---- Task attribute accessors ----
  void      SetWorkload(double gflops);
  double    GetWorkload() const;

  void      SetDataSize(uint32_t bytes);
  uint32_t  GetDataSize() const;

  void      SetDeadline(double seconds);
  double    GetDeadline() const;

  void      SetSourceNodeId(uint16_t id);
  uint16_t  GetSourceNodeId() const;

  void      SetTaskId(uint32_t id);
  uint32_t  GetTaskId() const;

private:
  double    m_workload;       // Workload in GFLOP.
  uint32_t  m_dataSize;       // Data size in bytes.
  double    m_deadline;       // Relative deadline in seconds from task emission.
  uint16_t  m_sourceNodeId;   // Source node ID.
  uint32_t  m_taskId;         // Globally unique task ID.
};

} // namespace ns3

#endif // CAFBCR_COMPUTE_TASK_TAG_H
