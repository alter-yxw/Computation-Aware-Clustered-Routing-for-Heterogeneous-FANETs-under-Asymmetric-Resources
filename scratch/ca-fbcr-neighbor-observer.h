/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef CA_FBCR_NEIGHBOR_OBSERVER_H
#define CA_FBCR_NEIGHBOR_OBSERVER_H

#include "ca-fbcr-task-observer.h"
#include "ns3/cafbcr-routing.h"
#include <cmath>

namespace ns3::cafbcr {

// Passive receipt snapshots; never purges neighbors or supplies selector input.
class NeighborComputeWriter
{
public:
  static constexpr const char* SCHEMA = "cafbcr-neighbor-compute-v1";
  NeighborComputeWriter(const std::string& directory, Time cutoff) : m_cutoff(cutoff)
  {
    auto path = std::filesystem::path(directory) / "neighbor-compute.jsonl";
    NS_ABORT_MSG_IF(std::filesystem::exists(path), "Neighbor observations already exist");
    m_output.open(path);
    NS_ABORT_MSG_IF(!m_output, "Cannot open neighbor observations");
    m_output << std::setprecision(17);
  }
  ~NeighborComputeWriter()
  {
    for (auto routing : m_routing) {
      routing->TraceDisconnectWithoutContext("NeighborComputeState",
        MakeCallback(&NeighborComputeWriter::Observe, this));
    }
  }
  void Attach(Ptr<RoutingProtocol> routing)
  {
    NS_ABORT_MSG_IF(!routing || !routing->TraceConnectWithoutContext("NeighborComputeState",
      MakeCallback(&NeighborComputeWriter::Observe, this)), "Missing CA neighbor trace");
    m_routing.push_back(routing);
  }
  void Finalize()
  {
    m_output.flush();
    NS_ABORT_MSG_IF(!m_output, "Cannot flush neighbor observations");
    m_finalized = true;
  }
private:
  static std::string TimeValue(const std::optional<Time>& value)
  { return value ? std::to_string(value->GetNanoSeconds()) : "null"; }
  static std::string Number(const std::optional<double>& value)
  {
    if (!value || !std::isfinite(*value)) return "null";
    std::ostringstream out; out << std::setprecision(17) << *value; return out.str();
  }
  void Observe(const NeighborComputeEvent& event)
  {
    if (event.time > m_cutoff) return;
    NS_ABORT_MSG_IF(m_finalized, "Neighbor trace after finalization");
    std::ostringstream ip; ip << event.neighborAddress;
    m_output << "{\"schema\":" << TaskObservationWriter::Quote(SCHEMA)
      << ",\"observer_node\":" << event.observerNodeId
      << ",\"neighbor_node\":" << event.neighborNodeId
      << ",\"neighbor_ip\":" << TaskObservationWriter::Quote(ip.str())
      << ",\"time_ns\":" << event.time.GetNanoSeconds()
      << ",\"message\":" << TaskObservationWriter::Quote(event.message)
      << ",\"neighbor_updated_ns\":" << event.neighborUpdatedAt.GetNanoSeconds()
      << ",\"neighbor_lifetime_ns\":" << event.neighborLifetime.GetNanoSeconds()
      << ",\"residual_energy\":" << Number(event.residualEnergy)
      << ",\"compute_capacity_gflops\":" << Number(event.computeCapacity)
      << ",\"current_load\":" << Number(event.currentLoad)
      << ",\"compute_state\":" << TaskObservationWriter::Quote(event.observation.StatusName())
      << ",\"compute_received_ns\":" << TimeValue(event.observation.receivedAt)
      << ",\"compute_expires_ns\":" << TimeValue(event.observation.expiresAt)
      << "}\n";
  }
  Time m_cutoff;
  std::ofstream m_output;
  bool m_finalized = false;
  std::vector<Ptr<RoutingProtocol>> m_routing;
};
} // namespace ns3::cafbcr
#endif
