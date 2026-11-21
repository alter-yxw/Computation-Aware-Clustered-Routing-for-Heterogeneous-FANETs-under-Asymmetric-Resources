/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef CA_FBCR_QUEUE_OBSERVER_H
#define CA_FBCR_QUEUE_OBSERVER_H

#include "ca-fbcr-task-observer.h"

namespace ns3::cafbcr {

// Passive output only. Queue truth is published by the local server application.
class QueueLoadWriter
{
public:
  static constexpr const char* SCHEMA = "cafbcr-queue-load-v1";
  QueueLoadWriter(const std::string& directory, Time cutoff) : m_cutoff(cutoff)
  {
    auto path = std::filesystem::path(directory) / "queue-load.jsonl";
    NS_ABORT_MSG_IF(std::filesystem::exists(path), "Queue output already exists");
    m_output.open(path);
    NS_ABORT_MSG_IF(!m_output, "Cannot open queue load observations");
    m_output << std::setprecision(17);
  }

  ~QueueLoadWriter()
  {
    for (auto app : m_servers) {
      app->TraceDisconnectWithoutContext("QueueState", MakeCallback(&QueueLoadWriter::Observe, this));
    }
  }

  void Attach(Ptr<ComputeTaskServerApp> app)
  {
    NS_ABORT_MSG_IF(!app->TraceConnectWithoutContext("QueueState",
      MakeCallback(&QueueLoadWriter::Observe, this)), "Missing server queue trace");
    m_servers.push_back(app);
  }

  void Finalize()
  {
    m_output.flush();
    NS_ABORT_MSG_IF(!m_output, "Cannot flush queue load observations");
    m_finalized = true;
  }

private:
  void Observe(const ComputeQueueState& state)
  {
    NS_ABORT_MSG_IF(m_finalized, "Queue trace after output finalization");
    m_output << "{\"schema\":" << TaskObservationWriter::Quote(SCHEMA)
      << ",\"node\":" << state.nodeId << ",\"time_ns\":" << state.time.GetNanoSeconds()
      << ",\"event\":" << TaskObservationWriter::Quote(state.event)
      << ",\"waiting\":" << state.waiting << ",\"capacity\":" << state.capacity
      << ",\"busy\":" << (state.busy ? "true" : "false")
      << ",\"running\":" << (state.running ? "true" : "false")
      << ",\"within_observation_window\":" << (state.time <= m_cutoff ? "true" : "false")
      << ",\"synchronized\":" << (state.localLoad ? "true" : "false")
      << ",\"local_waiting\":";
    if (state.localWaiting) m_output << *state.localWaiting; else m_output << "null";
    m_output << ",\"local_capacity\":";
    if (state.localCapacity) m_output << *state.localCapacity; else m_output << "null";
    m_output << ",\"local_load\":";
    if (state.localLoad) m_output << *state.localLoad; else m_output << "null";
    m_output << "}\n";
  }

  Time m_cutoff;
  std::ofstream m_output;
  bool m_finalized = false;
  std::vector<Ptr<ComputeTaskServerApp>> m_servers;
};
} // namespace ns3::cafbcr
#endif
