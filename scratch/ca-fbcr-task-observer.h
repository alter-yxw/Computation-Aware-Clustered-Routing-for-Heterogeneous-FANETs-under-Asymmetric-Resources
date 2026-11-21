/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef CA_FBCR_TASK_OBSERVER_H
#define CA_FBCR_TASK_OBSERVER_H

#include "ns3/cafbcr-compute-task-app.h"
#include "ns3/abort.h"
#include "ns3/simulator.h"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <vector>

namespace ns3::cafbcr {

// Scenario-local, passive collector. No reference to it is used by task selection/routing.
class TaskObservationWriter
{
public:
  static constexpr const char* SCHEMA = "cafbcr-task-observation-v1";
  struct Summary {
    ComputeTaskSummary tasks;
    uint64_t rightCensored = 0, received = 0, started = 0, finished = 0, rejected = 0;
    uint64_t queueSamples = 0, processingSamples = 0;
    int64_t queueNs = 0, processingNs = 0;
  };

  TaskObservationWriter(const std::string& directory, Time cutoff, const std::string& metadata,
                        bool writeArtifacts = true)
    : m_directory(directory), m_cutoff(cutoff), m_writeArtifacts(writeArtifacts)
  {
    if (!m_writeArtifacts) return;
    NS_ABORT_MSG_IF(!std::filesystem::create_directories(directory),
                    "Task output directory already exists: " << directory);
    std::ofstream manifest(m_directory / "run.json");
    manifest << metadata << '\n';
    NS_ABORT_MSG_IF(!manifest, "Cannot write task run metadata");
    m_events.open(m_directory / "events.jsonl");
    NS_ABORT_MSG_IF(!m_events, "Cannot open task event output");
  }

  void Attach(Ptr<ComputeTaskClientApp> app)
  {
    NS_ABORT_MSG_IF(!app->TraceConnectWithoutContext("TaskEvent",
                    MakeCallback(&TaskObservationWriter::Observe, this)), "Missing client trace");
    m_clients.push_back(app);
  }

  void Attach(Ptr<ComputeTaskServerApp> app)
  {
    NS_ABORT_MSG_IF(!app->TraceConnectWithoutContext("TaskEvent",
                    MakeCallback(&TaskObservationWriter::Observe, this)), "Missing server trace");
    m_servers.push_back(app);
  }

  // Optional external executor. Uses the same lifecycle and cutoff checks as
  // native servers; counts remain separate until final reconciliation.
  void ObserveExternalExecution(const ComputeTaskEvent& event)
  {
    if (event.time > m_cutoff) return;
    ++m_externalCounts[event.event];
    Observe(event);
  }

  // Read the already-finalized application ledger; never feed observations back
  // to routing or create a second per-event ledger.
  template<class Visitor> void VisitFinalTasks(Visitor visitor) const
  {
    NS_ABORT_MSG_IF(!m_finalized, "Read task outcomes only after finalization");
    for (const auto& [key, row] : m_rows)
      visitor(key.first, key.second, bool(row.received), bool(row.finished), bool(row.ack), row.failure);
  }

  static std::string Quote(const std::string& value)
  {
    std::ostringstream out;
    out << '"';
    for (unsigned char c : value) {
      if (c == '"' || c == '\\') out << '\\' << c;
      else if (c < 0x20) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << unsigned(c);
      else out << c;
    }
    out << '"';
    return out.str();
  }

  void Observe(const ComputeTaskEvent& event)
  {
    if (event.time > m_cutoff) return;
    NS_ABORT_MSG_IF(m_finalized, "Task event arrived after finalization");
    ++m_eventCounts[event.event];
    if (m_writeArtifacts) {
    std::ostringstream ip;
    ip << event.selectedServer;
    m_events << std::setprecision(17)
      << "{\"schema\":" << Quote(SCHEMA) << ",\"event\":" << Quote(event.event)
      << ",\"source_node\":" << event.sourceNodeId << ",\"task_sequence\":" << event.taskId
      << ",\"observer_node\":" << event.observerNodeId << ",\"time_ns\":" << event.time.GetNanoSeconds()
      << ",\"workload_gflop\":" << event.workload << ",\"payload_bytes\":" << event.dataSize
      << ",\"relative_deadline_ns\":" << event.deadline.GetNanoSeconds()
      << ",\"selected_server\":" << (event.event == "generated" ? Quote(ip.str()) : "null")
      << ",\"selection_policy\":" << (event.event == "generated" ? Quote(event.selectionPolicy) : "null")
      << ",\"visible_state_age_ns\":" << (event.event == "generated" ? OptionalTime(event.visibleStateAge) : "null")
      << ",\"visible_state_status\":" << (event.event == "generated" ? Quote(event.visibleStateStatus) : "null")
      << "}\n";
    }
    auto key = std::make_pair(event.sourceNodeId, event.taskId);
    if (event.event == "generated") {
      NS_ABORT_MSG_IF(m_rows.count(key), "Duplicate generated task identity");
      Row row;
      row.demand = event;
      m_rows.emplace(key, row);
      return;
    }
    auto it = m_rows.find(key);
    NS_ABORT_MSG_IF(it == m_rows.end(), "Task event has no generated demand");
    auto& row = it->second;
    if (event.event == "server_received") { row.received = event.time; row.server = event.observerNodeId; }
    else if (event.event == "execution_started") row.started = event.time;
    else if (event.event == "execution_finished") row.finished = event.time;
    else if (event.event == "response_sent") row.responseSent = event.time;
    else if (event.event == "acknowledged") row.ack = event.time;
    else if (event.event == "no_server" || event.event == "send_failed" ||
             event.event == "queue_rejected" || event.event == "server_cancelled" ||
             event.event == "response_send_failed" || event.event.rfind("transport_failed_", 0) == 0) {
      if (!row.failureTime) {
        row.failure = event.event;
        row.failureTime = event.time;
        row.failureObserver = event.observerNodeId;
      }
    }
  }

  void Finalize()
  {
    NS_ABORT_MSG_IF(m_finalized || Simulator::Now() < m_cutoff,
                    "Task finalization must occur once, after the observation cutoff");
    m_finalized = true;
    std::ofstream tasks;
    if (m_writeArtifacts) tasks.open(m_directory / "tasks.jsonl");
    ComputeTaskSummary counts;
    uint64_t censored = 0;
    uint64_t completeTimeline = 0;
    double matchedResponse = 0, request = 0, queue = 0, execution = 0, returned = 0;
    for (const auto& [key, row] : m_rows) {
      ++counts.generated;
      const auto& d = row.demand;
      const Time due = d.time + d.deadline;
      std::string state;
      if (row.ack) {
        state = *row.ack <= due ? "S" : "L";
        if (state == "S") ++counts.onTime;
        else ++counts.late;
        counts.responseSeconds += (*row.ack - d.time).GetSeconds();
      } else if (due <= m_cutoff) { state = "U_due"; ++counts.unconfirmedDue; }
      else { state = "U_future"; ++counts.unconfirmedFuture; }
      bool rightCensored = state == "U_future" && !row.failureTime;
      censored += rightCensored;
      bool timeline = row.received && row.started && row.finished && row.ack;
      if (row.received && row.started) {
        NS_ABORT_MSG_IF(*row.started < *row.received, "Negative task queue interval");
        ++m_summary.queueSamples;
        m_summary.queueNs += (*row.started - *row.received).GetNanoSeconds();
      }
      if (row.started && row.finished) {
        NS_ABORT_MSG_IF(*row.finished < *row.started, "Negative task execution interval");
        ++m_summary.processingSamples;
        m_summary.processingNs += (*row.finished - *row.started).GetNanoSeconds();
      }
      if (timeline) {
        NS_ABORT_MSG_IF(*row.received < d.time || *row.started < *row.received ||
                        *row.finished < *row.started || *row.ack < *row.finished,
                        "Task lifecycle times are out of order");
        ++completeTimeline;
        request += (*row.received - d.time).GetSeconds();
        queue += (*row.started - *row.received).GetSeconds();
        execution += (*row.finished - *row.started).GetSeconds();
        returned += (*row.ack - *row.finished).GetSeconds();
        matchedResponse += (*row.ack - d.time).GetSeconds();
      }
      if (m_writeArtifacts) {
      std::ostringstream ip;
      ip << d.selectedServer;
      tasks << std::setprecision(17) << "{\"schema\":" << Quote(SCHEMA)
        << ",\"source_node\":" << key.first << ",\"task_sequence\":" << key.second
        << ",\"generated_ns\":" << d.time.GetNanoSeconds()
        << ",\"deadline_ns\":" << due.GetNanoSeconds()
        << ",\"relative_deadline_ns\":" << d.deadline.GetNanoSeconds()
        << ",\"workload_gflop\":" << d.workload << ",\"payload_bytes\":" << d.dataSize
        << ",\"selected_server\":" << Quote(ip.str())
        << ",\"selection_policy\":" << Quote(d.selectionPolicy)
        << ",\"visible_state_age_ns\":" << OptionalTime(d.visibleStateAge)
        << ",\"visible_state_status\":" << Quote(d.visibleStateStatus)
        << ",\"executing_server_node\":" << OptionalId(row.server)
        << ",\"server_received_ns\":" << OptionalTime(row.received)
        << ",\"execution_started_ns\":" << OptionalTime(row.started)
        << ",\"execution_finished_ns\":" << OptionalTime(row.finished)
        << ",\"response_sent_ns\":" << OptionalTime(row.responseSent)
        << ",\"acknowledged_ns\":" << OptionalTime(row.ack)
        << ",\"state\":" << Quote(state) << ",\"right_censored\":" << (rightCensored ? "true" : "false")
        << ",\"failure_reason\":" << Quote(row.failure)
        << ",\"failure_time_ns\":" << OptionalTime(row.failureTime)
        << ",\"failure_observer_node\":" << OptionalId(row.failureObserver)
        << ",\"failure_evidence\":" << (row.failureTime ? Quote("application_event") : "null")
        << ",\"request_network_ns\":" << Difference(row.received, d.time)
        << ",\"queue_wait_ns\":" << Difference(row.started, row.received)
        << ",\"execution_ns\":" << Difference(row.finished, row.started)
        << ",\"return_network_ns\":" << Difference(row.ack, row.finished)
        << ",\"response_ns\":" << Difference(row.ack, d.time)
        << ",\"reaction_ns\":" << Difference(row.started, d.time) << "}\n";
      }
    }
    ComputeTaskSummary clientCounts;
    for (auto client : m_clients) {
      auto s = client->GetTaskSummary();
      clientCounts.generated += s.generated; clientCounts.onTime += s.onTime;
      clientCounts.late += s.late; clientCounts.unconfirmedDue += s.unconfirmedDue;
      clientCounts.unconfirmedFuture += s.unconfirmedFuture;
    }
    NS_ABORT_MSG_IF(counts.generated != clientCounts.generated || counts.onTime != clientCounts.onTime ||
                    counts.late != clientCounts.late || counts.unconfirmedDue != clientCounts.unconfirmedDue ||
                    counts.unconfirmedFuture != clientCounts.unconfirmedFuture,
                    "Passive event ledger disagrees with client records");
    uint64_t received = m_externalCounts["server_received"],
             started = m_externalCounts["execution_started"],
             finished = m_externalCounts["execution_finished"],
             dropped = m_externalCounts["queue_rejected"];
    for (auto server : m_servers) {
      received += server->GetTasksReceived(); started += server->GetTasksStarted();
      finished += server->GetTasksFinished(); dropped += server->GetTasksDropped();
    }
    NS_ABORT_MSG_IF(received != m_eventCounts["server_received"] ||
                    started != m_eventCounts["execution_started"] ||
                    finished != m_eventCounts["execution_finished"] ||
                    dropped != m_eventCounts["queue_rejected"], "Server counts disagree with event ledger");
    m_summary.tasks = counts;
    m_summary.rightCensored = censored;
    m_summary.received = received;
    m_summary.started = started;
    m_summary.finished = finished;
    m_summary.rejected = dropped;
    if (!m_writeArtifacts) return;
    std::ofstream summary(m_directory / "summary.json");
    summary << std::setprecision(17) << "{\"schema\":" << Quote(SCHEMA)
      << ",\"observation_stop_ns\":" << m_cutoff.GetNanoSeconds()
      << ",\"G\":" << counts.generated << ",\"S\":" << counts.onTime << ",\"L\":" << counts.late
      << ",\"U_due\":" << counts.unconfirmedDue << ",\"U_future\":" << counts.unconfirmedFuture
      << ",\"right_censored\":" << censored
      << ",\"TCR\":" << Ratio(counts.onTime + counts.late, counts.generated)
      << ",\"DSR\":" << Ratio(counts.onTime, counts.generated)
      << ",\"DMR_observed\":" << Ratio(counts.late + counts.unconfirmedDue, counts.generated)
      << ",\"acknowledged_samples\":" << counts.onTime + counts.late
      << ",\"mean_response_s\":" << Ratio(counts.responseSeconds, counts.onTime + counts.late)
      << ",\"complete_timeline_samples\":" << completeTimeline
      << ",\"matched_mean_response_s\":" << Ratio(matchedResponse, completeTimeline)
      << ",\"matched_mean_request_s\":" << Ratio(request, completeTimeline)
      << ",\"matched_mean_queue_s\":" << Ratio(queue, completeTimeline)
      << ",\"matched_mean_execution_s\":" << Ratio(execution, completeTimeline)
      << ",\"matched_mean_return_s\":" << Ratio(returned, completeTimeline)
      << ",\"server_received\":" << received << ",\"server_started\":" << started
      << ",\"server_finished\":" << finished << ",\"server_rejected\":" << dropped
      << ",\"duplicate_completions\":" << m_eventCounts["duplicate_completion"]
      << ",\"invalid_completions\":" << m_eventCounts["invalid_completion"]
      << ",\"duplicate_requests\":" << m_eventCounts["duplicate_request"]
      << ",\"counts_conserved\":true,\"formal_eligible\":false"
      << ",\"remaining_validation\":\"common_demand_load_information_baselines_and_measurement\"}\n";
    tasks.flush(); summary.flush(); m_events.flush();
    NS_ABORT_MSG_IF(!tasks || !summary || !m_events, "Failed to write task observation artifacts");
    m_events.close();
  }

  const Summary& GetSummary() const
  {
    NS_ABORT_MSG_IF(!m_finalized, "Task summary requested before finalization");
    return m_summary;
  }

private:
  struct Row {
    ComputeTaskEvent demand;
    std::optional<Time> received, started, finished, responseSent, ack, failureTime;
    std::optional<uint32_t> server, failureObserver;
    std::string failure = "unknown";
  };
  static std::string OptionalTime(std::optional<Time> value)
  { return value ? std::to_string(value->GetNanoSeconds()) : "null"; }
  static std::string OptionalId(std::optional<uint32_t> value)
  { return value ? std::to_string(*value) : "null"; }
  static std::string Difference(std::optional<Time> end, std::optional<Time> begin)
  { return end && begin ? std::to_string((*end - *begin).GetNanoSeconds()) : "null"; }
  static std::string Ratio(double numerator, uint64_t denominator)
  {
    if (!denominator) return "null";
    std::ostringstream out;
    out << std::setprecision(17) << numerator / denominator;
    return out.str();
  }
  std::filesystem::path m_directory;
  Time m_cutoff;
  bool m_finalized = false;
  bool m_writeArtifacts;
  Summary m_summary;
  std::ofstream m_events;
  std::map<std::pair<uint32_t, uint32_t>, Row> m_rows;
  std::map<std::string, uint64_t> m_eventCounts;
  std::map<std::string, uint64_t> m_externalCounts;
  std::vector<Ptr<ComputeTaskClientApp>> m_clients;
  std::vector<Ptr<ComputeTaskServerApp>> m_servers;
};
} // namespace ns3::cafbcr
#endif
