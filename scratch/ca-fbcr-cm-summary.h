/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef CA_FBCR_CM_SUMMARY_H
#define CA_FBCR_CM_SUMMARY_H

#include "ca-fbcr-task-observer.h"
#include "ns3/cafbcr-routing.h"
#include <array>
#include <fstream>
#include <filesystem>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <map>
#include <string>
#include <tuple>
#include <vector>

namespace ns3::cafbcr {

// Entry-only aggregation. Existing local trace facts are read synchronously:
// no packets, tags, RNG draws, simulation events or feedback to the protocol.
class CmSummary
{
public:
  static constexpr const char* SCHEMA = "cafbcr-cm-trigger-summary-v1";
  CmSummary(Time cutoff, Time generationStart, Time generationStop, bool reportSuppressed = false)
    : m_cutoff(cutoff), m_generationStart(generationStart), m_generationStop(generationStop),
      m_reportSuppressed(reportSuppressed) {}
  ~CmSummary()
  {
    for (auto route : m_routes) {
      route->TraceDisconnectWithoutContext("JoinMechanism", MakeCallback(&CmSummary::Join, this));
      route->TraceDisconnectWithoutContext("ProxyRenewal", MakeCallback(&CmSummary::Renewal, this));
    }
  }
  void Attach(Ptr<Node> node)
  {
    auto route = node->GetObject<RoutingProtocol>();
    NS_ABORT_MSG_IF(!route, "CM summary requires CA routing");
    NS_ABORT_MSG_IF(!route->TraceConnectWithoutContext("JoinMechanism", MakeCallback(&CmSummary::Join, this)) ||
                    !route->TraceConnectWithoutContext("ProxyRenewal", MakeCallback(&CmSummary::Renewal, this)),
                    "Cannot attach CM summary");
    m_routes.push_back(route);
  }
  void Save(const std::string& path, const std::string& build, uint32_t seed, uint64_t run,
            const std::string& scenario, uint64_t cmIpTx)
  {
    NS_ABORT_MSG_IF(m_saved || Simulator::Now() < m_cutoff, "Finalize CM summary after capture");
    m_saved = true;
    uint64_t successes = 0;
    for (const auto& [cause, count] : m_totals) successes += count.success;
    if (successes != cmIpTx) ++m_errors;
    std::ostringstream output;
    output << "{\"schema\":" << TaskObservationWriter::Quote(SCHEMA)
      << ",\"complete\":true,\"errors\":" << m_errors
      << ",\"build_label\":" << TaskObservationWriter::Quote(build)
      << ",\"seed\":" << seed << ",\"rng_run\":" << run
      << ",\"capture_end_ns\":" << m_cutoff.GetNanoSeconds()
      << ",\"task_generation_start_ns\":" << m_generationStart.GetNanoSeconds()
      << ",\"task_generation_stop_ns\":" << m_generationStop.GetNanoSeconds()
      << ",\"scenario\":" << scenario
      << ",\"cm_origin_ip_tx\":" << cmIpTx
      << ",\"socket_success_sum\":" << successes
      << ",\"counts_reconciled\":" << (successes == cmIpTx ? "true" : "false")
      << ",\"totals\":";
    WriteCounts(output, m_totals);
    output << ",\"windows\":{";
    const char* names[] = {"before_task_generation", "task_generation", "after_task_generation"};
    for (size_t i = 0; i < m_windows.size(); ++i) {
      output << (i ? "," : "") << TaskObservationWriter::Quote(names[i]) << ':';
      WriteCounts(output, m_windows[i]);
    }
    output << "},\"short_followups_le_50ms\":{";
    bool first = true;
    for (const auto& [causes, count] : m_pairs) {
      output << (first ? "" : ",") << TaskObservationWriter::Quote(causes)
             << ":{\"after_socket_success\":" << count.success
             << ",\"after_socket_failure\":" << count.failure << '}';
      first = false;
    }
    output << '}';
    if (m_reportSuppressed) {
      output << ",\"feedback_covered_responses_suppressed\":" << m_suppressed
        << ",\"suppressed_windows\":{";
      for (size_t i = 0; i < m_suppressedWindows.size(); ++i)
        output << (i ? "," : "") << TaskObservationWriter::Quote(names[i]) << ':' << m_suppressedWindows[i];
      output << '}';
    }
    output << ",\"scope\":\"Actual CM attempts only; skipped calls have no send event. "
              "Pair counts are observed near-time sends, not safely removable packets. "
              "Socket/IP counts do not imply CH delivery; no per-event output.\"}\n";
    std::filesystem::create_directories(std::filesystem::path(path).parent_path());
    const int file = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0644);
    NS_ABORT_MSG_IF(file < 0, "Cannot exclusively create CM summary: " << path);
    const std::string data = output.str();
    size_t written = 0;
    while (written < data.size()) {
      const auto count = ::write(file, data.data() + written, data.size() - written);
      if (count < 0 && errno == EINTR) continue;
      NS_ABORT_MSG_IF(count <= 0, "Cannot write CM summary");
      written += count;
    }
    NS_ABORT_MSG_IF(::close(file) != 0, "Cannot close CM summary");
  }

private:
  struct Counts { uint64_t attempts = 0, success = 0, failure = 0; };
  struct Previous { Time time; std::string cause; bool success; };
  using Table = std::map<std::string, Counts>;
  using Key = std::tuple<uint32_t, uint16_t, uint16_t, uint64_t>;
  static void WriteCounts(std::ostream& output, const Table& counts)
  {
    output << '{'; bool first = true;
    for (const auto& [cause, count] : counts) {
      output << (first ? "" : ",") << TaskObservationWriter::Quote(cause)
        << ":{\"attempts\":" << count.attempts << ",\"socket_success\":" << count.success
        << ",\"socket_failure\":" << count.failure << '}';
      first = false;
    }
    output << '}';
  }
  void Record(Time now, const std::string& cause, std::optional<int> result)
  {
    if (!result) { ++m_errors; return; }
    if (*result >= 0 && *result != 73) ++m_errors;
    const size_t window = now < m_generationStart ? 0 : now < m_generationStop ? 1 : 2;
    for (Counts* count : {&m_totals[cause], &m_windows[window][cause]}) {
      ++count->attempts;
      if (*result >= 0) ++count->success; else ++count->failure;
    }
  }
  void Join(const JoinMechanismEvent& event)
  {
    if (event.time > m_cutoff) return;
    if (event.event == "proxy_confirmed_at_source" || event.event == "ordinary_confirmed_at_source") {
      ++m_generation[event.observerNodeId]; return;
    }
    if (event.event == "cm_live_feedback_suppressed") {
      if (!m_reportSuppressed || !event.cmTrigger || std::string(event.cmTrigger) != "ch-live-response" ||
          event.socketResult) ++m_errors;
      ++m_suppressed;
      ++m_suppressedWindows[event.time < m_generationStart ? 0 : event.time < m_generationStop ? 1 : 2];
      return;
    }
    if (event.event != "cm_live_tx_attempt") return;
    const std::string cause = event.cmTrigger ? event.cmTrigger : "unclassified";
    if (cause != "periodic" && cause != "ch-live-response" && cause != "proxy-confirmation" &&
        cause != "ordinary-confirmation" && cause != "unspecified") ++m_errors;
    Record(event.time, cause, event.socketResult);
    const Key key{event.observerNodeId, event.headNodeId, event.clusterId,
                  m_generation[event.observerNodeId]};
    auto prior = m_previous.find(key);
    if (prior != m_previous.end() && event.time >= prior->second.time &&
        event.time - prior->second.time <= MilliSeconds(50)) {
      auto& pair = m_pairs[prior->second.cause + " -> " + cause];
      if (prior->second.success) ++pair.success; else ++pair.failure;
    }
    m_previous[key] = {event.time, cause, event.socketResult && *event.socketResult >= 0};
  }
  void Renewal(const ProxyRenewalEvent& event)
  {
    if (event.time <= m_cutoff && event.event == "relay_cm_tx_attempt")
      Record(event.time, "relay", event.socketResult);
  }
  Time m_cutoff;
  Time m_generationStart, m_generationStop;
  bool m_saved = false;
  bool m_reportSuppressed = false;
  uint64_t m_suppressed = 0;
  std::array<uint64_t, 3> m_suppressedWindows{};
  uint64_t m_errors = 0;
  Table m_totals;
  std::array<Table, 3> m_windows;
  std::map<uint32_t, uint64_t> m_generation;
  std::map<Key, Previous> m_previous;
  std::map<std::string, Counts> m_pairs;
  std::vector<Ptr<RoutingProtocol>> m_routes;
};
} // namespace ns3::cafbcr
#endif
