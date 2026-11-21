/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef CA_FBCR_RECOVERY_OBSERVER_H
#define CA_FBCR_RECOVERY_OBSERVER_H
#include "ca-fbcr-task-observer.h"
#include "ns3/cafbcr-routing.h"
#include "../contrib/cafbcr/model/cafbcr-revisit-recovery.h"

namespace ns3::cafbcr {
// Passive event/counter sink. Never inspects packets or changes history.
class RecoveryObservationWriter
{
public:
  RecoveryObservationWriter(const std::string& directory, Time cutoff, bool enabled)
    : m_directory(directory), m_cutoff(cutoff), m_enabled(enabled)
  {
    auto path = std::filesystem::path(directory) / "recovery-events.jsonl";
    NS_ABORT_MSG_IF(std::filesystem::exists(path), "Recovery observations already exist");
    m_output.open(path); NS_ABORT_MSG_IF(!m_output, "Cannot open recovery observations");
  }
  ~RecoveryObservationWriter()
  {
    for (size_t i = 0; i < m_histories.size(); ++i)
      m_histories[i]->TraceDisconnect("Decision", std::to_string(m_nodes[i]), m_callback);
  }
  void Attach(uint32_t node, Ptr<RoutingProtocol> route)
  {
    auto h = route->GetRevisitRecovery();
    NS_ABORT_MSG_IF(bool(h) != m_enabled, "Requested and installed recovery policies differ");
    if (!h) return;
    NS_ABORT_MSG_IF(!h->TraceConnect("Decision", std::to_string(node), m_callback), "Missing recovery trace");
    m_nodes.push_back(node); m_histories.push_back(h);
  }
  void Finalize()
  {
    m_output.flush(); NS_ABORT_MSG_IF(!m_output, "Cannot flush recovery observations"); m_finalized = true;
    std::ofstream s(std::filesystem::path(m_directory) / "recovery-summary.json");
    s << "{\"schema\":\"" << RevisitRecovery::SCHEMA << "\",\"formal_eligible\":false"
      << ",\"enabled\":" << (m_enabled ? "true" : "false")
      << ",\"status\":\"" << (m_enabled ? "observed" : "disabled") << "\""
      << ",\"scope\":\"local_untried_next_hops_on_transit_revisit_not_face_routing\""
      << ",\"wire_bytes_added\":0,\"source_failure_feedback_added\":false"
      << ",\"window_end_ns\":" << m_cutoff.GetNanoSeconds()
      << ",\"capture_end_ns\":" << Simulator::Now().GetNanoSeconds()
      << ",\"events\":" << m_events << ",\"after_cutoff_events\":" << m_afterCutoff
      << ",\"alternate_forwards_in_window\":" << (m_enabled ? std::to_string(m_alternates) : "null")
      << ",\"terminal_decisions_in_window\":" << (m_enabled ? std::to_string(m_stops) : "null")
      << ",\"memory_scope\":\"stored_payload_and_four_bytes_per_stored_next_hop_excludes_allocator_overhead\""
      << ",\"processing_scope\":\"copy_and_comparison_counts_not_CPU_time_or_energy\",\"nodes\":[";
    for (size_t i = 0; i < m_histories.size(); ++i) {
      if (i) s << ',';
      auto h = m_histories[i]; auto t = h->GetStats();
      NS_ABORT_MSG_IF(t.entries > h->GetEntryLimit() || t.payloadBytes > h->GetPayloadLimit() ||
        t.hopBytes > 4 * h->GetEntryLimit() * h->GetNextHopLimit(), "Recovery resource limits exceeded");
      s << "{\"node\":" << m_nodes[i] << ",\"entry_limit\":" << h->GetEntryLimit()
        << ",\"payload_limit\":" << h->GetPayloadLimit() << ",\"next_hop_limit\":" << h->GetNextHopLimit()
        << ",\"lifetime_ns\":" << h->GetLifetime().GetNanoSeconds()
        << ",\"checks\":" << t.checks << ",\"native_forwards\":" << t.nativeForwards
        << ",\"alternate_forwards\":" << t.alternateForwards << ",\"revisits\":" << t.revisits
        << ",\"exhausted\":" << t.exhausted << ",\"budget_stops\":" << t.budgetStops
        << ",\"invalid_position\":" << t.invalidPosition << ",\"expired\":" << t.expired
        << ",\"evicted\":" << t.evicted << ",\"copied_payload_bytes\":" << t.copiedPayloadBytes
        << ",\"identity_comparisons\":" << t.identityComparisons
        << ",\"fragments_untracked\":" << t.fragmentsUntracked << ",\"oversized_untracked\":" << t.oversizedUntracked
        << ",\"ttl_delegated\":" << t.ttlDelegated << ",\"duplicate_hop_overflow\":" << t.duplicateHopOverflow
        << ",\"entries\":" << t.entries << ",\"payload_bytes\":" << t.payloadBytes << ",\"hop_bytes\":" << t.hopBytes
        << ",\"peak_entries\":" << t.peakEntries << ",\"peak_payload_bytes\":" << t.peakPayloadBytes
        << ",\"peak_hop_bytes\":" << t.peakHopBytes << '}';
    }
    s << "]}\n"; s.close(); NS_ABORT_MSG_IF(!s, "Cannot save recovery summary");
  }
private:
  static std::string Ip(Ipv4Address ip) { std::ostringstream s; s << ip; return TaskObservationWriter::Quote(s.str()); }
  void Observe(std::string context, const RevisitRecoveryEvent& e)
  {
    NS_ABORT_MSG_IF(m_finalized, "Recovery event after finalization");
    ++m_events; const std::string action(e.action);
    if (e.time >= m_cutoff) ++m_afterCutoff;
    else if (action == "alternate_forward") ++m_alternates;
    else if (action == "budget_exhausted" || action == "candidates_exhausted" || action == "invalid_position") ++m_stops;
    m_output << "{\"schema\":\"" << RevisitRecovery::SCHEMA << "\",\"node\":" << context
      << ",\"time_ns\":" << e.time.GetNanoSeconds() << ",\"action\":" << TaskObservationWriter::Quote(action)
      << ",\"packet_uid\":" << e.observationUid << ",\"source_ip\":" << Ip(e.source)
      << ",\"destination_ip\":" << Ip(e.destination) << ",\"protocol\":" << unsigned(e.protocol)
      << ",\"ip_id\":" << e.identification << ",\"ttl\":" << unsigned(e.ttl) << ",\"packet_bytes\":" << e.packetBytes
      << ",\"first_forwarded_ns\":" << (e.firstForwarded ? std::to_string(e.firstForwarded->GetNanoSeconds()) : "null")
      << ",\"first_ttl\":" << (e.firstTtl ? std::to_string(*e.firstTtl) : "null")
      << ",\"selected_ip\":" << (e.selected ? Ip(*e.selected) : "null") << ",\"tried_before\":[";
    for (size_t i = 0; i < e.tried.size(); ++i) { if (i) m_output << ','; m_output << Ip(e.tried[i]); }
    m_output << "],\"entries\":" << e.entries << ",\"tracked_payload_bytes\":" << e.payloadBytes
      << ",\"tracked_hop_bytes\":" << e.hopBytes << "}\n";
  }
  std::string m_directory;
  Time m_cutoff;
  bool m_enabled, m_finalized = false;
  uint64_t m_events = 0, m_afterCutoff = 0, m_alternates = 0, m_stops = 0;
  std::ofstream m_output;
  std::vector<uint32_t> m_nodes;
  std::vector<Ptr<RevisitRecovery>> m_histories;
  Callback<void, std::string, const RevisitRecoveryEvent&> m_callback = MakeCallback(&RecoveryObservationWriter::Observe, this);
};
} // namespace ns3::cafbcr
#endif
