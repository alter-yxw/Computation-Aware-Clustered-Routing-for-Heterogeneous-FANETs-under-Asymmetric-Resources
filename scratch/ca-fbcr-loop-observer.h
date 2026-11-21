/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef CA_FBCR_LOOP_OBSERVER_H
#define CA_FBCR_LOOP_OBSERVER_H

#include "ca-fbcr-task-observer.h"
#include "ns3/cafbcr-routing.h"
#include "../contrib/cafbcr/model/cafbcr-forwarding-history.h"

namespace ns3::cafbcr {

// Reads supplied events and counters only; never calls Inspect/Remember,
// purges a history, queries mobility, or constructs a Packet/RNG/event.
class LoopObservationWriter
{
public:
  LoopObservationWriter(const std::string& directory, Time cutoff, bool enabled)
    : m_directory(directory), m_cutoff(cutoff), m_enabled(enabled)
  {
    auto p = std::filesystem::path(directory) / "loop-events.jsonl";
    NS_ABORT_MSG_IF(std::filesystem::exists(p), "Loop observations already exist");
    m_output.open(p); NS_ABORT_MSG_IF(!m_output, "Cannot open loop observations");
  }
  ~LoopObservationWriter()
  {
    for (size_t i = 0; i < m_histories.size(); ++i)
      m_histories[i]->TraceDisconnect("Decision", std::to_string(m_nodes[i]), m_callback);
  }
  void Attach(uint32_t node, Ptr<RoutingProtocol> route)
  {
    auto history = route->GetForwardingHistory();
    NS_ABORT_MSG_IF(bool(history) != m_enabled, "Requested and installed loop policies differ");
    if (!history) return;
    NS_ABORT_MSG_IF(!history->TraceConnect("Decision", std::to_string(node), m_callback), "Missing history trace");
    m_nodes.push_back(node); m_histories.push_back(history);
  }
  void Finalize()
  {
    m_output.flush(); NS_ABORT_MSG_IF(!m_output, "Cannot flush loop observations");
    m_finalized = true;
    std::ofstream summary(std::filesystem::path(m_directory) / "loop-summary.json");
    summary << "{\"schema\":\"" << ForwardingHistory::SCHEMA << "\",\"formal_eligible\":false"
      << ",\"enabled\":" << (m_enabled ? "true" : "false")
      << ",\"status\":\"" << (m_enabled ? "observed" : "disabled") << "\""
      << ",\"identity\":\"source_destination_protocol_ipid_exact_payload_no_uid_no_ttl\""
      << ",\"wire_bytes_added\":0,\"scope\":\"transit_unfragmented_early_termination_not_geographic_recovery\""
      << ",\"window_end_ns\":" << m_cutoff.GetNanoSeconds()
      << ",\"capture_end_ns\":" << Simulator::Now().GetNanoSeconds()
      << ",\"events\":" << m_events << ",\"after_cutoff_events\":" << m_afterCutoff
      << ",\"revisit_drops_in_window\":" << (m_enabled ? std::to_string(m_drops) : "null")
      << ",\"memory_scope\":\"exact_stored_payload_bytes_and_entry_counts_exclude_allocator_overhead\""
      << ",\"processing_scope\":\"payload_bytes_copied_and_entry_comparisons_not_CPU_time_or_energy\""
      << ",\"nodes\":[";
    for (size_t i = 0; i < m_histories.size(); ++i) {
      if (i) summary << ',';
      auto h = m_histories[i]; auto s = h->GetStats();
      NS_ABORT_MSG_IF(s.entries > h->GetEntryLimit() || s.payloadBytes > h->GetPayloadLimit(), "History exceeded resource limits");
      summary << "{\"node\":" << m_nodes[i] << ",\"entry_limit\":" << h->GetEntryLimit()
        << ",\"payload_limit\":" << h->GetPayloadLimit() << ",\"lifetime_ns\":" << h->GetLifetime().GetNanoSeconds()
        << ",\"checks\":" << s.checks << ",\"remembered\":" << s.remembered
        << ",\"revisit_drops\":" << s.revisitDrops << ",\"same_ttl_forwards\":" << s.sameTtlForwards
        << ",\"higher_ttl_forwards\":" << s.higherTtlForwards
        << ",\"expired\":" << s.expired << ",\"evicted\":" << s.evicted
        << ",\"copied_payload_bytes\":" << s.copiedPayloadBytes
        << ",\"identity_comparisons\":" << s.identityComparisons
        << ",\"fragments_untracked\":" << s.fragmentsUntracked << ",\"oversized_untracked\":" << s.oversizedUntracked
        << ",\"ttl_delegated\":" << s.ttlDelegated << ",\"entries\":" << s.entries
        << ",\"payload_bytes\":" << s.payloadBytes << ",\"peak_entries\":" << s.peakEntries
        << ",\"peak_payload_bytes\":" << s.peakPayloadBytes << '}';
    }
    summary << "]}\n"; summary.close(); NS_ABORT_MSG_IF(!summary, "Cannot save loop summary");
  }
private:
  void Observe(std::string context, const ForwardingHistoryEvent& e)
  {
    NS_ABORT_MSG_IF(m_finalized, "Loop event after finalization");
    ++m_events;
    if (e.time >= m_cutoff) ++m_afterCutoff;
    else if (std::string(e.action) == "revisit_drop") ++m_drops;
    std::ostringstream source, destination; source << e.source; destination << e.destination;
    m_output << "{\"schema\":\"" << ForwardingHistory::SCHEMA << "\",\"node\":" << context
      << ",\"time_ns\":" << e.time.GetNanoSeconds() << ",\"action\":" << TaskObservationWriter::Quote(e.action)
      << ",\"packet_uid\":" << e.observationUid << ",\"source_ip\":" << TaskObservationWriter::Quote(source.str())
      << ",\"destination_ip\":" << TaskObservationWriter::Quote(destination.str())
      << ",\"protocol\":" << unsigned(e.protocol) << ",\"ip_id\":" << e.identification
      << ",\"ttl\":" << unsigned(e.ttl) << ",\"fragment_offset\":" << e.fragmentOffset
      << ",\"more_fragments\":" << (e.moreFragments ? "true" : "false") << ",\"packet_bytes\":" << e.packetBytes
      << ",\"first_forwarded_ns\":" << (e.firstForwarded ? std::to_string(e.firstForwarded->GetNanoSeconds()) : "null")
      << ",\"first_ttl\":" << (e.firstTtl ? std::to_string(*e.firstTtl) : "null")
      << ",\"entries\":" << e.entries << ",\"tracked_payload_bytes\":" << e.trackedPayloadBytes
      << ",\"expired_total\":" << e.expired << ",\"evicted_total\":" << e.evicted << "}\n";
  }
  std::string m_directory;
  Time m_cutoff;
  bool m_enabled, m_finalized = false;
  uint64_t m_events = 0, m_afterCutoff = 0, m_drops = 0;
  std::ofstream m_output;
  std::vector<uint32_t> m_nodes;
  std::vector<Ptr<ForwardingHistory>> m_histories;
  Callback<void, std::string, const ForwardingHistoryEvent&> m_callback = MakeCallback(&LoopObservationWriter::Observe, this);
};
} // namespace ns3::cafbcr
#endif
