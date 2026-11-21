/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef CA_FBCR_TASK_WIRE_OBSERVER_H
#define CA_FBCR_TASK_WIRE_OBSERVER_H
#include "ca-fbcr-task-observer.h"
#include "ns3/inet-socket-address.h"
#include <algorithm>
#include <array>

namespace ns3::cafbcr {
class TaskWireWriter
{
public:
  static constexpr const char* SCHEMA = "cafbcr-task-wire-observation-v1";
  TaskWireWriter(const std::string& directory, Time cutoff, bool header)
    : m_directory(directory), m_cutoff(cutoff), m_header(header),
      m_events(std::filesystem::path(directory) / "task-wire.jsonl")
  { NS_ABORT_MSG_IF(!m_events, "Cannot open task wire observation output"); }
  void Attach(Ptr<ComputeTaskClientApp> app) { Connect(app); }
  void Attach(Ptr<ComputeTaskServerApp> app) { Connect(app); }
  void Observe(const ComputeTaskWireEvent& e)
  {
    if (e.time > m_cutoff) return;
    NS_ABORT_MSG_IF(m_finalized || !e.packet, "Invalid task wire observation lifetime");
    auto& total = m_totals[e.event];
    ++total.count; total.bytes += e.packet->GetSize();
    bool valid = e.valid && e.format == (m_header ? "header-v1" : "legacy-tag");
    if (valid) valid = e.payloadBytes && e.metadataBytes &&
      uint64_t(*e.payloadBytes) + *e.metadataBytes == e.packet->GetSize() &&
      *e.metadataBytes == (m_header ? ComputeTaskHeader::SIZE : 0);
    if (!valid) { ++total.invalid; ++m_errors; }
    if (e.payloadBytes) total.payload += *e.payloadBytes;
    if (e.metadataBytes) total.metadata += *e.metadataBytes;
    if (e.socketResult) {
      if (*e.socketResult >= 0) {
        ++total.accepted; total.acceptedBytes += e.packet->GetSize();
        if (e.metadataBytes) total.acceptedMetadata += *e.metadataBytes;
        if (static_cast<uint32_t>(*e.socketResult) != e.packet->GetSize()) ++m_errors;
      } else ++total.failed;
    }
    std::array<uint8_t, ComputeTaskHeader::SIZE> prefix{};
    uint32_t n = std::min<uint32_t>(prefix.size(), e.packet->GetSize());
    e.packet->CopyData(prefix.data(), n); // copies bytes, not a Packet or UID
    std::ostringstream hex;
    for (uint32_t i = 0; i < n; ++i) hex << std::hex << std::setw(2) << std::setfill('0') << unsigned(prefix[i]);
    std::string ip = "null", port = "null";
    if (InetSocketAddress::IsMatchingType(e.peer)) {
      auto peer = InetSocketAddress::ConvertFrom(e.peer);
      std::ostringstream text; text << peer.GetIpv4();
      ip = TaskObservationWriter::Quote(text.str()); port = std::to_string(peer.GetPort());
    }
    CafbcrComputeTaskTag cache;
    bool cachedTag = e.packet->PeekPacketTag(cache);
    auto quote = &TaskObservationWriter::Quote;
    m_events << "{\"schema\":" << quote(SCHEMA) << ",\"event\":" << quote(e.event)
      << ",\"format\":" << quote(e.format) << ",\"time_ns\":" << e.time.GetNanoSeconds()
      << ",\"observer_node\":" << e.observerNodeId << ",\"packet_uid\":" << e.packet->GetUid()
      << ",\"source_node\":" << Optional(e.sourceNodeId) << ",\"task_sequence\":" << Optional(e.taskId)
      << ",\"packet_bytes\":" << e.packet->GetSize() << ",\"payload_bytes\":" << Optional(e.payloadBytes)
      << ",\"metadata_bytes\":" << Optional(e.metadataBytes) << ",\"peer_ip\":" << ip << ",\"peer_port\":" << port
      << ",\"format_valid\":" << (e.valid ? "true" : "false") << ",\"record_valid\":" << (valid ? "true" : "false")
      << ",\"reason\":" << quote(e.reason) << ",\"socket_result\":" << Optional(e.socketResult)
      << ",\"cached_tag_present\":" << (cachedTag ? "true" : "false")
      << ",\"wire_prefix_hex\":" << quote(hex.str()) << "}\n";
  }
  void Finalize()
  {
    NS_ABORT_MSG_IF(m_finalized || Simulator::Now() < m_cutoff, "Premature task wire finalization");
    m_finalized = true;
    for (auto app : m_apps) app->TraceDisconnectWithoutContext("TaskWire", MakeCallback(&TaskWireWriter::Observe, this));
    m_events.close();
    std::ofstream output(std::filesystem::path(m_directory) / "task-wire-summary.json");
    NS_ABORT_MSG_IF(!output || !m_events, "Cannot write task wire artifacts");
    output << "{\"schema\":" << TaskObservationWriter::Quote(SCHEMA)
      << ",\"format\":\"" << (m_header ? "header-v1" : "legacy-tag")
      << "\",\"header_bytes\":" << (m_header ? ComputeTaskHeader::SIZE : 0)
      << ",\"response_payload_bytes\":8,\"formal_eligible\":false,\"errors\":" << m_errors
      << ",\"measurement_valid\":" << (m_errors == 0 ? "true" : "false")
      << ",\"byte_scope\":\"application_datagram_not_PHY_airtime_or_hop_sum\",\"events\":{";
    bool comma = false;
    for (const auto& [stage, t] : m_totals) {
      if (comma) output << ',';
      comma = true;
      output << TaskObservationWriter::Quote(stage) << ":{\"count\":" << t.count
        << ",\"packet_bytes\":" << t.bytes << ",\"payload_bytes\":" << t.payload
        << ",\"metadata_bytes\":" << t.metadata << ",\"accepted\":" << t.accepted
        << ",\"failed\":" << t.failed << ",\"accepted_packet_bytes\":" << t.acceptedBytes
        << ",\"accepted_metadata_bytes\":" << t.acceptedMetadata << ",\"invalid\":" << t.invalid << '}';
    }
    output << "}}\n"; output.close();
    NS_ABORT_MSG_IF(!output, "Cannot finish task wire summary");
  }
private:
  struct Total {
    uint64_t count = 0, bytes = 0, payload = 0, metadata = 0, accepted = 0, failed = 0;
    uint64_t acceptedBytes = 0, acceptedMetadata = 0, invalid = 0;
  };
  void Connect(Ptr<Object> app)
  {
    NS_ABORT_MSG_IF(!app->TraceConnectWithoutContext("TaskWire", MakeCallback(&TaskWireWriter::Observe, this)),
                    "Missing task wire trace");
    m_apps.push_back(app);
  }
  template <typename T> static std::string Optional(std::optional<T> value)
  { return value ? std::to_string(*value) : "null"; }
  std::string m_directory;
  Time m_cutoff;
  bool m_header, m_finalized = false;
  std::ofstream m_events;
  std::map<std::string, Total> m_totals;
  std::vector<Ptr<Object>> m_apps;
  uint64_t m_errors = 0;
};
} // namespace ns3::cafbcr
#endif
