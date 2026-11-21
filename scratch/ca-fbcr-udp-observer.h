/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef CA_FBCR_UDP_OBSERVER_H
#define CA_FBCR_UDP_OBSERVER_H

#include "ca-fbcr-udp-demand.h"
#include "ns3/abort.h"
#include "ns3/inet-socket-address.h"
#include "ns3/seq-ts-header.h"
#include "ns3/simulator.h"
#include "ns3/udp-client.h"
#include "ns3/udp-server.h"
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace ns3::cafbcr {

// Passive application-level accounting for the existing UdpClient/UdpServer.
// TxWithAddresses fires BEFORE SeqTsHeader and socket Send. It is an attempt,
// not a successful submission. No packet copies/tags, RNG or events are added.
class UdpObservationWriter
{
public:
  static constexpr const char* SCHEMA = "cafbcr-udp-observation-v2";
  struct Counts {
    uint64_t attempts = 0, windowAttempts = 0, receives = 0, unique = 0;
    uint64_t duplicates = 0, outsideReceives = 0, invalidReceives = 0;
    uint64_t datagramBytes = 0, payloadBytes = 0, delayNs = 0;
    uint64_t accepted = 0, failed = 0, errors = 0;
    uint64_t planned = 0, windowPlanned = 0;
  };

  UdpObservationWriter(const std::string& directory, Time begin, Time end, bool fixedDemand = false,
                       bool writeArtifacts = true)
    : m_directory(directory), m_begin(begin.GetNanoSeconds()), m_end(end.GetNanoSeconds()),
      m_fixed(fixedDemand), m_writeArtifacts(writeArtifacts)
  {
    if (!m_writeArtifacts) return;
    m_tx.open(m_directory / "udp-attempts.csv");
    m_rx.open(m_directory / "udp-receives.csv");
    NS_ABORT_MSG_IF(!m_tx || !m_rx, "Cannot open UDP observation output");
    m_tx << "flow_index,attempt_index,packet_uid,tx_ns,in_window,pre_header_bytes,"
            "datagram_bytes,source_port,socket_result\n";
    m_rx << "flow_index,receive_index,packet_uid,rx_ns,datagram_bytes,source_ip,source_port,"
            "local_port,sequence,header_tx_ns,attempt_index,valid,reason,in_window,"
            "first_observed,first_eligible,delay_ns\n";
  }

  void AddFlow(uint32_t index, uint32_t source, uint32_t destination,
               Ipv4Address sourceIp, Ipv4Address destinationIp, uint16_t port,
               Time start, Time stop, Time interval, uint32_t bytes, uint32_t cap,
               uint64_t plannedAttempts = 0)
  {
    NS_ABORT_MSG_IF(m_finalized || index != m_flows.size() || interval.IsNegative() ||
                    interval.IsZero() || bytes < 12, "Invalid UDP flow declaration");
    Flow flow;
    flow.source = source;
    flow.destination = destination;
    flow.sourceIp = sourceIp;
    flow.destinationIp = destinationIp;
    flow.port = port;
    flow.start = start.GetNanoSeconds();
    flow.stop = stop.GetNanoSeconds();
    flow.interval = interval.GetNanoSeconds();
    flow.bytes = bytes;
    flow.cap = cap;
    if (m_fixed) {
      flow.count.planned = plannedAttempts;
      flow.count.windowPlanned = udp_demand::InWindow(start, interval, plannedAttempts,
                                                     NanoSeconds(m_begin), NanoSeconds(m_end));
    }
    m_flows.push_back(flow);
  }

  void Attach(uint32_t index, Ptr<UdpClient> client, Ptr<UdpServer> server)
  {
    auto& flow = m_flows.at(index);
    NS_ABORT_MSG_IF(!client || !server || flow.client || flow.start >= flow.stop,
                    "Invalid UDP observation attachment");
    flow.client = client;
    flow.server = server;
    bool tx = client->TraceConnectWithoutContext("TxWithAddresses", MakeBoundCallback(&Tx, this, index));
    bool rx = server->TraceConnectWithoutContext("RxWithAddresses", MakeBoundCallback(&Rx, this, index));
    NS_ABORT_MSG_IF(!tx || !rx, "UDP application observation trace unavailable");
  }

  Counts GetCounts() const { return m_total; }

  using AttemptObserver = std::function<void(uint64_t, bool)>;
  void SetAttemptObserver(AttemptObserver observer) { m_attemptObserver = std::move(observer); }
  template<class Visitor> void VisitFinalAttempts(Visitor visitor) const
  {
    NS_ABORT_MSG_IF(!m_finalized, "Read UDP outcomes only after finalization");
    for (const auto& flow : m_flows)
      for (const auto& [uid, attempt] : flow.attempts)
        if (attempt.inWindow) visitor(uid, attempt.eligible);
  }

  void Finalize()
  {
    NS_ABORT_MSG_IF(m_finalized, "UDP observations already finalized");
    m_finalized = true;
    std::ofstream flows;
    if (m_writeArtifacts) flows.open(m_directory / "udp-flows-measured.csv");
    NS_ABORT_MSG_IF(m_writeArtifacts && !flows, "Cannot open UDP measured-flow output");
    flows << "flow_index,source_node,destination_node,source_ip,destination_ip,port,"
             "start_ns,stop_ns,interval_ns,datagram_bytes,max_successful_sends,"
             "time_opportunities,attempts,window_attempts,accepted_total,failed_total,"
             "server_received_total,raw_receives,unique_received,duplicates,"
             "outside_receives,invalid_receives,unique_datagram_bytes,unique_payload_bytes,"
             "delay_sum_ns,pdr_pct,mean_delay_ms,datagram_goodput_kbps,payload_goodput_kbps,"
             "stop_cause,errors,planned_attempts,window_planned_attempts\n";
    for (uint32_t i = 0; i < m_flows.size(); ++i) {
      auto& f = m_flows[i];
      const uint64_t opportunities = Opportunities(f);
      uint64_t serverReceived = 0;
      if (f.client) {
        uint64_t acceptedBytes = f.client->GetTotalTx(); // pure end-of-run getter
        f.count.accepted = acceptedBytes / f.bytes;
        if (acceptedBytes % f.bytes || f.count.accepted > f.count.attempts) ++f.count.errors;
        else f.count.failed = f.count.attempts - f.count.accepted;
        serverReceived = f.server->GetReceived(); // includes duplicates
        if (serverReceived != f.nonemptyReceives) ++f.count.errors;
        bool countError = m_fixed
          ? (f.cap != 0 || f.count.planned != opportunities || f.count.attempts != f.count.planned)
          : (f.count.attempts > opportunities || (f.cap && f.count.accepted > f.cap) ||
             ((!f.cap || f.count.accepted < f.cap) && f.count.attempts != opportunities));
        if (countError) ++f.count.errors;
        f.client->TraceDisconnectWithoutContext("TxWithAddresses", MakeBoundCallback(&Tx, this, i));
        f.server->TraceDisconnectWithoutContext("RxWithAddresses", MakeBoundCallback(&Rx, this, i));
      } else if (opportunities) {
        ++f.count.errors;
      }
      if (m_fixed && f.count.windowAttempts != f.count.windowPlanned) ++f.count.errors;
      if (Simulator::Now().GetNanoSeconds() < std::min(f.stop, m_end)) ++f.count.errors;
      std::string stop = opportunities == 0 ? "inactive" :
        (m_fixed ? "fixed_attempt_window" :
         (f.cap && f.count.accepted == f.cap ? "successful_submission_cap" : "application_stop"));
      if (m_writeArtifacts) flows << i << ',' << f.source << ',' << f.destination << ',' << f.sourceIp << ','
            << f.destinationIp << ',' << f.port << ',' << f.start << ',' << f.stop << ','
            << f.interval << ',' << f.bytes << ',' << f.cap << ',' << opportunities << ','
            << f.count.attempts << ',' << f.count.windowAttempts << ',' << f.count.accepted << ','
            << f.count.failed << ',' << serverReceived << ',' << f.count.receives << ','
            << f.count.unique << ',' << f.count.duplicates << ',' << f.count.outsideReceives << ','
            << f.count.invalidReceives << ',' << f.count.datagramBytes << ',' << f.count.payloadBytes
            << ',' << f.count.delayNs << ',' << Pdr(f.count) << ',' << Delay(f.count) << ','
            << Goodput(f.count.datagramBytes) << ',' << Goodput(f.count.payloadBytes) << ','
            << stop << ',' << f.count.errors << ','
            << (m_fixed ? std::to_string(f.count.planned) : "null") << ','
            << (m_fixed ? std::to_string(f.count.windowPlanned) : "null") << '\n';
      Add(m_total, f.count);
    }
    if (m_end > m_begin && Simulator::Now().GetNanoSeconds() < m_end) ++m_total.errors;
    if (!m_writeArtifacts) return;
    flows.close();
    m_tx.close();
    m_rx.close();
    NS_ABORT_MSG_IF(!flows || !m_tx || !m_rx, "Cannot write UDP observation records");
    WriteSummary();
  }

private:
  struct Attempt {
    uint64_t index, uid;
    int64_t time;
    uint16_t sourcePort;
    bool inWindow, observed = false, eligible = false;
  };
  struct Flow {
    uint32_t source = 0, destination = 0, bytes = 0, cap = 0;
    Ipv4Address sourceIp, destinationIp;
    uint16_t port = 0;
    int64_t start = 0, stop = 0, interval = 1;
    Ptr<UdpClient> client;
    Ptr<UdpServer> server;
    Counts count;
    uint64_t nonemptyReceives = 0;
    std::map<uint64_t, Attempt> attempts;
  };

  bool InWindow(int64_t t) const { return m_end > m_begin && t >= m_begin && t < m_end; }
  static uint64_t Opportunities(const Flow& f)
  {
    return f.stop <= f.start ? 0 : 1 + static_cast<uint64_t>((f.stop - f.start - 1) / f.interval);
  }
  static std::string Number(double value)
  {
    std::ostringstream out;
    out << std::setprecision(17) << value;
    return out.str();
  }
  std::string Pdr(const Counts& c) const
  {
    uint64_t denominator = m_fixed ? c.windowPlanned : c.windowAttempts;
    return denominator ? Number(100.0 * c.unique / denominator) : "null";
  }
  static std::string Delay(const Counts& c)
  {
    return c.unique ? Number(static_cast<double>(c.delayNs) / c.unique / 1e6) : "null";
  }
  std::string Goodput(uint64_t bytes) const
  {
    return m_end > m_begin ? Number(bytes * 8.0 * 1e6 / (m_end - m_begin)) : "null";
  }
  static void Add(Counts& a, const Counts& b)
  {
    a.attempts += b.attempts; a.windowAttempts += b.windowAttempts; a.receives += b.receives;
    a.unique += b.unique; a.duplicates += b.duplicates; a.outsideReceives += b.outsideReceives;
    a.invalidReceives += b.invalidReceives; a.datagramBytes += b.datagramBytes;
    a.payloadBytes += b.payloadBytes; a.delayNs += b.delayNs;
    a.accepted += b.accepted; a.failed += b.failed; a.errors += b.errors;
    a.planned += b.planned; a.windowPlanned += b.windowPlanned;
  }

  static void Tx(UdpObservationWriter* self, uint32_t index, Ptr<const Packet> packet,
                 const Address& from, const Address& to)
  {
    auto& f = self->m_flows.at(index);
    const int64_t now = Simulator::Now().GetNanoSeconds();
    const uint64_t attempt = f.count.attempts++;
    const bool window = self->InWindow(now);
    f.count.windowAttempts += window;
    uint16_t sourcePort = 0;
    // UdpClient normally binds ANY; its source IPv4 address is not known here.
    if (InetSocketAddress::IsMatchingType(from)) sourcePort = InetSocketAddress::ConvertFrom(from).GetPort();
    if (!InetSocketAddress::IsMatchingType(to) ||
        InetSocketAddress::ConvertFrom(to).GetIpv4() != f.destinationIp ||
        InetSocketAddress::ConvertFrom(to).GetPort() != f.port ||
        packet->GetSize() + 12 != f.bytes || now != f.start + static_cast<int64_t>(attempt) * f.interval ||
        now >= f.stop || !sourcePort) ++f.count.errors;
    auto inserted = f.attempts.emplace(packet->GetUid(), Attempt{attempt, packet->GetUid(), now, sourcePort, window});
    if (!inserted.second) ++f.count.errors;
    if (self->m_attemptObserver) self->m_attemptObserver(packet->GetUid(), window);
    if (self->m_writeArtifacts) self->m_tx << index << ',' << attempt << ',' << packet->GetUid() << ',' << now << ','
               << window << ',' << packet->GetSize() << ',' << f.bytes << ',' << sourcePort << ",unknown\n";
  }

  static void Rx(UdpObservationWriter* self, uint32_t index, Ptr<const Packet> packet,
                 const Address& from, const Address& local)
  {
    auto& f = self->m_flows.at(index);
    const int64_t now = Simulator::Now().GetNanoSeconds();
    const uint64_t receive = f.count.receives++;
    if (packet->GetSize()) ++f.nonemptyReceives;
    uint16_t sourcePort = 0, localPort = 0;
    Ipv4Address sourceIp;
    if (InetSocketAddress::IsMatchingType(from)) {
      auto address = InetSocketAddress::ConvertFrom(from);
      sourcePort = address.GetPort(); sourceIp = address.GetIpv4();
    }
    if (InetSocketAddress::IsMatchingType(local)) localPort = InetSocketAddress::ConvertFrom(local).GetPort();
    std::optional<uint32_t> sequence;
    std::optional<int64_t> headerTime;
    if (packet->GetSize() >= 12) {
      SeqTsHeader header;
      if (packet->PeekHeader(header) == 12) {
        sequence = header.GetSeq(); headerTime = header.GetTs().GetNanoSeconds();
      }
    }
    auto attempt = f.attempts.find(packet->GetUid());
    std::string reason = "ok";
    if (attempt == f.attempts.end()) reason = "unmatched_uid";
    else if (!headerTime || !sequence) reason = "missing_header";
    else if (sourceIp != f.sourceIp || localPort != f.port || sourcePort != attempt->second.sourcePort) reason = "endpoint_mismatch";
    else if (packet->GetSize() != f.bytes) reason = "size_mismatch";
    else if (*headerTime != attempt->second.time || now < *headerTime) reason = "timestamp_mismatch";
    const bool valid = reason == "ok";
    bool firstObserved = false, firstEligible = false;
    const bool window = self->InWindow(now);
    if (!valid) { ++f.count.invalidReceives; ++f.count.errors; }
    else {
      auto& a = attempt->second;
      firstObserved = !a.observed;
      if (a.observed) ++f.count.duplicates;
      a.observed = true;
      if (!a.inWindow || !window) ++f.count.outsideReceives;
      else if (!a.eligible) {
        a.eligible = true; firstEligible = true;
        ++f.count.unique;
        f.count.datagramBytes += packet->GetSize();
        f.count.payloadBytes += packet->GetSize() - 12;
        f.count.delayNs += now - a.time;
      }
    }
    if (self->m_writeArtifacts) self->m_rx << index << ',' << receive << ',' << packet->GetUid() << ',' << now << ','
               << packet->GetSize() << ',' << sourceIp << ',' << sourcePort << ',' << localPort << ','
               << (sequence ? std::to_string(*sequence) : "null") << ','
               << (headerTime ? std::to_string(*headerTime) : "null") << ','
               << (attempt != f.attempts.end() ? std::to_string(attempt->second.index) : "null") << ','
               << valid << ',' << reason << ',' << window << ',' << firstObserved << ',' << firstEligible << ','
               << (valid ? std::to_string(now - attempt->second.time) : "null") << '\n';
  }

  void WriteSummary()
  {
    std::ofstream json(m_directory / "udp-summary.json");
    std::ofstream csv(m_directory / "udp-summary.csv");
    NS_ABORT_MSG_IF(!json || !csv, "Cannot open UDP summary output");
    const bool valid = m_total.errors == 0;
    std::string status = !valid ? "invalid" : (m_end <= m_begin ? "empty_window" :
      ((m_fixed ? m_total.windowPlanned : m_total.windowAttempts) == 0 ? "no_traffic" : "observed"));
    json << "{\"schema\":\"" << SCHEMA << "\",\"formal_eligible\":false,"
         << "\"traffic_contract\":\"" << (m_fixed ? "fixed_attempts_v1" : "legacy_success_capped") << "\","
         << "\"fixed_attempt_demand\":" << (m_fixed ? "true" : "false") << ','
         << "\"planned_attempts\":" << (m_fixed ? std::to_string(m_total.planned) : "null") << ','
         << "\"window_planned_attempts\":" << (m_fixed ? std::to_string(m_total.windowPlanned) : "null") << ','
         << "\"pdr_denominator\":\"" << (m_fixed ? "planned_attempts_in_window" : "observed_attempts_in_window") << "\","
         << "\"packet_identity\":\"flow_index_and_local_attempt_index_bound_to_packet_uid\","
         << "\"window_rule\":\"tx_and_rx_in_half_open_window\","
         << "\"individual_socket_result\":\"unknown\",\"accepted_count_source\":\"UdpClient.GetTotalTx_divided_by_datagram_size\","
         << "\"hop_mean\":null,\"hop_status\":\"not_observed\","
         << "\"measurement_valid\":" << (valid ? "true" : "false") << ",\"status\":\"" << status << "\","
         << "\"window_complete\":" << (Simulator::Now().GetNanoSeconds() >= m_end ? "true" : "false") << ','
         << "\"window_begin_ns\":" << m_begin << ",\"window_end_ns\":" << m_end
         << ",\"capture_end_ns\":" << Simulator::Now().GetNanoSeconds()
         << ",\"flow_count\":" << m_flows.size() << ",\"attempts_total\":" << m_total.attempts
         << ",\"window_attempts\":" << m_total.windowAttempts << ",\"accepted_total\":" << m_total.accepted
         << ",\"failed_total\":" << m_total.failed << ",\"raw_receives\":" << m_total.receives
         << ",\"unique_received\":" << m_total.unique << ",\"duplicates\":" << m_total.duplicates
         << ",\"outside_receives\":" << m_total.outsideReceives << ",\"invalid_receives\":" << m_total.invalidReceives
         << ",\"unique_datagram_bytes\":" << m_total.datagramBytes << ",\"unique_payload_bytes\":" << m_total.payloadBytes
         << ",\"delay_sum_ns\":" << m_total.delayNs << ",\"errors\":" << m_total.errors
         << ",\"pdr_pct\":" << Pdr(m_total) << ",\"mean_delay_ms\":" << Delay(m_total)
         << ",\"datagram_goodput_kbps\":" << Goodput(m_total.datagramBytes)
         << ",\"payload_goodput_kbps\":" << Goodput(m_total.payloadBytes) << "}\n";
    csv << "schema,status,measurement_valid,window_begin_ns,window_end_ns,window_attempts,"
           "unique_received,pdr_pct,mean_delay_ms,datagram_goodput_kbps,payload_goodput_kbps,formal_eligible\n"
        << SCHEMA << ',' << status << ',' << valid << ',' << m_begin << ',' << m_end << ','
        << m_total.windowAttempts << ',' << m_total.unique << ',' << Pdr(m_total) << ',' << Delay(m_total)
        << ',' << Goodput(m_total.datagramBytes) << ',' << Goodput(m_total.payloadBytes) << ",0\n";
    json.close(); csv.close();
    NS_ABORT_MSG_IF(!json || !csv, "Cannot write UDP summary output");
  }

  std::filesystem::path m_directory;
  int64_t m_begin, m_end;
  std::ofstream m_tx, m_rx;
  std::vector<Flow> m_flows;
  Counts m_total;
  AttemptObserver m_attemptObserver;
  bool m_finalized = false;
  bool m_fixed;
  bool m_writeArtifacts;
};

} // namespace ns3::cafbcr
#endif
