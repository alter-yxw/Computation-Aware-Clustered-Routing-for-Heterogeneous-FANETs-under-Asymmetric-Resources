/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef CA_FBCR_JOIN_OBSERVER_H
#define CA_FBCR_JOIN_OBSERVER_H

#include "ca-fbcr-task-observer.h"
#include "ns3/cafbcr-routing.h"
#include <cmath>
#include <iomanip>

namespace ns3::cafbcr {

// Serialize only facts supplied by the protocol trace. No neighbor/routing
// query, shared-manager lookup, RNG draw, packet injection or feedback.
class JoinObservationWriter
{
public:
  static constexpr const char* SCHEMA = "cafbcr-join-events-v1";
  static constexpr const char* RENEWAL_SCHEMA = "cafbcr-proxy-renewal-events-v1";
  JoinObservationWriter(const std::string& directory, Time cutoff) : m_cutoff(cutoff)
  {
    auto path = std::filesystem::path(directory) / "join-events.jsonl";
    NS_ABORT_MSG_IF(std::filesystem::exists(path), "Join observations already exist");
    m_output.open(path);
    NS_ABORT_MSG_IF(!m_output, "Cannot open join observations");
    auto renewalPath = std::filesystem::path(directory) / "proxy-renewal.jsonl";
    NS_ABORT_MSG_IF(std::filesystem::exists(renewalPath), "Proxy renewal observations already exist");
    m_renewal.open(renewalPath);
    NS_ABORT_MSG_IF(!m_renewal, "Cannot open proxy renewal observations");
  }
  ~JoinObservationWriter()
  {
    for (auto route : m_routes) {
      route->TraceDisconnectWithoutContext("JoinMechanism",
        MakeCallback(&JoinObservationWriter::Observe, this));
      route->TraceDisconnectWithoutContext("ProxyRenewal",
        MakeCallback(&JoinObservationWriter::ObserveRenewal, this));
    }
  }
  void Attach(Ptr<RoutingProtocol> route)
  {
    NS_ABORT_MSG_IF(!route || !route->TraceConnectWithoutContext("JoinMechanism",
      MakeCallback(&JoinObservationWriter::Observe, this)), "Missing CA join trace");
    NS_ABORT_MSG_IF(!route->TraceConnectWithoutContext("ProxyRenewal",
      MakeCallback(&JoinObservationWriter::ObserveRenewal, this)), "Missing CA renewal trace");
    m_routes.push_back(route);
  }
  void Finalize()
  {
    m_output.flush();
    m_renewal.flush();
    NS_ABORT_MSG_IF(!m_output || !m_renewal, "Cannot flush join observations");
    m_finalized = true;
  }
private:
  static std::string Id(uint16_t id) { return id == 0xFFFF ? "null" : std::to_string(id); }
  void ObserveRenewal(const ProxyRenewalEvent& e)
  {
    if (e.time > m_cutoff) return;
    NS_ABORT_MSG_IF(m_finalized, "Renewal event after finalization");
    std::ostringstream ip; ip << e.peerAddress;
    m_renewal << std::setprecision(17) << "{\"schema\":" << TaskObservationWriter::Quote(RENEWAL_SCHEMA)
      << ",\"time_ns\":" << e.time.GetNanoSeconds()
      << ",\"observer_node\":" << e.observerNodeId
      << ",\"origin_node\":" << Id(e.originNodeId)
      << ",\"head_node\":" << Id(e.headNodeId)
      << ",\"cluster_id\":" << Id(e.clusterId)
      << ",\"peer_ip\":" << (e.peerAddress.IsAny() ? "null" : TaskObservationWriter::Quote(ip.str()))
      << ",\"event\":" << TaskObservationWriter::Quote(e.event)
      << ",\"packet_uid\":" << e.packetUid
      << ",\"hello_uid\":" << (e.helloUid ? std::to_string(*e.helloUid) : "null")
      << ",\"source_stamp_s\":";
    if (std::isfinite(e.sourceStamp)) m_renewal << e.sourceStamp; else m_renewal << "null";
    m_renewal << ",\"source_time_ns\":" << (e.sourceTime ? std::to_string(e.sourceTime->GetNanoSeconds()) : "null")
      << ",\"previous_refresh_ns\":" << (e.previousRefresh ? std::to_string(e.previousRefresh->GetNanoSeconds()) : "null")
      << ",\"lifetime_ns\":" << e.lifetime.GetNanoSeconds()
      << ",\"time_rejections\":" << unsigned(e.timeRejections)
      << ",\"socket_result\":" << (e.socketResult ? std::to_string(*e.socketResult) : "null");
    if (e.observationUs) m_renewal << ",\"observation_us\":" << *e.observationUs;
    if (e.relatedPacketUid) m_renewal << ",\"related_uid\":" << *e.relatedPacketUid;
    if (e.holdoff) m_renewal << ",\"holdoff_ns\":" << e.holdoff->GetNanoSeconds();
    m_renewal << "}\n";
  }
  void Observe(const JoinMechanismEvent& e)
  {
    if (e.time > m_cutoff) return;
    NS_ABORT_MSG_IF(m_finalized, "Join event after finalization");
    std::ostringstream ip; ip << e.peerAddress;
    m_output << "{\"schema\":" << TaskObservationWriter::Quote(SCHEMA)
      << ",\"time_ns\":" << e.time.GetNanoSeconds()
      << ",\"observer_node\":" << e.observerNodeId
      << ",\"origin_node\":" << Id(e.originNodeId)
      << ",\"head_node\":" << Id(e.headNodeId)
      << ",\"cluster_id\":" << Id(e.clusterId)
      << ",\"peer_ip\":" << (e.peerAddress.IsAny() ? "null" : TaskObservationWriter::Quote(ip.str()))
      << ",\"event\":" << TaskObservationWriter::Quote(e.event)
      << ",\"local_role\":" << static_cast<int>(e.localRole)
      << ",\"local_cluster_id\":" << Id(e.localClusterId)
      << ",\"intent_head\":" << Id(e.intentHeadId)
      << ",\"proxy_enabled\":" << (e.proxyEnabled ? "true" : "false")
      << ",\"range_enabled\":" << (e.rangeEnabled ? "true" : "false")
      << ",\"socket_result\":" << (e.socketResult ? std::to_string(*e.socketResult) : "null");
    if (e.cmTrigger) m_output << ",\"cm_trigger\":" << TaskObservationWriter::Quote(e.cmTrigger);
    m_output << "}\n";
  }
  Time m_cutoff;
  std::ofstream m_output;
  std::ofstream m_renewal;
  bool m_finalized = false;
  std::vector<Ptr<RoutingProtocol>> m_routes;
};
} // namespace ns3::cafbcr
#endif
