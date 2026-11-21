/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef CA_FBCR_LINK_OBSERVER_H
#define CA_FBCR_LINK_OBSERVER_H
#include "ca-fbcr-task-observer.h"
#include "ns3/cafbcr-routing.h"
#include <tuple>

namespace ns3::cafbcr {
// Passive trace serialization, without neighbor, mobility or global-state queries.
class LinkObservationWriter
{
public:
  static constexpr const char* SCHEMA="cafbcr-bidirectional-gate-v1";
  LinkObservationWriter(const std::string& directory,Time cutoff):m_directory(directory),m_cutoff(cutoff)
  {
    auto p=m_directory/"link-probes.csv";
    NS_ABORT_MSG_IF(std::filesystem::exists(p),"Preserve link observations");
    m_output.open(p);NS_ABORT_MSG_IF(!m_output,"Cannot open link observations");
    m_output<<"time_ns,observer,peer,peer_ip,event,state,nonce,packet_uid,source_ns,deadline_ns,confirmed_rx_ns,admitted\n";
  }
  ~LinkObservationWriter()
  {
    for (auto r:m_routes) r->TraceDisconnectWithoutContext("LinkProbe",MakeCallback(&LinkObservationWriter::Observe,this));
  }
  void Attach(Ptr<RoutingProtocol> route)
  {
    NS_ABORT_MSG_IF(!route || !route->TraceConnectWithoutContext("LinkProbe",MakeCallback(&LinkObservationWriter::Observe,this)),"Missing link trace");
    m_routes.push_back(route);
  }
  void Finalize()
  {
    if (m_finalized) return;
    m_finalized=true;m_output.flush();
    if (!m_output) ++m_errors;
    std::ofstream s(m_directory/"link-probes-summary.json");
    s<<"{\"schema\":\""<<SCHEMA<<"\",\"formal_eligible\":false,\"measurement_valid\":"<<(m_errors==0?"true":"false")
      <<",\"window_complete\":"<<(Simulator::Now()>=m_cutoff?"true":"false")
      <<",\"cutoff_ns\":"<<m_cutoff.GetNanoSeconds()<<",\"capture_ns\":"<<Simulator::Now().GetNanoSeconds()
      <<",\"errors\":"<<m_errors<<",\"events\":{";
    bool first=true;for (const auto& kv:m_counts) {if (!first)s<<',';first=false;s<<TaskObservationWriter::Quote(kv.first)<<':'<<kv.second;}
    s<<"},\"pending_at_cutoff\":[";first=true;
    for (const auto& kv:m_pending) {
      if (!first)s<<',';
      first=false;
      s<<"{\"observer\":"<<std::get<0>(kv.first)<<",\"peer\":"<<std::get<1>(kv.first)
        <<",\"nonce\":"<<std::get<2>(kv.first)<<",\"source_ns\":"<<kv.second.first.GetNanoSeconds()
        <<",\"deadline_ns\":"<<kv.second.second.GetNanoSeconds()<<'}';
    }
    s<<"],\"observed_roundtrip_is_not_future_delivery_guarantee\":true}\n";
    NS_ABORT_MSG_IF(!s,"Cannot write link summary");
  }
private:
  void Observe(const LinkProbeEvent& e)
  {
    if (m_finalized || e.time>m_cutoff) return;
    ++m_counts[e.event];
    const auto key=std::make_tuple(e.observer,e.peer,e.nonce.value_or(0));
    if (e.event=="request_scheduled") {
      if (!e.nonce || !e.sent || !e.deadline || m_pending.count(key)) ++m_errors;
      else m_pending.emplace(key,std::make_pair(*e.sent,*e.deadline));
    } else if (e.event=="reply_accepted" || e.event=="timeout" || e.event=="cancelled") {
      if (m_pending.erase(key)!=1) ++m_errors;
    }
    m_output<<e.time.GetNanoSeconds()<<','<<e.observer<<','<<e.peer<<','<<e.peerIp<<','<<e.event<<','<<e.state<<',';
    if(e.nonce)m_output<<*e.nonce;else m_output<<"null";m_output<<',';
    if(e.packetUid)m_output<<*e.packetUid;else m_output<<"null";m_output<<',';
    if(e.sent)m_output<<e.sent->GetNanoSeconds();else m_output<<"null";m_output<<',';
    if(e.deadline)m_output<<e.deadline->GetNanoSeconds();else m_output<<"null";m_output<<',';
    if(e.confirmed)m_output<<e.confirmed->GetNanoSeconds();else m_output<<"null";m_output<<',';
    if(e.admitted)m_output<<(*e.admitted?1:0);else m_output<<"null";m_output<<'\n';
  }
  std::filesystem::path m_directory;Time m_cutoff;std::ofstream m_output;
  bool m_finalized=false;uint64_t m_errors=0;
  std::vector<Ptr<RoutingProtocol>> m_routes;
  std::map<std::string,uint64_t> m_counts;
  std::map<std::tuple<uint32_t,uint16_t,uint64_t>,std::pair<Time,Time>> m_pending;
};
}
#endif
