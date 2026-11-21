/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef CA_FBCR_RUN_DIAGNOSTICS_H
#define CA_FBCR_RUN_DIAGNOSTICS_H

#include "ca-fbcr-task-observer.h"
#include "ca-fbcr-udp-observer.h"
#include "ns3/cafbcr-routing.h"
#include "ns3/cafbcr-packet.h"
#include "ns3/ipv4-l3-protocol.h"
#include "ns3/loopback-net-device.h"
#include "ns3/udp-client.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-mac.h"
#include "ns3/wifi-mpdu.h"
#include "ns3/wifi-phy.h"
#include "ns3/wifi-psdu.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <tuple>
#include <utility>
#include <vector>
#include <sys/resource.h>

namespace ns3::cafbcr {

// Entry-only passive diagnostics. No Packet creation/copy/tag writes, RNG,
// simulation events, peer-state queries or feedback to protocol decisions.
// PHY traces below pass existing PSDUs or scalar values, never GetPacket()
// on a received PSDU (which can allocate a new observation UID).
class RunDiagnostics
{
public:
  static constexpr const char* SCHEMA = "cafbcr-run-diagnostics-v2";
  struct Mark {
    std::chrono::steady_clock::time_point wall;
    std::clock_t cpu;
  };
  static Mark Now() { return {std::chrono::steady_clock::now(), std::clock()}; }
  enum Traffic { REQUEST, REPLY, UDP, TRAFFIC_COUNT };
  struct Counts {
    uint64_t ipTx = 0, revisitTx = 0, revisited = 0,
      missingRevisited = 0, missingTtlRevisited = 0, deliveredRevisited = 0;
  };

  RunDiagnostics(Time end, bool ca, std::string loop, std::string renewal,
                 bool compute, Mark setupStart)
    : m_end(end), m_ca(ca), m_compute(compute), m_loop(std::move(loop)),
      m_renewal(std::move(renewal)), m_setupStart(setupStart) {}
  ~RunDiagnostics() { Disconnect(); }

  void AttachNode(Ptr<Node> node, uint16_t controlPort)
  {
    Binding b; b.node = node->GetId(); b.port = controlPort;
    b.ip = node->GetObject<Ipv4L3Protocol>();
    NS_ABORT_MSG_IF(!b.ip, "Run diagnostics require IPv4");
    Check(b.ip->TraceConnectWithoutContext("Tx", MakeBoundCallback(&RunDiagnostics::IpTx, this, b.node, b.port)));
    Check(b.ip->TraceConnectWithoutContext("Drop", MakeCallback(&RunDiagnostics::Drop, this)));
    if (m_ca) {
      b.route = node->GetObject<RoutingProtocol>();
      NS_ABORT_MSG_IF(!b.route, "CA routing missing from diagnostic node");
      Check(b.route->TraceConnectWithoutContext("ProxyRenewal", MakeCallback(&RunDiagnostics::Renewal, this)));
    }
    for (uint32_t i = 0; i < node->GetNDevices(); ++i) {
      if (auto wifi = DynamicCast<WifiNetDevice>(node->GetDevice(i))) {
        auto phy = wifi->GetPhy(); auto mac = wifi->GetMac();
        Check(phy->TraceConnectWithoutContext("PhyTxPsduBegin", MakeCallback(&RunDiagnostics::PhyTx, this)));
        Check(phy->TraceConnectWithoutContext("PhyRxPayloadBegin", MakeCallback(&RunDiagnostics::PhyRx, this)));
        Check(mac->TraceConnectWithoutContext("MpduResponseTimeout", MakeCallback(&RunDiagnostics::ResponseTimeout, this)));
        b.phys.push_back(phy); b.macs.push_back(mac);
      }
    }
    m_bindings.push_back(std::move(b));
  }
  void AttachUdp(Ptr<UdpClient> client)
  {
    Check(client->TraceConnectWithoutContext("Tx", MakeCallback(&RunDiagnostics::UdpAttempt, this)));
    m_clients.push_back(client);
  }
  void BeginRun()
  {
    NS_ABORT_MSG_IF(m_begun, "Run diagnostic clock already started");
    m_runStart = Now(); m_eventsBefore = Simulator::GetEventCount(); m_begun = true;
  }
  void EndRun()
  {
    NS_ABORT_MSG_IF(!m_begun || m_ended, "Run diagnostic clock not active");
    m_runEnd = Now(); m_events = Simulator::GetEventCount() - m_eventsBefore; m_ended = true;
  }
  void Finalize(const TaskObservationWriter& tasks, const UdpObservationWriter& udp)
  {
    NS_ABORT_MSG_IF(m_finalized || !m_ended || Simulator::Now() < m_end, "Finalize diagnostics after the run");
    for (const auto& [key, r] : m_records) {
      if (!r.revisited) continue;
      ++m_counts[r.kind].revisited;
      Facts* f = r.kind == UDP ? &m_udpFacts[std::get<0>(key)] : &m_taskFacts[r.task];
      f->revisited = true; f->ttlRevisited |= r.ttl;
    }
    std::set<TaskKey> seen;
    tasks.VisitFinalTasks([&](uint32_t source, uint32_t sequence, bool rx, bool done, bool ack,
                             const std::string&) {
      for (auto kind : {REQUEST, REPLY}) {
        TaskKey key{source, sequence, kind}; seen.insert(key);
        const auto it = m_taskFacts.find(key);
        const Facts f = it == m_taskFacts.end() ? Facts{} : it->second;
        if (kind == REQUEST || done) Outcome(m_counts[kind], f, kind == REQUEST ? rx : ack);
      }
    });
    for (const auto& [key, facts] : m_taskFacts) if (!seen.count(key)) ++m_errors;
    uint64_t attempts = 0;
    udp.VisitFinalAttempts([&](uint64_t uid, bool delivered) {
      ++attempts;
      if (!m_udpAttempts.count(uid)) ++m_errors;
      const auto it = m_udpFacts.find(uid);
      Outcome(m_counts[UDP], it == m_udpFacts.end() ? Facts{} : it->second, delivered);
    });
    if (attempts != m_udpAttempts.size() || attempts != udp.GetCounts().windowAttempts) ++m_errors;
    m_finalized = true; Disconnect();
    NS_ABORT_MSG_IF(m_errors, "Run diagnostic identities do not reconcile");
  }
  const Counts& GetCounts(Traffic kind) const { return m_counts.at(kind); }

  static std::string CsvHeader()
  {
    std::ostringstream s;
    s << "RunDiagSchema,RunDiagErrors,LoopPolicy,ProxyRenewalPolicy,ComputeEnabled,"
      "SetupWall(s),SetupCpu(s),RunWall(s),RunCpu(s),PostprocessWall(s),PostprocessCpu(s),"
      "PeakRssKiB,RunEvents,WifiPpduTx,WifiPayloadRxBegin,MacResponseTimeoutEvents,"
      "ControlIpTx,ControlHelloIpTx,ControlCmIpTx,ControlOtherIpTx,"
      "RelayCmAttempts,HeadProxyAccepted,HeadProxyNotNewerRejected,HeadProxyOtherRejected,FragmentIpTxUntracked";
    for (const char* p : {"Req", "Reply", "UDP"})
      s << ',' << p << "IpTx," << p << "RevisitIpTx," << p << "RevisitedDatagrams,"
        << p << "MissingRevisited," << p << "MissingTtlRevisited," << p << "DeliveredRevisited";
    s << ",ControlIpBytes,ControlChLiveIpTx,ControlChLiveIpBytes,ControlChLiveTailBytes";
    return s.str();
  }
  uint64_t ControlCmIpTx() const
  {
    NS_ABORT_MSG_IF(!m_finalized || !m_ca, "CM summary requires finalized CA diagnostics");
    return m_cm;
  }

  std::string CsvValues() const
  {
    NS_ABORT_MSG_IF(!m_finalized, "Run diagnostics not finalized");
    const auto end = Now(); struct rusage usage{};
    NS_ABORT_MSG_IF(getrusage(RUSAGE_SELF, &usage) != 0, "Cannot read peak RSS");
    std::ostringstream s; s << std::setprecision(12);
    s << SCHEMA << ',' << m_errors << ',' << m_loop << ',' << m_renewal << ',' << m_compute
      << ',' << Wall(m_setupStart, m_runStart) << ',' << Cpu(m_setupStart, m_runStart)
      << ',' << Wall(m_runStart, m_runEnd) << ',' << Cpu(m_runStart, m_runEnd)
      << ',' << Wall(m_runEnd, end) << ',' << Cpu(m_runEnd, end)
      << ',' << usage.ru_maxrss << ',' << m_events << ',' << m_ppduTx << ',' << m_payloadRx
      << ',' << m_timeouts << ',' << m_control;
    if (m_ca) s << ',' << m_hello << ',' << m_cm << ',' << m_control - m_hello - m_cm
                << ',' << m_relayCm << ',' << m_headAccepted << ',' << m_headNotNewer << ',' << m_headOther;
    else s << ",N/A,N/A,N/A,N/A,N/A,N/A,N/A";
    s << ',' << m_fragments;
    for (const auto& c : m_counts)
      s << ',' << c.ipTx << ',' << c.revisitTx << ',' << c.revisited << ',' << c.missingRevisited
        << ',' << c.missingTtlRevisited << ',' << c.deliveredRevisited;
    s << ',' << m_controlBytes;
    if (m_ca) s << ',' << m_chLive << ',' << m_chLiveBytes << ',' << m_chLiveTailBytes;
    else s << ",N/A,N/A,N/A";
    return s.str();
  }

private:
  friend class CafbcrRunDiagnosticsCase;
  using TaskKey = std::tuple<uint32_t, uint32_t, Traffic>;
  using DatagramKey = std::tuple<uint64_t, uint32_t, uint32_t>;
  struct Facts { bool revisited = false, ttlRevisited = false; };
  struct Record {
    Traffic kind; TaskKey task{}; std::map<uint32_t,uint8_t> lowest;
    bool revisited = false, ttl = false;
  };
  struct Binding {
    uint32_t node; uint16_t port; Ptr<Ipv4L3Protocol> ip; Ptr<RoutingProtocol> route;
    std::vector<Ptr<WifiPhy>> phys; std::vector<Ptr<WifiMac>> macs;
  };
  static void Check(bool ok) { NS_ABORT_MSG_IF(!ok, "Missing native run diagnostic trace"); }
  static double Wall(Mark a, Mark b) { return std::chrono::duration<double>(b.wall-a.wall).count(); }
  static double Cpu(Mark a, Mark b) { return double(b.cpu-a.cpu)/CLOCKS_PER_SEC; }
  bool InCapture() const { return Simulator::Now() <= m_end; }
  static void Outcome(Counts& c, Facts f, bool delivered)
  {
    if (!f.revisited) return;
    if (delivered) ++c.deliveredRevisited;
    else { ++c.missingRevisited; c.missingTtlRevisited += f.ttlRevisited; }
  }
  Record* Identify(const Ipv4Header& h, Ptr<const Packet> p)
  {
    if (h.GetProtocol() != 17 || h.GetFragmentOffset() || !h.IsLastFragment()) return nullptr;
    const DatagramKey key{p->GetUid(), h.GetSource().Get(), h.GetDestination().Get()};
    auto it = m_records.find(key); if (it != m_records.end()) return &it->second;
    CafbcrComputeTaskTag tag;
    if (p->PeekPacketTag(tag) && std::isfinite(tag.GetWorkload()) && tag.GetWorkload() >= 0) {
      const Traffic kind = tag.GetWorkload() == 0 ? REPLY : REQUEST;
      return &m_records.emplace(key, Record{kind,{tag.GetSourceNodeId(),tag.GetTaskId(),kind}}).first->second;
    }
    if (m_udpAttempts.count(p->GetUid())) return &m_records.emplace(key, Record{UDP}).first->second;
    return nullptr;
  }
  void TrackTx(uint32_t node, const Ipv4Header& h, Ptr<const Packet> p)
  {
    if (!InCapture()) return;
    auto r = Identify(h,p); if (!r) return;
    ++m_counts[r->kind].ipTx;
    auto [it, fresh] = r->lowest.emplace(node,h.GetTtl());
    if (!fresh && h.GetTtl() < it->second) {
      ++m_counts[r->kind].revisitTx; r->revisited = true; it->second = h.GetTtl();
    }
  }
  static void IpTx(RunDiagnostics* self, uint32_t node, uint16_t port, Ptr<const Packet> p,
                   Ptr<Ipv4> ip, uint32_t interface)
  {
    if (!self->InCapture() || DynamicCast<LoopbackNetDevice>(ip->GetNetDevice(interface))) return;
    Ipv4Header h; p->PeekHeader(h);
    if (h.GetFragmentOffset() || !h.IsLastFragment()) { ++self->m_fragments; return; }
    std::array<uint8_t,69> bytes{};
    const auto n = p->CopyData(bytes.data(),std::min<uint32_t>(bytes.size(),p->GetSize()));
    const auto ihl = h.GetSerializedSize();
    if (h.GetProtocol()==17 && n >= ihl+8 && ((bytes[ihl]<<8)|bytes[ihl+1])==port &&
        ip->GetInterfaceForAddress(h.GetSource()) >= 0) {
      ++self->m_control;
      self->m_controlBytes += p->GetSize();
      if (self->m_ca && n > ihl+8) {
        self->m_hello += bytes[ihl+8]==CAFBCR_HELLO;
        self->m_cm += bytes[ihl+8]==CAFBCR_CM_LIVE;
        if (bytes[ihl+8]==CAFBCR_CH_LIVE) {
          ++self->m_chLive;
          self->m_chLiveBytes += p->GetSize();
          // Charge every byte beyond the fixed type + CH header, including
          // unknown/malformed tails. This is an IP cost, not proof that any
          // record was valid, received, or applied. No Packet copy/UID needed.
          const auto base = ihl + 8 + 1 + ClusterHeadBeaconHeader().GetSerializedSize();
          if (p->GetSize() > base) self->m_chLiveTailBytes += p->GetSize() - base;
        }
      }
      return;
    }
    self->TrackTx(node,h,p);
  }
  void Drop(const Ipv4Header& h, Ptr<const Packet> p, Ipv4L3Protocol::DropReason reason,
            Ptr<Ipv4>, uint32_t)
  {
    if (!InCapture() || reason != Ipv4L3Protocol::DROP_TTL_EXPIRED) return;
    if (auto r=Identify(h,p)) r->ttl=true;
  }
  void UdpAttempt(Ptr<const Packet> p)
  {
    if (Simulator::Now()>=Seconds(30) && Simulator::Now()<m_end &&
        !m_udpAttempts.insert(p->GetUid()).second) ++m_errors;
  }
  void PhyTx(WifiConstPsduMap, WifiTxVector, double) { if (InCapture()) ++m_ppduTx; }
  void PhyRx(WifiTxVector, Time) { if (InCapture()) ++m_payloadRx; }
  void ResponseTimeout(uint8_t, Ptr<const WifiMpdu>, const WifiTxVector&)
  { if (InCapture()) ++m_timeouts; }
  void Renewal(const ProxyRenewalEvent& e)
  {
    if (!InCapture()) return;
    if (e.event=="relay_cm_tx_attempt") ++m_relayCm;
    else if (e.event=="head_proxy_accepted") ++m_headAccepted;
    else if (e.event=="head_time_rejected") {
      if (e.timeRejections & ProxyRenewalEvent::NotNewer) ++m_headNotNewer;
      else ++m_headOther;
    } else if (e.event=="head_association_rejected") ++m_headOther;
  }
  void Disconnect()
  {
    for (auto& b : m_bindings) {
      b.ip->TraceDisconnectWithoutContext("Tx",MakeBoundCallback(&RunDiagnostics::IpTx,this,b.node,b.port));
      b.ip->TraceDisconnectWithoutContext("Drop",MakeCallback(&RunDiagnostics::Drop,this));
      if (b.route) b.route->TraceDisconnectWithoutContext("ProxyRenewal",MakeCallback(&RunDiagnostics::Renewal,this));
      for (auto p : b.phys) {
        p->TraceDisconnectWithoutContext("PhyTxPsduBegin",MakeCallback(&RunDiagnostics::PhyTx,this));
        p->TraceDisconnectWithoutContext("PhyRxPayloadBegin",MakeCallback(&RunDiagnostics::PhyRx,this));
      }
      for (auto m : b.macs) m->TraceDisconnectWithoutContext("MpduResponseTimeout",MakeCallback(&RunDiagnostics::ResponseTimeout,this));
    }
    for (auto c : m_clients) c->TraceDisconnectWithoutContext("Tx",MakeCallback(&RunDiagnostics::UdpAttempt,this));
    m_bindings.clear(); m_clients.clear();
  }

  Time m_end;
  bool m_ca, m_compute, m_begun=false, m_ended=false, m_finalized=false;
  std::string m_loop, m_renewal;
  Mark m_setupStart{},m_runStart{},m_runEnd{};
  uint64_t m_errors=0,m_eventsBefore=0,m_events=0,m_ppduTx=0,m_payloadRx=0,m_timeouts=0,
    m_control=0,m_hello=0,m_cm=0,m_relayCm=0,m_headAccepted=0,m_headNotNewer=0,m_headOther=0,m_fragments=0,
    m_controlBytes=0,m_chLive=0,m_chLiveBytes=0,m_chLiveTailBytes=0;
  std::array<Counts,TRAFFIC_COUNT> m_counts{};
  std::map<DatagramKey,Record> m_records;
  std::map<TaskKey,Facts> m_taskFacts;
  std::map<uint64_t,Facts> m_udpFacts;
  std::set<uint64_t> m_udpAttempts;
  std::vector<Binding> m_bindings;
  std::vector<Ptr<UdpClient>> m_clients;
};
} // namespace ns3::cafbcr
#endif
