/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef CA_FBCR_MOGNN_DIAGNOSTICS_H
#define CA_FBCR_MOGNN_DIAGNOSTICS_H

#include "ca-fbcr-task-observer.h"
#include "ns3/mo-routing.h"
#include "ns3/arp-cache.h"
#include "ns3/arp-l3-protocol.h"
#include "ns3/ipv4-l3-protocol.h"
#include "ns3/ipv4-interface.h"
#include "ns3/loopback-net-device.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-mac.h"
#include "ns3/wifi-mpdu.h"
#include "ns3/wifi-phy.h"
#include "ns3/wifi-phy-state-helper.h"
#include "ns3/udp-client.h"
#include <array>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <set>

namespace ns3::cafbcr {

// Measurement only. Never reads another node on behalf of a routing decision,
// schedules events, consumes random numbers, or modifies packet bytes/tags.
class MognnDiagnostics
{
public:
  MognnDiagnostics(const std::string& directory, Time start, Time stop, Time cutoff)
    : m_directory(directory), m_start(start), m_stop(stop), m_cutoff(cutoff)
  {
    const std::filesystem::path p(directory);
    if (!p.parent_path().empty()) std::filesystem::create_directories(p.parent_path());
    NS_ABORT_MSG_IF(!std::filesystem::create_directory(p), "MOGNN diagnostic directory already exists");
    m_taskDetails.open(p / "task-network.jsonl");
    NS_ABORT_MSG_IF(!m_taskDetails, "Cannot open task network trace");
    m_samples.open(p / "packet-events.jsonl");
    NS_ABORT_MSG_IF(!m_samples, "Cannot open MO diagnostic samples");
  }

  void Attach(Ptr<Node> node)
  {
    const auto id = std::to_string(node->GetId());
    auto routing = node->GetObject<mo::RoutingProtocol>();
    Connect(routing, "RouteDecision", id, MakeCallback(&MognnDiagnostics::Route, this));
    Connect(routing, "ControlEvent", id, MakeCallback(&MognnDiagnostics::Control, this));
    Connect(routing, "RelayQueueEvent", id, MakeCallback(&MognnDiagnostics::RelayQueue, this));
    Connect(routing, "GraphSnapshot", id, MakeCallback(&MognnDiagnostics::Graph, this));
    Connect(routing, "SelectedHop", id, MakeCallback(&MognnDiagnostics::SelectedHop, this));
    Connect(node->GetObject<ArpL3Protocol>(), "Drop", id, MakeCallback(&MognnDiagnostics::ArpProtocolDrop, this));
    auto ip = node->GetObject<Ipv4L3Protocol>();
    Connect(ip, "Drop", id, MakeCallback(&MognnDiagnostics::IpDrop, this));
    Connect(ip, "Tx", id, MakeCallback(&MognnDiagnostics::IpTx, this));
    for (uint32_t i = 0; i < ip->GetNInterfaces(); ++i) {
      auto cache = ip->GetInterface(i)->GetArpCache();
      if (cache) Connect(cache, "Drop", id, MakeCallback(&MognnDiagnostics::ArpDrop, this));
    }
    for (uint32_t i = 0; i < node->GetNDevices(); ++i) {
      auto wifi = DynamicCast<WifiNetDevice>(node->GetDevice(i));
      if (!wifi) continue;
      Connect(wifi->GetMac(), "MacTxDrop", id, MakeCallback(&MognnDiagnostics::MacDrop, this));
      Connect(wifi->GetMac(), "DroppedMpdu", id, MakeCallback(&MognnDiagnostics::MpduDrop, this));
      Connect(wifi->GetPhy(), "PhyRxDrop", id, MakeCallback(&MognnDiagnostics::PhyRxDrop, this));
      Connect(wifi->GetPhy(), "PhyTxDrop", id, MakeCallback(&MognnDiagnostics::PhyTxDrop, this));
      Connect(wifi->GetPhy()->GetState(), "State", id, MakeCallback(&MognnDiagnostics::State, this));
    }
  }
  template<class T> void AttachTask(Ptr<T> app)
  {
    NS_ABORT_MSG_IF(!app->TraceConnectWithoutContext("TaskWire", MakeCallback(&MognnDiagnostics::Wire, this)), "Missing MO task wire trace");
    NS_ABORT_MSG_IF(!app->TraceConnectWithoutContext("TaskEvent", MakeCallback(&MognnDiagnostics::Task, this)), "Missing MO task lifecycle trace");
  }
  void AttachUdp(Ptr<UdpClient> app)
  { Connect(app, "Tx", std::to_string(app->GetNode()->GetId()), MakeCallback(&MognnDiagnostics::UdpAttempt, this)); }
  uint64_t Count(const std::string& stage, const std::string& kind, const std::string& reason) const
  {
    uint64_t n = 0;
    for (const auto& [key, v] : m_counts) if (key[1] == stage && key[2] == kind && key[3] == reason) n += v.count;
    return n;
  }
  void Finalize(const std::string& build, uint32_t seed, uint64_t run, const std::string& scenario)
  {
    NS_ABORT_MSG_IF(m_finalized || Simulator::Now() < m_cutoff, "Premature MO diagnostic finalization");
    m_finalized = true;
    std::ofstream out(m_directory / "summary.json");
    out << "{\"schema\":\"mognn-passive-diagnostics-v1\",\"build_id\":" << Q(build)
        << ",\"seed\":" << seed << ",\"rng_run\":" << run << ",\"scenario\":" << scenario
        << ",\"start_ns\":" << m_start.GetNanoSeconds() << ",\"stop_ns\":" << m_stop.GetNanoSeconds()
        << ",\"cutoff_ns\":" << m_cutoff.GetNanoSeconds()
        << ",\"semantics\":{\"counts\":\"layer events, not additive unique packet/task losses; route probes separate\","
           "\"control_bytes\":\"ControlEvent is protocol payload; ipv4_tx includes IP/UDP; neither proves airtime\","
           "\"graph\":\"sampled only at existing policy refreshes; not a uniform-time average\","
           "\"phy_state\":\"reported intervals clipped to each window; unreported trailing intervals remain censored\","
           "\"phy_drop\":\"unclassified raw PHY events; may include overhearing and non-business frames\","
           "\"packet_uid\":\"lineage identifier, not independent replicate or universal per-hop identifier\"}"
        << ",\"sample_limit\":" << SAMPLE_LIMIT << ",\"samples_written\":" << m_written
        << ",\"samples_omitted\":" << m_omitted << ",\"counters\":[";
    bool comma = false;
    for (const auto& [k, v] : m_counts) {
      if (comma) out << ',';
      comma = true;
      out << "{\"window\":" << Q(k[0]) << ",\"stage\":" << Q(k[1]) << ",\"class\":" << Q(k[2])
          << ",\"reason\":" << Q(k[3]) << ",\"count\":" << v.count << ",\"bytes\":" << v.bytes << ",\"max_bytes\":" << v.maxBytes << ",\"nodes\":{";
      bool sep = false;
      for (const auto& [node, n] : v.nodes) { if (sep) out << ','; sep = true; out << Q(node) << ':' << n; }
      out << "}}";
    }
    out << "],\"relay_queue\":["; comma = false;
    for (const auto& [k, v] : m_relayQueue) {
      if (comma) out << ',';
      comma = true;
      out << "{\"window\":" << Q(k[0]) << ",\"node\":" << Q(k[1]) << ",\"reason\":" << Q(k[2])
          << ",\"count\":" << v.count << ",\"wait_sum_ns\":" << v.waitSum
          << ",\"max_wait_ns\":" << v.waitMax << ",\"max_pending\":" << v.maxPending << '}';
    }
    out << "],\"graphs\":["; comma = false;
    for (const auto& [key, g] : m_graphs) {
      if (comma) out << ',';
      comma = true;
      out << "{\"window\":" << Q(key.first) << ",\"node\":" << Q(key.second) << ",\"samples\":" << g.samples
          << ",\"zero_bidirectional\":" << g.zeroDirect << ",\"zero_reachable_servers\":" << g.zeroServers
          << ",\"fields\":[\"fresh_records\",\"heard_peers\",\"bidirectional_peers\",\"graph_edges\",\"reachable_remote_servers\"],\"sums\":[";
      for (int i = 0; i < 5; ++i) { if (i) out << ','; out << g.sums[i]; }
      out << "],\"last\":[";
      for (int i = 0; i < 5; ++i) { if (i) out << ','; out << g.last[i]; }
      out << "]}";
    }
    out << "],\"phy_state_intervals\":["; comma = false;
    for (const auto& [k, ns] : m_states) {
      if (comma) out << ',';
      comma = true;
      out << "{\"window\":" << Q(k[0]) << ",\"node\":" << Q(k[1]) << ",\"state\":" << Q(k[2]) << ",\"ns\":" << ns << '}';
    }
    out << "],\"task_network_written\":" << m_taskWritten << ",\"task_network_omitted\":" << m_taskOmitted << ",\"task_network_limit\":100000}\n"; out.close(); m_samples.close(); m_taskDetails.close();
    NS_ABORT_MSG_IF(!m_taskDetails, "Cannot finish task network trace");
    NS_ABORT_MSG_IF(!out || !m_samples, "Cannot finish MO diagnostics");
  }
private:
  struct Total { uint64_t count = 0, bytes = 0; uint32_t maxBytes = 0; std::map<std::string, uint64_t> nodes; };
  struct QueueTotal { uint64_t count = 0, waitSum = 0, waitMax = 0; uint32_t maxPending = 0; };
  struct Snapshot { uint64_t samples = 0, zeroDirect = 0, zeroServers = 0; std::array<uint64_t,5> sums{}, last{}; };
  static constexpr uint64_t SAMPLE_LIMIT = 20000;
  static std::string Q(const std::string& s) { return TaskObservationWriter::Quote(s); }
  template<class T> static void Connect(Ptr<T> object, const char* trace, const std::string& id, const CallbackBase& cb)
  { NS_ABORT_MSG_IF(!object || !object->TraceConnect(trace, id, cb), "Missing MO diagnostic trace " << trace); }
  std::string Window() const { return Simulator::Now() < m_start ? "warmup" : Simulator::Now() < m_stop ? "demand" : "drain"; }
  bool Active() const { return !m_finalized && Simulator::Now() <= m_cutoff; }
  static uint16_t U16(const uint8_t* p) { return (uint16_t(p[0]) << 8) | p[1]; }
  static std::string Ports(uint16_t src, uint16_t dst)
  {
    if (src == 26920 || dst == 26920) return "control";
    if (dst == 9999) return "task_request";
    if (src == 9999) return "task_reply";
    return "other_udp";
  }
  std::string Known(Ptr<const Packet> p) const
  {
    CafbcrComputeTaskTag tag;
    if (p->PeekPacketTag(tag)) return tag.GetWorkload()==0 ? "task_reply" : "task_request";
    if (m_udpAttempts.count(p->GetUid())) return "business_udp";
    return "";
  }
  std::string Udp(Ptr<const Packet> p, const Ipv4Header& h, bool output=false) const
  {
    if (!p) return "route_probe";
    const auto known=Known(p); if (!known.empty()) return known;
    // UdpSocketImpl calls RouteOutput before adding the UDP header. Do not
    // interpret application bytes as ports. IpL3/forwarded packets have an IP
    // payload size; registered business UIDs and task tags work in both paths.
    // The unset source is Ipv4Address's 0x66666666 sentinel, not Any(). A UDP
    // datagram with its header present has a nonzero IP payload size.
    if (output && h.GetPayloadSize()==0) return "socket_payload_unclassified";
    if (h.GetProtocol() != 17 || h.GetFragmentOffset() != 0 || p->GetSize() < 8) return "other_ip";
    uint8_t b[8]; p->CopyData(b,8); return Ports(U16(b), U16(b+2));
  }
  std::string Frame(Ptr<const Packet> p) const
  {
    if (!p) return "unknown";
    const auto known=Known(p); if (!known.empty()) return known;
    uint8_t b[76]{}; auto n=p->CopyData(b,std::min<uint32_t>(sizeof(b),p->GetSize()));
    uint32_t offset=0;
    if (n>=8 && b[0]==0xaa && b[1]==0xaa && b[2]==3) {
      if (U16(b+6)==0x0806) return "arp";
      if (U16(b+6)!=0x0800) return "other_l2";
      offset=8;
    }
    if (n<offset+20 || b[offset]>>4!=4) return "unclassified";
    auto ip=b+offset; uint32_t ihl=(ip[0]&15)*4;
    if (ihl<20 || n<offset+ihl+8 || ip[9]!=17 || (U16(ip+6)&0x1fff)) return "other_ip";
    return Ports(U16(ip+ihl),U16(ip+ihl+2));
  }
  void Add(const std::string& node, const std::string& stage, const std::string& kind, const std::string& reason,
           uint32_t bytes, Ptr<const Packet> p=nullptr)
  {
    if (!Active()) return;
    auto& v=m_counts[{Window(),stage,kind,reason}]; ++v.count; v.bytes+=bytes; v.maxBytes=std::max(v.maxBytes,bytes); ++v.nodes[node];
    if (p) TaskDetail(node,p,stage+":"+reason,"{}");
    if (!p || (kind!="task_request" && kind!="task_reply" && kind!="business_udp" && kind!="other_udp")) return;
    if (m_written>=SAMPLE_LIMIT) { ++m_omitted; return; }
    ++m_written;
    CafbcrComputeTaskTag tag; const bool task=p->PeekPacketTag(tag);
    m_samples << "{\"time_ns\":" << Simulator::Now().GetNanoSeconds() << ",\"node\":" << Q(node)
      << ",\"stage\":" << Q(stage) << ",\"class\":" << Q(kind) << ",\"reason\":" << Q(reason)
      << ",\"uid\":" << p->GetUid() << ",\"bytes\":" << bytes
      << ",\"task_source\":" << (task ? std::to_string(tag.GetSourceNodeId()) : "null")
      << ",\"task_id\":" << (task ? std::to_string(tag.GetTaskId()) : "null") << "}\n";
  }
  void Route(std::string node, Ptr<const Packet> p, const Ipv4Header& h, std::string stage, std::string reason)
  { Add(node,stage,Udp(p,h,stage=="route_output"),reason,p?p->GetSize():0,p); }
  void UdpAttempt(std::string node, Ptr<const Packet> p)
  {
    if (!Active()) return;
    m_udpAttempts.insert(p->GetUid());
    Add(node,"udp_application","business_udp","attempt",p->GetSize(),p);
  }
  void Control(std::string node, uint32_t bytes, std::string reason) { Add(node,"control","control",reason,bytes); }
  void RelayQueue(std::string node, std::string reason, uint32_t pending, uint64_t wait)
  {
    if (!Active()) return;
    auto& v = m_relayQueue[{Window(),node,reason}];
    ++v.count; v.waitSum += wait; v.waitMax = std::max(v.waitMax, wait);
    v.maxPending = std::max(v.maxPending, pending);
  }
  void Graph(std::string node, uint32_t records, uint32_t peers, uint32_t direct, uint32_t edges, uint32_t servers)
  {
    if (!Active()) return;
    auto& g=m_graphs[{Window(),node}]; ++g.samples; g.zeroDirect+=direct==0; g.zeroServers+=servers==0;
    g.last={records,peers,direct,edges,servers}; for(int i=0;i<5;++i) g.sums[i]+=g.last[i];
  }
  void IpDrop(std::string n, const Ipv4Header& h, Ptr<const Packet> p, Ipv4L3Protocol::DropReason why, Ptr<Ipv4>, uint32_t)
  { Add(n,"ipv4_drop",Udp(p,h),std::to_string(unsigned(why)),p->GetSize(),p); }
  void IpTx(std::string n, Ptr<const Packet> p, Ptr<Ipv4> ip, uint32_t interface)
  { Add(n,DynamicCast<LoopbackNetDevice>(ip->GetNetDevice(interface))?"ipv4_tx_loopback":"ipv4_tx",Frame(p),"candidate",p->GetSize(),p); }
  void TaskDetail(const std::string& node, Ptr<const Packet> p, const std::string& event, const std::string& detail)
  {
    if (!Active() || !p) return;
    CafbcrComputeTaskTag tag; if (!p->PeekPacketTag(tag)) return;
    if (m_taskWritten >= 100000) { ++m_taskOmitted; return; }
    ++m_taskWritten;
    m_taskDetails << "{\"time_ns\":" << Simulator::Now().GetNanoSeconds() << ",\"node\":" << Q(node)
      << ",\"uid\":" << p->GetUid() << ",\"task_source\":" << tag.GetSourceNodeId()
      << ",\"task_id\":" << tag.GetTaskId() << ",\"reply\":" << (tag.GetWorkload()==0?"true":"false")
      << ",\"event\":" << Q(event) << ",\"detail\":" << detail << "}\n";
  }
  void SelectedHop(std::string n, Ptr<const Packet> p, std::string detail)
  { TaskDetail(n,p,"selected_hop",detail); }
  void ArpProtocolDrop(std::string n, Ptr<const Packet> p)
  {
    // This trace also carries rejected ARP replies; only known task/UDP tags
    // are classified as business. Do not interpret raw ARP bytes as IPv4.
    const auto known=Known(p);
    Add(n,"arp_protocol_drop",known.empty()?"unclassified_arp_or_ip":known,"protocol_drop_reason_not_exposed",p->GetSize(),p);
  }
  void ArpDrop(std::string n, Ptr<const Packet> p) { Add(n,"arp_drop",Frame(p),"unresolved_or_pending_queue",p->GetSize(),p); }
  void MacDrop(std::string n, Ptr<const Packet> p) { Add(n,"mac_tx_drop",Frame(p),"drop",p->GetSize(),p); }
  void MpduDrop(std::string n, WifiMacDropReason reason, Ptr<const WifiMpdu> mpdu)
  { auto p=mpdu->GetPacket(); Add(n,"mac_mpdu_drop",Frame(p),std::to_string(unsigned(reason)),p->GetSize(),p); }
  void PhyRxDrop(std::string n, Ptr<const Packet> p, WifiPhyRxfailureReason why)
  { Add(n,"phy_rx_drop","unclassified",std::to_string(unsigned(why)),p->GetSize()); }
  void PhyTxDrop(std::string n, Ptr<const Packet> p) { Add(n,"phy_tx_drop","unclassified","drop",p->GetSize()); }
  void State(std::string n, Time start, Time duration, WifiPhyState state)
  {
    if (m_finalized) return;
    const std::array<Time,4> bounds={Seconds(0),m_start,m_stop,m_cutoff};
    const std::array<std::string,3> windows={"warmup","demand","drain"};
    std::ostringstream label; label << state;
    for (int i=0;i<3;++i) {
      auto a=std::max(start,bounds[i]), b=std::min(start+duration,bounds[i+1]);
      if (b>a) m_states[{windows[i],n,label.str()}]+=(b-a).GetNanoSeconds();
    }
  }
  void Wire(const ComputeTaskWireEvent& e)
  {
    std::string reason=e.event+":"+e.reason;
    if (e.socketResult) reason+=*e.socketResult>=0?":socket_accepted":":socket_failed";
    if (!e.valid) reason+=":invalid";
    Add(std::to_string(e.observerNodeId),"task_wire",e.event.find("response")==0?"task_reply":"task_request",reason,e.packet->GetSize(),e.packet);
  }
  void Task(const ComputeTaskEvent& e)
  { Add(std::to_string(e.observerNodeId),"task_lifecycle","task",e.event,0); }
  std::filesystem::path m_directory;
  Time m_start,m_stop,m_cutoff;
  bool m_finalized=false;
  std::ofstream m_samples, m_taskDetails;
  uint64_t m_taskWritten=0, m_taskOmitted=0;
  uint64_t m_written=0,m_omitted=0;
  std::map<std::array<std::string,4>,Total> m_counts;
  std::map<std::pair<std::string,std::string>,Snapshot> m_graphs;
  std::map<std::array<std::string,3>,QueueTotal> m_relayQueue;
  std::map<std::array<std::string,3>,uint64_t> m_states;
  std::set<uint64_t> m_udpAttempts;
};
}
#endif
