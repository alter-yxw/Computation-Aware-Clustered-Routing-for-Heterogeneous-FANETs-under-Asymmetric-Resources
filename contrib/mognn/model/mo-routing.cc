/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "mo-routing.h"
#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/inet-socket-address.h"
#include "ns3/ipv4-route.h"
#include "ns3/node.h"
#include "ns3/output-stream-wrapper.h"
#include "ns3/simulator.h"
#include "ns3/udp-socket-factory.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <stdexcept>
#include <sstream>

namespace ns3::mo
{
namespace
{
constexpr uint16_t PORT = 26920;
class PathTag : public Tag
{
  public:
    std::vector<uint32_t> path;
    static TypeId GetTypeId()
    {
        static TypeId id = TypeId("ns3::mo::PathTag").SetParent<Tag>().AddConstructor<PathTag>(); return id;
    }
    TypeId GetInstanceTypeId() const override { return GetTypeId(); }
    uint32_t GetSerializedSize() const override { return 4 + 4 * path.size(); }
    void Serialize(TagBuffer b) const override { b.WriteU32(path.size()); for (auto n : path) b.WriteU32(n); }
    void Deserialize(TagBuffer b) override { auto size = b.ReadU32(); path.resize(std::min(size, 256u)); for (auto& n : path) n = b.ReadU32(); }
    void Print(std::ostream& out) const override { out << "MO visited=" << path.size(); }
};
}
NS_OBJECT_ENSURE_REGISTERED(RoutingProtocol);
TypeId RoutingProtocol::GetTypeId()
{
    static TypeId id = TypeId("ns3::mo::RoutingProtocol").SetParent<Ipv4RoutingProtocol>()
        .SetGroupName("MoGnn").AddConstructor<RoutingProtocol>()
        .AddTraceSource("RouteDecision", "Passive per-call routing outcome; null packets are probes.",
                        MakeTraceSourceAccessor(&RoutingProtocol::m_routeDecision), "ns3::mo::RouteDecisionCallback")
        .AddTraceSource("ControlEvent", "Passive control stage and protocol-payload byte count.",
                        MakeTraceSourceAccessor(&RoutingProtocol::m_controlEvent), "ns3::mo::ControlEventCallback")
        .AddTraceSource("GraphSnapshot", "Fresh records, heard peers, bidirectional peers, graph edges and reachable remote servers at an existing refresh.",
                        MakeTraceSourceAccessor(&RoutingProtocol::m_graphSnapshot), "ns3::mo::GraphSnapshotCallback")
        .AddTraceSource("SelectedHop", "Passive selected gateway and original state ages in ns.",
                        MakeTraceSourceAccessor(&RoutingProtocol::m_selectedHop), "ns3::mo::SelectedHopCallback")
        .AddTraceSource("RelayQueueEvent", "Local relay queue reason, pending depth, and record waiting ns.",
                        MakeTraceSourceAccessor(&RoutingProtocol::m_relayQueueEvent), "ns3::mo::RelayQueueCallback")
        .AddAttribute("HelloInterval", "Period of flooded topology/CPU/task descriptors.", TimeValue(Seconds(1)),
                      MakeTimeAccessor(&RoutingProtocol::m_interval), MakeTimeChecker(MilliSeconds(50)))
        .AddAttribute("HoldTime", "Maximum ORIGINAL descriptor age.", TimeValue(Seconds(4)),
                      MakeTimeAccessor(&RoutingProtocol::m_hold), MakeTimeChecker(MilliSeconds(100)))
        .AddAttribute("RelayBatchWindow",
                      "Deadline for one batch-v1 relay flush. Larger windows give duplicate "
                      "suppression more time to cancel redundant relays.",
                      TimeValue(MilliSeconds(20)),
                      MakeTimeAccessor(&RoutingProtocol::m_relayBatchWindow),
                      MakeTimeChecker(MilliSeconds(1)))
        .AddAttribute("SuppressDuplicateRelays",
                      "In batch-v1, cancel a pending relay when the same descriptor is "
                      "received from another node before the flush.",
                      BooleanValue(false),
                      MakeBooleanAccessor(&RoutingProtocol::m_suppressDuplicateRelays),
                      MakeBooleanChecker())
        .AddAttribute("ServerHelloScale",
                      "Multiplier on the Hello period for COMMAND/RELAY server nodes. "
                      "Below 1 refreshes server descriptors faster; 1 keeps the "
                      "uniform period.",
                      DoubleValue(1.0),
                      MakeDoubleAccessor(&RoutingProtocol::m_serverHelloScale),
                      MakeDoubleChecker<double>(0.1, 4.0));
    return id;
}
void RoutingProtocol::Configure(uint32_t slot, uint32_t nodes, Vertex descriptor,
    std::shared_ptr<const Policy> policy, const std::string& mode, double seconds, double horizon, double rate, const std::string& relayMode)
{
    if (slot >= nodes || nodes > 4096 || !policy ||
        (relayMode != "legacy" && relayMode != "batch-v1") ||
        (relayMode == "batch-v1" && nodes > RelayBatchHeader::MAX_NODES) || (mode != "mognn" && mode != "mognn-baseline" && mode != "mognn-local") ||
        !std::isfinite(seconds) || seconds <= 0 || !std::isfinite(horizon) || horizon <= 0 ||
        !std::isfinite(rate) || rate <= 0) throw std::runtime_error("Invalid MO routing configuration");
    m_slot = slot; m_nodes = nodes; m_vertex = descriptor; m_policy = std::move(policy); m_mode = mode;
    m_slotSeconds = seconds; m_horizon = horizon; m_linkRate = rate;
    m_batchRelays = relayMode == "batch-v1";
}
void RoutingProtocol::OpenDecisions(const std::string& path)
{
    m_decisions.open(path);
    if (!m_decisions) throw std::runtime_error("Cannot write MO decision log: " + path);
    m_decisions << "time_s,task_id,source_slot,destination_slot,local,known_nodes,links,estimated_s\n";
}
void RoutingProtocol::SetIpv4(Ptr<Ipv4> ip) { m_ipv4 = ip; }
Ipv4Address RoutingProtocol::Address() const { return m_interface ? m_ipv4->GetAddress(m_interface, 0).GetLocal() : Ipv4Address(); }
bool RoutingProtocol::Fresh(Time t) const { return t <= Simulator::Now() && Simulator::Now() - t < m_hold; }
void RoutingProtocol::DoInitialize()
{
    if (!m_policy) throw std::runtime_error("MO routing is not configured");
    m_jitter = CreateObject<UniformRandomVariable>(); m_jitter->SetStream(710000 + m_slot);
    m_helloEvent = Simulator::Schedule(Seconds(.01 + m_jitter->GetValue(0, .05)), &RoutingProtocol::Start, this);
    Ipv4RoutingProtocol::DoInitialize();
}
void RoutingProtocol::Start()
{
    for (uint32_t i = 1; i < m_ipv4->GetNInterfaces(); ++i)
        if (m_ipv4->IsUp(i) && m_ipv4->GetNAddresses(i)) { m_interface = i; break; }
    if (!m_interface) throw std::runtime_error("MO requires an active IPv4 radio");
    if (m_batchRelays && m_ipv4->GetNetDevice(m_interface)->GetMtu() < RelayBatchHeader::MAX_BYTES + 28)
        throw std::runtime_error("MO relay batches require MTU >= 1228");
    m_socket = Socket::CreateSocket(m_node, UdpSocketFactory::GetTypeId());
    m_socket->SetAllowBroadcast(true); m_socket->BindToNetDevice(m_ipv4->GetNetDevice(m_interface));
    if (m_socket->Bind(InetSocketAddress(Ipv4Address::GetAny(), PORT)) < 0) throw std::runtime_error("Cannot bind MO control socket");
    m_socket->SetRecvCallback(MakeCallback(&RoutingProtocol::Receive, this)); Hello();
}
void RoutingProtocol::Broadcast(Ptr<Packet> packet, bool hello)
{
    if (!m_socket || !m_ipv4->IsUp(m_interface))
    { m_controlEvent(packet->GetSize(), hello ? "hello_interface_unavailable" : "relay_interface_unavailable"); return; }
    if (m_socket->SendTo(packet, 0, InetSocketAddress(m_ipv4->GetAddress(m_interface, 0).GetBroadcast(), PORT)) >= 0)
    { ++m_control; m_controlBytes += packet->GetSize() + 28; if (hello) ++m_hellos;
      m_controlEvent(packet->GetSize(), hello ? "hello_socket_accepted" : "relay_socket_accepted"); }
    else m_controlEvent(packet->GetSize(), hello ? "hello_socket_failed" : "relay_socket_failed");
}
void RoutingProtocol::Hello()
{
    for (auto it = m_peers.begin(); it != m_peers.end();)
        if (!Fresh(it->second.stamp))
        { m_closedSeconds += (it->second.stamp + m_hold - it->second.first).GetSeconds(); ++m_closedLinks; it = m_peers.erase(it); }
        else ++it;
    StateHeader h; h.slot = m_slot; h.sequence = ++m_sequence; h.address = Address(); h.vertex = m_vertex;
    h.stamp = Simulator::Now().GetNanoSeconds();
    for (const auto& [id, peer] : m_peers) h.neighbors.push_back(id);
    m_records[m_slot] = {h.sequence, Simulator::Now(), h.address, h.vertex, h.neighbors};
    auto p = Create<Packet>(); p->AddHeader(h); Broadcast(p, true);
    m_helloEvent = Simulator::Schedule(
        m_interval * (m_vertex.server ? m_serverHelloScale : 1.0) * m_jitter->GetValue(.9, 1.1),
        &RoutingProtocol::Hello,
        this);
}
void RoutingProtocol::Receive(Ptr<Socket> socket)
{
    ns3::Address from;
    while (auto p = socket->RecvFrom(from))
    {
        const auto bytes = p->GetSize();
        m_controlEvent(bytes, "rx_datagram");
        uint8_t first = 0;
        if (bytes) p->CopyData(&first, 1);
        const auto sender = InetSocketAddress::ConvertFrom(from).GetIpv4();
        if (first == 0x4d) {
            RelayBatchHeader batch;
            if (!p->RemoveHeader(batch)) { m_controlEvent(bytes, "rx_invalid_batch"); continue; }
            m_controlEvent(bytes, "rx_batch_decoded");
            for (const auto& h : batch.records) {
                m_controlEvent(h.GetSerializedSize(), "batch_records_received");
                ReceiveState(h, sender, h.GetSerializedSize(), true);
            }
        } else {
            StateHeader h;
            if (!p->RemoveHeader(h)) { m_controlEvent(bytes, "rx_invalid_header"); continue; }
            ReceiveState(h, sender, bytes, false);
        }
    }
}
void RoutingProtocol::ReceiveState(StateHeader h, Ipv4Address sender, uint32_t bytes, bool bundled)
{
    // Datagram dispositions remain rx_*; batch record dispositions are record_*.
    auto event = [&](const char* reason) { m_controlEvent(bytes, std::string(bundled ? "record_" : "rx_") + reason); };
    if (h.slot >= m_nodes) { event("invalid_origin"); return; }
    if (h.slot == m_slot) { event("own_origin"); return; }
    if ((h.kind != 1 && h.kind != 2) || h.ttl == 0 || h.ttl > 64)
    { event("invalid_kind_ttl"); return; }
    if (h.stamp > uint64_t(Simulator::Now().GetNanoSeconds()) || !Fresh(NanoSeconds(h.stamp)))
    { event("expired_or_future"); return; }
    if (!std::isfinite(h.vertex.rate) || h.vertex.rate < 0 ||
        !std::isfinite(h.vertex.arrival) || h.vertex.arrival < 0 ||
        std::any_of(h.neighbors.begin(), h.neighbors.end(), [this](auto n) { return n >= m_nodes; }) ||
        (m_batchRelays && h.GetSerializedSize() + 2 + RelayBatchHeader::PREFIX_BYTES > RelayBatchHeader::MAX_BYTES))
    { event("invalid_descriptor"); return; }
    // Only a direct original Hello can establish a heard peer. Bundles decode
    // kind-2 records exclusively, irrespective of their original IP address.
    if (h.kind == 1 && h.address == sender)
    {
        auto it = m_peers.find(h.slot);
        if (it == m_peers.end()) m_peers[h.slot] = {NanoSeconds(h.stamp), Simulator::Now()};
        else if (NanoSeconds(h.stamp) > it->second.stamp) it->second.stamp = NanoSeconds(h.stamp);
    }
    auto old = m_records.find(h.slot);
    if (old != m_records.end() && int32_t(h.sequence - old->second.sequence) <= 0)
    {
        event("duplicate_sequence");
        if (m_batchRelays && m_suppressDuplicateRelays && h.kind == 2 &&
            old->second.sequence == h.sequence && m_relayQueue.Cancel(h.slot, h.sequence))
        {
            m_controlEvent(bytes, "relay_duplicate_cancelled");
        }
        return;
    }
    event("new_descriptor");
    m_records[h.slot] = {h.sequence, NanoSeconds(h.stamp), h.address, h.vertex, h.neighbors};
    if (h.ttl <= 1) return;
    --h.ttl; h.kind = 2;
    // Consume the same per-accepted-record jitter draw in both modes. Batch
    // uses a fixed 20 ms deadline; actual Hello times are not a pairing proof
    // because reception/acceptance histories can differ between modes.
    const auto delay = Seconds(m_jitter->GetValue(.001, .02));
    if (m_batchRelays) {
        auto replaced = m_relayQueue.Put(h, Simulator::Now());
        if (replaced) {
            m_controlEvent(replaced->state.GetSerializedSize(), "relay_version_replaced");
            m_relayQueueEvent("replaced", m_relayQueue.Size(), (Simulator::Now() - replaced->enqueued).GetNanoSeconds());
        } else m_relayQueueEvent("enqueued", m_relayQueue.Size(), 0);
        if (!m_batchEvent.IsRunning())
            m_batchEvent = Simulator::Schedule(m_relayBatchWindow, &RoutingProtocol::FlushRelayBatch, this);
    } else {
        auto relay = Create<Packet>(); relay->AddHeader(h);
        m_events.erase(std::remove_if(m_events.begin(), m_events.end(), [](const auto& e) { return e.IsExpired(); }), m_events.end());
        m_events.push_back(Simulator::Schedule(delay, &RoutingProtocol::Broadcast, this, relay, false));
    }
}
void RoutingProtocol::FlushRelayBatch()
{
    auto flush = m_relayQueue.Drain(Simulator::Now(), m_hold);
    for (const auto& outcome : flush.outcomes) {
        const auto& p = outcome.record;
        m_relayQueueEvent(outcome.expired ? "expired" : "packed", 0,
                          (Simulator::Now() - p.enqueued).GetNanoSeconds());
        m_controlEvent(p.state.GetSerializedSize(), outcome.expired ? "relay_expired_before_send" : "batch_record_packed");
    }
    for (const auto& header : flush.packets) {
        auto packet = Create<Packet>(); packet->AddHeader(header);
        m_controlEvent(packet->GetSize(), "batch_datagram_built");
        Broadcast(packet, false);
    }
}
void RoutingProtocol::ClearRelayBatch(const char* reason)
{
    m_batchEvent.Cancel();
    for (const auto& p : m_relayQueue.Clear()) {
        m_relayQueueEvent(reason, 0, (Simulator::Now() - p.enqueued).GetNanoSeconds());
        m_controlEvent(p.state.GetSerializedSize(), "relay_pending_cancelled");
    }
}
bool RoutingProtocol::Direct(uint32_t slot) const
{
    auto p = m_peers.find(slot);
    if (p == m_peers.end() || !Fresh(p->second.stamp)) return false;
    auto r = m_records.find(slot);
    return r != m_records.end() && Fresh(r->second.stamp) &&
        std::find(r->second.neighbors.begin(), r->second.neighbors.end(), m_slot) != r->second.neighbors.end();
}
void RoutingProtocol::Refresh()
{
    if (Simulator::Now() < m_cacheUntil && !m_view.empty()) return;
    m_view.assign(m_nodes, {});
    for (const auto& [id, r] : m_records) if (Fresh(r.stamp)) m_view[id] = r.vertex;
    m_view[m_slot] = m_vertex;
    std::vector<Link> links;
    for (const auto& [id, r] : m_records) if (Fresh(r.stamp))
        for (auto peer : r.neighbors) if (peer > id)
        {
            auto q = m_records.find(peer);
            if (q != m_records.end() && Fresh(q->second.stamp) &&
                std::find(q->second.neighbors.begin(), q->second.neighbors.end(), id) != q->second.neighbors.end())
                links.push_back({id, peer, m_linkRate});
        }
    m_solution = m_policy->Solve(m_view, links, m_horizon, m_mode == "mognn");
    if (!m_graphSnapshot.IsEmpty())
    {
        uint32_t records = 0, peers = 0, direct = 0, servers = 0;
        for (const auto& [id, r] : m_records) if (Fresh(r.stamp)) ++records;
        for (const auto& [id, p] : m_peers) { if (Fresh(p.stamp)) ++peers; if (Direct(id)) ++direct; }
        for (uint32_t i = 0; i < m_view.size(); ++i)
            if (i != m_slot && m_view[i].server && std::isfinite(m_solution.distance[m_slot][i])) ++servers;
        m_graphSnapshot(records, peers, direct, links.size(), servers);
    }
    m_cacheUntil = Simulator::Now() + MilliSeconds(250);
}
Ipv4Address RoutingProtocol::SelectDestination(uint32_t taskId, double upload, double download)
{
    Refresh();
    const auto dest = Policy::Select(m_solution, m_view, m_slot, upload, download, m_mode == "mognn-local");
    if (m_decisions.is_open())
    {
        size_t known = 0;
        for (const auto& [id, r] : m_records) if (Fresh(r.stamp)) ++known;
        m_decisions << std::setprecision(17) << Simulator::Now().GetSeconds() << ',' << taskId << ',' << m_slot << ','
            << dest << ',' << (dest == m_slot) << ',' << known << ',' << m_solution.weights.size() << ','
            << Policy::Cost(m_solution, m_slot, dest, upload, download) * m_slotSeconds << '\n';
        m_decisions.flush();
    }
    return dest == m_slot ? Address() : m_records.at(dest).address;
}
Ptr<Ipv4Route> RoutingProtocol::Route(Ipv4Address dst, Ipv4Address gateway) const
{
    auto route = Create<Ipv4Route>(); route->SetDestination(dst); route->SetGateway(gateway);
    route->SetSource(Address()); route->SetOutputDevice(m_ipv4->GetNetDevice(m_interface)); return route;
}
Ptr<Ipv4Route> RoutingProtocol::RouteOutput(Ptr<Packet> p, const Ipv4Header& h, Ptr<NetDevice> oif, Socket::SocketErrno& error)
{
    return LookupRoute(p, h, oif, error, false);
}
void RoutingProtocol::ObserveRoute(Ptr<const Packet> p, const Ipv4Header& h, const char* stage, const char* outcome)
{
    if (!m_routeDecision.IsEmpty()) m_routeDecision(p, h, stage, outcome);
}
Ptr<Ipv4Route> RoutingProtocol::LookupRoute(Ptr<Packet> p, const Ipv4Header& h, Ptr<NetDevice> oif,
                                          Socket::SocketErrno& error, bool forwarding)
{
    const char* stage = forwarding ? "route_forward" : "route_output";
    auto reject = [&](const char* reason) -> Ptr<Ipv4Route> { ObserveRoute(p, h, stage, reason); return nullptr; };
    error = Socket::ERROR_NOROUTETOHOST;
    if (!m_interface || !m_ipv4->IsUp(m_interface)) return reject("reject_interface_down");
    if (oif && oif != m_ipv4->GetNetDevice(m_interface)) return reject("reject_output_device");
    const auto dst = h.GetDestination();
    if (dst == Address() || dst.IsBroadcast() || dst == m_ipv4->GetAddress(m_interface, 0).GetBroadcast())
    { error = Socket::ERROR_NOTERROR; ObserveRoute(p, h, stage, dst == Address() ? "accept_local" : "accept_broadcast"); return Route(dst, dst); }
    if (dst.IsMulticast()) return reject("reject_multicast");
    Refresh();
    int target = -1;
    for (const auto& [id, r] : m_records) if (r.address == dst && Fresh(r.stamp)) { target = id; break; }
    if (target < 0) {
        bool expired = false;
        for (const auto& [id, r] : m_records) if (r.address == dst) expired = true;
        return reject(expired ? "reject_target_expired" : "reject_target_unknown");
    }
    const auto next = m_solution.next[m_slot][target];
    if (next < 0) return reject("reject_no_path");
    if (!Direct(next)) {
        auto peer = m_peers.find(next);
        if (peer == m_peers.end()) return reject("reject_next_not_heard");
        if (!Fresh(peer->second.stamp)) return reject("reject_next_heard_expired");
        auto record = m_records.find(next);
        if (record == m_records.end() || !Fresh(record->second.stamp)) return reject("reject_next_descriptor_expired");
        return reject("reject_next_not_reciprocal");
    }
    PathTag tag;
    if (p) p->PeekPacketTag(tag);
    if (tag.path.size() >= 256) return reject("reject_path_limit");
    if (std::find(tag.path.begin(), tag.path.end(), m_slot) != tag.path.end()) return reject("reject_self_revisit");
    if (std::find(tag.path.begin(), tag.path.end(), next) != tag.path.end()) return reject("reject_next_revisit");
    tag.path.push_back(m_slot); if (p) p->ReplacePacketTag(tag);
    if (p && !m_selectedHop.IsEmpty()) {
        std::ostringstream out;
        out << "{\"next_slot\":" << next << ",\"gateway\":\"" << m_records.at(next).address
            << "\",\"destination\":\"" << dst << "\",\"hello_age_ns\":"
            << (Simulator::Now() - m_peers.at(next).stamp).GetNanoSeconds()
            << ",\"descriptor_age_ns\":" << (Simulator::Now() - m_records.at(next).stamp).GetNanoSeconds() << '}';
        m_selectedHop(p, out.str());
    }
    error = Socket::ERROR_NOTERROR; ObserveRoute(p, h, stage, "accept_unicast"); return Route(dst, m_records.at(next).address);
}
bool RoutingProtocol::RouteInput(Ptr<const Packet> p, const Ipv4Header& h, Ptr<const NetDevice> device,
    const UnicastForwardCallback& ucb, const MulticastForwardCallback&, const LocalDeliverCallback& lcb, const ErrorCallback& ecb)
{
    const int iface = m_ipv4->GetInterfaceForDevice(device);
    if (iface < 0) { ObserveRoute(p, h, "route_input", "reject_input_device"); return false; }
    if (m_ipv4->IsDestinationAddress(h.GetDestination(), iface)) { ObserveRoute(p, h, "route_input", "local_delivery"); lcb(p, h, iface); return true; }
    if (h.GetDestination().IsMulticast()) { ObserveRoute(p, h, "route_input", "reject_multicast"); return false; }
    if (!m_ipv4->IsForwarding(iface)) { ObserveRoute(p, h, "route_input", "reject_forwarding_disabled"); ecb(p, h, Socket::ERROR_NOROUTETOHOST); return true; }
    auto copy = p->Copy(); Socket::SocketErrno err;
    auto route = LookupRoute(copy, h, nullptr, err, true);
    if (route) ucb(route, copy, h); else ecb(p, h, err);
    return true;
}
double RoutingProtocol::GetAverageLinkLifetime() const
{
    double seconds = m_closedSeconds; auto count = m_closedLinks;
    for (const auto& [id, p] : m_peers)
    { seconds += (std::min(Simulator::Now(), p.stamp + m_hold) - p.first).GetSeconds(); ++count; }
    return count ? seconds / count : 0;
}
void RoutingProtocol::NotifyInterfaceUp(uint32_t) {}
void RoutingProtocol::NotifyInterfaceDown(uint32_t i)
{
    if (i == m_interface) { ClearRelayBatch("interface_down"); m_helloEvent.Cancel(); if (m_socket) m_socket->Close(); m_peers.clear(); m_records.clear(); m_cacheUntil = Seconds(0); }
}
void RoutingProtocol::NotifyAddAddress(uint32_t, Ipv4InterfaceAddress) {}
void RoutingProtocol::NotifyRemoveAddress(uint32_t i, Ipv4InterfaceAddress) { NotifyInterfaceDown(i); }
void RoutingProtocol::PrintRoutingTable(Ptr<OutputStreamWrapper> out, Time::Unit) const
{
    auto& os = *out->GetStream();
    os << m_mode << ' ' << Address() << " records=" << m_records.size() << " heard=" << m_peers.size() << '\n';
    for (const auto& [id, r] : m_records) {
        os << "  slot=" << id << " ip=" << r.address << " fresh=" << Fresh(r.stamp)
           << " direct=" << Direct(id) << " cpu=" << r.vertex.rate << " server=" << r.vertex.server << " neighbors=";
        for (auto n : r.neighbors) os << n << ',';
        os << '\n';
    }
}
void RoutingProtocol::DoDispose()
{
    ClearRelayBatch("disposed");
    m_helloEvent.Cancel(); for (auto& e : m_events) e.Cancel(); m_events.clear();
    if (m_socket) { m_socket->SetRecvCallback(MakeNullCallback<void, Ptr<Socket>>()); m_socket->Close(); }
    if (m_decisions.is_open()) m_decisions.close();
    m_socket = nullptr; m_ipv4 = nullptr; m_node = nullptr; m_policy.reset(); m_jitter = nullptr;
    m_records.clear(); m_peers.clear(); Ipv4RoutingProtocol::DoDispose();
}
}
