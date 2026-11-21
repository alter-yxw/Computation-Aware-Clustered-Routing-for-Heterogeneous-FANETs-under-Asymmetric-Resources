/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "cfr-routing.h"
#include "ns3/inet-socket-address.h"
#include "ns3/boolean.h"
#include "ns3/ipv4-route.h"
#include "ns3/mobility-model.h"
#include "ns3/node.h"
#include "ns3/output-stream-wrapper.h"
#include "ns3/random-variable-stream.h"
#include "ns3/simulator.h"
#include "ns3/udp-socket-factory.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ns3::cfr
{
NS_OBJECT_ENSURE_REGISTERED(RoutingProtocol);
static constexpr uint16_t HELLO_PORT = 26900;
TypeId RoutingProtocol::GetTypeId()
{
    static TypeId tid = TypeId("ns3::cfr::RoutingProtocol").SetParent<Ipv4RoutingProtocol>()
        .SetGroupName("Cfr").AddConstructor<RoutingProtocol>()
        .AddAttribute("RequireBidirectional", "Require fresh evidence that the peer received our Hello.",
                      BooleanValue(true), MakeBooleanAccessor(&RoutingProtocol::m_requireBidirectional),
                      MakeBooleanChecker())
        .AddAttribute("HelloInterval", "Period of position/CPU advertisements.", TimeValue(Seconds(1)),
                      MakeTimeAccessor(&RoutingProtocol::m_helloInterval), MakeTimeChecker(MilliSeconds(10)))
        .AddAttribute("NeighborHoldTime", "Maximum original-record age, also for two-hop entries.",
                      TimeValue(Seconds(3)), MakeTimeAccessor(&RoutingProtocol::m_holdTime),
                      MakeTimeChecker(MilliSeconds(10)));
    return tid;
}
void RoutingProtocol::Configure(uint16_t slot, double ghz, std::shared_ptr<const Policy> policy,
                                std::function<Vector(Ipv4Address)> location)
{
    m_slot = slot; m_ghz = ghz; m_policy = std::move(policy); m_location = std::move(location);
}
void RoutingProtocol::SetIpv4(Ptr<Ipv4> ip) { m_ipv4 = ip; }
void RoutingProtocol::DoInitialize()
{
    if (!m_policy || !m_location) throw std::runtime_error("CFR routing is not configured");
    // Separate assigned streams keep Hello jitter independent of application task streams.
    auto jitter = CreateObject<UniformRandomVariable>(); jitter->SetStream(700000 + m_slot);
    m_helloEvent = Simulator::Schedule(Seconds(.01 + jitter->GetValue(0, .05)), &RoutingProtocol::Start, this);
    Ipv4RoutingProtocol::DoInitialize();
}
void RoutingProtocol::Start()
{
    for (uint32_t i = 1; i < m_ipv4->GetNInterfaces(); ++i)
        if (m_ipv4->IsUp(i) && m_ipv4->GetNAddresses(i)) { m_interface = i; break; }
    if (!m_interface) throw std::runtime_error("CFR requires one active IPv4 radio interface");
    m_socket = Socket::CreateSocket(m_node, UdpSocketFactory::GetTypeId());
    m_socket->SetAllowBroadcast(true); m_socket->BindToNetDevice(m_ipv4->GetNetDevice(m_interface));
    if (m_socket->Bind(InetSocketAddress(Ipv4Address::GetAny(), HELLO_PORT)) < 0)
        throw std::runtime_error("Cannot bind CFR Hello socket");
    m_socket->SetRecvCallback(MakeCallback(&RoutingProtocol::Receive, this));
    SendHello();
}
Ipv4Address RoutingProtocol::Address() const
{
    return m_interface ? m_ipv4->GetAddress(m_interface, 0).GetLocal() : Ipv4Address();
}
Vector RoutingProtocol::Position() const
{
    return m_node->GetObject<MobilityModel>()->GetPosition();
}
bool RoutingProtocol::Fresh(const NeighborRecord& r) const
{
    auto now = static_cast<uint64_t>(Simulator::Now().GetNanoSeconds());
    return r.stampNs <= now && now - r.stampNs < static_cast<uint64_t>(m_holdTime.GetNanoSeconds());
}
bool RoutingProtocol::Usable(const Peer& p) const
{
    if (!Fresh(p.record)) return false;
    if (!m_requireBidirectional) return true;
    // Receiving a strong peer's broadcast proves only peer -> self. A fresh
    // echo of our own record proves self -> peer without any global position peek.
    return std::any_of(p.advertised.begin(), p.advertised.end(), [this](const NeighborRecord& r) {
        return r.slot == m_slot && r.address == Address() && Fresh(r);
    });
}
void RoutingProtocol::Purge()
{
    for (auto it = m_peers.begin(); it != m_peers.end();)
        if (!Fresh(it->second.record))
        {
            m_closedLinkSeconds += (NanoSeconds(it->second.record.stampNs) + m_holdTime - it->second.firstSeen).GetSeconds();
            ++m_closedLinks; it = m_peers.erase(it);
        }
        else ++it;
}
void RoutingProtocol::SendHello()
{
    Purge();
    HelloHeader h;
    h.records.push_back({m_slot, Address(), Position(), m_ghz, ++m_sequence,
                         static_cast<uint64_t>(Simulator::Now().GetNanoSeconds())});
    for (const auto& [id, peer] : m_peers) h.records.push_back(peer.record);
    auto packet = Create<Packet>(); packet->AddHeader(h);
    auto broadcast = m_ipv4->GetAddress(m_interface, 0).GetBroadcast();
    if (m_socket->SendTo(packet, 0, InetSocketAddress(broadcast, HELLO_PORT)) >= 0)
    {
        ++m_helloCount; m_controlBytes += packet->GetSize() + 28; // IPv4 + UDP, excluding L2
    }
    m_helloEvent = Simulator::Schedule(m_helloInterval, &RoutingProtocol::SendHello, this);
}
void RoutingProtocol::Receive(Ptr<Socket> socket)
{
    ns3::Address from;
    while (auto packet = socket->RecvFrom(from))
    {
        HelloHeader h;
        if (!packet->RemoveHeader(h) || h.records.empty()) continue;
        auto sender = h.records.front();
        if (sender.address != InetSocketAddress::ConvertFrom(from).GetIpv4() ||
            sender.slot >= m_policy->Nodes() || sender.slot == m_slot || !Fresh(sender) ||
            !std::isfinite(sender.ghz) || sender.ghz < 0) continue;
        auto it = m_peers.find(sender.slot);
        if (it != m_peers.end() && static_cast<int32_t>(sender.sequence - it->second.record.sequence) <= 0)
            continue; // duplicates and stale sequences do not renew leases
        auto& peer = m_peers[sender.slot];
        if (it == m_peers.end()) peer.firstSeen = Simulator::Now();
        peer.record = sender; peer.advertised.clear();
        for (size_t i = 1; i < h.records.size(); ++i)
            if (h.records[i].slot < m_policy->Nodes() && Fresh(h.records[i]))
                peer.advertised.push_back(h.records[i]);
    }
}
std::vector<double> RoutingProtocol::State(Ipv4Address destination, const TaskState& task) const
{
    std::map<uint16_t, Observation> observations;
    observations[m_slot] = {m_slot, Position(), m_ghz, 0};
    std::map<uint16_t, uint64_t> stamps;
    for (const auto& [id, p] : m_peers)
        if (Fresh(p.record))
            for (const auto& r : p.advertised)
                if (r.slot != m_slot && Fresh(r) && r.stampNs >= stamps[r.slot])
                {
                    observations[r.slot] = {r.slot, r.position, r.ghz, 2}; stamps[r.slot] = r.stampNs;
                }
    for (const auto& [id, p] : m_peers)
        if (Fresh(p.record)) observations[id] = {id, p.record.position, p.record.ghz, 1};
    std::vector<Observation> view;
    for (const auto& [id, o] : observations) view.push_back(o);
    return m_policy->Encode(view, Position(), m_location(destination), m_ghz, task);
}
std::vector<bool> RoutingProtocol::Mask(const std::vector<uint16_t>& visited) const
{
    std::vector<bool> valid(m_policy->Nodes(), false);
    for (const auto& [id, p] : m_peers) if (Usable(p)) valid[id] = true;
    valid[m_slot] = false;
    for (auto id : visited) if (id < valid.size()) valid[id] = false;
    return valid;
}
Action RoutingProtocol::Decide(Ipv4Address destination, const TaskState& task,
                               const std::vector<uint16_t>& visited) const
{
    int direct = -1;
    for (const auto& [id, p] : m_peers)
        if (Usable(p) && p.record.address == destination) direct = id;
    auto action = m_policy->Select(State(destination, task), Mask(visited), direct);
    if (m_ghz <= 0 || task.remainingCycles <= 0) action.fraction = 0;
    return action;
}
Ipv4Address RoutingProtocol::NeighborAddress(uint16_t slot) const
{
    auto it = m_peers.find(slot);
    return it != m_peers.end() && Usable(it->second) ? it->second.record.address : Ipv4Address();
}
bool RoutingProtocol::IsDirect(Ipv4Address address) const
{
    for (const auto& [id, p] : m_peers) if (Usable(p) && p.record.address == address) return true;
    return false;
}
size_t RoutingProtocol::OneHopCount() const
{
    size_t count = 0;
    for (const auto& [id, p] : m_peers) if (Fresh(p.record)) ++count;
    return count;
}
size_t RoutingProtocol::TwoHopCount() const
{
    std::vector<bool> seen(m_policy->Nodes(), false);
    for (const auto& [id, p] : m_peers) if (Fresh(p.record))
        for (const auto& r : p.advertised) if (Fresh(r)) seen[r.slot] = true;
    seen[m_slot] = false;
    for (const auto& [id, p] : m_peers) if (Fresh(p.record)) seen[id] = false;
    return std::count(seen.begin(), seen.end(), true);
}
double RoutingProtocol::GetAverageLinkLifetime() const
{
    double seconds = m_closedLinkSeconds; uint64_t count = m_closedLinks;
    for (const auto& [id, p] : m_peers)
    {
        seconds += (std::min(Simulator::Now(), NanoSeconds(p.record.stampNs) + m_holdTime) - p.firstSeen).GetSeconds();
        ++count;
    }
    return count ? seconds / count : 0;
}
Ptr<Ipv4Route> RoutingProtocol::Route(Ipv4Address destination, Ipv4Address gateway) const
{
    auto route = Create<Ipv4Route>(); route->SetDestination(destination); route->SetGateway(gateway);
    route->SetSource(Address()); route->SetOutputDevice(m_ipv4->GetNetDevice(m_interface)); return route;
}
Ptr<Ipv4Route> RoutingProtocol::RouteOutput(Ptr<Packet> p, const Ipv4Header& h,
                                          Ptr<NetDevice> oif, Socket::SocketErrno& err)
{
    err = Socket::ERROR_NOROUTETOHOST;
    if (!m_interface || !m_ipv4->IsUp(m_interface) || (oif && oif != m_ipv4->GetNetDevice(m_interface))) return nullptr;
    auto dst = h.GetDestination();
    if (dst == Address() || dst.IsBroadcast() || dst == m_ipv4->GetAddress(m_interface, 0).GetBroadcast() || IsDirect(dst))
    {
        err = Socket::ERROR_NOTERROR; return Route(dst, dst);
    }
    if (dst.IsMulticast()) return nullptr;
    PathTag tag;
    if (p) p->PeekPacketTag(tag);
    if (std::find(tag.path.begin(), tag.path.end(), m_slot) != tag.path.end()) return nullptr;
    tag.path.push_back(m_slot);
    auto action = Decide(dst, {}, tag.path);
    if (action.next < 0) return nullptr;
    if (p) p->ReplacePacketTag(tag);
    err = Socket::ERROR_NOTERROR; return Route(dst, NeighborAddress(action.next));
}
bool RoutingProtocol::RouteInput(Ptr<const Packet> p, const Ipv4Header& h, Ptr<const NetDevice> idev,
                                 const UnicastForwardCallback& ucb, const MulticastForwardCallback&,
                                 const LocalDeliverCallback& lcb, const ErrorCallback& ecb)
{
    auto iface = m_ipv4->GetInterfaceForDevice(idev);
    if (iface < 0) return false;
    if (m_ipv4->IsDestinationAddress(h.GetDestination(), iface)) { lcb(p, h, iface); return true; }
    if (h.GetDestination().IsMulticast()) return false;
    if (!m_ipv4->IsForwarding(iface)) { ecb(p, h, Socket::ERROR_NOROUTETOHOST); return true; }
    auto copy = p->Copy(); Socket::SocketErrno err;
    auto route = RouteOutput(copy, h, nullptr, err);
    if (route) ucb(route, copy, h); else ecb(p, h, err);
    return true;
}
void RoutingProtocol::NotifyInterfaceUp(uint32_t) {}
void RoutingProtocol::NotifyInterfaceDown(uint32_t i)
{
    if (i == m_interface) { m_helloEvent.Cancel(); if (m_socket) m_socket->Close(); m_peers.clear(); }
}
void RoutingProtocol::NotifyAddAddress(uint32_t, Ipv4InterfaceAddress) {}
void RoutingProtocol::NotifyRemoveAddress(uint32_t i, Ipv4InterfaceAddress) { NotifyInterfaceDown(i); }
void RoutingProtocol::PrintRoutingTable(Ptr<OutputStreamWrapper> out, Time::Unit) const
{
    *out->GetStream() << "CFR " << Address() << " oneHop=" << OneHopCount() << " twoHop=" << TwoHopCount() << '\n';
}
void RoutingProtocol::DoDispose()
{
    m_helloEvent.Cancel();
    if (m_socket) { m_socket->SetRecvCallback(MakeNullCallback<void, Ptr<Socket>>()); m_socket->Close(); }
    m_socket = nullptr; m_ipv4 = nullptr; m_node = nullptr; m_policy.reset(); m_location = {}; m_peers.clear();
    Ipv4RoutingProtocol::DoDispose();
}
}
