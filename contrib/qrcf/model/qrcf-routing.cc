/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "qrcf-routing.h"
#include "ns3/energy-source-container.h"
#include "ns3/energy-source.h"
#include "ns3/inet-socket-address.h"
#include "ns3/integer.h"
#include "ns3/ipv4-route.h"
#include "ns3/ipv4-interface.h"
#include "ns3/log.h"
#include "ns3/mobility-model.h"
#include "ns3/simulator.h"
#include "ns3/tag.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/uinteger.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-mpdu.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace ns3::qrcf
{
NS_LOG_COMPONENT_DEFINE("QrcfRoutingProtocol");
NS_OBJECT_ENSURE_REGISTERED(RoutingProtocol);
namespace
{
constexpr uint16_t PORT = 17930;
class ControlTag : public Tag
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::qrcf::ControlTag").SetParent<Tag>().AddConstructor<ControlTag>();
        return tid;
    }
    TypeId GetInstanceTypeId() const override { return GetTypeId(); }
    uint32_t GetSerializedSize() const override { return 1; }
    void Serialize(TagBuffer b) const override { b.WriteU8(1); }
    void Deserialize(TagBuffer b) override { b.ReadU8(); }
    void Print(std::ostream& os) const override { os << "QRCF control"; }
};
// User-approved simulation metadata representing the link sender and a unique
// per-hop transmission token. The token prevents a looping packet's old
// feedback from being applied to a later transmission of the same packet.
// Contains no energy, Q, gamma, or remote topology information.
class AgentTag : public Tag
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::qrcf::AgentTag").SetParent<Tag>().AddConstructor<AgentTag>();
        return tid;
    }
    TypeId GetInstanceTypeId() const override { return GetTypeId(); }
    uint32_t GetSerializedSize() const override { return 12; }
    void Serialize(TagBuffer b) const override { b.WriteU32(previous.Get()); b.WriteU64(dataId); }
    void Deserialize(TagBuffer b) override { previous = Ipv4Address(b.ReadU32()); dataId = b.ReadU64(); }
    void Print(std::ostream& os) const override { os << "QRCF previous=" << previous << " data=" << dataId; }
    Ipv4Address previous;
    uint64_t dataId = 0;
};
}
TypeId RoutingProtocol::GetTypeId()
{
    static TypeId tid = TypeId("ns3::qrcf::RoutingProtocol").SetParent<Ipv4RoutingProtocol>()
        .SetGroupName("Qrcf").AddConstructor<RoutingProtocol>();
    return tid;
}
RoutingProtocol::RoutingProtocol() : m_random(CreateObject<UniformRandomVariable>()) {}
void RoutingProtocol::Configure(Ptr<Node> node, Settings settings)
{
    settings.Validate(); m_node = node; m_settings = settings;
}
int64_t RoutingProtocol::AssignStreams(int64_t stream)
{
    m_random->SetStream(stream);
    if (m_settings.helloJitter)
    {
        // Construct with the explicit stream attribute, avoiding even a
        // temporary automatic RNG allocation that could shift ARP's streams.
        m_helloRandom = CreateObjectWithAttributes<UniformRandomVariable>(
            "Stream",IntegerValue(stream+1000000));
    }
    return 1; // Preserve the historical epsilon/action stream of every node.
}
void RoutingProtocol::SetIpv4(Ptr<Ipv4> ipv4) { m_ipv4 = ipv4; }
void RoutingProtocol::DoInitialize()
{
    NS_ABORT_MSG_IF(!m_node || !m_ipv4 || !m_node->GetObject<MobilityModel>(),
                    "QRCF needs QrcfHelper and a mobility model");
    m_discovery = std::make_unique<Discovery>(m_settings);
    m_epsilon = m_random->GetValue(0,1);
    if (m_settings.helloJitter && !m_helloRandom)
    { m_helloRandom = CreateObject<UniformRandomVariable>(); }
    m_l3 = DynamicCast<Ipv4L3Protocol>(m_ipv4);
    NS_ABORT_MSG_IF(!m_l3,"QRCF requires the ns-3 IPv4 L3 protocol");
    m_l3->TraceConnectWithoutContext("Tx",MakeCallback(&RoutingProtocol::OnIpv4Tx,this));
    if (m_settings.outcomeUpdates)
    {
        m_l3->TraceConnectWithoutContext("LocalDeliver",MakeCallback(&RoutingProtocol::OnLocalDeliver,this));
        m_l3->TraceConnectWithoutContext("Drop",MakeCallback(&RoutingProtocol::OnIpv4Drop,this));
        m_arp = m_node->GetObject<ArpL3Protocol>();
        if (m_arp) { m_arp->TraceConnectWithoutContext("Drop",MakeCallback(&RoutingProtocol::OnArpDrop,this)); }
        if (m_interface >= 0) { m_arpCache = m_l3->GetInterface(m_interface)->GetArpCache(); }
        if (m_arpCache)
        { m_arpCache->TraceConnectWithoutContext("Drop",MakeCallback(&RoutingProtocol::OnArpResolutionDrop,this)); }
    }
    if (m_interface >= 0)
    {
        auto wifi = DynamicCast<WifiNetDevice>(m_ipv4->GetNetDevice(m_interface));
        if (wifi)
        {
            m_mac = wifi->GetMac();
            m_mac->TraceConnectWithoutContext("DroppedMpdu",MakeCallback(&RoutingProtocol::OnMacDrop,this));
        }
    }
    m_helloEvent = m_settings.helloJitter ?
        Simulator::Schedule(Seconds(m_helloRandom->GetValue(0,m_discovery->Interval())),&RoutingProtocol::Hello,this) :
        Simulator::ScheduleNow(&RoutingProtocol::Hello,this);
    Ipv4RoutingProtocol::DoInitialize();
}
void RoutingProtocol::DoDispose()
{
    m_helloEvent.Cancel(); m_expiryEvent.Cancel(); m_pendingEvent.Cancel();
    if (m_l3) { m_l3->TraceDisconnectWithoutContext("Tx",MakeCallback(&RoutingProtocol::OnIpv4Tx,this)); }
    if (m_l3 && m_settings.outcomeUpdates)
    {
        m_l3->TraceDisconnectWithoutContext("LocalDeliver",MakeCallback(&RoutingProtocol::OnLocalDeliver,this));
        m_l3->TraceDisconnectWithoutContext("Drop",MakeCallback(&RoutingProtocol::OnIpv4Drop,this));
    }
    if (m_arp) { m_arp->TraceDisconnectWithoutContext("Drop",MakeCallback(&RoutingProtocol::OnArpDrop,this)); }
    if (m_arpCache)
    { m_arpCache->TraceDisconnectWithoutContext("Drop",MakeCallback(&RoutingProtocol::OnArpResolutionDrop,this)); }
    if (m_mac) { m_mac->TraceDisconnectWithoutContext("DroppedMpdu",MakeCallback(&RoutingProtocol::OnMacDrop,this)); }
    if (m_socket) { m_socket->SetRecvCallback(MakeNullCallback<void,Ptr<Socket>>()); m_socket->Close(); }
    m_socket = nullptr; m_mac = nullptr; m_l3 = nullptr; m_ipv4 = nullptr; m_node = nullptr; m_random = nullptr;
    m_helloRandom = nullptr; m_arp = nullptr; m_arpCache = nullptr; m_unavailableNeighbors.clear();
    m_destinationLookup = {}; m_neighbors.Clear(); m_discovery.reset();
    m_pending.clear();
    Ipv4RoutingProtocol::DoDispose();
}
void RoutingProtocol::NotifyInterfaceUp(uint32_t interface)
{
    if (!m_ipv4->GetNAddresses(interface)) { return; }
    const auto address = m_ipv4->GetAddress(interface,0);
    if (address.GetLocal() == Ipv4Address::GetLoopback()) { return; }
    NS_ABORT_MSG_IF(m_interface >= 0 && m_interface != static_cast<int32_t>(interface),
                    "QRCF currently supports one data interface per UAV");
    if (m_socket) { return; }
    NS_ABORT_MSG_IF(!m_node,"QrcfHelper must bind the owning node");
    m_interface = interface; m_address = address;
    m_socket = Socket::CreateSocket(m_node,UdpSocketFactory::GetTypeId());
    m_socket->BindToNetDevice(m_ipv4->GetNetDevice(interface));
    NS_ABORT_MSG_IF(m_socket->Bind(InetSocketAddress(Ipv4Address::GetAny(),PORT)) < 0,
                    "Cannot bind QRCF control socket");
    m_socket->SetAllowBroadcast(true); m_socket->SetIpTtl(1);
    m_socket->SetRecvCallback(MakeCallback(&RoutingProtocol::Receive,this));
}
void RoutingProtocol::NotifyInterfaceDown(uint32_t interface)
{
    if (static_cast<int32_t>(interface) != m_interface) { return; }
    if (m_socket) { m_socket->Close(); m_socket = nullptr; }
    m_expiryEvent.Cancel(); m_neighbors.Clear(); m_previousDistances.clear();
    m_unavailableNeighbors.clear();
    m_diagnostics.ObserveNeighbors({},Simulator::Now()); m_interface = -1;
}
void RoutingProtocol::NotifyAddAddress(uint32_t i,Ipv4InterfaceAddress)
{ if (m_ipv4->IsUp(i)) { NotifyInterfaceUp(i); } }
void RoutingProtocol::NotifyRemoveAddress(uint32_t i,Ipv4InterfaceAddress address)
{ if (address.GetLocal() == Address()) { NotifyInterfaceDown(i); } }
void RoutingProtocol::Purge()
{
    m_neighbors.Purge(Simulator::Now(),true);
    m_diagnostics.ObserveNeighbors(m_neighbors.NodeIds(),Simulator::Now());
    for (auto it = m_previousDistances.begin(); it != m_previousDistances.end();)
    {
        if (!m_neighbors.Entries().count(it->first)) { it = m_previousDistances.erase(it); }
        else { ++it; }
    }
}
void RoutingProtocol::ScheduleExpiry()
{
    m_expiryEvent.Cancel();
    std::optional<Time> earliest;
    for (const auto& [address,entry] : m_neighbors.Entries())
    { if (!earliest || entry.expires < *earliest) { earliest = entry.expires; } }
    if (earliest)
    {
        m_expiryEvent = Simulator::Schedule(*earliest-Simulator::Now(),[this] {
            Purge(); ScheduleExpiry();
        });
    }
}
void RoutingProtocol::Hello()
{
    if (!m_socket || m_interface < 0) { return; }
    Purge();
    const auto mobility = m_node->GetObject<MobilityModel>();
    const Vector position = mobility->GetPosition();
    const Motion motion = Settings::FromVelocity(mobility->GetVelocity());
    const auto sources = m_node->GetObject<EnergySourceContainer>();
    NS_ABORT_MSG_IF(!sources || sources->GetN() == 0,"QRCF Hello requires an energy source");
    const double energy = sources->Get(0)->GetRemainingEnergy();
    if (energy <= 0) { return; }
    NS_ABORT_MSG_IF(m_sequence == std::numeric_limits<uint32_t>::max(),"QRCF Hello sequence exhausted");
    const uint64_t id = (uint64_t{m_node->GetId()} << 32) | m_sequence++;
    // Preserve the advertised current HT, then compute the next HT and the
    // current neighbor-stability sample without advancing simulation time.
    const auto header = m_discovery->MakeHello(m_node->GetId(),id,position,motion,energy);
    std::vector<Scalar> connections;
    for (const auto& [address,entry] : m_neighbors.Entries())
    {
        const auto& peer = entry.hello;
        const double distance = CalculateDistance(position,peer.position);
        const auto old = m_previousDistances.find(address);
        const Scalar previous = old == m_previousDistances.end() ? Scalar{} : Scalar{old->second};
        if (distance <= m_settings.range)
        { connections.push_back(m_settings.Connection(position,motion,peer.position,peer.motion,previous)); }
        else { ++m_diagnostics.outOfRangeCtExclusions; }
        m_previousDistances[address] = distance;
    }
    const bool defined = m_discovery->AfterHello(position,motion,Simulator::Now(),connections,m_neighbors.NodeIds());
    NS_ABORT_MSG_IF(!defined,"QRCF encountered an unresolved in-range Hello formula domain");
    LearningHeader learning;
    learning.gamma = m_discovery->Gamma();
    MessageHeader envelope;
    auto packet = Create<Packet>(); packet->AddHeader(learning); packet->AddHeader(header); packet->AddHeader(envelope);
    packet->AddPacketTag(ControlTag());
    const uint32_t payloadBytes = packet->GetSize();
    const int sent = m_socket->SendTo(packet,0,InetSocketAddress(m_address.GetBroadcast(),PORT));
    if (sent >= 0)
    {
        ++m_diagnostics.helloTx; ++m_diagnostics.controlTx;
        m_controlPayloadBytes += payloadBytes;
        m_diagnostics.ObserveHelloInterval(Seconds(header.helloInterval));
    }
    double delay = m_discovery->Interval();
    if (m_settings.helloJitter)
    { delay = m_helloRandom->GetValue(std::max(m_settings.helloLower,0.9*delay),delay); }
    m_helloEvent = Simulator::Schedule(Seconds(delay),&RoutingProtocol::Hello,this);
    ScheduleExpiry();
}
void RoutingProtocol::Receive(Ptr<Socket> socket)
{
    ns3::Address from;
    while (auto packet = socket->RecvFrom(from))
    {
        MessageHeader envelope;
        if (!InetSocketAddress::IsMatchingType(from) || packet->RemoveHeader(envelope) == 0)
        { ++m_diagnostics.invalidControl; continue; }
        const Ipv4Address sender = InetSocketAddress::ConvertFrom(from).GetIpv4();
        if (sender == Address()) { continue; }
        if (envelope.kind == MessageHeader::Kind::HELLO)
        {
            HelloHeader hello; LearningHeader learning;
            if (packet->RemoveHeader(hello) == 0 || packet->RemoveHeader(learning) == 0 || packet->GetSize() != 0)
            { ++m_diagnostics.invalidControl; continue; }
            ++m_diagnostics.helloRx;
            Purge();
            if (!m_neighbors.Receive(sender,hello,Simulator::Now(),Seconds(m_settings.helloUpper)))
            { ++m_diagnostics.duplicateHello; }
            else
            {
                m_unavailableNeighbors.erase(sender);
                // A newly heard peer is evidence to retry address resolution.
                // ns-3 otherwise retains a failed ARP entry for 100 seconds.
                // Remove only that peer's DEAD entry; never invent its MAC.
                if (m_settings.outcomeUpdates && m_arpCache)
                {
                    auto entry = m_arpCache->Lookup(sender);
                    if (entry && entry->IsDead()) { m_arpCache->Remove(entry); }
                }
            }
            m_diagnostics.ObserveNeighbors(m_neighbors.NodeIds(),Simulator::Now());
            ScheduleExpiry();
        }
        else if (envelope.kind == MessageHeader::Kind::FEEDBACK)
        {
            FeedbackHeader feedback;
            if (packet->RemoveHeader(feedback) == 0 || packet->GetSize() != 0)
            { ++m_diagnostics.invalidControl; continue; }
            ++m_diagnostics.feedbackRx;
            Penalize(feedback.dataId,feedback.destination,sender);
        }
        else if (envelope.kind == MessageHeader::Kind::VALUE)
        {
            ValueHeader value;
            if (packet->RemoveHeader(value) == 0 || packet->GetSize() != 0)
            { ++m_diagnostics.invalidControl; continue; }
            if (m_settings.outcomeUpdates)
            { ++m_diagnostics.valueRx; AcceptValue(value,sender); }
        }
        else { ++m_diagnostics.invalidControl; }
    }
}
void RoutingProtocol::SendFeedback(uint64_t dataId,Ipv4Address destination,Ipv4Address previous)
{
    if (!m_socket || previous == Ipv4Address::GetAny() || previous == Address()) { return; }
    FeedbackHeader feedback; feedback.dataId = dataId; feedback.destination = destination;
    MessageHeader envelope; envelope.kind = MessageHeader::Kind::FEEDBACK;
    auto packet = Create<Packet>(); packet->AddHeader(feedback); packet->AddHeader(envelope);
    packet->AddPacketTag(ControlTag());
    const uint32_t payloadBytes = packet->GetSize();
    const int sent = m_socket->SendTo(packet,0,InetSocketAddress(previous,PORT));
    if (sent >= 0)
    {
        ++m_diagnostics.controlTx; ++m_diagnostics.feedbackTx;
        m_controlPayloadBytes += payloadBytes;
    }
}
void RoutingProtocol::SendValue(uint64_t dataId,Ipv4Address destination,Ipv4Address previous,
                                double maximumQ,bool terminal)
{
    if (!m_socket || previous == Ipv4Address::GetAny() || previous == Address()) { return; }
    ValueHeader value;
    value.dataId = dataId; value.destination = destination;
    value.gamma = terminal ? 0 : Gamma(); value.maximumQ = terminal ? 0 : maximumQ;
    value.status = terminal ? ValueHeader::Status::TERMINAL : ValueHeader::Status::TRANSIT;
    MessageHeader envelope; envelope.kind = MessageHeader::Kind::VALUE;
    auto packet = Create<Packet>(); packet->AddHeader(value); packet->AddHeader(envelope);
    packet->AddPacketTag(ControlTag());
    const uint32_t payloadBytes = packet->GetSize();
    if (m_socket->SendTo(packet,0,InetSocketAddress(previous,PORT)) >= 0)
    {
        ++m_diagnostics.controlTx; ++m_diagnostics.valueTx;
        m_controlPayloadBytes += payloadBytes;
    }
}
void RoutingProtocol::AcceptValue(const ValueHeader& value,Ipv4Address sender)
{
    const auto action = m_pending.find(value.dataId);
    if (action == m_pending.end() || action->second.destination != value.destination ||
        action->second.next != sender) { return; }
    // Destination receipts can only terminate a transition into that destination.
    if (value.IsTerminal() != (sender == value.destination))
    { ++m_diagnostics.invalidControl; return; }
    const auto pending = action->second;
    m_pending.erase(action);
    const double gamma = m_settings.bootstrap && !value.IsTerminal() ? value.gamma : 0;
    UpdateQ(value.destination,sender,value.IsTerminal() ? m_settings.rewardMax : pending.reward,
            pending.alpha,gamma,value.maximumQ);
    if (m_settings.bootstrap && !value.IsTerminal()) { ++m_diagnostics.bootstrapUpdates; }
}
void RoutingProtocol::ExpirePending()
{
    const auto now = Simulator::Now();
    std::optional<Time> next;
    for (auto it = m_pending.begin(); it != m_pending.end();)
    {
        if (it->second.expires <= now)
        {
            // A lost receipt does not establish that the data transmission failed.
            ++m_diagnostics.outcomeTimeouts;
            it = m_pending.erase(it);
        }
        else { if (!next || it->second.expires < *next) { next = it->second.expires; } ++it; }
    }
    if (next) { m_pendingEvent = Simulator::Schedule(*next-now,&RoutingProtocol::ExpirePending,this); }
}
void RoutingProtocol::UpdateQ(Ipv4Address destination,Ipv4Address next,double reward,double alpha,
                              double gamma,double maximumQ)
{
    m_qTables[destination].Update(next.Get(),reward,alpha,gamma,maximumQ);
    ++m_diagnostics.qUpdates;
}
void RoutingProtocol::Penalize(uint64_t dataId,Ipv4Address destination,Ipv4Address next)
{
    const auto action = m_pending.find(dataId);
    if (action == m_pending.end() || action->second.penalized ||
        action->second.destination != destination || action->second.next != next) { return; }
    const double alpha = action->second.alpha;
    if (m_settings.outcomeUpdates) { m_pending.erase(action); }
    else { action->second.penalized = true; }
    UpdateQ(destination,next,m_settings.rewardMin,alpha);
}
void RoutingProtocol::OnMacDrop(WifiMacDropReason reason,Ptr<const WifiMpdu> mpdu)
{
    if (m_settings.outcomeUpdates)
    {
        OnDataDrop(mpdu->GetPacket(),reason == WIFI_MAC_DROP_REACHED_RETRY_LIMIT,false);
        return;
    }
    if (reason != WIFI_MAC_DROP_REACHED_RETRY_LIMIT) { return; }
    AgentTag tag; ControlTag control;
    const auto packet = mpdu->GetPacket();
    if (packet->PeekPacketTag(control) || !packet->PeekPacketTag(tag) || tag.previous != Address()) { return; }
    const auto action = m_pending.find(tag.dataId);
    if (action == m_pending.end() || action->second.penalized) { return; }
    ++m_diagnostics.linkFailures;
    Penalize(tag.dataId,action->second.destination,action->second.next);
}
void RoutingProtocol::OnDataDrop(Ptr<const Packet> packet,bool unavailable,bool arp)
{
    AgentTag tag; ControlTag control;
    if (packet->PeekPacketTag(control) || !packet->PeekPacketTag(tag) || tag.previous != Address()) { return; }
    const auto action = m_pending.find(tag.dataId);
    if (action == m_pending.end()) { return; }
    const auto destination = action->second.destination;
    const auto next = action->second.next;
    if (unavailable) { m_unavailableNeighbors.insert(next); ++m_diagnostics.linkFailures; }
    if (arp) { ++m_diagnostics.arpFailures; }
    Penalize(tag.dataId,destination,next);
}
void RoutingProtocol::OnArpDrop(Ptr<const Packet> packet) { OnDataDrop(packet,false,true); }
void RoutingProtocol::OnArpResolutionDrop(Ptr<const Packet> packet) { OnDataDrop(packet,true,true); }
void RoutingProtocol::OnIpv4Drop(const Ipv4Header&,Ptr<const Packet> packet,
                                Ipv4L3Protocol::DropReason,Ptr<Ipv4>,uint32_t)
{ OnDataDrop(packet,false,false); }
void RoutingProtocol::OnLocalDeliver(const Ipv4Header& header,Ptr<const Packet> packet,uint32_t)
{
    AgentTag tag; ControlTag control;
    if (header.GetDestination() != Address() || packet->PeekPacketTag(control) ||
        !packet->PeekPacketTag(tag)) { return; }
    SendValue(tag.dataId,header.GetDestination(),tag.previous,0,true);
}
void RoutingProtocol::OnIpv4Tx(Ptr<const Packet> packet,Ptr<Ipv4>,uint32_t)
{
    ControlTag control;
    if (packet->PeekPacketTag(control))
    {
        ++m_diagnostics.controlL3Packets;
        m_diagnostics.controlBytes += packet->GetSize();
    }
}
Ptr<Ipv4Route> RoutingProtocol::MakeRoute(Ipv4Address destination,Ipv4Address gateway) const
{
    if (m_interface < 0) { return nullptr; }
    auto route = Create<Ipv4Route>();
    route->SetDestination(destination); route->SetSource(Address()); route->SetGateway(gateway);
    route->SetOutputDevice(m_ipv4->GetNetDevice(m_interface));
    return route;
}
Ptr<Ipv4Route> RoutingProtocol::Lookup(Ptr<Packet> packet,const Ipv4Header& header,bool source)
{
    const auto destination = header.GetDestination();
    if (destination == Address() || destination == Ipv4Address::GetLoopback())
    {
        // Ordinary local IP delivery (including ICMP errors addressed back
        // to this UAV) is not a zero-length QRCF routing problem.
        auto route = Create<Ipv4Route>();
        route->SetDestination(destination); route->SetSource(destination);
        route->SetGateway(Ipv4Address::GetLoopback());
        route->SetOutputDevice(m_ipv4->GetNetDevice(0));
        return route;
    }
    if (destination.IsBroadcast() || destination == m_address.GetBroadcast())
    { return MakeRoute(destination,Ipv4Address::GetAny()); }
    ControlTag control;
    if (packet && packet->PeekPacketTag(control)) { return MakeRoute(destination,destination); }
    if (!DataRoutingReady()) { return nullptr; }
    if (packet) { ++m_diagnostics.routeRequests; }
    Purge();
    const auto target = m_destinationLookup(destination);
    if (!target) { if (packet) { ++m_diagnostics.noRoute; } return nullptr; }
    const auto mobility = m_node->GetObject<MobilityModel>();
    const auto plan = RoutePlanner::Prepare(m_settings,mobility->GetPosition(),
        Settings::FromVelocity(mobility->GetVelocity()),destination,*target,m_neighbors,m_previousDistances,
        m_unavailableNeighbors);
    if (plan.undefinedFilters)
    { NS_LOG_DEBUG("Undefined cylinder node=" << Address() << " destination=" << destination
        << " self=" << mobility->GetPosition() << " target=" << *target); }
    if (packet)
    {
        m_diagnostics.ObserveCandidates(plan.neighborCount,plan.kind == RoutePlan::Kind::DIRECT ? 1 : plan.scores.size());
        m_diagnostics.undefinedFormulas += plan.undefinedFilters;
    }
    if (plan.kind == RoutePlan::Kind::LOCAL_MINIMUM)
    {
        if (packet)
        {
            ++m_diagnostics.localMinima; ++m_diagnostics.noRoute;
            AgentTag previous;
            if (!source && packet->PeekPacketTag(previous))
            { SendFeedback(previous.dataId,destination,previous.previous); }
        }
        return nullptr;
    }
    Ipv4Address next;
    double alpha, reward;
    if (plan.kind == RoutePlan::Kind::DIRECT)
    {
        next = destination; alpha = Model::LearningRate(0.5); reward = m_settings.rewardMax;
    }
    else
    {
        auto& table = m_qTables[destination];
        std::vector<uint32_t> allowed;
        for (auto address : plan.addresses) { allowed.push_back(address.Get()); }
        if (packet && m_random->GetValue(0,1) <= m_epsilon)
        {
            next = plan.addresses.at(m_random->GetInteger(0,plan.addresses.size()-1));
            ++m_diagnostics.explorations;
        }
        else
        {
            const auto best = table.BestActions(allowed);
            next = Ipv4Address(best.at(packet ? m_random->GetInteger(0,best.size()-1) : 0));
            if (packet) { ++m_diagnostics.exploitations; }
        }
        const auto position = std::find(plan.addresses.begin(),plan.addresses.end(),next)-plan.addresses.begin();
        const auto& score = plan.scores.at(position);
        if (!score.learningRate || !score.ordinaryReward)
        {
            NS_LOG_DEBUG("Undefined candidate score node=" << Address() << " destination=" << destination
                << " next=" << next << " raw_ct=" << score.raw[1].value_or(-1)
                << " raw_angle=" << score.raw[3].value_or(-1));
            if (packet)
            {
                ++m_diagnostics.undefinedFormulas; ++m_diagnostics.noRoute;
                AgentTag previous;
                if (m_settings.outcomeUpdates && !source && packet->PeekPacketTag(previous))
                { SendFeedback(previous.dataId,destination,previous.previous); }
            }
            return nullptr;
        }
        alpha = *score.learningRate; reward = *score.ordinaryReward;
    }
    if (packet)
    {
        AgentTag tag;
        const bool hasPrevious = packet->RemovePacketTag(tag);
        if (m_settings.outcomeUpdates && !source && hasPrevious)
        {
            std::vector<uint32_t> allowed;
            if (plan.kind == RoutePlan::Kind::DIRECT) { allowed.push_back(destination.Get()); }
            else
            {
                for (std::size_t i=0; i<plan.addresses.size(); ++i)
                {
                    if (plan.scores[i].learningRate && plan.scores[i].ordinaryReward)
                    { allowed.push_back(plan.addresses[i].Get()); }
                }
            }
            const double maximum = m_qTables[destination].Maximum(allowed).value_or(0);
            SendValue(tag.dataId,destination,tag.previous,maximum,false);
        }
        NS_ABORT_MSG_IF(m_actionSequence == std::numeric_limits<uint64_t>::max(),"QRCF action sequence exhausted");
        tag.dataId = m_actionSequence++;
        tag.previous = Address(); packet->AddPacketTag(tag);
        Time horizon = Seconds(std::max(m_settings.connectionCap,m_settings.helloUpper));
        if (m_settings.outcomeUpdates && m_arpCache)
        {
            UintegerValue retries; m_arpCache->GetAttribute("MaxRetries",retries);
            const Time arpHorizon = Seconds(m_arpCache->GetWaitReplyTimeout().GetSeconds()*
                                            (double(retries.Get())+1)) + NanoSeconds(1);
            horizon = std::max(horizon,arpHorizon);
        }
        m_pending[tag.dataId] = {destination,next,alpha,false,reward,Simulator::Now()+horizon};
        if (m_settings.outcomeUpdates)
        {
            if (!m_pendingEvent.IsRunning())
            { m_pendingEvent = Simulator::Schedule(horizon,&RoutingProtocol::ExpirePending,this); }
        }
        else { UpdateQ(destination,next,reward,alpha); }
    }
    return MakeRoute(destination,next);
}
Ptr<Ipv4Route> RoutingProtocol::RouteOutput(Ptr<Packet> packet,const Ipv4Header& header,
                                           Ptr<NetDevice> output,Socket::SocketErrno& error)
{
    // Do not train on a route rejected by the requested output-interface rule.
    if (output && (m_interface < 0 || m_ipv4->GetNetDevice(m_interface) != output))
    { error = Socket::ERROR_NOROUTETOHOST; return nullptr; }
    auto route = Lookup(packet,header,true);
    error = route ? Socket::ERROR_NOTERROR : Socket::ERROR_NOROUTETOHOST;
    return route;
}
bool RoutingProtocol::RouteInput(Ptr<const Packet> packet,const Ipv4Header& header,Ptr<const NetDevice> input,
    const UnicastForwardCallback& forward,const MulticastForwardCallback&,
    const LocalDeliverCallback& local,const ErrorCallback& error)
{
    const int32_t interface = m_ipv4->GetInterfaceForDevice(input);
    if (interface < 0) { return false; }
    if (m_ipv4->IsDestinationAddress(header.GetDestination(),interface))
    {
        if (!local.IsNull()) { local(packet,header,interface); return true; }
        return false;
    }
    if (header.GetDestination().IsMulticast() || header.GetDestination().IsBroadcast()) { return false; }
    ControlTag control;
    if (packet->PeekPacketTag(control)) { return false; }
    if (!m_ipv4->IsForwarding(interface))
    { if (!error.IsNull()) { error(packet,header,Socket::ERROR_NOROUTETOHOST); } return true; }
    auto copy = packet->Copy();
    auto route = Lookup(copy,header,false);
    if (!route) { return false; }
    forward(route,copy,header); return true;
}
void RoutingProtocol::PrintRoutingTable(Ptr<OutputStreamWrapper> stream,Time::Unit) const
{
    auto& out = *stream->GetStream();
    out << "QRCF address=" << Address() << " cylinder=" << "aligned"
        << " learning=" << (m_settings.bootstrap ? "outcome-bellman" : "immediate-reward")
        << " neighbors=" << m_neighbors.Entries().size() << " epsilon=" << m_epsilon << '\n';
    for (const auto& [destination,table] : m_qTables)
    { for (const auto& [next,q] : table.Values()) { out << destination << " via " << Ipv4Address(next) << " Q=" << q << '\n'; } }
}
} // namespace ns3::qrcf
