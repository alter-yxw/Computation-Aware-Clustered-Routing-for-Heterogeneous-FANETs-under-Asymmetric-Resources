/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "cfr-eng-task-application.h"
#include "ns3/double.h"
#include "ns3/inet-socket-address.h"
#include "ns3/arp-cache.h"
#include "ns3/ipv4-interface.h"
#include "ns3/ipv4-l3-protocol.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/uinteger.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <stdexcept>

namespace ns3::cfreng
{
static constexpr uint16_t TASK_PORT = 26911;
using cfr::Policy;
NS_OBJECT_ENSURE_REGISTERED(TaskApplication);
TypeId TaskApplication::GetTypeId()
{
    static TypeId tid = TypeId("ns3::cfreng::TaskApplication").SetParent<Application>()
        .SetGroupName("CfrEngineering").AddConstructor<TaskApplication>()
        .AddAttribute("ResultRatio", "Output bytes / processed input bytes.", DoubleValue(.05),
                      MakeDoubleAccessor(&TaskApplication::m_resultRatio), MakeDoubleChecker<double>(0, 1))
        .AddAttribute("HardwareK", "Eq.13 coefficient with frequency in Hz.", DoubleValue(1e-28),
                      MakeDoubleAccessor(&TaskApplication::m_k), MakeDoubleChecker<double>(0))
        .AddAttribute("PacingBps", "Sender fragment pacing; PHY/MAC measures actual network delay.", DoubleValue(6e6),
                      MakeDoubleAccessor(&TaskApplication::m_pacingBps), MakeDoubleChecker<double>(1))
        .AddAttribute("FragmentBytes", "Application payload per UDP fragment.", UintegerValue(900),
                      MakeUintegerAccessor(&TaskApplication::m_fragmentBytes), MakeUintegerChecker<uint32_t>(1, 1000))
        .AddAttribute("MaxHops", "Task path bound.", UintegerValue(64),
                      MakeUintegerAccessor(&TaskApplication::m_maxHops), MakeUintegerChecker<uint32_t>(1, 256))
        .AddAttribute("ReassemblyTimeout", "Per-hop reassembly deadline.", TimeValue(Seconds(10)),
                      MakeTimeAccessor(&TaskApplication::m_reassemblyTimeout), MakeTimeChecker(MilliSeconds(1)));
    return tid;
}
void TaskApplication::Configure(Ptr<RoutingProtocol> routing, std::shared_ptr<TaskStatistics> statistics,
                                 double watts, uint32_t capacity)
{
    m_routing = routing; m_statistics = std::move(statistics); m_txWatts = watts; m_queueCapacity = capacity;
}
void TaskApplication::ConfigureEndToEnd(uint16_t slot, double ghz, Ipv4Address address,
    std::shared_ptr<TaskStatistics> statistics, double watts, uint32_t capacity)
{
    m_slot = slot; m_ghz = ghz; m_address = address;
    m_statistics = std::move(statistics); m_txWatts = watts; m_queueCapacity = capacity;
    m_strategy = "destination";
}
void TaskApplication::SetStrategy(const std::string& strategy)
{
    if (strategy != "joint" && strategy != "destination" && strategy != "source")
        throw std::runtime_error("Engineering strategy must be joint|destination|source");
    m_strategy = strategy;
}

void TaskApplication::StartApplication()
{
    if ((!m_routing && m_address == Ipv4Address()) || !m_statistics) throw std::runtime_error("CFR task app is not configured");
    m_socket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
    if (m_socket->Bind(InetSocketAddress(Ipv4Address::GetAny(), TASK_PORT)) < 0)
        throw std::runtime_error("Cannot bind CFR task socket");
    m_socket->SetRecvCallback(MakeCallback(&TaskApplication::Receive, this)); m_running = true;
}
void TaskApplication::Submit(uint32_t id, double cycles, uint32_t bytes, Ipv4Address destination,
                             Time deadline, std::function<void(uint32_t)> complete)
{
    if (!m_running || !std::isfinite(cycles) || cycles < 0 || bytes == 0 || !deadline.IsStrictlyPositive())
        throw std::runtime_error("Invalid CFR task submission");
    TaskHeader h; h.taskId = id; h.originalCycles = h.remainingCycles = cycles;
    h.originalBytes = bytes; h.source = Address(); h.destination = destination;
    h.createdNs = Simulator::Now().GetNanoSeconds(); h.deadlineNs = (Simulator::Now() + deadline).GetNanoSeconds();
    if (m_submitted.count(id)) throw std::runtime_error("Duplicate CFR source task");
    m_callbacks[id] = std::move(complete); m_submitted[id] = h; m_statistics->Generated(h);
    m_events.erase(std::remove_if(m_events.begin(), m_events.end(), [](const EventId& e) { return e.IsExpired(); }), m_events.end());
    if (!m_evaluationDeadlineOnly) m_events.push_back(Simulator::Schedule(deadline, [this, h]() {
        Drop(h, "deadline"); m_callbacks.erase(h.taskId);
    }));
    Process(h);
}
bool TaskApplication::Expired(const TaskHeader& h) const
{
    return !m_evaluationDeadlineOnly &&
        static_cast<uint64_t>(Simulator::Now().GetNanoSeconds()) >= h.deadlineNs;
}
void TaskApplication::Drop(const TaskHeader& h, const std::string& reason)
{
    if (m_executionObserver) m_executionObserver("transport_failed_" + reason, h);
    m_statistics->Finish(h, reason);
    m_statistics->AbortTransitions(h);
}
uint32_t TaskApplication::WireBytes(const TaskHeader& h) const
{
    const double remaining = h.originalCycles > 0 ? h.remainingCycles / h.originalCycles : 0;
    return std::max(1u, static_cast<uint32_t>(std::ceil(h.originalBytes *
        (remaining + m_resultRatio * (1 - remaining)))));
}
bool TaskApplication::ShouldCompute(const TaskHeader& h, double wait) const
{
    if (h.remainingCycles <= 0 || Ghz() <= 0) return false;
    if (h.destination == Address()) return true;
    if (m_strategy == "destination") return false;
    if (m_strategy == "source") return h.hop == 0 && h.source == Address();
    const auto next = m_routing->Decide(h.destination, {}, h.path);
    if (next.next < 0) return true; // computation can proceed while links change
    const auto view = m_routing->View();
    const auto peer = std::find_if(view.begin(), view.end(), [&](const Observation& o) {
        return o.slot == next.next;
    });
    if (peer == view.end() || peer->ghz <= 0) return true;
    // An explicit optimistic estimate: remote queues and MAC contention unknown.
    // Shared fixed 6 Mbit/s pacing, not a global PHY oracle.
    const double raw = 8.0 * WireBytes(h) / m_pacingBps;
    const double result = 8.0 * std::max(1.0, std::ceil(h.originalBytes * m_resultRatio)) / m_pacingBps;
    const double localTime = wait + Policy::ComputeSeconds(h.remainingCycles, Ghz()) + result;
    const double remoteTime = raw + Policy::ComputeSeconds(h.remainingCycles, peer->ghz);
    return localTime <= remoteTime;
}
void TaskApplication::Process(TaskHeader h)
{
    if (!m_running) return;
    m_events.erase(std::remove_if(m_events.begin(), m_events.end(), [](const EventId& e) {
        return e.IsExpired(); }), m_events.end());
    if (Expired(h)) { Drop(h, "deadline"); return; }
    const auto slot = Slot();
    if (std::find(h.path.begin(), h.path.end(), slot) != h.path.end()) { Drop(h, "loop"); return; }
    h.path.push_back(slot);
    if (h.hop >= m_maxHops) { Drop(h, "hop_limit"); return; }
    const double wait = std::max(0.0, (m_cpuAvailable - Simulator::Now()).GetSeconds());
    double cycles = ShouldCompute(h, wait) ? h.remainingCycles : 0;
    const double duration = Policy::ComputeSeconds(cycles, Ghz());
    const bool unavailable = cycles > 0 && (m_cpuReservations >= m_queueCapacity ||
        (!m_evaluationDeadlineOnly && Simulator::Now() + Seconds(wait + duration) >= NanoSeconds(h.deadlineNs)));
    if (unavailable)
    {
        // Offload if possible; a destination cannot manufacture unserved completion.
        if (h.destination == Address() || m_strategy == "source")
        {
            if (m_executionObserver) {
                m_executionObserver("server_received", h);
                m_executionObserver("queue_rejected", h);
            }
            Drop(h, m_cpuReservations >= m_queueCapacity ? "cpu_queue_full" : "compute_deadline"); return;
        }
        cycles = 0;
    }
    const double compute = Policy::ComputeSeconds(cycles, Ghz());
    const double queue = cycles > 0 ? wait : 0;
    if (cycles > 0) {
        ++m_cpuReservations; m_cpuAvailable = Simulator::Now() + Seconds(queue + compute);
        if (m_executionObserver) {
            m_executionObserver("server_received", h);
            m_events.push_back(Simulator::Schedule(Seconds(queue), [this, h]() {
                m_executionObserver("execution_started", h);
            }));
        }
    }
    // Only endpoint fractions 0 and 1: exact zero requires actual charged service.
    h.remainingCycles -= cycles;
    h.computeSeconds += compute; h.queueSeconds += queue;
    h.computeJoules += Policy::ComputeJoules(cycles, Ghz(), m_k);
    m_events.push_back(Simulator::Schedule(Seconds(queue + compute), [this, h, cycles]() {
        if (cycles > 0) {
            --m_cpuReservations;
            if (m_executionObserver) m_executionObserver("execution_finished", h);
        }
        ComputeDone(h);
    }));
}
void TaskApplication::ComputeDone(TaskHeader h)
{
    if (!m_running) return;
    if (h.hop == 0 && h.source == Address()) m_statistics->SourceComputed(h);
    if (Expired(h)) { Drop(h, "deadline"); return; }
    if (h.destination == Address()) { Arrive(h); return; }
    Ipv4Address next = h.destination;
    if (m_routing)
    {
        // Choose from the current received Hello table AFTER FIFO/CPU service.
        const auto action = m_routing->Decide(h.destination, {}, h.path);
        if (action.next < 0) { Drop(h, "no_neighbor"); return; }
        next = m_routing->NeighborAddress(action.next);
    }
    ++h.hop; h.wireBytes = WireBytes(h); h.chunk = 0;
    h.chunks = (h.wireBytes + m_fragmentBytes - 1) / m_fragmentBytes;
    h.sentNs = Simulator::Now().GetNanoSeconds(); h.txWatts = m_txWatts;
    m_events.push_back(Simulator::ScheduleNow(&TaskApplication::SendChunk, this, h, next,
                                             std::min(m_fragmentBytes, h.wireBytes)));
}
bool TaskApplication::AwaitingArp(Ipv4Address next) const
{
    auto ipv4 = GetNode()->GetObject<Ipv4L3Protocol>();
    const auto interface = ipv4->GetInterfaceForAddress(Address());
    if (interface < 0) return true;
    auto cache = ipv4->GetInterface(interface)->GetArpCache();
    if (!cache) return false;
    auto entry = cache->Lookup(next);
    return !entry || (!(entry->IsPermanent() || entry->IsAutoGenerated()) &&
                     (!entry->IsAlive() || entry->IsExpired()));
}
void TaskApplication::SendChunk(TaskHeader h, Ipv4Address next, uint32_t bytes)
{
    if (!m_running || Expired(h)) return;
    // The first fragment triggers ordinary ARP. Keep subsequent fragments in
    // CFR until resolution, rather than overflowing the shared ARP pending
    // queue (three packets by default). No cache/PHY/global settings change.
    if (h.chunk > 0 && m_routing && AwaitingArp(next)) {
        if (Simulator::Now() - NanoSeconds(h.sentNs) >= m_reassemblyTimeout) {
            Drop(h, "arp_timeout"); return;
        }
        m_events.push_back(Simulator::Schedule(MilliSeconds(1), &TaskApplication::SendChunk,
                                               this, h, next, bytes));
        return;
    }
    // A hop transfer cannot secretly become a multi-hop IP route after mobility.
    auto packet = Create<Packet>(bytes); packet->AddHeader(h);
    if (m_decorate) m_decorate(packet, h);
    if (m_routing) { SocketIpTtlTag ttl; ttl.SetTtl(1); packet->AddPacketTag(ttl); }
    if (m_socket->SendTo(packet, 0, InetSocketAddress(next, TASK_PORT)) < 0) {
        Drop(h, "send_error"); return;
    }
    ++h.chunk;
    if (h.chunk < h.chunks) {
        const auto remaining = std::min(m_fragmentBytes, h.wireBytes - h.chunk * m_fragmentBytes);
        const auto spacing = Seconds((bytes + h.GetSerializedSize() + 28) * 8 / m_pacingBps);
        m_events.push_back(Simulator::Schedule(spacing, &TaskApplication::SendChunk,
                                               this, h, next, remaining));
    }
}
void TaskApplication::Receive(Ptr<Socket> socket)
{
    ns3::Address from;
    while (auto p = socket->RecvFrom(from))
    {
        TaskHeader h;
        if (!p->RemoveHeader(h)) continue;
        if (h.kind == 1)
        {
            if (h.source != Address()) continue;
            auto submitted = m_submitted.find(h.taskId);
            if (submitted == m_submitted.end() || h.remainingCycles != 0 ||
                h.destination != submitted->second.destination || h.createdNs != submitted->second.createdNs ||
                h.deadlineNs != submitted->second.deadlineNs || h.originalBytes != submitted->second.originalBytes ||
                h.originalCycles != submitted->second.originalCycles ||
                !InetSocketAddress::IsMatchingType(from) ||
                InetSocketAddress::ConvertFrom(from).GetPort() != TASK_PORT ||
                InetSocketAddress::ConvertFrom(from).GetIpv4() != h.destination) continue;
            // Observe late replies even after the source callback has expired.
            // Accepted completion still requires the existing live callback.
            m_statistics->ObserveAck(h.source, h.taskId);
            auto it = m_callbacks.find(h.taskId);
            if (it != m_callbacks.end())
            {
                m_statistics->Acknowledge(h.source, h.taskId); auto callback = std::move(it->second);
                m_callbacks.erase(it); m_submitted.erase(h.taskId); callback(h.taskId);
            }
            else m_submitted.erase(h.taskId);
            continue;
        }
        if (h.kind != 0 || Expired(h) || h.chunks == 0 || h.chunks > 1000000 || h.chunk >= h.chunks ||
            h.path.empty() || h.path.size() > m_maxHops || h.wireBytes == 0 ||
            h.chunks != (h.wireBytes + m_fragmentBytes - 1) / m_fragmentBytes ||
            p->GetSize() != std::min(m_fragmentBytes, h.wireBytes - h.chunk * m_fragmentBytes)) continue;
        Key key{h.source.Get(), h.taskId, h.hop};
        for (auto it = m_seen.begin(); it != m_seen.end();)
            if (it->second <= Simulator::Now()) it = m_seen.erase(it); else ++it;
        if (m_seen.count(key)) continue;
        auto it = m_assemblies.find(key);
        if (it == m_assemblies.end())
        {
            Assembly a; a.task = h; a.received.resize(h.chunks, false);
            a.timeout = Simulator::Schedule(m_evaluationDeadlineOnly ? m_reassemblyTimeout :
                                            std::min(m_reassemblyTimeout, NanoSeconds(h.deadlineNs) - Simulator::Now()),
                                            &TaskApplication::AssemblyTimeout, this, key);
            it = m_assemblies.emplace(key, std::move(a)).first;
        }
        auto& a = it->second;
        if (a.task.chunks != h.chunks || a.task.wireBytes != h.wireBytes || a.received[h.chunk]) continue;
        a.received[h.chunk] = true; ++a.count; a.bytes += p->GetSize();
        if (a.count == h.chunks && a.bytes == h.wireBytes)
        {
            a.timeout.Cancel(); h = a.task; m_assemblies.erase(it);
            m_seen[key] = m_evaluationDeadlineOnly ? Time::Max() : NanoSeconds(h.deadlineNs);
            double hop = (Simulator::Now() - NanoSeconds(h.sentNs)).GetSeconds();
            h.networkSeconds += hop; h.communicationJoules += hop * h.txWatts;
            Process(h);
        }
    }
}
void TaskApplication::AssemblyTimeout(Key key)
{
    auto it = m_assemblies.find(key);
    if (it != m_assemblies.end())
    {
        Drop(it->second.task, "reassembly_timeout");
        m_seen[key] = m_evaluationDeadlineOnly ? Time::Max() : NanoSeconds(it->second.task.deadlineNs);
        m_assemblies.erase(it);
    }
}
void TaskApplication::Arrive(TaskHeader h)
{
    bool complete = h.remainingCycles == 0;
    m_statistics->Finish(h, complete ? "complete" : "incomplete");
    if (!complete) return; // No ACK without actual complete service.
    if (h.source == Address())
    {
        auto it = m_callbacks.find(h.taskId);
        if (it != m_callbacks.end())
        {
            auto callback = std::move(it->second); m_callbacks.erase(it); m_submitted.erase(h.taskId);
            m_statistics->Acknowledge(h.source, h.taskId); callback(h.taskId);
        }
        return;
    }
    h.kind = 1; h.path.clear(); auto packet = Create<Packet>(8); packet->AddHeader(h);
    if (m_decorate) m_decorate(packet, h);
    m_socket->SendTo(packet, 0, InetSocketAddress(h.source, TASK_PORT));
}
void TaskApplication::StopApplication()
{
    m_running = false;
    for (auto& e : m_events) e.Cancel();
    m_events.clear();
    for (auto& [key, a] : m_assemblies) a.timeout.Cancel();
    m_assemblies.clear();
    if (m_socket) { m_socket->SetRecvCallback(MakeNullCallback<void, Ptr<Socket>>()); m_socket->Close(); }
    m_callbacks.clear(); m_submitted.clear();
    m_cpuReservations = 0; m_cpuAvailable = Simulator::Now(); m_seen.clear();
}
void TaskApplication::DoDispose()
{
    StopApplication(); m_socket = nullptr; m_routing = nullptr; m_statistics.reset(); m_decorate = {};
    m_executionObserver = {};
    Application::DoDispose();
}
}
