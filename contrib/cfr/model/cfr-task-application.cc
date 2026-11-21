/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "cfr-task-application.h"
#include "ns3/double.h"
#include "ns3/inet-socket-address.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/uinteger.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <stdexcept>

namespace ns3::cfr
{
static constexpr uint16_t TASK_PORT = 26901;
NS_OBJECT_ENSURE_REGISTERED(TaskApplication);
void TaskStatistics::OpenReplay(const std::string& path)
{
    m_replay.open(path);
    if (!m_replay) throw std::runtime_error("Cannot open CFR replay: " + path);
    m_replay << std::setprecision(17);
}
void TaskStatistics::BeginTransition(const TaskHeader& h, const std::vector<double>& state,
                                    const std::vector<double>& parameters, const std::vector<bool>& mask, int chosen)
{
    if (!m_replay.is_open()) return;
    m_transitions[{h.source.Get(), h.taskId, static_cast<uint16_t>(h.hop + 1)}] =
        {state, parameters, mask, chosen, m_slots.at(h.destination.Get())};
}
void TaskStatistics::EndTransition(const TaskHeader& h, const std::vector<double>& nextState,
                                  const std::vector<bool>& mask, double reward, bool done)
{
    if (!m_replay.is_open()) return;
    auto it = m_transitions.find({h.source.Get(), h.taskId, h.hop});
    if (it == m_transitions.end()) return;
    const auto& t = it->second;
    auto array = [this](const auto& values) {
        m_replay << '[';
        for (size_t i = 0; i < values.size(); ++i) { if (i) m_replay << ','; m_replay << +values[i]; }
        m_replay << ']';
    };
    m_replay << "{\"s\":"; array(t.state); m_replay << ",\"a\":"; array(t.parameters);
    m_replay << ",\"mask\":"; array(t.mask);
    m_replay << ",\"chosen\":" << t.chosen << ",\"dest\":" << t.destination << ",\"r\":" << reward
             << ",\"done\":" << (done ? "true" : "false") << ",\"ns\":";
    array(nextState.empty() ? std::vector<double>(t.state.size(), 0) : nextState);
    m_replay << ",\"next_mask\":";
    array(mask.empty() ? std::vector<bool>(t.mask.size(), false) : mask);
    m_replay << "}\n"; m_transitions.erase(it);
}
void TaskStatistics::AbortTransitions(const TaskHeader& h)
{
    for (auto it = m_transitions.begin(); it != m_transitions.end();)
    {
        if (std::get<0>(it->first) == h.source.Get() && std::get<1>(it->first) == h.taskId)
        {
            auto failed = h; failed.hop = std::get<2>(it->first); ++it;
            EndTransition(failed, {}, {}, -20, true);
        }
        else ++it;
    }
}
void TaskStatistics::Generated(const TaskHeader& h) { m_rows[{h.source.Get(), h.taskId}] = {h, "pending", 0, false}; }
void TaskStatistics::Finish(const TaskHeader& h, const std::string& status)
{
    auto it = m_rows.find({h.source.Get(), h.taskId});
    if (it == m_rows.end() || it->second.status != "pending") return;
    it->second.task = h; it->second.status = status; it->second.finished = Simulator::Now().GetSeconds();
}
void TaskStatistics::Acknowledge(Ipv4Address source, uint32_t id)
{
    ObserveAck(source, id);
    auto it = m_rows.find({source.Get(), id}); if (it != m_rows.end()) it->second.acknowledged = true;
}
void TaskStatistics::ObserveAck(Ipv4Address source, uint32_t id)
{
    auto it = m_rows.find({source.Get(), id});
    if (it != m_rows.end() && it->second.status == "complete" && it->second.ackReceivedNs < 0)
        it->second.ackReceivedNs = Simulator::Now().GetNanoSeconds();
}
void TaskStatistics::SourceComputed(const TaskHeader& task)
{
    auto it = m_rows.find({task.source.Get(), task.taskId});
    if (it != m_rows.end()) it->second.sourceComputeCycles = task.originalCycles - task.remainingCycles;
}
std::string TaskStatistics::AckStatus(const Row& row) const
{
    if (row.acknowledged) return "accepted";
    if (row.status != "complete") return "not_applicable";
    if (row.ackReceivedNs >= 0)
        return static_cast<uint64_t>(row.ackReceivedNs) >= row.task.deadlineNs ? "late" : "received_not_accepted";
    return static_cast<uint64_t>(Simulator::Now().GetNanoSeconds()) >= row.task.deadlineNs
        ? "not_observed_by_deadline" : "waiting_at_end";
}
uint32_t TaskStatistics::Count(const std::string& status, bool acknowledgedOnly) const
{
    uint32_t count = 0;
    for (const auto& [key, r] : m_rows)
        if (r.status == status && (!acknowledgedOnly || r.acknowledged)) ++count;
    return count;
}
const TaskStatistics::Row* TaskStatistics::Find(Ipv4Address source, uint32_t id) const
{
    auto it = m_rows.find({source.Get(), id});
    return it == m_rows.end() ? nullptr : &it->second;
}
double TaskStatistics::Average(double TaskHeader::*field, bool acknowledgedOnly) const
{
    double sum = 0; uint32_t count = 0;
    for (const auto& [key, r] : m_rows)
        if (r.status == "complete" && (!acknowledgedOnly || r.acknowledged))
        { sum += r.task.*field; ++count; }
    return count ? sum / count : 0;
}
double TaskStatistics::AverageE2e() const
{
    double sum = 0; uint32_t count = 0;
    for (const auto& [key, r] : m_rows) if (r.status == "complete")
    { sum += r.finished - r.task.createdNs * 1e-9; ++count; }
    return count ? sum / count : 0;
}
void TaskStatistics::WriteCsv(const std::string& path) const
{
    std::ofstream out(path);
    if (!out) throw std::runtime_error("Cannot write CFR task ledger: " + path);
    out << "source,task_id,destination,status,acknowledged,created_s,finished_s,e2e_s,original_cycles,remaining_cycles,original_bytes,last_wire_bytes,hops,compute_s,cpu_queue_s,network_s,compute_j,communication_j,cost_j_plus_s,path,deadline_s,deadline_budget_s,ack_observed,ack_received_s,ack_return_s,ack_status,source_compute_cycles,source_compute_fraction\n";
    out << std::setprecision(17);
    for (const auto& [key, r] : m_rows)
    {
        const auto& h = r.task;
        double e2e = r.status == "pending" ? 0 : r.finished - h.createdNs * 1e-9;
        out << h.source << ',' << h.taskId << ',' << h.destination << ',' << r.status << ','
            << r.acknowledged << ',' << h.createdNs * 1e-9 << ',' << r.finished << ',' << e2e << ','
            << h.originalCycles << ',' << h.remainingCycles << ',' << h.originalBytes << ',' << h.wireBytes << ','
            << h.hop << ',' << h.computeSeconds << ',' << h.queueSeconds << ',' << h.networkSeconds << ','
            << h.computeJoules << ',' << h.communicationJoules << ',' << e2e + h.computeJoules + h.communicationJoules << ',';
        for (size_t i = 0; i < h.path.size(); ++i) out << (i ? ":" : "") << h.path[i];
        bool observed = r.ackReceivedNs >= 0;
        double ackReceived = observed ? r.ackReceivedNs * 1e-9 : -1;
        out << ',' << h.deadlineNs * 1e-9 << ',' << (h.deadlineNs - h.createdNs) * 1e-9
            << ',' << observed << ',' << ackReceived << ',' << (observed ? ackReceived - r.finished : -1)
            << ',' << AckStatus(r) << ',' << r.sourceComputeCycles << ','
            << (h.originalCycles > 0 ? r.sourceComputeCycles / h.originalCycles : 0) << '\n';
    }
}
void TaskStatistics::Print(std::ostream& out) const
{
    auto flags = out.flags(); auto precision = out.precision();
    out << std::defaultfloat << std::setprecision(6);
    double generated = GeneratedCount(); auto complete = Count("complete"), incomplete = Count("incomplete");
    out << "\nCFR task metrics (destination delivery, before ACK): generated=" << generated
        << " delivered=" << complete + incomplete << " complete=" << complete << " pending=" << Count("pending")
        << " PDR=" << (generated ? (complete + incomplete) / generated : 0)
        << " TCR/all=" << (generated ? complete / generated : 0)
        << " TCR/delivered=" << (complete + incomplete ? double(complete) / (complete + incomplete) : 0)
        << " E2E(s)=" << AverageE2e()
        << " compute(J)=" << Average(&TaskHeader::computeJoules)
        << " comm(J,Eq14)=" << Average(&TaskHeader::communicationJoules)
        << " cost(J+s)=" << AverageE2e() + Average(&TaskHeader::computeJoules) + Average(&TaskHeader::communicationJoules)
        << " (means over complete tasks)\n";
    uint32_t remote = 0, remoteComplete = 0, remoteDelivered = 0, remoteAck = 0;
    std::map<std::string, uint32_t> outcomes;
    std::map<std::string, uint32_t> ackOutcomes;
    for (const auto& [key, r] : m_rows) if (r.task.source != r.task.destination)
    {
        ++remote; ++outcomes[r.status];
        remoteComplete += r.status == "complete";
        remoteDelivered += r.status == "complete" || r.status == "incomplete";
        remoteAck += r.acknowledged;
        if (r.status == "complete") ++ackOutcomes[AckStatus(r)];
    }
    out << "CFR remote tasks (local jobs excluded): generated=" << remote
        << " delivered=" << remoteDelivered << " complete=" << remoteComplete
        << " acknowledged=" << remoteAck << " TCR=" << (remote ? double(remoteComplete) / remote : 0)
        << " outcomes={";
    bool first = true;
    for (const auto& [status, count] : outcomes)
    { out << (first ? "" : ", ") << status << ':' << count; first = false; }
    out << "}\n";
    out << "CFR remote completion replies: "; first = true;
    for (const auto& [status, count] : ackOutcomes)
    { out << (first ? "" : ", ") << status << '=' << count; first = false; }
    out << " (observations by simulation end)\n";
    out.flags(flags); out.precision(precision);
}
TypeId TaskApplication::GetTypeId()
{
    static TypeId tid = TypeId("ns3::cfr::TaskApplication").SetParent<Application>()
        .SetGroupName("Cfr").AddConstructor<TaskApplication>()
        .AddAttribute("ResultRatio", "Output bytes / processed input bytes.", DoubleValue(.05),
                      MakeDoubleAccessor(&TaskApplication::m_resultRatio), MakeDoubleChecker<double>(0, 1))
        .AddAttribute("HardwareK", "Eq.13 coefficient with frequency in Hz.", DoubleValue(1e-28),
                      MakeDoubleAccessor(&TaskApplication::m_k), MakeDoubleChecker<double>(0))
        .AddAttribute("PacingBps", "Sender fragment pacing; PHY/MAC measures actual network delay.", DoubleValue(6e6),
                      MakeDoubleAccessor(&TaskApplication::m_pacingBps), MakeDoubleChecker<double>(1))
        .AddAttribute("CompletionTolerance", "Residual fraction rounded to zero after computation.", DoubleValue(1e-3),
                      MakeDoubleAccessor(&TaskApplication::m_completionTolerance), MakeDoubleChecker<double>(0, .01))
        .AddAttribute("DiagnosticCompute", "Diagnostic-only execution override: normal|none|source-full.",
                      StringValue("normal"), MakeStringAccessor(&TaskApplication::m_diagnosticCompute),
                      MakeStringChecker())
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
    if (routing->GetPolicy().IsPaperReference())
    {
        // These assumptions travel with the frozen model, so analytical
        // training and ns-3 never silently use different result/completion rules.
        m_resultRatio = routing->GetPolicy().ResultRatio();
        m_completionTolerance = routing->GetPolicy().CompletionTolerance();
    }
}
void TaskApplication::StartApplication()
{
    if (!m_routing || !m_statistics) throw std::runtime_error("CFR task app is not configured");
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
    h.originalBytes = bytes; h.source = m_routing->Address(); h.destination = destination;
    h.createdNs = Simulator::Now().GetNanoSeconds(); h.deadlineNs = (Simulator::Now() + deadline).GetNanoSeconds();
    m_callbacks[id] = std::move(complete); m_statistics->Generated(h);
    m_events.erase(std::remove_if(m_events.begin(), m_events.end(), [](const EventId& e) { return e.IsExpired(); }), m_events.end());
    m_events.push_back(Simulator::Schedule(deadline, [this, h]() {
        Drop(h, "deadline"); m_callbacks.erase(h.taskId);
    }));
    Process(h);
}
bool TaskApplication::Expired(const TaskHeader& h) const
{
    return static_cast<uint64_t>(Simulator::Now().GetNanoSeconds()) >= h.deadlineNs;
}
void TaskApplication::Drop(const TaskHeader& h, const std::string& reason)
{
    m_statistics->Finish(h, reason);
    m_statistics->AbortTransitions(h);
}
bool TaskApplication::BitMode() const
{
    return m_routing->GetPolicy().BitGranularity();
}
uint64_t TaskApplication::OriginalBits(const TaskHeader& h) const
{
    return 8ull * h.originalBytes;
}
uint64_t TaskApplication::RemainingBits(const TaskHeader& h) const
{
    uint64_t bits = OriginalBits(h);
    if (bits == 0 || !(h.originalCycles > 0) || !std::isfinite(h.originalCycles)) return 0;
    double cyclesPerBit = h.originalCycles / static_cast<double>(bits);
    double count = std::llround(std::max(0., h.remainingCycles) / cyclesPerBit);
    return std::min<uint64_t>(static_cast<uint64_t>(count), bits);
}
uint32_t TaskApplication::WireBytes(const TaskHeader& h) const
{
    double remaining;
    if (BitMode())
    {
        uint64_t bits = OriginalBits(h);
        remaining = bits ? static_cast<double>(RemainingBits(h)) / static_cast<double>(bits) : 0;
    }
    else
        remaining = h.originalCycles > 0 ? h.remainingCycles / h.originalCycles : 0;
    return std::max(1u, static_cast<uint32_t>(std::ceil(h.originalBytes * (remaining + m_resultRatio * (1 - remaining)))));
}
void TaskApplication::Process(TaskHeader h)
{
    if (!m_running) return;
    m_events.erase(std::remove_if(m_events.begin(), m_events.end(), [](const EventId& e) { return e.IsExpired(); }), m_events.end());
    if (Expired(h)) { Drop(h, "deadline"); return; }
    auto slot = m_routing->Slot();
    if (std::find(h.path.begin(), h.path.end(), slot) != h.path.end()) { Drop(h, "loop"); return; }
    h.path.push_back(slot);
    // Local submissions use the common application's local execution semantics.
    bool local = h.destination == m_routing->Address() && h.hop == 0;
    if (h.destination == m_routing->Address() && !local) { Arrive(h); return; }
    if (h.hop >= m_maxHops) { Drop(h, "hop_limit"); return; }
    bool bitMode = BitMode();
    uint64_t originalBits = 0, remainingBits = 0, processedBits = 0;
    double cyclesPerBit = 0;
    if (bitMode)
    {
        originalBits = OriginalBits(h);
        if (originalBits == 0 || !(h.originalCycles > 0) || !std::isfinite(h.originalCycles))
        { Drop(h, "invalid_bit_task"); return; }
        cyclesPerBit = h.originalCycles / static_cast<double>(originalBits);
        remainingBits = RemainingBits(h);
        // The analytical environment and ns-3 both carry the quantized task
        // size across hops. Quantize before deciding so state/Q see the same
        // integer-bit task actually being routed.
        h.remainingCycles = remainingBits ? remainingBits * cyclesPerBit : 0.;
    }
    Action action = local ? Action{-1, 1, 0} : m_routing->Decide(h.destination,
        {h.originalCycles, h.remainingCycles, h.originalBytes}, h.path);
    if (action.next < 0 && !local) { Drop(h, "no_neighbor"); return; }
    auto next = local ? h.destination : m_routing->NeighborAddress(action.next);
    // Diagnostic-only ablation of execution; routing decisions still come from
    // the frozen policy. It is not a paper mechanism and defaults to "normal".
    double executed = action.fraction;
    if (m_diagnosticCompute == "none") executed = 0.;
    else if (m_diagnosticCompute == "source-full")
        executed = (h.hop == 0 && h.source == m_routing->Address()) ? 1. : 0.;
    else if (m_diagnosticCompute != "normal")
        throw std::runtime_error("Invalid CFR DiagnosticCompute");
    double cycles;
    if (bitMode)
    {
        uint64_t budget = m_routing->GetPolicy().OriginalPartition() ? originalBits : remainingBits;
        processedBits = m_routing->Ghz() > 0 ? Policy::ProcessedBits(executed, budget) : 0;
        processedBits = std::min(processedBits, remainingBits);
        cycles = processedBits * cyclesPerBit;
    }
    else
    {
        cycles = h.remainingCycles * executed;
        if (m_routing->Ghz() <= 0) cycles = 0;
        // Charge the small residual as real computation when applying the tolerance.
        if (cycles > 0 && h.remainingCycles - cycles <= m_completionTolerance * h.originalCycles)
            cycles = h.remainingCycles;
    }
    if (!local) {
        auto state = m_routing->State(h.destination, {h.originalCycles, h.remainingCycles, h.originalBytes});
        auto parameters = m_routing->Parameters(state);
        // Reference Q is evaluated on the requested proposal vector. Actual
        // work may be zero, but replacing that vector only in replay changes
        // the learned action representation. Historical formats retain theirs.
        // Diagnostic execution overrides record the executed fraction so a
        // diagnostic replay cannot claim the unexecuted actor proposal.
        if (!m_routing->GetPolicy().IsPaperReference() || m_diagnosticCompute != "normal")
            parameters[action.next] = h.remainingCycles > 0 ? cycles / h.remainingCycles : 0;
        else if (m_routing->GetPolicy().IsEndpointDiagnostic())
            // The diagnostic selector may replace the actor's coordinate by
            // zero/full. Replay must store that requested action, before the
            // physical no-op mapping, just as its Python trainer does.
            parameters[action.next] = action.requestedFraction;
        m_statistics->BeginTransition(h, state, parameters, m_routing->Mask(h.path), action.next);
    }
    double compute = Policy::ComputeSeconds(cycles, m_routing->Ghz());
    double energy = Policy::ComputeJoules(cycles, m_routing->Ghz(), m_k);
    double wait = 0;
    if (cycles > 0)
    {
        if (m_cpuReservations >= m_queueCapacity) { Drop(h, "cpu_queue_full"); return; }
        wait = std::max(0.0, (m_cpuAvailable - Simulator::Now()).GetSeconds());
    }
    if (Simulator::Now() + Seconds(wait + compute) >= NanoSeconds(h.deadlineNs))
    {
        Drop(h, "compute_deadline"); return;
    }
    if (cycles > 0) {
        ++m_cpuReservations;
        m_cpuAvailable = Simulator::Now() + Seconds(wait + compute);
    }
    if (bitMode)
        h.remainingCycles = remainingBits == processedBits ? 0. :
            (remainingBits - processedBits) * cyclesPerBit;
    else
        h.remainingCycles = std::max(0.0, h.remainingCycles - cycles);
    h.computeSeconds += compute; h.queueSeconds += wait; h.computeJoules += energy;
    h.hopCompute = compute; h.hopEnergy = energy;
    m_events.push_back(Simulator::Schedule(Seconds(wait + compute), [this, h, next, cycles]() {
        if (cycles > 0) --m_cpuReservations;
        ComputeDone(h, next);
    }));
}
void TaskApplication::ComputeDone(TaskHeader h, Ipv4Address next)
{
    if (!m_running) return;
    if (h.hop == 0 && h.source == m_routing->Address()) m_statistics->SourceComputed(h);
    if (Expired(h)) { Drop(h, "deadline"); return; }
    if (h.destination == m_routing->Address()) { Arrive(h); return; }
    if (!m_routing->IsDirect(next)) { Drop(h, "neighbor_expired_during_compute"); return; }
    ++h.hop; h.wireBytes = WireBytes(h); h.chunk = 0;
    h.chunks = (h.wireBytes + m_fragmentBytes - 1) / m_fragmentBytes;
    h.sentNs = Simulator::Now().GetNanoSeconds(); h.txWatts = m_txWatts;
    double offset = 0;
    for (uint32_t j = 0; j < h.chunks; ++j)
    {
        auto bytes = std::min(m_fragmentBytes, h.wireBytes - j * m_fragmentBytes);
        h.chunk = j;
        m_events.push_back(Simulator::Schedule(Seconds(offset), &TaskApplication::SendChunk, this, h, next, bytes));
        offset += (bytes + h.GetSerializedSize() + 28) * 8 / m_pacingBps;
    }
}
void TaskApplication::SendChunk(TaskHeader h, Ipv4Address next, uint32_t bytes)
{
    if (!m_running || Expired(h)) return;
    // A hop transfer cannot secretly become a multi-hop IP route after mobility.
    auto packet = Create<Packet>(bytes); packet->AddHeader(h);
    SocketIpTtlTag ttl; ttl.SetTtl(1); packet->AddPacketTag(ttl);
    if (m_socket->SendTo(packet, 0, InetSocketAddress(next, TASK_PORT)) < 0) Drop(h, "send_error");
}
void TaskApplication::Receive(Ptr<Socket> socket)
{
    Address from;
    while (auto p = socket->RecvFrom(from))
    {
        TaskHeader h;
        if (!p->RemoveHeader(h)) continue;
        if (h.kind == 1)
        {
            if (h.source != m_routing->Address()) continue;
            // Observe late replies even after the source callback has expired.
            // Accepted completion still requires the existing live callback.
            m_statistics->ObserveAck(h.source, h.taskId);
            auto it = m_callbacks.find(h.taskId);
            if (it != m_callbacks.end())
            {
                m_statistics->Acknowledge(h.source, h.taskId); auto callback = std::move(it->second);
                m_callbacks.erase(it); callback(h.taskId);
            }
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
            a.timeout = Simulator::Schedule(std::min(m_reassemblyTimeout, NanoSeconds(h.deadlineNs) - Simulator::Now()),
                                            &TaskApplication::AssemblyTimeout, this, key);
            it = m_assemblies.emplace(key, std::move(a)).first;
        }
        auto& a = it->second;
        if (a.task.chunks != h.chunks || a.task.wireBytes != h.wireBytes || a.received[h.chunk]) continue;
        a.received[h.chunk] = true; ++a.count; a.bytes += p->GetSize();
        if (a.count == h.chunks && a.bytes == h.wireBytes)
        {
            a.timeout.Cancel(); h = a.task; m_assemblies.erase(it); m_seen[key] = NanoSeconds(h.deadlineNs);
            double hop = (Simulator::Now() - NanoSeconds(h.sentNs)).GetSeconds();
            h.networkSeconds += hop; h.communicationJoules += hop * h.txWatts;
            bool arrived = h.destination == m_routing->Address();
            auto visited = h.path; visited.push_back(m_routing->Slot());
            auto mask = m_routing->Mask(visited);
            bool deadEnd = std::none_of(mask.begin(), mask.end(), [](bool v) { return v; });
            double compute = h.hopCompute, energy = h.hopEnergy + hop * h.txWatts;
            if (arrived && h.remainingCycles > 0) {
                // Eq.19's hypothetical remaining computation is a reward only.
                compute += m_routing->Ghz() > 0 ? Policy::ComputeSeconds(h.remainingCycles, m_routing->Ghz()) : 100;
                energy += Policy::ComputeJoules(h.remainingCycles, m_routing->Ghz(), m_k);
            }
            double reward = Policy::Reward(arrived, h.remainingCycles == 0, h.networkSeconds, hop, compute, energy);
            if (!arrived && (deadEnd || h.hop >= m_maxHops)) reward -= 20;
            m_statistics->EndTransition(h, m_routing->State(h.destination,
                {h.originalCycles, h.remainingCycles, h.originalBytes}), mask, reward,
                arrived || deadEnd || h.hop >= m_maxHops);
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
        m_seen[key] = NanoSeconds(it->second.task.deadlineNs); m_assemblies.erase(it);
    }
}
void TaskApplication::Arrive(TaskHeader h)
{
    bool complete = h.remainingCycles == 0;
    m_statistics->Finish(h, complete ? "complete" : "incomplete");
    if (!complete) return; // destination does not silently compute unfinished input
    if (h.source == m_routing->Address())
    {
        auto it = m_callbacks.find(h.taskId);
        if (it != m_callbacks.end())
        {
            auto callback = std::move(it->second); m_callbacks.erase(it);
            m_statistics->Acknowledge(h.source, h.taskId); callback(h.taskId);
        }
        return;
    }
    h.kind = 1; h.path.clear(); auto packet = Create<Packet>(8); packet->AddHeader(h);
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
    m_callbacks.clear();
    m_cpuReservations = 0; m_cpuAvailable = Simulator::Now(); m_seen.clear();
}
void TaskApplication::DoDispose()
{
    StopApplication(); m_socket = nullptr; m_routing = nullptr; m_statistics.reset(); Application::DoDispose();
}
}
