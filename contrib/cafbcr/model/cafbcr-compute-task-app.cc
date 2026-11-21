/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "cafbcr-compute-task-app.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/socket-factory.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/inet-socket-address.h"
#include "ns3/ipv4-address.h"
#include "ns3/random-variable-stream.h"
#include "ns3/object.h"
#include "ns3/fanet-node.h"
#include "ns3/fanet-neighbor.h"
#include <cmath>
#include <limits>
#include <algorithm>

namespace ns3 {
namespace cafbcr {

NS_LOG_COMPONENT_DEFINE("CafbcrComputeTaskApp");

namespace {
ComputeTaskWireEvent WireEvent(const std::string& stage, uint32_t node, bool header,
    Ptr<const Packet> packet, const Address& peer, const CafbcrComputeTaskTag* task,
    uint32_t payload, bool valid, const std::string& reason,
    std::optional<int32_t> result = std::nullopt)
{
  ComputeTaskWireEvent event;
  event.event = stage; event.observerNodeId = node; event.time = Simulator::Now();
  event.format = header ? "header-v1" : "legacy-tag";
  event.packet = packet; event.peer = peer; event.valid = valid;
  event.reason = reason; event.socketResult = result;
  if (task) { event.sourceNodeId = task->GetSourceNodeId(); event.taskId = task->GetTaskId(); }
  if (valid) { event.payloadBytes = payload; event.metadataBytes = header ? ComputeTaskHeader::SIZE : 0; }
  return event;
}
bool ReadTask(Ptr<Packet> packet, bool header, ComputeTaskHeader::Kind kind,
              CafbcrComputeTaskTag& task, Time& deadline, std::string& reason)
{
  reason = "ok";
  if (!header) {
    if (!packet->RemovePacketTag(task)) { reason = "missing_tag"; return false; }
    if (!std::isfinite(task.GetWorkload()) ||
        (kind == ComputeTaskHeader::REQUEST ? task.GetWorkload() <= 0 : task.GetWorkload() != 0)) {
      reason = "invalid_tag"; return false;
    }
    deadline = Seconds(task.GetDeadline());
    return true;
  }
  // The routing tag is deliberately not consulted in this branch.
  ComputeTaskHeader wire;
  if (packet->GetSize() < ComputeTaskHeader::SIZE) { reason = "short_header"; return false; }
  if (packet->PeekHeader(wire) != ComputeTaskHeader::SIZE || !wire.IsValid()) {
    reason = "invalid_header"; return false;
  }
  if (wire.GetKind() != kind) { reason = "wrong_message_kind"; return false; }
  if (packet->GetSize() - ComputeTaskHeader::SIZE != wire.GetPayloadBytes()) {
    reason = "payload_size_mismatch"; return false;
  }
  task = wire.ToTag(); deadline = wire.GetDeadline();
  return true;
}
} // namespace

// ========== ComputeTaskClientApp 实现 ==========

NS_OBJECT_ENSURE_REGISTERED(ComputeTaskClientApp);

TypeId ComputeTaskClientApp::GetTypeId()
{
  static TypeId tid = TypeId("ns3::cafbcr::ComputeTaskClientApp")
    .SetParent<Application>()
    .SetGroupName("Cafbcr")
    .AddConstructor<ComputeTaskClientApp>()
    .AddTraceSource("ServerDiscoveryDecision", "Passive candidate and selected information provenance",
                    MakeTraceSourceAccessor(&ComputeTaskClientApp::m_discoveryEvent),
                    "ns3::cafbcr::ServerDiscoveryDecision::TracedCallback")
    .AddTraceSource("TaskEvent", "Passive task lifecycle observations",
                    MakeTraceSourceAccessor(&ComputeTaskClientApp::m_taskEvent),
                    "ns3::cafbcr::ComputeTaskEvent::TracedCallback")
    .AddTraceSource("TaskWire", "Passive task packet bytes and socket results",
                    MakeTraceSourceAccessor(&ComputeTaskClientApp::m_wireEvent),
                    "ns3::cafbcr::ComputeTaskWireEvent::TracedCallback");
  return tid;
}

ComputeTaskClientApp::ComputeTaskClientApp()
  : m_remotePort(889),
    m_taskInterval(Seconds(5.0)),
    m_minWorkload(10.0),
    m_maxWorkload(100.0),
    m_minDataSize(512),
    m_maxDataSize(2048),
    m_minDeadline(Seconds(10.0)),
    m_maxDeadline(Seconds(30.0)),
    m_nextTaskId(0),
    m_demandRandom(CreateObject<UniformRandomVariable>()),
    m_selectionRandom(CreateObject<UniformRandomVariable>())
{
}

ComputeTaskClientApp::~ComputeTaskClientApp()
{
}

void ComputeTaskClientApp::SetRemoteAddress(Ipv4Address addr)
{
  m_remoteAddress = addr;
}

void ComputeTaskClientApp::SetRemotePort(uint16_t port)
{
  m_remotePort = port;
}

void ComputeTaskClientApp::SetServerSelectionPolicy(ServerSelectionPolicy policy)
{
  NS_ABORT_MSG_IF(m_running || !m_taskMap.empty(), "Set server selection before starting");
  NS_ABORT_MSG_IF(policy != ServerSelectionPolicy::NATIVE_NEIGHBOR &&
                  policy != ServerSelectionPolicy::FIXED_ASSIGNMENT, "Unknown server selection policy");
  m_serverSelectionPolicy = policy;
}

void ComputeTaskClientApp::SetAdditionalServerCandidates(AdditionalServerCandidates provider)
{
  NS_ABORT_MSG_IF(m_running || !m_taskMap.empty(), "Set candidate provider before generation");
  m_additionalServerCandidates = std::move(provider);
}

void ComputeTaskClientApp::SetTaskInterval(Time interval)
{
  NS_ABORT_MSG_IF(interval <= TimeStep(0), "Task interval must be positive");
  m_taskInterval = interval;
}

void ComputeTaskClientApp::SetTaskWorkload(double minGflops, double maxGflops)
{
  NS_ABORT_MSG_IF(!std::isfinite(minGflops) || !std::isfinite(maxGflops) ||
                  minGflops <= 0 || maxGflops < minGflops, "Invalid task workload range");
  m_minWorkload = minGflops;
  m_maxWorkload = maxGflops;
}

void ComputeTaskClientApp::SetTaskDataSize(uint32_t minBytes, uint32_t maxBytes)
{
  NS_ABORT_MSG_IF(minBytes == 0 || maxBytes < minBytes, "Invalid task payload range");
  m_minDataSize = minBytes;
  m_maxDataSize = maxBytes;
}

void ComputeTaskClientApp::SetTaskDeadline(Time minDeadline, Time maxDeadline)
{
  NS_ABORT_MSG_IF(minDeadline < TimeStep(0) || maxDeadline < minDeadline,
                  "Invalid task deadline range");
  m_minDeadline = minDeadline;
  m_maxDeadline = maxDeadline;
}

int64_t ComputeTaskClientApp::AssignStreams(int64_t stream)
{
  NS_ABORT_MSG_IF(m_running || !m_taskMap.empty(), "Assign task streams before starting");
  NS_ABORT_MSG_IF(stream < 0 || stream == std::numeric_limits<int64_t>::max(),
                  "Task streams require two nonnegative adjacent stream IDs");
  m_demandRandom->SetStream(stream);
  m_selectionRandom->SetStream(stream + 1);
  return 2;
}

void ComputeTaskClientApp::SetObservationWindow(Time start, Time stop, Time observationStop)
{
  NS_ABORT_MSG_IF(m_running || start < TimeStep(0) || stop < TimeStep(0) ||
                  stop > observationStop, "Invalid task observation window");
  // A node phase may fall after stop; that node simply generates no tasks.
  m_generationStart = start;
  m_generationStop = stop;
  m_observationStop = observationStop;
}

ComputeTaskSummary ComputeTaskClientApp::GetTaskSummary() const
{
  ComputeTaskSummary summary;
  Time cutoff = std::min(Simulator::Now(), m_observationStop);
  for (const auto& [id, task] : m_taskMap) {
    if (task.generatedAt > cutoff) continue;
    ++summary.generated;
    if (task.acknowledgedAt && *task.acknowledgedAt <= cutoff) {
      if (*task.acknowledgedAt <= task.generatedAt + task.deadline) ++summary.onTime;
      else ++summary.late;
      summary.responseSeconds += (*task.acknowledgedAt - task.generatedAt).GetSeconds();
    } else if (task.generatedAt + task.deadline <= cutoff) {
      ++summary.unconfirmedDue;
    } else {
      ++summary.unconfirmedFuture;
    }
  }
  return summary;
}

uint32_t ComputeTaskClientApp::GetTasksCompleted() const
{
  auto s = GetTaskSummary();
  return s.onTime + s.late;
}

uint32_t ComputeTaskClientApp::GetTasksMissedDeadline() const
{
  auto s = GetTaskSummary();
  return s.late + s.unconfirmedDue;
}

uint32_t ComputeTaskClientApp::GetTasksTimeout() const
{
  return GetTaskSummary().unconfirmedDue;
}

double ComputeTaskClientApp::GetAverageResponseTime() const
{
  auto s = GetTaskSummary();
  auto completed = s.onTime + s.late;
  return completed ? s.responseSeconds / completed : std::numeric_limits<double>::quiet_NaN();
}

double ComputeTaskClientApp::GetDeadlineMissRate() const
{
  auto s = GetTaskSummary();
  return s.generated ? static_cast<double>(s.late + s.unconfirmedDue) / s.generated
                     : std::numeric_limits<double>::quiet_NaN();
}

void ComputeTaskClientApp::DoDispose()
{
  StopApplication();
  m_taskTransport = {};
  m_taskDestinationSelector = {};
  m_additionalServerCandidates = {};
  m_externalSelectionPolicy.clear();
  m_socket = nullptr;
  m_taskMap.clear();
  Application::DoDispose();
}

void ComputeTaskClientApp::StartApplication()
{
  NS_LOG_FUNCTION(this);
  NS_ABORT_MSG_IF(m_additionalServerCandidates &&
      (m_serverSelectionPolicy != ServerSelectionPolicy::NATIVE_NEIGHBOR || m_taskDestinationSelector),
      "Additional candidates require native selection and no external destination selector");
  NS_ABORT_MSG_IF(GetNode()->GetId() > std::numeric_limits<uint16_t>::max(),
                  "Task tag source ID cannot represent this node");
  // One task-producing client per node is required by (source, sequence) identity.
  for (uint32_t i = 0; i < GetNode()->GetNApplications(); ++i) {
    auto other = DynamicCast<ComputeTaskClientApp>(GetNode()->GetApplication(i));
    NS_ABORT_MSG_IF(other && PeekPointer(other) != this,
                    "Multiple compute clients on one node would reuse task identities");
  }
  m_running = true;
  m_generating = true;

  if (!m_socket) {
    m_socket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
    NS_ABORT_MSG_IF(m_socket->Bind() < 0, "Cannot bind compute client socket");
    m_socket->SetRecvCallback(MakeCallback(&ComputeTaskClientApp::HandleRead, this));
  }

  // 开始发送任务
  Time first = std::max(Simulator::Now(), m_generationStart);
  if (first < m_generationStop) {
    m_sendEvent = Simulator::Schedule(first - Simulator::Now(),
                                     &ComputeTaskClientApp::GenerateTask, this);
  }
}

void ComputeTaskClientApp::StopGenerating()
{
  m_generating = false;
  if (m_sendEvent.IsRunning()) {
    Simulator::Remove(m_sendEvent);
  }
}

void ComputeTaskClientApp::StopApplication()
{
  StopGenerating();
  m_running = false;
  if (m_socket) {
    m_socket->Close();
    m_socket = nullptr;
  }
}

void ComputeTaskClientApp::Emit(const std::string& event, const ComputeTaskRecord& task)
{
  m_taskEvent(ComputeTaskEvent{event, task.sourceNodeId, task.taskId, GetNode()->GetId(),
    Simulator::Now(), task.workload, task.dataSize, task.deadline,
    task.selectedServer, task.selectionPolicy, task.visibleStateAge, task.visibleStateStatus});
}

void ComputeTaskClientApp::SetTaskTransport(TaskTransport transport)
{
  NS_ABORT_MSG_IF(m_running || !m_taskMap.empty(), "Set task transport before generation");
  m_taskTransport = std::move(transport);
}

void ComputeTaskClientApp::SetTaskDestinationSelector(TaskDestinationSelector selector,
                                                      const std::string& policy)
{
  NS_ABORT_MSG_IF(m_running || !m_taskMap.empty(), "Set task destination selector before generation");
  NS_ABORT_MSG_IF(bool(selector) != !policy.empty(), "External selector requires its own policy identity");
  m_taskDestinationSelector = std::move(selector);
  m_externalSelectionPolicy = policy;
}

void ComputeTaskClientApp::TransportCompleted(uint32_t taskId)
{
  if (!m_running || Simulator::Now() > m_observationStop) return;
  auto it = m_taskMap.find(taskId);
  NS_ABORT_MSG_IF(it == m_taskMap.end(), "Transport completed an unknown task");
  auto& task = it->second;
  if (task.acknowledgedAt) { Emit("duplicate_completion", task); return; }
  task.acknowledgedAt = Simulator::Now();
  Emit("acknowledged", task);
}

void ComputeTaskClientApp::GenerateTask()
{
  NS_LOG_FUNCTION(this);
  if (!m_running || !m_generating || Simulator::Now() >= m_generationStop ||
      Simulator::Now() > m_observationStop) return;
  NS_ABORT_MSG_IF(m_nextTaskId == std::numeric_limits<uint32_t>::max(),
                  "Compute task sequence exhausted");

  // 生成任务属性
  double workload = m_demandRandom->GetValue(m_minWorkload, m_maxWorkload);
  uint32_t dataSize = static_cast<uint32_t>(
      m_demandRandom->GetValue(m_minDataSize, m_maxDataSize));
  Time deadline = Seconds(m_demandRandom->GetValue(m_minDeadline.GetSeconds(),
                                                   m_maxDeadline.GetSeconds()));

  // 创建任务包
  CafbcrComputeTaskTag taskTag;
  taskTag.SetTaskId(m_nextTaskId);
  taskTag.SetWorkload(workload);
  taskTag.SetDataSize(dataSize);
  taskTag.SetDeadline(deadline.GetSeconds());
  taskTag.SetSourceNodeId(GetNode()->GetId());

  // 创建数据包
  Ptr<Packet> packet = Create<Packet>(dataSize);
  packet->AddPacketTag(taskTag);
  if (m_useTaskHeader) {
    ComputeTaskHeader wire;
    wire.SetTask(taskTag, deadline, ComputeTaskHeader::REQUEST);
    packet->AddHeader(wire);
  }

  // Select the destination before each task: prefer neighbours with high
  // compute capacity and low load, falling back to m_remoteAddress.
  const bool fixed = m_serverSelectionPolicy == ServerSelectionPolicy::FIXED_ASSIGNMENT;
  m_discoveryDecision = {};
  const auto selection = fixed ? ServerSelection{m_remoteAddress, std::nullopt, "fixed_assignment"}
                               : PickDynamicServer();
  Ipv4Address dynDst = selection.address;
  Ipv4Address actualDst = dynDst.IsInitialized() ? dynDst : m_remoteAddress;

  // A generated demand belongs to the denominator even if local sending fails.
  ComputeTaskRecord task{GetNode()->GetId(), m_nextTaskId++, Simulator::Now(), deadline,
    workload, dataSize, actualDst,
    fixed ? "fixed_assignment" : dynDst.IsInitialized() ? "neighbor_tier_power_of_two" : "fixed_fallback", std::nullopt,
    selection.age, selection.status};
  if (m_taskDestinationSelector) {
    actualDst = m_taskDestinationSelector(task);
    task.selectedServer = actualDst;
    task.selectionPolicy = m_externalSelectionPolicy;
    task.visibleStateAge.reset();
    task.visibleStateStatus = "protocol_received_state";
  }
  if (fixed || m_taskDestinationSelector) {
    m_discoveryDecision = {};
    m_discoveryDecision.reason = task.selectionPolicy;
  }
  m_discoveryDecision.sourceNode = task.sourceNodeId;
  m_discoveryDecision.taskId = task.taskId;
  m_discoveryDecision.time = task.generatedAt;
  m_discoveryDecision.selected = actualDst;
  m_discoveryEvent(m_discoveryDecision);
  m_taskMap.emplace(task.taskId, task);
  Emit("generated", task);
  if (!actualDst.IsInitialized() || actualDst == Ipv4Address::GetAny()) {
    m_wireEvent(WireEvent("request_not_sent", GetNode()->GetId(), m_useTaskHeader,
      packet, Address(), &taskTag, dataSize, true, "no_server"));
    Emit("no_server", task);
  } else if (m_taskTransport) {
    m_taskTransport(task, [this, id = task.taskId]() { TransportCompleted(id); });
  } else {
    Address peer = InetSocketAddress(actualDst, m_remotePort);
    int sent = m_socket ? m_socket->SendTo(packet, 0, peer) : -1;
    m_wireEvent(WireEvent("request_tx", GetNode()->GetId(), m_useTaskHeader,
      packet, peer, &taskTag, dataSize, true, "ok", sent));
    Emit(sent < 0 ? "send_failed" : "request_sent", task);
  }

  // 调度下一个任务
  if (Simulator::Now() + m_taskInterval < m_generationStop) {
    m_sendEvent = Simulator::Schedule(m_taskInterval, &ComputeTaskClientApp::GenerateTask, this);
  }
}

// Tiered fallback keeps a nearby scheduling path available to SWARM tasks:
// Tier 1: neighbours with compute >= 10 GFLOP/s (RELAY/COMMAND), fast service;
// Tier 2: a Head neighbour, which has a wider view for scheduling;
// Tier 3: neighbours with compute >= 5 GFLOP/s (TASK class), slower service;
// Tier 4: m_remoteAddress, the slowest remote fallback.
// Each tier uses power-of-two choices to limit herding.
ComputeTaskClientApp::ServerSelection ComputeTaskClientApp::PickDynamicServer()
{
  Ptr<Node> node = GetNode();
  if (!node) return {};
  Ptr<ClusterNode> cn = node->GetObject<ClusterNode>();
  if (!cn) return {};
  Ptr<Neighbors> neigh = cn->GetObject<Neighbors>();
  if (!neigh) return {};

  struct Cand {
    Ipv4Address ip; double score; Neighbors::ComputeObservation observation;
    uint16_t origin; uint8_t tier;
    std::optional<ServerDiscoveryCandidate> extra;
  };
  std::vector<Cand> tier1, tier2, tier3;
  std::set<uint16_t> usableDirect;

  for (const auto& kv : neigh->GetAll()) {
    const auto& info = kv.second;
    const auto observation = neigh->GetComputeObservation(kv.first);
    if (!observation.IsUsable()) {
      if (observation.status == Neighbors::ComputeObservation::INVALID) ++m_discoveryDecision.invalid;
      else if (observation.status == Neighbors::ComputeObservation::EXPIRED) ++m_discoveryDecision.expired;
      else ++m_discoveryDecision.unknown;
      continue;
    }
    if (observation.status == Neighbors::ComputeObservation::UNTRACKED) ++m_discoveryDecision.untracked;
    usableDirect.insert(kv.first);
    double score = info.computeCapacity *
                   (1.0 - static_cast<double>(info.currentLoad));
    if (info.computeCapacity >= 10.0) {
      tier1.push_back({info.ipAddress, score, observation, kv.first, 1, std::nullopt});
    } else if (info.Role == Head) {
      // CH 不一定算力强，但是簇调度入口，路径稳定
      tier2.push_back({info.ipAddress, score + 1.0, observation, kv.first, 2, std::nullopt});
    } else if (info.computeCapacity >= 5.0) {
      tier3.push_back({info.ipAddress, score, observation, kv.first, 3, std::nullopt});
    }
  }

  m_discoveryDecision.direct = {uint32_t(tier1.size()), uint32_t(tier2.size()), uint32_t(tier3.size())};
  if (m_additionalServerCandidates) {
    std::set<uint16_t> seen;
    const auto now = Simulator::Now();
    for (const auto& extra : m_additionalServerCandidates()) {
      if (extra.nodeId == node->GetId() || usableDirect.count(extra.nodeId)) continue;
      const bool valid = extra.discoveryHops == 2 && extra.address.IsInitialized() &&
        extra.address != Ipv4Address::GetAny() && std::isfinite(extra.capacity) && extra.capacity > 0 &&
        std::isfinite(extra.load) && extra.load >= 0 && extra.load <= 1 && extra.role >= Idle && extra.role <= Isolate &&
        extra.generatedAt >= Time(0) && extra.generatedAt <= extra.firstReceivedAt &&
        extra.firstReceivedAt <= extra.localReceivedAt && extra.localReceivedAt <= now && extra.expiresAt > now;
      if (!valid || !seen.insert(extra.nodeId).second) { ++m_discoveryDecision.invalid; continue; }
      uint8_t tier = extra.capacity >= 10 ? 1 : extra.role == Head ? 2 : extra.capacity >= 5 ? 3 : 0;
      if (!tier) continue;
      Neighbors::ComputeObservation observation;
      observation.status = Neighbors::ComputeObservation::FRESH;
      observation.receivedAt = extra.localReceivedAt; observation.expiresAt = extra.expiresAt;
      const double score = extra.capacity * (1.0 - static_cast<double>(extra.load)) + (tier == 2 ? 1.0 : 0.0);
      auto& pool = tier == 1 ? tier1 : tier == 2 ? tier2 : tier3;
      pool.push_back({extra.address, score, observation, extra.nodeId, tier, extra});
      ++m_discoveryDecision.additional[tier - 1];
    }
    for (auto* pool : {&tier1, &tier2, &tier3})
      std::sort(pool->begin(), pool->end(), [](const Cand& a, const Cand& b) { return a.origin < b.origin; });
  }

  std::vector<Cand>* picked = nullptr;
  if (!tier1.empty()) picked = &tier1;
  else if (!tier2.empty()) picked = &tier2;
  else if (!tier3.empty()) picked = &tier3;
  if (!picked) return {};
  auto selected = [this](const Cand& candidate) {
    m_discoveryDecision.selectedOrigin = candidate.origin;
    m_discoveryDecision.selectedTier = candidate.tier;
    m_discoveryDecision.selectedHop = candidate.extra ? 2 : 1;
    m_discoveryDecision.firstReceivedAt = candidate.observation.receivedAt;
    m_discoveryDecision.localReceivedAt = candidate.observation.receivedAt;
    m_discoveryDecision.expiresAt = candidate.observation.expiresAt;
    m_discoveryDecision.reason = candidate.extra ? "received_two_hop" : "received_neighbor";
    if (candidate.extra) {
      m_discoveryDecision.generatedAt = candidate.extra->generatedAt;
      m_discoveryDecision.firstReceivedAt = candidate.extra->firstReceivedAt;
    }
    return ServerSelection{candidate.ip,
      candidate.observation.receivedAt
        ? std::optional<Time>(Simulator::Now() - *candidate.observation.receivedAt) : std::nullopt,
      candidate.observation.StatusName()};
  };
  if (picked->size() == 1) return selected((*picked)[0]);

  // Power-of-two-choices in the chosen tier
  uint32_t i = m_selectionRandom->GetInteger(0, picked->size() - 1);
  uint32_t j = m_selectionRandom->GetInteger(0, picked->size() - 1);
  while (j == i && picked->size() > 1) {
    j = m_selectionRandom->GetInteger(0, picked->size() - 1);
  }
  return selected(((*picked)[i].score >= (*picked)[j].score) ? (*picked)[i] : (*picked)[j]);
}

void ComputeTaskClientApp::HandleRead(Ptr<Socket> socket)
{
  NS_LOG_FUNCTION(this << socket);

  Ptr<Packet> packet;
  Address from;

  while ((packet = socket->RecvFrom(from))) {
    if (!m_running || Simulator::Now() > m_observationStop) continue;
    CafbcrComputeTaskTag completeTag;
    Time deadline;
    std::string reason;
    bool valid = ReadTask(packet, m_useTaskHeader, ComputeTaskHeader::COMPLETION, completeTag, deadline, reason);
    m_wireEvent(WireEvent("response_rx", GetNode()->GetId(), m_useTaskHeader,
      packet, from, valid ? &completeTag : nullptr, 8, valid, reason));
    if (!valid) continue;
    auto it = m_taskMap.find(completeTag.GetTaskId());
    if (it == m_taskMap.end()) continue;
    auto& task = it->second;
    if (completeTag.GetSourceNodeId() != task.sourceNodeId ||
        (m_useTaskHeader && deadline != task.deadline) ||
        !InetSocketAddress::IsMatchingType(from) ||
        InetSocketAddress::ConvertFrom(from).GetIpv4() != task.selectedServer ||
        InetSocketAddress::ConvertFrom(from).GetPort() != m_remotePort) {
      Emit("invalid_completion", task);
      continue;
    }
    if (task.acknowledgedAt) {
      Emit("duplicate_completion", task);
      continue;
    }
    task.acknowledgedAt = Simulator::Now();
    Emit("acknowledged", task);
  }
}

// ========== ComputeTaskServerApp 实现 ==========

NS_OBJECT_ENSURE_REGISTERED(ComputeTaskServerApp);

TypeId ComputeTaskServerApp::GetTypeId()
{
  static TypeId tid = TypeId("ns3::cafbcr::ComputeTaskServerApp")
    .SetParent<Application>()
    .SetGroupName("Cafbcr")
    .AddConstructor<ComputeTaskServerApp>()
    .AddTraceSource("TaskEvent", "Passive server lifecycle observations",
                    MakeTraceSourceAccessor(&ComputeTaskServerApp::m_taskEvent),
                    "ns3::cafbcr::ComputeTaskEvent::TracedCallback")
    .AddTraceSource("QueueState", "Actual waiting queue and optional local load snapshot",
                    MakeTraceSourceAccessor(&ComputeTaskServerApp::m_queueState),
                    "ns3::cafbcr::ComputeQueueState::TracedCallback")
    .AddTraceSource("TaskWire", "Passive task packet bytes and socket results",
                    MakeTraceSourceAccessor(&ComputeTaskServerApp::m_wireEvent),
                    "ns3::cafbcr::ComputeTaskWireEvent::TracedCallback");
  return tid;
}

ComputeTaskServerApp::ComputeTaskServerApp()
  : m_localPort(889),
    m_computeCapacity(1.0),
    m_maxQueueCapacity(10),
    m_processing(false),
    m_tasksReceived(0),
    m_tasksStarted(0),
    m_tasksFinished(0),
    m_tasksDropped(0),
    m_totalProcessingDelay(0.0),
    m_totalWaitTime(0.0)
{
}

ComputeTaskServerApp::~ComputeTaskServerApp()
{
}

void ComputeTaskServerApp::SetLocalPort(uint16_t port)
{
  m_localPort = port;
}

void ComputeTaskServerApp::SetComputeCapacity(double gflops)
{
  NS_ABORT_MSG_IF(!std::isfinite(gflops) || gflops <= 0, "Compute capacity must be positive");
  m_computeCapacity = gflops;
}

void ComputeTaskServerApp::SetQueueCapacity(uint32_t maxTasks)
{
  NS_ABORT_MSG_IF(m_running && m_syncQueueLoad, "Configure the synchronized queue before starting");
  m_maxQueueCapacity = maxTasks;
}

void ComputeTaskServerApp::SetSynchronizeQueueLoad(bool enabled)
{
  NS_ABORT_MSG_IF(m_running, "Configure load synchronization before starting");
  m_syncQueueLoad = enabled;
}

void ComputeTaskServerApp::SetObservationStop(Time stop)
{
  NS_ABORT_MSG_IF(m_running || stop < TimeStep(0), "Invalid server observation cutoff");
  m_observationStop = stop;
}

double ComputeTaskServerApp::GetAverageProcessingDelay() const
{
  if (m_tasksFinished == 0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return m_totalProcessingDelay / m_tasksFinished;
}

double ComputeTaskServerApp::GetAverageQueueWaitTime() const
{
  if (m_tasksFinished == 0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return m_totalWaitTime / m_tasksFinished;
}

void ComputeTaskServerApp::DoDispose()
{
  StopApplication();
  m_seenTasks.clear();
  m_loadState = nullptr;
  Application::DoDispose();
}

void ComputeTaskServerApp::StartApplication()
{
  NS_LOG_FUNCTION(this);
  m_loadState = nullptr;
  if (m_syncQueueLoad) {
    m_loadState = GetNode()->GetObject<ClusterNode>();
    NS_ABORT_MSG_IF(!m_loadState || m_maxQueueCapacity == 0,
                    "Queue synchronization requires a local ClusterNode and positive waiting capacity");
    NS_ABORT_MSG_IF(m_loadState->GetCurrentTaskQueueLength() != 0 || m_loadState->GetCurrentLoad() != 0,
                    "Local load is already owned by another service");
    for (uint32_t i = 0; i < GetNode()->GetNApplications(); ++i) {
      auto other = DynamicCast<ComputeTaskServerApp>(GetNode()->GetApplication(i));
      NS_ABORT_MSG_IF(other && PeekPointer(other) != this && other->m_syncQueueLoad,
                      "Only one synchronized compute server is allowed per node");
    }
  }
  m_running = true;

  if (!m_socket) {
    m_socket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
    InetSocketAddress local(Ipv4Address::GetAny(), m_localPort);
    NS_ABORT_MSG_IF(m_socket->Bind(local) < 0, "Cannot bind compute server socket");
    m_socket->SetRecvCallback(MakeCallback(&ComputeTaskServerApp::HandleRead, this));
  }
  PublishQueueState("started");

  NS_LOG_INFO("ComputeTaskServer started on port " << m_localPort
             << " with capacity " << m_computeCapacity << " GFLOP/s");
}

void ComputeTaskServerApp::StopApplication()
{
  NS_LOG_FUNCTION(this);
  if (!m_running && !m_socket && !m_processing && m_taskQueue.empty()) return;
  m_running = false;
  if (m_processingEvent.IsRunning()) Simulator::Remove(m_processingEvent);
  if (m_processing) {
    m_processing = false;
    PublishQueueState("execution_cancelled");
    Emit("server_cancelled", m_currentTask.task.taskTag);
  }
  while (!m_taskQueue.empty()) {
    auto tag = m_taskQueue.front().task.taskTag;
    m_taskQueue.pop();
    PublishQueueState("waiting_cancelled");
    Emit("server_cancelled", tag);
  }
  PublishQueueState("stopped");

  if (m_socket) {
    m_socket->Close();
    m_socket = nullptr;
  }
}

void ComputeTaskServerApp::PublishQueueState(const std::string& event)
{
  uint32_t waiting = m_taskQueue.size();
  if (m_loadState) {
    NS_ABORT_MSG_IF(!m_loadState->SetTaskQueueSnapshot(waiting, m_maxQueueCapacity),
                    "Actual waiting queue is outside its advertised capacity");
  }
  // Includes cleanup after observationStop; collectors must mark that boundary.
  m_queueState(ComputeQueueState{GetNode()->GetId(), Simulator::Now(), event,
    waiting, m_maxQueueCapacity, m_processing, m_running,
    m_loadState ? std::optional<uint32_t>(m_loadState->GetCurrentTaskQueueLength()) : std::nullopt,
    m_loadState ? std::optional<uint32_t>(m_loadState->GetTaskQueueCapacity()) : std::nullopt,
    m_loadState ? std::optional<double>(m_loadState->GetCurrentLoad()) : std::nullopt});
}

void ComputeTaskServerApp::Emit(const std::string& event, const CafbcrComputeTaskTag& tag)
{
  if (Simulator::Now() > m_observationStop) return;
  m_taskEvent(ComputeTaskEvent{event, tag.GetSourceNodeId(), tag.GetTaskId(), GetNode()->GetId(),
    Simulator::Now(), tag.GetWorkload(), tag.GetDataSize(), Seconds(tag.GetDeadline()),
    Ipv4Address(), ""});
}

bool ComputeTaskServerApp::HandleAccept(Ptr<Socket> s, const Address& from)
{
  NS_LOG_FUNCTION(this << s << from);
  return true;
}

void ComputeTaskServerApp::HandleRead(Ptr<Socket> socket)
{
  NS_LOG_FUNCTION(this << socket);

  Ptr<Packet> packet;
  Address from;

  while ((packet = socket->RecvFrom(from))) {
    if (!m_running || Simulator::Now() > m_observationStop) continue;
    CafbcrComputeTaskTag taskTag;
    Time deadline;
    std::string reason;
    bool valid = ReadTask(packet, m_useTaskHeader, ComputeTaskHeader::REQUEST, taskTag, deadline, reason);
    m_wireEvent(WireEvent("request_rx", GetNode()->GetId(), m_useTaskHeader,
      packet, from, valid ? &taskTag : nullptr, valid ? taskTag.GetDataSize() : 0, valid, reason));
    if (valid) {
      if (!m_seenTasks.emplace(taskTag.GetSourceNodeId(), taskTag.GetTaskId()).second) {
        PublishQueueState("duplicate_request");
        Emit("duplicate_request", taskTag);
        continue;
      }
      // 成功提取任务标签
      m_tasksReceived++;
      Emit("server_received", taskTag);

      QueuedTaskWithSource task;
      task.task.taskTag = taskTag;
      task.task.arrivalTime = Simulator::Now();
      task.task.relativeDeadline = deadline;
      task.sourceAddress = from;

      // 检查队列是否已满
      if (m_taskQueue.size() >= m_maxQueueCapacity) {
        m_tasksDropped++;
        PublishQueueState("queue_rejected");
        Emit("queue_rejected", taskTag);
        NS_LOG_WARN("Task queue full, dropping task " << taskTag.GetTaskId());
      } else {
        m_taskQueue.push(task);
        PublishQueueState("enqueued");
        NS_LOG_INFO("Task enqueued: taskId=" << taskTag.GetTaskId()
                   << " workload=" << taskTag.GetWorkload() << "GFLOP"
                   << " queue_size=" << m_taskQueue.size());

        // 如果不在处理，开始处理下一个任务
        if (!m_processing && !m_taskQueue.empty()) {
          ProcessNextTask();
        }
      }
    }
  }
}

void ComputeTaskServerApp::ProcessNextTask()
{
  NS_LOG_FUNCTION(this);

  if (!m_running || Simulator::Now() > m_observationStop) return;

  if (m_taskQueue.empty()) {
    m_processing = false;
    PublishQueueState("idle");
    return;
  }

  m_processing = true;
  m_currentTask = m_taskQueue.front();
  m_taskQueue.pop();

  m_processingStartTime = Simulator::Now();
  Time waitTime = m_processingStartTime - m_currentTask.task.arrivalTime;

  // 计算处理时间
  double processingTime = m_currentTask.task.taskTag.GetWorkload() / m_computeCapacity;
  Time processingDuration = Seconds(processingTime);

  m_tasksStarted++;
  PublishQueueState("execution_started");
  Emit("execution_started", m_currentTask.task.taskTag);

  NS_LOG_INFO("Processing task " << m_currentTask.task.taskTag.GetTaskId()
             << " (duration=" << processingTime << "s"
             << " wait=" << waitTime.GetSeconds() << "s)");

  // 调度任务完成并发送响应
  m_processingEvent = Simulator::Schedule(processingDuration,
                                         &ComputeTaskServerApp::OnTaskComplete,
                                         this, m_currentTask.task.taskTag.GetTaskId(),
                                         m_currentTask.sourceAddress);
}

void ComputeTaskServerApp::OnTaskComplete(uint32_t taskId, Address clientAddress)
{
  NS_LOG_FUNCTION(this << taskId << clientAddress);
  if (!m_running || !m_processing || Simulator::Now() > m_observationStop) return;
  m_tasksFinished++;
  m_totalProcessingDelay += (Simulator::Now() - m_processingStartTime).GetSeconds();
  m_totalWaitTime += (m_processingStartTime - m_currentTask.task.arrivalTime).GetSeconds();
  m_processing = false;
  PublishQueueState("execution_finished");
  Emit("execution_finished", m_currentTask.task.taskTag);

  // 创建完成响应包
  Ptr<Packet> responsePacket = Create<Packet>(8); // Placeholder completion body, not a computed result.

  // 创建完成标签（复用ComputeTaskTag，设置特殊值表示完成）
  CafbcrComputeTaskTag completeTag = m_currentTask.task.taskTag;
  completeTag.SetWorkload(0.0); // 0工作量表示这是完成通知
  completeTag.SetDataSize(0);
  responsePacket->AddPacketTag(completeTag);
  if (m_useTaskHeader) {
    ComputeTaskHeader wire;
    wire.SetTask(completeTag, m_currentTask.task.relativeDeadline, ComputeTaskHeader::COMPLETION);
    responsePacket->AddHeader(wire);
  }

  // 发送响应回客户端
  if (m_socket) {
    int sent = m_socket->SendTo(responsePacket, 0, clientAddress);
    m_wireEvent(WireEvent("response_tx", GetNode()->GetId(), m_useTaskHeader,
      responsePacket, clientAddress, &completeTag, 8, true, "ok", sent));
    Emit(sent < 0 ? "response_send_failed" : "response_sent", m_currentTask.task.taskTag);
    NS_LOG_INFO("Task completed and response sent: taskId=" << taskId
               << " to client at " << InetSocketAddress::ConvertFrom(clientAddress).GetIpv4());
  }

  // 继续处理队列中的下一个任务
  if (!m_taskQueue.empty()) {
    ProcessNextTask();
  } else {
    m_processing = false;
  }
}

} // namespace cafbcr
} // namespace ns3
