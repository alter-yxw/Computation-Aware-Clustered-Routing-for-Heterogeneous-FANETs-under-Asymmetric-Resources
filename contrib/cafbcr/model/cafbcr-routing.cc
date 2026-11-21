/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#define NS_LOG_APPEND_CONTEXT                                                  \
  if(m_ipv4)                                                                   \
  {                                                                            \
    std::clog<<"[node " << (uint16_t)m_ipv4->GetObject<Node>()->GetId()<<"]";  \
  }

#include "cafbcr-routing.h"
#include "cafbcr-forwarding-history.h"
#include "cafbcr-revisit-recovery.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/node-list.h"
#include "ns3/ipv4-route.h"
#include "ns3/ipv4-packet-info-tag.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/wifi-net-device.h"
#include "ns3/adhoc-wifi-mac.h"
#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/uinteger.h"
#include "ns3/udp-header.h"
#include <iomanip>
#include <sstream>
#include <unordered_map>
#include <algorithm>

namespace ns3 {
namespace cafbcr {

NS_LOG_COMPONENT_DEFINE("cafbcrRoutingProtocol");
NS_OBJECT_ENSURE_REGISTERED(RoutingProtocol);

const uint32_t RoutingProtocol::CAFBCR_PORT = 889;

namespace {
// Per-routing-instance ownership, deliberately outside the Node aggregate.
// ClusterNode inherits Node in this tree; another aggregate lookup can change
// which Node GetObject<Node>() returns. This table never reads another node's
// history and is erased on Dispose/destruction. No RoutingProtocol field or
// layout change is needed. The default off setting allocates no history.
auto& PerRoutingHistories()
{
  // Process-lifetime registry avoids static destruction-order dependencies;
  // individual values are removed by normal per-instance disposal.
  static auto* histories = new std::unordered_map<const RoutingProtocol*, Ptr<ForwardingHistory>>;
  return *histories;
}
auto& PerRoutingRecoveries()
{
  static auto* recoveries = new std::unordered_map<const RoutingProtocol*, Ptr<RevisitRecovery>>;
  return *recoveries;
}
// Opt-in local diagnostics, enabled with NS_LOG=cafbcrRoutingProtocol=logic.
// This does not use the file's NS_LOG_APPEND_CONTEXT (which performs another
// GetObject<Node> lookup). It reads only existing snapshots, PeekAll, and the
// manager's read-only ID lookup. No packets, tags, RNG, events or routing
// objects are created and no mobility query or neighbor purge is added.
struct ForwardAudit
{
  const char* stage;
  const Packet* packet;
  const Ipv4Header& header;
  uint32_t node;
  uint16_t role, cluster;
  ClusterManager* manager;
  const Neighbors* neighbors;
  uint16_t incomingHead = 0xFFFF;
  bool task = false;
  const char* cacheState = "not_checked";
  std::optional<Time> cacheAge = std::nullopt;
  std::optional<Ipv4Address> cachedGateway = std::nullopt;
  std::optional<Vector> ownPosition = std::nullopt, destinationPosition = std::nullopt;

  void Emit(const char* branch, std::optional<Ipv4Address> nextHop = std::nullopt,
            bool cacheWritten = false) const
  {
#ifdef NS3_LOG_ENABLE
    if (!g_log.IsEnabled(LOG_LOGIC)) return;
    const uint16_t head = manager ? manager->GetClusterHeadId(cluster) : 0xFFFF;
    uint16_t nextId = 0xFFFF, nextRole = 0xFFFF;
    std::optional<Time> nextAge;
    std::optional<Vector> nextPosition;
    if (nextHop && neighbors) {
      for (const auto& [id, info] : neighbors->PeekAll()) {
        if (info.ipAddress == *nextHop) {
          nextId = id; nextRole = info.Role;
          nextAge = Simulator::Now() - info.UpdateTime;
          nextPosition = info.Position;
          break;
        }
      }
    }
    std::ostringstream out;
    out << std::setprecision(17) << "CA_FORWARD_V1," << Simulator::Now().GetNanoSeconds()
        << ',' << stage << ',' << branch << ',' << node << ',' << packet->GetUid() << ',';
    if (header.GetSource().IsInitialized()) out << header.GetSource(); else out << "null";
    out << ',' << header.GetDestination() << ',' << unsigned(header.GetProtocol()) << ',';
    const bool forwarded = std::string(stage) == "forward";
    if (forwarded) out << header.GetIdentification() << ',' << unsigned(header.GetTtl());
    else out << "null,null"; // RouteOutput precedes construction of the IPv4 header.
    out << ',' << packet->GetSize() << ',' << task << ',' << role << ',' << cluster << ',' << head
        << ',' << incomingHead << ',';
    if (nextHop) out << *nextHop; else out << "null";
    out << ',' << nextId << ',' << nextRole << ',';
    if (nextAge) out << nextAge->GetNanoSeconds(); else out << "null";
    out << ',' << cacheState << ',';
    if (cacheAge) out << cacheAge->GetNanoSeconds(); else out << "null";
    out << ',';
    if (cachedGateway) out << *cachedGateway; else out << "null";
    out << ',' << cacheWritten;
    for (const auto& position : {ownPosition, destinationPosition, nextPosition}) {
      if (position) out << ',' << position->x << ',' << position->y << ',' << position->z;
      else out << ",null,null,null";
    }
    std::clog << out.str() << '\n';
#endif
  }
};

// A node may retain older registrations in other clusters. Those records do
// not authorize it to maintain that cluster or admit new members into it.
Ptr<Cluster> CurrentOwnedCluster(Ptr<ClusterManager> manager, Ptr<ClusterNode> node)
{
  if (node->GetClusterId() == 0xFFFF) return nullptr;
  auto cluster = manager->GetCluster(node->GetClusterId());
  return cluster && cluster->GetClusterHeadId() == node->GetNodeId() &&
         cluster->ExistMember(node->GetNodeId()) ? cluster : nullptr;
}
} // namespace

//-----------------------------------------------------------------------------
// RoutingProtocol
//-----------------------------------------------------------------------------
RoutingProtocol::RoutingProtocol()
  : m_hasReceivedChCall(false),
    m_isRejoinScheduled(false),
    m_ipv4(nullptr),
    m_mainAddress(Ipv4Address()),
    m_socket(nullptr),
    m_lo(nullptr),
    m_helloTimer(Timer::CANCEL_ON_DESTROY),
    m_clusteringTimer(Timer::CANCEL_ON_DESTROY),
    m_beaconTimer(Timer::CANCEL_ON_DESTROY),
    m_helloInterval(Seconds(2.0)),  // 减少到0.5秒，提高邻居发现率
    m_beaconInterval(Seconds(0.75)),
    m_clusteringTime(Seconds(2.0)),  // 延迟到2秒，确保节点收集到邻居信息
    m_maxRange(500.0),
    m_enableAsymHandling(true),
    m_wLHTP(0.2),
    m_wEnergy(0.2),
    m_wDegree(0.2),
    m_wDistance(0.2),
    m_wCompute(0.2),
    m_helloMode(FIXED_HELLO),  // 默认使用固定Hello间隔
    m_fuzzyHello(nullptr),
    m_qHello(nullptr),
    m_cafbcrElection(nullptr),
    m_clusterManager(nullptr),
    m_neighbors(nullptr),
    m_helloPacketCount(0),
    m_chCallPacketCount(0),
    m_joinReqPacketCount(0),
    m_joinRespPacketCount(0),
    m_chBeaconCount(0),
    m_cmBeaconCount(0),
    m_ControlPacketSize(0)
{
  NS_LOG_FUNCTION(this);
  m_clusterManager = ClusterManager::GetInstance();
  m_uniformRandomVariable = CreateObject<UniformRandomVariable>();
  m_fuzzyHello = CreateObject<FuzzyHelloManager>();  // 创建FuzzyHelloManager
  m_qHello = CreateObject<QLearningHelloManager>();  // 创建QLearningHelloManager
}

RoutingProtocol::~RoutingProtocol()
{
  ClearPendingRenewals();
  NS_LOG_FUNCTION(this);
  SetTerminateRepeatedForward(false);
  SetAlternateRepeatedForward(false);
}

void RoutingProtocol::SetTerminateRepeatedForward(bool enabled)
{
  if (!enabled) {
    auto it = PerRoutingHistories().find(this);
    if (it != PerRoutingHistories().end()) { it->second->Dispose(); PerRoutingHistories().erase(it); }
    return;
  }
  if (PerRoutingHistories().find(this) == PerRoutingHistories().end())
  {
    SetAlternateRepeatedForward(false);
    PerRoutingHistories().emplace(this, CreateObject<ForwardingHistory>());
  }
}

void RoutingProtocol::SetAlternateRepeatedForward(bool enabled)
{
  auto it = PerRoutingRecoveries().find(this);
  if (!enabled) {
    if (it != PerRoutingRecoveries().end()) { it->second->Dispose(); PerRoutingRecoveries().erase(it); }
    return;
  }
  SetTerminateRepeatedForward(false);
  if (it == PerRoutingRecoveries().end())
    PerRoutingRecoveries().emplace(this, CreateObject<RevisitRecovery>());
}

bool RoutingProtocol::GetAlternateRepeatedForward() const
{
  return PerRoutingRecoveries().find(this) != PerRoutingRecoveries().end();
}

Ptr<RevisitRecovery> RoutingProtocol::GetRevisitRecovery() const
{
  auto it = PerRoutingRecoveries().find(this);
  return it == PerRoutingRecoveries().end() ? nullptr : it->second;
}

Ptr<ForwardingHistory> RoutingProtocol::GetForwardingHistory() const
{
  auto it = PerRoutingHistories().find(this);
  return it == PerRoutingHistories().end() ? nullptr : it->second;
}

bool RoutingProtocol::GetTerminateRepeatedForward() const
{
  return PerRoutingHistories().find(this) != PerRoutingHistories().end();
}

TypeId RoutingProtocol::GetTypeId()
{
  static TypeId tid = TypeId("ns3::cafbcr::RoutingProtocol")
    .SetParent<Ipv4RoutingProtocol>()
    .SetGroupName("cafbcr")
    .AddConstructor<RoutingProtocol>()
    .AddTraceSource("DiscoveryHelloRx", "Actual original HELLO computation fields and local receipt lease",
                    MakeTraceSourceAccessor(&RoutingProtocol::m_discoveryHelloRx),
                    "ns3::cafbcr::DiscoveryHelloRxEvent::TracedCallback")
    .AddTraceSource("DiscoveryHelloTx", "One local HELLO generation opportunity, not delivery confirmation",
                    MakeTraceSourceAccessor(&RoutingProtocol::m_discoveryHelloTx),
                    "ns3::cafbcr::DiscoveryHelloTxEvent::TracedCallback")
    .AddTraceSource("NeighborComputeState", "Local neighbor snapshot after a CA control update",
                    MakeTraceSourceAccessor(&RoutingProtocol::m_neighborComputeEvent),
                    "ns3::cafbcr::NeighborComputeEvent::TracedCallback")
    .AddTraceSource("JoinMechanism", "Local join attempts, CH admission and source confirmation",
                    MakeTraceSourceAccessor(&RoutingProtocol::m_joinEvent),
                    "ns3::cafbcr::JoinMechanismEvent::TracedCallback")
    .AddTraceSource("ProxyRenewal", "Passive HELLO/CM packet correlation and lease rejection evidence",
                    MakeTraceSourceAccessor(&RoutingProtocol::m_proxyRenewalEvent),
                    "ns3::cafbcr::ProxyRenewalEvent::TracedCallback")
    .AddTraceSource("LinkProbe", "Local real one-hop round trips and data candidate eligibility",
                    MakeTraceSourceAccessor(&RoutingProtocol::m_linkProbeEvent),
                    "ns3::cafbcr::LinkProbeEvent::TracedCallback")
    .AddAttribute("EnableLinkProbes", "Generate one-hop unicast request/reply measurements; default off",
                  BooleanValue(false),MakeBooleanAccessor(&RoutingProtocol::m_linkProbes),MakeBooleanChecker())
    .AddAttribute("UseHandshakeGate", "Replace the data candidate distance gate by recent real round-trip evidence",
                  BooleanValue(false),MakeBooleanAccessor(&RoutingProtocol::m_handshakeGate),MakeBooleanChecker())
    .AddAttribute("LinkProbeInterval", "Interval between probes of currently heard neighbors",
                  TimeValue(Seconds(2)),MakeTimeAccessor(&RoutingProtocol::m_linkProbeInterval),MakeTimeChecker(MilliSeconds(10)))
    .AddAttribute("LinkProbeTimeout", "A response must arrive strictly before this request timeout",
                  TimeValue(MilliSeconds(250)),MakeTimeAccessor(&RoutingProtocol::m_linkProbeTimeout),MakeTimeChecker(MilliSeconds(1)))
    .AddAttribute("LinkEvidenceValidity", "Evidence age measured from the original local request send time",
                  TimeValue(Seconds(6)),MakeTimeAccessor(&RoutingProtocol::m_linkEvidenceValidity),MakeTimeChecker(MilliSeconds(10)))
    .AddAttribute("HelloInterval",
                  "Hello message emission interval",
                  TimeValue(Seconds(2.0)),  // 默认0.5秒，提高邻居发现率
                  MakeTimeAccessor(&RoutingProtocol::m_helloInterval),
                  MakeTimeChecker())
    .AddAttribute("BeaconInterval",
                  "Beacon message emission interval",
                  TimeValue(Seconds(0.75)),
                  MakeTimeAccessor(&RoutingProtocol::m_beaconInterval),
                  MakeTimeChecker())
    .AddAttribute("MaxRange",
                  "Maximum communication range",
                  DoubleValue(500.0),
                  MakeDoubleAccessor(&RoutingProtocol::m_maxRange),
                  MakeDoubleChecker<double>())
    .AddAttribute("EnableAsymHandling",
                  "Compatibility master; false disables both proxy join and range gating",
                  BooleanValue(true),
                  MakeBooleanAccessor(&RoutingProtocol::m_enableAsymHandling),
                  MakeBooleanChecker())
    .AddAttribute("EnableProxyJoin",
                  "Enable proxy requests, HELLO intent and CH_LIVE grants; false uses ordinary JOIN",
                  BooleanValue(true),
                  MakeBooleanAccessor(&RoutingProtocol::m_enableProxyJoin),
                  MakeBooleanChecker())
    .AddAttribute("EnableRangeGating",
                  "Filter data forwarding by local range; proxy eligibility and PHY stay configured",
                  BooleanValue(true),
                  MakeBooleanAccessor(&RoutingProtocol::m_enableRangeGating),
                  MakeBooleanChecker())
    .AddAttribute("EnableProxyRenewal",
                  "Renew grant-confirmed members from received HELLOs relayed as CM_LIVE; requires proxy join",
                  BooleanValue(true),
                  MakeBooleanAccessor(&RoutingProtocol::m_enableProxyRenewal),
                  MakeBooleanChecker())
    .AddAttribute("CoordinateProxyRenewal",
                  "Opt-in staggered CM relays cancelled only by received CH observation acknowledgments",
                  BooleanValue(false),
                  MakeBooleanAccessor(&RoutingProtocol::m_coordinateProxyRenewal),
                  MakeBooleanChecker())
    .AddAttribute("DirectMaintenanceFeedback",
                  "Opt-in CH_LIVE direct-CM evidence with expiring source-side proxy suppression",
                  BooleanValue(false),
                  MakeBooleanAccessor(&RoutingProtocol::m_directMaintenanceFeedback),
                  MakeBooleanChecker())
    .AddAttribute("FeedbackCoveredCmResponse",
                  "Opt-in omission of extra CH_LIVE response CMs only while received current-association feedback is valid",
                  BooleanValue(false),
                  MakeBooleanAccessor(&RoutingProtocol::m_feedbackCoveredCmResponse),
                  MakeBooleanChecker())
    .AddAttribute("OneHopControlUnicast",
                  "Opt-in: own TTL-one unicast controls use their actual peer as the link destination",
                  BooleanValue(false),
                  MakeBooleanAccessor(&RoutingProtocol::m_oneHopControlUnicast),
                  MakeBooleanChecker())
    .AddAttribute("ProxyRenewalWindow",
                  "Maximum 32-slot relay holdoff, also bounded by remaining observation lifetime",
                  TimeValue(MilliSeconds(155)),
                  MakeTimeAccessor(&RoutingProtocol::m_proxyRenewalWindow),
                  MakeTimeChecker(MilliSeconds(1), Seconds(1)))
    .AddAttribute("CoalesceProxyRenewalAck",
                  "Opt-in bounded CH feedback batching; relay holdoffs are unchanged",
                  BooleanValue(false),
                  MakeBooleanAccessor(&RoutingProtocol::m_coalesceRenewalAck),
                  MakeBooleanChecker())
    .AddAttribute("ProxyRenewalAckBatchWindow",
                  "Maximum wait from the first queued feedback record; flush also at eight records",
                  TimeValue(MilliSeconds(20)),
                  MakeTimeAccessor(&RoutingProtocol::m_renewalAckBatchWindow),
                  MakeTimeChecker(MilliSeconds(1), MilliSeconds(100)))
    .AddAttribute("TerminateRepeatedForward",
                  "Opt-in local history: terminate exact wire-datagram revisits with lower TTL; no geographic recovery",
                  BooleanValue(false),
                  MakeBooleanAccessor(&RoutingProtocol::SetTerminateRepeatedForward,
                                      &RoutingProtocol::GetTerminateRepeatedForward),
                  MakeBooleanChecker())
    .AddAttribute("AlternateRepeatedForward",
                  "Opt-in node-local untried next hop on lower-TTL revisit; bounded alternatives, not face routing",
                  BooleanValue(false),
                  MakeBooleanAccessor(&RoutingProtocol::SetAlternateRepeatedForward,
                                      &RoutingProtocol::GetAlternateRepeatedForward),
                  MakeBooleanChecker())
    .AddAttribute("HelloMode",
                  "Hello interval optimization mode (0=FIXED_HELLO, 1=FUZZY_HELLO, 2=QLEARNING_HELLO)",
                  UintegerValue(0),
                  MakeUintegerAccessor(&RoutingProtocol::m_helloMode),
                  MakeUintegerChecker<uint32_t>(0, 2))
    .AddAttribute("UniformRv",
                  "Access to the underlying UniformRandomVariable",
                  StringValue("ns3::UniformRandomVariable"),
                  MakePointerAccessor(&RoutingProtocol::m_uniformRandomVariable),
                  MakePointerChecker<UniformRandomVariable>());
  return tid;
}

void RoutingProtocol::TraceJoin(const char* event, uint16_t origin, uint16_t head,
                                uint16_t cluster, Ipv4Address peer,
                                std::optional<int> socketResult, const char* cmTrigger)
{
  if (m_joinEvent.IsEmpty()) return;
  auto local = m_ipv4->GetObject<Node>()->GetObject<ClusterNode>();
  if (!local) return;
  m_joinEvent(JoinMechanismEvent{Simulator::Now(), MyNodeId(), origin, head, cluster, peer,
    event, local->GetRoleType(), local->GetClusterId(), m_intentJoinChId,
    IsProxyJoinEnabled(), IsRangeGatingEnabled(), socketResult, cmTrigger});
}

void RoutingProtocol::TraceProxyRenewal(const char* event, uint16_t origin, uint16_t head,
    uint16_t cluster, Ipv4Address peer, uint64_t packetUid, double sourceStamp,
    std::optional<Time> previous, std::optional<uint64_t> helloUid,
    std::optional<int> socketResult, uint8_t timeRejections)
{
  if (m_proxyRenewalEvent.IsEmpty()) return;
  std::optional<Time> sourceTime;
  if (std::isfinite(sourceStamp) && sourceStamp >= 0) sourceTime = Seconds(sourceStamp);
  m_proxyRenewalEvent(ProxyRenewalEvent{Simulator::Now(), MyNodeId(), origin, head,
    cluster, peer, event, packetUid, helloUid, sourceStamp, sourceTime, previous,
    m_beaconInterval * 10, timeRejections, socketResult});
}

void RoutingProtocol::DoInitialize()
{
  NS_LOG_FUNCTION(this);

  // 获取Helper已经聚合的cafbcrElection对象
  if (!m_cafbcrElection && m_ipv4) {
    Ptr<Node> node = m_ipv4->GetObject<Node>();
    if (node) {
      m_cafbcrElection = node->GetObject<cafbcrElection>();
      if (!m_cafbcrElection) {
        NS_LOG_WARN("cafbcrElection not found on node, creating new one");
        m_cafbcrElection = CreateObject<cafbcrElection>();
        node->AggregateObject(m_cafbcrElection);
      } else {
        NS_LOG_DEBUG("Found cafbcrElection on node " << node->GetId());
      }
    }
  }

  // 用 ClusterNode 上注入的 per-node maxRange 覆盖 helper 注入的全局值。
  // 异构 FANET 下 m_maxRange 的语义就是"本节点自己的有效信号传输范围"，
  // LHTP / fuzzy LinkLifetime / EstimateLinkRate 都以此为准计算"我视角"
  // 下的链路寿命。同构场景下两个值相等，行为不变。
  // 时机说明：scratch 的 ClusterNode 同步循环在 internet.Install 之后执行，
  // 比 SetIpv4 晚但比 DoInitialize 早；所以这里读取能拿到 per-type 值。
  if (m_ipv4) {
    Ptr<Node> node = m_ipv4->GetObject<Node>();
    if (node) {
      Ptr<ClusterNode> clusterNode = node->GetObject<ClusterNode>();
      if (clusterNode) {
        double perNodeRange = clusterNode->GetMaxRange();
        if (perNodeRange > 0.0) {
          m_maxRange = perNodeRange;
          NS_LOG_DEBUG("Node " << node->GetId() << ": m_maxRange = "
                       << perNodeRange << "m (from ClusterNode)");
        }
      }
    }
  }

  // 初始化Hello管理器
  if (m_ipv4) {
    Ptr<Node> node = m_ipv4->GetObject<Node>();
    if (node) {
      uint16_t nodeId = node->GetId();
      if (m_fuzzyHello) {
        m_fuzzyHello->SetNodeId(nodeId);
        m_fuzzyHello->SetMaxRange(m_maxRange);
      }
      if (m_qHello) {
        m_qHello->SetNodeId(nodeId);
        m_qHello->SetMaxRange(m_maxRange);
      }
    }
  }

  Ipv4RoutingProtocol::DoInitialize();
}

void RoutingProtocol::DoDispose()
{
  NS_LOG_FUNCTION(this);
  m_linkProbeStarted=false;
  ClearLinkProbes();
  ClearPendingRenewals();
  SetTerminateRepeatedForward(false);
  SetAlternateRepeatedForward(false);
  
  m_ipv4 = nullptr;
  m_cafbcrElection = nullptr;
  m_neighbors = nullptr;
  
  for (auto iter = m_socketAddrs.begin(); iter != m_socketAddrs.end(); ++iter) {
    iter->first->Close();
  }
  m_socketAddrs.clear();
  
  Ipv4RoutingProtocol::DoDispose();
}

void RoutingProtocol::SetIpv4(Ptr<Ipv4> ipv4)
{
  NS_LOG_FUNCTION(this << ipv4);
  
  NS_ASSERT(ipv4 != nullptr);
  NS_ASSERT(m_ipv4 == nullptr);
  
  m_ipv4 = ipv4;
  
  // 获取ClusterNode和Neighbors指针
  Ptr<Node> node = m_ipv4->GetObject<Node>();
  if (!node) {
    NS_LOG_ERROR("Failed to get Node object from Ipv4");
    return;
  }

  Ptr<ClusterNode> clusterNode = node->GetObject<ClusterNode>();
  if (!clusterNode) {
    NS_LOG_ERROR("Node " << node->GetId() << ": ClusterNode not found! "
                 "Make sure ClusterNode is aggregated before routing installation.");
    return;
  }

  m_neighbors = clusterNode->GetObject<Neighbors>();
  if (!m_neighbors) {
    NS_LOG_ERROR("Node " << node->GetId() << ": Neighbors object not found on ClusterNode!");
    // 创建一个默认的Neighbors对象作为备用
    m_neighbors = CreateObject<Neighbors>(node->GetId());
    clusterNode->AggregateObject(m_neighbors);
    NS_LOG_WARN("Created fallback Neighbors for node " << node->GetId());
  } else {
    NS_LOG_DEBUG("Node " << node->GetId() << ": Successfully got Neighbors object");
  }

  m_neighbors->EnableComputeFreshness();

  // 初始化FuzzyHelloManager
  if (m_fuzzyHello && m_neighbors) {
    m_fuzzyHello->SetNodeId(node->GetId());
    m_fuzzyHello->SetNeighborTable(m_neighbors);
    m_fuzzyHello->SetMaxRange(m_maxRange);
    NS_LOG_DEBUG("Node " << node->GetId() << ": FuzzyHelloManager initialized");
  }

  // 初始化QLearningHelloManager
  if (m_qHello && m_neighbors) {
    m_qHello->SetNodeId(node->GetId());
    m_qHello->SetNeighborTable(m_neighbors);
    m_qHello->SetMaxRange(m_maxRange);
    if (m_helloMode == QLEARNING_HELLO) {
      // CA receive-age TTL is twice its base Hello interval (4s in the matrix),
      // independently of the chosen adaptive interval. Do not import FBCR's TTL.
      m_qHello->SetIntervalLimit(2 * m_helloInterval - MilliSeconds(50));
    }
    NS_LOG_DEBUG("Node " << node->GetId() << ": QLearningHelloManager initialized");
  }
  
  NS_ASSERT(m_ipv4->GetNInterfaces() == 1 && 
            m_ipv4->GetAddress(0, 0).GetLocal() == Ipv4Address("127.0.0.1"));
  
  m_lo = m_ipv4->GetNetDevice(0);
  NS_ASSERT(m_lo != nullptr);
  
  Simulator::ScheduleNow(&RoutingProtocol::Start, this);
}

void RoutingProtocol::Start()
{
  NS_LOG_FUNCTION(this);

  if (!m_uniformRandomVariable) {
    NS_LOG_ERROR("m_uniformRandomVariable not initialized");
    return;
  }

  if (m_socketAddrs.empty()) {
    NS_LOG_ERROR("No sockets available, cannot start routing");
    return;
  }

  NS_LOG_INFO("CAFBCR routing starting for node " << MyNodeId() << " with " << m_socketAddrs.size() << " sockets");

  // 调度Hello定时器
  m_helloTimer.SetFunction(&RoutingProtocol::HelloTimerExpire, this);
  double helloDelay = m_uniformRandomVariable->GetValue(0.0, 0.1);
  NS_LOG_DEBUG("Scheduling Hello timer for node " << MyNodeId() << " at delay " << helloDelay);
  m_helloTimer.Schedule(Seconds(helloDelay));

  // 调度分簇定时器
  m_clusteringTimer.SetFunction(&RoutingProtocol::ClusteringTimerExpire, this);
  NS_LOG_DEBUG("Scheduling Clustering timer for node " << MyNodeId() << " at delay " << m_clusteringTime);
  m_clusteringTimer.Schedule(m_clusteringTime);
  if (m_linkProbes) {
    m_linkProbeStarted=true;
    m_linkProbeTimer=Simulator::Schedule(MilliSeconds(200+MyNodeId()%1000),&RoutingProtocol::LinkProbeTick,this);
  }

  NS_LOG_INFO("CAFBCR routing started for node " << MyNodeId());
}

void RoutingProtocol::NotifyInterfaceUp(uint32_t interface)
{
  NS_LOG_FUNCTION(this << interface);
  
  Ptr<Ipv4L3Protocol> l3 = m_ipv4->GetObject<Ipv4L3Protocol>();
  if (l3->GetNAddresses(interface) > 1) {
    NS_LOG_WARN("cafbcr does not work with more than one address per interface");
  }
  
  Ipv4InterfaceAddress iface = l3->GetAddress(interface, 0);
  if (iface.GetLocal() == Ipv4Address("127.0.0.1")) {
    return;
  }
  
  m_mainAddress = iface.GetLocal();

  // 创建监听socket
  Ptr<Node> node = GetObject<Node>();
  if (!node) {
    NS_LOG_ERROR("Failed to get Node object for socket creation");
    return;
  }

  Ptr<Socket> socket = Socket::CreateSocket(node, UdpSocketFactory::GetTypeId());
  if (!socket) {
    NS_LOG_ERROR("Failed to create socket");
    return;
  }

  socket->SetRecvCallback(MakeCallback(&RoutingProtocol::Recvcafbcr, this));
  // 先Bind再BindToNetDevice
  int ret = socket->Bind(InetSocketAddress(Ipv4Address::GetAny(), CAFBCR_PORT));
  NS_LOG_DEBUG("Socket Bind returned: " << ret);
  socket->BindToNetDevice(l3->GetNetDevice(interface));
  socket->SetAllowBroadcast(true);
  socket->SetIpTtl(1);
  m_socketAddrs.insert(std::make_pair(socket, iface));
  if (m_linkProbes && m_linkProbeStarted && !m_linkProbeTimer.IsRunning())
    m_linkProbeTimer=Simulator::Schedule(MilliSeconds(200+MyNodeId()%1000),&RoutingProtocol::LinkProbeTick,this);
  
  // 监听链路层错误
  Ptr<NetDevice> dev = m_ipv4->GetNetDevice(m_ipv4->GetInterfaceForAddress(iface.GetLocal()));
  Ptr<WifiNetDevice> wifi = dev->GetObject<WifiNetDevice>();
  if (wifi != nullptr) {
    Ptr<WifiMac> mac = wifi->GetMac();
    if (mac != nullptr) {
      mac->TraceConnectWithoutContext("DroppedMpdu", 
                                      MakeCallback(&RoutingProtocol::NotifyTxError, this));
    }
  }
}

void RoutingProtocol::NotifyInterfaceDown(uint32_t interface)
{
  NS_LOG_FUNCTION(this << interface);
  
  Ptr<Socket> socket = FindSocketWithInterfaceAddress(m_ipv4->GetAddress(interface, 0));
  if (socket) {
    socket->Close();
    m_socketAddrs.erase(socket);
  }
  
  if (m_socketAddrs.empty()) {
    NS_LOG_ERROR("No cafbcr interfaces");
    ClearLinkProbes();
    if (m_neighbors) {
      m_neighbors->Clear();
    }
    m_helloTimer.Cancel();
    m_beaconTimer.Cancel();
  }
}

void RoutingProtocol::NotifyAddAddress(uint32_t interface, Ipv4InterfaceAddress address)
{
  NS_LOG_FUNCTION(this << interface << address);
  
  Ptr<Ipv4L3Protocol> l3 = m_ipv4->GetObject<Ipv4L3Protocol>();
  if (!l3->IsUp(interface)) {
    return;
  }
  
  if (l3->GetNAddresses(interface) == 1) {
    Ipv4InterfaceAddress iface = l3->GetAddress(interface, 0);
    Ptr<Socket> socket = FindSocketWithInterfaceAddress(iface);
    
    if (!socket) {
      if (iface.GetLocal() == Ipv4Address("127.0.0.1")) {
        return;
      }
      
      socket = Socket::CreateSocket(GetObject<Node>(), UdpSocketFactory::GetTypeId());
      NS_ASSERT(socket != nullptr);
      socket->SetRecvCallback(MakeCallback(&RoutingProtocol::Recvcafbcr, this));
      socket->Bind(InetSocketAddress(Ipv4Address::GetAny(), CAFBCR_PORT));
      socket->BindToNetDevice(l3->GetNetDevice(interface));
      socket->SetAllowBroadcast(true);
      m_socketAddrs.insert(std::make_pair(socket, iface));
    }
  }
}

void RoutingProtocol::NotifyRemoveAddress(uint32_t interface, Ipv4InterfaceAddress address)
{
  NS_LOG_FUNCTION(this << interface << address);
  
  Ptr<Socket> socket = FindSocketWithInterfaceAddress(address);
  if (socket) {
    socket->Close();
    m_socketAddrs.erase(socket);
    
    if (m_socketAddrs.empty()) {
      NS_LOG_ERROR("No cafbcr interfaces");
      if (m_neighbors) {
        m_neighbors->Clear();
      }
      m_helloTimer.Cancel();
      m_beaconTimer.Cancel();
    }
  }
}

void RoutingProtocol::PrintRoutingTable(Ptr<OutputStreamWrapper> stream, Time::Unit unit) const
{
  NS_LOG_FUNCTION(this << stream);
  *stream->GetStream() << "cafbcr Routing Table - Node " << MyNodeId() << std::endl;
}

//-----------------------------------------------------------------------------
// 定时器处理
//-----------------------------------------------------------------------------
void RoutingProtocol::HelloTimerExpire()
{
  NS_LOG_FUNCTION(this << "at time " << Simulator::Now().GetSeconds());

  if (m_ipv4 == nullptr) {
    NS_LOG_ERROR("m_ipv4 is null");
    return;
  }

  if (m_socketAddrs.empty()) {
    NS_LOG_DEBUG("No sockets available at time " << Simulator::Now().GetSeconds());
    return;
  }

  NS_LOG_DEBUG("HelloTimerExpire: About to call UpdateNodeFitness");
  double fitness = UpdateNodeFitness();
  NS_LOG_DEBUG("HelloTimerExpire: UpdateNodeFitness returned fitness=" << fitness);

  NS_LOG_DEBUG("HelloTimerExpire: About to call SendHello");
  SendHello();
  NS_LOG_DEBUG("HelloTimerExpire: SendHello completed");

  // 根据Hello模式选择间隔
  Time nextInterval = m_helloInterval;
  if (m_helloMode == FUZZY_HELLO && m_fuzzyHello) {
    nextInterval = m_fuzzyHello->GetOptimalHelloInterval();
    NS_LOG_DEBUG("Node " << MyNodeId() << " using Fuzzy Hello interval: " << nextInterval.GetSeconds() << "s");
  } else if (m_helloMode == QLEARNING_HELLO && m_qHello) {
    nextInterval = m_qHello->GetOptimalHelloInterval();
    NS_LOG_DEBUG("Node " << MyNodeId() << " using Q-Learning Hello interval: " << nextInterval.GetSeconds() << "s");
  } else {
    NS_LOG_DEBUG("Node " << MyNodeId() << " using Fixed Hello interval: " << nextInterval.GetSeconds() << "s");
  }

  m_helloTimer.Cancel();
  m_helloTimer.Schedule(nextInterval);
}

void RoutingProtocol::ClusteringTimerExpire()
{
  NS_LOG_FUNCTION(this << "at time " << Simulator::Now().GetSeconds());

  if (!m_ipv4) {
    NS_LOG_ERROR("m_ipv4 is null");
    return;
  }

  Ptr<Node> node = m_ipv4->GetObject<Node>();
  if (!node) {
    NS_LOG_ERROR("node pointer null");
    return;
  }

  Ptr<ClusterNode> clusterNode = node->GetObject<ClusterNode>();
  if (!clusterNode) {
    NS_LOG_ERROR("ClusterNode ptr null");
    return;
  }

  // 更新能量和邻居度
  Ptr<EnergySourceContainer> sources = node->GetObject<EnergySourceContainer>();
  if (sources && sources->GetN() > 0) {
    clusterNode->updateResidualEnergy(sources->Get(0)->GetRemainingEnergy());
  }  
  clusterNode->UpdateNeighborDegree();
  double fitness = UpdateNodeFitness();  // 计算Fitness
  NS_ASSERT_MSG(fitness <= 1.0, "Invalid fitness value");
  // 执行cafbcr选举
  if (m_cafbcrElection) { m_cafbcrElection->StartElection(); }

  // 如果被选为簇头候选，延迟广播CH_CALL
  if (clusterNode->GetRoleType() == BackupHead) {
    NS_LOG_INFO("Elected as BackupHead, will broadcast CH_CALL");
    double interval = 0.2 * std::sqrt(1.0 - fitness); // 根据fitness调整延迟
    Simulator::Schedule(Seconds(interval), &RoutingProtocol::SendClusterHeadAnnounceMent, this);
  }
  // 如果是Idle节点，给一定时间尝试加入簇，否则自建孤立簇
  else if (clusterNode->GetRoleType() == Idle) {
    // 延迟2.5秒后检查：要么加入了簇，要么创建孤立簇
    Simulator::Schedule(Seconds(2.5), &RoutingProtocol::ProactivelyJoinCluster, this);
  }

  // 调度簇维护检查
  Simulator::Schedule(Seconds(5.0), &ClusterManager::CheckClusteringStatus, m_clusterManager, MyNodeId());
}

void RoutingProtocol::BeaconTimerExpire()
{
  NS_LOG_FUNCTION(this);
  
  Ptr<Node> node = m_ipv4->GetObject<Node>();
  Ptr<ClusterNode> clusterNode = node->GetObject<ClusterNode>();
  
  NS_ASSERT_MSG(clusterNode,"clusternode ptr ins null. ");
  
  // 更新能量和fitness
  Ptr<EnergySourceContainer> sources = node->GetObject<EnergySourceContainer>();
  if (sources && sources->GetN() > 0) {
    clusterNode->updateResidualEnergy(sources->Get(0)->GetRemainingEnergy());
  }
  clusterNode->UpdateNeighborDegree();
  UpdateNodeFitness();
  
  NodeRole role = clusterNode->GetRoleType();

  if (role == Head) {
    // 簇头维护簇成员
    uint16_t clusterId = clusterNode->GetClusterId();
    Ptr<Cluster> cluster = CurrentOwnedCluster(m_clusterManager, clusterNode);

    if (cluster == nullptr) {
      // Do not park an orphaned head in Isolate: every join path accepts only
      // Idle, so an Isolate node would never rejoin and would become a zombie.
      // Isolate carries no protective protocol semantics here; reset to Idle
      // so the periodic join paths can adopt the node again.
      NS_LOG_WARN("Cluster missing for CH " << MyNodeId() << ", reset to Idle and rejoin");
      TraceJoin("head_cluster_invalid", MyNodeId(), MyNodeId(), clusterId);
      clusterNode->SetClusterId(0xFFFF);
      clusterNode->SetRole(Idle);
      if (!m_isRejoinScheduled) {
        m_isRejoinScheduled = true;
        Simulator::Schedule(Seconds(1.0),
                            &RoutingProtocol::RejoinClusterAfterTimeout, this);
      }
      return;
    }

    // 发送CH_LIVE信标
    SendClusterHeadBeacon();

    // 检查成员超时(超时时间设为5个beacon周期,给更多时间)
    std::vector<uint16_t> expiredMembers;
    auto members = cluster->GetAllMembers();

    for (auto it = members.begin(); it != members.end(); ++it) {
      uint16_t memberId = it->second->GetNodeId();
      if (memberId != MyNodeId()) {
        double lastUpdate = cluster->GetMemberUpdateTime(memberId).GetSeconds();
        double now = Simulator::Now().GetSeconds();
        double elapsed = now - lastUpdate;

        // 10 x beacon interval = 7.5 s. A 5 x beacon threshold (3.75 s) was too
        // strict on asymmetric links with CH_LIVE broadcast loss: members
        // repeatedly timed out, left and rejoined. Ten consecutive CH_LIVE
        // losses (about 1e-6 probability) are required to evict a member.
        if (elapsed > 10.0 * m_beaconInterval.GetSeconds()) {
          expiredMembers.push_back(memberId);
          NS_LOG_INFO("CH " << MyNodeId() << " Member " << memberId
                      << " timeout (now=" << now << "s, lastUpdate=" << lastUpdate
                      << "s, elapsed=" << elapsed << "s, threshold="
                      << (10.0 * m_beaconInterval.GetSeconds()) << "s)");
        }
      }
    }

    // 移除超时成员
    for (auto id : expiredMembers) {
      Ptr<ClusterNode> memberNode = NodeList::GetNode(id)->GetObject<ClusterNode>();
      if (memberNode) {
        // This head owns its registration, not an association established
        // elsewhere since admission. Retire both old bookkeeping entries
        // without resetting the node's current role/ID or scheduling a rejoin.
        if (memberNode->GetClusterId() != clusterId) {
          cluster->RemoveMemberRecord(id);
          TraceJoin("stale_member_record_expired", id, MyNodeId(), clusterId);
          continue;
        }
        cluster->RemoveMember(memberNode);
        memberNode->SetRole(Idle);
        TraceJoin("member_expired_at_head", id, MyNodeId(), clusterId);
        Simulator::Schedule(MilliSeconds(20), &ClusterManager::CheckClusteringStatus,
                           m_clusterManager, id);
      }
    }

    // 检查簇是否为空(只剩簇头自己)
    auto remainingMembers = cluster->GetAllMembers();
    if (remainingMembers.size() <= 1) {  // 只有簇头自己
      // 检查邻居表中是否有有效邻居
      bool hasValidNeighbors = false;
      double now = Simulator::Now().GetSeconds();
      double neighborTimeout = 3.0 * m_helloInterval.GetSeconds();  // 3倍Hello间隔

      if (m_neighbors && !m_neighbors->IsEmpty()) {
        auto neighborTable = m_neighbors->GetAll();
        for (auto it = neighborTable.begin(); it != neighborTable.end(); ++it) {
          double elapsed = now - it->second.UpdateTime.GetSeconds();
          if (elapsed < neighborTimeout) {
            hasValidNeighbors = true;
            break;
          }
        }
      }

      if (hasValidNeighbors) {
        // 有有效邻居,说明附近有更优簇头,解散簇并加入其他簇
        NS_LOG_INFO("CH " << MyNodeId() << " cluster " << clusterId
                    << " is empty but has valid neighbors, dissolving and rejoining");

        // 移除簇(会自动记录簇解散)
        m_clusterManager->RemoveCluster(clusterId);

        // 簇头变为Idle
        clusterNode->SetRole(Idle);
        clusterNode->SetClusterId(0xFFFF);

        // 调度重新加入簇
        if (!m_isRejoinScheduled) {
          m_isRejoinScheduled = true;
          Simulator::Schedule(Seconds(1.0), &RoutingProtocol::RejoinClusterAfterTimeout, this);
        }
        return;  // 不再继续发送CH_LIVE
      } else {
        // 没有有效邻居,保持孤立簇头状态
        NS_LOG_INFO("CH " << MyNodeId() << " cluster " << clusterId
                    << " is empty with no valid neighbors, keeping as isolated cluster");
      }
    }
  }
  else if (role == Member) {
    // 成员节点定期发送CM_LIVE维持簇成员身份
    SendClusterMemberBeaconFor("periodic");

    // 检查簇头是否超时(通过邻居表中簇头的UpdateTime)
    uint16_t clusterId = clusterNode->GetClusterId();
    uint16_t chNodeId = m_clusterManager->GetClusterHeadId(clusterId);

    if (chNodeId != 0xFFFF && m_neighbors) {
      auto neighborTable = m_neighbors->GetAll();
      bool chFound = false;

      for (auto it = neighborTable.begin(); it != neighborTable.end(); ++it) {
        if (it->second.NodeId == chNodeId) {
          chFound = true;
          double lastUpdate = it->second.UpdateTime.GetSeconds();
          double now = Simulator::Now().GetSeconds();
          double elapsed = now - lastUpdate;
          double threshold = 10.0 * m_beaconInterval.GetSeconds();

          if (elapsed > threshold) {
            // 簇头超时,退出当前簇
            NS_LOG_INFO("Member " << MyNodeId() << " lost CH " << chNodeId
                        << " (elapsed=" << elapsed << "s > threshold=" << threshold
                        << "s), leaving cluster " << clusterId);

            // 从簇中移除自己
            Ptr<Cluster> cluster = m_clusterManager->GetCluster(clusterId);
            if (cluster) {
              cluster->RemoveMember(clusterNode);
            }

            // 变为Idle,停止发送CM_LIVE
            clusterNode->SetRole(Idle);
            clusterNode->SetClusterId(0xFFFF);

            // 调度重新加入簇(延迟1秒),避免重复调度
            if (!m_isRejoinScheduled) {
              m_isRejoinScheduled = true;
              Simulator::Schedule(Seconds(1.0), &RoutingProtocol::RejoinClusterAfterTimeout, this);
            }
          }
          break;
        }
      }

      if (!chFound) {
        // 邻居表中找不到簇头(可能移出通信范围)
        NS_LOG_INFO("Member " << MyNodeId() << " can't find CH " << chNodeId
                    << " in neighbor table, leaving cluster " << clusterId);

        Ptr<Cluster> cluster = m_clusterManager->GetCluster(clusterId);
        if (cluster) {
          cluster->RemoveMember(clusterNode);
        }

        clusterNode->SetRole(Idle);
        clusterNode->SetClusterId(0xFFFF);

        // 调度重新加入簇(延迟1秒),避免重复调度
        if (!m_isRejoinScheduled) {
          m_isRejoinScheduled = true;
          Simulator::Schedule(Seconds(1.0), &RoutingProtocol::RejoinClusterAfterTimeout, this);
        }
      }
    }
  }

  m_beaconTimer.Cancel();
  m_beaconTimer.Schedule(m_beaconInterval);
}

//-----------------------------------------------------------------------------
// 消息发送
//-----------------------------------------------------------------------------
void RoutingProtocol::SendHello()
{
  NS_LOG_FUNCTION(this);
  
  Ptr<Node> node = m_ipv4->GetObject<Node>();
  Ptr<ClusterNode> clusterNode = node->GetObject<ClusterNode>();
  
  if (!clusterNode) return;
  
  Vector pos = GetMyPosition();
  Vector vel = GetVelocity();
  double energy = GetResidualEnergy();
  double fitness = clusterNode->GetFitnessValue();
  uint16_t clusterId = clusterNode->GetClusterId();
  NodeRole role = clusterNode->GetRoleType();
  
  HelloHeader helloHeader(MyNodeId(), clusterId, m_mainAddress, pos, vel,
                          energy, fitness, role, Simulator::Now().GetSeconds());
  // 填充计算能力字段
  helloHeader.SetComputeCapacity(clusterNode->GetComputeCapacity());
  helloHeader.SetCurrentLoad(static_cast<float>(clusterNode->GetCurrentLoad()));
  // [E2 piggyback] 携带 join intent：如果本节点是 Idle 且想加入某个 CH，
  // hello broadcast 让能直达该 CH 的中继节点听到后代理发 PROXY_JOIN_REQ
  helloHeader.SetIntentJoinChId(IsProxyJoinEnabled() && role == Idle ? m_intentJoinChId : 0xFFFF);
  if (m_proxyMembership && (role != Member || m_proxyMembership->second != clusterId))
    m_proxyMembership.reset();
  if (IsProxyJoinEnabled() && m_enableProxyRenewal && role == Member && m_proxyMembership &&
      !HasDirectMaintenanceCoverage())
    helloHeader.SetIntentJoinChId(m_proxyMembership->first);
  
  Ptr<Packet> packet = Create<Packet>();
  packet->AddHeader(helloHeader);
  
  TypeHeader typeHeader(CAFBCR_HELLO);
  packet->AddHeader(typeHeader);
  
  // 广播Hello消息
  for (auto it = m_socketAddrs.begin(); it != m_socketAddrs.end(); ++it) {
    Ptr<Socket> socket = it->first;
    Ipv4InterfaceAddress iface = it->second;

    Ipv4Address dest = iface.GetBroadcast();
    if (iface.GetMask() == Ipv4Mask::GetOnes()) {
      dest = Ipv4Address("255.255.255.255");
    }

    NS_LOG_DEBUG("Attempting to send Hello to " << dest << " from " << iface.GetLocal());
    int ret = SendControlPacket(socket,packet->Copy(),dest);
    if (helloHeader.GetIntentJoinChId() != 0xFFFF) {
      TraceJoin("intent_tx_attempt", MyNodeId(), helloHeader.GetIntentJoinChId(), 0xFFFF, dest, ret);
      if (role == Member)
        TraceProxyRenewal("source_hello_tx_attempt", MyNodeId(), helloHeader.GetIntentJoinChId(),
          clusterId, dest, packet->GetUid(), helloHeader.GetTimeStamp(), std::nullopt,
          packet->GetUid(), ret);
    }
    m_helloPacketCount++;
    m_ControlPacketSize += packet->GetSize();
    /*if (ret < 0) {
      Socket::SocketErrno err = socket->GetErrno();
      const char* errName = "UNKNOWN";
      switch(err) {
        case Socket::ERROR_NOTERROR: errName = "NOTERROR"; break;
        case Socket::ERROR_ISCONN: errName = "ISCONN"; break;
        case Socket::ERROR_NOTCONN: errName = "NOTCONN"; break;
        case Socket::ERROR_MSGSIZE: errName = "MSGSIZE"; break;
        case Socket::ERROR_AGAIN: errName = "AGAIN"; break;
        case Socket::ERROR_SHUTDOWN: errName = "SHUTDOWN"; break;
        case Socket::ERROR_OPNOTSUPP: errName = "OPNOTSUPP"; break;
        case Socket::ERROR_AFNOSUPPORT: errName = "AFNOSUPPORT"; break;
        case Socket::ERROR_INVAL: errName = "INVAL"; break;
        case Socket::ERROR_BADF: errName = "BADF"; break;
        case Socket::ERROR_NOROUTETOHOST: errName = "NOROUTETOHOST"; break;
        case Socket::ERROR_NODEV: errName = "NODEV"; break;
        case Socket::ERROR_ADDRNOTAVAIL: errName = "ADDRNOTAVAIL"; break;
        case Socket::ERROR_ADDRINUSE: errName = "ADDRINUSE"; break;
        default: break;
      }
      NS_LOG_ERROR("Failed to send Hello packet to " << dest << ", error: " << err << " (" << errName << ")");
    } else {
      NS_LOG_DEBUG("Successfully sent Hello to " << dest << ", size: " << ret);
      m_helloPacketCount++;
    }*/
  }

  NS_LOG_DEBUG("Sent Hello from Node " << MyNodeId() << " with Fitness " << fitness);
  if (!m_socketAddrs.empty()) m_discoveryHelloTx(DiscoveryHelloTxEvent{MyNodeId(), Simulator::Now()});
}

void RoutingProtocol::SendClusterHeadAnnounceMent()
{
  NS_LOG_FUNCTION(this);
  
  Ptr<Node> node = m_ipv4->GetObject<Node>();
  Ptr<ClusterNode> clusterNode = node->GetObject<ClusterNode>();
  
  if (!clusterNode || clusterNode->GetRoleType() != BackupHead) {
    return;
  }
  
  Vector pos = GetMyPosition();
  Vector vel = GetVelocity();
  double energy = GetResidualEnergy();
  double fitness = clusterNode->GetFitnessValue();
  uint16_t clusterId = clusterNode->GetClusterId();
  
  ClusterHeadAnnounceHeader chCallHeader(MyNodeId(), clusterId, m_mainAddress,
                                         pos, vel, energy, fitness,
                                         Simulator::Now().GetSeconds());
  
  Ptr<Packet> packet = Create<Packet>();
  packet->AddHeader(chCallHeader);
  
  TypeHeader typeHeader(CAFBCR_CH_CALL);
  packet->AddHeader(typeHeader);
  
  // 广播CH_CALL
  for (auto it = m_socketAddrs.begin(); it != m_socketAddrs.end(); ++it) {
    Ptr<Socket> socket = it->first;
    Ipv4InterfaceAddress iface = it->second;

    Ipv4Address dest;
    if (iface.GetMask() == Ipv4Mask::GetOnes()) {
      dest = Ipv4Address("255.255.255.255");
    } else {
      dest = iface.GetBroadcast();
    }

    SendControlPacket(socket,packet->Copy(),dest);
    m_chCallPacketCount++;
    m_ControlPacketSize += packet -> GetSize();
    // 调度超时检查（1.5秒后检查是否有申请者）
    Simulator::Schedule(Seconds(1.5), &RoutingProtocol::CheckPacketStatus,
                       this, socket, std::string("CH_CALL"), dest);
  }

  NS_LOG_INFO("Node " << MyNodeId() << " sent CH_CALL with Fitness " << fitness);
}

void RoutingProtocol::SendClusterJoinRequest(Ipv4Address chAddr)
{
  NS_LOG_FUNCTION(this << chAddr);
  
  Ptr<Node> node = m_ipv4->GetObject<Node>();
  Ptr<ClusterNode> clusterNode = node->GetObject<ClusterNode>();
  
  if (!clusterNode) return;
  
  Vector pos = GetMyPosition();
  Vector vel = GetVelocity();
  double energy = GetResidualEnergy();
  double fitness = clusterNode->GetFitnessValue();
  NodeRole role = clusterNode->GetRoleType();
  
  JoinClusterRequestHeader joinReqHeader(MyNodeId(), m_mainAddress, pos, vel,
                                         energy, fitness, role,
                                         Simulator::Now().GetSeconds());
  
  Ptr<Packet> packet = Create<Packet>();
  packet->AddHeader(joinReqHeader);
  
  TypeHeader typeHeader(CAFBCR_JOIN_REQ);
  packet->AddHeader(typeHeader);
  
  // 单播到簇头
  for (auto it = m_socketAddrs.begin(); it != m_socketAddrs.end(); ++it) {
    Ptr<Socket> socket = it->first;
    int ret = SendControlPacket(socket,packet,chAddr);
    TraceJoin("ordinary_request_tx_attempt", MyNodeId(), 0xFFFF, 0xFFFF, chAddr, ret);
    m_joinReqPacketCount++;
    m_ControlPacketSize += packet -> GetSize();
    // 调度超时检查（1.5秒后检查是否收到 JOIN_RESP）
    Simulator::Schedule(Seconds(1.5), &RoutingProtocol::CheckPacketStatus,
                       this, socket, std::string("JOIN_REQ"), chAddr);
    break;
  }

  NS_LOG_INFO("Node " << MyNodeId() << " sent JOIN_REQ to " << chAddr);
}

void RoutingProtocol::SendClusterJoinResponse(Ipv4Address memberAddr)
{
  NS_LOG_FUNCTION(this << memberAddr);
  
  Ptr<Node> node = m_ipv4->GetObject<Node>();
  Ptr<ClusterNode> clusterNode = node->GetObject<ClusterNode>();
  
  if (!clusterNode) return;
  
  uint16_t clusterId = clusterNode->GetClusterId();
  
  ClusterHeadResponseHeader joinRespHeader(MyNodeId(), clusterId, m_mainAddress,
                                           Simulator::Now().GetSeconds());
  
  Ptr<Packet> packet = Create<Packet>();
  packet->AddHeader(joinRespHeader);
  
  TypeHeader typeHeader(CAFBCR_JOIN_RESP);
  packet->AddHeader(typeHeader);
  
  // 单播到成员
  for (auto it = m_socketAddrs.begin(); it != m_socketAddrs.end(); ++it) {
    Ptr<Socket> socket = it->first;
    int ret = SendControlPacket(socket,packet,memberAddr);
    // The destination IP is known locally; do not use NodeList just to enrich a trace.
    TraceJoin("ordinary_response_tx_attempt", 0xFFFF, MyNodeId(), clusterId, memberAddr, ret);
    m_joinRespPacketCount++;
    m_ControlPacketSize += packet -> GetSize();
    break;
  }
  
  NS_LOG_INFO("Node " << MyNodeId() << " sent JOIN_RESP to " << memberAddr);
}

void RoutingProtocol::SendProxyJoinRequest(uint16_t originNodeId, Ipv4Address originIp,
                                           Vector originPos, Vector originVel,
                                           double originEnergy, double originFitness,
                                           Ipv4Address chAddr)
{
  NS_LOG_FUNCTION(this << originNodeId << chAddr);

  if (!IsProxyJoinEnabled()) {
    TraceJoin("proxy_tx_disabled", originNodeId, 0xFFFF, 0xFFFF, chAddr);
    return;
  }
  bool qualified = false;
  if (m_neighbors) {
    for (const auto& [id, info] : m_neighbors->PeekAll()) {
      if (info.ipAddress == chAddr &&
          info.UpdateTime + info.NeighborLifeLength > Simulator::Now() &&
          WithinConfiguredRange(info.Position)) {
        qualified = true;
        break;
      }
    }
  }
  if (!qualified) {
    TraceJoin("proxy_tx_unqualified", originNodeId, 0xFFFF, 0xFFFF, chAddr);
    return;
  }

  // 防重：1 秒内不重复为同一 origin 发 PROXY_JOIN_REQ（多个邻居都听到 hello.intent）
  Time now = Simulator::Now();
  auto pit = m_proxyJoinSentAt.find(originNodeId);
  if (pit != m_proxyJoinSentAt.end() && (now - pit->second) < Seconds(1.0)) {
    NS_LOG_DEBUG("Skip duplicate proxy for origin " << originNodeId);
    TraceJoin("proxy_suppressed_duplicate", originNodeId, 0xFFFF, 0xFFFF, chAddr);
    return;
  }
  m_proxyJoinSentAt[originNodeId] = now;

  // 复用 JoinClusterRequestHeader：填 origin 信息（不是本节点信息）。
  // CH 端 RecvProxyJoinRequest 把 originNodeId/originIp 当作真实成员加入。
  JoinClusterRequestHeader joinReqHeader(originNodeId, originIp, originPos, originVel,
                                         originEnergy, originFitness, Idle,
                                         now.GetSeconds());
  Ptr<Packet> packet = Create<Packet>();
  packet->AddHeader(joinReqHeader);
  TypeHeader typeHeader(CAFBCR_PROXY_JOIN_REQ);
  packet->AddHeader(typeHeader);

  for (auto it = m_socketAddrs.begin(); it != m_socketAddrs.end(); ++it) {
    Ptr<Socket> socket = it->first;
    int ret = SendControlPacket(socket,packet,chAddr);
    TraceJoin("proxy_request_tx_attempt", originNodeId, 0xFFFF, 0xFFFF, chAddr, ret);
    m_joinReqPacketCount++;
    m_ControlPacketSize += packet->GetSize();
    break;
  }
  NS_LOG_INFO("Node " << MyNodeId() << " sent PROXY_JOIN_REQ for origin "
              << originNodeId << " to CH " << chAddr);
}

bool RoutingProtocol::DirectMaintenanceEnabled() const
{
  return m_directMaintenanceFeedback && IsProxyJoinEnabled() && m_enableProxyRenewal;
}

bool RoutingProtocol::HasCurrentProxyMembership() const
{
  auto local = m_ipv4 ? m_ipv4->GetObject<Node>()->GetObject<ClusterNode>() : nullptr;
  return local && local->GetRoleType() == Member && m_proxyMembership &&
    local->GetClusterId() == m_proxyMembership->second;
}

void RoutingProtocol::ClearDirectMaintenanceCoverage()
{
  m_directMaintenanceFallback.Cancel();
  m_directMaintenanceCoverage.reset();
}

bool RoutingProtocol::HasDirectMaintenanceCoverage()
{
  if (!DirectMaintenanceEnabled() || !HasCurrentProxyMembership() ||
      !m_directMaintenanceCoverage ||
      RenewalAssociation{MyNodeId(), m_proxyMembership->first, m_proxyMembership->second} !=
        RenewalAssociation{std::get<0>(*m_directMaintenanceCoverage),
                           std::get<1>(*m_directMaintenanceCoverage),
                           std::get<2>(*m_directMaintenanceCoverage)} ||
      Simulator::Now() >= m_directMaintenanceCoverageUntil) {
    ClearDirectMaintenanceCoverage();
    return false;
  }
  return true;
}

bool RoutingProtocol::CanSuppressClusterMemberResponse(uint16_t head, uint16_t cluster,
                                                       Time membershipSince) const
{
  // Pure query: an expiry-time response must not cancel the separate HELLO
  // fallback event. Socket success is deliberately not evidence of coverage.
  if (!m_feedbackCoveredCmResponse || !DirectMaintenanceEnabled() ||
      !HasCurrentProxyMembership() || !m_directMaintenanceCoverage ||
      m_proxyMembership->first != head || m_proxyMembership->second != cluster ||
      m_proxyMembershipSince != membershipSince)
    return false;
  const auto& key = *m_directMaintenanceCoverage;
  return RenewalAssociation{MyNodeId(), head, cluster} ==
           RenewalAssociation{std::get<0>(key), std::get<1>(key), std::get<2>(key)} &&
    std::get<3>(key) >= membershipSince.GetMicroSeconds() &&
    MicroSeconds(std::get<3>(key)) <= Simulator::Now() &&
    Simulator::Now() < m_directMaintenanceCoverageUntil;
}

void RoutingProtocol::SendClusterMemberResponse(uint16_t head, uint16_t cluster,
                                                Time membershipSince)
{
  if (!m_ipv4) return;
  // Recheck at execution, after the original 0--50 ms jitter, because feedback
  // or the association may have expired since CH_LIVE was received.
  if (CanSuppressClusterMemberResponse(head, cluster, membershipSince)) {
    TraceJoin("cm_live_feedback_suppressed", MyNodeId(), head, cluster,
              Ipv4Address::GetAny(), std::nullopt, "ch-live-response");
    return;
  }
  SendClusterMemberBeaconFor("ch-live-response");
}

void RoutingProtocol::RecordDirectMaintenance(const ClusterMemberBeaconHeader& beacon, uint64_t packetUid)
{
  if (!DirectMaintenanceEnabled()) return;
  auto local = m_ipv4->GetObject<Node>()->GetObject<ClusterNode>();
  if (!local || local->GetRoleType() != Head) return;
  auto cluster = CurrentOwnedCluster(m_clusterManager, local);
  const auto stamp = beacon.GetReceivedTimeStampUs();
  const auto nowUs = Simulator::Now().GetMicroSeconds();
  const auto windowUs = (m_beaconInterval * 5).GetMicroSeconds();
  if (!cluster || cluster->GetClusterId() != beacon.GetClusterId() ||
      beacon.GetNodeId() == MyNodeId() || !cluster->ExistMember(beacon.GetNodeId()) ||
      !stamp || *stamp < 0 || *stamp > nowUs || windowUs <= 0 || nowUs - *stamp >= windowUs ||
      cluster->GetMemberUpdateTime(beacon.GetNodeId()) + m_beaconInterval * 10 < Simulator::Now())
    return;
  auto member = cluster->GetAllMembers().at(beacon.GetNodeId());
  if (!member || member->GetRoleType() != Member || member->GetClusterId() != beacon.GetClusterId()) return;
  const RenewalAssociation association{beacon.GetNodeId(), MyNodeId(), beacon.GetClusterId()};
  auto it = m_directMaintenanceRecords.find(association);
  if (it == m_directMaintenanceRecords.end() || *stamp > it->second.sourceUs)
    m_directMaintenanceRecords[association] = {*stamp, packetUid};
}

DirectMaintenanceHeader RoutingProtocol::SelectDirectMaintenanceRecords()
{
  const auto windowUs = (m_beaconInterval * 5).GetMicroSeconds();
  DirectMaintenanceHeader feedback(windowUs);
  if (!DirectMaintenanceEnabled() || windowUs <= 0) return feedback;
  auto local = m_ipv4->GetObject<Node>()->GetObject<ClusterNode>();
  if (!local || local->GetRoleType() != Head) {
    m_directMaintenanceRecords.clear(); m_directMaintenanceCursor.reset();
    return feedback;
  }
  auto cluster = CurrentOwnedCluster(m_clusterManager, local);
  const auto nowUs = Simulator::Now().GetMicroSeconds();
  for (auto it = m_directMaintenanceRecords.begin(); it != m_directMaintenanceRecords.end();) {
    const auto [origin, head, id] = it->first;
    bool valid = cluster && head == MyNodeId() && id == cluster->GetClusterId() &&
      cluster->ExistMember(origin) && it->second.sourceUs >= 0 && it->second.sourceUs <= nowUs &&
      nowUs - it->second.sourceUs < windowUs;
    if (valid) {
      auto member = cluster->GetAllMembers().at(origin);
      valid = member && member->GetRoleType() == Member && member->GetClusterId() == id;
    }
    if (!valid) it = m_directMaintenanceRecords.erase(it); else ++it;
  }
  if (m_directMaintenanceRecords.empty()) return feedback;
  auto it = m_directMaintenanceCursor ? m_directMaintenanceRecords.upper_bound(*m_directMaintenanceCursor) :
    m_directMaintenanceRecords.begin();
  const auto count = std::min<size_t>(m_directMaintenanceRecords.size(), DirectMaintenanceHeader::kMaxRecords);
  for (size_t i = 0; i < count; ++i) {
    if (it == m_directMaintenanceRecords.end()) it = m_directMaintenanceRecords.begin();
    feedback.AddRecord(std::get<0>(it->first), it->second.sourceUs);
    m_directMaintenanceCursor = it->first;
    ++it;
  }
  return feedback;
}

void RoutingProtocol::ApplyDirectMaintenance(const ClusterHeadBeaconHeader& head,
    const DirectMaintenanceHeader& feedback, Ipv4Address sourceIp, uint64_t packetUid)
{
  if (!DirectMaintenanceEnabled() || !HasCurrentProxyMembership() || !feedback.IsValid() ||
      head.GetChNodeId() != m_proxyMembership->first || head.GetClusterId() != m_proxyMembership->second ||
      sourceIp != head.GetChIpAddress()) return;
  const auto nowUs = Simulator::Now().GetMicroSeconds();
  // A shorter local beacon period can only shorten suppression. The CH's
  // advertised half-lease window always limits it, even with unequal periods.
  const auto windowUs = std::min(feedback.GetWindowUs(), (m_beaconInterval * 5).GetMicroSeconds());
  if (windowUs <= 0) return;
  for (const auto& record : feedback.GetRecords()) {
    if (record.origin != MyNodeId() || record.sourceUs < m_proxyMembershipSince.GetMicroSeconds() ||
        record.sourceUs < 0 || record.sourceUs > nowUs || nowUs - record.sourceUs >= windowUs ||
        (m_directMaintenanceCoverage && record.sourceUs <= std::get<3>(*m_directMaintenanceCoverage)))
      continue;
    const RenewalKey key{MyNodeId(), head.GetChNodeId(), head.GetClusterId(), record.sourceUs};
    ClearDirectMaintenanceCoverage();
    m_directMaintenanceCoverage = key;
    const auto remaining = MicroSeconds(windowUs - (nowUs - record.sourceUs));
    // Anchor to the received source timestamp, never to feedback arrival.
    m_directMaintenanceCoverageUntil = MicroSeconds(record.sourceUs) + MicroSeconds(windowUs);
    m_directMaintenanceFallback = Simulator::Schedule(
      m_directMaintenanceCoverageUntil - Simulator::Now(), &RoutingProtocol::ResumeProxyMaintenance, this, key);
    TraceRenewalCoordination("source_direct_feedback_rx", key, sourceIp, packetUid,
      std::nullopt, std::nullopt, std::nullopt, remaining);
  }
}

void RoutingProtocol::ResumeProxyMaintenance(RenewalKey key)
{
  if (!m_directMaintenanceCoverage || *m_directMaintenanceCoverage != key) return;
  const bool current = DirectMaintenanceEnabled() && HasCurrentProxyMembership() &&
    m_proxyMembership->first == std::get<1>(key) && m_proxyMembership->second == std::get<2>(key);
  ClearDirectMaintenanceCoverage();
  if (!current) return;
  TraceRenewalCoordination("source_proxy_resumed_no_feedback", key, Ipv4Address::GetAny(), 0);
  // Fuzzy/periodic HELLO may be later than the remaining lease. Send one real
  // fallback HELLO now without changing the normal HELLO schedule or CM sends.
  SendHello();
}

void RoutingProtocol::SendClusterHeadBeacon()
{
  NS_LOG_FUNCTION(this);
  
  Ptr<Node> node = m_ipv4->GetObject<Node>();
  Ptr<ClusterNode> clusterNode = node->GetObject<ClusterNode>();
  
  if (!clusterNode || clusterNode->GetRoleType() != Head) {
    return;
  }
  
  uint16_t clusterId = clusterNode->GetClusterId();
  Vector pos = GetMyPosition();
  Vector vel = GetVelocity();
  double fitness = clusterNode->GetFitnessValue();
  
  ClusterHeadBeaconHeader chBeaconHeader(MyNodeId(), clusterId, m_mainAddress,
                                         pos, vel, fitness,
                                         Simulator::Now().GetSeconds());
  // Grants describe outstanding local admissions, not an unbounded history of
  // origin IDs. BeaconTimerExpire sends before removing timed-out members, so
  // apply the same freshness predicate here before anything reaches the wire.
  if (IsProxyJoinEnabled()) {
    auto cluster = m_clusterManager->GetCluster(clusterId);
    for (auto it = m_recentGranted.begin(); it != m_recentGranted.end();) {
      bool admitted = cluster && cluster->GetClusterHeadId() == MyNodeId() && cluster->ExistMember(*it);
      bool expired = admitted &&
        (Simulator::Now().GetSeconds() - cluster->GetMemberUpdateTime(*it).GetSeconds() >
         10.0 * m_beaconInterval.GetSeconds());
      if (!admitted || expired) {
        TraceJoin(expired ? "grant_retired_expired" : "grant_retired_absent", *it, MyNodeId(), clusterId);
        it = m_recentGranted.erase(it);
      } else {
        ++it;
      }
    }
  }
  // [E2 piggyback] 把最近接受的成员 NodeId 列表广播出去；
  // 远 SWARM 听到自己 ID → 自确认入簇（无需 unicast JOIN_RESP）
  for (uint8_t i = 0; i < ClusterHeadBeaconHeader::kGrantedMembersCap; ++i) {
    uint16_t nid = (IsProxyJoinEnabled() && i < m_recentGranted.size()) ? m_recentGranted[i] : 0xFFFF;
    chBeaconHeader.SetGrantedMember(i, nid);
  }

  Ptr<Packet> packet = Create<Packet>();
  const auto directFeedback = SelectDirectMaintenanceRecords();
  if (!directFeedback.GetRecords().empty()) packet->AddHeader(directFeedback);
  packet->AddHeader(chBeaconHeader);

  TypeHeader typeHeader(CAFBCR_CH_LIVE);
  packet->AddHeader(typeHeader);
  
  // 广播CH_LIVE
  for (auto it = m_socketAddrs.begin(); it != m_socketAddrs.end(); ++it) {
    Ptr<Socket> socket = it->first;
    Ipv4InterfaceAddress iface = it->second;
    
    Ipv4Address dest = iface.GetBroadcast();
    int ret = SendControlPacket(socket,packet->Copy(),dest);
    for (const auto& record : directFeedback.GetRecords()) {
      const RenewalAssociation association{record.origin, MyNodeId(), clusterId};
      TraceRenewalCoordination("head_direct_feedback_tx", {record.origin, MyNodeId(), clusterId, record.sourceUs},
        dest, packet->GetUid(), std::nullopt, ret, m_directMaintenanceRecords.at(association).receivedUid);
    }
    if (IsProxyJoinEnabled()) {
      for (auto origin : m_recentGranted) {
        TraceJoin("grant_tx_attempt", origin, MyNodeId(), clusterId, dest, ret);
      }
    }
    m_chBeaconCount++;
    m_ControlPacketSize += packet -> GetSize();
  }
  
  NS_LOG_DEBUG("Node " << MyNodeId() << " sent CH_LIVE for cluster " << clusterId);
}

void RoutingProtocol::SendClusterMemberBeacon()
{
  SendClusterMemberBeaconFor("unspecified");
}

void RoutingProtocol::SendClusterMemberBeaconFor(const char* trigger)
{
  NS_LOG_FUNCTION(this);

  Ptr<Node> node = m_ipv4->GetObject<Node>();
  Ptr<ClusterNode> clusterNode = node->GetObject<ClusterNode>();

  if (!clusterNode) {
    NS_LOG_WARN("Node " << MyNodeId() << " SendCM_LIVE: clusterNode is NULL");
    return;
  }

  if (clusterNode->GetRoleType() != Member) {
    NS_LOG_INFO("Node " << MyNodeId() << " SendCM_LIVE: not Member (role=" << (int)clusterNode->GetRoleType() << ")");
    return;
  }

  uint16_t clusterId = clusterNode->GetClusterId();
  uint16_t chNodeId = m_clusterManager->GetClusterHeadId(clusterId);

  if (chNodeId == 0xFFFF) {
    NS_LOG_WARN("Node " << MyNodeId() << " SendCM_LIVE: No cluster head found for cluster " << clusterId);
    return;
  }
  
  Ptr<Node> chNode = NodeList::GetNode(chNodeId);
  Ptr<ClusterNode> chClusterNode = chNode->GetObject<ClusterNode>();
  Ipv4Address chAddr = chNode->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();
  
  Vector pos = GetMyPosition();
  Vector vel = GetVelocity();
  double fitness = clusterNode->GetFitnessValue();
  
  ClusterMemberBeaconHeader cmBeaconHeader(MyNodeId(), clusterId, m_mainAddress,
                                           pos, vel, fitness,
                                           Simulator::Now().GetSeconds());
  
  Ptr<Packet> packet = Create<Packet>();
  packet->AddHeader(cmBeaconHeader);
  
  TypeHeader typeHeader(CAFBCR_CM_LIVE);
  packet->AddHeader(typeHeader);
  
  // 单播到簇头
  for (auto it = m_socketAddrs.begin(); it != m_socketAddrs.end(); ++it) {
    Ptr<Socket> socket = it->first;
    int ret = SendControlPacket(socket,packet,chAddr);
    TraceJoin("cm_live_tx_attempt", MyNodeId(), chNodeId, clusterId, chAddr, ret, trigger);
    m_cmBeaconCount++;
    m_ControlPacketSize += packet -> GetSize();
    break;
  }

  NS_LOG_INFO("Member " << MyNodeId() << " sent CM_LIVE to CH " << chNodeId
              << " at time " << Simulator::Now().GetSeconds() << "s");
}

//-----------------------------------------------------------------------------
// 数据包接收
//-----------------------------------------------------------------------------
void RoutingProtocol::Recvcafbcr(Ptr<Socket> socket)
{
  NS_LOG_FUNCTION(this << socket);
  
  Address srcAddr;
  Ptr<Packet> packet = socket->RecvFrom(srcAddr);
  InetSocketAddress inetSrcAddr = InetSocketAddress::ConvertFrom(srcAddr);
  Ipv4Address sender = inetSrcAddr.GetIpv4();
  Ipv4Address receiver = m_socketAddrs[socket].GetLocal();
  
  TypeHeader typeHeader;
  packet->RemoveHeader(typeHeader);
  
  if (!typeHeader.IsValid()) {
    NS_LOG_WARN("Received invalid packet type");
    return;
  }
  
  switch (typeHeader.Get()) {
    case CAFBCR_HELLO:
      RecvHello(packet, receiver, sender);
      break;
    case CAFBCR_CH_CALL:
      RecvClusterHeadAnnouncement(packet, receiver, sender);
      break;
    case CAFBCR_JOIN_REQ:
      RecvJoinRequest(packet, receiver, sender);
      break;
    case CAFBCR_JOIN_RESP:
      RecvJoinResponse(packet, receiver, sender);
      break;
    case CAFBCR_CH_LIVE:
      RecvClusterHeadBeacon(packet, receiver, sender);
      break;
    case CAFBCR_CM_LIVE:
      RecvClusterMemberBeacon(packet, receiver, sender);
      break;
    case CAFBCR_PROXY_JOIN_REQ:
      RecvProxyJoinRequest(packet, receiver, sender);
      break;
    case CAFBCR_RENEWAL_ACK:
      RecvRenewalAck(packet, sender);
      break;
    case CAFBCR_LINK_REQUEST:
      RecvLinkProbe(packet,sender,false);
      break;
    case CAFBCR_LINK_REPLY:
      RecvLinkProbe(packet,sender,true);
      break;
    default:
      NS_LOG_WARN("Unknown packet type: " << (uint16_t)typeHeader.Get());
  }
}

void RoutingProtocol::UpdateNeighbor(Neighbors::NeighborInfo info, const char* message,
                                     bool carriesCompute, bool carriesEnergy)
{
  if (!m_neighbors) return;
  // Purge before merging so expired entries cannot supply omitted fields.
  // AddNeighbor also purges, but that would be too late for this merge.
  const auto& table = m_neighbors->GetAll();
  auto previous = table.find(info.NodeId);
  if (previous != table.end()) {
    if (!carriesCompute) {
      info.computeCapacity = previous->second.computeCapacity;
      info.currentLoad = previous->second.currentLoad;
    }
    if (!carriesEnergy) info.ResEnergy = previous->second.ResEnergy;
  }
  m_neighbors->AddNeighbor(info);
  if (carriesCompute) {
    // Receive-age contract, using the current CA base TTL captured at receipt.
    // Other messages may keep the neighbor alive but never extend this deadline.
    m_neighbors->RecordComputeAdvertisement(info.NodeId, info.NeighborLifeLength);
  }
  const auto observation = m_neighbors->GetComputeObservation(info.NodeId);
  m_neighborComputeEvent(NeighborComputeEvent{MyNodeId(), info.NodeId, info.ipAddress,
    Simulator::Now(), message, info.UpdateTime, info.NeighborLifeLength, info.ResEnergy,
    observation.receivedAt ? std::optional<double>(info.computeCapacity) : std::nullopt,
    observation.receivedAt ? std::optional<double>(info.currentLoad) : std::nullopt,
    observation});
}

void RoutingProtocol::RecvHello(Ptr<Packet> packet, Ipv4Address myIp, Ipv4Address srcIp)
{
  NS_LOG_FUNCTION(this << srcIp);

  if (!m_neighbors) {
    NS_LOG_ERROR("Neighbors object not initialized");
    return;
  }

  HelloHeader helloHeader;
  packet->RemoveHeader(helloHeader);

  NS_LOG_DEBUG("Received Hello from Node " << helloHeader.GetNodeId());

  // 计算LHTP并应用最小值
  Vector myPos = GetMyPosition();
  Vector myVel = GetVelocity();
  double lhtp = CalculateLHTP(myPos, myVel, helloHeader.GetPosition(),
                              helloHeader.GetVelocity());

  NS_LOG_DEBUG("Node " << MyNodeId() << " received Hello from " << helloHeader.GetNodeId()
               << ", LHTP=" << lhtp << "s, UpdateTime=" << Simulator::Now().GetSeconds());

  // 如果使用FuzzyHello模式，处理信标信息
  if (m_helloMode == FUZZY_HELLO && m_fuzzyHello) {
    m_fuzzyHello->ProcessHelloInfo(helloHeader.GetNodeId(),
                                    helloHeader.GetPosition(),
                                    helloHeader.GetVelocity(),
                                    Simulator::Now());
    NS_LOG_DEBUG("Node " << MyNodeId() << " processed Hello info for FuzzyHelloManager from node "
                 << helloHeader.GetNodeId());
  } else if (m_helloMode == QLEARNING_HELLO && m_qHello) {
    m_qHello->ProcessHelloInfo(helloHeader.GetNodeId(), helloHeader.GetPosition(),
                             helloHeader.GetVelocity(), Simulator::Now());
  }

  // 添加或更新邻居信息
  Neighbors::NeighborInfo neighborInfo(
    helloHeader.GetNodeId(),
    helloHeader.GetIpAddress(),
    helloHeader.GetClusterId(),
    helloHeader.GetPosition(),
    helloHeader.GetVelocity(),
    helloHeader.GetResidualEnergy(),
    helloHeader.GetFitness(),
    helloHeader.GetRole(),
    Simulator::Now(),
    Seconds(m_helloInterval.GetSeconds() * 2),
    helloHeader.GetComputeCapacity(),
    helloHeader.GetCurrentLoad()
  );

  UpdateNeighbor(neighborInfo, "HELLO", true, true);
  const auto discoveryObservation = m_neighbors->GetComputeObservation(helloHeader.GetNodeId());
  if (discoveryObservation.status == Neighbors::ComputeObservation::FRESH &&
      discoveryObservation.receivedAt && discoveryObservation.expiresAt)
    m_discoveryHelloRx(DiscoveryHelloRxEvent{MyNodeId(), helloHeader.GetNodeId(), helloHeader.GetIpAddress(),
      static_cast<uint8_t>(helloHeader.GetRole()), helloHeader.GetComputeCapacity(), helloHeader.GetCurrentLoad(),
      helloHeader.GetTimeStamp(), *discoveryObservation.receivedAt, *discoveryObservation.expiresAt});
  NS_LOG_DEBUG("Node " << MyNodeId() << " added neighbor " << helloHeader.GetNodeId()
               << ", now has " << m_neighbors->GetAll().size() << " neighbors");

  // [E2 piggyback] 看 hello.intentJoinChId：如果某邻居 Idle 想加入 CH-X，且本节点
  // 物理可达 CH-X（CH-X 是我邻居 + WithinMyTxRange），就代理发 PROXY_JOIN_REQ。
  // 这是异构非对称链路下让远 SWARM 入簇的关键路径——SWARM 自己回不去 CH，但
  // 它的某个 SWARM 邻居能直达 CH，由邻居代发。
  uint16_t intentCh = helloHeader.GetIntentJoinChId();
  if (intentCh != 0xFFFF && intentCh != MyNodeId()) {
    TraceJoin("intent_rx", helloHeader.GetNodeId(), intentCh, 0xFFFF, srcIp);
  }
  if (IsProxyJoinEnabled() && helloHeader.GetRole() == Idle && intentCh != 0xFFFF && intentCh != MyNodeId()) {
    // 我能直达该 CH？
    if (m_neighbors->IsNeighbor(intentCh) && WithinConfiguredRange(intentCh)) {
      const auto& tab = m_neighbors->GetAll();
      auto chIt = tab.find(intentCh);
      if (chIt != tab.end()) {
        Ipv4Address chIp = chIt->second.ipAddress;
        SendProxyJoinRequest(helloHeader.GetNodeId(), helloHeader.GetIpAddress(),
                             helloHeader.GetPosition(), helloHeader.GetVelocity(),
                             helloHeader.GetResidualEnergy(),
                             helloHeader.GetFitness(), chIp);
      }
    }
  }
  if (IsProxyJoinEnabled() && m_enableProxyRenewal && helloHeader.GetRole() == Member)
    RelayProxyMemberHello(helloHeader, srcIp, packet->GetUid());
}

void RoutingProtocol::RelayProxyMemberHello(const HelloHeader& hello, Ipv4Address srcIp,
                                          std::optional<uint64_t> helloUid, bool holdoffComplete)
{
  if (!m_ipv4 || !IsProxyJoinEnabled() || !m_enableProxyRenewal) return;
  const auto now = Simulator::Now();
  const auto head = hello.GetIntentJoinChId();
  if (!m_neighbors || head == 0xFFFF || head == MyNodeId() ||
      hello.GetNodeId() == MyNodeId() || hello.GetNodeId() == head ||
      hello.GetClusterId() == 0xFFFF || hello.GetIpAddress() != srcIp) return;
  const auto& neighbors = m_neighbors->PeekAll();
  auto ch = neighbors.find(head);
  if (ch == neighbors.end() || ch->second.Role != Head ||
      ch->second.clusterId != hello.GetClusterId() ||
      ch->second.UpdateTime + ch->second.NeighborLifeLength <= now ||
      !WithinConfiguredRange(ch->second.Position)) {
    TraceJoin("proxy_renewal_unqualified", hello.GetNodeId(), head, hello.GetClusterId(), srcIp);
    return;
  }
  const double stamp = hello.GetTimeStamp();
  if (!std::isfinite(stamp) || stamp < 0 || Seconds(stamp) > now) return;
  const auto observed = Seconds(stamp);
  // Bound duplicate state to recent observations; forwarding never refreshes
  // a source timestamp. One HELLO is sent at most once by this relay.
  const auto lifetime = m_beaconInterval * 10;
  for (auto it = m_proxyRenewalSentAt.begin(); it != m_proxyRenewalSentAt.end();) {
    if (it->second + lifetime < now) it = m_proxyRenewalSentAt.erase(it); else ++it;
  }
  if (observed + lifetime < now) return;
  auto previous = m_proxyRenewalSentAt.find(hello.GetNodeId());
  if (previous != m_proxyRenewalSentAt.end() && previous->second >= observed) return;
  const RenewalKey key{hello.GetNodeId(), head, hello.GetClusterId(),
    static_cast<int64_t>(stamp * 1000000)};
  if (m_coordinateProxyRenewal) {
    for (auto it=m_confirmedRenewals.begin(); it!=m_confirmedRenewals.end();) {
      if (MicroSeconds(it->second)+lifetime < now) it=m_confirmedRenewals.erase(it); else ++it;
    }
    auto confirmed=m_confirmedRenewals.find({hello.GetNodeId(),head,hello.GetClusterId()});
    if (confirmed!=m_confirmedRenewals.end() && confirmed->second>=std::get<3>(key)) {
      TraceRenewalCoordination("relay_cm_suppressed_feedback",key,srcIp,helloUid.value_or(0),helloUid);
      return;
    }
    if (!holdoffComplete) {
      if (m_pendingRenewals.find(key)!=m_pendingRenewals.end()) return;
      const auto delay=RenewalHoldoff(key);
      auto [it,inserted]=m_pendingRenewals.emplace(key,PendingRenewal{hello,srcIp,helloUid,EventId()});
      it->second.timer=Simulator::Schedule(delay,&RoutingProtocol::SendPendingRenewal,this,key);
      TraceRenewalCoordination("relay_cm_scheduled",key,srcIp,helloUid.value_or(0),helloUid,
        std::nullopt,std::nullopt,delay);
      return;
    }
  }
  ClusterMemberBeaconHeader beacon(hello.GetNodeId(), hello.GetClusterId(), hello.GetIpAddress(),
      hello.GetPosition(), hello.GetVelocity(), hello.GetFitness(), stamp);
  auto packet = Create<Packet>(); packet->AddHeader(beacon); packet->AddHeader(TypeHeader(CAFBCR_CM_LIVE));
  for (const auto& [socket, address] : m_socketAddrs) {
    const int result = SendControlPacket(socket,packet,ch->second.ipAddress);
    TraceJoin("proxy_cm_live_tx_attempt", hello.GetNodeId(), head, hello.GetClusterId(), ch->second.ipAddress, result);
    TraceProxyRenewal("relay_cm_tx_attempt", hello.GetNodeId(), head, hello.GetClusterId(),
      ch->second.ipAddress, packet->GetUid(), stamp, std::nullopt, helloUid, result);
    if (m_coordinateProxyRenewal)
      TraceRenewalCoordination("relay_cm_dispatched",key,ch->second.ipAddress,packet->GetUid(),helloUid,result);
    ++m_cmBeaconCount; m_ControlPacketSize += packet->GetSize();
    if (result >= 0) m_proxyRenewalSentAt[hello.GetNodeId()] = observed;
    break;
  }
}

Time RoutingProtocol::RenewalHoldoff(const RenewalKey& key) const
{
  // Only locally received identifiers and source time participate. This is a
  // rotating deterministic slot, not a global relay election or an RNG draw.
  const auto [origin,head,cluster,stamp]=key;
  uint64_t mix=static_cast<uint64_t>(stamp) ^ (uint64_t(origin)<<48) ^
    (uint64_t(head)<<32) ^ (uint64_t(cluster)<<16) ^ uint64_t(MyNodeId());
  mix^=mix>>33; mix*=UINT64_C(0xff51afd7ed558ccd);
  mix^=mix>>33; mix*=UINT64_C(0xc4ceb9fe1a85ec53); mix^=mix>>33;
  const auto lifetime=m_beaconInterval*10;
  const auto remaining=std::max(Time(),MicroSeconds(stamp)+lifetime-Simulator::Now());
  const auto window=std::min({m_proxyRenewalWindow,lifetime/4,remaining/2});
  return NanoSeconds(window.GetNanoSeconds()*int64_t(mix%32)/31);
}

void RoutingProtocol::SendPendingRenewal(RenewalKey key)
{
  auto it=m_pendingRenewals.find(key);
  if (it==m_pendingRenewals.end()) return;
  auto pending=it->second; m_pendingRenewals.erase(it);
  if (!m_ipv4 || !m_coordinateProxyRenewal) return;
  TraceRenewalCoordination("relay_holdoff_completed",key,pending.sourceIp,
    pending.helloUid.value_or(0),pending.helloUid);
  // Recheck permission, source age and the current local CH view at send time.
  RelayProxyMemberHello(pending.hello,pending.sourceIp,pending.helloUid,true);
}

void RoutingProtocol::ClearPendingRenewals()
{
  for (auto& [key,pending]:m_pendingRenewals) pending.timer.Cancel();
  m_pendingRenewals.clear(); m_confirmedRenewals.clear();
  m_renewalAckBatchTimer.Cancel(); m_pendingRenewalAcks.clear();
  ClearDirectMaintenanceCoverage();
  m_directMaintenanceRecords.clear(); m_directMaintenanceCursor.reset();
}

void RoutingProtocol::TraceRenewalCoordination(const char* event, const RenewalKey& key,
    Ipv4Address peer, uint64_t packetUid, std::optional<uint64_t> helloUid,
    std::optional<int> socketResult, std::optional<uint64_t> relatedUid, std::optional<Time> holdoff)
{
  if (m_proxyRenewalEvent.IsEmpty()) return;
  const auto [origin,head,cluster,stamp]=key;
  const auto sourceTime=MicroSeconds(stamp);
  ProxyRenewalEvent e{Simulator::Now(),MyNodeId(),origin,head,cluster,peer,event,packetUid,
    helloUid,sourceTime.GetSeconds(),sourceTime,std::nullopt,m_beaconInterval*10,0,socketResult};
  e.observationUs=stamp; e.relatedPacketUid=relatedUid; e.holdoff=holdoff;
  m_proxyRenewalEvent(e);
}

void RoutingProtocol::SendRenewalAck(const ClusterMemberBeaconHeader& beacon, uint64_t receivedUid)
{
  if (!m_coordinateProxyRenewal || !IsProxyJoinEnabled() || !m_enableProxyRenewal) return;
  const auto stamp=beacon.GetReceivedTimeStampUs();
  if (!stamp || *stamp<0) return;
  const RenewalKey key{beacon.GetNodeId(),MyNodeId(),beacon.GetClusterId(),*stamp};
  if (!m_coalesceRenewalAck) {
    SendRenewalAckRecords({PendingRenewalAck{key,receivedUid}});
    return;
  }
  const RenewalAssociation association{beacon.GetNodeId(),MyNodeId(),beacon.GetClusterId()};
  auto previous=m_pendingRenewalAcks.find(association);
  if (previous!=m_pendingRenewalAcks.end()) {
    if (std::get<3>(previous->second.key)>=*stamp) {
      TraceRenewalCoordination("head_feedback_covered_pending",key,Ipv4Address(),receivedUid,
        std::nullopt,std::nullopt,previous->second.receivedUid);
      return;
    }
    TraceRenewalCoordination("head_feedback_replaced_pending",previous->second.key,Ipv4Address(),
      previous->second.receivedUid,std::nullopt,std::nullopt,receivedUid);
  }
  m_pendingRenewalAcks.insert_or_assign(association,PendingRenewalAck{key,receivedUid});
  if (!m_renewalAckBatchTimer.IsRunning()) {
    m_renewalAckBatchDue=Simulator::Now()+m_renewalAckBatchWindow;
    m_renewalAckBatchTimer=Simulator::Schedule(m_renewalAckBatchWindow,&RoutingProtocol::FlushRenewalAcks,this);
  }
  TraceRenewalCoordination("head_feedback_queued",key,Ipv4Address(),receivedUid,
    std::nullopt,std::nullopt,std::nullopt,m_renewalAckBatchDue-Simulator::Now());
  if (m_pendingRenewalAcks.size()==MAX_RENEWAL_ACK_RECORDS) FlushRenewalAcks();
}

bool RoutingProtocol::IsRenewalAckCurrent(const RenewalKey& key) const
{
  if (!m_ipv4 || !m_coordinateProxyRenewal || !m_coalesceRenewalAck ||
      !IsProxyJoinEnabled() || !m_enableProxyRenewal) return false;
  auto state=m_ipv4->GetObject<Node>()->GetObject<ClusterNode>();
  if (!state || state->GetRoleType()!=Head) return false;
  auto cluster=CurrentOwnedCluster(m_clusterManager,state);
  const auto [origin,head,id,stamp]=key;
  if (!cluster || head!=MyNodeId() || state->GetClusterId()!=id ||
      !cluster->ExistMember(origin)) return false;
  auto member=cluster->GetAllMembers().at(origin);
  const auto now=Simulator::Now(),lifetime=m_beaconInterval*10;
  const auto refresh=cluster->GetMemberUpdateTime(origin);
  // The shared member object is the existing CH lease contract, not relay
  // knowledge. Recheck it after the wait; never create or revive an entry.
  return member && member->GetRoleType()==Member && member->GetClusterId()==id &&
    stamp>=0 && MicroSeconds(stamp)<=now && MicroSeconds(stamp)+lifetime>=now &&
    refresh+lifetime>=now && refresh+NanoSeconds(1)>=MicroSeconds(stamp);
}

void RoutingProtocol::FlushRenewalAcks()
{
  m_renewalAckBatchTimer.Cancel();
  auto pending=std::move(m_pendingRenewalAcks); m_pendingRenewalAcks.clear();
  std::vector<PendingRenewalAck> records;
  for (const auto& [association,record]:pending) {
    if (IsRenewalAckCurrent(record.key)) records.push_back(record);
    else TraceRenewalCoordination("head_feedback_discarded_pending",record.key,Ipv4Address(),record.receivedUid);
  }
  if (!records.empty()) SendRenewalAckRecords(records);
}

void RoutingProtocol::SendRenewalAckRecords(const std::vector<PendingRenewalAck>& records)
{
  NS_ASSERT(!records.empty() && records.size()<=MAX_RENEWAL_ACK_RECORDS);
  auto packet=Create<Packet>();
  // Fixed 14-byte records, no simulator UID on wire. One record retains the
  // existing type-eight packet exactly; UDP length delimits a bounded list.
  for (auto it=records.rbegin();it!=records.rend();++it) {
    const auto [origin,head,cluster,stamp]=it->key;
    packet->AddHeader(RenewalAckHeader(origin,head,cluster,stamp));
  }
  packet->AddHeader(TypeHeader(CAFBCR_RENEWAL_ACK));
  for (const auto& [socket,address]:m_socketAddrs) {
    const auto destination=address.GetMask()==Ipv4Mask::GetOnes() ?
      Ipv4Address("255.255.255.255") : address.GetBroadcast();
    const auto result=SendControlPacket(socket,packet,destination);
    ++m_renewalAckCount; m_ControlPacketSize+=packet->GetSize();
    for (const auto& record:records)
      TraceRenewalCoordination("head_feedback_tx_attempt",record.key,destination,packet->GetUid(),
        std::nullopt,result,record.receivedUid);
    break;
  }
}

void RoutingProtocol::RecvRenewalAck(Ptr<Packet> packet, Ipv4Address sourceIp)
{
  if (!m_coordinateProxyRenewal || !IsProxyJoinEnabled() || !m_enableProxyRenewal ||
      !m_neighbors || !packet->GetSize() || packet->GetSize()%14 ||
      packet->GetSize()>14*MAX_RENEWAL_ACK_RECORDS) return;
  while (packet->GetSize()) {
    RenewalAckHeader ack; packet->RemoveHeader(ack);
    ApplyRenewalAck(ack,sourceIp,packet->GetUid());
  }
}

void RoutingProtocol::ApplyRenewalAck(const RenewalAckHeader& ack, Ipv4Address sourceIp, uint64_t packetUid)
{
  const auto now=Simulator::Now(), lifetime=m_beaconInterval*10;
  if (ack.GetOrigin()==0xffff || ack.GetOrigin()==MyNodeId() ||
      ack.GetHead()==MyNodeId() || ack.GetHead()==ack.GetOrigin() || ack.GetCluster()==0xffff ||
      ack.GetSourceUs()<0 || ack.GetSourceUs()>now.GetMicroSeconds() ||
      MicroSeconds(ack.GetSourceUs())+lifetime<now) return;
  const auto& neighbors=m_neighbors->PeekAll(); auto ch=neighbors.find(ack.GetHead());
  if (ch==neighbors.end() || ch->second.Role!=Head || ch->second.ipAddress!=sourceIp ||
      ch->second.clusterId!=ack.GetCluster() ||
      ch->second.UpdateTime+ch->second.NeighborLifeLength<=now) return;
  const RenewalKey key{ack.GetOrigin(),ack.GetHead(),ack.GetCluster(),ack.GetSourceUs()};
  TraceRenewalCoordination("relay_feedback_rx",key,sourceIp,packetUid);
  const RenewalAssociation association{ack.GetOrigin(),ack.GetHead(),ack.GetCluster()};
  auto [it,inserted]=m_confirmedRenewals.try_emplace(association,ack.GetSourceUs());
  if (!inserted) it->second=std::max(it->second,ack.GetSourceUs());
  for (auto pending=m_pendingRenewals.begin();pending!=m_pendingRenewals.end();) {
    const auto& [origin,head,cluster,stamp]=pending->first;
    if (RenewalAssociation{origin,head,cluster}==association && stamp<=ack.GetSourceUs()) {
      pending->second.timer.Cancel();
      TraceRenewalCoordination("relay_pending_cancelled_feedback",pending->first,sourceIp,
        packetUid,pending->second.helloUid);
      pending=m_pendingRenewals.erase(pending);
    } else ++pending;
  }
}

void RoutingProtocol::RecvClusterHeadAnnouncement(Ptr<Packet> packet, Ipv4Address myIp, Ipv4Address srcIp)
{
  NS_LOG_FUNCTION(this << srcIp);
  
  ClusterHeadAnnounceHeader chCallHeader;
  packet->RemoveHeader(chCallHeader);
  
  NS_LOG_INFO("Received CH_CALL from Node " << chCallHeader.GetNodeId() 
              << " at time " << Simulator::Now().GetSeconds());
  
  Ptr<Node> node = m_ipv4->GetObject<Node>();
  Ptr<ClusterNode> clusterNode = node->GetObject<ClusterNode>();
  
  if (!clusterNode) return;
  
  // 更新邻居信息
  // Vector myPos = GetMyPosition();
  // Vector myVel = GetVelocity();
  /*double lhtp = CalculateLHTP(myPos, myVel, chCallHeader.GetPosition(), 
                              chCallHeader.GetVelocity());*/
  
  Neighbors::NeighborInfo neighborInfo(
    chCallHeader.GetNodeId(),
    chCallHeader.GetIpAddress(),
    0xFFFF,
    chCallHeader.GetPosition(),
    chCallHeader.GetVelocity(),
    chCallHeader.GetResidualEnergy(),
    chCallHeader.GetFitnessValue(),
    BackupHead,
    Simulator::Now(),
    Seconds(m_helloInterval.GetSeconds() * 2)
  );
  
  UpdateNeighbor(neighborInfo, "CH_CALL", false, true);

  // 判断是否已经加入簇
  NodeRole myRole = clusterNode->GetRoleType();
  bool alreadyClustered = (myRole == Member || myRole == Head);

  if (myRole == BackupHead && Simulator::Now().GetSeconds() < (m_clusteringTime.GetSeconds() + 1.0)) {
    alreadyClustered = true; // 簇头候选在早期阶段不加入其他簇，缩短等待时间以提高入簇率
  }

  NS_LOG_INFO("Node " << MyNodeId() << " received CH_CALL from " << chCallHeader.GetNodeId()
              << ", myRole=" << (int)myRole << ", alreadyClustered=" << alreadyClustered);

  // 如果未加入簇，收集簇头候选
  if (!alreadyClustered) {
    // 计算距离
    Vector myPos = GetMyPosition();
    double dx = chCallHeader.GetPosition().x - myPos.x;
    double dy = chCallHeader.GetPosition().y - myPos.y;
    double dz = chCallHeader.GetPosition().z - myPos.z;
    double distance = std::sqrt(dx*dx + dy*dy + dz*dz);

    // [D-γ] 不再硬过滤超距 candidate——只要本节点能"通过 reachable 邻居
    // 多跳到达"该 CH，仍可作为合法 join 目标。距离信息保留在 candidate.distance
    // 字段，由 SelectNearestClusterHeadAndJoin 做两轮筛选（reachable 优先，
    // multi-hop reachable fallback）。JOIN_REQ 经 RouteOutput 走多跳路径，
    // 路由层的距离 check 仍守门，物理上不可达的 CH 会在 select 阶段被排除。

    // 添加到候选列表
    ClusterHeadCandidate candidate;
    candidate.nodeId = chCallHeader.GetNodeId();
    candidate.ipAddress = chCallHeader.GetIpAddress();
    candidate.position = chCallHeader.GetPosition();
    candidate.distance = distance;
    candidate.fitness = chCallHeader.GetFitnessValue();
    m_chCandidates.push_back(candidate);

    // 防御性上限：单轮内不可能有超过 64 个邻居 BackupHead 同时发出 CH_CALL，
    // 超过该值说明上游 reset 出了问题，直接丢掉最旧的候选避免向量无界增长
    if (m_chCandidates.size() > 64) {
      m_chCandidates.erase(m_chCandidates.begin());
    }

    NS_LOG_INFO("Node " << MyNodeId() << " added CH candidate " << candidate.nodeId
                << " at distance " << distance << "m, fitness=" << candidate.fitness);

    // 第一次收到CH_CALL时,调度选择函数(延迟1秒让所有CH_CALL到达)
    if (!m_hasReceivedChCall) {
      m_hasReceivedChCall = true;
      Simulator::Schedule(Seconds(1.0), &RoutingProtocol::SelectNearestClusterHeadAndJoin, this);
      NS_LOG_INFO("Node " << MyNodeId() << " scheduled SelectNearestClusterHeadAndJoin");
    }
  }
}

void RoutingProtocol::RecvJoinRequest(Ptr<Packet> packet, Ipv4Address myIp, Ipv4Address srcIp)
{
  NS_LOG_FUNCTION(this << srcIp);
  
  JoinClusterRequestHeader joinReqHeader;
  packet->RemoveHeader(joinReqHeader);
  
  Ptr<Node> node = m_ipv4->GetObject<Node>();
  Ptr<ClusterNode> clusterNode = node->GetObject<ClusterNode>();
  
  if (!clusterNode) return;
  
  NS_LOG_INFO("Received JOIN_REQ from Node " << joinReqHeader.GetNodeId());
  TraceJoin("ordinary_request_rx", joinReqHeader.GetNodeId(), MyNodeId(), 0xFFFF, srcIp);

  // Reuse the shared applicant object already needed for JoinCluster below.
  // A stale registry head ID alone does not mean the applicant is still a Head.
  Ptr<ClusterNode> applicant = joinReqHeader.GetNodeId() < NodeList::GetNNodes()
    ? NodeList::GetNode(joinReqHeader.GetNodeId())->GetObject<ClusterNode>() : nullptr;
  if ((clusterNode->GetRoleType() == Head || clusterNode->GetRoleType() == BackupHead) &&
      applicant && applicant->GetRoleType() == Head &&
      m_clusterManager->ExistHead(joinReqHeader.GetNodeId())) {
    TraceJoin("ordinary_request_rejected_head", joinReqHeader.GetNodeId(), MyNodeId(),
              m_clusterManager->GetClusterId(joinReqHeader.GetNodeId()), srcIp);
    return;
  }
  
  // 更新邻居信息
  // Vector myPos = GetMyPosition();
  // Vector myVel = GetVelocity();
  // double lhtp = CalculateLHTP(myPos, myVel, joinReqHeader.GetPosition(),
  //                             joinReqHeader.GetVelocity());
  
  Neighbors::NeighborInfo neighborInfo(
    joinReqHeader.GetNodeId(),
    joinReqHeader.GetIpAddress(),
    0xFFFF,
    joinReqHeader.GetPosition(),
    joinReqHeader.GetVelocity(),
    joinReqHeader.GetResidualEnergy(),
    joinReqHeader.GetFitness(),
    Member,
    Simulator::Now(),
    Seconds(m_helloInterval.GetSeconds() * 2)
  );
  
  UpdateNeighbor(neighborInfo, "JOIN_REQ", false, true);
  
  // 只有簇头或候选簇头可以接受成员
  NodeRole myRole = clusterNode->GetRoleType();
  if (myRole != Head && myRole != BackupHead) {
    return;
  }
  
  uint16_t reqNodeId = joinReqHeader.GetNodeId();
  bool createNewCluster = false;
  uint16_t myClusterId = 0xFFFF;
  
  // Reuse only the current cluster actually headed by this receiver. An old
  // head/member registration must not redirect admission or suppress promotion.
  if (auto owned = CurrentOwnedCluster(m_clusterManager, clusterNode)) {
    myClusterId = owned->GetClusterId();
  } else {
    createNewCluster = true;
  }
  
  // 检查请求节点是否已在其他簇
  if (m_clusterManager->ExistMember(reqNodeId)) {
    uint16_t oldClusterId = m_clusterManager->GetClusterId(reqNodeId);
    uint16_t oldHeadId = m_clusterManager->GetClusterHeadId(oldClusterId);
    if (oldHeadId != MyNodeId()) {
      // 从旧簇移除
      m_clusterManager->LeaveCluster(reqNodeId, oldClusterId);
      NS_LOG_INFO("Remove Node " << reqNodeId << " from old cluster " << oldClusterId);
    }
  }
  
  // 创建簇或加入现有簇
  if (createNewCluster) {
    myClusterId = m_clusterManager->ArrangeClusterId();
    m_clusterManager->CreateCluster(myClusterId, clusterNode);
    NS_LOG_INFO("Node " << MyNodeId() << " created cluster " << myClusterId);

    // 开始簇维护（如果BeaconTimer已在运行，先取消再重新调度）
    if (m_beaconTimer.IsRunning()) {
      m_beaconTimer.Cancel();
    }
    m_beaconTimer.SetFunction(&RoutingProtocol::BeaconTimerExpire, this);
    m_beaconTimer.Schedule(m_beaconInterval);  // 立即开始，使用beacon间隔
  }
  
  // 添加成员
  Ptr<ClusterNode> memberNode = applicant;
  if (memberNode) {
    m_clusterManager->JoinCluster(myClusterId, memberNode);
    // 更新成员的时间戳，防止立即超时
    Ptr<Cluster> cluster = m_clusterManager->GetCluster(myClusterId);
    if (cluster) {
      cluster->UpdateMemberTime(reqNodeId, Simulator::Now());
    }
    NS_LOG_INFO("Node " << reqNodeId << " joined cluster " << myClusterId);
    TraceJoin("ordinary_admitted_at_head", reqNodeId, MyNodeId(), myClusterId, srcIp);
  }

  // 发送JOIN_RESP
  Simulator::Schedule(MilliSeconds(2), &RoutingProtocol::SendClusterJoinResponse,
                     this, joinReqHeader.GetIpAddress());
}

void RoutingProtocol::RecvProxyJoinRequest(Ptr<Packet> packet, Ipv4Address myIp, Ipv4Address srcIp)
{
  NS_LOG_FUNCTION(this << srcIp);

  JoinClusterRequestHeader joinReqHeader;
  packet->RemoveHeader(joinReqHeader);

  if (!IsProxyJoinEnabled()) {
    TraceJoin("proxy_rx_disabled", joinReqHeader.GetNodeId(), MyNodeId(), 0xFFFF, srcIp);
    return;
  }

  Ptr<Node> node = m_ipv4->GetObject<Node>();
  Ptr<ClusterNode> clusterNode = node->GetObject<ClusterNode>();
  if (!clusterNode) return;

  uint16_t originNodeId = joinReqHeader.GetNodeId();  // 真正想入簇的远 SWARM
  Ptr<ClusterNode> memberNode = originNodeId < NodeList::GetNNodes()
    ? NodeList::GetNode(originNodeId)->GetObject<ClusterNode>() : nullptr;
  TraceJoin("proxy_request_rx", originNodeId, MyNodeId(), 0xFFFF, srcIp);
  NS_LOG_INFO("Received PROXY_JOIN_REQ for origin " << originNodeId
              << " (proxied by " << srcIp << ")");

  // 注意：不把 origin 节点加进邻居表——它是远节点，不是真正邻居；srcIp 才是真正邻居（已在其它收包路径加过）
  // 只有簇头或候选簇头处理
  NodeRole myRole = clusterNode->GetRoleType();
  if (myRole != Head && myRole != BackupHead) {
    NS_LOG_DEBUG("Not Head/BackupHead, drop PROXY_JOIN_REQ");
    TraceJoin("proxy_rejected_role", originNodeId, MyNodeId(), 0xFFFF, srcIp);
    return;
  }

  // 跟 RecvJoinRequest 同款 cluster 创建/迁移逻辑
  uint16_t myClusterId = 0xFFFF;
  bool createNewCluster = false;
  if (auto owned = CurrentOwnedCluster(m_clusterManager, clusterNode)) {
    myClusterId = owned->GetClusterId();
  } else {
    createNewCluster = true;
  }
  if (m_clusterManager->ExistMember(originNodeId)) {
    uint16_t oldClusterId = m_clusterManager->GetClusterId(originNodeId);
    uint16_t oldHeadId = m_clusterManager->GetClusterHeadId(oldClusterId);
    if (oldHeadId == originNodeId && memberNode && memberNode->GetRoleType() == Head) {
      TraceJoin("proxy_request_rejected_head", originNodeId, MyNodeId(), oldClusterId, srcIp);
      return;
    }
    if (oldHeadId != MyNodeId()) {
      m_clusterManager->LeaveCluster(originNodeId, oldClusterId);
    }
  }
  if (createNewCluster) {
    myClusterId = m_clusterManager->ArrangeClusterId();
    m_clusterManager->CreateCluster(myClusterId, clusterNode);
    if (m_beaconTimer.IsRunning()) m_beaconTimer.Cancel();
    m_beaconTimer.SetFunction(&RoutingProtocol::BeaconTimerExpire, this);
    m_beaconTimer.Schedule(m_beaconInterval);
  }
  if (memberNode) {
    m_clusterManager->JoinCluster(myClusterId, memberNode);
    Ptr<Cluster> cluster = m_clusterManager->GetCluster(myClusterId);
    if (cluster) cluster->UpdateMemberTime(originNodeId, Simulator::Now());
    TraceJoin("proxy_admitted_at_head", originNodeId, MyNodeId(), myClusterId, srcIp);
  }

  // [E2 关键差异] 不发 JOIN_RESP（远 SWARM 路径不可靠），改成把 origin NodeId
  // 加进 m_recentGranted 队列。下次 SendClusterHeadBeacon 时通过 CH_LIVE
  // broadcast 的 grantedMembers 字段告诉远 SWARM "你已被接受"。
  // 队列容量 = ClusterHeadBeaconHeader::kGrantedMembersCap (8)，FIFO 滚动。
  // 去重：如果 origin 已在队列里就不加（避免反复同一个）
  bool already = false;
  for (auto id : m_recentGranted) { if (id == originNodeId) { already = true; break; } }
  if (!already) {
    m_recentGranted.push_back(originNodeId);
    TraceJoin("grant_enqueued", originNodeId, MyNodeId(), myClusterId);
    while (m_recentGranted.size() > ClusterHeadBeaconHeader::kGrantedMembersCap) {
      TraceJoin("grant_evicted", m_recentGranted.front(), MyNodeId(), myClusterId);
      m_recentGranted.pop_front();
    }
    NS_LOG_INFO("CH " << MyNodeId() << " granted origin " << originNodeId
                << " into cluster " << myClusterId
                << "; recentGranted size=" << m_recentGranted.size());
  }
}

void RoutingProtocol::RecvJoinResponse(Ptr<Packet> packet, Ipv4Address myIp, Ipv4Address srcIp)
{
  NS_LOG_FUNCTION(this << srcIp);
  
  ClusterHeadResponseHeader joinRespHeader;
  packet->RemoveHeader(joinRespHeader);
  
  Ptr<Node> node = m_ipv4->GetObject<Node>();
  Ptr<ClusterNode> clusterNode = node->GetObject<ClusterNode>();
  
  if (!clusterNode) return;
  
  NS_LOG_INFO("Received JOIN_RESP from CH " << joinRespHeader.GetChNodeId()
              << " for cluster " << joinRespHeader.GetClusterId());
  TraceJoin("ordinary_response_rx", MyNodeId(), joinRespHeader.GetChNodeId(),
            joinRespHeader.GetClusterId(), srcIp);
  
  // 设置为簇成员
  NodeRole myRole = clusterNode->GetRoleType();
  // Even an in-flight response to an originally valid request must not demote
  // a node that has since formed a cluster; head handover needs an explicit path.
  if (myRole == Head) {
    TraceJoin("ordinary_response_ignored_head", MyNodeId(), joinRespHeader.GetChNodeId(),
              joinRespHeader.GetClusterId(), srcIp);
    return;
  }
  if (myRole == Member) {
    // 已经是成员，检查是否是同一个簇
    uint16_t myClusterId = clusterNode->GetClusterId();
    if (myClusterId != joinRespHeader.GetClusterId()) {
      NS_LOG_WARN("Already member of cluster " << myClusterId);
    }
  } else {
    clusterNode->SetRole(Member);
    clusterNode->SetClusterId(joinRespHeader.GetClusterId());
    m_proxyMembership.reset();
    ClearDirectMaintenanceCoverage();
    TraceJoin("ordinary_confirmed_at_source", MyNodeId(), joinRespHeader.GetChNodeId(),
              joinRespHeader.GetClusterId(), srcIp);
    NS_LOG_INFO("Node " << MyNodeId() << " became member of cluster "
                << joinRespHeader.GetClusterId());

    // 延迟50ms发送第一个CM_LIVE,让ClusterManager更新完成,但时间要足够短防止超时
    Simulator::Schedule(MilliSeconds(50), &RoutingProtocol::SendClusterMemberBeaconFor,
                        this, "ordinary-confirmation");

    // 开始发送CM_LIVE（如果BeaconTimer已在运行，先取消再重新调度）
    if (m_beaconTimer.IsRunning()) {
      m_beaconTimer.Cancel();
    }
    m_beaconTimer.SetFunction(&RoutingProtocol::BeaconTimerExpire, this);
    m_beaconTimer.Schedule(m_beaconInterval);
  }
  
  // 更新邻居簇头信息
  if (m_neighbors) {
    auto neighbors = m_neighbors->GetAll();
    auto it = neighbors.find(joinRespHeader.GetChNodeId());
    if (it != neighbors.end()) {
      Neighbors::NeighborInfo info = it->second;
      info.Role = Head;
      info.clusterId = joinRespHeader.GetClusterId();
      info.UpdateTime = Simulator::Now();
      UpdateNeighbor(info, "JOIN_RESP", false, false);
    }
  }
}

void RoutingProtocol::RecvClusterHeadBeacon(Ptr<Packet> packet, Ipv4Address myIp, Ipv4Address srcIp)
{
  NS_LOG_FUNCTION(this << srcIp);
  
  ClusterHeadBeaconHeader chBeaconHeader;
  packet->RemoveHeader(chBeaconHeader);
  
  NS_LOG_DEBUG("Received CH_LIVE from CH " << chBeaconHeader.GetChNodeId());
  
  if (!m_neighbors) return;
  
  // 更新邻居信息
  // Vector myPos = GetMyPosition();
  // Vector myVel = GetVelocity();
  // double lhtp = CalculateLHTP(myPos, myVel, chBeaconHeader.GetPosition(),
  //                             chBeaconHeader.GetVelocity());
  
  Neighbors::NeighborInfo neighborInfo(
    chBeaconHeader.GetChNodeId(),
    chBeaconHeader.GetChIpAddress(),
    chBeaconHeader.GetClusterId(),
    chBeaconHeader.GetPosition(),
    chBeaconHeader.GetVelocity(),
    0.0, // Energy not included in beacon
    chBeaconHeader.GetFitness(),
    Head,
    Simulator::Now(),
    Seconds(m_helloInterval.GetSeconds() * 2)
  );
  
  UpdateNeighbor(neighborInfo, "CH_LIVE", false, false);

  // 如果是簇成员，检查簇头是否是自己的簇头
  Ptr<Node> node = m_ipv4->GetObject<Node>();
  Ptr<ClusterNode> clusterNode = node->GetObject<ClusterNode>();

  // [E2 piggyback] 扫 grantedMembers 列表，看里面有没有自己 NodeId
  // 有 → 自确认入簇（无需 unicast JOIN_RESP）
  if (clusterNode) {
    uint16_t myId = MyNodeId();
    for (uint8_t i = 0; i < ClusterHeadBeaconHeader::kGrantedMembersCap; ++i) {
      uint16_t nid = chBeaconHeader.GetGrantedMember(i);
      if (nid == 0xFFFF) continue;
      if (nid == myId) {
        uint16_t newClusterId = chBeaconHeader.GetClusterId();
        if (!IsProxyJoinEnabled()) {
          TraceJoin("grant_ignored_disabled", myId, chBeaconHeader.GetChNodeId(), newClusterId, srcIp);
          break;
        }
        TraceJoin("grant_rx", myId, chBeaconHeader.GetChNodeId(), newClusterId, srcIp);
        NodeRole curRole = clusterNode->GetRoleType();
        // Only an idle applicant may confirm its current intended CH. A grant
        // from an earlier association must not move a stable member/backup CH.
        if (curRole != Idle) {
          TraceJoin("grant_ignored_role", myId, chBeaconHeader.GetChNodeId(), newClusterId, srcIp);
          break;
        }
        if (m_intentJoinChId != chBeaconHeader.GetChNodeId()) {
          TraceJoin("grant_ignored_intent", myId, chBeaconHeader.GetChNodeId(), newClusterId, srcIp);
          break;
        }
        // Admission may have been replaced or removed while this grant was
        // in flight. Do not overwrite that local state and create a Member
        // binding absent from the target's registration. Remain Idle with
        // the current intent so the existing request/grant retry can recover.
        // This uses only this node's state, not a remote CH membership query.
        if (clusterNode->GetClusterId() != newClusterId) {
          TraceJoin("grant_ignored_admission", myId, chBeaconHeader.GetChNodeId(), newClusterId, srcIp);
          break;
        }
        {
          clusterNode->SetRole(Member);
          clusterNode->SetClusterId(newClusterId);
          m_proxyMembership = std::make_pair(chBeaconHeader.GetChNodeId(), newClusterId);
          m_proxyMembershipSince = Simulator::Now();
          ClearDirectMaintenanceCoverage();
          m_intentJoinChId = 0xFFFF;  // 入簇成功，清掉 intent
          TraceJoin("proxy_confirmed_at_source", myId, chBeaconHeader.GetChNodeId(), newClusterId, srcIp);
          NS_LOG_INFO("Node " << myId << " self-confirmed JOIN via grantedMembers, "
                      << "clusterId=" << newClusterId
                      << " CH=" << chBeaconHeader.GetChNodeId());
          // 启动 BeaconTimer 发 CM_LIVE
          if (m_beaconTimer.IsRunning()) m_beaconTimer.Cancel();
          m_beaconTimer.SetFunction(&RoutingProtocol::BeaconTimerExpire, this);
          m_beaconTimer.Schedule(m_beaconInterval);
          // 立即发一个 CM_LIVE 给 CH (best-effort，CH 视角已经把我加进 cluster 了)
          Simulator::Schedule(MilliSeconds(50),
                              &RoutingProtocol::SendClusterMemberBeaconFor, this, "proxy-confirmation");
        }
        break;
      }
    }
  }

  if (DirectMaintenanceEnabled() && packet->GetSize()) {
    DirectMaintenanceHeader feedback;
    if (packet->RemoveHeader(feedback) && feedback.IsValid())
      ApplyDirectMaintenance(chBeaconHeader, feedback, srcIp, packet->GetUid());
  }

  if (clusterNode && clusterNode->GetRoleType() == Member) {
    uint16_t myClusterId = clusterNode->GetClusterId();
    uint16_t chNodeId = chBeaconHeader.GetChNodeId();

    if (m_clusterManager->GetClusterHeadId(myClusterId) == chNodeId) {
      // 收到自己簇头的CH_LIVE，发送CM_LIVE响应
      // 添加随机延迟避免冲突
      double delay = m_uniformRandomVariable->GetValue(0.0, 50.0);  // 0-50ms
      if (m_feedbackCoveredCmResponse)
        Simulator::Schedule(MilliSeconds(delay), &RoutingProtocol::SendClusterMemberResponse,
                            this, chNodeId, myClusterId, m_proxyMembershipSince);
      else
        Simulator::Schedule(MilliSeconds(delay), &RoutingProtocol::SendClusterMemberBeaconFor,
                            this, "ch-live-response");
      NS_LOG_DEBUG("Member " << MyNodeId() << " will respond to CH " << chNodeId << " with CM_LIVE");
    }
  }
  // [D-β + D-γ] Idle 节点把 CH_LIVE 当作"持续可加入"信号——CH_CALL 是一次性的，
  // 后到的 SWARM 永远收不到。让 Idle 节点把 CH_LIVE 的发起者加进 m_chCandidates，
  // 不在此处做距离硬过滤；SelectNearestClusterHeadAndJoin 会做两轮筛选
  // （reachable 优先，multi-hop reachable fallback）。
  else if (clusterNode && clusterNode->GetRoleType() == Idle) {
    Vector myPos = GetMyPosition();
    Vector chPos = chBeaconHeader.GetPosition();
    double dx = chPos.x - myPos.x;
    double dy = chPos.y - myPos.y;
    double dz = chPos.z - myPos.z;
    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    ClusterHeadCandidate candidate;
    candidate.nodeId = chBeaconHeader.GetChNodeId();
    candidate.ipAddress = chBeaconHeader.GetChIpAddress();
    candidate.position = chPos;
    candidate.distance = distance;
    candidate.fitness = chBeaconHeader.GetFitness();
    m_chCandidates.push_back(candidate);
    if (m_chCandidates.size() > 64) {
      m_chCandidates.erase(m_chCandidates.begin());
    }

    NS_LOG_INFO("Idle node " << MyNodeId() << " picked CH " << candidate.nodeId
                << " from CH_LIVE at " << distance << "m, fitness=" << candidate.fitness);

    // 用 m_hasReceivedChCall 的同款去抖：1s 收集窗口内 schedule 一次 select
    if (!m_hasReceivedChCall) {
      m_hasReceivedChCall = true;
      Simulator::Schedule(Seconds(1.0),
                          &RoutingProtocol::SelectNearestClusterHeadAndJoin, this);
    }
  }
}

void RoutingProtocol::RecvClusterMemberBeacon(Ptr<Packet> packet, Ipv4Address myIp, Ipv4Address srcIp)
{
  NS_LOG_FUNCTION(this << srcIp);
  
  ClusterMemberBeaconHeader cmBeaconHeader;
  packet->RemoveHeader(cmBeaconHeader);
  
  NS_LOG_DEBUG("Received CM_LIVE from Node " << cmBeaconHeader.GetNodeId());
  
  Ptr<Node> node = m_ipv4->GetObject<Node>();
  Ptr<ClusterNode> clusterNode = node->GetObject<ClusterNode>();
  
  if (!clusterNode || clusterNode->GetRoleType() != Head) {
    return; // 只有簇头处理CM_LIVE
  }
  
  // 更新簇成员时间戳
  uint16_t clusterId = clusterNode->GetClusterId();
  uint16_t memberNodeId = cmBeaconHeader.GetNodeId();
  TraceJoin("cm_live_rx", memberNodeId, MyNodeId(), clusterId, srcIp);
  
  Ptr<Cluster> cluster = m_clusterManager->GetCluster(clusterId);
  if (srcIp != cmBeaconHeader.GetIpAddress()) {
    // A relayed CM carries the real origin. It cannot admit a member, migrate
    // an association, revive an expired record, or create a one-hop neighbor.
    TraceJoin("proxy_cm_live_rx", memberNodeId, MyNodeId(), clusterId, srcIp);
    TraceProxyRenewal("head_proxy_received", memberNodeId, MyNodeId(), cmBeaconHeader.GetClusterId(),
      srcIp, packet->GetUid(), cmBeaconHeader.GetTimeStamp());
    auto owned = CurrentOwnedCluster(m_clusterManager, clusterNode);
    if (!IsProxyJoinEnabled() || !m_enableProxyRenewal || !owned || owned != cluster ||
        cmBeaconHeader.GetClusterId() != clusterId || memberNodeId == MyNodeId() ||
        !cluster->ExistMember(memberNodeId)) {
      TraceJoin("proxy_renewal_rejected_association", memberNodeId, MyNodeId(), clusterId, srcIp);
      TraceProxyRenewal("head_association_rejected", memberNodeId, MyNodeId(), cmBeaconHeader.GetClusterId(),
        srcIp, packet->GetUid(), cmBeaconHeader.GetTimeStamp());
      return;
    }
    auto member = cluster->GetAllMembers().at(memberNodeId);
    if (!member || member->GetRoleType() != Member || member->GetClusterId() != clusterId) {
      TraceJoin("proxy_renewal_rejected_association", memberNodeId, MyNodeId(), clusterId, srcIp);
      TraceProxyRenewal("head_association_rejected", memberNodeId, MyNodeId(), cmBeaconHeader.GetClusterId(),
        srcIp, packet->GetUid(), cmBeaconHeader.GetTimeStamp());
      return;
    }
    const auto previous = cluster->GetMemberUpdateTime(memberNodeId);
    const auto now = Simulator::Now();
    const auto lifetime = m_beaconInterval * 10;
    const double stamp = cmBeaconHeader.GetTimeStamp();
    if (!std::isfinite(stamp) || stamp < 0 || Seconds(stamp) > now ||
        Seconds(stamp) <= previous || Seconds(stamp) + lifetime < now || previous + lifetime < now) {
      TraceJoin("proxy_renewal_rejected_time", memberNodeId, MyNodeId(), clusterId, srcIp);
      if (!m_proxyRenewalEvent.IsEmpty()) {
        uint8_t reasons = 0;
        if (!std::isfinite(stamp)) reasons |= ProxyRenewalEvent::NonFinite;
        else {
          if (stamp < 0) reasons |= ProxyRenewalEvent::Negative;
          if (Seconds(stamp) > now) reasons |= ProxyRenewalEvent::Future;
          if (Seconds(stamp) <= previous) reasons |= ProxyRenewalEvent::NotNewer;
          if (Seconds(stamp) + lifetime < now) reasons |= ProxyRenewalEvent::ObservationExpired;
        }
        if (previous + lifetime < now) reasons |= ProxyRenewalEvent::LeaseExpired;
        TraceProxyRenewal("head_time_rejected", memberNodeId, MyNodeId(), clusterId,
          srcIp, packet->GetUid(), stamp, previous, std::nullopt, std::nullopt, reasons);
      }
      // A recent observation can already be covered by a valid direct/newer
      // refresh. Confirm coverage without advancing or reviving its lease.
      if (std::isfinite(stamp) && stamp>=0 && Seconds(stamp)<=now &&
          Seconds(stamp)<=previous && Seconds(stamp)+lifetime>=now && previous+lifetime>=now)
        SendRenewalAck(cmBeaconHeader,packet->GetUid());
      return;
    }
    cluster->UpdateMemberTime(memberNodeId, Seconds(stamp));
    TraceJoin("proxy_cm_live_refresh_at_head", memberNodeId, MyNodeId(), clusterId, srcIp);
    TraceProxyRenewal("head_proxy_accepted", memberNodeId, MyNodeId(), clusterId,
      srcIp, packet->GetUid(), stamp, previous);
    SendRenewalAck(cmBeaconHeader,packet->GetUid());
    auto grant = std::find(m_recentGranted.begin(), m_recentGranted.end(), memberNodeId);
    if (grant != m_recentGranted.end()) m_recentGranted.erase(grant);
    return;
  }
  if (cluster && cluster->ExistMember(memberNodeId)) {
    RecordDirectMaintenance(cmBeaconHeader, packet->GetUid());
    TraceProxyRenewal("head_direct_accepted", memberNodeId, MyNodeId(), clusterId,
      srcIp, packet->GetUid(), cmBeaconHeader.GetTimeStamp(), cluster->GetMemberUpdateTime(memberNodeId));
    cluster->UpdateMemberTime(memberNodeId, Simulator::Now());
    TraceJoin("cm_live_refresh_at_head", memberNodeId, MyNodeId(), clusterId, srcIp);
    auto grant = std::find(m_recentGranted.begin(), m_recentGranted.end(), memberNodeId);
    if (grant != m_recentGranted.end()) {
      TraceJoin("grant_retired_cm", memberNodeId, MyNodeId(), clusterId, srcIp);
      m_recentGranted.erase(grant);
    }
    NS_LOG_DEBUG("Updated member " << memberNodeId << " timestamp in cluster " << clusterId);
  }
  
  // 更新邻居信息
  if (m_neighbors) {
    // Vector myPos = GetMyPosition();
    // Vector myVel = GetVelocity();
    // double lhtp = CalculateLHTP(myPos, myVel, cmBeaconHeader.GetPosition(),
    //                             cmBeaconHeader.GetVelocity());
    
    Neighbors::NeighborInfo neighborInfo(
      cmBeaconHeader.GetNodeId(),
      cmBeaconHeader.GetIpAddress(),
      cmBeaconHeader.GetClusterId(),
      cmBeaconHeader.GetPosition(),
      cmBeaconHeader.GetVelocity(),
      0.0,
      cmBeaconHeader.GetFitness(),
      Member,
      Simulator::Now(),
      Seconds(m_helloInterval.GetSeconds() * 2)
    );
    
    UpdateNeighbor(neighborInfo, "CM_LIVE", false, false);
  }
}

//-----------------------------------------------------------------------------
// 路由相关
//-----------------------------------------------------------------------------
Ptr<Ipv4Route> RoutingProtocol::RouteOutput(Ptr<Packet> p, const Ipv4Header& header,
                                             Ptr<NetDevice> oif, Socket::SocketErrno& sockerr)
{
  NS_LOG_FUNCTION(this << header.GetDestination());

  if (!p) {
    NS_LOG_DEBUG("Packet is empty");
    sockerr = Socket::ERROR_NOTERROR;
    return nullptr;
  }

  Ipv4Address dst = header.GetDestination();
  Ipv4Address src = header.GetSource();

  // 处理广播包
  if (dst.IsBroadcast() || (m_socketAddrs.size() > 0 && dst == m_socketAddrs.begin()->second.GetBroadcast())) {
    NS_LOG_DEBUG("Creating broadcast route to " << dst);
    Ptr<Ipv4Route> route = Create<Ipv4Route>();
    route->SetDestination(dst);

    if (src == Ipv4Address()) {
      if (m_socketAddrs.size() > 0) {
        route->SetSource(m_socketAddrs.begin()->second.GetLocal());
      } else {
        route->SetSource(m_mainAddress);
      }
    } else {
      route->SetSource(src);
    }

    route->SetGateway(Ipv4Address::GetBroadcast());
    route->SetOutputDevice(m_ipv4->GetNetDevice(1));
    sockerr = Socket::ERROR_NOTERROR;
    return route;
  }

  // 检查是否是环回
  if (IsMyOwnAddress(dst)) {
    sockerr = Socket::ERROR_NOTERROR;
    return LoopbackRoute(header, oif);
  }

  // 处理单播包：使用簇结构和贪婪地理路由的混合策略
  Ptr<Ipv4Route> route = Create<Ipv4Route>();
  route->SetDestination(dst);

  Ipv4Address routeSource = src;
  if (src == Ipv4Address()) {
    if (m_socketAddrs.size() > 0) {
      routeSource = m_socketAddrs.begin()->second.GetLocal();
    } else {
      routeSource = m_mainAddress;
    }
  }
  route->SetSource(routeSource);

  // SendControlPacket's synchronous local scope identifies our own controls.
  // Their sockets have TTL=1: a business-route gateway different from dst
  // cannot forward them to that peer. Preserve TTL and let native ARP/Wi-Fi
  // attempt the intended one-hop exchange, including genuine radio failure.
  // This is not a reachability proof and adds no peer/global-state query.
  // Broadcast, loopback and all application/forwarded packets stay above/below.
  if (m_oneHopControlUnicast && m_controlSendDepth > 0) {
    route->SetGateway(dst);
    route->SetOutputDevice(m_ipv4->GetNetDevice(1));
    sockerr = Socket::ERROR_NOTERROR;
    return route;
  }

  // 获取当前节点信息
  Ptr<Node> node = m_ipv4->GetObject<Node>();
  Ptr<ClusterNode> clusterNode = node->GetObject<ClusterNode>();
  uint16_t myClusterId = clusterNode ? clusterNode->GetClusterId() : 0xFFFF;
  NodeRole myRole = clusterNode ? clusterNode->GetRoleType() : Idle;
  uint16_t dstNodeId = GetNodeIdFromIp(dst);
  uint16_t dstClusterId = (dstNodeId != 0xFFFF) ? m_clusterManager->GetClusterId(dstNodeId) : 0xFFFF;

  Ipv4Address nextHop = Ipv4Address();

  // [2.2] 异构链路：每个候选 next-hop 都要做 WithinMyTxRange 守门，
  // 邻居表 IsNeighbor 为 true 不代表本节点能在自己 TX 功率下打回去
  // (asymmetric link from receiving high-TX hello/CH_LIVE)。

  // [C-final] 任务包检测：如果 packet 带 CafbcrComputeTaskTag，跳过簇结构 (策略 2/3)，
  // 直接走 GreedyReachableNextHop 朝 dst（dst 已由 client 端 PickDynamicServer
  // 选成近邻 RELAY，1-2 跳直达）。普通包仍按策略 1→2→3→4 走簇结构。
  CafbcrComputeTaskTag srcTaskTag;
  bool isComputeTask = (p && p->PeekPacketTag(srcTaskTag));
  ForwardAudit audit{"output", PeekPointer(p), header, node->GetId(),
    static_cast<uint16_t>(myRole), myClusterId, PeekPointer(m_clusterManager), PeekPointer(m_neighbors)};
  audit.task = isComputeTask;
  const char* selectedBranch = "none";

  // 策略1: 目标是直接邻居？直接发送
  if (dstNodeId != 0xFFFF && m_neighbors && m_neighbors->IsNeighbor(dstNodeId)
      && WithinMyTxRange(dstNodeId)) {
    nextHop = dst;
    selectedBranch = "direct_neighbor";
    NS_LOG_DEBUG("RouteOutput: Direct neighbor, send to " << dst);
  }
  // [C-final] 策略 1.5: 任务包直接 greedy 朝 dst，跳过簇结构中转
  else if (isComputeTask && m_neighbors && !m_neighbors->IsEmpty()) {
    Vector dstPos = GetPositionFromIp(dst);
    uint16_t nh = GreedyReachableNextHop(dstPos);
    const bool fallback = nh == 0xFFFF;
    if (fallback) nh = NearestReachableNeighbor(dstPos);
    if (nh != 0xFFFF) {
      auto it = m_neighbors->GetAll().find(nh);
      if (it != m_neighbors->GetAll().end()) {
        nextHop = it->second.ipAddress;
        selectedBranch = fallback ? "task_nearest" : "task_greedy";
        NS_LOG_DEBUG("RouteOutput: Task direct-greedy to " << nh << " (bypass cluster)");
      }
    }
  }
  // 策略2: 簇内转发 - 如果在同一簇
  else if (myClusterId != 0xFFFF && myClusterId == dstClusterId && dstNodeId != 0xFFFF) {
    // 簇内转发，通过簇头中继
    if (myRole != Head && m_neighbors) {
      uint16_t chId = m_clusterManager->GetClusterHeadId(myClusterId);
      if (chId != 0xFFFF && chId != MyNodeId() && m_neighbors->IsNeighbor(chId)
          && WithinMyTxRange(chId)) {
        nextHop = NodeList::GetNode(chId)->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();
        selectedBranch = "intra_via_head";
        NS_LOG_DEBUG("RouteOutput: Intra-cluster via CH " << chId);
      } else if (chId != 0xFFFF) {
        // [新增 Round 3] CH 不是直接邻居/超距，回退到贪婪转发朝向目标方向
        Vector dstPos2 = GetPositionFromIp(dst);
        Vector myPos2 = GetMyPosition();
        // [2.2] 用 GreedyReachableNextHop 替代裸 GreedyNeighbor，避免因最优
        // 邻居超距而退化到 perimeter 模式（导致 TTL 耗尽）
        uint16_t nh = GreedyReachableNextHop(dstPos2);
        if (nh != 0xFFFF) {
          auto it = m_neighbors->GetAll().find(nh);
          if (it != m_neighbors->GetAll().end()) {
            nextHop = it->second.ipAddress;
            selectedBranch = "intra_head_unreachable_greedy";
            NS_LOG_DEBUG("RouteOutput: Intra-cluster CH unreachable, greedy to " << nh);
          }
        }
        // 贪婪也失败，尝试周边转发
        if (nextHop == Ipv4Address()) {
          Vector lastHopPos = myPos2;
          // [2.A] 异构 perimeter：候选池预先按 m_maxRange 过滤
          uint16_t perimId = PerimeterReachableNextHop(dstPos2,myPos2,lastHopPos);
          if (perimId != 0xFFFF) {
            auto it = m_neighbors->GetAll().find(perimId);
            if (it != m_neighbors->GetAll().end()) {
              nextHop = it->second.ipAddress;
              selectedBranch = "intra_head_unreachable_perimeter";
              NS_LOG_DEBUG("RouteOutput: Intra-cluster CH unreachable, perimeter to " << perimId);
            }
          }
        }
      }
    } else if (myRole == Head && m_neighbors) {
      // 如果自己是簇头，尝试直接路由或通过邻居
      if (m_neighbors->IsNeighbor(dstNodeId) && WithinMyTxRange(dstNodeId)) {
        nextHop = dst;
        selectedBranch = "head_direct_member";
        NS_LOG_DEBUG("RouteOutput: CH direct to member " << dstNodeId);
      } else {
        // 簇内多跳，使用贪婪转发朝向目标
        Vector dstPos = GetPositionFromIp(dst);
        // [2.2] 用 GreedyReachableNextHop 限范围内挑次优
        uint16_t nextHopId = GreedyReachableNextHop(dstPos);
        if (nextHopId != 0xFFFF) {
          auto neighbors = m_neighbors->GetAll();
          auto it = neighbors.find(nextHopId);
          if (it != neighbors.end()) {
            nextHop = it->second.ipAddress;
            selectedBranch = "head_intra_greedy";
            NS_LOG_DEBUG("RouteOutput: Greedy to " << nextHopId << " toward intra-cluster dst");
          }
        }
      }
    }
  }
  // 策略3: 簇间转发 - 转发到自己的簇头，让簇头处理跨簇路由
  else if (myClusterId != 0xFFFF && myClusterId != dstClusterId && myRole != Head && m_neighbors) {
    uint16_t chId = m_clusterManager->GetClusterHeadId(myClusterId);
    if (chId != 0xFFFF && chId != MyNodeId() && m_neighbors->IsNeighbor(chId)
        && WithinMyTxRange(chId)) {
      nextHop = NodeList::GetNode(chId)->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();
      selectedBranch = "inter_via_head";
      NS_LOG_DEBUG("RouteOutput: Inter-cluster via my CH " << chId);
    } else if (chId != 0xFFFF) {
      // === 本簇头不是直接邻居/超距，寻找最近的邻居簇头作为接力 ===
      NS_LOG_DEBUG("RouteOutput: My CH " << chId << " not reachable, looking for nearest CH");
      Vector dstPos = GetPositionFromIp(dst);
      uint16_t nearestCh = m_neighbors->GetNearestClusterHead(dstPos);
      if (nearestCh != 0xFFFF && WithinMyTxRange(nearestCh)) {
        auto neighbors = m_neighbors->GetAll();
        auto it = neighbors.find(nearestCh);
        if (it != neighbors.end()) {
          nextHop = it->second.ipAddress;
          selectedBranch = "inter_nearest_head";
          NS_LOG_DEBUG("RouteOutput: Inter-cluster via nearest CH " << nearestCh);
        }
      }
    }
  }
  // 策略4: 回退到贪婪地理路由
  else if (m_neighbors && !m_neighbors->IsEmpty()) {
    Vector dstPos = GetPositionFromIp(dst);
    Vector myPos = GetMyPosition();
    // [2.2] GreedyReachableNextHop 自带距离过滤，相比 GreedyNeighbor 更可能
    // 找到合规 next-hop，避免直接退到周边模式造成路径过长
    uint16_t nextHopId = GreedyReachableNextHop(dstPos);
    if (nextHopId != 0xFFFF) {
      auto neighbors = m_neighbors->GetAll();
      auto it = neighbors.find(nextHopId);
      if (it != neighbors.end()) {
        nextHop = it->second.ipAddress;
        selectedBranch = "greedy";
        NS_LOG_DEBUG("RouteOutput: Greedy forward to neighbor " << nextHopId);
      }
    } else {
      // === 真正的局部最大值（范围内无 progress>0 邻居）才进入周边转发 ===
      NS_LOG_DEBUG("RouteOutput: True local maximum, trying perimeter routing for " << dst);
      Vector lastHopPos = myPos;
      // [2.A] 候选池预过滤：避免 RightHandRule 选出超距邻居
      uint16_t perimId = PerimeterReachableNextHop(dstPos,myPos,lastHopPos);
      if (perimId != 0xFFFF) {
        auto neighbors = m_neighbors->GetAll();
        auto it = neighbors.find(perimId);
        if (it != neighbors.end()) {
          nextHop = it->second.ipAddress;
          selectedBranch = "perimeter";
          NS_LOG_DEBUG("RouteOutput: Perimeter routing to neighbor " << perimId);
        }
      }
    }
  }

  // Final fallback: when the cluster strategies, greedy and perimeter all fail
  // to find a next hop, do not return NO_ROUTE directly. Returning nullptr
  // makes IPv4::Send raise ERROR_NOROUTETOHOST, and the packet is not counted
  // 包**不计入 FlowMonitor TX trace**，OnOffApp 的发包统计静默丢失（实测
  // tx 从 3800→2402，1371 个包消失但 NO_ROUTE drop 只 27 个）。
  // 改用 NearestReachableNeighbor 兜底——挑一个能打到的、距 dst 最近的邻居。
  // 即使 progress<0，TTL=64 也能限制最坏环路代价，且包计入 tx，统计真实。
  // 仅当本节点完全孤立（连一个 reachable 邻居都没有）才返回 nullptr。
  if (nextHop == Ipv4Address()) {
    Vector dstPos = GetPositionFromIp(dst);
    uint16_t fallbackId = NearestReachableNeighbor(dstPos);
    if (fallbackId != 0xFFFF) {
      auto it = m_neighbors->GetAll().find(fallbackId);
      if (it != m_neighbors->GetAll().end()) {
        nextHop = it->second.ipAddress;
        selectedBranch = "nearest_fallback";
        NS_LOG_DEBUG("RouteOutput: fallback nearest-reachable to " << fallbackId
                     << " for dst " << dst);
      }
    }
  }
  if (m_helloMode == QLEARNING_HELLO && m_qHello && m_controlSendDepth == 0 &&
      QLearningHelloManager::IsBusinessPacket(p, header, false)) {
    // Source-local route availability, not delivery PDR. The existing CA control
    // send scope also covers queries made before UDP adds its header.
    m_qHello->ObserveRoutingResult(nextHop != Ipv4Address());
  }
  if (nextHop == Ipv4Address()) {
    NS_LOG_DEBUG("RouteOutput: completely isolated for " << dst << " (NO_ROUTE)");
    sockerr = Socket::ERROR_NOROUTETOHOST;
    audit.Emit("no_route");
    return nullptr;
  }
  route->SetGateway(nextHop);
  route->SetOutputDevice(m_ipv4->GetNetDevice(1));
  sockerr = Socket::ERROR_NOTERROR;

  NS_LOG_DEBUG("Creating unicast route to " << dst);
  TraceDataLinkSelection(p,nextHop);
  audit.Emit(selectedBranch, nextHop);
  return route;
}

bool RoutingProtocol::RouteInput(Ptr<const Packet> p, const Ipv4Header& header,
                                  Ptr<const NetDevice> idev,
                                  const UnicastForwardCallback& ucb,
                                  const MulticastForwardCallback& mcb,
                                  const LocalDeliverCallback& lcb,
                                  const ErrorCallback& ecb)
{
  NS_LOG_FUNCTION(this << header.GetDestination());
  
  Ipv4Address dst = header.GetDestination();
  // Ipv4Address src = header.GetSource();
  
  // 本地交付
  if (m_ipv4->IsDestinationAddress(dst, m_ipv4->GetInterfaceForDevice(idev))) {
    if (!lcb.IsNull()) {
      NS_LOG_DEBUG("Local delivery to " << dst);
      lcb(p, header, m_ipv4->GetInterfaceForDevice(idev));
      return true;
    }
    return false;
  }
  
  // 广播
  for (auto k = m_socketAddrs.begin(); k != m_socketAddrs.end(); ++k) {
    auto iface = k->second;
    if (dst.IsBroadcast() || dst == iface.GetBroadcast()) {
      if (!lcb.IsNull()) {
        lcb(p, header, m_ipv4->GetInterfaceForDevice(idev));
      }
      return true;
    }
  }
  
  // TTL检查
  if (header.GetTtl() == 0) {
    NS_LOG_DEBUG("TTL expired");
    return false;
  }
  
  // 转发
  if (m_helloMode == QLEARNING_HELLO && m_qHello &&
      QLearningHelloManager::IsBusinessPacket(p, header, true)) {
    const bool forwarded = Forwarding(p, header, ucb, ecb);
    m_qHello->ObserveRoutingResult(forwarded);
    return forwarded;
  }
  return Forwarding(p, header, ucb, ecb);
}

bool RoutingProtocol::Forwarding(Ptr<const Packet> p, const Ipv4Header& header,
                                  const UnicastForwardCallback& ucb, const ErrorCallback& ecb)
{
  NS_LOG_FUNCTION(this);

  Ipv4Address dst = header.GetDestination();

  // [2.B] 簇间防环：拷贝一份可写 packet，进入函数前先读 incoming CafbcrClusterHopTag。
  // 如果是 Head 转发，最后会覆盖写自己的 ID 再交给 ucb；如果是 member 且
  // tag.lastCh == 自己 CH，说明刚从自己 CH 转过来，A3 跳过避免乒乓回送。
  Ptr<Packet> pktForFwd = p->Copy();
  CafbcrClusterHopTag incomingHopTag;
  uint16_t incomingLastCh = pktForFwd->PeekPacketTag(incomingHopTag)
                            ? incomingHopTag.GetClusterHeadId() : 0xFFFF;

  // 提前拿到本节点角色给 stamp 用
  Ptr<Node> selfNode0 = m_ipv4->GetObject<Node>();
  Ptr<ClusterNode> selfClusterNode0 = selfNode0 ? selfNode0->GetObject<ClusterNode>() : nullptr;
  NodeRole earlyRole = selfClusterNode0 ? selfClusterNode0->GetRoleType() : Idle;
  ForwardAudit audit{"forward", PeekPointer(p), header, selfNode0 ? selfNode0->GetId() : UINT32_MAX,
    static_cast<uint16_t>(earlyRole), selfClusterNode0 ? selfClusterNode0->GetClusterId() : uint16_t(0xFFFF),
    PeekPointer(m_clusterManager), PeekPointer(m_neighbors)};
  audit.incomingHead = incomingLastCh;
  audit.cacheState = "miss";

  auto history = GetForwardingHistory();
  auto recovery = GetRevisitRecovery();
  ForwardingHistory::Candidate candidate;
  RevisitRecovery::Candidate recoveryCandidate;
  if (history) {
    CafbcrComputeTaskTag classify;
    audit.task = p->PeekPacketTag(classify);
    candidate = history->Inspect(p, header);
    if (candidate.revisit) {
      audit.Emit("loop_revisit");
      return false; // RouteInput propagates false; IPv4 records DROP_NO_ROUTE.
    }
  }
  if (recovery) {
    CafbcrComputeTaskTag classify;
    audit.task = p->PeekPacketTag(classify);
    recoveryCandidate = recovery->Inspect(p, header);
  }
  auto forwardPacket = [&](Ptr<Ipv4Route> route) {
    if (history) history->Remember(std::move(candidate));
    if (recovery) recovery->Remember(std::move(recoveryCandidate), route->GetGateway());
    TraceDataLinkSelection(p,route->GetGateway(),"selected_forward");
    ucb(route, pktForFwd, header);
  };

  // 如果自己是 Head，stamp 自己 ID 进 tag——一次性写入，后续所有 ucb 路径都受益
  if (earlyRole == Head) {
    CafbcrClusterHopTag stampTag;
    pktForFwd->RemovePacketTag(stampTag);  // 覆盖之前的（如果有）
    stampTag.SetClusterHeadId(MyNodeId());
    pktForFwd->AddPacketTag(stampTag);
  }

  if (recovery && recoveryCandidate.revisit) {
    if (recoveryCandidate.tried.size() >= recovery->GetNextHopLimit()) {
      recovery->Stop(recoveryCandidate, "budget_exhausted");
      audit.Emit("revisit_budget_exhausted"); return false;
    }
    // Reuse the existing ideal destination-position service. Candidate state
    // comes only from this node's current advertisements, never peer histories.
    Vector dstPos = GetPositionFromIp(dst);
    Vector myPos = GetMyPosition();
    audit.ownPosition = myPos; audit.destinationPosition = dstPos;
    auto finite = [](const Vector& v) {
      return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
    };
    if (!finite(dstPos) || !finite(myPos)) {
      recovery->Stop(recoveryCandidate, "invalid_position");
      audit.Emit("revisit_invalid_position"); return false;
    }
    std::optional<Ipv4Address> next;
    double bestDistance = std::numeric_limits<double>::infinity();
    if (m_neighbors) {
      for (const auto& [id, info] : m_neighbors->GetAll()) {
        const auto ip = info.ipAddress;
        if (ip == Ipv4Address::GetAny() || ip == m_mainAddress || ip.IsMulticast() ||
            ip.IsBroadcast() || ip.IsLocalhost() || !finite(info.Position) ||
            !DataNeighborAllowed(info) ||
            std::find(recoveryCandidate.tried.begin(), recoveryCandidate.tried.end(), ip) !=
              recoveryCandidate.tried.end()) continue;
        double distance = CalculateDistance(info.Position, dstPos);
        // Direct neighbor first, otherwise deterministic closest-to-destination
        // order. Negative geographic progress is deliberately permitted.
        if (ip == dst) distance = -1;
        if (distance < bestDistance || (distance == bestDistance && next && ip.Get() < next->Get())) {
          next = ip; bestDistance = distance;
        }
      }
    }
    if (!next) {
      recovery->Stop(recoveryCandidate, "candidates_exhausted");
      audit.Emit("revisit_candidates_exhausted"); return false;
    }
    Ptr<Ipv4Route> route = Create<Ipv4Route>();
    route->SetDestination(dst); route->SetSource(m_mainAddress); route->SetGateway(*next);
    route->SetOutputDevice(m_ipv4->GetNetDevice(1));
    // Per-datagram alternative: do not contaminate the destination route cache.
    audit.Emit("revisit_alternate", *next);
    forwardPacket(route); return true;
  }

  // ---- Direct forwarding for compute tasks (no CH relay) ----
  // A packet carrying CafbcrComputeTaskTag bypasses the cluster structure and
  // uses GreedyReachableNextHop(dst) instead: the client has already selected a
  // nearby RELAY server, so the path is normally one or two hops. Fallback
  // tasks with a distant destination also follow a GPSR-style shortest path,
  // avoiding the extra CH hop that cross-cluster forwarding would add.
  Ptr<Packet> pCopy = p->Copy();
  CafbcrComputeTaskTag taskTag;
  if (pCopy->PeekPacketTag(taskTag)) {
    audit.task = true;
    Vector dstPos = GetPositionFromIp(dst);
    audit.destinationPosition = dstPos;
    uint16_t nh = GreedyReachableNextHop(dstPos);
    const bool fallback = nh == 0xFFFF;
    if (nh == 0xFFFF) {
      // greedy 无 progress 邻居，兜底找最近 reachable 邻居
      nh = NearestReachableNeighbor(dstPos);
    }
    if (nh != 0xFFFF) {
      auto it = m_neighbors->GetAll().find(nh);
      if (it != m_neighbors->GetAll().end()) {
        Ipv4Address nextHop = it->second.ipAddress;
        Ptr<Ipv4Route> route = Create<Ipv4Route>();
        route->SetDestination(dst);
        route->SetSource(m_mainAddress);
        route->SetGateway(nextHop);
        route->SetOutputDevice(m_ipv4->GetNetDevice(1));
        RouteCache cache;
        cache.nextHop = nextHop;
        cache.lastUpdate = Simulator::Now();
        m_routeCache[dst] = cache;
        NS_LOG_DEBUG("Task direct-greedy to " << nextHop << " task=" << taskTag.GetTaskId());
        audit.Emit(fallback ? "task_nearest" : "task_greedy", nextHop, true);
        forwardPacket(route);
        return true;
      }
    }
    NS_LOG_WARN("Task no reachable neighbor, falling through to default");
  }

  // 尝试查找缓存的路由（如果仍然有效）
  auto cacheIter = m_routeCache.find(dst);
  if (cacheIter != m_routeCache.end()) {
    Time age = Simulator::Now() - cacheIter->second.lastUpdate;
    audit.cacheAge = age;
    audit.cachedGateway = cacheIter->second.nextHop;
    audit.cacheState = "expired";
    if (age < m_routeCacheTTL) {
      audit.cacheState = "invalid_next_hop";
      // 缓存仍然有效，直接使用
      Ipv4Address nextHop = cacheIter->second.nextHop;
      uint16_t cachedId = GetNodeIdFromIp(nextHop);
      // [2.2] 缓存命中也要确认仍在 TX 范围内，节点可能已漂出
      if (m_neighbors->IsNeighbor(cachedId) && WithinMyTxRange(cachedId)) {
        // The destination-only cache must honor the same incoming-head
        // context as A3 below. A member-to-head route learned on an outgoing
        // packet is not reusable when that head sends the packet back.
        // Keep direct delivery and same-cluster policy unchanged; a rejected
        // cache entry falls through to the existing greedy/perimeter path.
        const uint16_t myCid = selfClusterNode0 ? selfClusterNode0->GetClusterId() : 0xFFFF;
        bool returnsToOwnHead = false;
        if (earlyRole != Head && myCid != 0xFFFF && incomingLastCh != 0xFFFF &&
            incomingLastCh == cachedId && nextHop != dst &&
            m_clusterManager->GetClusterHeadId(myCid) == cachedId) {
          const uint16_t dstId = GetNodeIdFromIp(dst);
          const uint16_t dstCid = dstId != 0xFFFF ? m_clusterManager->GetClusterId(dstId) : 0xFFFF;
          returnsToOwnHead = myCid != dstCid;
        }
        if (returnsToOwnHead) {
          audit.cacheState = "blocked_own_head_return";
        } else {
          Ptr<Ipv4Route> route = Create<Ipv4Route>();
          route->SetDestination(dst);
          route->SetSource(m_mainAddress);
          route->SetGateway(nextHop);
          route->SetOutputDevice(m_ipv4->GetNetDevice(1));

          NS_LOG_DEBUG("Using cached route to " << nextHop << " for dst " << dst);
          audit.cacheState = "hit";
          audit.Emit("cache", nextHop);
          forwardPacket(route);
          return true;
        }
      }
    }
    // 缓存过期或下一跳无效/超距，删除并重新计算
    m_routeCache.erase(cacheIter);
  }

  // 检查邻居表是否可用
  if (!m_neighbors || m_neighbors->IsEmpty()) {
    NS_LOG_WARN("Neighbors table empty, unable to forward to " << dst);
    audit.Emit("empty_neighbors");
    return false;
  }

  Vector dstPos = GetPositionFromIp(dst);
  Vector myPos = GetMyPosition();
  audit.ownPosition = myPos;
  audit.destinationPosition = dstPos;

  // 优化：如果目标是邻居，直接转发（避免位置计算错误）
  // [2.2] 但邻居也得在我 TX 范围内（asymmetric link 防御）
  const auto& neighbors = m_neighbors->GetAll();
  for (const auto& entry : neighbors) {
    if (entry.second.ipAddress == dst && DataNeighborAllowed(entry.second)) {
      Ipv4Address nextHop = entry.second.ipAddress;
      Ptr<Ipv4Route> route = Create<Ipv4Route>();
      route->SetDestination(dst);
      route->SetSource(m_mainAddress);
      route->SetGateway(nextHop);
      route->SetOutputDevice(m_ipv4->GetNetDevice(1));
      NS_LOG_DEBUG("Direct neighbor forward to " << nextHop);
      audit.Emit("direct_neighbor", nextHop);
      forwardPacket(route);
      return true;
    }
  }

  // === [新增 Round 3] 策略：簇感知路由（补齐与 RouteOutput 策略2/3 的对称性）===
  // 获取本节点的簇信息
  uint16_t dstNodeId2 = GetNodeIdFromIp(dst);
  uint16_t dstClusterId = (dstNodeId2 != 0xFFFF) ?
      m_clusterManager->GetClusterId(dstNodeId2) : 0xFFFF;

  Ptr<Node> selfNode = m_ipv4->GetObject<Node>();
  Ptr<ClusterNode> clusterNode = selfNode ? selfNode->GetObject<ClusterNode>() : nullptr;
  if (clusterNode && m_neighbors && !m_neighbors->IsEmpty()) {
    uint16_t myClusterId = clusterNode->GetClusterId();
    uint16_t myRole = clusterNode->GetRoleType();

    // 策略A2：自己是簇头，目标在本簇 → 贪婪转发给成员
    if (myRole == Head && myClusterId != 0xFFFF && myClusterId == dstClusterId) {
      // [2.2] 用 GreedyReachableNextHop 替代裸 GreedyNeighbor + 距离 check
      uint16_t nextHopId = GreedyReachableNextHop(dstPos);
      if (nextHopId != 0xFFFF) {
        auto it = neighbors.find(nextHopId);
        if (it != neighbors.end()) {
          Ipv4Address nextHop = it->second.ipAddress;
          RouteCache cache;
          cache.nextHop = nextHop;
          cache.lastUpdate = Simulator::Now();
          cache.hopCount = 1;
          m_routeCache[dst] = cache;
          Ptr<Ipv4Route> route = Create<Ipv4Route>();
          route->SetDestination(dst);
          route->SetSource(m_mainAddress);
          route->SetGateway(nextHop);
          route->SetOutputDevice(m_ipv4->GetNetDevice(1));
          NS_LOG_DEBUG("Forwarding (CH intra-cluster) to " << nextHop);
          audit.Emit("head_intra_greedy", nextHop, true);
          forwardPacket(route);
          return true;
        }
      }
    }
    // 策略A3：自己是成员，目标在不同簇 → 上报给本簇头
    else if (myRole != Head && myClusterId != 0xFFFF && myClusterId != dstClusterId) {
      uint16_t chId = m_clusterManager->GetClusterHeadId(myClusterId);
      // Inter-cluster loop prevention: if the packet just came from this
      // node's own CH, sending it back would create a member-CH-member ping-pong
      // loop. Skip this strategy and let greedy/perimeter forwarding terminate.
      bool fromMyOwnCh = (incomingLastCh != 0xFFFF && incomingLastCh == chId);
      // [2.2] 自己的 CH 必须在自己 TX 范围内才能上报；超距时 fall through 到下面的 greedy/perimeter
      if (!fromMyOwnCh && chId != 0xFFFF && m_neighbors->IsNeighbor(chId) && WithinMyTxRange(chId)) {
        Ipv4Address chIp = NodeList::GetNode(chId)->GetObject<Ipv4>()
                           ->GetAddress(1, 0).GetLocal();
        RouteCache cache;
        cache.nextHop = chIp;
        cache.lastUpdate = Simulator::Now();
        cache.hopCount = 1;
        m_routeCache[dst] = cache;
        Ptr<Ipv4Route> route = Create<Ipv4Route>();
        route->SetDestination(dst);
        route->SetSource(m_mainAddress);
        route->SetGateway(chIp);
        route->SetOutputDevice(m_ipv4->GetNetDevice(1));
        NS_LOG_DEBUG("Forwarding (member inter-cluster) via CH " << chId);
        audit.Emit("member_via_head", chIp, true);
        forwardPacket(route);
        return true;
      }
    }
  }

  NS_LOG_DEBUG("Forwarding: dst=" << dst << " dstPos=" << dstPos << " myPos=" << myPos);

  // 使用贪婪路由选择最接近目标的邻居
  // 注意：无法可靠地从packet header中获取前一跳节点ID，因为src地址在转发过程中不变
  // 因此不进行前一跳避免（GreedyReachableNextHop会通过不断靠近目标位置来自然地防止环路）
  // [2.2] GreedyReachableNextHop 已经在自己的 TX 范围内挑 progress 最大的邻居
  uint16_t nextHopId = GreedyReachableNextHop(dstPos);

  if (nextHopId != 0xFFFF) {
    auto neighbors = m_neighbors->GetAll();
    auto it = neighbors.find(nextHopId);
    if (it != neighbors.end()) {
      Ipv4Address nextHop = it->second.ipAddress;

      // 更新缓存
      RouteCache cache;
      cache.nextHop = nextHop;
      cache.lastUpdate = Simulator::Now();
      cache.hopCount = 1;
      m_routeCache[dst] = cache;

      Ptr<Ipv4Route> route = Create<Ipv4Route>();
      route->SetDestination(dst);
      route->SetSource(m_mainAddress);
      route->SetGateway(nextHop);
      route->SetOutputDevice(m_ipv4->GetNetDevice(1));

      NS_LOG_DEBUG("Greedy forwarding to " << nextHop << " (position-based) for dst " << dst);
      audit.Emit("greedy", nextHop, true);
      forwardPacket(route);
      return true;
    } else {
      NS_LOG_WARN("GreedyNeighbor returned " << nextHopId << " but not found in neighbors map");
    }
  } else {
    NS_LOG_DEBUG("GreedyReachableNextHop returned 0xFFFF (local maximum or empty in-range pool) for dst "
        << dst << ", trying perimeter routing");

    // ===== 真正的局部最大值时 void recovery via PerimeterNextHopWithinRange =====
    // [2.A] 候选池预过滤，避免 RightHandRule 选出超距邻居造成无效转发
    Vector lastHopPos = myPos;  // 进入周边模式时的基准位置
    uint16_t perimeterNextHopId = PerimeterReachableNextHop(dstPos,myPos,lastHopPos);

    if (perimeterNextHopId != 0xFFFF) {
      auto neighbors = m_neighbors->GetAll();
      auto it = neighbors.find(perimeterNextHopId);
      if (it != neighbors.end()) {
        Ipv4Address nextHop = it->second.ipAddress;

        // 更新缓存
        RouteCache cache;
        cache.nextHop = nextHop;
        cache.lastUpdate = Simulator::Now();
        cache.hopCount = 1;
        m_routeCache[dst] = cache;

        Ptr<Ipv4Route> route = Create<Ipv4Route>();
        route->SetDestination(dst);
        route->SetSource(m_mainAddress);
        route->SetGateway(nextHop);
        route->SetOutputDevice(m_ipv4->GetNetDevice(1));

        NS_LOG_DEBUG("Perimeter (GPSR) routing to " << nextHop
            << " (Node " << perimeterNextHopId << ") for dst " << dst);
        audit.Emit("perimeter", nextHop, true);
        forwardPacket(route);
        return true;
      }
    }
  }

  NS_LOG_WARN("No route found (greedy + perimeter) for " << dst);
  audit.Emit("no_route");
  return false;
}

//-----------------------------------------------------------------------------
// 辅助函数
//-----------------------------------------------------------------------------
double RoutingProtocol::UpdateNodeFitness()
{
  NS_LOG_FUNCTION(this);

  Ptr<Node> node = m_ipv4->GetObject<Node>();
  Ptr<ClusterNode> clusterNode = node->GetObject<ClusterNode>();

  if (!clusterNode) {
    return 0.0;
  }

  // 在计算fitness前检查邻居数量
  size_t neighborCount = m_neighbors ? m_neighbors->GetAll().size() : 0;
  NS_LOG_DEBUG("Node " << node->GetId() << " UpdateNodeFitness: has "
               << neighborCount << " neighbors at time " << Simulator::Now().GetSeconds());

  // ABC算法中，Fitness由ClusterNode的ComputeFitness计算
  // 权重由cafbcrHelper从ScenarioConfig传入，与cafbcrElection::WeightTable保持一致
  double fitness = clusterNode->ComputeFitness(m_wLHTP, m_wEnergy,
                                               m_wDegree, m_wDistance, m_wCompute);
  clusterNode->UpdateFitness(fitness);

  return fitness;
}

Vector RoutingProtocol::GetMyPosition()
{
  Ptr<Node> node = m_ipv4->GetObject<Node>();
  Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();

  if (mobility) {
    return mobility->GetPosition();
  }

  return Vector(0, 0, 0);
}

// Asymmetric-link distance gate: whether this node can reach pos at its own
// TX power. The router checks it before writing a next hop, so a distant
// asymmetric neighbor or a CH that has drifted out of range is not treated as
// a valid next hop.
// 否则 MAC 必然 7 次重传后 REACHED_RETRY_LIMIT。
bool RoutingProtocol::WithinMyTxRange(const Vector& pos)
{
  return !IsRangeGatingEnabled() || WithinConfiguredRange(pos);
}

bool RoutingProtocol::WithinConfiguredRange(const Vector& pos)
{
  Vector my = GetMyPosition();
  double dx = pos.x - my.x;
  double dy = pos.y - my.y;
  double dz = pos.z - my.z;
  return (dx * dx + dy * dy + dz * dz) <= (m_maxRange * m_maxRange);
}

bool RoutingProtocol::WithinConfiguredRange(uint16_t neighborId)
{
  if (!m_neighbors) return false;
  const auto& table = m_neighbors->GetAll();
  auto it = table.find(neighborId);
  return it != table.end() && WithinConfiguredRange(it->second.Position);
}

bool RoutingProtocol::WithinMyTxRange(uint16_t neighborId)
{
  if (!m_neighbors) return false;
  const auto& table = m_neighbors->GetAll();
  auto it = table.find(neighborId);
  if (it == table.end()) return false;
  return DataNeighborAllowed(it->second);
}

// [2.2] 限范围内的 greedy。fanet-neighbor::GreedyNeighbor 不知道 m_maxRange，
// 经常选出"最优但超距"的 next-hop；我们在自己 TX 范围内重新找 progress 最大
// 的邻居，避免无谓地跌进周边模式。语义跟 GreedyNeighbor 等价（progress > 0 +
// 距离/能量/链路质量加权），只是邻居池被 m_maxRange 截断。
uint16_t RoutingProtocol::GreedyReachableNextHop(const Vector& dstPos)
{
  return GreedyNextHop(dstPos,IsRangeGatingEnabled() && !MeasuredDataGateActive(),MeasuredDataGateActive());
}

uint16_t RoutingProtocol::GreedyNextHop(const Vector& dstPos, bool filterConfiguredRange,bool filterMeasured)
{
  if (!m_neighbors) return 0xFFFF;
  Vector myPos = GetMyPosition();
  double myDist = CalculateDistance(myPos, dstPos);
  uint16_t bestId = 0xFFFF;
  double bestProgress = 0.0;
  Time now = Simulator::Now();

  for (const auto& kv : m_neighbors->GetAll()) {
    const auto& info = kv.second;
    // 距离守门：超出我的 TX 范围直接跳过
    if (filterConfiguredRange && !WithinConfiguredRange(info.Position)) continue;
    if (filterMeasured && !DataNeighborAllowed(info)) continue;

    double nbrDist = CalculateDistance(info.Position, dstPos);
    double progress = myDist - nbrDist;
    if (progress <= 0.0) continue;

    // 与 fanet-neighbor::GreedyNeighbor 同构的加权：簇头 1.5×、剩余能量、
    // 时新性、链路质量、负载，全部沿用以保持选择行为一致
    double weigh = 1.0;
    if (info.Role == Head) weigh *= 1.5;
    weigh *= (info.ResEnergy / 100.0);
    if ((now - info.UpdateTime) < Seconds(1.0)) weigh *= 1.2;

    double dx = info.Position.x - myPos.x;
    double dy = info.Position.y - myPos.y;
    double dz = info.Position.z - myPos.z;
    double distKm = std::sqrt(dx * dx + dy * dy + dz * dz) / 1000.0;
    double linkScore = (distKm <= 0.5) ? 1.0
                       : (distKm > 5.0) ? 0.0 : std::exp(-distKm);
    double loadScore = 1.0 - static_cast<double>(info.currentLoad);
    // Missing/expired computation contributes no load bonus; geographic routing
    // remains available and never treats unknown as an observed empty queue.
    weigh *= (0.7 * linkScore +
              (m_neighbors->HasUsableComputeState(kv.first) ? 0.3 * loadScore : 0.0));

    double weightedProgress = progress * weigh;
    if (weightedProgress > bestProgress) {
      bestProgress = weightedProgress;
      bestId = kv.first;
    }
  }
  return bestId;
}

// Fallback next hop: pick the neighbor closest to dst within TX range, even
// when its progress is not positive.
// 用途：RouteOutput 末尾如果所有 cluster 策略 + greedy + perimeter 都没找到
// 合规 next-hop，与其返回 nullptr 让 OnOffApp 的 tx 静默丢失（包不计入
// FlowMonitor），不如挑一个能打到的邻居把包扔出去——靠 TTL 限定环路风险，
// 至少包计入 tx 统计且有机会到达。等价于"路由不知道怎么走但还是试一下"。
uint16_t RoutingProtocol::NearestReachableNeighbor(const Vector& dstPos)
{
  if (!m_neighbors) return 0xFFFF;
  uint16_t bestId = 0xFFFF;
  double bestDist = std::numeric_limits<double>::max();
  for (const auto& kv : m_neighbors->GetAll()) {
    const auto& info = kv.second;
    if (!DataNeighborAllowed(info)) continue;
    double dx = info.Position.x - dstPos.x;
    double dy = info.Position.y - dstPos.y;
    double dz = info.Position.z - dstPos.z;
    double d2 = dx * dx + dy * dy + dz * dz;
    if (d2 < bestDist) {
      bestDist = d2;
      bestId = kv.first;
    }
  }
  return bestId;
}

bool RoutingProtocol::MeasuredDataGateActive() const
{ return m_linkProbes && m_handshakeGate && IsRangeGatingEnabled() && m_controlSendDepth==0; }

std::string RoutingProtocol::GetLinkEvidenceState(uint16_t peer) const
{
  if (!m_linkProbes) return "disabled";
  if (!m_neighbors) return "neighbor_missing";
  const auto& neighbors=m_neighbors->PeekAll();auto neighbor=neighbors.find(peer);const auto now=Simulator::Now();
  if (neighbor==neighbors.end() || neighbor->second.UpdateTime+neighbor->second.NeighborLifeLength<=now)
    return "neighbor_missing";
  auto proof=m_linkConfirmed.find(peer);
  if (proof==m_linkConfirmed.end()) return m_linkPending.count(peer) ? "pending" : "unknown";
  if (proof->second.ip!=neighbor->second.ipAddress) return "ip_changed";
  if (proof->second.sent>now || now>=proof->second.sent+m_linkEvidenceValidity) return "expired";
  return "confirmed";
}

bool RoutingProtocol::DataNeighborAllowed(const Neighbors::NeighborInfo& info)
{
  if (!MeasuredDataGateActive()) return WithinMyTxRange(info.Position);
  const bool admitted=GetLinkEvidenceState(info.NodeId)=="confirmed";
  auto p=m_linkConfirmed.find(info.NodeId);
  TraceLink("candidate",info.NodeId,info.ipAddress,
    p==m_linkConfirmed.end()?std::nullopt:std::optional<uint64_t>(p->second.nonce),std::nullopt,
    p==m_linkConfirmed.end()?std::nullopt:std::optional<Time>(p->second.sent),
    p==m_linkConfirmed.end()?std::nullopt:std::optional<Time>(p->second.sent+m_linkEvidenceValidity),admitted);
  return admitted;
}

uint16_t RoutingProtocol::PerimeterReachableNextHop(Vector dst,Vector my,Vector& previous)
{
  if (!m_neighbors) return 0xffff;
  if (!MeasuredDataGateActive())
    return m_neighbors->PerimeterNextHopWithinRange(dst,my,previous,IsRangeGatingEnabled()?m_maxRange:1e7);
  // Same XY angle/tie ordering as the existing helper; only eligibility differs.
  uint16_t best=0xffff;double angleBest=2*std::acos(-1.0);const Vector toDst=dst-my;
  for (const auto& kv:m_neighbors->GetAll()) {
    if (!DataNeighborAllowed(kv.second)) continue;
    const Vector v=kv.second.Position-my;
    double angle=std::atan2(toDst.x*v.y-toDst.y*v.x,toDst.x*v.x+toDst.y*v.y);
    if (angle<0) angle+=2*std::acos(-1.0);
    if (best==0xffff || angle<angleBest) { best=kv.first;angleBest=angle; }
  }
  return best;
}

int RoutingProtocol::SendControlPacket(Ptr<Socket> socket,Ptr<Packet> packet,Ipv4Address destination)
{
  if (m_helloMode == QLEARNING_HELLO && m_qHello)
    QLearningHelloManager::MarkOwnControlPacket(packet);
  // Source-local traffic context, not a packet UID/tag or remote-node oracle.
  struct Scope { uint32_t& depth;explicit Scope(uint32_t& d):depth(d){++depth;}~Scope(){--depth;} } scope(m_controlSendDepth);
  return socket->SendTo(packet,0,InetSocketAddress(destination,CAFBCR_PORT));
}

void RoutingProtocol::TraceLink(const char* event,uint16_t peer,Ipv4Address ip,
    std::optional<uint64_t> nonce,std::optional<uint64_t> uid,std::optional<Time> sent,
    std::optional<Time> deadline,std::optional<bool> admitted)
{
  if (m_linkProbeEvent.IsEmpty() || !m_ipv4) return;
  std::optional<Time> received;auto p=m_linkConfirmed.find(peer);
  if (p!=m_linkConfirmed.end()) received=p->second.received;
  m_linkProbeEvent(LinkProbeEvent{Simulator::Now(),MyNodeId(),peer,ip,event,GetLinkEvidenceState(peer),
    nonce,uid,sent,deadline,received,admitted});
}

void RoutingProtocol::TraceDataLinkSelection(Ptr<const Packet> packet,Ipv4Address gateway,const char* event)
{
  if (!m_linkProbes || m_controlSendDepth || m_linkProbeEvent.IsEmpty() || !m_neighbors) return;
  for (const auto& kv:m_neighbors->PeekAll()) if (kv.second.ipAddress==gateway) {
    auto p=m_linkConfirmed.find(kv.first);
    TraceLink(event,kv.first,gateway,p==m_linkConfirmed.end()?std::nullopt:std::optional<uint64_t>(p->second.nonce),
      packet->GetUid(),p==m_linkConfirmed.end()?std::nullopt:std::optional<Time>(p->second.sent),
      p==m_linkConfirmed.end()?std::nullopt:std::optional<Time>(p->second.sent+m_linkEvidenceValidity),true);
    return;
  }
  TraceLink("selected_missing_neighbor",0xffff,gateway,std::nullopt,packet->GetUid());
}

void RoutingProtocol::LinkProbeTick()
{
  if (!m_linkProbes || !m_ipv4 || m_socketAddrs.empty() || !m_neighbors) return;
  const auto now=Simulator::Now();const auto& neighbors=m_neighbors->GetAll();
  for (auto p=m_linkConfirmed.begin();p!=m_linkConfirmed.end();) {
    if (!neighbors.count(p->first) && now>=p->second.sent+m_linkEvidenceValidity) p=m_linkConfirmed.erase(p);
    else ++p;
  }
  for (const auto& kv:neighbors) {
    const auto& ip=kv.second.ipAddress;
    if (kv.first==MyNodeId() || kv.first==0xffff || m_linkPending.count(kv.first) ||
        ip==m_mainAddress || ip==Ipv4Address::GetAny() || ip.IsBroadcast() || ip.IsMulticast() || ip.IsLocalhost()) continue;
    const auto nonce=++m_linkNonce;
    auto result=m_linkPending.emplace(kv.first,PendingLinkProbe{ip,nonce,now,EventId()});
    result.first->second.timeout=Simulator::Schedule(m_linkProbeTimeout,&RoutingProtocol::LinkProbeExpired,this,kv.first,nonce);
    TraceLink("request_scheduled",kv.first,ip,nonce,std::nullopt,now,now+m_linkProbeTimeout);
    SendLinkProbe(LinkProbeHeader(MyNodeId(),kv.first,nonce,now.GetNanoSeconds()),ip,false);
  }
  m_linkProbeTimer=Simulator::Schedule(m_linkProbeInterval,&RoutingProtocol::LinkProbeTick,this);
}

void RoutingProtocol::SendLinkProbe(const LinkProbeHeader& header,Ipv4Address peer,bool reply)
{
  if (!m_linkProbes || !m_ipv4 || m_socketAddrs.empty()) return;
  const auto iface=m_socketAddrs.begin()->second;
  const auto index=m_ipv4->GetInterfaceForAddress(iface.GetLocal());
  if (index<0 || !m_ipv4->IsUp(index)) return;
  auto p=Create<Packet>();p->AddHeader(header);p->AddHeader(TypeHeader(reply?CAFBCR_LINK_REPLY:CAFBCR_LINK_REQUEST));
  const auto bodyBytes=p->GetSize();
  UdpHeader udp;udp.SetSourcePort(CAFBCR_PORT);udp.SetDestinationPort(CAFBCR_PORT);
  if (Node::ChecksumEnabled()) { udp.EnableChecksums();udp.InitializeChecksum(iface.GetLocal(),peer,17); }
  p->AddHeader(udp);SocketIpTtlTag ttl;ttl.SetTtl(1);p->AddPacketTag(ttl);
  auto route=Create<Ipv4Route>();route->SetSource(iface.GetLocal());route->SetDestination(peer);
  route->SetGateway(peer);route->SetOutputDevice(m_ipv4->GetNetDevice(index));
  const auto id=reply?header.GetOrigin():header.GetPeer();
  TraceLink(reply?"reply_ip_submit":"request_ip_submit",id,peer,header.GetNonce(),p->GetUid(),
      NanoSeconds(header.GetSentNs()),NanoSeconds(header.GetSentNs())+m_linkProbeTimeout);
  ++m_linkProbeCount;m_ControlPacketSize+=bodyBytes;
  // Explicit link-local route bootstraps measurement. Actual ARP/MAC/PHY and
  // UDP delivery run normally; an IPv4 submission is not a successful probe.
  m_ipv4->Send(p,iface.GetLocal(),peer,17,route);
}

void RoutingProtocol::RecvLinkProbe(Ptr<Packet> packet,Ipv4Address source,bool reply)
{
  if (!m_linkProbes) return;
  if (packet->GetSize()!=20) { TraceLink("malformed",0xffff,source,std::nullopt,packet->GetUid());return; }
  LinkProbeHeader header;packet->RemoveHeader(header);const auto now=Simulator::Now();
  const auto peer=reply?header.GetPeer():header.GetOrigin();
  const auto nonce=header.GetNonce();const auto uid=packet->GetUid();
  if (peer==0xffff || peer==MyNodeId() || nonce==0 ||
      header.GetSentNs()>static_cast<uint64_t>(now.GetNanoSeconds())) {
    TraceLink("invalid_identity_or_time",peer,source,nonce,uid);return;
  }
  const auto sent=NanoSeconds(header.GetSentNs());
  TraceLink(reply?"reply_received":"request_received",peer,source,nonce,uid,sent,sent+m_linkProbeTimeout);
  if (!reply) {
    if (header.GetPeer()!=MyNodeId() || now>=sent+m_linkProbeTimeout) {
      TraceLink("request_rejected",peer,source,nonce,uid,sent,sent+m_linkProbeTimeout);return;
    }
    SendLinkProbe(header,source,true);return;
  }
  auto pending=m_linkPending.find(peer);
  if (header.GetOrigin()!=MyNodeId() || pending==m_linkPending.end() || pending->second.ip!=source ||
      pending->second.nonce!=nonce || pending->second.sent!=sent || now>=sent+m_linkProbeTimeout) {
    TraceLink("reply_rejected",peer,source,nonce,uid,sent,sent+m_linkProbeTimeout);return;
  }
  const auto& neighbors=m_neighbors->GetAll();auto neighbor=neighbors.find(peer);
  if (neighbor==neighbors.end() || neighbor->second.ipAddress!=source) {
    TraceLink("reply_neighbor_missing",peer,source,nonce,uid,sent,sent+m_linkProbeTimeout);return;
  }
  Simulator::Cancel(pending->second.timeout);m_linkPending.erase(pending);
  m_linkConfirmed.insert_or_assign(peer,LinkConfirmation{source,nonce,sent,now});
  TraceLink("reply_accepted",peer,source,nonce,uid,sent,sent+m_linkProbeTimeout);
}

void RoutingProtocol::LinkProbeExpired(uint16_t peer,uint64_t nonce)
{
  auto p=m_linkPending.find(peer);if (p==m_linkPending.end() || p->second.nonce!=nonce) return;
  const auto value=p->second;m_linkPending.erase(p);
  TraceLink("timeout",peer,value.ip,nonce,std::nullopt,value.sent,value.sent+m_linkProbeTimeout);
}

void RoutingProtocol::ClearLinkProbes()
{
  Simulator::Cancel(m_linkProbeTimer);
  for (const auto& kv:m_linkPending) {
    Simulator::Cancel(kv.second.timeout);
    TraceLink("cancelled",kv.first,kv.second.ip,kv.second.nonce,std::nullopt,kv.second.sent,kv.second.sent+m_linkProbeTimeout);
  }
  m_linkPending.clear();m_linkConfirmed.clear();
}

Vector RoutingProtocol::GetVelocity()
{
  Ptr<Node> node = m_ipv4->GetObject<Node>();
  Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
  
  if (mobility) {
    return mobility->GetVelocity();
  }
  
  return Vector(0, 0, 0);
}

double RoutingProtocol::GetResidualEnergy()
{
  Ptr<Node> node = m_ipv4->GetObject<Node>();
  Ptr<EnergySourceContainer> sources = node->GetObject<EnergySourceContainer>();
  
  if (sources && sources->GetN() > 0) {
    return sources->Get(0)->GetRemainingEnergy();
  }
  
  return 0.0;
}

Vector RoutingProtocol::GetPositionFromIp(Ipv4Address addr)
{
  uint16_t nodeId = GetNodeIdFromIp(addr);
  if (nodeId != 0xFFFF && nodeId < NodeList::GetNNodes()) {
    Ptr<Node> node = NodeList::GetNode(nodeId);
    Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
    if (mobility) {
      return mobility->GetPosition();
    }
  }
  
  return Vector(0, 0, 0);
}


double RoutingProtocol::GetControlThroughput()
{
  double thp = 0.0;
  thp =(double) m_ControlPacketSize / 1000;
  return thp;
  /*HelloHeader hello;
  ClusterHeadAnnounceHeader  chCall;
  JoinClusterRequestHeader   joinReq;
  ClusterHeadResponseHeader  joinResp;
  ClusterHeadBeaconHeader    chLive;
  ClusterMemberBeaconHeader  cmLive;
  uint64_t result = GetHelloCount()        * hello.GetSerializedSize() 
                  + GetChCallCount()       * chCall.GetSerializedSize()
                  + GetJoinReqCount()      * joinReq.GetSerializedSize()
                  + GetJoinReqCount()      * joinResp.GetSerializedSize()
                  + GetCmBeaconCount()     * cmLive.GetSerializedSize()
                  + GetChBeaconCount()     * chLive.GetSerializedSize();
  return thp =(double) result / 1000;*/

}



uint16_t RoutingProtocol::GetNodeIdFromIp(Ipv4Address addr)
{
  for (uint32_t i = 0; i < NodeList::GetNNodes(); ++i) {
    Ptr<Node> node = NodeList::GetNode(i);
    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();

    if (ipv4) {
      // 检查所有接口（包括接口0）以支持不同配置
      for (uint32_t j = 0; j < ipv4->GetNInterfaces(); ++j) {
        if (ipv4->GetAddress(j, 0).GetLocal() == addr) {
          NS_LOG_DEBUG("GetNodeIdFromIp: Found node " << i << " for address " << addr << " on interface " << j);
          return static_cast<uint16_t>(i);
        }
      }
    }
  }

  NS_LOG_WARN("GetNodeIdFromIp: Could not find node for address " << addr);
  return 0xFFFF;
}

double RoutingProtocol::CalculateLHTP(Vector aPos, Vector aVel, Vector bPos, Vector bVel)
{
  // LHTP计算 - 移植自biocluster的实现
  Vector dPos = bPos - aPos;    // 相对位置
  Vector dVel = bVel - aVel;    // 相对速度

  auto dot = [](Vector u, Vector v) {
    return u.x * v.x + u.y * v.y + u.z * v.z;
  };

  double d0 = dot(dPos, dPos);  // 初始距离平方
  double r2 = m_maxRange * m_maxRange;

  if (d0 > r2) { return 0.0; }  // 已经超出通信范围

  double a = dot(dVel, dVel);
  double b = 2 * dot(dPos, dVel);
  double c = d0 - r2;

  if (a == 0.0) { return 10000.0; }  // 相对静止，返回大值

  double discriminant = b*b - 4*a*c;
  if (discriminant < 0) { return 10000.0; }  // 永远不会超出

  double sqrtD = std::sqrt(discriminant);
  double t1 = (-b - sqrtD) / (2*a);
  double t2 = (-b + sqrtD) / (2*a);

  double result = 10000.0;
  if (t1 > 0) result = t1;
  if (t2 > 0 && t2 < result) result = t2;

  if (result == 10000.0)
    return 0.0;

  return result;
}

bool RoutingProtocol::IsMyOwnAddress(Ipv4Address src)
{
  for (auto it = m_socketAddrs.begin(); it != m_socketAddrs.end(); ++it) {
    if (src == it->second.GetLocal()) {
      return true;
    }
  }
  return false;
}

Ptr<Socket> RoutingProtocol::FindSocketWithInterfaceAddress(Ipv4InterfaceAddress iface) const
{
  for (auto it = m_socketAddrs.begin(); it != m_socketAddrs.end(); ++it) {
    if (it->second == iface) {
      return it->first;
    }
  }
  
  return nullptr;
}

Ptr<Ipv4Route> RoutingProtocol::LoopbackRoute(const Ipv4Header& header, Ptr<NetDevice> oif) const
{
  NS_LOG_FUNCTION(this << header);
  
  NS_ASSERT(m_lo != nullptr);
  Ptr<Ipv4Route> rt = Create<Ipv4Route>();
  rt->SetDestination(header.GetDestination());
  
  auto j = m_socketAddrs.begin();
  if (oif) {
    for (j = m_socketAddrs.begin(); j != m_socketAddrs.end(); ++j) {
      Ipv4InterfaceAddress iface = j->second;
      int32_t interface = m_ipv4->GetInterfaceForAddress(iface.GetLocal());
      if (oif == m_ipv4->GetNetDevice(static_cast<uint32_t>(interface))) {
        rt->SetSource(iface.GetLocal());
        break;
      }
    }
  } else {
    rt->SetSource(j->second.GetLocal());
  }
  
  rt->SetGateway(Ipv4Address("127.0.0.1"));
  rt->SetOutputDevice(m_lo);
  
  return rt;
}

void RoutingProtocol::NotifyTxError(WifiMacDropReason reason, Ptr<const WifiMpdu> mpdu)
{
  NS_LOG_FUNCTION(this);
  
  if (m_neighbors) {
    m_neighbors->GetTxErrorCallBack()(mpdu->GetHeader());
  }
}

void RoutingProtocol::SendTo(Ptr<Socket> socket, Ptr<Packet> packet, Ipv4Address dst)
{
  SendControlPacket(socket,packet,dst);
}

void RoutingProtocol::SelectNearestClusterHeadAndJoin()
{
  NS_LOG_FUNCTION(this);

  Ptr<Node> node = m_ipv4->GetObject<Node>();
  Ptr<ClusterNode> clusterNode = node->GetObject<ClusterNode>();

  if (!clusterNode) return;

  // 检查是否还是Idle
  NodeRole myRole = clusterNode->GetRoleType();
  if (myRole != Idle) {
    NS_LOG_INFO("Node " << MyNodeId() << " is no longer Idle (role=" << (int)myRole
                << "), skip joining");
    return;
  }

  // 检查是否有候选簇头
  if (m_chCandidates.empty()) {
    NS_LOG_WARN("Node " << MyNodeId() << " has no CH candidates");
    // 复位标志位，下一轮 CH_CALL 才能重新触发本函数（否则 m_chCandidates
    // 会在节点反复 Idle 时单调增长直到 OOM）
    m_hasReceivedChCall = false;
    return;
  }

  // [D-γ] 两轮筛选：reachable 优先，multi-hop reachable 兜底。
  // Tier 1: 距离 ≤ m_maxRange 的 CH（SWARM 单跳就能发 JOIN_REQ）
  // Tier 2: 超距但通过 GreedyReachableNextHop 多跳能到达的 CH
  //         （JOIN_REQ 走 RouteOutput 多跳路径，路由层守门保 MAC 不死）
  // 都没有则放弃，等下一轮 CH_LIVE/CH_CALL 触发。
  // 用 candidate.position 的瞬时距离重算，避免 1s 延迟里节点位移导致误判。
  Vector myPosNow = GetMyPosition();
  std::vector<ClusterHeadCandidate> tier1;
  std::vector<ClusterHeadCandidate> tier2;
  for (const auto& c : m_chCandidates) {
    double dx = c.position.x - myPosNow.x;
    double dy = c.position.y - myPosNow.y;
    double dz = c.position.z - myPosNow.z;
    double d = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (d <= m_maxRange) {
      tier1.push_back(c);
    } else if (GreedyNextHop(c.position, true) != 0xFFFF) {
      tier2.push_back(c);  // 超距但多跳可达
    }
  }

  std::vector<ClusterHeadCandidate>* picked = nullptr;
  const char* tierName = "";
  if (!tier1.empty()) { picked = &tier1; tierName = "Tier1-reachable"; }
  else if (!tier2.empty()) { picked = &tier2; tierName = "Tier2-multihop"; }

  if (!picked) {
    NS_LOG_INFO("Node " << MyNodeId()
                << " no reachable nor multi-hop CH; will retry on next CH_LIVE/CH_CALL");
    m_chCandidates.clear();
    m_hasReceivedChCall = false;
    return;
  }

  // 选 fitness 最高的
  std::sort(picked->begin(), picked->end(),
            [](const ClusterHeadCandidate& a, const ClusterHeadCandidate& b) {
              return a.fitness > b.fitness;
            });
  const ClusterHeadCandidate& best = (*picked)[0];
  NS_LOG_INFO("Node " << MyNodeId() << " selecting CH " << best.nodeId
              << " (" << tierName << ", fitness=" << best.fitness
              << ", distance=" << best.distance << "m)");

  // [E2 piggyback] 设 intent，让 hello 携带广播；同时如果本节点物理可达 CH，
  // 立即自代理发 PROXY_JOIN_REQ（跳过 hello 周期），加速 Tier1 入簇。
  m_intentJoinChId = IsProxyJoinEnabled() ? best.nodeId : 0xFFFF;
  TraceJoin("join_selected", MyNodeId(), best.nodeId, 0xFFFF, best.ipAddress);
  if (!IsProxyJoinEnabled()) {
    SendClusterJoinRequest(best.ipAddress);
  } else if (m_neighbors->IsNeighbor(best.nodeId) && WithinConfiguredRange(best.nodeId)) {
    Ptr<Node> me = m_ipv4->GetObject<Node>();
    Ptr<ClusterNode> cn = me ? me->GetObject<ClusterNode>() : nullptr;
    if (cn) {
      SendProxyJoinRequest(MyNodeId(), m_mainAddress, GetMyPosition(), GetVelocity(),
                           GetResidualEnergy(), cn->GetFitnessValue(), best.ipAddress);
    }
  }

  // 清空候选列表 + 复位标志位
  m_chCandidates.clear();
  m_hasReceivedChCall = false;
}

void RoutingProtocol::ProactivelyJoinCluster()
{
  NS_LOG_FUNCTION(this);

  Ptr<Node> node = m_ipv4->GetObject<Node>();
  Ptr<ClusterNode> clusterNode = node->GetObject<ClusterNode>();

  if (!clusterNode) return;

  // [D-α] 周期性入簇重评估：原版本只在 t=2.5s 调度一次，导致 SWARM 一旦
  // 在初始时刻找不到 reachable CH 就永远卡 Idle 或 isolated cluster。
  // 现在每 5s 自调度一次，让节点在移动 / 邻居拓扑变化时能动态加入新出现的 CH。
  // 已入簇 (Member/Head/BackupHead) 的节点直接返回——不重评估也不重新调度，
  // 避免对稳定簇结构造成扰动。退簇由 RejoinClusterAfterTimeout 走另一条路径。
  static constexpr double kRejoinPeriodSec = 5.0;
  if (clusterNode->GetRoleType() != Idle) {
    return;  // 已入簇，不再周期性重评估
  }
  Simulator::Schedule(Seconds(kRejoinPeriodSec),
                      &RoutingProtocol::ProactivelyJoinCluster, this);

  // 检查邻居表状态
  uint32_t neighborCount = 0;
  uint32_t idleNeighborCount = 0;

  if (m_neighbors) {
    auto neighborTable = m_neighbors->GetAll();
    neighborCount = neighborTable.size();

    NS_LOG_INFO("Node " << MyNodeId() << " ProactivelyJoinCluster at " << Simulator::Now().GetSeconds()
                << "s, has " << neighborCount << " neighbors");

    for (auto it = neighborTable.begin(); it != neighborTable.end(); ++it) {
      NS_LOG_INFO("  Neighbor " << it->second.NodeId << " role=" << (int)it->second.Role
                  << " fitness=" << it->second.FitnessValue);
      if (it->second.Role == Idle) {
        idleNeighborCount++;
      }
    }
  } else {
    NS_LOG_WARN("Node " << MyNodeId() << " ProactivelyJoinCluster: neighbors is NULL");
  }

  // CAFBCR改进: 查找Fitness最高的BackupHead或Head邻居
  // [D-γ] 两轮筛选：reachable 优先，multi-hop reachable 兜底
  double tier1MaxFit = -1.0, tier2MaxFit = -1.0;
  Ipv4Address tier1Addr, tier2Addr;
  uint16_t tier1NodeId = 0xFFFF, tier2NodeId = 0xFFFF;
  Vector myPos = GetMyPosition();

  if (m_neighbors && !m_neighbors->IsEmpty()) {
    auto neighborTable = m_neighbors->GetAll();
    for (auto it = neighborTable.begin(); it != neighborTable.end(); ++it) {
      const auto& neighborInfo = it->second;
      if (neighborInfo.Role != BackupHead && neighborInfo.Role != Head) continue;

      double dx = neighborInfo.Position.x - myPos.x;
      double dy = neighborInfo.Position.y - myPos.y;
      double dz = neighborInfo.Position.z - myPos.z;
      double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

      if (distance <= m_maxRange) {
        // Tier 1：单跳可达
        if (neighborInfo.FitnessValue > tier1MaxFit) {
          tier1MaxFit = neighborInfo.FitnessValue;
          tier1Addr = neighborInfo.ipAddress;
          tier1NodeId = neighborInfo.NodeId;
        }
      } else if (GreedyNextHop(neighborInfo.Position, true) != 0xFFFF) {
        // Tier 2：超距但多跳可达（JOIN_REQ 走 RouteOutput 多跳）
        if (neighborInfo.FitnessValue > tier2MaxFit) {
          tier2MaxFit = neighborInfo.FitnessValue;
          tier2Addr = neighborInfo.ipAddress;
          tier2NodeId = neighborInfo.NodeId;
        }
      }
    }
  }

  Ipv4Address bestChAddr;
  double maxFitness = -1.0;
  const char* tierName = "";
  if (tier1Addr.IsInitialized()) {
    bestChAddr = tier1Addr; maxFitness = tier1MaxFit; tierName = "Tier1-reachable";
    NS_LOG_INFO("Node " << MyNodeId() << " Tier1 CH=" << tier1NodeId);
  } else if (tier2Addr.IsInitialized()) {
    bestChAddr = tier2Addr; maxFitness = tier2MaxFit; tierName = "Tier2-multihop";
    NS_LOG_INFO("Node " << MyNodeId() << " Tier2 CH=" << tier2NodeId
                << " (multi-hop, JOIN_REQ via RouteOutput)");
  }

  uint16_t bestChNodeId = tier1Addr.IsInitialized() ? tier1NodeId
                          : (tier2Addr.IsInitialized() ? tier2NodeId : 0xFFFF);

  if (bestChAddr.IsInitialized() && !bestChAddr.IsBroadcast()) {
    // [E2 piggyback] 设 intent + 自代理（Tier1 才能直接代理；Tier2 等 hello 让中继代理）
    NS_LOG_INFO("Idle node " << MyNodeId() << " proactively setting intent to "
                << bestChNodeId << " (" << tierName
                << ", maxFitness=" << maxFitness << ")");
    m_intentJoinChId = IsProxyJoinEnabled() ? bestChNodeId : 0xFFFF;
    TraceJoin("join_selected", MyNodeId(), bestChNodeId, 0xFFFF, bestChAddr);
    if (!IsProxyJoinEnabled()) {
      SendClusterJoinRequest(bestChAddr);
    } else if (tier1Addr.IsInitialized()) {
      SendProxyJoinRequest(MyNodeId(), m_mainAddress, GetMyPosition(), GetVelocity(),
                           GetResidualEnergy(), clusterNode->GetFitnessValue(),
                           tier1Addr);
    }
  } else if (neighborCount >= 5 && idleNeighborCount >= 3) {
    // 没找到簇头,但有足够的邻居(>=5个)且有多个Idle(>=3个)
    // 增加随机延迟(0-1秒),避免所有节点同时自提升
    double delay = m_uniformRandomVariable->GetValue(0.0, 1.0);

    NS_LOG_INFO("Idle node " << MyNodeId() << " will try self-promotion after " << delay
                << "s (neighbors=" << neighborCount << ", idle=" << idleNeighborCount << ")");

    // 延迟后尝试自提升
    Simulator::Schedule(Seconds(delay), &RoutingProtocol::TrySelfPromotion, this);
  } else {
    // 没有簇头可加入,也没有足够邻居自建簇,创建孤立簇
    NS_LOG_INFO("Idle node " << MyNodeId() << " has no accessible cluster heads, created isolated cluster "
                << "(neighbors=" << neighborCount << ", idle=" << idleNeighborCount << ")");

    uint16_t newClusterId = m_clusterManager->ArrangeClusterId();
    m_clusterManager->CreateCluster(newClusterId, clusterNode);

    // 启动BeaconTimer
    if (m_beaconTimer.IsRunning()) {
      m_beaconTimer.Cancel();
    }
    m_beaconTimer.SetFunction(&RoutingProtocol::BeaconTimerExpire, this);
    m_beaconTimer.Schedule(m_beaconInterval);
  }
}

void RoutingProtocol::TrySelfPromotion()
{
  NS_LOG_FUNCTION(this);

  Ptr<Node> node = m_ipv4->GetObject<Node>();
  Ptr<ClusterNode> clusterNode = node->GetObject<ClusterNode>();

  if (!clusterNode || clusterNode->GetRoleType() != Idle) {
    NS_LOG_INFO("Node " << MyNodeId() << " is no longer Idle (role=" << (int)clusterNode->GetRoleType()
                << "), canceling self-promotion");
    return; // 已经不是Idle了,说明在延迟期间已经加入了其他簇
  }

  // 再次检查邻居中是否有BackupHead或Head
  // [D-γ] 两轮筛选：reachable 优先，multi-hop reachable 兜底
  double tier1MaxFit = -1.0, tier2MaxFit = -1.0;
  Ipv4Address tier1Addr, tier2Addr;
  uint16_t tier1NodeIdLocal = 0xFFFF, tier2NodeIdLocal = 0xFFFF;
  Vector myPos = GetMyPosition();

  if (m_neighbors && !m_neighbors->IsEmpty()) {
    auto neighborTable = m_neighbors->GetAll();
    for (auto it = neighborTable.begin(); it != neighborTable.end(); ++it) {
      const auto& neighborInfo = it->second;
      if (neighborInfo.Role != BackupHead && neighborInfo.Role != Head) continue;

      double dx = neighborInfo.Position.x - myPos.x;
      double dy = neighborInfo.Position.y - myPos.y;
      double dz = neighborInfo.Position.z - myPos.z;
      double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

      if (distance <= m_maxRange) {
        if (neighborInfo.FitnessValue > tier1MaxFit) {
          tier1MaxFit = neighborInfo.FitnessValue;
          tier1Addr = neighborInfo.ipAddress;
          tier1NodeIdLocal = neighborInfo.NodeId;
        }
      } else if (GreedyNextHop(neighborInfo.Position, true) != 0xFFFF) {
        if (neighborInfo.FitnessValue > tier2MaxFit) {
          tier2MaxFit = neighborInfo.FitnessValue;
          tier2Addr = neighborInfo.ipAddress;
          tier2NodeIdLocal = neighborInfo.NodeId;
        }
      }
    }
  }

  uint16_t bestNodeId = tier1Addr.IsInitialized() ? tier1NodeIdLocal
                       : (tier2Addr.IsInitialized() ? tier2NodeIdLocal : 0xFFFF);
  Ipv4Address bestChAddr;
  if (tier1Addr.IsInitialized()) bestChAddr = tier1Addr;
  else if (tier2Addr.IsInitialized()) bestChAddr = tier2Addr;

  if (bestChAddr.IsInitialized() && !bestChAddr.IsBroadcast()) {
    NS_LOG_INFO("Node " << MyNodeId() << " found CH during delay, set intent to "
                << bestNodeId << " instead of self-promoting");
    m_intentJoinChId = IsProxyJoinEnabled() ? bestNodeId : 0xFFFF;
    TraceJoin("join_selected", MyNodeId(), bestNodeId, 0xFFFF, bestChAddr);
    if (!IsProxyJoinEnabled()) {
      SendClusterJoinRequest(bestChAddr);
    } else if (tier1Addr.IsInitialized()) {
      SendProxyJoinRequest(MyNodeId(), m_mainAddress, GetMyPosition(), GetVelocity(),
                           GetResidualEnergy(), clusterNode->GetFitnessValue(),
                           tier1Addr);
    }
    return;
  }

  // 仍然没有簇头可加入,自提升为BackupHead
  NS_LOG_INFO("Node " << MyNodeId() << " promoting self to BackupHead");
  clusterNode->SetRole(BackupHead);

  // 立即广播CH_CALL
  SendClusterHeadAnnounceMent();

  // 调度等待JOIN_REQ (1.5秒后检查,如果没人加入则创建孤立簇)
  Simulator::Schedule(Seconds(1.5), &RoutingProtocol::CheckSelfPromotionResult, this);
}

void RoutingProtocol::RejoinClusterAfterTimeout()
{
  NS_LOG_FUNCTION(this);

  // 复位标志位
  m_isRejoinScheduled = false;

  // 调用ProactivelyJoinCluster尝试重新加入
  ProactivelyJoinCluster();
}

void RoutingProtocol::CheckSelfPromotionResult()
{
  NS_LOG_FUNCTION(this);

  Ptr<Node> node = m_ipv4->GetObject<Node>();
  Ptr<ClusterNode> clusterNode = node->GetObject<ClusterNode>();

  if (!clusterNode || clusterNode->GetRoleType() != BackupHead) {
    return; // 已经不是BackupHead了,说明可能已经加入了其他簇
  }

  // 检查是否有成员加入
  uint16_t myClusterId = clusterNode->GetClusterId();
  Ptr<Cluster> cluster = m_clusterManager->GetCluster(myClusterId);

  uint32_t memberCount = 0;
  if (cluster) {
    memberCount = cluster->GetAllMembers().size();
  }

  if (!cluster || memberCount <= 1) {
    // 没有成员加入(只有自己),创建孤立簇作为Head
    NS_LOG_INFO("Self-promoted node " << MyNodeId() << " has no members, becoming isolated cluster head");

    uint16_t newClusterId = m_clusterManager->ArrangeClusterId();
    m_clusterManager->CreateCluster(newClusterId, clusterNode);

    // 启动BeaconTimer
    if (m_beaconTimer.IsRunning()) {
      m_beaconTimer.Cancel();
    }
    m_beaconTimer.SetFunction(&RoutingProtocol::BeaconTimerExpire, this);
    m_beaconTimer.Schedule(m_beaconInterval);
  } else {
    // 有成员加入,保持BackupHead状态,等待RecvJoinRequest将其提升为Head
    NS_LOG_INFO("Self-promoted node " << MyNodeId() << " has " << memberCount
                << " members (including self), waiting for JOIN_REQ to become Head");
  }
}

void RoutingProtocol::CheckPacketStatus(Ptr<Socket> socket, std::string pktType, Ipv4Address dst)
{
  NS_LOG_FUNCTION(this << pktType << dst);

  Ptr<Node> node = GetObject<Node>();
  Ptr<ClusterNode> clusterNode = node->GetObject<ClusterNode>();

  if (pktType == "CH_CALL") {
    // 检查 CH_CALL 是否有申请者
    NodeRole myRole = clusterNode->GetRoleType();

    // 如果已经成为Head或Member，说明已经有人响应或已加入簇，不需要处理
    if (myRole == Head || myRole == Member) {
      return;
    }

    bool noApplicant = true;
    bool hasBackupNeighbor = false;
    Ipv4Address backupNeighborAddr;

    auto neighbors = m_neighbors->GetAll();
    for (auto it = neighbors.begin(); it != neighbors.end(); ++it) {
      if (it->second.Role == Member) {
        noApplicant = false;
        break;
      } else if (it->second.Role == BackupHead) {
        hasBackupNeighbor = true;
        backupNeighborAddr = it->second.ipAddress;
      }
    }

    if (noApplicant) {
      if (hasBackupNeighbor) {
        NS_LOG_INFO("CH_CALL timeout, joining backup neighbor at " << backupNeighborAddr);
        SendClusterJoinRequest(backupNeighborAddr);
      } else {
        // 创建孤立簇
        uint16_t newClusterId = m_clusterManager->ArrangeClusterId();
        m_clusterManager->CreateCluster(newClusterId, clusterNode);
        NS_LOG_INFO("CH_CALL timeout with no applicants, created lonely cluster " << newClusterId);

        // 启动 Beacon
        if (m_beaconTimer.IsRunning()) {
          m_beaconTimer.Cancel();
        }
        m_beaconTimer.SetFunction(&RoutingProtocol::BeaconTimerExpire, this);
        m_beaconTimer.Schedule(m_beaconInterval);  // 立即开始
      }
    }
  } else if (pktType == "JOIN_REQ") {
    // 检查 JOIN_REQ 是否收到 JOIN_RESP
    NodeRole myRole = clusterNode->GetRoleType();
    if (myRole == Member || myRole == Head) {
      return;  // 已经加入簇或成为簇头
    }
    TraceJoin("ordinary_join_timeout", MyNodeId(), 0xFFFF, 0xFFFF, dst);

    auto neighbors = m_neighbors->GetAll();
    bool noOtherBackup = true;
    uint8_t failedCounter = 0;

    // 标记失败的目标簇头
    for (auto it = neighbors.begin(); it != neighbors.end(); ++it) {
      if (it->second.ipAddress == dst) {
        NS_LOG_INFO("JOIN_REQ timeout from Node " << MyNodeId() << " to Node " << it->first);
        Neighbors::NeighborInfo info = it->second;
        info.UpdateTime = Seconds(0.0);  // 标记为失败
        UpdateNeighbor(info, "JOIN_TIMEOUT", false, false);
      }
      if (it->second.Role == BackupHead && it->second.UpdateTime.GetSeconds() == 0.0) {
        failedCounter++;
      }
    }

    // 尝试其他 BackupHead 邻居
    for (auto it = neighbors.begin(); it != neighbors.end(); ++it) {
      if (it->second.Role == BackupHead && it->second.UpdateTime.GetSeconds() > 0) {
        noOtherBackup = false;
        NS_LOG_INFO("Retrying JOIN_REQ to Node " << it->first);
        SendClusterJoinRequest(it->second.ipAddress);
        break;
      }
    }

    // 如果没有其他 BackupHead 或失败次数过多，创建孤立簇
    if (noOtherBackup || failedCounter > 3) {
      uint16_t newClusterId = m_clusterManager->ArrangeClusterId();

      if (m_clusterManager->ExistMember(MyNodeId())) {
        m_clusterManager->LeaveCluster(MyNodeId(), m_clusterManager->GetClusterId(MyNodeId()));
      }

      m_clusterManager->CreateCluster(newClusterId, clusterNode);
      NS_LOG_INFO("No other backup to join, created lonely cluster " << newClusterId);

      // 启动 Beacon
      if (m_beaconTimer.IsRunning()) {
        m_beaconTimer.Cancel();
      }
      m_beaconTimer.SetFunction(&RoutingProtocol::BeaconTimerExpire, this);
      m_beaconTimer.Schedule(m_beaconInterval);  // 立即开始
    }
  }
}

int64_t RoutingProtocol::AssignStreams(int64_t stream)
{
  NS_LOG_FUNCTION(this << stream);
  int64_t used = 0;
  m_uniformRandomVariable->SetStream(stream + used++);
  if (!m_cafbcrElection && m_ipv4) {
    m_cafbcrElection = m_ipv4->GetObject<Node>()->GetObject<cafbcrElection>();
  }
  if (m_cafbcrElection) used += m_cafbcrElection->AssignStreams(stream + used);
  if (m_qHello) used += m_qHello->AssignStreams(stream + used);
  return used;
}

// ========== 计算感知转发实现 ==========

Ipv4Address RoutingProtocol::ComputeAwareForward(const CafbcrComputeTaskTag& task)
{
  NS_LOG_FUNCTION(this << "ComputeAwareForward");

  // 获取当前节点信息
  Ptr<Node> node = m_ipv4->GetObject<Node>();
  Ptr<ClusterNode> clusterNode = node->GetObject<ClusterNode>();

  if (!clusterNode) {
    NS_LOG_WARN("ClusterNode not found, using default routing");
    return Ipv4Address("0.0.0.0");
  }

  // 获取剩余截止时间（秒）
  double remainingDeadline = task.GetDeadline();

  // 1. 检查本节点是否能处理该任务
  if (CanProcessLocally(clusterNode, task)) {
    NS_LOG_INFO("Task " << task.GetTaskId() << " can be processed locally");
    return Ipv4Address("127.0.0.1");  // 本地处理，返回环回地址表示本地
  }

  // 2. 根据剩余截止时间决策转发策略
  if (remainingDeadline < 2.0) {
    // 严格时间约束：采用贪心策略，找最近的好邻居
    NS_LOG_INFO("Strict deadline, using greedy compute forward");
    return GreedyComputeForward(task);
  } else {
    // 充足时间：优先用簇头处理
    NS_LOG_INFO("Sufficient deadline, forwarding to cluster head");
    return ForwardToClusterHead(task);
  }
}

bool RoutingProtocol::CanProcessLocally(Ptr<ClusterNode> myNode, const CafbcrComputeTaskTag& task)
{
  NS_LOG_FUNCTION(this << "CanProcessLocally task=" << task.GetTaskId());

  if (!myNode) {
    return false;
  }

  // 获取任务属性
  double taskWorkload = task.GetWorkload();      // GFLOP
  double taskDeadline = task.GetDeadline();      // 秒

  // 获取节点计算能力和当前状态
  double nodeCapacity = myNode->GetComputeCapacity();  // GFLOP/s
  double currentLoad = myNode->GetCurrentLoad();       // 0.0-1.0

  // 条件1：节点负载不能过高（保留20%的余量）
  if (currentLoad > 0.8) {
    NS_LOG_DEBUG("Node load too high, cannot process locally");
    return false;
  }

  // 条件2：节点有足够计算能力
  // 计算此任务需要的时间（不考虑当前负载的理想时间）
  double requiredTime = (nodeCapacity > 0) ? (taskWorkload / nodeCapacity) : 1e6;

  // 条件3：检查截止时间是否足够（留出20%的余量）
  if (requiredTime > taskDeadline * 0.8) {
    NS_LOG_DEBUG("Not enough time to process: required=" << requiredTime << " deadline=" << taskDeadline);
    return false;
  }

  NS_LOG_INFO("Node " << MyNodeId() << " can process task " << task.GetTaskId()
             << " (capacity=" << nodeCapacity << " load=" << currentLoad << ")");
  return true;
}

Ipv4Address RoutingProtocol::ForwardToClusterHead(const CafbcrComputeTaskTag& task)
{
  NS_LOG_FUNCTION(this << "ForwardToClusterHead task=" << task.GetTaskId());

  Ptr<Node> node = m_ipv4->GetObject<Node>();
  Ptr<ClusterNode> clusterNode = node->GetObject<ClusterNode>();

  if (!clusterNode) {
    return Ipv4Address("0.0.0.0");
  }

  // 获取簇头地址
  uint16_t chId = clusterNode->GetClusterHeadId();
  if (chId == 0xFFFF) {
    NS_LOG_DEBUG("No cluster head found");
    return GreedyComputeForward(task);  // 降级到贪心转发
  }

  // 在邻居表中查找簇头的IP地址
  if (m_neighbors) {
    const auto& neighbors = m_neighbors->GetAll();
    for (const auto& entry : neighbors) {
      const auto& neighbor = entry.second;
      if (entry.first == chId) {
        NS_LOG_INFO("Forwarding task to cluster head " << chId << " (" << neighbor.ipAddress << ")");
        return neighbor.ipAddress;
      }
    }
  }

  // 簇头不在邻居表中，降级到贪心转发
  return GreedyComputeForward(task);
}

Ipv4Address RoutingProtocol::ScheduleWithinCluster(const CafbcrComputeTaskTag& task)
{
  NS_LOG_FUNCTION(this << "ScheduleWithinCluster task=" << task.GetTaskId());

  if (!m_neighbors) {
    return Ipv4Address("0.0.0.0");
  }

  Ptr<Node> node = m_ipv4->GetObject<Node>();
  Ptr<ClusterNode> clusterNode = node->GetObject<ClusterNode>();
  if (!clusterNode) {
    return Ipv4Address("0.0.0.0");
  }

  // 在簇内邻居中找到最佳处理节点
  double bestScore = -1.0;
  Ipv4Address bestNeighbor = Ipv4Address("0.0.0.0");

  Vector myPos = GetMyPosition();
  const auto& neighbors = m_neighbors->GetAll();

  for (const auto& entry : neighbors) {
    const auto& neighbor = entry.second;

    if (!m_neighbors->HasUsableComputeState(entry.first)) continue;
    // 获取邻居的计算能力和当前负载
    double computeCapacity = neighbor.computeCapacity;
    float currentLoad = neighbor.currentLoad;

    // 计算距离（米）
    double dx = myPos.x - neighbor.Position.x;
    double dy = myPos.y - neighbor.Position.y;
    double dz = myPos.z - neighbor.Position.z;
    double distanceMeters = std::sqrt(dx*dx + dy*dy + dz*dz);
    double distanceKm = distanceMeters / 1000.0;

    // 计算服务分数：考虑计算能力、负载、链路质量
    double capacityScore = std::min(1.0, computeCapacity / 100.0);  // 归一化到100 GFLOP/s
    double loadScore = (1.0 - static_cast<double>(currentLoad));
    double linkScore = EstimateLinkRate(distanceKm);

    double score = 0.5 * capacityScore + 0.3 * loadScore + 0.2 * linkScore;

    if (score > bestScore) {
      bestScore = score;
      bestNeighbor = neighbor.ipAddress;
    }
  }

  if (bestNeighbor != Ipv4Address("0.0.0.0")) {
    NS_LOG_INFO("Best node within cluster: " << bestNeighbor << " (score=" << bestScore << ")");
  }

  return bestNeighbor;
}

Ipv4Address RoutingProtocol::GreedyComputeForward(const CafbcrComputeTaskTag& task)
{
  NS_LOG_FUNCTION(this << "GreedyComputeForward task=" << task.GetTaskId());

  if (!m_neighbors) {
    return Ipv4Address("0.0.0.0");
  }

  double bestScore = -1.0;
  Ipv4Address bestNeighbor = Ipv4Address("0.0.0.0");

  Vector myPos = GetMyPosition();
  const auto& neighbors = m_neighbors->GetAll();

  for (const auto& entry : neighbors) {
    const auto& neighbor = entry.second;

    if (!m_neighbors->HasUsableComputeState(entry.first)) continue;
    // 获取邻居的计算能力和当前负载
    double computeCapacity = neighbor.computeCapacity;
    float currentLoad = neighbor.currentLoad;

    // 计算距离（米）
    double dx = myPos.x - neighbor.Position.x;
    double dy = myPos.y - neighbor.Position.y;
    double dz = myPos.z - neighbor.Position.z;
    double distanceMeters = std::sqrt(dx*dx + dy*dy + dz*dz);

    // 计算综合评分
    // 计算能力得分（0-1）
    double capacityScore = std::min(1.0, computeCapacity / 80.0);  // 归一化到80 GFLOP/s (COMMAND节点)

    // 负载得分（0-1，负载越低越好）
    double loadScore = (1.0 - static_cast<double>(currentLoad));

    // 距离得分（距离越近越好）
    double distanceScore = 1.0 / (1.0 + distanceMeters / m_maxRange);

       
    // 链路质量得分
    double linkRate = EstimateLinkRate(distanceMeters / 1000.0);

    // 综合评分：计算能力40%，负载20%，距离20%，链路质量20%
    double score = 0.4 * capacityScore + 0.2 * loadScore + 0.2 * distanceScore + 0.2 * linkRate;

    if (score > bestScore) {
      bestScore = score;
      bestNeighbor = neighbor.ipAddress;
    }
  }

  if (bestNeighbor != Ipv4Address("0.0.0.0")) {
    NS_LOG_INFO("Greedy forward to neighbor " << bestNeighbor << " (score=" << bestScore << ")");
  }

  return bestNeighbor;
}

double RoutingProtocol::EstimateLinkRate(double distanceKm)
{
  NS_LOG_FUNCTION(this << "EstimateLinkRate distance=" << distanceKm << "km");

  double maxRangeKm = m_maxRange / 1000.0;  // m_maxRange 单位米，转 km

  if (maxRangeKm <= 0.0) {
    return 0.0;
  }

  if (distanceKm <= 0.0) {
    return 1.0;
  }

  if (distanceKm >= maxRangeKm) {
    return 0.0;  // 超出通信范围
  }

  // 二次衰减模型：近距离衰减慢，远距离衰减快
  double ratio = 1.0 - (distanceKm / maxRangeKm);
  return ratio * ratio;
}

} // namespace cafbcr
} // namespace ns3
