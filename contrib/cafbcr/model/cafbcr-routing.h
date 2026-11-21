/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef CAFBCR_ROUTING_H
#define CAFBCR_ROUTING_H

#include "ns3/object.h"
#include "ns3/ipv4-address.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/ipv4-l3-protocol.h"
#include "ns3/socket.h"
#include "ns3/vector.h"
#include "ns3/timer.h"
#include "ns3/traced-callback.h"
#include "ns3/wifi-mpdu.h"
#include "ns3/mobility-model.h"
#include "ns3/energy-module.h"
#include "ns3/fanet-clustermanager.h"
#include "ns3/fanet-neighbor.h"
#include "ns3/fanet-node.h"
#include "cafbcr-election.h"
#include "cafbcr-packet.h"
#include "cafbcr-fuzzy-hello.h"
#include "cafbcr-qhello.h"
#include "cafbcr-compute-task-tag.h"
#include "cafbcr-cluster-hop-tag.h"
#include <map>
#include <deque>
#include <tuple>

namespace ns3 {

class WifiMacQueueItem;
enum WifiMacDropReason : uint8_t;

namespace cafbcr {
class ForwardingHistory;
class RevisitRecovery;

struct NeighborComputeEvent
{
  uint32_t observerNodeId;
  uint16_t neighborNodeId;
  Ipv4Address neighborAddress;
  Time time;
  std::string message;
  Time neighborUpdatedAt;
  Time neighborLifetime;
  double residualEnergy;
  std::optional<double> computeCapacity;
  std::optional<double> currentLoad;
  Neighbors::ComputeObservation observation;
};

struct DiscoveryHelloRxEvent {
  uint32_t observer;
  uint16_t origin;
  Ipv4Address address;
  uint8_t role;
  double capacity;
  float load;
  double wireTimestampSeconds;
  Time receivedAt, expiresAt;
};
struct DiscoveryHelloTxEvent { uint32_t source; Time generatedAt; };

// Local control-plane facts. Admission at a CH is distinct from a source-side
// confirmation. SendTo's return value is not an over-the-air delivery result.
struct JoinMechanismEvent
{
  Time time;
  uint32_t observerNodeId;
  uint16_t originNodeId;
  uint16_t headNodeId;
  uint16_t clusterId;
  Ipv4Address peerAddress;
  std::string event;
  NodeRole localRole;
  uint16_t localClusterId;
  uint16_t intentHeadId;
  bool proxyEnabled;
  bool rangeEnabled;
  std::optional<int> socketResult;
  // Optional observation only; never serialized or used by protocol decisions.
  const char* cmTrigger = nullptr;
};

// Passive packet correlation and lease evidence. Packet UIDs are simulator
// observations, never serialized or consulted by forwarding decisions.
struct ProxyRenewalEvent
{
  enum TimeRejection : uint8_t {
    NonFinite = 1, Negative = 2, Future = 4, NotNewer = 8,
    ObservationExpired = 16, LeaseExpired = 32
  };
  Time time;
  uint32_t observerNodeId;
  uint16_t originNodeId, headNodeId, clusterId;
  Ipv4Address peerAddress;
  std::string event;
  uint64_t packetUid;
  std::optional<uint64_t> helloUid;
  double sourceStamp;
  std::optional<Time> sourceTime, previousRefresh;
  Time lifetime;
  uint8_t timeRejections;
  std::optional<int> socketResult;
  std::optional<int64_t> observationUs = std::nullopt;
  std::optional<uint64_t> relatedPacketUid = std::nullopt;
  std::optional<Time> holdoff = std::nullopt;
};

struct LinkProbeEvent
{
  Time time;
  uint32_t observer;
  uint16_t peer;
  Ipv4Address peerIp;
  std::string event,state;
  std::optional<uint64_t> nonce,packetUid;
  std::optional<Time> sent,deadline,confirmed;
  std::optional<bool> admitted;
};

/**
 * \brief Hello interval mode.
 */
enum HelloIntervalMode
{
  FIXED_HELLO,      ///< Fixed Hello interval.
  FUZZY_HELLO,      ///< Fuzzy-logic-optimized Hello interval.
  QLEARNING_HELLO   ///< Q-learning-optimized Hello interval.
};

/**
 * \brief CA-FBCR routing protocol.
 *
 * Cluster-based routing for UAV ad hoc networks:
 * - multi-factor fitness for cluster-head election;
 * - neighbor discovery through Hello messages;
 * - cluster-head and member maintenance;
 * - intra-cluster and inter-cluster routing.
 */
class RoutingProtocol : public Ipv4RoutingProtocol
{
public:
  static TypeId GetTypeId();
  static const uint32_t CAFBCR_PORT;

  RoutingProtocol();
  ~RoutingProtocol() override;
  void DoDispose() override;

  // Ipv4RoutingProtocol interface.
  Ptr<Ipv4Route> RouteOutput(Ptr<Packet> p,
                             const Ipv4Header& header,
                             Ptr<NetDevice> oif,
                             Socket::SocketErrno &sockerr) override;
                             
  bool RouteInput(Ptr<const Packet> p,
                  const Ipv4Header& header,
                  Ptr<const NetDevice> idev,
                  const UnicastForwardCallback& ucb,
                  const MulticastForwardCallback& mcb,
                  const LocalDeliverCallback& lcb,
                  const ErrorCallback& ecb) override;
                  
  void NotifyInterfaceUp(uint32_t interface) override;
  void NotifyInterfaceDown(uint32_t interface) override;
  void NotifyAddAddress(uint32_t interface, Ipv4InterfaceAddress address) override;
  void NotifyRemoveAddress(uint32_t interface, Ipv4InterfaceAddress address) override;
  void SetIpv4(Ptr<Ipv4> ipv4) override;
  void PrintRoutingTable(Ptr<OutputStreamWrapper> stream, Time::Unit unit = Time::S) const override;

  // Message send interface.
  void SendHello();
  void SendClusterHeadAnnounceMent();
  void SendClusterJoinRequest(Ipv4Address chAddr);
  void SendClusterJoinResponse(Ipv4Address memberAddr);
  void SendClusterHeadBeacon();
  void SendClusterMemberBeacon();
  /// Asymmetric-link join: proxy a distant SWARM node's join request to a CH.
  /// The origin* arguments describe the joining node; this node only relays
  /// the request because it can physically reach the CH.
  void SendProxyJoinRequest(uint16_t originNodeId, Ipv4Address originIp,
                            Vector originPos, Vector originVel,
                            double originEnergy, double originFitness,
                            Ipv4Address chAddr);

  // Getters
  uint16_t MyNodeId() const { return (uint16_t)m_ipv4->GetObject<Node>()->GetId(); }
  Time GetHelloInterval() const { return m_helloInterval; }
  Time GetBeaconInterval() const { return m_beaconInterval; }
  bool IsProxyJoinEnabled() const { return m_enableAsymHandling && m_enableProxyJoin; }
  bool IsRangeGatingEnabled() const { return m_enableAsymHandling && m_enableRangeGating; }
  std::string GetLinkEvidenceState(uint16_t peer) const;
  void SetTerminateRepeatedForward(bool enabled);
  bool GetTerminateRepeatedForward() const;
  Ptr<ForwardingHistory> GetForwardingHistory() const;
  void SetAlternateRepeatedForward(bool enabled);
  bool GetAlternateRepeatedForward() const;
  Ptr<RevisitRecovery> GetRevisitRecovery() const;
  
  // Setters
  void SetHelloInterval(Time interval) { m_helloInterval = interval; }
  void SetBeaconInterval(Time interval) { m_beaconInterval = interval; }
  void SetMaxRange(double range) { m_maxRange = range; }
  void SetHelloMode(HelloIntervalMode mode) { m_helloMode = mode; }
  // Order matches ComputeFitness: LHTP, Energy, Degree, Distance, Compute.
  void SetFitnessWeights(double wLHTP, double wEnergy,
                         double wDegree, double wDistance, double wCompute)
  {
    m_wLHTP = wLHTP; m_wEnergy = wEnergy; m_wDegree = wDegree;
    m_wDistance = wDistance; m_wCompute = wCompute;
  }

  // Getters
  HelloIntervalMode GetHelloMode() const { return m_helloMode; }
  Ptr<FuzzyHelloManager> GetFuzzyHelloManager() const { return m_fuzzyHello; }
  Ptr<QLearningHelloManager> GetQLearningHelloManager() const { return m_qHello; }

  // Performance counters.
  uint64_t GetControlPacketCount() const {
    return m_helloPacketCount + m_chCallPacketCount + m_joinReqPacketCount +
           m_joinRespPacketCount + m_chBeaconCount + m_cmBeaconCount + m_renewalAckCount + m_linkProbeCount;}
  double   GetControlThroughput();
  uint64_t GetHelloCount() const { return m_helloPacketCount; }
  uint64_t GetChCallCount() const { return m_chCallPacketCount; }
  uint64_t GetJoinReqCount() const { return m_joinReqPacketCount; }
  uint64_t GetJoinRespCount() const { return m_joinRespPacketCount; }
  uint64_t GetChBeaconCount() const { return m_chBeaconCount; }
  uint64_t GetCmBeaconCount() const { return m_cmBeaconCount; }
  uint64_t GetRenewalAckCount() const { return m_renewalAckCount; }

  int64_t AssignStreams(int64_t stream);

protected:
  virtual void DoInitialize() override;

private:
  friend class CafbcrLinkProbeCase;
  friend class CafbcrNeighborComputeTestCase;
  friend class CafbcrJoinMechanismTestCase;
  friend class CafbcrProxyRenewalCase;
  friend class CafbcrRenewalFeedbackCase;
  friend class CafbcrRenewalAckBatchCase;
  friend class CafbcrDirectMaintenanceCase;
  friend class CafbcrFeedbackCoveredCmCase;
  friend class CafbcrForwardingCacheTestCase;
  friend class CafbcrRevisitNetworkCase;
  friend class CafbcrAlternateNetworkCase;
  // Merge only fields carried by this message; computation ages independently.
  void UpdateNeighbor(Neighbors::NeighborInfo info, const char* message,
                      bool carriesCompute, bool carriesEnergy);
  TracedCallback<const NeighborComputeEvent&> m_neighborComputeEvent;
  TracedCallback<const DiscoveryHelloRxEvent&> m_discoveryHelloRx;
  TracedCallback<const DiscoveryHelloTxEvent&> m_discoveryHelloTx;
  // Initialization and timers.
  void Start();
  void HelloTimerExpire();
  void ClusteringTimerExpire();
  void BeaconTimerExpire();

  // Packet receive handlers.
  void Recvcafbcr(Ptr<Socket> socket);
  void RecvHello(Ptr<Packet> packet, Ipv4Address myIp, Ipv4Address srcIp);
  void RecvClusterHeadAnnouncement(Ptr<Packet> packet, Ipv4Address myIp, Ipv4Address srcIp);
  void RecvJoinRequest(Ptr<Packet> packet, Ipv4Address myIp, Ipv4Address srcIp);
  void RecvJoinResponse(Ptr<Packet> packet, Ipv4Address myIp, Ipv4Address srcIp);
  /// CH-side PROXY_JOIN_REQ handling: add the origin to the cluster and the
  /// grantedMembers queue, without a unicast response. The distant SWARM node
  /// confirms membership from CH_LIVE.grantedMembers.
  void RecvProxyJoinRequest(Ptr<Packet> packet, Ipv4Address myIp, Ipv4Address srcIp);
  void RecvClusterHeadBeacon(Ptr<Packet> packet, Ipv4Address myIp, Ipv4Address srcIp);
  void RecvClusterMemberBeacon(Ptr<Packet> packet, Ipv4Address myIp, Ipv4Address srcIp);
  void RelayProxyMemberHello(const HelloHeader& hello, Ipv4Address srcIp,
                            std::optional<uint64_t> helloUid = std::nullopt,
                            bool holdoffComplete = false);

  // Routing.
  bool Forwarding(Ptr<const Packet> p, const Ipv4Header& header,
                  const UnicastForwardCallback& ucb, const ErrorCallback& ecb);
  bool IsMyOwnAddress(Ipv4Address src);
  Ptr<Socket> FindSocketWithInterfaceAddress(Ipv4InterfaceAddress iface) const;
  Ptr<Ipv4Route> LoopbackRoute(const Ipv4Header& header, Ptr<NetDevice> oif) const;

  // Helpers.
  Vector GetMyPosition();
  /// Whether the target position is within this node's TX reach (asymmetric distance gating).
  bool WithinMyTxRange(const Vector& pos);
  // Association eligibility keeps its configured-range check even when the
  // data-plane range ablation is disabled.
  bool WithinConfiguredRange(const Vector& pos);
  bool WithinConfiguredRange(uint16_t neighborId);
  /// Neighbor version of the same check using the registered neighbor position;
  /// returns false when the neighbor is unknown.
  bool WithinMyTxRange(uint16_t neighborId);
  /// Range-limited greedy choice: pick the neighbor with maximum progress
  /// among those within this node's TX range. If all reachable neighbors have
  /// non-positive progress (local maximum), return 0xFFFF so the caller falls
  /// back to perimeter forwarding.
  uint16_t GreedyReachableNextHop(const Vector& dstPos);
  uint16_t GreedyNextHop(const Vector& dstPos, bool filterConfiguredRange, bool filterMeasured=false);
  bool DataNeighborAllowed(const Neighbors::NeighborInfo& info);
  bool MeasuredDataGateActive() const;
  uint16_t PerimeterReachableNextHop(Vector dst, Vector my, Vector& previous);
  int SendControlPacket(Ptr<Socket> socket, Ptr<Packet> packet, Ipv4Address destination);
  /// Fallback next hop: pick the neighbor closest to dst within this node's TX
  /// range even if progress is negative. Used when greedy and perimeter both
  /// fail, so the application does not silently lose the transmission.
  uint16_t NearestReachableNeighbor(const Vector& dstPos);
  Vector GetVelocity();
  double GetResidualEnergy();
  double UpdateNodeFitness();
  Vector GetPositionFromIp(Ipv4Address addr);
  uint16_t GetNodeIdFromIp(Ipv4Address addr);
  double CalculateLHTP(Vector aPos, Vector aVel, Vector bPos, Vector bVel);
  void NotifyTxError(WifiMacDropReason reason, Ptr<const WifiMpdu> mpdu);
  void SendTo(Ptr<Socket> socket, Ptr<Packet> packet, Ipv4Address dst);

  // ---- Computation-aware routing ----
  Ipv4Address ComputeAwareForward(const CafbcrComputeTaskTag& task);
  bool CanProcessLocally(Ptr<ClusterNode> myNode, const CafbcrComputeTaskTag& task);
  Ipv4Address ForwardToClusterHead(const CafbcrComputeTaskTag& task);
  Ipv4Address ScheduleWithinCluster(const CafbcrComputeTaskTag& task);
  Ipv4Address GreedyComputeForward(const CafbcrComputeTaskTag& task);
  double EstimateLinkRate(double distanceMeters);

  // Timeout retry handling.
  void CheckPacketStatus(Ptr<Socket> socket, std::string pktType, Ipv4Address dst);

  // Actively join a cluster when an Idle node has a BackupHead/Head neighbor.
  void ProactivelyJoinCluster();

  // Select the nearest cluster head and send JOIN_REQ.
  void SelectNearestClusterHeadAndJoin();

  // Attempt self-promotion to BackupHead after rechecking that the node is still Idle.
  void TrySelfPromotion();

  // Check the self-promotion result; create an isolated cluster if nobody joined.
  void CheckSelfPromotionResult();

  // Rejoin after leaving a cluster: reset the flags, then call ProactivelyJoinCluster.
  void RejoinClusterAfterTimeout();

  // Piggybacked intent: the CH node ID an Idle or joining node wants to join,
  // carried in Hello. 0xFFFF means no intent; clustered nodes keep 0xFFFF.
  uint16_t m_intentJoinChId = 0xFFFF;
  // Rolling queue of recently granted member node IDs. The next CH_LIVE
  // broadcasts it; a distant SWARM node confirms membership on hearing its ID.
  std::deque<uint16_t> m_recentGranted;
  // Duplicate-proxy guard: last PROXY_JOIN_REQ time per origin.
  std::map<uint16_t, Time> m_proxyJoinSentAt;
  // A source may request renewal only for the association it confirmed by grant.
  std::optional<std::pair<uint16_t, uint16_t>> m_proxyMembership; // head, cluster
  std::map<uint16_t, Time> m_proxyRenewalSentAt; // origin -> last forwarded HELLO time

  // Cluster-head candidates.
  struct ClusterHeadCandidate {
    uint16_t nodeId;
    Ipv4Address ipAddress;
    Vector position;
    double distance;      // Cached distance (diagnostics).
    double fitness;       // Candidate fitness.
  };
  std::vector<ClusterHeadCandidate> m_chCandidates;  ///< Received cluster-head candidates.
  bool m_hasReceivedChCall;                          ///< Whether a CH_CALL has been received.
  bool m_isRejoinScheduled;                          ///< Whether a rejoin has been scheduled.

  // Route cache for repeated destinations.
  struct RouteCache {
    Ipv4Address nextHop;
    Time lastUpdate;
    uint16_t hopCount;
  };
  std::map<Ipv4Address, RouteCache> m_routeCache;    ///< Destination IP to next-hop cache.
  /// Route-cache lifetime. One second keeps the cached next hop within about
  /// 15 m of the current position and avoids stale-destination forwarding.
  Time m_routeCacheTTL = Seconds(1.0);

  // Member variables.
  Ptr<Ipv4> m_ipv4;                                       ///< IPv4 object.
  Ipv4Address m_mainAddress;                              ///< Main IP address.
  Ptr<Socket> m_socket;                                   ///< Socket.
  Ptr<NetDevice> m_lo;                                    ///< Loopback device.
  std::map<Ptr<Socket>, Ipv4InterfaceAddress> m_socketAddrs; ///< Socket-to-address map.

  // Timers.
  Timer m_helloTimer;                                     ///< Hello timer.
  Timer m_clusteringTimer;                                ///< Clustering timer.
  Timer m_beaconTimer;                                    ///< Beacon timer.

  // Intervals.
  Time m_helloInterval;                                   ///< Hello interval (default 1.0 s).
  Time m_beaconInterval;                                  ///< Beacon interval (default 0.75 s).
  Time m_clusteringTime;                                  ///< Clustering start time (default 1.0 s).

  // Network parameters.
  double m_maxRange;                                      ///< Per-node TX range in meters.
                                                          ///< Degenerates to a global value in
                                                          ///< homogeneous scenarios; heterogeneous
                                                          ///< FANETs inject the actual reachable
                                                          ///< distance from ClusterNode.m_maxRange.
                                                          ///< LHTP, fuzzy link lifetime,
                                                          ///< EstimateLinkRate and distance gating
                                                          ///< all use this field.
  /// Compatibility master: false disables both independent mechanisms.
  bool m_enableAsymHandling;

  // Fitness weights, injected by cafbcrHelper from ScenarioConfig and used by UpdateNodeFitness.
  double m_wLHTP;                                         ///< LHTP weight.
  double m_wEnergy;                                       ///< Energy weight.
  double m_wDegree;                                       ///< Degree weight.
  double m_wDistance;                                     ///< Distance weight.
  double m_wCompute;                                      ///< Compute-capacity weight.

  // Hello interval optimization.
  HelloIntervalMode m_helloMode;                          ///< Hello interval mode (fixed/fuzzy/Q-learning).
  Ptr<FuzzyHelloManager> m_fuzzyHello;                    ///< Fuzzy Hello manager.
  Ptr<QLearningHelloManager> m_qHello;                    ///< Q-learning Hello manager.

  // CA-FBCR clustering.
  Ptr<cafbcrElection> m_cafbcrElection;                   ///< CA-FBCR election module.
  Ptr<ClusterManager> m_clusterManager;                   ///< Cluster manager.
  Ptr<Neighbors> m_neighbors;                             ///< Neighbor table.

  // Random variables.
  Ptr<UniformRandomVariable> m_uniformRandomVariable;     ///< Uniform random variable.

  // Performance counters.
  uint64_t m_helloPacketCount;                            ///< Hello packet count.
  uint64_t m_chCallPacketCount;                           ///< CH_CALL packet count.
  uint64_t m_joinReqPacketCount;                          ///< JOIN_REQ packet count.
  uint64_t m_joinRespPacketCount;                         ///< JOIN_RESP packet count.
  uint64_t m_chBeaconCount;                               ///< CH_LIVE packet count.
  uint64_t m_cmBeaconCount;                               ///< CM_LIVE packet count.

  uint64_t m_ControlPacketSize;

  // Independent CA-only ablations; defaults preserve the complete mechanism.
  bool m_enableProxyJoin = true;
  bool m_enableProxyRenewal = true;
  bool m_enableRangeGating = true;
  void TraceJoin(const char* event, uint16_t origin, uint16_t head = 0xFFFF,
                 uint16_t cluster = 0xFFFF, Ipv4Address peer = Ipv4Address::GetAny(),
                 std::optional<int> socketResult = std::nullopt,
                 const char* cmTrigger = nullptr);
  void SendClusterMemberBeaconFor(const char* trigger);
  void SendClusterMemberResponse(uint16_t head, uint16_t cluster, Time membershipSince);
  bool CanSuppressClusterMemberResponse(uint16_t head, uint16_t cluster, Time membershipSince) const;
  TracedCallback<const JoinMechanismEvent&> m_joinEvent;
  void TraceProxyRenewal(const char* event, uint16_t origin, uint16_t head,
                        uint16_t cluster, Ipv4Address peer, uint64_t packetUid,
                        double sourceStamp, std::optional<Time> previous = std::nullopt,
                        std::optional<uint64_t> helloUid = std::nullopt,
                        std::optional<int> socketResult = std::nullopt,
                        uint8_t timeRejections = 0);
  TracedCallback<const ProxyRenewalEvent&> m_proxyRenewalEvent;
  using RenewalKey = std::tuple<uint16_t, uint16_t, uint16_t, int64_t>; // origin, head, cluster, CM wire us
  using RenewalAssociation = std::tuple<uint16_t, uint16_t, uint16_t>;
  // Opt-in source-side gating. CH records originate only from received direct
  // CMs; source decisions use its own association and received control bytes.
  bool m_directMaintenanceFeedback = false;
  // Default off: only an extra CH_LIVE response may use received feedback.
  // Periodic and admission-confirmation CMs never pass through this gate.
  bool m_feedbackCoveredCmResponse = false;
  Time m_proxyMembershipSince;
  struct DirectMaintenanceRecord { int64_t sourceUs; uint64_t receivedUid; };
  std::map<RenewalAssociation, DirectMaintenanceRecord> m_directMaintenanceRecords;
  std::optional<RenewalAssociation> m_directMaintenanceCursor;
  std::optional<RenewalKey> m_directMaintenanceCoverage;
  Time m_directMaintenanceCoverageUntil;
  EventId m_directMaintenanceFallback;
  bool DirectMaintenanceEnabled() const;
  bool HasCurrentProxyMembership() const;
  bool HasDirectMaintenanceCoverage();
  void ClearDirectMaintenanceCoverage();
  void RecordDirectMaintenance(const ClusterMemberBeaconHeader& beacon, uint64_t packetUid);
  DirectMaintenanceHeader SelectDirectMaintenanceRecords();
  void ApplyDirectMaintenance(const ClusterHeadBeaconHeader& head,
                              const DirectMaintenanceHeader& feedback,
                              Ipv4Address sourceIp, uint64_t packetUid);
  void ResumeProxyMaintenance(RenewalKey key);
  struct PendingRenewal {
    HelloHeader hello;
    Ipv4Address sourceIp;
    std::optional<uint64_t> helloUid;
    EventId timer;
  };
  bool m_coordinateProxyRenewal = false; // eager remains the default/control
  Time m_proxyRenewalWindow = MilliSeconds(155);
  bool m_coalesceRenewalAck = false;
  Time m_renewalAckBatchWindow = MilliSeconds(20);
  static constexpr size_t MAX_RENEWAL_ACK_RECORDS = 8;
  struct PendingRenewalAck {
    RenewalKey key;
    uint64_t receivedUid;
  };
  std::map<RenewalAssociation, PendingRenewalAck> m_pendingRenewalAcks;
  EventId m_renewalAckBatchTimer;
  Time m_renewalAckBatchDue;
  uint64_t m_renewalAckCount = 0;
  std::map<RenewalKey, PendingRenewal> m_pendingRenewals;
  std::map<RenewalAssociation, int64_t> m_confirmedRenewals;
  Time RenewalHoldoff(const RenewalKey& key) const;
  void SendPendingRenewal(RenewalKey key);
  void ClearPendingRenewals();
  void SendRenewalAck(const ClusterMemberBeaconHeader& beacon, uint64_t receivedUid);
  void FlushRenewalAcks();
  bool IsRenewalAckCurrent(const RenewalKey& key) const;
  void SendRenewalAckRecords(const std::vector<PendingRenewalAck>& records);
  void ApplyRenewalAck(const RenewalAckHeader& ack, Ipv4Address sourceIp, uint64_t packetUid);
  void RecvRenewalAck(Ptr<Packet> packet, Ipv4Address sourceIp);
  void TraceRenewalCoordination(const char* event, const RenewalKey& key, Ipv4Address peer,
      uint64_t packetUid, std::optional<uint64_t> helloUid = std::nullopt,
      std::optional<int> socketResult = std::nullopt,
      std::optional<uint64_t> relatedUid = std::nullopt,
      std::optional<Time> holdoff = std::nullopt);

  // CA-only optional measurements. They do not read another node's state.
  bool m_linkProbes=false,m_handshakeGate=false,m_linkProbeStarted=false;
  Time m_linkProbeInterval=Seconds(2),m_linkProbeTimeout=MilliSeconds(250),m_linkEvidenceValidity=Seconds(6);
  uint32_t m_controlSendDepth=0;
  bool m_oneHopControlUnicast=false;
  uint64_t m_linkNonce=0,m_linkProbeCount=0;
  EventId m_linkProbeTimer;
  struct PendingLinkProbe { Ipv4Address ip;uint64_t nonce;Time sent;EventId timeout; };
  struct LinkConfirmation { Ipv4Address ip;uint64_t nonce;Time sent,received; };
  std::map<uint16_t,PendingLinkProbe> m_linkPending;
  std::map<uint16_t,LinkConfirmation> m_linkConfirmed;
  TracedCallback<const LinkProbeEvent&> m_linkProbeEvent;
  void LinkProbeTick();
  void LinkProbeExpired(uint16_t peer,uint64_t nonce);
  void SendLinkProbe(const LinkProbeHeader& header,Ipv4Address peer,bool reply);
  void RecvLinkProbe(Ptr<Packet> packet,Ipv4Address source,bool reply);
  void ClearLinkProbes();
  void TraceLink(const char* event,uint16_t peer,Ipv4Address ip,
      std::optional<uint64_t> nonce=std::nullopt,std::optional<uint64_t> uid=std::nullopt,
      std::optional<Time> sent=std::nullopt,std::optional<Time> deadline=std::nullopt,
      std::optional<bool> admitted=std::nullopt);
  void TraceDataLinkSelection(Ptr<const Packet> packet,Ipv4Address gateway,const char* event="selected_output");
};

} // namespace cafbcr
} // namespace ns3

#endif /* CAFBCR_ROUTING_H */
