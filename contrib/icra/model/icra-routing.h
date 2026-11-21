/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef ICRA_ROUTING_H
#define ICRA_ROUTING_H

#include "ns3/object.h"
#include "ns3/ipv4-address.h"
#include "ns3/wifi-mpdu.h"
#include "ns3/socket.h"
#include "ns3/vector.h"
#include "ns3/mobility-model.h"
#include "ns3/energy-module.h"
#include "ns3/ipv4-l3-protocol.h"
#include "icra-rtable.h"
#include "icra-rqueue.h"
// #include "performance-metrics.h"
#include "icra-Qweight.h"
#include "ns3/fanet-clustermanager.h"
#include <map>
#include <deque>
#include <set>
#include <array>

namespace ns3{

class WifiMacQueueItem;
class ArpCache;
class ArpL3Protocol;
class WifiMac;
enum WifiMacDropReason : uint8_t;

namespace icra{

class RoutingProtocol : public Ipv4RoutingProtocol
{
public:
  static TypeId GetTypeId(void);
  static const uint32_t ICRA_PORT;

  RoutingProtocol();
  ~RoutingProtocol() override;
  void DoDispose() override;

  Ptr<Ipv4Route> RouteOutput (Ptr<Packet> p,
                              const Ipv4Header& header,
                              Ptr<NetDevice> oif,
                              Socket::SocketErrno &sockerr) override;
  bool RouteInput (Ptr<const Packet> p,
                   const Ipv4Header& header,
                   Ptr<const NetDevice> idev,
                   const UnicastForwardCallback& ucb,
                   const MulticastForwardCallback& mcb,
                   const LocalDeliverCallback& lcb,
                   const ErrorCallback& ecb) override;

  void NotifyInterfaceUp (uint32_t interface) override;
  void NotifyInterfaceDown (uint32_t interface) override;
  void NotifyAddAddress (uint32_t interface, Ipv4InterfaceAddress address) override;
  void NotifyRemoveAddress (uint32_t interface, Ipv4InterfaceAddress address) override;
  void SetIpv4 (Ptr<Ipv4> ipv4) override;
  void PrintRoutingTable (Ptr<OutputStreamWrapper> stream, Time::Unit unit = Time::S) const override;

  void UpdateRouteToNeighbor(Ipv4Address sender, Ipv4Address receiver);
  void SendHello();
  void SendClusterHeadAnnounceMent();
  void SendClusterJoinRequest(Ipv4Address ip);
  void SendClusterJoinResponse( Ipv4Address ip);
  void SendClusterHeadBeacon(uint16_t ReceiverId);
  void SendClusterMemberBeacon(Ipv4Address ip);

  Time      GetMaxQueueTime() const    {return m_maxQueueTime;}
  void      SetMaxQueueTime(Time t);
  uint32_t  GetMaxQueueLen() const {return m_maxQueueLen; }
  void      SetMaxQueueLen(uint32_t len);

  // void SetNode(Ptr<Node> node);
  uint16_t MyNodeId() const {return (uint16_t)m_Ipv4->GetObject<Node>()->GetId();}
  // void ReceivePacket(Ptr<Socket> socket);
  int64_t AssignStreams(int64_t stream);

  // Paper mode is independent of the shared FANET clustering policy.
  void Configure(bool paper, double range, Time epoch, double sigma, uint32_t phi,
                 Time gsDelay, uint16_t excludedNodeId = 0xffff);
  void SetPaperMode(bool paper) { m_paperMode = paper; }
  bool GetPaperMode() const { return m_paperMode; }
  void SetMaxRange(double range) { m_maxRange = range; }
  void SetParticipatesInClustering(bool value) { m_participates = value; }
  bool IsInterClusterForwarder() const { return m_paperForwarder; }
  uint64_t GetAppliedStrategyCount() const { return m_appliedStrategyCount; }
  uint64_t GetGsReportCount() const { return m_gsReportCount; }
  uint64_t GetGsPolicyCount() const { return m_gsPolicyCount; }
  uint64_t GetPaperRouteCount() const { return m_paperRouteCount; }
  uint64_t GetAerialControlPackets() const { return m_paperControlPackets; }
  uint64_t GetAerialControlBytes() const { return m_paperControlBytes; }
  uint64_t GetGsControlBytes() const { return m_gsControlBytes; }
  uint32_t GetSemanticRoleChanges() const { return m_lastRoleChangeCount; }
  bool ParticipatesInClustering() const { return m_participates; }
  uint16_t GetPaperHeadId() const { return m_paperHeadId; }
  uint16_t GetPaperClusterId() const { return m_paperClusterId; }
  bool IsPaperClustering() const { return m_paperClustering; }
  // Observational counters; collecting them does not schedule events or draw RNG.
  struct PaperAuditCounters {
    uint64_t formationStarts = 0, rejoinStarts = 0;
    uint64_t headElection = 0, headNoCandidate = 0, headJoinFailure = 0;
    uint64_t headDeadline = 0, headAllCh = 0;
    uint64_t joinAttempts = 0, joinAccepts = 0, joinTimeouts = 0;
    uint64_t policyRebuilds = 0;
    // A committed CH that becomes a member after a later round. This is the
    // paper's Step 1--3 rearrangement, not an extra merger rule.
    uint64_t headDemoted = 0;
  };
  const PaperAuditCounters& GetPaperAuditCounters() const { return m_paperAuditCounters; }

public:
  // ICRA-Q-Weight
  void    SaveQLearningModel(const std::string& filename);
  void    LoadQLearningModel(const std::string& filename);
  void    SendUtility();                             // Broadcast utility messages.
  void    SetLHTPThreshold(double threshold) { m_lhtpThreshold = threshold; }
  double  GetLHTPThreshold() const { return m_lhtpThreshold; }
  /* Performance Benchmarks*/
  uint64_t GetControlPacketCount() const {
    if (m_paperMode) { return m_paperControlPackets + m_gsReportCount + m_gsPolicyCount; }
    return m_helloPacketCount + m_chCallPacketCount + m_joinReqPacketCount
          + m_joinRespPacketCount + m_headbeaconCount + m_memberbeaconCount + m_utilityPacketCount;}
  double   GetControlThroughput();
  uint64_t GetHelloCount()        const { return m_helloPacketCount;   }
  uint64_t GetChCallCount()       const { return m_chCallPacketCount;  }
  uint64_t GetJoinReqCount()      const { return m_joinReqPacketCount; }
  uint64_t GetJoinRespCount()     const { return m_joinRespPacketCount;}
  uint64_t GetHeadBeaconCount()   const { return m_headbeaconCount;    }
  uint64_t GetMemberBeaconCount() const { return m_memberbeaconCount;  }
  uint64_t GetUtilityCount()      const { return m_utilityPacketCount; }

protected:
  virtual void DoInitialize();

private:
  friend class IcraFidelityTestCase;
  struct PaperNeighbor {
    uint16_t nodeId = 0xffff, headId = 0xffff, clusterId = 0xffff;
    uint32_t epoch = 0, stateEpoch = 0;
    NodeRole role = Idle;
    bool forwarder = false;
    Vector position, velocity;
    double originEnergy = 1, energy = 1, utility = 0;
    Time updated;
    Ipv4Address address;
  };
  void PaperInitialize();
  void PaperSyncLocalState();
  void PaperMaintenance();
  void PaperPruneNeighbors();
  bool PaperNeighborUsable(const PaperNeighbor& neighbor) const;
  Vector PaperNeighborPosition(const PaperNeighbor& neighbor) const;
  void PaperInvalidateNeighbor(uint16_t id);
  void PaperRemoveMemberMirror(uint16_t id);
  void PaperOnArpDrop(Ptr<const Packet> packet);
  void PaperStartClustering();
  void PaperStartRound(bool recovery);
  void PaperRecoverHead();
  void PaperElection();
  void PaperPublishUtility();
  void PaperSelectHead();
  void PaperJoinTimeout();
  enum class PaperHeadReason { Election, NoCandidate, JoinFailure, Deadline, AllCh };
  void PaperBecomeHead(PaperHeadReason reason = PaperHeadReason::Election);
  void PaperDeclareHead(PaperHeadReason reason);
  void PaperCommitAsMember(uint16_t head, uint16_t cluster);
  void PaperFinishFormation();
  void PaperApplyPolicy();
  bool PaperRoundHead() const { return m_paperClustering && m_paperRoundHead == MyNodeId(); }
  bool PaperValidHeadForJoin() const
  {
    return m_paperStateEpoch == m_paperEpoch
      && (m_paperClustering ? m_paperRoundHead == MyNodeId() : m_paperHeadId == MyNodeId());
  }
  Time PaperCollectionWindow() const;
  Time PaperRoundBudget() const;
  void PaperLeaveCluster();
  void PaperCommitSemanticRole();
  void PaperUpdateBackbone();
  void PaperSendControl(PaperControlHeader::Kind kind,
                        Ipv4Address destination = Ipv4Address::GetBroadcast(),
                        uint16_t targetHead = 0xffff, bool forwarder = false,
                        uint16_t transaction = 0);
  void PaperReceiveControl(Ptr<Packet> packet, Ipv4Address sender);
  Ipv4Address PaperNextHop(Ipv4Address destination);
  Ptr<Ipv4Route> PaperMakeRoute(Ipv4Address destination, Ipv4Address nextHop,
                               Ipv4Address source = Ipv4Address());
  Ptr<Ipv4Route> PaperRouteOutput(Ptr<Packet> packet, const Ipv4Header& header,
                                 Ptr<NetDevice> oif, Socket::SocketErrno& error);
  void PaperSendReport();
  void PaperReceiveReport(Ptr<Packet> packet);
  void PaperReceivePolicy(Ptr<Packet> packet);
  void PaperGroundControlTick();
  void PaperEnqueue(Ptr<const Packet> packet, const Ipv4Header& header,
                    const UnicastForwardCallback& ucb, const ErrorCallback& ecb,
                    int32_t outputInterface = -1);
  void PaperExpireQueue();
  void PaperFlushQueue();
  struct PaperQueuedPacket {
    Ptr<const Packet> packet;
    Ipv4Header header;
    UnicastForwardCallback ucb;
    ErrorCallback ecb;
    Time expires;
    int32_t outputInterface = -1;
  };

  bool m_paperMode = true;
  bool m_paperArpAwareJoin = false;
  bool m_paperRejoinOnly = false;
  bool m_paperRebuildEveryPolicy = false;
  bool m_paperAllCh = false;
  bool m_participates = true;
  uint16_t m_excludedNodeId = 0xffff;
  bool m_paperClustering = false;
  bool m_paperUtilityFrozen = false;
  // The published W_i remains the epoch's election/declaration value after
  // this node finishes formation; other nodes may still be selecting a CH.
  bool m_paperUtilityPublished = false;
  double m_paperRoundUtility = 0;
  // A formation round decides a round-scoped head first and only then replaces
  // the committed association. The old topology keeps serving data meanwhile.
  enum class PaperRoundKind : uint8_t { None, Full, Recovery };
  PaperRoundKind m_paperRoundKind = PaperRoundKind::None;
  uint16_t m_paperRoundHead = 0xffff, m_paperRoundCluster = 0xffff;
  uint32_t m_paperStateEpoch = 0;
  uint16_t m_paperJoinTransaction = 0, m_pendingJoinTransaction = 0;
  Time m_paperRoundStart, m_paperUtilityWindowStart, m_paperRoundDeadline;
  bool m_hasPendingPolicy = false;
  uint32_t m_pendingPolicyEpoch = 0;
  std::array<double,4> m_pendingPolicyWeights{{0.25,0.25,0.25,0.25}};
  bool m_paperForwarder = false;
  bool m_ifnRequested = false;
  uint16_t m_paperHeadId = 0xffff, m_paperClusterId = 0xffff;
  uint16_t m_pendingJoinHead = 0xffff;
  uint8_t m_pendingJoinAttempts = 0;
  Ipv4Address m_pendingJoinAddress;
  uint32_t m_paperEpoch = 0, m_gsEpoch = 0;
  uint8_t m_semanticRole = 0;
  uint16_t m_semanticHead = 0xffff;
  Time m_groundControlDelay = MilliSeconds(1);
  Time m_paperHelloInterval = Seconds(0.25);
  Time m_paperNeighborTimeout = Seconds(1);
  Time m_paperFormationDeadline;
  Time m_lastIfnRequest, m_lastIfnAck;
  std::map<uint16_t, PaperNeighbor> m_paperNeighbors;
  std::map<uint16_t, Time> m_paperMembers, m_paperPendingMembers, m_paperForwarders;
  std::deque<PaperQueuedPacket> m_paperQueue;
  std::set<uint16_t> m_triedPaperHeads, m_reportedNodes;
  std::map<uint16_t, Time> m_lastGsReports;
  std::map<uint16_t, uint32_t> m_lastGsReportEpochs;
  EventId m_paperUtilityEvent, m_paperElectionEvent, m_paperSelectionEvent, m_paperJoinEvent;
  EventId m_paperFormationEvent, m_paperReportEvent, m_paperQueueEvent;
  EventId m_paperRecoveryEvent;
  Ptr<ArpCache> m_paperArpCache;
  Ptr<ArpL3Protocol> m_paperArpProtocol;
  Ptr<WifiMac> m_paperMac;
  uint64_t m_appliedStrategyCount = 0, m_gsReportCount = 0, m_gsPolicyCount = 0;
  uint64_t m_paperControlPackets = 0, m_paperControlBytes = 0, m_gsControlBytes = 0;
  uint64_t m_paperRouteCount = 0;
  PaperAuditCounters m_paperAuditCounters;
  int64_t m_paperOptimizerStream = -1;
  static std::map<uint16_t, RoutingProtocol*> m_paperProtocols;
  static RoutingProtocol* m_paperController;

  void Start();
  void NotifyTxError(WifiMacDropReason reason, Ptr<const WifiMpdu> mpdu);

  void HelloTimerExpire();
  void ClusteringTimerExpire();
  void BeaconTimerExpire();
  void ClusteringPerformanceTimerExpire();
  void UpdateRoutingTable(Ipv4Address dst, Ipv4Address nexthop, uint16_t hopCount, RouteType t);

  void DeferredRouteOutput (Ptr<const Packet> p, const Ipv4Header &header, const UnicastForwardCallback& ucb, const ErrorCallback& ecb);
  bool Forwarding (Ptr<const Packet> p, const Ipv4Header & header, const UnicastForwardCallback& ucb, const ErrorCallback& ecb);
  bool IsMyOwnAddress(Ipv4Address src);

  Ptr<Socket> FindSocketWithInterfaceAddress(Ipv4InterfaceAddress iface) const;
  // Ptr<Socket> FindSubnetBroadcastSocketWithInterfaceAddress(Ipv4InterfaceAddress iface) const;
  Ptr<Ipv4Route> LoopbackRoute(const Ipv4Header &header, Ptr<NetDevice> oif) const;

  void RecvIcra     (Ptr<Socket> socket);
  void RecvHello          (Ptr<Packet> packet, Ipv4Address myIp, Ipv4Address srcIp);
  void RecvHeadAnnounce   (Ptr<Packet> packet, Ipv4Address myIp, Ipv4Address srcIp);
  void RecvJoinRequest    (Ptr<Packet> packet, Ipv4Address myIp, Ipv4Address srcIp);
  void RecvJoinResponse   (Ptr<Packet> packet, Ipv4Address myIp, Ipv4Address srcIp);
  void RecvHeadBeacon     (Ptr<Packet> packet, Ipv4Address myIp, Ipv4Address srcIp);
  void RecvMemberBeacon   (Ptr<Packet> packet, Ipv4Address myIp, Ipv4Address srcIp);
  void RecvUtility        (Ptr<Packet> packet, Ipv4Address myIp, Ipv4Address srcIp);

  void CheckPacketStatus  (Ptr<Socket> socket, std::string pktpe, Ipv4Address dst);
  bool LookupRoute    (Ipv4Address dst, RoutingTableEntry & rt);
  void SendPacketFromQueue(Ipv4Address dst, Ptr<Ipv4Route> route);
  // void ProcessQueuedPackets(Ipv4Address dst);
  void SendTo(Ptr<Socket> socket, Ptr<Packet> packet, Ipv4Address dst);
  // Node information.
  Vector      GetMyPosition();
  Vector      GetPositionFromIp(Ipv4Address addr);
  uint16_t    GetNodeIdFromIp(Ipv4Address addr);
  double      UpdateNodeFitness();
  double      ComputeFitnessICRA();   // ICRA fitness: energy, degree, LHTP and velocity similarity.
  double      GetMaxRange() const { return m_maxRange;}
  Ipv4Address FindGatewayToCluster(uint16_t targetClusterId);
  Ipv4Address FindNextHopToCluster(uint16_t targetClusterId);

  // For Qlearning-Hello
  // void        CollectEnvironmentFeedback();
  // uint16_t    CountLostNeighbors();
  // uint16_t    CountNewNeighbors();
  // double      CalculatePDR();
  void        CleanupPacketStats();
  double      CalculateLHTP(Vector aPos,Vector aVel, Vector bPos, Vector bVel);
  // Node-state collection.
  void CollectNodeStatusForQLearning();
  void CollectFeedbackForQLearning();
  void QLearningEpisodeExpire();  // Q-learning episode timer callback.

  /*ICRA Core Functions*/
  void IcraElectionPhase();           // ICRA election phase.
  void IcraClusterFormation();        // ICRA cluster-formation phase.
  bool CheckLHTP(uint16_t neighborId); // Check the link-holding-time condition.

  /* Cluster-stability helpers. */
  void SelectBestClusterHeadAndJoin();     // Select and join the best CH.
  bool ShouldSwitchCluster(uint16_t newChId, double newChFitness); // Cluster-stickiness check.
  void ProactivelyJoinCluster();           // Proactive join for Idle nodes.
  void TryToJoinCluster();                 // Rejoin after a member loses its CH.


  // Ptr<Node>                                       m_node;
  // Ptr<ClusterNode>                                m_clusternode;
  Ipv4Address                                     m_address;
  Ptr<Socket>                                     m_socket;
  Ptr<Ipv4>                                       m_Ipv4;
  Ptr<NetDevice>                                  m_lo;
  std::map<Ptr<Socket>, Ipv4InterfaceAddress>     m_socketAddrs;

  uint32_t                                        m_seqNo;
  uint32_t                                        m_requestId;
  uint16_t                                        m_rreqCount;
  uint32_t                                        m_txErrorCount;
  Timer                                           m_helloTimer;
  Timer                                           m_clusteringTimer;
  Timer                                           m_beacon_timer;
  Timer                                           m_clusteringStatusTimer;
  // Timer                                           m_checkQueueTimer;
  uint32_t                                        m_maxQueueLen;
  Time                                            m_maxQueueTime;
  Time                                            m_lastBroadcastTime;
  Time                                            m_BeaconInterval;
  Time                                            m_ClusteringTime;
  RequestQueue                                    m_queue;
  Ptr<Neighbors>                                  m_neighbors;    // Neighbor table.
  std::map<uint16_t, Ptr<Cluster>>                m_Clusters;     // Cluster table.
  RoutingTable                                    m_routingTable; // Route table.
  Ptr<ClusterManager>                             m_clusterManager;
  Ptr<UniformRandomVariable>                      m_uniformRandomVariable; // random value tooler

  QLearningWeightOptimizer::Weights               m_weights;
  double                                          m_maxRange;
  // bool                                            m_GpsrEnabled;

  // Statistics.
  std::set<uint16_t>         m_previousNeighbors;
  std::set<uint16_t>         m_currentNeighbors;
  struct PacketStats {
    uint32_t sent;
    uint32_t received;
    uint32_t dropped;
    Time lastUpdate;
  };
  std::map<Ipv4Address, PacketStats> m_packetStats;
  uint32_t m_totalPacketsSent;
  uint32_t m_totalPacketsReceived;
  uint32_t m_totalPacketsDropped;

  /*For Q-learning weight optimize*/
  Ptr<QLearningWeightOptimizer>  m_qLearningOptimizer;
  uint32_t                       m_lastRoleChangeCount;
  Time                           m_lastRoleChangeTime;
  NodeRole                       m_previousRole;
  Timer                          m_qlearningEpisodeTimer;  // Episode timer.
  Time                           m_episodeInterval;        // Episode interval.
  bool                           m_isQLearningMaster;      // Whether this node owns Q-learning.
  /* ICRA data structures. */
  std::map<uint16_t, double>     m_utilityTable;      // Neighbor utilities (W_table_i).
  Timer                          m_utilityExchangeTimer; // Utility-exchange completion timer.
  double                         m_lhtpThreshold;     // LHTP threshold sigma.
  Time                           m_utilityWaitTime;   // Utility-collection wait time.
  std::vector<uint16_t>          m_receivedCHs;       // Received CH announcements (CH_table_i).
  uint16_t                       m_attemptedCHs;      // Number of CHs already attempted.
  bool                           m_icraElectionDone;  // Whether ICRA election completed.

  /* ICRA cluster-stability mechanisms. */
  struct ClusterHeadCandidate {
    uint16_t nodeId;
    Ipv4Address ipAddress;
    double fitness;
    double distance;
  };
  std::vector<ClusterHeadCandidate>  m_chCandidates;      // Collected CH candidates.
  bool                               m_hasReceivedChCall;  // Whether CH_CALL was received.
  Timer                              m_chSelectionTimer;   // Delayed CH-selection timer.
  std::map<uint16_t, uint8_t>        m_memberHeartbeatMissCount; // Member heartbeat-miss count.
  /*Performance Metrics*/
  uint64_t m_helloPacketCount;
  uint64_t m_chCallPacketCount;
  uint64_t m_joinReqPacketCount;
  uint64_t m_joinRespPacketCount;
  uint64_t m_headbeaconCount;
  uint64_t m_memberbeaconCount;
  uint64_t m_utilityPacketCount;


};

} // namespace icra
} // namespace ns3

#endif /* ICRA_ROUTING_H */
