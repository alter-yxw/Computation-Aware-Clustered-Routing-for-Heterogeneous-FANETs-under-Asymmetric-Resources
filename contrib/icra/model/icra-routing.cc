/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
/* icra routing protocol*/
#define NS_LOG_APPEND_CONTEXT                                                  \
  if(m_Ipv4)                                                                   \
  {                                                                            \
    std::clog<<"[node " << (uint16_t)m_Ipv4->GetObject<Node>()->GetId()<<"]";  \
  }

#include "icra-routing.h"
#include "icra-packet.h"
#include "ns3/fanet-node.h"
#include "ns3/log.h"
#include "ns3/ipv4.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/adhoc-wifi-mac.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/wifi-net-device.h"
#include "ns3/simulator.h"
#include "ns3/inet-socket-address.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/arp-cache.h"
#include "ns3/arp-l3-protocol.h"
#include "ns3/ipv4-interface.h"
#include "ns3/boolean.h"
#include "ns3/double.h"
#include <cmath>
#include <algorithm>
#include <vector>

namespace ns3 {
namespace icra {
    
NS_LOG_COMPONENT_DEFINE("IcraRoutingProtocol");
NS_OBJECT_ENSURE_REGISTERED(RoutingProtocol);

const uint32_t RoutingProtocol::ICRA_PORT = 888;
std::map<uint16_t, RoutingProtocol*> RoutingProtocol::m_paperProtocols;
RoutingProtocol* RoutingProtocol::m_paperController = nullptr;

// Trace bookkeeping only: this tag never supplies a route or a remote measurement.
class PaperLinkTag : public Tag
{
public:
  uint16_t owner = 0xffff, next = 0xffff;
  static TypeId GetTypeId()
  {
    static TypeId tid = TypeId("ns3::icra::PaperLinkTag").SetParent<Tag>()
      .AddConstructor<PaperLinkTag>();
    return tid;
  }
  TypeId GetInstanceTypeId() const override { return GetTypeId(); }
  uint32_t GetSerializedSize() const override { return 4; }
  void Serialize(TagBuffer b) const override { b.WriteU16(owner); b.WriteU16(next); }
  void Deserialize(TagBuffer b) override { owner = b.ReadU16(); next = b.ReadU16(); }
  void Print(std::ostream& os) const override { os << owner << "->" << next; }
};
class PaperControlTag : public Tag
{
public:
  static TypeId GetTypeId()
  {
    static TypeId tid = TypeId("ns3::icra::PaperControlTag").SetParent<Tag>()
      .AddConstructor<PaperControlTag>();
    return tid;
  }
  TypeId GetInstanceTypeId() const override { return GetTypeId(); }
  uint32_t GetSerializedSize() const override { return 1; }
  void Serialize(TagBuffer b) const override { b.WriteU8(1); }
  void Deserialize(TagBuffer b) override { b.ReadU8(); }
  void Print(std::ostream& os) const override { os << "ICRA control"; }
};


class DeferredRouteOutputTag : public Tag
{
public:    
  DeferredRouteOutputTag(int32_t o = -1)
  : Tag(),
    m_oif(o){}

  static TypeId GetTypeId()
  {
    static TypeId tid = TypeId("ns3::icra::DeferredRouteOutputTag")
    .SetParent<Tag>()
    .SetGroupName("icra")
    .AddConstructor<DeferredRouteOutputTag>();
    return tid;
  }

  uint32_t GetInterface()          const { return m_oif;           }
  void     SetInterface(uint32_t oif)    { m_oif = oif;            }
  TypeId   GetInstanceTypeId ()    const { return GetTypeId ();    }
  uint32_t GetSerializedSize ()    const { return sizeof(int32_t); }
  void     Serialize (TagBuffer i) const { i.WriteU32 (m_oif);     }
  void     Deserialize (TagBuffer i)     { m_oif = i.ReadU32 ();   }

  void     Print (std::ostream &os) const{ os << "DeferredRouteOutputTag: output interface = " << m_oif;}

private:
  int32_t m_oif;
};

NS_OBJECT_ENSURE_REGISTERED(DeferredRouteOutputTag);



/* RoutingProtocol Main Member Functions */
RoutingProtocol::RoutingProtocol():
  m_requestId(0),
  m_rreqCount(0),
  m_txErrorCount(0),
  m_helloTimer(Timer::CANCEL_ON_DESTROY),
  m_clusteringTimer(Timer::CANCEL_ON_DESTROY),
  m_beacon_timer(Timer::CANCEL_ON_DESTROY),
  m_clusteringStatusTimer(Timer::CANCEL_ON_DESTROY),
  m_maxQueueLen (64),
  m_maxQueueTime (Seconds(30)),
  m_BeaconInterval(Seconds(0.75)), /* Check the cluster member info each *** s */
  m_ClusteringTime(Seconds(1.0)), /* Start to elect clusterHead at time 1.0s */
  m_queue (m_maxQueueLen, m_maxQueueTime),
  m_maxRange(500),
  m_lastRoleChangeCount(0),
  m_previousRole(Idle),
  m_qlearningEpisodeTimer(Timer::CANCEL_ON_DESTROY),
  m_episodeInterval(Seconds(10.0)),
  m_isQLearningMaster(false),
  m_utilityExchangeTimer(Timer::CANCEL_ON_DESTROY),
  m_lhtpThreshold(0.1),                    // Paper Table II; legacy preset supplies 2 s.
  m_utilityWaitTime(Seconds(0.5)),         // 等待收集效用值的时间
  m_attemptedCHs(0),
  m_icraElectionDone(false),
  m_hasReceivedChCall(false),              // 优化: CH候选收集标志
  m_chSelectionTimer(Timer::CANCEL_ON_DESTROY), // 优化: CH选择延迟定时器
  m_helloPacketCount(0),
  m_chCallPacketCount(0),
  m_joinReqPacketCount(0),
  m_joinRespPacketCount(0),
  m_headbeaconCount(0),
  m_memberbeaconCount(0),
  m_utilityPacketCount(0)
  
  // m_neighbors(m_HelloInterval * 2)
{
  m_neighbors = nullptr;
  m_clusterManager = ClusterManager::GetInstance();
  m_qLearningOptimizer = QLearningWeightOptimizer::GetInstance();
}


RoutingProtocol::~RoutingProtocol()
{
}


TypeId RoutingProtocol::GetTypeId(){
  static TypeId tid = TypeId("ns3::icra::RoutingProtocol")
    .SetParent<Ipv4RoutingProtocol>()
    .SetGroupName("icra")
    .AddConstructor<RoutingProtocol>()
    .AddAttribute("PaperMode", "Use the paper clustering, IFN and online strategy path",
                  BooleanValue(true), MakeBooleanAccessor(&RoutingProtocol::m_paperMode),
                  MakeBooleanChecker())
    .AddAttribute("PaperArpAwareJoin", "Diagnostic: always wait one ARP retry before the first JOIN timeout",
                  BooleanValue(false), MakeBooleanAccessor(&RoutingProtocol::m_paperArpAwareJoin),
                  MakeBooleanChecker())
    .AddAttribute("PaperRejoinOnly", "Paper III-C.5.2: a CM losing its CH resumes at Step 3 without a new election",
                  BooleanValue(true), MakeBooleanAccessor(&RoutingProtocol::m_paperRejoinOnly),
                  MakeBooleanChecker())
    .AddAttribute("PaperRebuildEveryPolicy", "Paper III-C.5.5: a strategy update restarts Step 1 even when weights are unchanged (false selects the legacy changed-only rule)",
                  BooleanValue(true), MakeBooleanAccessor(&RoutingProtocol::m_paperRebuildEveryPolicy),
                  MakeBooleanChecker())
    .AddAttribute("PaperAllCh", "Diagnostic control: elect every participating node as CH",
                  BooleanValue(false), MakeBooleanAccessor(&RoutingProtocol::m_paperAllCh),
                  MakeBooleanChecker())
    .AddAttribute("MaxRange", "Local radio range used for link prediction (m)",
                  DoubleValue(500), MakeDoubleAccessor(&RoutingProtocol::m_maxRange),
                  MakeDoubleChecker<double>(0.001))
    .AddAttribute("EpisodeInterval", "Ground controller decision window",
                  TimeValue(Seconds(2)), MakeTimeAccessor(&RoutingProtocol::m_episodeInterval),
                  MakeTimeChecker())
    .AddAttribute("GroundControlDelay", "Ideal out-of-band ground control delivery delay",
                  TimeValue(MilliSeconds(1)), MakeTimeAccessor(&RoutingProtocol::m_groundControlDelay),
                  MakeTimeChecker())
    .AddAttribute("NeighborTimeout", "Maximum age of a paper-mode neighbor observation",
                  TimeValue(Seconds(1)), MakeTimeAccessor(&RoutingProtocol::m_paperNeighborTimeout),
                  MakeTimeChecker())
    .AddAttribute ("BeaconInterval", "HELLO messages emission interval",
                    TimeValue (Seconds (5.0)),
                    MakeTimeAccessor(&RoutingProtocol::m_BeaconInterval),
                    MakeTimeChecker())
    .AddAttribute ("UniformRv",
                    "Access to the underlying UniformRandomVariable",
                    StringValue("ns3::UniformRandomVariable"),
                    MakePointerAccessor(&RoutingProtocol::m_uniformRandomVariable),
                    MakePointerChecker<UniformRandomVariable>())
    /*.AddAttribute ("WeightManager",
                    "weight manager type",
                    EnumValue(WEIGHT_FUZZY),
                    MakeEnumAccessor(&RoutingProtocol::m_weightMethod),
                    MakeEnumChecker<WeightControlMethod>(
                      WEIGHT_FUZZY,    "Fuzzy", WEIGHT_QLEARNING,"QLearning"))*/;
  return tid;
}


void RoutingProtocol::DoDispose()
{
  if (m_paperMode && m_Ipv4 && GetObject<Node>()) { PaperLeaveCluster(); }
  m_paperUtilityEvent.Cancel();
  m_paperElectionEvent.Cancel(); m_paperSelectionEvent.Cancel();
  m_paperJoinEvent.Cancel(); m_paperFormationEvent.Cancel(); m_paperReportEvent.Cancel();
  m_paperRecoveryEvent.Cancel();
  m_paperQueueEvent.Cancel(); m_paperQueue.clear();
  m_helloTimer.Cancel(); m_clusteringTimer.Cancel(); m_beacon_timer.Cancel();
  m_qlearningEpisodeTimer.Cancel(); m_clusteringStatusTimer.Cancel();
  if (m_paperMac) {
    m_paperMac->TraceDisconnectWithoutContext("DroppedMpdu", MakeCallback(&RoutingProtocol::NotifyTxError,this));
  }
  if (m_paperArpCache) {
    m_paperArpCache->TraceDisconnectWithoutContext("Drop",MakeCallback(&RoutingProtocol::PaperOnArpDrop,this));
  }
  if (m_paperArpProtocol) {
    m_paperArpProtocol->TraceDisconnectWithoutContext("Drop",MakeCallback(&RoutingProtocol::PaperOnArpDrop,this));
  }
  for (auto it = m_paperProtocols.begin(); it != m_paperProtocols.end();) {
    if (it->second == this) { it = m_paperProtocols.erase(it); } else { ++it; }
  }
  if (m_paperController == this) { m_paperController = nullptr; }
  m_paperMac = nullptr; m_paperArpCache = nullptr; m_paperArpProtocol = nullptr;
  m_Ipv4 = nullptr;
  for(auto iter = m_socketAddrs.begin(); iter != m_socketAddrs.end(); ++iter)
  {
    iter->first->Close();
  }
  m_socketAddrs.clear();
  // for(auto iter = m_socketSubnetBroadAddr.begin(); iter != m_socketSubnetBroadAddr.end(); ++iter)
  // {
  //   iter->first->Close();
  // }
  // m_socketSubnetBroadAddr.clear();
  Ipv4RoutingProtocol::DoDispose();
}


bool RoutingProtocol::RouteInput(Ptr<const Packet> p, const Ipv4Header &header, Ptr<const NetDevice> idev, const UnicastForwardCallback& ucb,
          /*RouteInput*/         const MulticastForwardCallback& mcb, const LocalDeliverCallback& lcb, const ErrorCallback& ecb)
{
  NS_LOG_FUNCTION(this <<"pktId: "<< p->GetUid()<<",dst: " << header.GetDestination());
  
  Ipv4Address dst = header.GetDestination();
  Ipv4Address src = header.GetSource();
  if (m_paperMode && idev == m_lo) {
    DeferredRouteOutputTag deferred;
    if (p->PeekPacketTag(deferred)) {
      auto copy = p->Copy(); copy->RemovePacketTag(deferred);
      Ipv4Header queuedHeader = header;
      // IpForward will consume one TTL for the artificial source loopback hop.
      if (queuedHeader.GetTtl() < 255) { queuedHeader.SetTtl(queuedHeader.GetTtl()+1); }
      int32_t interface = static_cast<int32_t>(deferred.GetInterface());
      if (m_paperClustering) { PaperEnqueue(copy,queuedHeader,ucb,ecb,interface); }
      else {
        auto route = PaperMakeRoute(dst,PaperNextHop(dst));
        if (interface >= 0 && route && m_Ipv4->GetInterfaceForDevice(route->GetOutputDevice()) != interface) {
          if (!ecb.IsNull()) { ecb(copy,queuedHeader,Socket::ERROR_NOROUTETOHOST); }
        } else { Forwarding(copy,queuedHeader,ucb,ecb); }
      }
      return true;
    }
  }
  
  // 1. check if arrived the dst node
  if (m_Ipv4->IsDestinationAddress(dst, m_Ipv4->GetInterfaceForDevice(idev))) {
    if (lcb.IsNull() == false) {
      NS_LOG_DEBUG("Local delivery to " << dst);
      lcb(p, header, m_Ipv4->GetInterfaceForDevice(idev));

      m_totalPacketsReceived++;
      auto &stat = m_packetStats[dst];
      stat.received++;
      stat.lastUpdate = Simulator::Now();      
      return true;
    }
    else{
      m_totalPacketsDropped++;
      auto & stat = m_packetStats[dst];
      stat.dropped++;
      stat.lastUpdate = Simulator::Now();
      ecb(p, header, Socket::ERROR_NOROUTETOHOST);
    }
    return false;
  }

  /* multicast */
  if (dst.IsMulticast()) {
    NS_LOG_DEBUG("Multicast destination does not enabled");
    return false;
  }
  /* Broadcast*/
  for(auto k = m_socketAddrs.begin(); k != m_socketAddrs.end(); ++k)
  {
    auto iface = k->second;
    if(dst.IsBroadcast() || dst == iface.GetBroadcast())
    {
      if(lcb.IsNull() == false){
        NS_LOG_DEBUG("Broadcast local delivery to "<< iface.GetLocal());
        lcb(p,header, m_Ipv4->GetInterfaceForDevice(idev));
      }
      else{
        NS_LOG_DEBUG("Unable to deliver packet locally casue null callback "<<p->GetUid()<<" from "<< src);
        ecb(p, header, Socket::ERROR_NOROUTETOHOST);
      }
      return true;
    }
    if (header.GetTtl() == 0)
    {
      NS_LOG_DEBUG("TTL expired");
      return false;
    }
  }
  
  return Forwarding(p, header, ucb, ecb);  
}


bool RoutingProtocol::Forwarding(Ptr<const Packet> p, const Ipv4Header & header, const UnicastForwardCallback& ucb, const ErrorCallback& ecb)
{
  if (m_paperMode) {
    Ipv4Address next = PaperNextHop(header.GetDestination());
    if (next != Ipv4Address()) {
      auto route = PaperMakeRoute(header.GetDestination(), next);
      if (route) {
        Ptr<Packet> copy = p->Copy();
        PaperLinkTag tag; tag.owner = MyNodeId();
        for (const auto& n : m_paperNeighbors) { if (n.second.address == next) { tag.next = n.first; break; } }
        copy->ReplacePacketTag(tag);
        ++m_paperRouteCount;
        ucb(route, copy, header);
        return true;
      }
    }
    if (m_paperClustering) {
      PaperEnqueue(p,header,ucb,ecb);
      return true;
    }
    if (!ecb.IsNull()) {
      ecb(p,header,Socket::ERROR_NOROUTETOHOST);
      return true; // handled drop: do not emit a second Ipv4 DROP_NO_ROUTE event
    }
    return false;
  }
  NS_LOG_FUNCTION(this);
  Ptr<ClusterNode> myNode = m_Ipv4->GetObject<Node>()->GetObject<ClusterNode>();
  uint16_t myClusterId = myNode->GetClusterId();
  NodeRole myRole = myNode->GetRoleType();
  Ipv4Address dst = header.GetDestination();
  
  m_totalPacketsSent++;
  auto &status = m_packetStats[dst];
  status.sent++;
  status.lastUpdate = Simulator::Now();

  RoutingTableEntry rt;  // 查找路由
  bool hasRoute = LookupRoute(dst, rt);

  if (myRole == Head) {  // 簇头，负责簇间路由决策
    uint16_t dstNodeId = GetNodeIdFromIp(dst);
    uint16_t dstClusterId = (dstNodeId != 0xFFFF) ? m_clusterManager->GetClusterId(dstNodeId) : 0xFFFF;
    
    if (dstClusterId == myClusterId) {
      // 目的节点在本簇内
      if (m_neighbors->IsNeighbor(dstNodeId)) {
        // 直接转发
        Ptr<Ipv4Route> route = Create<Ipv4Route>();
        route->SetDestination(dst);
        route->SetSource(m_Ipv4->GetAddress(1, 0).GetLocal());
        route->SetGateway(dst);
        route->SetOutputDevice(m_Ipv4->GetNetDevice(m_Ipv4->GetInterfaceForAddress(route->GetSource())));
        ucb(route, p, header);
        return true;
      }
    } else if (dstClusterId != 0xFFFF) {
      // 簇间转发，查找到目的簇的路径
      Ipv4Address nextHop = FindNextHopToCluster(dstClusterId);
      if (nextHop != Ipv4Address()) {
        Ptr<Ipv4Route> route = Create<Ipv4Route>();
        route->SetDestination(dst);
        route->SetSource(m_Ipv4->GetAddress(1, 0).GetLocal());
        route->SetGateway(nextHop);
        route->SetOutputDevice(m_Ipv4->GetNetDevice(m_Ipv4->GetInterfaceForAddress(route->GetSource())));
        ucb(route, p, header);
        return true;
      }
    }
  }
  
  // 使用现有路由表
  if (hasRoute) {
    Ptr<Ipv4Route> route = Create<Ipv4Route>();
    route->SetDestination(dst);
    route->SetSource(m_Ipv4->GetAddress(1, 0).GetLocal());
    route->SetGateway(rt.GetNextHopAddress());
    route->SetOutputDevice(m_Ipv4->GetNetDevice(m_Ipv4->GetInterfaceForAddress(route->GetSource())));
    
    NS_LOG_DEBUG(" Forward packet to " << dst << " via " << rt.GetNextHopAddress() <<", now "<<Simulator::Now().GetSeconds()<<" s.");
    ucb(route, p, header);
    return true;
  }
  
  // 使用贪婪转发
  Vector dstPos = GetPositionFromIp(dst);
  Vector myPos = GetMyPosition();
  uint16_t nextHopId = m_neighbors->GreedyNeighbor(dstPos, myPos);
  
  if (nextHopId != 0xFFFF) {
    Ipv4Address nextHopAddr = NodeList::GetNode(nextHopId)->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();
    Ptr<Ipv4Route> route = Create<Ipv4Route>();
    route->SetDestination(dst);
    route->SetSource(m_Ipv4->GetAddress(1, 0).GetLocal());
    route->SetGateway(nextHopAddr);
    route->SetOutputDevice(m_Ipv4->GetNetDevice(m_Ipv4->GetInterfaceForAddress(route->GetSource())));
    
    NS_LOG_DEBUG(" Greedy forward to " << dst << ", NextHopIp: " << nextHopAddr <<", now "<<Simulator::Now().GetSeconds()<<" s.");
    ucb(route, p, header);
    return true;
  }  

  NS_LOG_DEBUG(" No route found for " << dst);
  m_totalPacketsDropped++;
  auto& stat = m_packetStats[dst];
  stat.dropped++;
  stat.lastUpdate = Simulator::Now();
  ecb(p, header, Socket::ERROR_NOROUTETOHOST);
  return false;
}


Ptr<Ipv4Route> RoutingProtocol::RouteOutput(Ptr<Packet> p, const Ipv4Header &header,
                    /*RouteOutput*/         Ptr<NetDevice> oif, Socket::SocketErrno & sockerr)
{ NS_LOG_FUNCTION(this << header << "oif? " << (oif ? oif->GetIfIndex() : 0));
  if (m_paperMode) { return PaperRouteOutput(p,header,oif,sockerr); }
  
  Ipv4Address dst = header.GetDestination();
  Ipv4Address src = header.GetSource();
  if(m_Ipv4->GetAddress(1, 0).GetBroadcast() == dst || dst.IsBroadcast())  // arrange broadcast packet
  {
    // NS_LOG_DEBUG("BroadCast Destination");
    Ptr<Ipv4Route> route = Create<Ipv4Route>();
    route->SetDestination(dst);
    if (src == Ipv4Address())
    {
      if (m_socketAddrs.size() > 0) 
      {
        route->SetSource(m_socketAddrs.begin()->second.GetLocal());
      }
      else { route->SetSource(m_Ipv4->GetAddress(1, 0).GetLocal()); }
    }
    else {  route->SetSource(src); }    
    route->SetGateway(Ipv4Address::GetBroadcast());
    route->SetOutputDevice(m_Ipv4->GetNetDevice(1));
    sockerr = Socket::ERROR_NOTERROR;
    return route;

  }  

  Ipv4Address sourceAddress;  // 获取合适的源地址
  if (src == Ipv4Address()) {  // 如果没有指定源地址，使用第一个有效接口的地址
    sourceAddress = m_Ipv4->GetAddress(1, 0).GetLocal();
  } else {
    sourceAddress = src;
  }
  RoutingTableEntry rt; // 查找路由表中是否有现成路由
  if (LookupRoute(dst, rt))
  {
    Ptr<Ipv4Route> route = Create<Ipv4Route>();
    route->SetDestination(dst);
    route->SetSource(sourceAddress);

    Ipv4Address gateway = rt.GetNextHopAddress();
    if(gateway == Ipv4Address()) { gateway = dst; }
    route->SetGateway(gateway);

    uint32_t interfaceIdx = m_Ipv4->GetInterfaceForAddress(route->GetSource());
    route->SetOutputDevice(m_Ipv4->GetNetDevice(interfaceIdx));
    if(dst != rt.GetNextHopAddress())
    {
      NS_LOG_DEBUG("------------------------------------------------------------------------------------------");;
      NS_LOG_DEBUG(" Found route to " << dst << " via " << rt.GetNextHopAddress()
                      <<", now "<<Simulator::Now().GetSeconds()<<" s.");
    }
    sockerr = Socket::ERROR_NOTERROR;
    return route;
  }
  // 基于簇结构创建路由
  Ptr<ClusterNode> myNode = m_Ipv4->GetObject<Node>()->GetObject<ClusterNode>();
  uint16_t myNodeId = myNode->GetNodeId();
  uint16_t myClusterId = myNode->GetClusterId();
  uint16_t dstNodeId = GetNodeIdFromIp(dst);  // 获取目的节点信息
  uint16_t dstClusterId = (dstNodeId != 0xFFFF) ? m_clusterManager->GetClusterId(dstNodeId) : 0xFFFF;  
  Ptr<Ipv4Route> route = Create<Ipv4Route>();
  route->SetDestination(dst);
  route->SetSource(m_Ipv4->GetAddress(1, 0).GetLocal());
  Ipv4Address nextHopAddr;
  bool routeFound = false;
  // 簇内
  if (myClusterId != 0xFFFF && myClusterId == dstClusterId) {  
    NS_LOG_DEBUG("Intra-cluster routing from " << myNodeId << " to " << dstNodeId);
    if (m_neighbors->IsNeighbor(dstNodeId)) {  // 检查是否是直接邻居
      nextHopAddr = dst;  // 直接发送到目的地
      routeFound = true;
      NS_LOG_DEBUG("Direct neighbor, send directly to "<< dst);
    }
    else if (myNode->GetRoleType() != Head)
    {   // 普通成员通过簇头转发
      uint16_t chNodeId = m_clusterManager->GetClusterHeadId(myClusterId);
      if (chNodeId != 0xFFFF && chNodeId != myNodeId && m_neighbors->IsNeighbor(chNodeId)) {
        nextHopAddr = NodeList::GetNode(chNodeId)->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();
        routeFound = true;
        NS_LOG_DEBUG("Send to cluster head " << chNodeId);
      }
    }
    else // means the dst is not my neighbor, but i am a clusterHead
    {
      auto neighbors = m_neighbors->GetAll();
      for (auto &n : neighbors) {
        if (n.second.clusterId == myClusterId && n.second.NodeId == dstNodeId) {
          nextHopAddr = n.second.ipAddress;
          routeFound = true;
          NS_LOG_DEBUG("CH: forward to cluster member " << n.second.ipAddress);
          break;
        }
      }
      nextHopAddr = dst;
      NS_LOG_DEBUG("The destNode is not in the neighborTable of ClusterHead");
    }
  }
  //  簇间路由
  else if (myClusterId != 0xFFFF && dstClusterId != 0xFFFF && myClusterId != dstClusterId) {
    NS_LOG_DEBUG("Inter-cluster routing from cluster " << myClusterId << " to cluster " << dstClusterId);
    
    if (myNode->GetRoleType() == Head) {   // 簇头，尝试找到达目的簇的路径
      uint16_t dstChId = m_clusterManager->GetClusterHeadId(dstClusterId);
      if (dstChId != 0xFFFF) {     // 目的簇头是否是邻居
        if (m_neighbors->IsNeighbor(dstChId)) {
          nextHopAddr = NodeList::GetNode(dstChId)->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();
          routeFound = true;
          NS_LOG_DEBUG("Destination CH is neighbor, send directly");
        } else {  // 需要通过中继节点
          nextHopAddr = FindGatewayToCluster(dstClusterId);
          if (nextHopAddr != Ipv4Address()) {
            routeFound = true;
            NS_LOG_DEBUG("Use gateway " << nextHopAddr << " to reach cluster " << dstClusterId);
          }
        }
      }
    } else { // 普通成员节点，发送给自己的簇头
      uint16_t myChId = m_clusterManager->GetClusterHeadId(myClusterId);
      if (myChId != 0xFFFF && m_neighbors->IsNeighbor(myChId)) {
        nextHopAddr = NodeList::GetNode(myChId)->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();
        routeFound = true;
        NS_LOG_DEBUG("Forward to my cluster head " << myChId);
      }
    }
  }
  
  if (!routeFound)
  { // 如果簇路由失败，使用贪婪转发
    NS_LOG_DEBUG("------------------------------------------------------------------------------------------");
    NS_LOG_DEBUG(" \t\t Unable to find route to "<<dst
            << " by cluster, try greedy mode, now "<<Simulator::Now().GetSeconds()<<" s.");
    Vector dstPos = GetPositionFromIp(dst);
    Vector myPos = GetMyPosition();
    uint16_t nextHopId = m_neighbors->GreedyNeighbor(dstPos, myPos);
    
    if (nextHopId != 0xFFFF)
    {
      nextHopAddr = NodeList::GetNode(nextHopId)->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();
      routeFound = true;
      NS_LOG_DEBUG("------------------------------------------------------------------------------------------");;
      NS_LOG_DEBUG(" Greedy forward to "<< dst<< " via node " << nextHopId
                        <<", now "<<Simulator::Now().GetSeconds()<<" s.");
    }
  }

  if (routeFound && nextHopAddr != Ipv4Address()) {  // 如果找到路由，创建并返回
    Ptr<Ipv4Route> route = Create<Ipv4Route>();
    route->SetDestination(dst);
    route->SetSource(sourceAddress);
    route->SetGateway(nextHopAddr);
    
    uint32_t interfaceIdx = m_Ipv4->GetInterfaceForAddress(sourceAddress);
    route->SetOutputDevice(m_Ipv4->GetNetDevice(interfaceIdx));    

    if (dstNodeId != 0xFFFF) {    // 更新路由表
      RouteType rType = (myClusterId == dstClusterId) ? ROUTE_INTRA_CLUSTER : ROUTE_INTER_CLUSTER;
      RoutingTableEntry newEntry(dst, dstNodeId, dstClusterId, GetPositionFromIp(dst),
                                 nextHopAddr, GetNodeIdFromIp(nextHopAddr),
                                 Idle, GetPositionFromIp(nextHopAddr), rType, 1);
      m_routingTable.AddRoute(newEntry);
    }
    
    sockerr = Socket::ERROR_NOTERROR;
    return route;
  }

  NS_LOG_DEBUG("No route found to " << dst << ", returning ERROR_NOROUTETOHOST. " << "Upper layer (TCP/UDP) will handle retry or drop.");
  sockerr = Socket::ERROR_NOROUTETOHOST;
  return nullptr;
}


Ipv4Address RoutingProtocol::FindGatewayToCluster(uint16_t targetClusterId)
{
  NS_LOG_FUNCTION(this << targetClusterId);
  
  auto neighbors = m_neighbors->GetAll();  // 查找邻居中是否有目标簇的成员
  for (auto &n : neighbors) {
    if (n.second.clusterId == targetClusterId) {
      NS_LOG_DEBUG("Found gateway " << n.second.ipAddress << " to cluster " << targetClusterId);
      return n.second.ipAddress;
    }
  }  

  uint16_t targetChId = m_clusterManager->GetClusterHeadId(targetClusterId);  // 查找可以到达目标簇头的中继节点
  if (targetChId != 0xFFFF) {
    Vector targetChPos = NodeList::GetNode(targetChId)->GetObject<MobilityModel>()->GetPosition();
    Vector myPos = GetMyPosition();
    
    // 使用贪婪策略找最接近目标簇头的邻居
    double minDistance = CalculateDistance(myPos, targetChPos);
    Ipv4Address bestGateway;
    
    for (auto &n : neighbors) {
      Vector nPos = n.second.Position;
      double distance = CalculateDistance(nPos, targetChPos);
      if (distance < minDistance) {
        minDistance = distance;
        bestGateway = n.second.ipAddress;
      }
    }
    
    if (bestGateway != Ipv4Address()) {
      NS_LOG_DEBUG("Found relay node " << bestGateway << " toward cluster " << targetClusterId);
      return bestGateway;
    }
  }
  
  return Ipv4Address();
}


// 查找到达指定簇的下一跳
Ipv4Address RoutingProtocol::FindNextHopToCluster(uint16_t targetClusterId)
{
  NS_LOG_FUNCTION(this << targetClusterId);
  
  // 首先尝试直接到达
  uint16_t targetChId = m_clusterManager->GetClusterHeadId(targetClusterId);
  if (targetChId != 0xFFFF && m_neighbors->IsNeighbor(targetChId)) {
    return NodeList::GetNode(targetChId)->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();
  }  
  // 否则使用网关
  return FindGatewayToCluster(targetClusterId);
}


bool RoutingProtocol::LookupRoute(Ipv4Address dst, RoutingTableEntry & rt)// 查找路由（内部函数）
{
  NS_LOG_FUNCTION(this << dst);  

  if (m_routingTable.LookupRoute(dst, rt))  // 查找路由表
  {
    if (rt.GetNextHopAddress() == Ipv4Address()) {    // 确保有有效的 Gateway
      rt.SetNextHopAddress(dst);// 如果 Gateway 为空，设置为目的地址（直连路由）
    }
    return true;
  }
  
  uint16_t dstNodeId = GetNodeIdFromIp(dst);  // 检查是否是直接邻居
  if (dstNodeId != 0xFFFF && m_neighbors->IsNeighbor(dstNodeId)) {
    rt = RoutingTableEntry(dst, dstNodeId, m_clusterManager->GetClusterId(dstNodeId),
                           GetPositionFromIp(dst), dst, dstNodeId, Idle,
                           GetPositionFromIp(dst), ROUTE_DIRECT, 1);
    rt.UpdateLifetime(Seconds(10));
    return true;
  }
  
  return false;
}


void RoutingProtocol::UpdateRoutingTable(Ipv4Address dst, Ipv4Address nexthop, uint16_t hopCount, RouteType t)
{ // 更新路由表（从控制消息中提取路由信息）
  NS_LOG_FUNCTION(this << dst << nexthop << hopCount);
  
  uint16_t dstNodeId = GetNodeIdFromIp(dst);
  uint16_t dstClusterId = (dstNodeId != 0xFFFF) ? m_clusterManager->GetClusterId(dstNodeId) : 0xFFFF;
  uint16_t nextNodeId = GetNodeIdFromIp(nexthop);
  
  RoutingTableEntry entry(dst, dstNodeId, dstClusterId, GetPositionFromIp(dst),  nexthop,
                           nextNodeId, Idle, GetPositionFromIp(nexthop), t, hopCount);  
  m_routingTable.AddRoute(entry);
}



Ptr<Ipv4Route> RoutingProtocol::LoopbackRoute(const Ipv4Header &header, Ptr<NetDevice> oif) const
{ // 添加 LoopbackRoute 实现
  NS_LOG_FUNCTION(this << header);
  NS_ASSERT(m_lo != nullptr);
  Ptr<Ipv4Route> rt = Create<Ipv4Route>();
  rt->SetDestination(header.GetDestination());
  
  std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator j = m_socketAddrs.begin();
  if (oif) {
    for (j = m_socketAddrs.begin(); j != m_socketAddrs.end(); ++j) {
      Ipv4InterfaceAddress iface = j->second;
      int32_t interface = m_Ipv4->GetInterfaceForAddress(iface.GetLocal());
      if (oif == m_Ipv4->GetNetDevice(static_cast<uint32_t> (interface))) {
        rt->SetSource(iface.GetLocal());
        break;
      }
    }
  }
  else{
    rt->SetSource(j->second.GetLocal());
  }
  // 使用环回设备
  NS_ASSERT_MSG(rt->GetSource()!=Ipv4Address()," Valid icra src addr not found! ");
  rt->SetGateway(Ipv4Address("127.0.0.1"));
  rt->SetOutputDevice(m_lo);
  return rt;
}


void RoutingProtocol::SetIpv4(Ptr<Ipv4> ipv4)
{
  NS_ASSERT(ipv4 != nullptr);
  NS_ASSERT(m_Ipv4 == nullptr);
  // NS_LOG_FUNCTION(this);
  m_Ipv4 = ipv4;

  // Get the pointer to Neighbors
  Ptr<ClusterNode> clusternode = m_Ipv4->GetObject<Node>()->GetObject<ClusterNode>();
  if (clusternode) {
    // uint16_t nodeId = clusternode->GetNodeId();
    m_neighbors = clusternode->GetObject<Neighbors>();
  }

  NS_ASSERT(m_Ipv4->GetNInterfaces() == 1 && m_Ipv4->GetAddress(0,0).GetLocal() == Ipv4Address("127.0.0.1"));

  m_lo = m_Ipv4->GetNetDevice(0);
  NS_ASSERT(m_lo != nullptr);
  
  /*Maybe Add route Here~~*/
  // RouteEntry route();

  Simulator::ScheduleNow(&RoutingProtocol::Start, this);
}


void RoutingProtocol::HelloTimerExpire(){
  if (m_paperMode) { PaperMaintenance(); return; }
  NS_LOG_FUNCTION(this->m_address);
  if (m_Ipv4 == nullptr) return; // 节点已销毁
  UpdateNodeFitness();
  SendHello();
  m_helloTimer.Cancel();

  Time optimalInterval = Seconds(0.25);            
            
  m_helloTimer.Schedule(optimalInterval);

}


/*Start Clustering*/
void RoutingProtocol::ClusteringTimerExpire()
{
  if (m_paperMode) { PaperStartClustering(); return; }
  NS_LOG_FUNCTION(this);
  Ptr<Node> node = m_Ipv4->GetObject<Node>();
  NS_ASSERT_MSG(node,"node pointer NULL");
  /*Update node Fitness*/
  Ptr<ClusterNode> clusternode = node->GetObject<ClusterNode>();
  NS_ASSERT_MSG(clusternode," ClusterNode ptr null");
  Ptr<EnergySourceContainer> sources = node->GetObject<EnergySourceContainer>();
  clusternode->updateResidualEnergy(sources->Get(0)->GetRemainingEnergy());
  clusternode->UpdateNeighborDegree();
  double fit = UpdateNodeFitness();//clusternode->ComputeFitness(0.25, 0.25, 0.25, 0.25);
  NS_ASSERT_MSG(fit<=1.0,"invalid fitness");
  // clusternode->UpdateFitness(fit);
  


  NS_LOG_INFO("Starting ICRA clustering at time " << Simulator::Now().GetSeconds() << "s");
  m_receivedCHs.clear();
  m_utilityTable.clear();
  m_attemptedCHs = 0;
  m_icraElectionDone = false;
  // ICRA Step 2.1 & 2.2: 计算效用值（已在UpdateNodeFitness中完成）
  double myUtility = clusternode->GetFitnessValue();
  NS_LOG_DEBUG("Node " << MyNodeId() << " utility value: " << myUtility);      
  // ICRA Step 2.3: 广播效用值消息
  SendUtility();      
  // 调度选举判断阶段（等待收集其他节点效用值）
  m_utilityExchangeTimer.SetFunction(&RoutingProtocol::IcraElectionPhase, this);
  m_utilityExchangeTimer.Schedule(m_utilityWaitTime);
 
  
  Simulator::Schedule(Seconds(5.0), &ClusterManager::CheckClusteringStatus, m_clusterManager, MyNodeId());
  // m_beacon_timer.Schedule(Seconds(5.0));
}

// clusterMaintainence
void RoutingProtocol::BeaconTimerExpire() 
{
  if (m_paperMode) { return; }
  // NS_LOG_FUNCTION(this);
  /*ClusterMaintainence*/
  // update fitness  
  Ptr<Node> node = GetObject<Node>();
  Ptr<ClusterNode> clusternode = node->GetObject<ClusterNode>();
  std::vector<uint16_t> DeleteNodeList;
  Ptr<EnergySourceContainer> sources = GetObject<Node>() -> GetObject<EnergySourceContainer>();
  clusternode->updateResidualEnergy(sources->Get(0) -> GetRemainingEnergy());
  clusternode->UpdateNeighborDegree();
  double fit =  UpdateNodeFitness();//clusternode -> ComputeFitness(0.25, 0.25, 0.25, 0.25);
  
  CollectFeedbackForQLearning();// 如果在Q学习训练模式，定期收集反馈


  NS_ASSERT_MSG(fit<=1.0,"invalid fitness");
  // clusternode->UpdateFitness(fit);

  if(clusternode->GetRoleType() == Head){
    uint16_t clusterId = m_clusterManager -> GetClusterId(MyNodeId());
    Ptr<Cluster> cluster = m_clusterManager -> GetCluster(clusterId);
    if(cluster == nullptr)
    {
      clusternode->SetClusterId(0xFFFF);
      clusternode->SetRole(Isolate);
      return;
    }
    SendClusterHeadBeacon(0xFFFF); // Broadcast CH_LIVE Beacon

    // ===== 优化：多阶段心跳容错机制（参考BioCluster）=====
    auto members = cluster->GetAllMembers();
    for(auto member=members.begin(); member!=members.end(); ++member) {
      uint16_t MemberNodeId = member->second->GetNodeId();
      if(MemberNodeId != MyNodeId()){ // check each member
        double latestUpdateTime = cluster->GetMemberUpdateTime(MemberNodeId).GetSeconds();
        double timelength = Simulator::Now().GetSeconds() - latestUpdateTime;

        // 硬失败：超过4s，确实丢失
        if(timelength > 4.0){
          DeleteNodeList.push_back(MemberNodeId);
          Simulator::Schedule(MilliSeconds(20), &ClusterManager::CheckClusteringStatus, m_clusterManager, MemberNodeId);
          m_memberHeartbeatMissCount.erase(MemberNodeId);  // 清除计数
          NS_LOG_INFO("ICRA: Hard failure detected for member " << MemberNodeId << " (>4s), removing");
        }
        // 软失败：2.5s-4s之间，可能是临时问题
        else if(timelength > 2.5){
          uint8_t missCount = 0;
          if(m_memberHeartbeatMissCount.find(MemberNodeId) != m_memberHeartbeatMissCount.end()) {
            missCount = m_memberHeartbeatMissCount[MemberNodeId];
          }
          missCount++;

          if(missCount >= 2) {
            // 连续2次丢失 -> 删除
            DeleteNodeList.push_back(MemberNodeId);
            Simulator::Schedule(MilliSeconds(20), &ClusterManager::CheckClusteringStatus, m_clusterManager, MemberNodeId);
            m_memberHeartbeatMissCount.erase(MemberNodeId);
            NS_LOG_INFO("ICRA: Soft failure confirmed for member " << MemberNodeId << " (2 consecutive misses), removing");
          }
          else {
            // 第一次/偶发丢失 -> 发送多次确认
            SendClusterHeadBeacon(MemberNodeId);
            SendClusterHeadBeacon(MemberNodeId);  // 额外确认
            m_memberHeartbeatMissCount[MemberNodeId] = missCount;
            NS_LOG_DEBUG("ICRA: Soft failure for member " << MemberNodeId << " (miss count: " << (int)missCount << "), sending extra beacons");
          }
        }
        // 轻微延迟：1.5s-2.5s之间
        else if(timelength > 1.5){
          SendClusterHeadBeacon(MemberNodeId);  // 单次确认
        }
        // 正常：心跳及时
        else {
          m_memberHeartbeatMissCount[MemberNodeId] = 0;  // 重置计数
        }
      }
    } // end of for member in members

    for (uint16_t id:DeleteNodeList){
      Ptr<ClusterNode> a=NodeList::GetNode(id)->GetObject<ClusterNode>();
      cluster -> RemoveMember(a);
      a->SetRole(Idle);
      NS_LOG_INFO("Cluster "<<cluster->GetClusterId()<<" remove member(NodeId) "<<id <<", ChNode: "<<cluster->GetClusterHeadId());
    }
  }
  // ===== 优化：Member端CH监控（参考BioCluster）=====
  else if (clusternode->GetRoleType() == Member) {
    uint16_t clusterId = clusternode->GetClusterId();
    if (clusterId != 0xFFFF) {
      uint16_t chNodeId = m_clusterManager->GetClusterHeadId(clusterId);

      if (chNodeId != 0xFFFF && m_neighbors) {
        auto neighborTable = m_neighbors->GetAll();
        bool chFound = false;

        for (auto it = neighborTable.begin(); it != neighborTable.end(); ++it) {
          if (it->second.NodeId == chNodeId) {
            chFound = true;
            double elapsed = Simulator::Now().GetSeconds() - it->second.UpdateTime.GetSeconds();
            double threshold = 6.0 * m_BeaconInterval.GetSeconds();  // 6倍Beacon间隔 = 4.5s（更宽容）

            if (elapsed > threshold) {
              // CH超时，离开簇
              NS_LOG_WARN("ICRA Member: CH " << chNodeId << " timeout (" << elapsed << "s > " << threshold << "s), leaving cluster " << clusterId);
              Ptr<Cluster> cluster = m_clusterManager->GetCluster(clusterId);
              if (cluster) {
                cluster->RemoveMember(clusternode);
              }
              clusternode->SetRole(Idle);
              clusternode->SetClusterId(0xFFFF);

              // 延迟1.0s后尝试重新加入（使用ICRA专用的TryToJoinCluster）
              Simulator::Schedule(Seconds(1.0), &RoutingProtocol::TryToJoinCluster, this);
            }
            break;
          }
        }

        if (!chFound) {
          // CH不在邻居表，离开簇
          NS_LOG_WARN("ICRA Member: CH " << chNodeId << " not in neighbor table, leaving cluster " << clusterId);
          Ptr<Cluster> cluster = m_clusterManager->GetCluster(clusterId);
          if (cluster) {
            cluster->RemoveMember(clusternode);
          }
          clusternode->SetRole(Idle);
          clusternode->SetClusterId(0xFFFF);

          // 延迟1.0s后尝试重新加入（使用ICRA专用的TryToJoinCluster）
          Simulator::Schedule(Seconds(1.0), &RoutingProtocol::TryToJoinCluster, this);
        }
      }
    }
  }

  // ===== 优化：Idle/Isolate节点周期性尝试加入簇（参考BioCluster维护机制）=====
  // 在维护阶段（t>6s），Idle节点每隔一定时间尝试加入簇
  else if (clusternode->GetRoleType() == Idle || clusternode->GetRoleType() == Isolate) {
    double currentTime = Simulator::Now().GetSeconds();
    if (currentTime > 6.0) {
      // 维护阶段：每2个Beacon周期尝试一次重新加入（提高频率以应对拓扑变化）
      static std::map<uint16_t, uint32_t> idleBeaconCounter;
      uint16_t myId = MyNodeId();
      idleBeaconCounter[myId]++;

      if (idleBeaconCounter[myId] % 2 == 0) {  // 每2次Beacon = 1.5s尝试一次
        NS_LOG_DEBUG("ICRA Beacon: Idle node " << myId << " periodically trying to join cluster (t=" << currentTime << "s)");
        Simulator::Schedule(MilliSeconds(100), &RoutingProtocol::TryToJoinCluster, this);
      }
    }
  }

  m_beacon_timer.Cancel();
  m_beacon_timer.Schedule(m_BeaconInterval);
}


void RoutingProtocol::ClusteringPerformanceTimerExpire()
{
  Ptr<ClusterNode> cNode = m_Ipv4->GetObject<Node>()->GetObject<ClusterNode>();

  if(cNode->GetNodeId() == 0)
  {
    // NS_LOG_LOGIC("--------------------Global Clusters Infomation--------------------");
    auto clusters = m_clusterManager->GetAllClusters();
    uint32_t ClusteredNodeNum = 0;
    uint16_t singleClusters = 0;
    for(auto cluster = clusters.begin();cluster!=clusters.end();++cluster)
    {
      ClusteredNodeNum += cluster->second->GetAllMembers().size();
      auto members = cluster->second->GetAllMembers();   
      // NS_LOG_LOGIC("----------ClusterId: "<<cluster->second->GetClusterId()<<"-----ClusterHeadId: "<<cluster->second->GetClusterHeadId()
                    // <<"-----Scale: "<<members.size()<<"---------");
      
      for(auto member = members.begin();member!=members.end();++member)
      {
        // auto Node = member->second;
        // NS_LOG_LOGIC("NodeId: "<<member->second->GetNodeId()<<"\t\t\t Role: "<<member->second->GetRoleType()<<"\t\t\t\t\t\t\t\t\t |");
      }

      if(members.size() <= 1){
        singleClusters ++;
      }
    }
    // NS_LOG_LOGIC("******************************************************************");
    // NS_LOG_LOGIC("簇结构数量: "<<clusters.size()<<", 入簇个数: "<<ClusteredNodeNum<<", 孤立簇个数: "<<singleClusters);
    // NS_LOG_LOGIC("-----------------------------"<<m_clusterManager->ArrangeClusterId()<<"-----------------------------------");
  }

  // NS_LOG_LOGIC("Role: "<< cNode->GetRoleType() <<", ClusterId: "<<cNode->GetClusterId()<<", FitValue: "<<cNode->GetFitnessValue()
  //               <<", time: "<<Simulator::Now().GetSeconds()<<" s");

  NS_LOG_LOGIC("Fitness Weight Value: " << "Deg: "<< m_weights.degreeWeight
                  << ", Dis: " << m_weights.distanceWeight
                  << ", Ener: " << m_weights.energyWeight
                  << ", Vel: " << m_weights.velocityWeight);
  m_clusteringStatusTimer.Cancel();
  m_clusteringStatusTimer.Schedule(Seconds(10.0));
}


void RoutingProtocol::SendUtility()
{
  NS_LOG_FUNCTION(this);  
  Ptr<ClusterNode> clusternode = GetObject<Node>()->GetObject<ClusterNode>();
  Ptr<MobilityModel> mobility = GetObject<Node>()->GetObject<MobilityModel>();  
  double utilityValue = clusternode->GetFitnessValue();
  Vector pos = mobility->GetPosition();
  Vector vel = mobility->GetVelocity();
  Ipv4Address addr = m_Ipv4->GetAddress(1,0).GetLocal();  
  for(auto socketAddr = m_socketAddrs.begin(); socketAddr != m_socketAddrs.end(); ++socketAddr)
  {
    Ptr<Socket> socket = socketAddr->first;
    Ipv4InterfaceAddress iface = socketAddr->second;    

    UtilityHeader utilityHeader(MyNodeId(), utilityValue, addr, pos, vel);    // 创建效用值消息
    Ptr<Packet> packet = Create<Packet>();
    packet->AddHeader(utilityHeader);    
    TypeHeader typeheader(ICRA_UTILITY);
    packet->AddHeader(typeheader);
    Ipv4Address dst;
    if (iface.GetMask() == Ipv4Mask::GetOnes()){
      dst = Ipv4Address ("255.255.255.255");
    } else {
      dst = iface.GetBroadcast();
    }
    
    Time jitter = Time(MilliSeconds(m_uniformRandomVariable->GetInteger(0, 10)));
    Simulator::Schedule(jitter, &RoutingProtocol::SendTo, this, socket, packet, dst);
    m_utilityPacketCount++;
    NS_LOG_INFO("Broadcast UTILITY message, value: " << utilityValue);
  }
}


void RoutingProtocol::RecvUtility(Ptr<Packet> packet, Ipv4Address myIp, Ipv4Address srcIp)
{
  NS_LOG_FUNCTION(this << srcIp);  
  UtilityHeader utilityHeader;
  packet->RemoveHeader(utilityHeader);  
  uint16_t senderNodeId = utilityHeader.GetNodeId();
  double senderUtility = utilityHeader.GetUtilityValue();  
  m_utilityTable[senderNodeId] = senderUtility;  // 存储效用值

  NS_LOG_INFO("Received UTILITY from node " << senderNodeId << ", utility: " << senderUtility);
  auto mobi = GetObject<Node>()->GetObject<MobilityModel>();  // 同时更新邻居表
  double life = CalculateLHTP(mobi->GetPosition(), mobi->GetVelocity(), utilityHeader.GetPosition(), utilityHeader.GetVelocity());
  
  Neighbors::NeighborInfo newNeighbor(
    senderNodeId,
    utilityHeader.GetIpAddress(),
    0xFFFF,
    utilityHeader.GetPosition(),
    utilityHeader.GetVelocity(),
    0.0,
    senderUtility,
    Idle,
    Simulator::Now(),
    Seconds(life)
  );
  m_neighbors->AddNeighbor(newNeighbor);
}


void RoutingProtocol::IcraElectionPhase()// ICRA选举判断阶段 (Step 2.3后半部分)
{
  NS_LOG_FUNCTION(this);  
  Ptr<ClusterNode> clusternode = GetObject<Node>()->GetObject<ClusterNode>();
  double myUtility = clusternode->GetFitnessValue();
  bool shouldBeCH = true;  

  for (const auto& entry : m_utilityTable) {  // 检查是否有邻居效用值大于自己
    if (entry.second > myUtility) {
      shouldBeCH = false;
      NS_LOG_DEBUG("Node " << entry.first << " has higher utility: " << entry.second << " > " << myUtility);
      break;
    }
  }  
  if (shouldBeCH) {    // 成为簇头候选
    clusternode->SetRole(BackupHead);
    NS_LOG_INFO("Node " << MyNodeId() << " elected as CH candidate (utility: " << myUtility << ")");
    double delay = m_uniformRandomVariable->GetValue(0, 0.2);    // 广播CH声明 (复用CH_CALL消息)
    Simulator::Schedule(Seconds(delay), &RoutingProtocol::SendClusterHeadAnnounceMent, this);
    m_icraElectionDone = true;
  } else {    // 不成为CH，等待接收CH声明后进入簇形成阶段
    NS_LOG_INFO("Node " << MyNodeId() << " will not be CH, waiting for CH announcements");
    Simulator::Schedule(Seconds(1.0), &RoutingProtocol::IcraClusterFormation, this);    // 调度簇形成检查（给CH广播留出时间）
  }

  // ===== BioCluster-style优化：在t=2.5s时调度ProactivelyJoinCluster，为Idle节点提供第二次加入机会 =====
  Simulator::Schedule(Seconds(2.5), &RoutingProtocol::ProactivelyJoinCluster, this);
}


void RoutingProtocol::IcraClusterFormation()// ICRA簇形成阶段 (Algorithm 1) - 优化版本
{
  NS_LOG_FUNCTION(this);
  Ptr<ClusterNode> clusternode = GetObject<Node>()->GetObject<ClusterNode>();

  if (clusternode->GetRoleType() == Head ||  clusternode->GetRoleType() == Member ||  clusternode->GetRoleType() == BackupHead) {
    NS_LOG_DEBUG("Node " << MyNodeId() << " already has role, skip formation");
    return;  // 如果已经是CH或已加入簇，直接返回
  }

  // ===== 优化：不再立即遍历CH列表，而是等待RecvHeadAnnounce触发SelectBestClusterHeadAndJoin =====
  // 如果还没有收到任何CH_CALL，可能需要等待或自己成为CH

  uint16_t totalCHs = m_receivedCHs.size();
  NS_LOG_INFO("ICRA Formation: Node " << MyNodeId() << " checking cluster formation, received " << totalCHs << " CH announcements");

  // 如果1秒后仍然没有收到CH_CALL（m_hasReceivedChCall == false），说明附近没有CH
  if (totalCHs == 0 || !m_hasReceivedChCall) {
    // 优化：检查邻居数量再决定是否成为CH（参考BioCluster）
    uint32_t neighborCount = 0;
    uint32_t idleNeighborCount = 0;
    if (m_neighbors) {
      auto neighbors = m_neighbors->GetAll();
      neighborCount = neighbors.size();
      for(auto it = neighbors.begin(); it != neighbors.end(); ++it) {
        if(it->second.Role == Idle) {
          idleNeighborCount++;
        }
      }
    }

    // 只有在满足以下条件之一时才成为CH：
    // 1. 没有任何邻居（真正孤立）
    // 2. 邻居数量>=5 且 Idle邻居>=3（足够形成有意义的簇）
    if(neighborCount == 0 || (neighborCount >= 5 && idleNeighborCount >= 3)) {
      NS_LOG_INFO("ICRA Formation: Node " << MyNodeId() << " has no CH candidates, becoming CH"
                  << " (neighbors=" << neighborCount << ", idle=" << idleNeighborCount << ")");
      clusternode->SetRole(BackupHead);
      double delay = m_uniformRandomVariable->GetValue(0, 0.2);
      Simulator::Schedule(Seconds(delay), &RoutingProtocol::SendClusterHeadAnnounceMent, this);
    }
    else {
      // 邻居不足，保持Idle状态等待
      NS_LOG_DEBUG("ICRA Formation: Not enough neighbors to become CH (neighbors=" << neighborCount
                   << ", idle=" << idleNeighborCount << "), staying Idle");
      clusternode->SetRole(Idle);
    }
    return;
  }

  // 如果已经收到CH_CALL并调度了SelectBestClusterHeadAndJoin，就等待它执行
  // 否则手动触发一次（以防定时器出问题）
  if (m_hasReceivedChCall && !m_chSelectionTimer.IsSuspended() && m_chSelectionTimer.IsRunning()) {
    NS_LOG_DEBUG("ICRA Formation: CH selection timer already scheduled, waiting for it to execute");
  }
  else if (m_hasReceivedChCall && !m_chCandidates.empty()) {
    NS_LOG_INFO("ICRA Formation: Manually triggering CH selection for node " << MyNodeId());
    SelectBestClusterHeadAndJoin();
  }
  else {
    // 意外状态：也需要检查邻居数量
    uint32_t neighborCount = 0;
    uint32_t idleNeighborCount = 0;
    if (m_neighbors) {
      auto neighbors = m_neighbors->GetAll();
      neighborCount = neighbors.size();
      for(auto it = neighbors.begin(); it != neighbors.end(); ++it) {
        if(it->second.Role == Idle) {
          idleNeighborCount++;
        }
      }
    }

    if(neighborCount == 0 || (neighborCount >= 5 && idleNeighborCount >= 3)) {
      NS_LOG_WARN("ICRA Formation: Node " << MyNodeId() << " in unexpected state, becoming CH"
                  << " (neighbors=" << neighborCount << ", idle=" << idleNeighborCount << ")");
      clusternode->SetRole(BackupHead);
      double delay = m_uniformRandomVariable->GetValue(0, 0.2);
      Simulator::Schedule(Seconds(delay), &RoutingProtocol::SendClusterHeadAnnounceMent, this);
    }
    else {
      NS_LOG_DEBUG("ICRA Formation: Not enough neighbors, staying Idle");
      clusternode->SetRole(Idle);
    }
  }
}


bool RoutingProtocol::CheckLHTP(uint16_t neighborId)// 检查链路保持时间条件
{
  NS_LOG_FUNCTION(this << neighborId);
  auto neighbors = m_neighbors->GetAll();
  if (neighbors.find(neighborId) != neighbors.end()) {
    Time lhtp = neighbors.at(neighborId).NeighborLifeLength;
    bool result = lhtp.GetSeconds() > m_lhtpThreshold;
    NS_LOG_DEBUG("LHTP check for neighbor " << neighborId << ": " << lhtp.GetSeconds() << "s > "
                      << m_lhtpThreshold  << "s ? " << (result ? "YES" : "NO"));
    return result;
  }
  NS_LOG_WARN("Neighbor " << neighborId << " not in neighbor table");
  return false;
}


void RoutingProtocol::SelectBestClusterHeadAndJoin()
{
  // KMeans-style CH选择：从收集的所有候选中选择最佳CH并加入
  NS_LOG_FUNCTION(this);
  Ptr<ClusterNode> clusternode = GetObject<Node>()->GetObject<ClusterNode>();
  NodeRole myRole = clusternode->GetRoleType();

  // 如果已经有角色，清空候选列表并返回
  if (myRole != Idle) {
    NS_LOG_DEBUG("ICRA: Node " << MyNodeId() << " already has role " << myRole << ", canceling CH selection");
    m_chCandidates.clear();
    m_hasReceivedChCall = false;
    return;
  }

  if (m_chCandidates.empty()) {
    NS_LOG_INFO("ICRA: Node " << MyNodeId() << " has no CH candidates");
    m_hasReceivedChCall = false;
    return;
  }

  double myUtility = clusternode->GetFitnessValue();

  // 过滤候选：只保留满足ICRA条件的CH (myUtility > chUtility && LHTP > threshold)
  std::vector<ClusterHeadCandidate> validCandidates;
  for (const auto& candidate : m_chCandidates) {
    if (myUtility > candidate.fitness && CheckLHTP(candidate.nodeId)) {
      validCandidates.push_back(candidate);
    }
  }

  if (validCandidates.empty()) {
    NS_LOG_INFO("ICRA: Node " << MyNodeId() << " has no valid CH candidates (ICRA条件不满足)");

    // 优化：检查邻居数量再决定是否成为CH（参考BioCluster）
    uint32_t neighborCount = 0;
    uint32_t idleNeighborCount = 0;
    if (m_neighbors) {
      auto neighbors = m_neighbors->GetAll();
      neighborCount = neighbors.size();
      for(auto it = neighbors.begin(); it != neighbors.end(); ++it) {
        if(it->second.Role == Idle) {
          idleNeighborCount++;
        }
      }
    }

    // 只有在满足以下条件之一时才成为CH：
    // 1. 没有任何邻居（真正孤立）
    // 2. 邻居数量>=5 且 Idle邻居>=3（足够形成有意义的簇）
    if(neighborCount == 0 || (neighborCount >= 5 && idleNeighborCount >= 3)) {
      NS_LOG_INFO("ICRA: Becoming CH itself (neighbors=" << neighborCount << ", idle=" << idleNeighborCount << ")");
      clusternode->SetRole(BackupHead);
      double delay = m_uniformRandomVariable->GetValue(0, 0.2);
      Simulator::Schedule(Seconds(delay), &RoutingProtocol::SendClusterHeadAnnounceMent, this);
    }
    else {
      // 邻居不足，保持Idle状态
      NS_LOG_DEBUG("ICRA: Not enough neighbors to become CH (neighbors=" << neighborCount
                   << ", idle=" << idleNeighborCount << "), staying Idle");
      clusternode->SetRole(Idle);
    }

    m_chCandidates.clear();
    m_hasReceivedChCall = false;
    return;
  }

  // 从有效候选中选择最佳CH：按FITNESS从高到低排序
  std::sort(validCandidates.begin(), validCandidates.end(),
            [](const ClusterHeadCandidate& a, const ClusterHeadCandidate& b) {
              return a.fitness > b.fitness;  // Fitness高的优先
            });

  // 选择Fitness最高的CH
  const ClusterHeadCandidate& bestCH = validCandidates[0];
  NS_LOG_INFO("ICRA: Node " << MyNodeId() << " selected best CH " << bestCH.nodeId
              << " (fitness: " << bestCH.fitness << " from " << validCandidates.size() << " valid candidates)");

  // 发送JOIN_REQ到最佳CH
  SendClusterJoinRequest(bestCH.ipAddress);

  m_chCandidates.clear();
  m_hasReceivedChCall = false;
}


bool RoutingProtocol::ShouldSwitchCluster(uint16_t newChId, double newChFitness)
{
  // Stickiness机制：检查是否应该切换到新簇
  NS_LOG_FUNCTION(this << newChId << newChFitness);

  Ptr<ClusterNode> clusternode = GetObject<Node>()->GetObject<ClusterNode>();
  uint16_t currentClusterId = clusternode->GetClusterId();

  if (currentClusterId == 0xFFFF) {
    return true;  // 没有当前簇，可以加入
  }

  Ptr<Cluster> currentCluster = m_clusterManager->GetCluster(currentClusterId);
  if (!currentCluster) {
    return true;  // 当前簇不存在，可以加入新簇
  }

  // 如果簇规模太小(<=1，只有自己)，容易切换
  uint16_t clusterSize = currentCluster->GetAllMembers().size();
  if (clusterSize <= 1) {
    NS_LOG_DEBUG("ICRA Stickiness: Current cluster size <= 1, allow switching");
    return true;
  }

  // 获取当前CH的Fitness
  uint16_t currentChId = currentCluster->GetClusterHeadId();
  double currentChFitness = 0.0;

  auto neighbors = m_neighbors->GetAll();
  bool foundCurrentCH = false;
  for (const auto& neighbor : neighbors) {
    if (neighbor.first == currentChId) {
      currentChFitness = neighbor.second.FitnessValue;
      foundCurrentCH = true;
      break;
    }
  }

  if (!foundCurrentCH) {
    NS_LOG_WARN("ICRA Stickiness: Current CH " << currentChId << " not in neighbor table, allow switching");
    return true;  // 当前CH不在邻居表，可能已失联
  }

  // 计算Fitness改进比例
  double fitnessImprovement = (newChFitness - currentChFitness) / (currentChFitness + 1e-6);

  // 自适应Stickiness阈值（参考BioCluster，但更保守以减少切换）
  double stickynessThreshold = 0.35;  // 默认35%（更高阈值）
  if (clusterSize <= 2) {
    stickynessThreshold = 0.20;  // 小簇：20%阈值
  } else if (clusterSize <= 4) {
    stickynessThreshold = 0.28;  // 中簇：28%阈值
  }

  bool shouldSwitch = (fitnessImprovement >= stickynessThreshold);

  NS_LOG_INFO("ICRA Stickiness: Node " << MyNodeId()
              << " current CH " << currentChId << " (fitness " << currentChFitness << ")"
              << " vs new CH " << newChId << " (fitness " << newChFitness << ")"
              << ", improvement: " << (fitnessImprovement*100) << "%"
              << ", threshold: " << (stickynessThreshold*100) << "%"
              << ", cluster size: " << clusterSize
              << " -> " << (shouldSwitch ? "SWITCH" : "STAY"));

  return shouldSwitch;
}


void RoutingProtocol::ProactivelyJoinCluster()
{
  // BioCluster-style主动加入簇机制（在t=3.5s左右执行）
  // 目的：为初始阶段未能加入簇的Idle节点提供第二次机会
  NS_LOG_FUNCTION(this);

  Ptr<ClusterNode> clusterNode = GetObject<Node>()->GetObject<ClusterNode>();
  if (!clusterNode || clusterNode->GetRoleType() != Idle) {
    NS_LOG_DEBUG("ICRA Proactive: Node " << MyNodeId() << " is not Idle, skip proactive join");
    return;  // 只处理Idle节点
  }

  // 统计邻居情况
  auto neighborTable = m_neighbors->GetAll();
  uint32_t neighborCount = neighborTable.size();
  uint32_t idleNeighborCount = 0;

  for (auto it = neighborTable.begin(); it != neighborTable.end(); ++it) {
    if (it->second.Role == Idle) {
      idleNeighborCount++;
    }
  }

  NS_LOG_INFO("ICRA Proactive: Node " << MyNodeId() << " checking neighbors"
              << " (total=" << neighborCount << ", idle=" << idleNeighborCount << ")");

  // 寻找距离最近的Head或BackupHead邻居
  Vector myPos = GetMyPosition();
  double minDistance = std::numeric_limits<double>::max();
  Ipv4Address bestChAddr;
  uint16_t bestChId = 0xFFFF;

  for (auto it = neighborTable.begin(); it != neighborTable.end(); ++it) {
    const auto& neighborInfo = it->second;
    if (neighborInfo.Role == Head || neighborInfo.Role == BackupHead) {
      double distance = CalculateDistance(myPos, neighborInfo.Position);
      if (distance < minDistance) {
        minDistance = distance;
        bestChAddr = neighborInfo.ipAddress;
        bestChId = it->first;
      }
    }
  }

  // 决策流程（参考BioCluster）
  if (bestChAddr.IsInitialized() && !bestChAddr.IsBroadcast()) {
    // 找到了CH/BackupHead，发送JOIN_REQ
    NS_LOG_INFO("ICRA Proactive: Idle node " << MyNodeId() << " proactively joining CH " << bestChId);
    SendClusterJoinRequest(bestChAddr);
  }
  else if (neighborCount >= 5 && idleNeighborCount >= 3) {
    // 没有CH但邻居充足，尝试自提升为CH（随机延迟0-1秒避免冲突）
    double delay = m_uniformRandomVariable->GetValue(0.0, 1.0);
    NS_LOG_INFO("ICRA Proactive: Node " << MyNodeId() << " has sufficient neighbors, scheduling self-promotion in "
                << delay << "s");
    Simulator::Schedule(Seconds(delay), [this]() {
      Ptr<ClusterNode> cn = GetObject<Node>()->GetObject<ClusterNode>();
      if (cn && cn->GetRoleType() == Idle) {  // 再次检查状态
        NS_LOG_INFO("ICRA Proactive: Node " << MyNodeId() << " promoting to BackupHead");
        cn->SetRole(BackupHead);
        double announceDelay = m_uniformRandomVariable->GetValue(0, 0.2);
        Simulator::Schedule(Seconds(announceDelay), &RoutingProtocol::SendClusterHeadAnnounceMent, this);
      }
    });
  }
  else {
    // 邻居不足，保持Idle状态（不强制创建孤立簇）
    NS_LOG_DEBUG("ICRA Proactive: Idle node " << MyNodeId() << " has no accessible cluster heads"
                 << " and insufficient neighbors (total=" << neighborCount << ", idle=" << idleNeighborCount << ")");
  }
}


void RoutingProtocol::TryToJoinCluster()
{
  // 维护阶段Member节点失去CH后，尝试重新加入簇
  // 参考BioCluster，但简化流程（不需要完整的ICRA选举）
  NS_LOG_FUNCTION(this);

  Ptr<ClusterNode> clusternode = GetObject<Node>()->GetObject<ClusterNode>();

  if (!clusternode || !m_neighbors) {
    return;
  }

  // 只处理Idle/Isolate节点（已经失去簇的节点）
  NodeRole myRole = clusternode->GetRoleType();
  if (myRole == Member || myRole == Head || myRole == BackupHead) {
    NS_LOG_DEBUG("ICRA TryToJoin: Node " << MyNodeId() << " already in cluster, skip");
    return;  // 已经在簇中，不需要重新加入
  }

  // 在邻居表中寻找最优簇头
  auto neighbors = m_neighbors->GetAll();
  Ipv4Address bestHead;
  uint16_t bestHeadId = 0xFFFF;
  double bestFitness = -1.0;
  double minDistance = std::numeric_limits<double>::max();

  Vector myPos = GetMyPosition();

  uint32_t headCount = 0;
  uint32_t backupHeadCount = 0;

  for (auto &neighbor : neighbors) {
    // Count Head/BackupHead nodes.
    if (neighbor.second.Role == Head) headCount++;
    if (neighbor.second.Role == BackupHead) backupHeadCount++;

    // 寻找角色为Head或BackupHead的邻居
    if (neighbor.second.Role == Head || neighbor.second.Role == BackupHead) {
      // 优先选择fitness最高的簇头，距离作为次要条件
      double fitness = neighbor.second.FitnessValue;
      double distance = CalculateDistance(myPos, neighbor.second.Position);

      if (fitness > bestFitness || (fitness == bestFitness && distance < minDistance)) {
        bestFitness = fitness;
        minDistance = distance;
        bestHead = neighbor.second.ipAddress;
        bestHeadId = neighbor.first;
      }
    }
  }

  NS_LOG_DEBUG("ICRA TryToJoin: Node " << MyNodeId() << " found " << headCount << " Heads and "
               << backupHeadCount << " BackupHeads in " << neighbors.size() << " neighbors");

  if (bestHead.IsInitialized() && !bestHead.IsBroadcast()) {
    // 找到了合适的簇头，发送JOIN_REQ
    NS_LOG_INFO("ICRA TryToJoin: Node " << MyNodeId() << " trying to rejoin cluster, sending JOIN_REQ to CH "
                << bestHeadId << " (fitness=" << bestFitness << ")");
    SendClusterJoinRequest(bestHead);
  }
  else {
    // 没有找到任何Head/BackupHead邻居
    NS_LOG_DEBUG("ICRA TryToJoin: Node " << MyNodeId() << " found no available CH in neighbors");

    // 检查是否应该成为孤立簇（但标准更严格，避免过多孤立簇）
    uint32_t neighborCount = neighbors.size();
    uint32_t idleNeighborCount = 0;

    for (auto &neighbor : neighbors) {
      if (neighbor.second.Role == Idle) {
        idleNeighborCount++;
      }
    }

    // 参考BioCluster的CheckClusteringStatus逻辑：
    // 只有在满足严格邻居条件时才创建簇
    // 条件：neighborCount == 0 (真正孤立) 或 (neighborCount >= 5 && idleNeighborCount >= 3)
    if (neighborCount == 0 || (neighborCount >= 5 && idleNeighborCount >= 3)) {
      NS_LOG_INFO("ICRA TryToJoin: Node " << MyNodeId() << " creating cluster (neighbors="
                  << neighborCount << ", idle=" << idleNeighborCount << ")");
      clusternode->SetRole(BackupHead);
      double delay = m_uniformRandomVariable->GetValue(0, 0.2);
      Simulator::Schedule(Seconds(delay), &RoutingProtocol::SendClusterHeadAnnounceMent, this);
    }
    else {
      // 邻居不足，保持Idle状态等待
      NS_LOG_DEBUG("ICRA TryToJoin: Node " << MyNodeId() << " staying Idle (neighbors="
                   << neighborCount << ", idle=" << idleNeighborCount << ")");
    }
  }
}


void RoutingProtocol::PrintRoutingTable(Ptr<OutputStreamWrapper> stream, Time::Unit unit) const{
}

void RoutingProtocol::UpdateRouteToNeighbor(Ipv4Address sender, Ipv4Address receiver){
}




void RoutingProtocol::DeferredRouteOutput(Ptr<const Packet> p, const Ipv4Header &header, const UnicastForwardCallback &ucb, const ErrorCallback & ecb){
  NS_LOG_FUNCTION(this << p <<header);
  NS_ASSERT(p!=nullptr && p!= Ptr<Packet>());  
  // if(m_queue.GetSize() == 0){
  //   m_checkQueueTimer.Cancel();
  //   m_checkQueueTimer.Schedule(Seconds(0.5));
  // }
  Ipv4Address dst = header.GetDestination();
  NS_LOG_DEBUG("Deferring packet " << p->GetUid() << " to " << dst << " (Protocol: " << (uint16_t)header.GetProtocol() << ")");  
  QueueEntry newEntry(p, header, ucb, ecb);
  bool result = m_queue.Enqueue(newEntry);
  // m_queueAddr.Insert
  if(result){
    NS_LOG_INFO("Successfully queued packet " << p->GetUid() << " to " << dst << ", queue size: " << m_queue.GetSize());
    m_totalPacketsSent++;
    auto & stat = m_packetStats[dst];
    stat.sent++;
    stat.lastUpdate = Simulator::Now();
  }else {
    NS_LOG_WARN("Failed to queue packet " << p->GetUid() << " to " << dst << ", queue may be full or packet is duplicate");
    m_totalPacketsDropped++;
    auto & stat = m_packetStats[dst];
    stat.dropped++;
    stat.lastUpdate = Simulator::Now();
    ecb(p, header, Socket::ERROR_NOTERROR);    // 队列满或重复，直接触发错误回调
  }

}


Ptr<Socket> RoutingProtocol::FindSocketWithInterfaceAddress (Ipv4InterfaceAddress iface) const{
  NS_LOG_FUNCTION(this << iface);
  for(auto j = m_socketAddrs.begin();j!=m_socketAddrs.end();++j){
    Ptr<Socket> socket = j -> first;
    Ipv4InterfaceAddress ifaceAddr = j->second;
    if(iface == ifaceAddr){
      return socket;
    }
  }
  Ptr<Socket> nullsocket;
  return nullsocket;
}


bool RoutingProtocol::IsMyOwnAddress(Ipv4Address src){
  // NS_LOG_FUNCTION(this << src);
  for (auto k = m_socketAddrs.begin(); k!= m_socketAddrs.end();++k){
    Ipv4InterfaceAddress iface = k->second;
    if(src == iface.GetLocal()){
      return true;
    }
  }
  return false;
}


void RoutingProtocol::SendTo(Ptr<Socket> socket, Ptr<Packet> packet, Ipv4Address dst){
  if(socket == nullptr){
    NS_LOG_DEBUG("socket is null");
    return;
  }
  socket->SendTo(packet,0,InetSocketAddress(dst,ICRA_PORT));
}

void RoutingProtocol::SendPacketFromQueue(Ipv4Address dst, Ptr<Ipv4Route> route)
{
  NS_LOG_FUNCTION(this);
  QueueEntry entry;
   if (!m_queue.Dequeue(dst, entry)) {
    NS_LOG_DEBUG("No queued packets for destination " << dst);
    return;
  }  
  NS_LOG_INFO("Dequeued packet " << entry.GetPacket()->GetUid() << " to " << dst);  

  if (!route || route->GetGateway() == Ipv4Address()) {  // 验证路由的有效性
    NS_LOG_WARN("Invalid route for dequeued packet to " << dst << ", re-queueing");
    // 重新入队（但要小心避免无限循环）
    if (entry.GetExpireTime() > Seconds(0)) {
      m_queue.Enqueue(entry);
    } 
    else {
      m_totalPacketsDropped++;
      auto &stat = m_packetStats[dst];
      stat.dropped++;
      stat.lastUpdate = Simulator::Now();
      
      NS_LOG_WARN("Packet expired, dropping");
      ErrorCallback ecb = entry.GetErrorCallback();
      ecb(entry.GetPacket(), entry.GetIpv4Header(), Socket::ERROR_NOROUTETOHOST);
    }
    return;
  }  
  UnicastForwardCallback ucb = entry.GetUnicastForwardCallback();  // 使用回调发送数据包
  try {
    ucb(route, entry.GetPacket(), entry.GetIpv4Header());
    NS_LOG_INFO("Successfully sent dequeued packet " << entry.GetPacket()->GetUid() << " to " << dst << " via " << route->GetGateway());
    m_totalPacketsSent++;
    auto & stat = m_packetStats[dst];
    stat.sent++;
    stat.lastUpdate = Simulator::Now();
  } catch (...) {
    NS_LOG_ERROR("Failed to send dequeued packet, calling error callback");
    m_totalPacketsDropped++;
    auto & stat = m_packetStats[dst];
    stat.dropped++;
    stat.lastUpdate = Simulator::Now();
    ErrorCallback ecb = entry.GetErrorCallback();
    ecb(entry.GetPacket(), entry.GetIpv4Header(), Socket::ERROR_NOROUTETOHOST);
  }
}


/* HELLO */
void RoutingProtocol::SendHello(){
  // NS_LOG_FUNCTION(this);
  /*packet entry parameters initialize*/
  Ptr<ClusterNode> clusternode = GetObject<Node>()->GetObject<ClusterNode>();
  Ptr<MobilityModel> mobility =  GetObject<Node>()->GetObject<MobilityModel>();
  Ptr<EnergySourceContainer> sources = GetObject<Node>()->GetObject<EnergySourceContainer>();
  NS_ASSERT(clusternode!=nullptr);
  NS_ASSERT(mobility!=nullptr);
  NS_ASSERT(sources!=nullptr);

  NodeRole myrole = GetObject<Node>()->GetObject<ClusterNode>()->GetRoleType();
  uint16_t clusterId = clusternode->GetClusterId();
  Ipv4Address addr = m_Ipv4->GetAddress(1,0).GetLocal();
  Vector pos = mobility->GetPosition();
  Vector vel = mobility->GetVelocity();
  double InitialEnergy  = sources->Get(0)->GetInitialEnergy();
  double ResidualEnergy = sources->Get(0)->GetRemainingEnergy();
  // double fitness = UpdateNodeFitness();
  double fitness = UpdateNodeFitness();//clusternode->ComputeFitness(0.25, 0.25, 0.25, 0.25);
  // clusternode->UpdateFitness(fitness);
  
  for(auto socketAddr = m_socketAddrs.begin(); socketAddr != m_socketAddrs.end(); ++socketAddr)
  {
    Ptr<Socket> socket = socketAddr -> first;
    Ipv4InterfaceAddress iface = socketAddr -> second;
    /*Create Packet*/
    HelloHeader helloheader(myrole,MyNodeId(),clusterId, pos, addr, vel, InitialEnergy, ResidualEnergy, fitness);
    Ptr<Packet> packet = Create<Packet>();
    packet->AddHeader(helloheader);
    TypeHeader typeheader(ICRA_HELLO);
    packet->AddHeader(typeheader);
    /*Setting to Send*/
    Ipv4Address dst;// = Ipv4Address ("255.255.255.255");
    if (iface.GetMask() == Ipv4Mask::GetOnes()){
      dst = Ipv4Address ("255.255.255.255");
    }
    else {
      dst = iface.GetBroadcast();
    }
    Time jitter = Time (MilliSeconds (m_uniformRandomVariable->GetInteger (0, 10)));
    // Simulator::Schedule (jitter, [socket, packet, dst]() 
    //                     {
    //                       socket->SendTo(packet, 0, InetSocketAddress(dst, ICRA_PORT));
    //                     });
    Simulator::Schedule(jitter, &RoutingProtocol::SendTo, this, socket, packet, dst);
    m_helloPacketCount++;
    // SendTo(socket, packet, dst);
    NS_LOG_INFO(" Broadcast Hello packet");
  }
}

/*CH_CALL*/
void RoutingProtocol::SendClusterHeadAnnounceMent()
{
  // NS_LOG_FUNCTION(this);
  if(GetObject<Node>()->GetObject<ClusterNode>()->GetRoleType() == Member)
  {
    if(!m_clusterManager->ExistMember(MyNodeId()))
    {
      GetObject<Node>()->GetObject<ClusterNode>()->SetClusterId(0xFFFF);
      GetObject<Node>()->GetObject<ClusterNode>()->SetRole(Idle);
    }
    return;
  }
  bool IsPioneer = true;
  if(GetObject<Node>()->GetObject<ClusterNode>()->GetRoleType() == Isolate){
    auto neighbors = m_neighbors->GetAll();
    for(auto neighbor = neighbors.begin(); neighbor!=neighbors.end();++neighbor){
      if(neighbor->second.Role == BackupHead)  {
        if(Simulator::Now().GetSeconds() - neighbor->second.UpdateTime.GetSeconds() > 0.03)
        {
          continue;
        }
        IsPioneer = false;
        return;
      }
    }
    if(IsPioneer)
    {
      GetObject<Node>()->GetObject<ClusterNode>()->SetRole(BackupHead);
    }
  }

  if(IsPioneer)
  {
    Ptr<Node> node = GetObject<Node>();
    Ptr<Packet> CH_CALL_pkt = Create<Packet>();
    Vector pos = node->GetObject<MobilityModel>()->GetPosition();
    Vector vel = node->GetObject<MobilityModel>()->GetVelocity();
    Ptr <EnergySourceContainer> sources = node->GetObject<EnergySourceContainer>();
    double ResEnergy = sources->Get(0)->GetRemainingEnergy();
    double fitness = node->GetObject<ClusterNode>()->GetFitnessValue();
    for(auto j = m_socketAddrs.begin();j!=m_socketAddrs.end();++j)
    {
      Ptr<Socket> socket = j->first;
      Ipv4InterfaceAddress iface = j->second;
      Ipv4Address addr = socket->GetNode()->GetObject<Ipv4>()->GetAddress(1,0).GetLocal();
      ClusterHeadAnnounceHeader clusterheadheader(MyNodeId(), ResEnergy,addr, pos, vel, fitness, Simulator::Now().GetSeconds());
      CH_CALL_pkt->AddHeader(clusterheadheader);
      TypeHeader typeHeader(ICRA_CH_CALL);    // Set Message Type
      CH_CALL_pkt->AddHeader(typeHeader);
      Ipv4Address dst;
      if(iface.GetMask() == Ipv4Mask()){
        dst = Ipv4Address("255.255.255.255");
      }
      else {
        dst = iface.GetBroadcast();
      }
      socket->SendTo(CH_CALL_pkt, 0, InetSocketAddress(dst, ICRA_PORT));
      m_chCallPacketCount++;
      Simulator::Schedule(MilliSeconds(1500),&RoutingProtocol::CheckPacketStatus, this, socket,"CH_CALL",dst);      
      NS_LOG_INFO("\t Broadcast \t CH_CALL, Time: "<<Simulator::Now().GetSeconds()<<" s");
    }
  }
}


/*JOIN_REQ*/
void RoutingProtocol::SendClusterJoinRequest( Ipv4Address dst)
{
  // NS_LOG_FUNCTION(this);
  Ptr<Node> node = GetObject<Node>();
  NS_ASSERT_MSG(node,"node ptr is == 0");
  Ptr<ClusterNode> clusternode = node->GetObject<ClusterNode>();
  if(node->GetObject<ClusterNode>()->GetRoleType()==Member){
    if(clusternode->GetClusterId() < NodeList::GetNNodes()) {
      NS_LOG_INFO("Node "<<MyNodeId()<<" has joined a cluster.");
      return;
    }
  }
  else{
    if(clusternode->GetRoleType() == Head){
      NS_LOG_INFO("Node "<<MyNodeId()<<" has already been the Head of cluster "<<clusternode->GetClusterId());
      return;
    }
  }
  
  /*Create a New Packet*/
  Ptr<Packet> packet = Create<Packet>();
  Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
  Ptr<EnergySourceContainer> sources = node->GetObject<EnergySourceContainer>();
  NS_ASSERT(mobility != nullptr);
  NS_ASSERT(sources != nullptr);
  Vector pos = mobility->GetPosition();
  Vector vel = mobility->GetVelocity();
  NodeRole role = clusternode->GetRoleType();
  double ResEnergy = sources->Get(0)->GetRemainingEnergy();

  for(auto j = m_socketAddrs.begin(); j!=m_socketAddrs.end();++j){
    Ptr<Socket> socket = j->first;
    // Ipv4InterfaceAddress iface = j->second;
    Ipv4Address myAddr = m_Ipv4->GetAddress(1,0).GetLocal();
    JoinClusterRequestHeader joinheader(MyNodeId(),role,myAddr, ResEnergy, Simulator::Now().GetSeconds(), pos, vel);
    packet->AddHeader(joinheader);
    TypeHeader typeheader(ICRA_JOIN_REQ);
    packet->AddHeader(typeheader);
    socket->SendTo(packet, 0, InetSocketAddress(dst, ICRA_PORT));
    m_joinReqPacketCount++;
    Simulator::Schedule(MilliSeconds(1500),&RoutingProtocol::CheckPacketStatus,this,socket,"JOIN_REQ",dst);
    NS_LOG_INFO(" Send JOIN_REQ to "<<dst<<" at time "<<Simulator::Now().GetSeconds()<<"s");
  }
}


/*JOIN_RESP*/
void RoutingProtocol::SendClusterJoinResponse(Ipv4Address dst)
{
  // NS_LOG_FUNCTION(this<< dst);
  uint16_t myNodeId = MyNodeId();
  uint16_t ClusterId = m_clusterManager->GetClusterId(myNodeId);
  Ipv4Address myIp = m_Ipv4->GetAddress(1,0).GetLocal();
  Ptr<Packet> JOIN_RESP = Create<Packet>();
  ClusterHeadResponseHeader JoinRespHeader(myNodeId,ClusterId,myIp,Simulator::Now().GetSeconds());
  JOIN_RESP->AddHeader(JoinRespHeader);
  TypeHeader typehdr(ICRA_JOIN_RESP);
  JOIN_RESP->AddHeader(typehdr);
  for(auto j = m_socketAddrs.begin();j!=m_socketAddrs.end();++j)
  {
    Ptr<Socket>socket = j->first;
    // Ipv4InterfaceAddress iface = j->second;
    if (socket == nullptr) {
      NS_LOG_WARN(" Invalid socket, skip");
      continue;
    }
    int ret = socket->SendTo(JOIN_RESP, 0, InetSocketAddress(dst, ICRA_PORT));
    if (ret < 0) {
      NS_LOG_WARN(" Failed to send JOIN_RESP to " << dst);
    } else {
      m_joinRespPacketCount++;
      NS_LOG_INFO(" Send \t\t JOIN_RESP \t\t to " << dst <<", Time: " << Simulator::Now().GetSeconds()<<" s");
    }
    break;  // 只用第一个有效的 socket    
  }
}


/*CH_LIVE*/
void RoutingProtocol::SendClusterHeadBeacon(uint16_t id)
{
  uint16_t NodeId = MyNodeId();
  uint16_t clusterId = m_clusterManager->GetClusterId(NodeId);
  uint16_t realHeadId = m_clusterManager->GetClusterHeadId(clusterId);
  if(NodeId != realHeadId)
  {
    // m_beacon_timer.Cancel();
    NS_LOG_INFO(" not the clusterhead of cluster "<<clusterId);
    m_clusterManager->LeaveCluster(NodeId, clusterId);
    return;
  }
  for(auto j = m_socketAddrs.begin();j!=m_socketAddrs.end();++j){
    Ptr<Socket> socket = j -> first;
    Ipv4InterfaceAddress iface = j -> second;
    Ipv4Address dst;
    bool broad = true;
    if(id < NodeList::GetNNodes()) {
      dst = NodeList::GetNode(id)->GetObject<Ipv4>()->GetAddress(1,0).GetLocal();
      broad = false;
    }
    else{
      if (iface.GetMask() == Ipv4Mask::GetOnes()){
        dst = Ipv4Address ("255.255.255.255");
      }
      else {
        dst = iface.GetBroadcast();
      }
    }
    Ptr<Packet> packet = Create<Packet>();
    Ptr<EnergySourceContainer> sources = GetObject<Node>()->GetObject<EnergySourceContainer>();
    double energy = sources->Get(0)->GetRemainingEnergy();
    Vector pos = GetObject<Node>()->GetObject<MobilityModel>()->GetPosition();
    Vector vel = GetObject<Node>()->GetObject<MobilityModel>()->GetVelocity();
    uint16_t degree = m_neighbors->GetNeighborDegree();
    // GetObject<Node>()->GetObject<ClusterNode>()->UpdateNeighbors(m_neighbors);
    double fit = UpdateNodeFitness();//GetObject<Node>()->GetObject<ClusterNode>()->ComputeFitness(0.25,0.25,0.25,0.25);
    // NS_ASSERT_MSG(fit<=1.0,"invalid fitness");
    // GetObject<Node>()->GetObject<ClusterNode>()->UpdateFitness(fit);
    // double fit = UpdateNodeFitness();
    ClusterHeadBeaconHeader ChLiverHeader(NodeId, m_clusterManager->GetClusterId(NodeId),
                        m_Ipv4->GetAddress(1,0).GetLocal(), pos, vel, degree, energy, fit, Simulator::Now().GetSeconds());
    packet->AddHeader(ChLiverHeader);
    TypeHeader typeHeader(ICRA_CH_LIVE);
    // NS_LOG_DEBUG(" Send CH_LIVE, type is: "<<typeHeader.Get());
    packet->AddHeader(typeHeader);
    socket -> SendTo(packet,0,InetSocketAddress(dst,ICRA_PORT));
    m_headbeaconCount++;
    if(broad){
      NS_LOG_INFO(" Broadcast CH_LIVE at time: "<<Simulator::Now().GetSeconds()<<", clusterId: "<<m_clusterManager->GetClusterId(NodeId)
            <<", HeadId "<<m_clusterManager->GetClusterHeadId(m_clusterManager->GetClusterId(NodeId))); 
    }
    else{
      NS_LOG_INFO(" send CH_LIVE packet to "<< dst <<" at time: "<<Simulator::Now().GetSeconds()<<", clusterId: "<<m_clusterManager->GetClusterId(NodeId)
            <<", HeadId "<<m_clusterManager->GetClusterHeadId(m_clusterManager->GetClusterId(NodeId))); 
    }
    
  }
}


void RoutingProtocol::SendClusterMemberBeacon(Ipv4Address dst){
  Ptr<Node> node = GetObject<Node>();
  uint16_t clusterId = m_clusterManager->GetClusterId(MyNodeId());
  uint16_t ChNodeId = m_clusterManager->GetClusterHeadId(clusterId);  

  Ptr<EnergySourceContainer> sources = node->GetObject<EnergySourceContainer>();
  double remainE = sources->Get(0)->GetRemainingEnergy();
  Vector pos = node->GetObject<MobilityModel>()->GetPosition();
  Vector vel = node->GetObject<MobilityModel>()->GetVelocity();
  node->GetObject<ClusterNode>()->updateResidualEnergy(remainE);
  double fit = UpdateNodeFitness();//node->GetObject<ClusterNode>()->ComputeFitness(0.25,0.25,0.25,0.25);
  NS_ASSERT_MSG(fit<=1.0,"invalid fitness");
  // node->GetObject<ClusterNode>()->UpdateFitness(fit);
  
  Ptr<Packet> packet = Create<Packet>();
  Ipv4Address src = m_Ipv4->GetAddress(1,0).GetLocal();
  ClusterMemberBeaconHeader CmLiveHeader(MyNodeId(),clusterId,src,pos,vel,fit,Simulator::Now().GetSeconds());
  packet->AddHeader(CmLiveHeader);
  TypeHeader typeHeader(ICRA_CM_LIVE);
  packet->AddHeader(typeHeader);
  for(auto k = m_socketAddrs.begin(); k!=m_socketAddrs.end(); ++k){
    auto socket = k->first;
    auto iface = k->second;

    Ipv4Address dest;
    if(iface.GetMask() == Ipv4Mask()){
      dest = Ipv4Address("255.255.255.255");
    }
    else {
      dest = iface.GetBroadcast();
    }
    socket->SendTo(packet, 0, InetSocketAddress(dst, ICRA_PORT));
    m_memberbeaconCount++; 
    NS_LOG_INFO(" Send CM_LIVE to its Cluster Head(ChNode "<< ChNodeId <<"), time: "<<Simulator::Now().GetSeconds()<<" s");
  }

}


int64_t RoutingProtocol::AssignStreams(int64_t stream){
  NS_LOG_FUNCTION(this<<stream);
  m_uniformRandomVariable->SetStream(stream);
  if (m_paperMode) { m_paperOptimizerStream = stream + 1; return 2; }
  return 1;
}


void RoutingProtocol::Start()
{
  // NS_LOG_FUNCTION(this);
  // m_helloTimer.Schedule()
   
}

void RoutingProtocol::NotifyInterfaceUp(uint32_t interface)
{
  // NS_LOG_FUNCTION (this << m_Ipv4->GetAddress (interface, 0).GetLocal ());
  Ptr<Ipv4L3Protocol> l3 = m_Ipv4->GetObject<Ipv4L3Protocol> ();
  if (l3->GetNAddresses (interface) > 1) {
    NS_LOG_WARN ("ICRA does not work with more then one address per each interface.");
  }
  Ipv4InterfaceAddress iface = l3->GetAddress (interface, 0);
  if (iface.GetLocal () == Ipv4Address ("127.0.0.1")) {
    // NS_LOG_DEBUG("Ip is 127.0.0.1!");
    return;
  }
  m_address = iface.GetLocal();
  // Create a socket to listen only on this interface
  Ptr<Socket> socket = Socket::CreateSocket(GetObject<Node>(), UdpSocketFactory::GetTypeId());
  NS_ASSERT(socket!=nullptr);
  socket->SetRecvCallback(MakeCallback(&RoutingProtocol::RecvIcra,this));
  // socket->Bind(InetSocketAddress(Ipv4Address::GetAny(), ICRA_PORT));
  socket->BindToNetDevice(l3->GetNetDevice(interface));
  socket->Bind(InetSocketAddress(Ipv4Address::GetAny(), ICRA_PORT));
  socket->SetAllowBroadcast(true);
  // socket->Connect(m_address);
  socket->SetIpTtl(1); // socket->SetAttribute ("IpTtl", UintegerValue (1)); /*let the broadcast just one hop*/
  m_socketAddrs.insert(std::make_pair(socket,iface));

  // Allow neighbor manager use this interface for layer 2 feedback if possible
  Ptr<NetDevice> dev = m_Ipv4 -> GetNetDevice(m_Ipv4 -> GetInterfaceForAddress(iface.GetLocal ()));  
  /*
  */

  Ptr<WifiNetDevice> wifi = dev->GetObject<WifiNetDevice> ();
  if (wifi == nullptr) {
    // NS_LOG_DEBUG("Wifi device is null");
    return;
  }
  
  Ptr<WifiMac> mac = wifi -> GetMac();
  if (mac == nullptr) {
    // NS_LOG_DEBUG("Mac is null");
    return;
  }

  mac -> TraceConnectWithoutContext("DroppedMpdu", MakeCallback(&RoutingProtocol::NotifyTxError, this));
  if (m_paperMode) {
    m_paperMac = mac;
    m_paperArpCache = l3->GetInterface(interface)->GetArpCache();
    if (m_paperArpCache) {
      m_paperArpCache->TraceConnectWithoutContext("Drop",MakeCallback(&RoutingProtocol::PaperOnArpDrop,this));
    }
  }
}


void RoutingProtocol::NotifyInterfaceDown(uint32_t interface)
{
  // NS_LOG_FUNCTION(this << m_Ipv4 -> GetAddress(interface, 0).GetLocal());
  // Disable layer 2 link state monitoring (if possible)
  Ptr<Ipv4L3Protocol> l3 = m_Ipv4->GetObject<Ipv4L3Protocol>();
  Ptr<NetDevice> dev = l3 -> GetNetDevice(interface);
  Ptr<WifiNetDevice> wifi = dev -> GetObject<WifiNetDevice>();
  if(wifi != nullptr)
  {
    Ptr<WifiMac> mac = wifi -> GetMac() -> GetObject<AdhocWifiMac>();
    if(mac != nullptr)
    {
      mac->TraceConnectWithoutContext("DroppedMpdu", MakeCallback(&ns3::icra::RoutingProtocol::NotifyTxError, this));
    }
  }

  // Close socket
  Ptr<Socket> socket = FindSocketWithInterfaceAddress(m_Ipv4->GetAddress(interface, 0));
  NS_ASSERT(socket);
  socket -> Close();
  m_socketAddrs.erase(socket);

  if(m_socketAddrs.empty())
  {
    NS_LOG_ERROR("No Icra Interfaces");
    m_neighbors->Clear();
    return;
  }

}


void RoutingProtocol::NotifyAddAddress(uint32_t interface, Ipv4InterfaceAddress address)
{
  // NS_LOG_FUNCTION(this <<" interface " << interface << " address " << address);
  Ptr<Ipv4L3Protocol> l3 = m_Ipv4->GetObject<Ipv4L3Protocol>();
  if (!l3 -> IsUp(interface))
  {  
    return;
  }
  if(l3 -> GetNAddresses(interface) == 1)
  {
    Ipv4InterfaceAddress iface = l3->GetAddress(interface, 0);
    Ptr<Socket> socket = FindSocketWithInterfaceAddress(iface);
    if(socket == nullptr)
    {
      if(iface.GetLocal() == Ipv4Address("127.0.0.1"))
      {
        return;
      }
      Ptr<Socket> socket = Socket::CreateSocket(GetObject<Node>(), UdpSocketFactory::GetTypeId());
      NS_ASSERT(socket != nullptr);
      socket->SetRecvCallback(MakeCallback(&RoutingProtocol::RecvIcra, this));
      socket->Bind(InetSocketAddress(Ipv4Address::GetAny(), ICRA_PORT));
      socket->BindToNetDevice(l3->GetNetDevice(interface));
      socket->SetAllowBroadcast(true);
      m_socketAddrs.insert(std::make_pair(socket, iface));

      // Ptr<NetDevice> dev = m_Ipv4 -> GetNetDevice(m_Ipv4 -> GetInterfaceForAddress(iface.GetLocal()));
      /*
      *
      * 
      * */
    }
  }
  else
  {
    NS_LOG_ERROR("Icra does not work with more then one address per each interface. Ignore added address");
  }
}


void RoutingProtocol::NotifyRemoveAddress(uint32_t interface, Ipv4InterfaceAddress address)
{
  // NS_LOG_FUNCTION(this);
  Ptr<Socket> socket = FindSocketWithInterfaceAddress(address);
  if(socket)
  {
    socket->Close();
    m_socketAddrs.erase(socket);
    Ptr<Ipv4L3Protocol> l3 = m_Ipv4->GetObject<Ipv4L3Protocol>();
    if(l3->GetNAddresses(interface))
    {
      Ipv4InterfaceAddress iface = l3->GetAddress (interface, 0);
        // Create a socket to listen only on this interface
      Ptr<Socket> socket = Socket::CreateSocket(GetObject<Node>(), UdpSocketFactory::GetTypeId());
      NS_ASSERT (socket != nullptr);
      socket->SetRecvCallback(MakeCallback(&RoutingProtocol::RecvIcra, this));
      socket->BindToNetDevice(l3->GetNetDevice(interface));
      // Bind to any IP address so that broadcasts can be received
      socket->Bind(InetSocketAddress (Ipv4Address::GetAny(), ICRA_PORT));
      socket->SetAllowBroadcast (true);
      socket->SetIpTtl(1);
      m_socketAddrs.insert(std::make_pair (socket, iface));

      // Add local broadcast record to the routing table
      // Ptr<NetDevice> dev = m_Ipv4->GetNetDevice (m_Ipv4->GetInterfaceForAddress (iface.GetLocal ()));
    }
    if(m_socketAddrs.empty())
    {
      NS_LOG_ERROR("No icra interfaces");
      m_neighbors->Clear();
      m_helloTimer.Cancel();

      return;
    }
  }
  else
  {
    NS_LOG_INFO ("Remove address not participating in icra operation");
  }
}


void RoutingProtocol::NotifyTxError(WifiMacDropReason reason, Ptr<const WifiMpdu> mpdu)
{
  if (m_paperMode) {
    if (reason == WIFI_MAC_DROP_REACHED_RETRY_LIMIT && mpdu) {
      PaperLinkTag tag;
      if (mpdu->GetPacket()->PeekPacketTag(tag) && tag.owner == MyNodeId()) {
        PaperInvalidateNeighbor(tag.next);
      }
    }
    return;
  }
  // NS_LOG_FUNCTION(this);
  m_txErrorCount++;
  m_neighbors->GetTxErrorCallBack()(mpdu->GetHeader());
}


void RoutingProtocol::RecvIcra(Ptr<Socket> socket)
{
  // NS_LOG_FUNCTION(this<<socket);
  Address srcAddr;
  Ptr<Packet> packet = socket -> RecvFrom(srcAddr);
  InetSocketAddress inetSourceAddr = InetSocketAddress::ConvertFrom(srcAddr);
  Ipv4Address sender = inetSourceAddr.GetIpv4();
  Ipv4Address receiver = m_socketAddrs[socket].GetLocal();
  // NS_LOG_DEBUG(" Received a icra packet from "<< sender << " to "<< receiver);  
  TypeHeader typeheader;
  packet->RemoveHeader(typeheader);
  if(!typeheader.IsValid())
  {
    // NS_LOG_DEBUG(" Receive packet " << packet->GetUid() << " with unknown type: " << (uint16_t)typeheader.Get() << " [Ignored]");
    return; // drop the invalid packet
  }
  if (m_paperMode) {
    if (typeheader.Get() == ICRA_PAPER_CONTROL) { PaperReceiveControl(packet,sender); }
    return;
  }
  switch(typeheader.Get()){
    case ICRA_HELLO:    { RecvHello(packet, receiver, sender);        break; }
    case ICRA_CH_CALL:  { RecvHeadAnnounce(packet, receiver, sender); break; }
    case ICRA_JOIN_REQ: { RecvJoinRequest(packet, receiver, sender);  break; }
    case ICRA_JOIN_RESP:{ RecvJoinResponse(packet, receiver, sender); break; }
    case ICRA_CH_LIVE:  { RecvHeadBeacon(packet, receiver, sender);   break; }
    case ICRA_CM_LIVE:  { RecvMemberBeacon(packet, receiver, sender); break; }
    case ICRA_UTILITY:  { RecvUtility(packet, receiver, sender);      break; }
    case ICRA_PAPER_CONTROL:
    case ICRA_GS_REPORT:
    case ICRA_GS_POLICY: break;
  } // end of switch

}


void RoutingProtocol::RecvHello(Ptr<Packet> packet, Ipv4Address myIp, Ipv4Address srcIp)
{
  // NS_LOG_FUNCTION(this);
  if (!m_neighbors) {
    NS_LOG_ERROR("Neighbors object not initialized!");
    return;
  }
  NS_LOG_INFO(" Receive Hello from "<<srcIp);
  Ptr<ClusterNode> clusternode = GetObject<Node>()->GetObject<ClusterNode>();
  HelloHeader header;
  packet -> RemoveHeader(header);  
  auto mobi = GetObject<Node>()->GetObject<MobilityModel>();
  double life = CalculateLHTP(mobi->GetPosition(),mobi->GetVelocity(),header.GetPosition(),header.GetVelocity());
  Neighbors::NeighborInfo newneighbor(  header.GetNodeId(), header.GetIpAddress(), header.GetClusterId(),
                                        header.GetPosition(), header.GetVelocity(), header.GetResidualEnergy(), 
                                        header.GetFitness(), header.GetRole(), Simulator::Now(), Seconds(life));
  m_neighbors->AddNeighbor(newneighbor);
  // UpdateRoutingTable(srcIp, srcIp, 1, ROUTE_DIRECT);
  // ProcessQueuedPackets(srcIp); //检查队列中是否有去往该邻居的数据包
  
}


void RoutingProtocol::RecvHeadAnnounce(Ptr<Packet> packet, Ipv4Address myIp, Ipv4Address srcIp)
{
  /*Handle CH_CALL - 优化版本：添加Stickiness和CH候选收集机制*/
  NS_LOG_INFO(" Receive CH_CALL From "<<srcIp<<", time: "<<Simulator::Now().GetSeconds()<<" s");

  ClusterHeadAnnounceHeader ChHeader;
  packet->RemoveHeader(ChHeader);
  Ipv4Address ChAddr = ChHeader.GetIpAddress();
  uint16_t chNodeId = ChHeader.GetNodeId();
  double chFitness = ChHeader.GetFitnessValue();

  NS_LOG_INFO(" Receive CH_CALL from "<<chNodeId<<", Fitness: "<<chFitness<<", Time: "<<Simulator::Now().GetSeconds()<<" s");

  /* Update Neighbor Cluster Infomation*/
  auto mobi = GetObject<Node>()->GetObject<MobilityModel>();
  double life = CalculateLHTP(mobi->GetPosition(),mobi->GetVelocity(),ChHeader.GetPosition(),ChHeader.GetVelocity());
  Neighbors::NeighborInfo newneighbor(chNodeId, ChAddr, /*Clus-Id*/0xFFFF, ChHeader.GetPosition(),
                                      ChHeader.GetVelocity(), ChHeader.GetResidualEnergy(),
                                      chFitness, BackupHead, Simulator::Now(), Seconds(life));
  m_neighbors->AddNeighbor(newneighbor);

  Ptr<ClusterNode> clusternode = GetObject<Node>()->GetObject<ClusterNode>();
  NodeRole myRole = clusternode->GetRoleType();
  uint16_t myClusterId = clusternode->GetClusterId();

  // ===== STICKINESS机制：如果已经是Member，检查是否值得切换 =====
  if (myRole == Member && myClusterId != 0xFFFF) {
    if (!ShouldSwitchCluster(chNodeId, chFitness)) {
      NS_LOG_DEBUG("ICRA Stickiness: Node " << MyNodeId() << " stays in current cluster " << myClusterId);
      return;  // 保持在当前簇，不切换
    }
    else {
      NS_LOG_INFO("ICRA Stickiness: Node " << MyNodeId() << " switching from cluster " << myClusterId << " to CH " << chNodeId);
      // 离开当前簇，稍后会加入新簇
      Ptr<Cluster> currentCluster = m_clusterManager->GetCluster(myClusterId);
      if (currentCluster) {
        currentCluster->RemoveMember(clusternode);
      }
      clusternode->SetRole(Idle);
      clusternode->SetClusterId(0xFFFF);
    }
  }

  // 记录收到的CH声明
  if (std::find(m_receivedCHs.begin(), m_receivedCHs.end(), chNodeId) == m_receivedCHs.end()) {
    m_receivedCHs.push_back(chNodeId);
    NS_LOG_DEBUG("ICRA: Added CH " << chNodeId << " to CH list, total: " << m_receivedCHs.size());
  }

  // ===== KMeans-style CH候选收集机制 =====
  // 如果选举已完成且当前是Idle状态，收集CH候选而非立即加入
  if (m_icraElectionDone && myRole != BackupHead && myRole != Head && myRole != Member) {
    // 计算距离
    Vector myPos = mobi->GetPosition();
    Vector chPos = ChHeader.GetPosition();
    double dx = myPos.x - chPos.x;
    double dy = myPos.y - chPos.y;
    double dz = myPos.z - chPos.z;
    double distance = std::sqrt(dx*dx + dy*dy + dz*dz);

    // 检查是否已在候选列表中
    bool alreadyInCandidates = false;
    for (auto& candidate : m_chCandidates) {
      if (candidate.nodeId == chNodeId) {
        // 更新候选信息
        candidate.fitness = chFitness;
        candidate.distance = distance;
        alreadyInCandidates = true;
        NS_LOG_DEBUG("ICRA: Updated CH candidate " << chNodeId);
        break;
      }
    }

    if (!alreadyInCandidates) {
      // 添加到候选列表
      ClusterHeadCandidate candidate;
      candidate.nodeId = chNodeId;
      candidate.ipAddress = ChAddr;
      candidate.fitness = chFitness;
      candidate.distance = distance;
      m_chCandidates.push_back(candidate);
      NS_LOG_DEBUG("ICRA: Added CH candidate " << chNodeId << ", total candidates: " << m_chCandidates.size());
    }

    // 如果是第一次收到CH_CALL，调度延迟选择（200ms后从所有候选中选最佳）
    if (!m_hasReceivedChCall) {
      m_hasReceivedChCall = true;
      m_chSelectionTimer.SetFunction(&RoutingProtocol::SelectBestClusterHeadAndJoin, this);
      m_chSelectionTimer.Schedule(MilliSeconds(200));  // 等待200ms收集所有CH_CALL（增加收集时间）
      NS_LOG_INFO("ICRA: Scheduled CH selection in 200ms for node " << MyNodeId());
    }
  }

  return;
}


void RoutingProtocol::CheckPacketStatus(Ptr<Socket>socket, std::string pktpe, Ipv4Address dst){
  // NS_LOG_FUNCTION(this);
  Ptr<Node> node = GetObject<Node>();
  if(pktpe == "CH_CALL"){
    bool NoApplicant = true;
    bool hasBackupNeighbor = false;
    Ipv4Address BackupNeighborAddr;
    auto neighbors = m_neighbors->GetAll();
    for(auto neighbor = neighbors.begin();neighbor!=neighbors.end();++neighbor){
      if(neighbor->second.Role == Member || node->GetObject<ClusterNode>()->GetRoleType()==Member){
        NoApplicant = false;
        return;
      }
      else if(neighbor->second.Role == BackupHead)
      {
        hasBackupNeighbor = true;
        BackupNeighborAddr = neighbor->second.ipAddress;
      }
    }
    if(NoApplicant){
      if(hasBackupNeighbor){
        SendClusterJoinRequest(BackupNeighborAddr);
      }
      else{
        /*Create A lonely Cluster - 优化：添加邻居数量检查（参考BioCluster）*/
        // 统计邻居数量和Idle邻居数量
        uint32_t neighborCount = neighbors.size();
        uint32_t idleNeighborCount = 0;
        for(auto neighbor = neighbors.begin(); neighbor != neighbors.end(); ++neighbor){
          if(neighbor->second.Role == Idle) {
            idleNeighborCount++;
          }
        }

        // BioCluster-style优化：限制簇创建仅在初始阶段（<6s），维护阶段由BeaconTimer和ProactivelyJoinCluster处理
        double currentTime = Simulator::Now().GetSeconds();
        bool isInitialPhase = (currentTime < 6.0);

        if (isInitialPhase) {
          // 初始阶段：允许创建簇，但需要检查邻居条件
          // 只有在满足以下条件之一时才创建孤立簇：
          // 1. 没有任何邻居（真正孤立）
          // 2. 邻居数量>=5 且 Idle邻居>=3（足够形成有意义的簇）
          if(neighborCount == 0 || (neighborCount >= 5 && idleNeighborCount >= 3)) {
            uint16_t newClusterId = m_clusterManager -> ArrangeClusterId();
            m_clusterManager->CreateCluster(newClusterId, node->GetObject<ClusterNode>());
            Simulator::Schedule(Seconds(2.0), &RoutingProtocol::BeaconTimerExpire, this);
            NS_LOG_INFO("ICRA: CH_CALL no applicants (t=" << currentTime << "s), creating cluster-" << newClusterId
                        << " (neighbors=" << neighborCount << ", idle=" << idleNeighborCount << ")");
          }
          else {
            // 邻居不足，保持Idle状态等待
            NS_LOG_DEBUG("ICRA: Not enough neighbors to create cluster (neighbors=" << neighborCount
                         << ", idle=" << idleNeighborCount << "), staying Idle");
            node->GetObject<ClusterNode>()->SetRole(Idle);
          }
        }
        else {
          // 维护阶段(>=6s)：不再通过CH_CALL超时创建簇，保持Idle状态（依赖ProactivelyJoinCluster和TryToJoinCluster）
          NS_LOG_DEBUG("ICRA: Maintenance phase (t=" << currentTime << "s), CH_CALL timeout won't create cluster, staying Idle");
          node->GetObject<ClusterNode>()->SetRole(Idle);
        }
      }
    } // end of if noapplicant
  } // \end of if pktpe == CH_CALL

  if(pktpe=="JOIN_REQ"){
    NodeRole myrole = node->GetObject<ClusterNode>()->GetRoleType();
    if(myrole==Member||myrole==Head) {
      return;// means have already received JOIN_RESP from the ChNode, or has been a clusterHead
    }
    else{
      auto neighbors = m_neighbors->GetAll();
      bool NoOtherBackup = true;
      uint8_t counter = 0;
      for (auto neighbor = neighbors.begin();neighbor!=neighbors.end();++neighbor) {
        if(neighbor->second.ipAddress == dst){
          neighbor->second.UpdateTime = Seconds(0.0);
          NS_LOG_INFO(" JOIN_REQ Time out! From Node "<<MyNodeId()<<" To Node "<<neighbor->first);
          m_neighbors->AddNeighbor(neighbor->second);
          // GetObject<Node>()->GetObject<ClusterNode>()->UpdateNeighbors(m_neighbors);
        }
        if(neighbor->second.Role==BackupHead && neighbor->second.UpdateTime.GetSeconds() == 0.0)
        {
          counter++;
        }
      }
      for(auto neighbor = neighbors.begin(); neighbor!= neighbors.end(); ++neighbor){
        if(neighbor->second.Role == BackupHead && neighbor->second.UpdateTime.GetSeconds() > 0)  {
          NoOtherBackup = false;
          NS_LOG_INFO(" Resend JOIN_REQ to Node "<<neighbor->first<<", time: "<<Simulator::Now().GetSeconds());
          SendClusterJoinRequest(neighbor->second.ipAddress);
          break;
        }
      }
      if(NoOtherBackup || counter > 3)
      {
        /*Create A lonely Cluster - 优化：添加邻居数量检查（参考BioCluster）*/
        uint32_t neighborCount = neighbors.size();
        uint32_t idleNeighborCount = 0;
        for(auto neighbor = neighbors.begin(); neighbor != neighbors.end(); ++neighbor){
          if(neighbor->second.Role == Idle) {
            idleNeighborCount++;
          }
        }

        // BioCluster-style优化：限制簇创建仅在初始阶段（<6s）
        double currentTime = Simulator::Now().GetSeconds();
        bool isInitialPhase = (currentTime < 6.0);

        if (isInitialPhase) {
          // 初始阶段：允许创建簇，但需要检查邻居条件
          // 只有在满足以下条件之一时才创建孤立簇：
          // 1. 没有任何邻居（真正孤立）
          // 2. 邻居数量>=5 且 Idle邻居>=3（足够形成有意义的簇）
          if(neighborCount == 0 || (neighborCount >= 5 && idleNeighborCount >= 3)) {
            uint16_t newClusterId = m_clusterManager -> ArrangeClusterId();
            if(m_clusterManager->ExistMember(MyNodeId()))
            {
              m_clusterManager->LeaveCluster(MyNodeId(), m_clusterManager->GetClusterId(MyNodeId()));
            }
            m_clusterManager->CreateCluster(newClusterId, node->GetObject<ClusterNode>());
            Simulator::Schedule(Seconds(2.0), &RoutingProtocol::BeaconTimerExpire, this);
            NS_LOG_INFO("ICRA: JOIN_REQ failed (t=" << currentTime << "s), creating cluster-" << newClusterId
                        << " (neighbors=" << neighborCount << ", idle=" << idleNeighborCount << ")");
          }
          else {
            // 邻居不足，保持Idle状态
            NS_LOG_DEBUG("ICRA: Not enough neighbors to create cluster (neighbors=" << neighborCount
                         << ", idle=" << idleNeighborCount << "), staying Idle");
            if(m_clusterManager->ExistMember(MyNodeId())) {
              m_clusterManager->LeaveCluster(MyNodeId(), m_clusterManager->GetClusterId(MyNodeId()));
            }
            node->GetObject<ClusterNode>()->SetRole(Idle);
          }
        }
        else {
          // 维护阶段(>=6s)：不再通过JOIN_REQ超时创建簇，保持Idle状态
          NS_LOG_DEBUG("ICRA: Maintenance phase (t=" << currentTime << "s), JOIN_REQ timeout won't create cluster, staying Idle");
          if(m_clusterManager->ExistMember(MyNodeId())) {
            m_clusterManager->LeaveCluster(MyNodeId(), m_clusterManager->GetClusterId(MyNodeId()));
          }
          node->GetObject<ClusterNode>()->SetRole(Idle);
        }
      }
    } /// end of else (the node has not received the JPIN_RESP)
  }   /// end of if : msg tpe == JOIN_REQ

}


void RoutingProtocol::RecvJoinRequest(Ptr<Packet> packet, Ipv4Address myIp, Ipv4Address srcIp)
{
  // NS_LOG_FUNCTION(this);
  JoinClusterRequestHeader CMHeader;
  packet->RemoveHeader(CMHeader);
  uint16_t NodeId = MyNodeId();
  Ptr<ClusterNode> clusternode = GetObject<Node>()->GetObject<ClusterNode>();
  NS_LOG_INFO(" Receive JOIN_REQ from node "<<CMHeader.GetNodeId()<<", The receiver is Member?(1:yes,0:No): "
      << clusternode->IsClusterMember()<<" NowTime: "<<Simulator::Now().GetSeconds());
  /* Update Neighbor Infomation*/
  auto mobi = GetObject<Node>()->GetObject<MobilityModel>();
  double life = CalculateLHTP(mobi->GetPosition(),mobi->GetVelocity(),CMHeader.GetPosition(),CMHeader.GetVelocity());
  Neighbors::NeighborInfo newNeighbor(CMHeader.GetNodeId(), CMHeader.GetIpAddress(),0xFFFF, CMHeader.GetPosition(),
                      CMHeader.GetVelocity(), CMHeader.GetResidualEnergy(), 0.0, Member,Simulator::Now(),Seconds(life));
  m_neighbors->AddNeighbor(newNeighbor);
  // GetObject<Node>()->GetObject<ClusterNode>()->UpdateNeighbors(m_neighbors);
  /* Add New Member to Cluster*/
  if(clusternode->GetRoleType() == Head ||clusternode->GetRoleType() == BackupHead){ // confirm the role
    bool HaveCreateCluster = false , InSomeCluster = false;
    uint16_t MyclusterId = 0xFFFF, TheNodeClusterId;
    if(m_clusterManager->ExistMember(CMHeader.GetNodeId())){
      InSomeCluster = true;
      TheNodeClusterId = m_clusterManager->GetClusterId(CMHeader.GetNodeId());
    }
    if(m_clusterManager->ExistHead(NodeId)){  // judge if the receiver ChNode/backup-CH have create a cluster
      MyclusterId = m_clusterManager->GetClusterId(NodeId);
      HaveCreateCluster = true;
    }
    if(InSomeCluster) { // the sender have join in a cluster, judge if in the receiver's cluster
      if(m_clusterManager->GetClusterHeadId(TheNodeClusterId) != NodeId)  {  // i am not the CH of the cMNode
        if(CMHeader.GetRole()==Member){
          NS_LOG_INFO("The Node "<< CMHeader.GetNodeId()<<" have joined in cluster-" << TheNodeClusterId<<" before. ");
        }
        else{ // means the nodes has not receive JOIN_RESP yet, but already in the cluster
          m_clusterManager->LeaveCluster(CMHeader.GetNodeId(),TheNodeClusterId);
          NS_LOG_INFO("Remove Node "<<CMHeader.GetNodeId()<<" from cluster-"<<TheNodeClusterId<<" , cause the node fail to receive JOIN_RESP.");
          if(m_clusterManager->GetCluster(TheNodeClusterId)->GetAllMembers().size() <= 1){
            m_clusterManager->RemoveCluster(TheNodeClusterId);/* Remove the cluster from clustertable */
            NS_LOG_INFO(" Cluster-"<<TheNodeClusterId<<" have no other members, need to delete.");
          }
          if(HaveCreateCluster){
            NS_LOG_INFO("ChNode "<<NodeId<<" Add Cluster Member: Node-"<<CMHeader.GetNodeId()<<" to cluster-"
                            <<MyclusterId<<", time:"<<Simulator::Now().GetSeconds());
            if(m_clusterManager->GetCluster(MyclusterId)==nullptr)
            {
              return;
            }
            m_clusterManager->JoinCluster(MyclusterId, NodeList::GetNode(CMHeader.GetNodeId())->GetObject<ClusterNode>());
            m_clusterManager->GetCluster(MyclusterId)->UpdateLatestTime(Simulator::Now());
          }
          else{
            // BackupHead收到JOIN_REQ但还没创建簇 - 创建新簇
            uint16_t newClusterId = m_clusterManager->ArrangeClusterId();
            m_clusterManager->CreateCluster(newClusterId, clusternode);
            Simulator::Schedule(Seconds(2.0), &RoutingProtocol::BeaconTimerExpire, this);
            NS_LOG_INFO("RecvJoinRequest: Creating cluster " << newClusterId);
          }
          /*Send JOIN_RESP*/
          Time timer = MilliSeconds(2);
          Simulator::Schedule(timer, &RoutingProtocol::SendClusterJoinResponse, this, CMHeader.GetIpAddress());
        } // end of else, means that the node is in a cluster but not receive join_resp
      }  // end of if Receiver NOT the Head of Sender
    }
    else{ // the node has not join in any cluster
      if(HaveCreateCluster){
        NS_LOG_INFO("ChNode "<<NodeId<<" Add Cluster Member: Node-"<<CMHeader.GetNodeId()<<" to cluster-"
                        <<MyclusterId<<", time:"<<Simulator::Now().GetSeconds());
        m_clusterManager->JoinCluster(MyclusterId, NodeList::GetNode(CMHeader.GetNodeId())->GetObject<ClusterNode>());
        m_clusterManager->GetCluster(MyclusterId)->UpdateLatestTime(Simulator::Now());
      }
      else{
        // BackupHead收到JOIN_REQ但还没创建簇 - 创建新簇
        uint16_t newclusterId = m_clusterManager->ArrangeClusterId();
        m_clusterManager->CreateCluster(newclusterId,clusternode);
        Simulator::Schedule(Seconds(2.0), &RoutingProtocol::BeaconTimerExpire, this);
        NS_LOG_INFO("RecvJoinRequest: Creating cluster " << newclusterId);
      }
      // Send JOIN_RESP
      Time timer = MilliSeconds(2);
      Simulator::Schedule(timer,&RoutingProtocol::SendClusterJoinResponse, this, CMHeader.GetIpAddress());
    }

  } // end of if the receivernode is Head or BackupHead
}

/* Handle JOIN_RESP CH ->>> CM*/
void RoutingProtocol::RecvJoinResponse(Ptr<Packet> packet, Ipv4Address myIp, Ipv4Address srcIp)
{ 
  // NS_LOG_FUNCTION(this);
  Ptr<Node> node = GetObject<Node>();
  Ptr<ClusterNode> clusternode = node->GetObject<ClusterNode>();
  ClusterHeadResponseHeader RespHeader;
  packet->RemoveHeader(RespHeader);
  auto neighbors = m_neighbors->GetAll();
  if(!m_neighbors->IsNeighbor(RespHeader.GetChNodeId()))
  {
    NS_LOG_INFO("JOIN_RESP WARNING! "<<RespHeader.GetChNodeId()<<" -->>> " << MyNodeId()
                      <<" The Sender does not in the neighborTable of the Receiver.");
    return;
  }
  NS_LOG_INFO(" receive JOIN_RESP from node "<<RespHeader.GetChNodeId() <<", Now: "<<Simulator::Now().GetSeconds()<<" s");
  if(clusternode->GetRoleType() == Member){
    uint16_t MyClusterId = clusternode->GetClusterId();
    if(MyClusterId != RespHeader.GetClusterId()){
      NS_LOG_INFO("But Node " << MyNodeId() << " has been a member of another cluster: "
                  << clusternode->GetClusterId() <<", the Sender Ch ClusterId is " << RespHeader.GetClusterId());
      /*Update NeighborInfo*/
      Neighbors::NeighborInfo newNeighbor = neighbors.at(RespHeader.GetChNodeId());
      newNeighbor.Role = Head;
      newNeighbor.clusterId = RespHeader.GetClusterId();
      newNeighbor.UpdateTime = Simulator::Now();
      m_neighbors->AddNeighbor(newNeighbor);
      // GetObject<Node>()->GetObject<ClusterNode>()->UpdateNeighbors(m_neighbors);
    } // end of if:  myClusterId != the senderCH's clusterId
    else{
      NS_LOG_INFO(" is already in the cluster-"<<MyClusterId<<" CH-"<<RespHeader.GetChNodeId());
    }
  }// end of if: role is Member
  else{
    clusternode->SetRole(Member);
    clusternode->SetClusterId(m_clusterManager->GetClusterId(RespHeader.GetChNodeId()));
    for (auto j = neighbors.begin();j!= neighbors.end();++j){
      auto neigh = j->second;
      if(neigh.NodeId == RespHeader.GetChNodeId()){
        neigh.Role = Head;
        neigh.clusterId = RespHeader.GetClusterId();
        neigh.UpdateTime = Simulator::Now();
        m_neighbors->AddNeighbor(neigh);
        return;
      }
    }
    // GetObject<Node>()->GetObject<ClusterNode>()->UpdateNeighbors(m_neighbors);
  }
  // ProcessQueuedPackets(srcIp);
}


void  RoutingProtocol::RecvHeadBeacon(Ptr<Packet> packet, Ipv4Address myIp, Ipv4Address srcIp)
{ // receive CH_LIVE
  ClusterHeadBeaconHeader ChLiveHeader;
  packet->RemoveHeader(ChLiveHeader);
  
  uint16_t ChNodeId = ChLiveHeader.GetChNodeId();
  NS_LOG_INFO(" Receive CH_LIVE from "<<srcIp);
  /*Update neighbor info*/
  // NS_LOG_INFO("Before adding neighbor " << ChNodeId << ", neighbor count: " << m_neighbors->GetAll().size());
  auto neighbors = m_neighbors->GetAll();
  auto cNode = GetObject<Node>()->GetObject<ClusterNode>();
  Ptr<ClusterNode> ChNode  = NodeList::GetNode(ChNodeId)->GetObject<ClusterNode>();
  auto mobi = GetObject<Node>()->GetObject<MobilityModel>();
  double life = CalculateLHTP(mobi->GetPosition(),mobi->GetVelocity(),ChLiveHeader.GetPosition(),ChLiveHeader.GetVelocity());
  Neighbors::NeighborInfo newNeighbor(ChNodeId,ChLiveHeader.GetAddress(),ChLiveHeader.GetClusterId(),
                                    ChLiveHeader.GetPosition(),ChLiveHeader.GetVelocity(),ChLiveHeader.GetResidualEnergy(),
                                    ChLiveHeader.GetFitnessValue(),Head,Simulator::Now(),Seconds(life));
  m_neighbors->AddNeighbor(newNeighbor);
  // NS_LOG_INFO(" Receive CH_LIVE from "<<srcIp<<", NeighborSize: "<< m_neighbors->GetAll().size()
  //                                             <<", test: "<<YXW->GetAll().size());
  // GetObject<Node>()->GetObject<ClusterNode>()->UpdateNeighbors(m_neighbors);
  /*Check Each Neighbor*/
  auto neighborhoods = m_neighbors->GetAll();  
  /* Update Routing info*/
  RouteType RouteTpe = ROUTE_INTRA_CLUSTER;
  if(m_clusterManager->GetClusterId(MyNodeId()) != ChLiveHeader.GetChNodeId()) { RouteTpe = ROUTE_INTER_CLUSTER;}
  UpdateRoutingTable(srcIp, srcIp, 1, RouteTpe);
  if (GetObject<Node>()->GetObject<ClusterNode>()->GetRoleType() == Head)
  {
    uint16_t srcClusterId = ChLiveHeader.GetClusterId();
    if (srcClusterId != m_clusterManager->GetClusterId(MyNodeId()))
    {
      // NS_LOG_DEBUG("Discovered inter-cluster route to cluster " << srcClusterId
      //       <<" by ChId "<<ChLiveHeader.GetChNodeId()<<", myClusterId: "<<m_clusterManager->GetClusterId(MyNodeId()) );   // 记录簇间路由
    }
  }

  for (auto neighbor = neighborhoods.begin(); neighbor!=neighborhoods.end(); ++neighbor){
    if(neighbor->first == ChNodeId){
      if(ChLiveHeader.GetClusterId() == m_clusterManager->GetClusterId(MyNodeId())
                && ChLiveHeader.GetClusterId() <= m_clusterManager->ArrangeClusterId()){ // the Normal clusderHead in the same cluster
        NS_LOG_INFO(" Receive CH_LIVE from its ChNode "<< ChNodeId <<" in Cluster-"<<ChLiveHeader.GetClusterId()
                        <<" ,Time:"<<Simulator::Now().GetSeconds()<<" s.");
        /*Send CM_LIVE to the ChNode As response*/
        double timer = m_uniformRandomVariable->GetValue(0,50);
        Simulator::Schedule(MilliSeconds(timer),&RoutingProtocol::SendClusterMemberBeacon,this, srcIp);
        NS_LOG_INFO(" Send CM_LIVE to "<<srcIp);
      }
      else{ // receive CH_LIVE from another cluster.
        if(cNode->GetRoleType() == Head){
          double MyFit = UpdateNodeFitness();//cNode->ComputeFitness(0.25,0.25,0.25,0.25);
          NS_ASSERT_MSG(MyFit<=1.0,"invalid fitness");
          // cNode->UpdateFitness(MyFit);
          uint16_t clusterScale = 0;
          uint16_t clusterId = m_clusterManager->GetClusterId(MyNodeId());
          if(clusterId < m_clusterManager->ArrangeClusterId()){
            clusterScale = m_clusterManager->GetCluster(clusterId)->GetAllMembers().size();
          }
          if(clusterScale <= 1 && MyFit < ChLiveHeader.GetFitnessValue() 
                  && MyNodeId() == m_clusterManager->GetClusterHeadId(clusterId)){

            m_clusterManager->LeaveCluster(MyNodeId(),clusterId);
            m_clusterManager->RemoveCluster(clusterId);
            m_beacon_timer.Cancel();
            NS_LOG_INFO("Cluster "<<clusterId<<" ready to delete");
            cNode->SetRole(Idle);
            // cNode->SetClusterId(0xFFFF);
          }
        }
        if(cNode->GetRoleType() == Member)
        {
          uint16_t recClusterId = cNode->GetClusterId();
          if(! m_clusterManager->IsInCluster(MyNodeId(), recClusterId)) // the member node does not realize has been delete from
          {
            cNode->SetRole(Isolate);
          }
        }
        if(cNode->GetRoleType() == Idle || cNode->GetRoleType() == Isolate){
          // if(cNode->GetFitnessValue() > ChLiveHeader.GetFitnessValue()) { continue; }
          double timer = m_uniformRandomVariable->GetValue(0,20);
          Simulator::Schedule(MilliSeconds(timer),&RoutingProtocol::SendClusterJoinRequest, this, srcIp);
        }
        if(cNode->GetRoleType() == Member && !m_clusterManager->ExistMember(cNode->GetNodeId()))
        {
          for(auto neighbor = neighborhoods.begin();neighbor!=neighborhoods.end();++neighbor)
          {
            if(neighbor->second.NodeId == m_clusterManager->GetClusterHeadId(cNode->GetClusterId()))
            {
              double timelength = Simulator::Now().GetSeconds() - neighbor->second.UpdateTime.GetSeconds();
              if(timelength > 5.0)
              cNode->SetRole(Idle);
              cNode->SetClusterId(0xFFFF);
              SendClusterJoinRequest(srcIp);
            }
          }
        }
      }// end of [Else] the sender is in another cluster
      break;
    } // end of [IF] find the sender CH in neighbor    
  }  // end of [For] neighbor in neighbors

  // ProcessQueuedPackets(srcIp);
}


void  RoutingProtocol::RecvMemberBeacon(Ptr<Packet> packet, Ipv4Address myIp, Ipv4Address srcIp)
{
  ClusterMemberBeaconHeader CmLiveHeader;
  packet->RemoveHeader(CmLiveHeader);
  
  /*Update Neighbor infomation*/
  auto mobi = GetObject<Node>()->GetObject<MobilityModel>();
  double life = CalculateLHTP(mobi->GetPosition(),mobi->GetVelocity(),CmLiveHeader.GetPosition(),CmLiveHeader.GetVelocity());
  Neighbors::NeighborInfo newNeighbor(CmLiveHeader.GetNodeId(),CmLiveHeader.GetAddress(),CmLiveHeader.GetClusterId(),
                CmLiveHeader.GetPosition(),CmLiveHeader.GetVelocity(), 0,
                CmLiveHeader.GetFitnessValue(), Member, Simulator::Now(),Seconds(life));
  m_neighbors->AddNeighbor(newNeighbor);
  // GetObject<Node>()->GetObject<ClusterNode>()->UpdateNeighbors(m_neighbors);
  auto neighbors = m_neighbors->GetAll();
  
  /* Update Routing info*/
  UpdateRoutingTable(srcIp, srcIp, 1, ROUTE_INTRA_CLUSTER);

  if(GetObject<Node>()->GetObject<ClusterNode>()->GetRoleType() != Head) {return;}
  
  for ( auto neighbor= neighbors.begin();neighbor!=neighbors.end();++neighbor){
    if(neighbor->second.NodeId == CmLiveHeader.GetNodeId()){      
      if(CmLiveHeader.GetClusterId() == m_clusterManager->GetClusterId(MyNodeId()) ){        
        NS_LOG_INFO(" Receive CM_LIVE from Member Node "<< CmLiveHeader.GetNodeId()
                <<" in Cluster-"<<CmLiveHeader.GetClusterId()<<" ,Time:"<<Simulator::Now().GetSeconds()<<" s.");
        if(!m_clusterManager->ExistMember(CmLiveHeader.GetNodeId())){
          NS_LOG_DEBUG("But Node "<<CmLiveHeader.GetNodeId()<< " does not in the cluster-"
                  << CmLiveHeader.GetClusterId()<<" yet.");
          continue;
        }
        if(!m_clusterManager->GetCluster(CmLiveHeader.GetClusterId()))
        {
          NS_LOG_DEBUG("Received CM_LIVE from Node "<<CmLiveHeader.GetNodeId()<<", But Cluster Id "
                  << CmLiveHeader.GetClusterId()<<" does not exist yet.");
          continue;
        }
        m_clusterManager->GetCluster(CmLiveHeader.GetClusterId())->UpdateMemberTime(CmLiveHeader.GetNodeId(), Simulator::Now());
      }
    }
  }

}



Vector RoutingProtocol::GetMyPosition()
{ 
  Ptr<MobilityModel> mobility = m_Ipv4->GetObject<Node>()->GetObject<MobilityModel>();
  NS_ASSERT(mobility);
  return mobility->GetPosition();
}


double RoutingProtocol::GetControlThroughput()
{
  if (m_paperMode) { return static_cast<double>(m_paperControlBytes + m_gsControlBytes) / 1000.0; }
  double thp = 0.0;
  HelloHeader hello;
  ClusterHeadAnnounceHeader chCall;
  JoinClusterRequestHeader joinReq;
  ClusterHeadResponseHeader joinResp;
  ClusterHeadBeaconHeader chLive;
  ClusterMemberBeaconHeader cmLive;
  UtilityHeader utility;
  uint64_t result = GetHelloCount()        * hello.GetSerializedSize() 
                  + GetChCallCount()       * chCall.GetSerializedSize()
                  + GetJoinReqCount()      * joinReq.GetSerializedSize()
                  + GetJoinReqCount()      * joinResp.GetSerializedSize()
                  + GetMemberBeaconCount() * cmLive.GetSerializedSize()
                  + GetHeadBeaconCount()   * chLive.GetSerializedSize()
                  + GetUtilityCount()      * utility.GetSerializedSize();
  return thp = (double)result / 1000;
}







// Get the Node Position by IP Addresss
Vector RoutingProtocol::GetPositionFromIp(Ipv4Address addr)
{
  if (m_paperMode) {
    for (const auto& item : m_paperNeighbors) {
      if (item.second.address == addr) { return PaperNeighborPosition(item.second); }
    }
    // Explicit ideal destination-location service, independent of cluster state.
    for (uint32_t i = 0; i < NodeList::GetNNodes(); ++i) {
      auto node = NodeList::GetNode(i);
      auto ipv4 = node->GetObject<Ipv4>();
      if (!ipv4) { continue; }
      for (uint32_t iface = 1; iface < ipv4->GetNInterfaces(); ++iface) {
        if (ipv4->GetNAddresses(iface) && ipv4->GetAddress(iface,0).GetLocal() == addr) {
          auto mobility = node->GetObject<MobilityModel>();
          return mobility ? mobility->GetPosition() : Vector();
        }
      }
    }
    return Vector();
  }
  auto neighbors = m_neighbors->GetAll();   // 先在邻居表中查找
  for (auto &n : neighbors) {
    if (n.second.ipAddress == addr) {
      return n.second.Position;
    }
  }
  RoutingTableEntry entry;  // 在路由表中查找
  if (m_routingTable.LookupRoute(addr, entry)) {
    return entry.GetDestPosition();
  }
  // 尝试从节点列表获取
  for (uint32_t i = 0; i < NodeList::GetNNodes(); i++) {
    Ptr<Ipv4> ipv4 = NodeList::GetNode(i)->GetObject<Ipv4>();
    if (ipv4 && ipv4->GetNAddresses(1) > 0)
    {
      if (ipv4->GetAddress(1, 0).GetLocal() == addr)
      {
        Ptr<MobilityModel> mob = ipv4->GetObject<Node>()->GetObject<MobilityModel>();
        return mob->GetPosition();
      }
    }
  }
  
  return Vector(0, 0, 0);
}


// Get the NodeId by IP address
uint16_t RoutingProtocol::GetNodeIdFromIp(Ipv4Address addr)
{
  for (uint32_t i = 0; i < NodeList::GetNNodes(); i++) {
    Ptr<Ipv4> ipv4 = NodeList::GetNode(i)->GetObject<Ipv4>();
    if (ipv4 && ipv4->GetNAddresses(1) > 0) {
      if (ipv4->GetAddress(1, 0).GetLocal() == addr) {
        return i;
      }
    }
  }
  return 0xFFFF;
}


double RoutingProtocol::UpdateNodeFitness()
{
  if (m_paperMode) {
    double value = ComputeFitnessICRA();
    if (auto c = GetObject<Node>()->GetObject<ClusterNode>()) { c->UpdateFitness(value); }
    return value;
  }
  Ptr<ClusterNode> clusternode = GetObject<Node>()->GetObject<ClusterNode>();
  // 仅 legacy 保留历史固定等权行为；论文的在线调权由上面的 paper 分支实现。
  m_weights.energyWeight   = 0.25;
  m_weights.degreeWeight   = 0.25;
  m_weights.velocityWeight = 0.25;
  m_weights.distanceWeight = 0.25;
  CollectNodeStatusForQLearning();   // legacy 收集状态，但保留历史路由等权行为

  // ICRA 专用适应值：Energy/Degree/LHTP/速度相似性(s³)，不再复用 ClusterNode::ComputeFitness。
  // Unlike distance-based clustering, the fourth factor uses the velocity
  // similarity variance from Eq. 9-12 of the ICRA paper, not positional distance.
  double fitness = ComputeFitnessICRA();
  // if(fitness>1)
  // {
  //   std::cout<<"Node "<<clusternode->GetNodeId()<<"Fitness out of range: "<<fitness<<std::endl;
  //   std::cout << "VelWeigh =  "<<m_weights.velocityWeight
  //             << ", EnerWeigh = "<< m_weights.energyWeight
  //             << ", DegreeWeigh = "<<m_weights.degreeWeight
  //             << ", DisWeigh = "<<m_weights.distanceWeight<<std::endl;
  // }
  // NS_ASSERT_MSG(fitness<1,"Node "<<clusternode->GetNodeId()<<" fitness="<<fitness<<" out of range! Now: "<<Simulator::Now().GetSeconds());
  clusternode->UpdateFitness(fitness);

  return fitness;
}


/**
 * \brief ICRA 论文专用的簇头适应值计算
 *
 * Unlike the shared ClusterNode::ComputeFitness path, the four ICRA utility factors are
 * Energy、Degree、LHTP、以及"速度相似性"。前三个因子与 ClusterNode 一致；第四个因子
 * 用论文 Eq.9~12 的速度相似性方差 s³（而非 ClusterNode 里的位置相似性 Distance）。
 * paper 分支采用 GS 下发的在线权重；legacy 分支保留固定 0.25。
 *
 *   Eq.9  d_ij = sqrt((v_i - v_j)^2 + (θ_i - θ_j)^2)   速度(速率,方向角)的欧氏距离
 *   Eq.10 V_ij = 1 / (1 + d_ij)                        节点 i、j 的速度相似性
 *   Eq.11 V̄_s(i) = (1/n) Σ_{j∈N} V_ij                 与邻居的平均速度相似性
 *   Eq.12 s³_i  = (1/n) Σ_{j∈N} (V_ij - V̄)^2          速度相似性方差(第三效用因子)
 *
 * 其中 v 取速度向量的 2D 速率 |（vx,vy）|，θ 取 atan2(vy,vx) 的弧度，方向角差归一到 [-π,π]。
 */
double RoutingProtocol::ComputeFitnessICRA()
{
  if (m_paperMode) {
    PaperSyncLocalState();
    if (m_paperUtilityFrozen) { return m_paperRoundUtility; }
    auto factors = QLearningWeightOptimizer::GetUtilityFactors(GetObject<Node>(),m_maxRange,50.0);
    return m_weights.energyWeight * factors[0] + m_weights.degreeWeight * factors[1]
      + m_weights.velocityWeight * factors[2] + m_weights.distanceWeight * factors[3];
  }
  // 单次仿真内固定均等权重
  const double W_ENERGY = 0.25;
  const double W_DEGREE = 0.25;
  const double W_LHTP   = 0.25;
  const double W_VELSIM = 0.25;

  const double MAX_LHTP_VALUE = 200.0;   // 仿真时长上限(秒)
  const double REF_LHTP_TIME  = 50.0;    // LHTP 归一化参考值

  Ptr<ClusterNode> cnode = GetObject<Node>()->GetObject<ClusterNode>();
  if (!cnode) { return 0.0; }

  if (Simulator::Now().GetSeconds() > 1.0)
  {
    cnode->UpdateNeighborDegree();
  }

  Ptr<Neighbors> nb = cnode->GetNeighborTable();
  uint32_t n = (nb ? nb->GetAll().size() : 0);

  // 无邻居或无能量 -> 不具备当簇头的资格
  if (n == 0 || cnode->GetResidualEnergy() <= 0.0)
  {
    return 0.0;
  }

  Vector myPos = cnode->GetPosition();
  Vector myVel = cnode->GetVelocity();

  // ===== 因子1：Energy（剩余能量比）=====
  double energy_factor = cnode->GetResidualEnergy() / cnode->GetOriginEnergy();
  energy_factor = std::min(1.0, std::max(0.0, energy_factor));

  // ===== 因子2：Degree（邻居度归一）=====
  double degree_factor = static_cast<double>(n) /
                         std::max(1, cnode->GetNetworkScale() - 1);
  degree_factor = std::min(1.0, std::max(0.0, degree_factor));

  // ===== 因子3：LHTP（与邻居的平均链路保持时间，越长越稳）=====
  double lhtpSum = 0.0;
  for (const auto & entry : nb->GetAll())
  {
    const auto & info = entry.second;
    if (info.NodeId == cnode->GetNodeId()) { continue; }
    double lhtp = CalculateLHTP(myPos, myVel, info.Position, info.Velocity);
    if (std::isinf(lhtp)) { lhtp = MAX_LHTP_VALUE; }   // 永不断链 -> 视为最大保持时间
    lhtpSum += lhtp;
  }
  double avgLHTP = lhtpSum / n;
  double lhtp_factor = (avgLHTP >= MAX_LHTP_VALUE * 0.99)
                       ? 1.0
                       : std::min(1.0, avgLHTP / REF_LHTP_TIME);

  // ===== 因子4：速度相似性 s³（论文 Eq.9~12）=====
  double myV     = std::sqrt(myVel.x * myVel.x + myVel.y * myVel.y);  // 速率 |v|
  double myTheta = std::atan2(myVel.y, myVel.x);                      // 方向角(弧度)

  std::vector<double> vsim;
  vsim.reserve(n);
  double vsimSum = 0.0;
  for (const auto & entry : nb->GetAll())
  {
    const auto & info = entry.second;
    if (info.NodeId == cnode->GetNodeId()) { continue; }
    double nv     = std::sqrt(info.Velocity.x * info.Velocity.x +
                              info.Velocity.y * info.Velocity.y);
    double ntheta = std::atan2(info.Velocity.y, info.Velocity.x);
    double dv     = myV - nv;
    double dth    = myTheta - ntheta;
    // 方向角差归一到 [-π, π]，避免 ±π 分支跳变把"几乎同向"误判为"完全反向"
    while (dth >  M_PI) { dth -= 2.0 * M_PI; }
    while (dth < -M_PI) { dth += 2.0 * M_PI; }
    double d_ij = std::sqrt(dv * dv + dth * dth);   // Eq.9
    double V_ij = 1.0 / (1.0 + d_ij);               // Eq.10
    vsim.push_back(V_ij);
    vsimSum += V_ij;
  }

  double velsim_factor = 0.0;
  if (!vsim.empty())
  {
    double Vbar = vsimSum / vsim.size();            // Eq.11 平均相似性
    double var  = 0.0;
    for (double v : vsim) { var += (v - Vbar) * (v - Vbar); }
    velsim_factor = var / vsim.size();              // Eq.12 方差 s³
  }
  // s³ 是 V_ij∈(0,1] 的方差，天然落在 [0,0.25] 区间，量纲与其它[0,1]因子不同，按论文原样代入
  velsim_factor = std::min(1.0, std::max(0.0, velsim_factor));

  // ===== 组合（等权）=====
  double result = W_ENERGY * energy_factor
                + W_DEGREE * degree_factor
                + W_LHTP   * lhtp_factor
                + W_VELSIM * velsim_factor;
  result = std::min(1.0, std::max(0.0, result));

  NS_LOG_DEBUG("ICRA NodeId: " << cnode->GetNodeId() << "\tFitness = " << result
               << " (Energy:" << energy_factor << " Degree:" << degree_factor
               << " LHTP:" << lhtp_factor << " VelSim(s3):" << velsim_factor
               << ") at " << Simulator::Now().GetSeconds() << "s");

  return result;
}


void RoutingProtocol::DoInitialize()
{
  if (m_paperMode) { PaperInitialize(); Ipv4RoutingProtocol::DoInitialize(); return; }
  // NS_LOG_FUNCTION(this);
  Ptr<ClusterNode> cNode = GetObject<Node>()->GetObject<ClusterNode>();
  m_neighbors = cNode->GetObject<Neighbors>();
  if (!m_neighbors)
  {
    // 如果还没有 Neighbors，创建一个
    Ptr<Neighbors> neighbor = CreateObject<Neighbors>(cNode->GetNodeId());
    cNode->AggregateObject(neighbor);
    m_neighbors = neighbor;
  }
  NS_ASSERT_MSG(m_neighbors,"neighbor table is null");
 

  uint32_t Jitter = m_uniformRandomVariable -> GetInteger(0, 500);
  m_helloTimer.SetFunction(&RoutingProtocol::HelloTimerExpire, this);
  m_clusteringTimer.SetFunction(&RoutingProtocol::ClusteringTimerExpire, this);
  m_beacon_timer.SetFunction(&RoutingProtocol::BeaconTimerExpire,this);
  m_clusteringStatusTimer.SetFunction(&RoutingProtocol::ClusteringPerformanceTimerExpire,this);
  // uint32_t StartTime = m_uniformRandomVariable -> GetInteger(0, 100); // random initial time to send hello
  // Time ClusterBeaconStartTime = MilliSeconds(StartTime);
  
  if (MyNodeId() == 0) {
    m_isQLearningMaster = true;
    m_qlearningEpisodeTimer.SetFunction(&RoutingProtocol::QLearningEpisodeExpire, this);
    // 在t=10s启动第一个episode（等网络稳定）
    m_qlearningEpisodeTimer.Schedule(Seconds(10.0));
    NS_LOG_INFO("Q-Learning episode timer initialized on master node (Node 0)");
    NS_LOG_INFO("First episode will start at t=10s, interval=" << m_episodeInterval.GetSeconds() << "s");
  }

  m_helloTimer.Schedule(MilliSeconds(Jitter)); // after 0s~0.5s, get neighborinfo by hello packet
  // NS_LOG_DEBUG(" Starting after "<< Jitter <<" ms");
  m_clusteringTimer.Schedule(m_ClusteringTime); // at 1.0s, start CH election and get result
  m_clusteringStatusTimer.Schedule(Seconds(9.9));

  // 初始化统计
  m_totalPacketsSent = 0;
  m_totalPacketsReceived = 0;
  m_totalPacketsDropped = 0;
  Simulator::Schedule(Seconds(10.0), &RoutingProtocol::CleanupPacketStats, this);  // 定期清理统计数据

  Ipv4RoutingProtocol::DoInitialize();

}

void RoutingProtocol::QLearningEpisodeExpire()
{
  NS_LOG_FUNCTION(this);  
  if (!m_isQLearningMaster) {
    NS_LOG_WARN("QLearning Episode Expire called on non-master node");
    return;
  }  
  static bool firstEpisode = true;  
  // 步骤1: 收集所有节点状态（上帝视角）
  NS_LOG_DEBUG("Collecting status from all " << NodeList::GetNNodes() << " nodes");
  
  for (uint32_t i = 0; i < NodeList::GetNNodes(); i++) {
    Ptr<Node> node = NodeList::GetNode(i);
    Ptr<ClusterNode> clusterNode = node->GetObject<ClusterNode>();
    
    if (clusterNode) {
      QLearningWeightOptimizer::NodeStatus status;
      status.nodeId = i;
      status.energy = clusterNode->GetResidualEnergy();
      
      Vector vel = clusterNode->GetVelocity();
      status.speed = std::sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z);
      
      status.degree = clusterNode->GetNeighborDegree();
      status.position = clusterNode->GetPosition();
      status.role = clusterNode->GetRoleType();
      status.lastRoleChange = Simulator::Now();
      
      m_qLearningOptimizer->CollectNodeStatus(i, status);
    }
  }  
  // 步骤2: Episode控制
  if (firstEpisode) {
    NS_LOG_INFO("Starting first Q-Learning episode at t=" << Simulator::Now().GetSeconds() << "s");
    m_qLearningOptimizer->StartEpisode();
    firstEpisode = false;
  } else {
    NS_LOG_DEBUG("Ending current episode and starting next");
    m_qLearningOptimizer->EndEpisode();
    m_qLearningOptimizer->StartEpisode();
  }  
  // 步骤3: 调度下一次Episode
  // 检查是否接近仿真结束
  Time nextEpisodeTime = Simulator::Now() + m_episodeInterval;
  Time simulationEndTime = Simulator::GetMaximumSimulationTime();
  
  if (nextEpisodeTime < simulationEndTime) {
    m_qlearningEpisodeTimer.Schedule(m_episodeInterval);
    NS_LOG_DEBUG("Next episode scheduled at t=" << nextEpisodeTime.GetSeconds() << "s");
  } else {
    NS_LOG_INFO("No more episodes scheduled (approaching simulation end)");
  }
}


// uint16_t RoutingProtocol::CountLostNeighbors()
// {
//   m_currentNeighbors.clear();
//   auto neighbors = m_neighbors->GetAll();

//   for (const auto& n : neighbors) {
//     m_currentNeighbors.insert(n.first);
//   }
  
//   std::set<uint16_t> lostNeighbors;
//   std::set_difference(m_previousNeighbors.begin(), m_previousNeighbors.end(),
//                       m_currentNeighbors.begin(), m_currentNeighbors.end(),
//                       std::inserter(lostNeighbors, lostNeighbors.begin()));
  
//   return lostNeighbors.size();
// }

// uint16_t RoutingProtocol::CountNewNeighbors()
// {
//   if (m_currentNeighbors.empty()) {
//     auto neighbors = m_neighbors->GetAll();
//     for (const auto& n : neighbors) {
//       m_currentNeighbors.insert(n.first);
//     }
//   }  
//   std::set<uint16_t> newNeighbors;
//   std::set_difference(m_currentNeighbors.begin(), m_currentNeighbors.end(),
//                       m_previousNeighbors.begin(), m_previousNeighbors.end(),
//                       std::inserter(newNeighbors, newNeighbors.begin()));  
//   m_previousNeighbors = m_currentNeighbors;
//   return newNeighbors.size();
// }


// double RoutingProtocol::CalculatePDR()
// {
//   Time windowSize = Seconds(10.0);
//   Time now = Simulator::Now();  
//   uint32_t recentSent = 0;
//   uint32_t recentReceived = 0;
  
//   for (const auto& stat : m_packetStats) {
//     if (now - stat.second.lastUpdate <= windowSize) {
//       recentSent += stat.second.sent;
//       recentReceived += stat.second.received;
//     }
//   }
  
//   if (recentSent == 0) {
//     return m_totalPacketsSent > 0 ? (double)m_totalPacketsReceived / m_totalPacketsSent : 1.0;
//   }
  
//   return std::min(1.0, (double)recentReceived / recentSent);
// }


double RoutingProtocol::CalculateLHTP(Vector aPos, Vector aVel, Vector bPos, Vector bVel)
{
  if (m_paperMode) {
    return QLearningWeightOptimizer::LinkHoldingTime(aPos,aVel,bPos,bVel,m_maxRange);
  }
  double range = GetMaxRange();
  Vector dPos = bPos - aPos;    // 相对位置和相对速度
  Vector dVel = bVel - aVel;
  auto dot = [](Vector u, Vector v) {
    return u.x * v.x + u.y * v.y + u.z * v.z;    // 向量点积工具
  };
  double d0 = dot(dPos, dPos);  // 初始距离平方
  double r2 = range * range;

  if (d0 > r2) { return 0.0; }       // 已经超出通信范围  

  double a = dot(dVel, dVel);
  double b = 2 * dot(dPos, dVel);
  double c = d0 - r2;
  if (a == 0.0) { return std::numeric_limits<double>::infinity(); }   // 相对静止 
  
  double discriminant = b*b - 4*a*c;    // 解二次方程 a t^2 + b t + c = 0
  if (discriminant < 0) {  return std::numeric_limits<double>::infinity(); } // 永远不会超出 

  double sqrtD = std::sqrt(discriminant);
  double t1 = (-b - sqrtD) / (2*a);
  double t2 = (-b + sqrtD) / (2*a);

  double result = std::numeric_limits<double>::infinity();    // 大于 0 的最小根
  if (t1 > 0) result = t1;
  if (t2 > 0 && t2 < result) result = t2;

  if (result == std::numeric_limits<double>::infinity()) 
    return 0.0; // 不会断，或者已经在边界情况
  
  
  return result;
}




void RoutingProtocol::CleanupPacketStats()
{
  Time maxAge = Seconds(30.0);
  Time now = Simulator::Now();  
  auto it = m_packetStats.begin();
  while (it != m_packetStats.end()) {
    if (now - it->second.lastUpdate > maxAge) {
      it = m_packetStats.erase(it);
    } else {
      ++it;
    }
  }  
  Simulator::Schedule(Seconds(10.0), &RoutingProtocol::CleanupPacketStats, this);
}


/*Qlearning Weight Funcitons*/
void RoutingProtocol::CollectNodeStatusForQLearning()
{
  NS_LOG_FUNCTION(this);
  
  Ptr<ClusterNode> clusternode = GetObject<Node>()->GetObject<ClusterNode>();
  Ptr<MobilityModel> mobility = GetObject<Node>()->GetObject<MobilityModel>();
  Ptr<EnergySourceContainer> sources = GetObject<Node>()->GetObject<EnergySourceContainer>();
  
  QLearningWeightOptimizer::NodeStatus status;
  status.nodeId = MyNodeId();
  status.energy = sources->Get(0)->GetRemainingEnergy();
  status.speed = std::sqrt(std::pow(mobility->GetVelocity().x, 2) + 
                          std::pow(mobility->GetVelocity().y, 2) + 
                          std::pow(mobility->GetVelocity().z, 2));
  status.degree = m_neighbors->GetAll().size();
  status.position = mobility->GetPosition();
  status.role = clusternode->GetRoleType();
  status.lastRoleChange = m_lastRoleChangeTime;  
  // 检测角色变化
  if (m_previousRole != status.role) {
    m_lastRoleChangeCount++;
    m_lastRoleChangeTime = Simulator::Now();
    m_previousRole = status.role;
    NS_LOG_DEBUG("Node " << MyNodeId() << " role changed to " << status.role);
  }
  
  m_qLearningOptimizer->CollectNodeStatus(MyNodeId(), status);
}


void RoutingProtocol::CollectFeedbackForQLearning()
{
  NS_LOG_FUNCTION(this);  
  Ptr<ClusterNode> clusternode = GetObject<Node>()->GetObject<ClusterNode>();
  Ptr<EnergySourceContainer> sources = GetObject<Node>()->GetObject<EnergySourceContainer>();  
  QLearningWeightOptimizer::Feedback feedback;
  feedback.nodeId = MyNodeId();
  feedback.roleChanges = m_lastRoleChangeCount;
  feedback.timestamp = Simulator::Now();
  // 计算能量变化
  double currentEnergy = sources->Get(0)->GetRemainingEnergy();
  double initialEnergy = sources->Get(0)->GetInitialEnergy();
  feedback.energyChange = (initialEnergy - currentEnergy) / initialEnergy;
  
  m_qLearningOptimizer->CollectFeedback(MyNodeId(), feedback);
}

void RoutingProtocol::Configure(bool paper, double range, Time epoch, double sigma,
                                uint32_t phi, Time gsDelay, uint16_t excludedNodeId)
{
  NS_ABORT_MSG_IF(range <= 0 || epoch <= Seconds(0) || sigma < 0 || gsDelay < Seconds(0),
                  "Invalid ICRA configuration");
  // A paper round has three one-HELLO collection windows plus a bounded JOIN
  // budget. The period must be long enough to contain the round; otherwise a
  // policy update would always arrive in the middle of the previous round.
  NS_ABORT_MSG_IF(paper && epoch <= Seconds(1.2),
                  "Paper mode requires a strategy period longer than one round");
  m_paperMode = paper;
  m_maxRange = range;
  m_episodeInterval = epoch;
  m_groundControlDelay = gsDelay;
  m_lhtpThreshold = sigma;
  m_excludedNodeId = excludedNodeId;
  m_qLearningOptimizer->SetLegacyMode(!paper);
  m_qLearningOptimizer->SetMetricParameters(range,50.0);
  m_qLearningOptimizer->SetEvaluationPeriod(epoch);
  m_qLearningOptimizer->SetRoleChangeThreshold(phi);
}

void RoutingProtocol::PaperSyncLocalState()
{
  Ptr<Node> node = GetObject<Node>();
  if (!node) { return; }
  auto c = node->GetObject<ClusterNode>();
  if (!c) { return; }
  if (auto mobility = node->GetObject<MobilityModel>()) {
    c->SetPosition(mobility->GetPosition());
    c->SetVelocity(mobility->GetVelocity());
  }
  auto sources = node->GetObject<EnergySourceContainer>();
  if (sources && sources->GetN() > 0) {
    c->SetOriginEnergy(sources->Get(0)->GetInitialEnergy());
    c->updateResidualEnergy(sources->Get(0)->GetRemainingEnergy());
  }
  c->SetMaxRange(m_maxRange);
  if (m_neighbors) { c->SetNeighborDegree(m_neighbors->GetAll().size()); }
}

void RoutingProtocol::PaperInitialize()
{
  auto node = GetObject<Node>();
  auto c = node->GetObject<ClusterNode>();
  if (node->GetId() == m_excludedNodeId) { m_participates = false; }
  NS_ASSERT_MSG(c,"ICRA requires ClusterNode storage");
  if (m_excludedNodeId != 0xffff && c->GetNetworkScale() > 1) {
    c->SetNetworkScale(c->GetNetworkScale()-1);
  }
  m_neighbors = c->GetObject<Neighbors>();
  if (!m_neighbors) {
    m_neighbors = CreateObject<Neighbors>(c->GetNodeId());
    c->AggregateObject(m_neighbors);
  }
  PaperSyncLocalState();
  c->SetRole(m_participates ? Idle : Isolate);
  c->SetClusterId(0xffff);
  m_paperStateEpoch = m_paperEpoch;
  m_totalPacketsSent = m_totalPacketsReceived = m_totalPacketsDropped = 0;
  m_paperProtocols[MyNodeId()] = this;
  if (m_participates && !m_paperController) {
    m_paperController = this;
    m_isQLearningMaster = true;
    m_qLearningOptimizer->Reset(true);
    m_qLearningOptimizer->SetLegacyMode(false);
    if (m_paperOptimizerStream >= 0) { m_qLearningOptimizer->AssignStreams(m_paperOptimizerStream); }
    m_qlearningEpisodeTimer.SetFunction(&RoutingProtocol::PaperGroundControlTick,this);
    m_qlearningEpisodeTimer.Schedule(m_episodeInterval + m_groundControlDelay*2 + NanoSeconds(1));
  }
  m_helloTimer.SetFunction(&RoutingProtocol::HelloTimerExpire,this);
  m_clusteringTimer.SetFunction(&RoutingProtocol::ClusteringTimerExpire,this);
  // Paper mode spreads the initial HELLO phase over one full period. A 0--20 ms
  // spread keeps all nodes transmitting in the same contention window, which
  // destroys broadcast utility/declaration reception before any clustering
  // rule can be applied. This is a transmission-phase choice, not a rule change.
  m_helloTimer.Schedule(MilliSeconds(
      m_uniformRandomVariable->GetInteger(0, m_paperMode ? 250 : 20)));
  if (m_participates) {
    m_clusteringTimer.Schedule(Seconds(0.3));
    m_paperReportEvent = Simulator::Schedule(m_episodeInterval,
                                            &RoutingProtocol::PaperSendReport,this);
  }
  m_paperArpProtocol = node->GetObject<ArpL3Protocol>();
  if (m_paperArpProtocol) {
    m_paperArpProtocol->TraceConnectWithoutContext("Drop",MakeCallback(&RoutingProtocol::PaperOnArpDrop,this));
  }
}

Vector RoutingProtocol::PaperNeighborPosition(const PaperNeighbor& neighbor) const
{
  double age = std::max(0.0,(Simulator::Now()-neighbor.updated).GetSeconds());
  return Vector(neighbor.position.x + neighbor.velocity.x*age,
                neighbor.position.y + neighbor.velocity.y*age,
                neighbor.position.z + neighbor.velocity.z*age);
}

bool RoutingProtocol::PaperNeighborUsable(const PaperNeighbor& neighbor) const
{
  // Epoch equality is a packet-admission and candidacy condition, not a routing
  // condition. After a policy update the previous topology stays usable until
  // the new round commits or the observation ages out.
  if (Simulator::Now()-neighbor.updated > m_paperNeighborTimeout) {
    return false;
  }
  auto node = GetObject<Node>();
  auto mobility = node ? node->GetObject<MobilityModel>() : nullptr;
  return mobility && CalculateDistance(mobility->GetPosition(),PaperNeighborPosition(neighbor)) <= m_maxRange;
}

void RoutingProtocol::PaperPruneNeighbors()
{
  for (auto it = m_paperNeighbors.begin(); it != m_paperNeighbors.end();) {
    if (!PaperNeighborUsable(it->second)) {
      uint16_t id = it->first;
      if (m_paperMembers.count(id)) { PaperRemoveMemberMirror(id); }
      m_paperMembers.erase(id); m_paperPendingMembers.erase(id); m_paperForwarders.erase(id);
      if (m_neighbors) { m_neighbors->DeleteNeighbor(id); }
      it = m_paperNeighbors.erase(it);
      m_routingTable.Clear();
    } else { ++it; }
  }
  for (auto it = m_paperPendingMembers.begin(); it != m_paperPendingMembers.end();) {
    if (!m_paperNeighbors.count(it->first) || Simulator::Now()-it->second > m_paperNeighborTimeout) {
      it = m_paperPendingMembers.erase(it);
    } else { ++it; }
  }
  for (auto it = m_paperForwarders.begin(); it != m_paperForwarders.end();) {
    if (!m_paperMembers.count(it->first) || Simulator::Now()-it->second > m_paperNeighborTimeout) {
      it = m_paperForwarders.erase(it);
    } else { ++it; }
  }
}

void RoutingProtocol::PaperInvalidateNeighbor(uint16_t id)
{
  if (id == 0xffff) { return; }
  if (m_paperMembers.count(id)) { PaperRemoveMemberMirror(id); }
  m_paperNeighbors.erase(id); m_paperMembers.erase(id); m_paperPendingMembers.erase(id); m_paperForwarders.erase(id);
  if (m_neighbors) { m_neighbors->DeleteNeighbor(id); }
  m_routingTable.Clear();
  if (m_paperHeadId == id && id != MyNodeId() && !m_paperClustering) {
    if (!m_paperRejoinOnly || !m_paperRecoveryEvent.IsRunning()) {
      m_paperRecoveryEvent = Simulator::ScheduleNow(&RoutingProtocol::PaperRecoverHead,this);
    }
  }
}

void RoutingProtocol::PaperRemoveMemberMirror(uint16_t id)
{
  auto cluster = m_clusterManager->GetCluster(m_paperClusterId);
  if (!cluster) { return; }
  auto& members = cluster->GetAllMembers();
  auto found = members.find(id);
  if (found == members.end()) { return; }
  if (found->second->GetClusterId() == m_paperClusterId) { cluster->RemoveMember(found->second); }
  else { members.erase(found); }
}

void RoutingProtocol::PaperOnArpDrop(Ptr<const Packet> packet)
{
  if (!m_paperMode || !packet || !m_Ipv4) { return; }
  PaperLinkTag tag;
  if (packet->PeekPacketTag(tag) && tag.owner == MyNodeId()) { PaperInvalidateNeighbor(tag.next); }
}

void RoutingProtocol::PaperMaintenance()
{
  if (!m_Ipv4) { return; }
  PaperSyncLocalState();
  PaperPruneNeighbors();
  if (m_participates && m_paperHeadId != 0xffff && m_paperHeadId != MyNodeId() && !m_paperClustering) {
    auto head = m_paperNeighbors.find(m_paperHeadId);
    if (head == m_paperNeighbors.end() || head->second.role != Head || head->second.headId != m_paperHeadId) {
      PaperRecoverHead();
    }
  }
  PaperUpdateBackbone();
  UpdateNodeFitness();
  PaperSendControl(PaperControlHeader::Kind::HELLO);
  // Safety net in case a scheduled round event was lost while a client pending
  // policy waited: apply it as soon as the node is outside a round.
  if (!m_paperClustering && m_hasPendingPolicy) { PaperApplyPolicy(); }
  // Independent per-period jitter avoids permanently colliding Hello phases
  // after two nodes draw the same initial millisecond (including hidden peers).
  m_helloTimer.Schedule(m_paperHelloInterval +
                        MilliSeconds(static_cast<int32_t>(m_uniformRandomVariable->GetInteger(0,20))-10));
}

void RoutingProtocol::PaperLeaveCluster()
{
  auto c = GetObject<Node>()->GetObject<ClusterNode>();
  // This shared object is only a metrics mirror; no remote state drives decisions.
  auto cluster = m_clusterManager->GetCluster(m_paperClusterId);
  if (cluster) {
    if (m_paperHeadId == MyNodeId()) {
      // Drop stale registrations without touching a member's new association.
      // Leave current members in place so RemoveCluster records dissolution
      // size and lifetime before it clears them.
      auto members = cluster->GetAllMembers();
      for (const auto& item : members) {
        auto peer = m_paperProtocols.find(item.first);
        const bool stillMine = peer == m_paperProtocols.end()
          || (peer->second->m_paperHeadId == MyNodeId() && peer->second->m_paperClusterId == m_paperClusterId);
        if (!stillMine || item.second->GetClusterId() != m_paperClusterId) {
          cluster->RemoveMemberRecord(item.first);
        }
      }
      m_clusterManager->RemoveCluster(m_paperClusterId);
    }
    else if (cluster->ExistMember(MyNodeId())) { cluster->RemoveMember(c); }
  }
  m_paperHeadId = m_paperClusterId = 0xffff;
  m_paperForwarder = m_ifnRequested = false;
  m_paperMembers.clear(); m_paperPendingMembers.clear(); m_paperForwarders.clear();
  c->SetRole(m_participates ? Idle : Isolate); c->SetClusterId(0xffff);
  m_paperStateEpoch = m_paperEpoch;
  m_routingTable.Clear();
}

Time RoutingProtocol::PaperCollectionWindow() const
{
  // One complete HELLO period plus the maintenance jitter. Waiting a full
  // period is the smallest bound that guarantees every neighbor has had a
  // scheduled transmission opportunity, independent of its phase.
  return m_paperHelloInterval + MilliSeconds(20);
}

Time RoutingProtocol::PaperRoundBudget() const
{
  // Keep the whole round inside one strategy period. The paper reports cluster
  // creation below 0.5 s; in ns-3 the JOIN must additionally pass real ARP, so
  // a bounded 1.7 s round is the engineering completion of an unspecified
  // delivery budget, not a change to the clustering rule.
  Time limit = m_episodeInterval - m_groundControlDelay*2 - MilliSeconds(20);
  return std::min(limit,Seconds(1.7));
}

void RoutingProtocol::PaperStartClustering()
{
  PaperStartRound(false);
}

void RoutingProtocol::PaperStartRound(bool recovery)
{
  if (!m_Ipv4 || !m_participates || m_paperClustering) { return; }
  ++m_paperAuditCounters.formationStarts;
  if (recovery) { ++m_paperAuditCounters.rejoinStarts; }
  m_paperRecoveryEvent.Cancel();
  m_paperUtilityEvent.Cancel(); m_paperElectionEvent.Cancel(); m_paperSelectionEvent.Cancel();
  m_paperJoinEvent.Cancel(); m_paperFormationEvent.Cancel();
  if (recovery) {
    // III-C.5.2 says the member exits the cluster before Step 3. A full round
    // instead keeps the committed topology serving data until it commits.
    PaperLeaveCluster();
  }
  m_paperClustering = true;
  m_paperRoundKind = recovery ? PaperRoundKind::Recovery : PaperRoundKind::Full;
  m_paperRoundHead = 0xffff; m_paperRoundCluster = 0xffff;
  m_pendingJoinHead = 0xffff; m_pendingJoinAttempts = 0;
  m_pendingJoinAddress = Ipv4Address(); m_pendingJoinTransaction = 0;
  m_triedPaperHeads.clear();
  m_paperUtilityFrozen = false;
  const Time now = Simulator::Now();
  const Time window = PaperCollectionWindow();
  m_paperRoundStart = now;
  m_paperUtilityWindowStart = recovery ? now : now + window;
  m_paperRoundDeadline = now + (recovery ? std::min(PaperRoundBudget(),Seconds(1.2))
                                         : PaperRoundBudget());
  m_paperFormationDeadline = m_paperRoundDeadline;
  if (recovery) {
    // Step 3 freezes the utility already in use: no utility broadcast and no
    // fresh local election, only reselection of a CH from received declarations.
    if (!m_paperUtilityPublished) { m_paperRoundUtility = UpdateNodeFitness(); }
    m_paperUtilityPublished = true;
    m_paperUtilityFrozen = true;
  } else {
    m_paperUtilityPublished = false;
    UpdateNodeFitness();
  }
  PaperSendControl(PaperControlHeader::Kind::HELLO);
  if (recovery) {
    m_paperSelectionEvent = Simulator::Schedule(window + MilliSeconds(
      m_uniformRandomVariable->GetInteger(0,40)),&RoutingProtocol::PaperSelectHead,this);
  } else {
    const Time spread = std::min(MilliSeconds(100),window/2);
    m_paperUtilityEvent = Simulator::Schedule(window + MilliSeconds(
      m_uniformRandomVariable->GetInteger(0,spread.GetMilliSeconds())),
      &RoutingProtocol::PaperPublishUtility,this);
    m_paperElectionEvent = Simulator::Schedule(window + window,
      &RoutingProtocol::PaperElection,this);
    // Stagger JOIN initiators over a bounded window: simultaneous cold-ARP
    // requests to a handful of heads collide, and hidden senders cannot hear
    // each other's collision. Hashing the node id over 20 slots spreads them
    // deterministically without changing which CH they select.
    const uint32_t slot = (static_cast<uint32_t>(MyNodeId()) * 2654435761u) % 20;
    Time selection = window + window + window + MilliSeconds(
      slot * 25 + m_uniformRandomVariable->GetInteger(0,15));
    if (selection >= m_paperRoundDeadline) { selection = m_paperRoundDeadline - MilliSeconds(20); }
    m_paperSelectionEvent = Simulator::Schedule(selection,
      &RoutingProtocol::PaperSelectHead,this);
  }
  m_paperFormationEvent = Simulator::Schedule(m_paperRoundDeadline - now,
    &RoutingProtocol::PaperFinishFormation,this);
}

void RoutingProtocol::PaperRecoverHead()
{
  if (!m_paperRejoinOnly) { PaperStartClustering(); return; }
  if (!m_Ipv4 || !m_participates || m_paperClustering) { return; }
  m_paperRecoveryEvent.Cancel();
  PaperStartRound(true);
}

void RoutingProtocol::PaperPublishUtility()
{
  if (!m_paperClustering || m_paperRoundKind != PaperRoundKind::Full) { return; }
  m_paperRoundUtility = UpdateNodeFitness();
  // All decisions in this formation compare the value that this node actually
  // published, rather than a newer, energy-depleted value against old neighbors.
  m_paperUtilityFrozen = true;
  m_paperUtilityPublished = true;
  PaperSendControl(PaperControlHeader::Kind::UTILITY);
}

void RoutingProtocol::PaperElection()
{
  if (!m_paperClustering || m_paperRoundKind != PaperRoundKind::Full
      || m_paperRoundHead != 0xffff) { return; }
  if (m_paperAllCh) { PaperDeclareHead(PaperHeadReason::AllCh); return; }
  PaperPruneNeighbors();
  double mine = m_paperUtilityFrozen ? m_paperRoundUtility : UpdateNodeFitness();
  NS_LOG_DEBUG("t=" << Simulator::Now().GetSeconds() << " paper election utility=" << mine << " neighbors=" << m_paperNeighbors.size());
  for (const auto& item : m_paperNeighbors) {
    const auto& n = item.second;
    if (n.role == Isolate || n.epoch != m_paperEpoch) { continue; }
    // W_table_i only contains utility received in this round. A stale value
    // from before the strategy update must not suppress a legitimate CH.
    if (n.updated < m_paperUtilityWindowStart) { continue; }
    if (n.utility > mine + 1e-12) { return; }
  }
  PaperDeclareHead(PaperHeadReason::Election);
}

void RoutingProtocol::PaperSelectHead()
{
  if (!m_paperClustering || m_paperRoundHead != 0xffff) { return; }
  PaperPruneNeighbors();
  double mine = m_paperUtilityFrozen ? m_paperRoundUtility : UpdateNodeFitness();
  auto mobility = GetObject<Node>()->GetObject<MobilityModel>();
  std::vector<PaperNeighbor> heads;
  for (const auto& item : m_paperNeighbors) {
    const auto& n = item.second;
    if (n.role != Head || n.headId != n.nodeId || m_triedPaperHeads.count(n.nodeId)) { continue; }
    // Only a head that belongs to this strategy epoch was declared after the
    // update. The still-serving old head is deliberately not a candidate.
    if (n.epoch != m_paperEpoch || n.stateEpoch != m_paperEpoch) { continue; }
    if (n.updated < m_paperRoundStart) { continue; }
    // Algorithm 1 line 8 says Wi > Wk while III-C.2 elects the maximum-utility
    // node. The literal inequality would stop the lower-utility node from
    // joining the only elected (higher-utility) CH. Keep the self-consistent
    // interpretation used by this implementation and state it explicitly.
    bool higher = n.utility + 1e-12 >= mine;
    double lifetime = CalculateLHTP(mobility->GetPosition(),mobility->GetVelocity(),
                                     PaperNeighborPosition(n),n.velocity);
    if (higher && lifetime > m_lhtpThreshold) { heads.push_back(n); }
  }
  std::sort(heads.begin(),heads.end(),[](const PaperNeighbor& a,const PaperNeighbor& b) {
    if (a.utility != b.utility) { return a.utility > b.utility; }
    return a.nodeId < b.nodeId;
  });
  const Time normalWait = MilliSeconds(m_uniformRandomVariable->GetInteger(100,140));
  if (!heads.empty() && Simulator::Now() + normalWait < m_paperRoundDeadline) {
    m_pendingJoinHead = heads.front().nodeId;
    m_pendingJoinAddress = heads.front().address; m_pendingJoinAttempts = 1;
    ++m_paperJoinTransaction;
    if (m_paperJoinTransaction == 0) { ++m_paperJoinTransaction; }
    m_pendingJoinTransaction = m_paperJoinTransaction;
    m_triedPaperHeads.insert(m_pendingJoinHead);
    NS_LOG_DEBUG("t=" << Simulator::Now().GetSeconds() << " paper JOIN target=" << m_pendingJoinHead << " mine=" << mine
                 << " targetUtility=" << heads.front().utility
                 << " transaction=" << m_pendingJoinTransaction);
    PaperSendControl(PaperControlHeader::Kind::JOIN,heads.front().address,m_pendingJoinHead);
    Time wait = normalWait;
    bool unresolved = true;
    if (m_paperArpCache) {
      auto entry = m_paperArpCache->Lookup(heads.front().address);
      unresolved = !entry || entry->IsWaitReply() || entry->IsDead();
    }
    if (m_paperArpAwareJoin || unresolved) {
      // Permit one real ARP recovery opportunity. The wait is capped by the
      // round deadline, so the whole process still fits the policy period.
      Time arpWait = m_paperArpCache ? m_paperArpCache->GetWaitReplyTimeout() : Seconds(1);
      wait = std::max(wait,arpWait+MilliSeconds(140));
    }
    const Time remaining = m_paperRoundDeadline - Simulator::Now() - MilliSeconds(20);
    if (wait > remaining) { wait = remaining; }
    NS_LOG_DEBUG("t=" << Simulator::Now().GetSeconds() << " paper JOIN wait="
                 << wait.GetSeconds() << " unresolved=" << unresolved
                 << " deadline=" << m_paperRoundDeadline.GetSeconds());
    if (wait <= Time(0)) {
      m_pendingJoinHead = 0xffff; m_pendingJoinAttempts = 0;
      m_pendingJoinAddress = Ipv4Address();
      PaperDeclareHead(m_triedPaperHeads.empty() ? PaperHeadReason::NoCandidate
                                                : PaperHeadReason::JoinFailure);
      return;
    }
    m_paperJoinEvent = Simulator::Schedule(wait,
                                          &RoutingProtocol::PaperJoinTimeout,this);
  } else {
    PaperDeclareHead(m_triedPaperHeads.empty() ? PaperHeadReason::NoCandidate
                                              : PaperHeadReason::JoinFailure);
  }
}

void RoutingProtocol::PaperJoinTimeout()
{
  if (!m_paperClustering || m_paperRoundHead != 0xffff || m_pendingJoinHead == 0xffff) { return; }
  ++m_paperAuditCounters.joinTimeouts;
  NS_LOG_DEBUG("t=" << Simulator::Now().GetSeconds() << " paper JOIN timeout target=" << m_pendingJoinHead);
  // The paper leaves reliable request/accept delivery unspecified. Bounded
  // retransmission tolerates a lost control frame before trying the next
  // utility-ranked candidate; it never supplies a free acceptance.
  if (m_pendingJoinAttempts < 3
      && Simulator::Now() + MilliSeconds(140) < m_paperRoundDeadline) {
    ++m_pendingJoinAttempts;
    PaperSendControl(PaperControlHeader::Kind::JOIN,m_pendingJoinAddress,m_pendingJoinHead);
    m_paperJoinEvent = Simulator::Schedule(MilliSeconds(m_uniformRandomVariable->GetInteger(100,140)),
                                          &RoutingProtocol::PaperJoinTimeout,this);
    return;
  }
  m_pendingJoinHead = 0xffff;
  m_pendingJoinAttempts = 0; m_pendingJoinAddress = Ipv4Address();
  m_pendingJoinTransaction = 0;
  PaperSelectHead();
}

void RoutingProtocol::PaperDeclareHead(PaperHeadReason reason)
{
  if (!m_participates || !m_paperClustering) { return; }
  m_paperRoundHead = MyNodeId();
  m_paperRoundCluster = MyNodeId();
  PaperBecomeHead(reason);
  // The declaration is final for this node; it can serve JOINs immediately.
  PaperFinishFormation();
}

void RoutingProtocol::PaperBecomeHead(PaperHeadReason reason)
{
  if (!m_participates) { return; }
  switch (reason) {
    case PaperHeadReason::Election: ++m_paperAuditCounters.headElection; break;
    case PaperHeadReason::NoCandidate: ++m_paperAuditCounters.headNoCandidate; break;
    case PaperHeadReason::JoinFailure: ++m_paperAuditCounters.headJoinFailure; break;
    case PaperHeadReason::Deadline: ++m_paperAuditCounters.headDeadline; break;
    case PaperHeadReason::AllCh: ++m_paperAuditCounters.headAllCh; break;
  }
  NS_LOG_DEBUG("t=" << Simulator::Now().GetSeconds() << " paper becomes CH");
  auto c = GetObject<Node>()->GetObject<ClusterNode>();
  if (m_paperHeadId != MyNodeId()) {
    PaperLeaveCluster();
    m_paperHeadId = MyNodeId();
    // Node identity provides a stable, local cluster identifier; not a global lookup.
    m_paperClusterId = MyNodeId();
  }
  m_paperStateEpoch = m_paperEpoch;
  c->SetRole(Head); c->SetClusterId(m_paperClusterId);
  if (!m_clusterManager->GetCluster(m_paperClusterId)) {
    m_clusterManager->CreateCluster(m_paperClusterId,c);
  }
  PaperSendControl(PaperControlHeader::Kind::CALL);
  PaperSendControl(PaperControlHeader::Kind::HELLO);
  PaperCommitSemanticRole();
}

void RoutingProtocol::PaperCommitAsMember(uint16_t head, uint16_t cluster)
{
  if (!m_participates || head == 0xffff || head == MyNodeId()) { return; }
  auto c = GetObject<Node>()->GetObject<ClusterNode>();
  if (m_paperHeadId == head && m_paperClusterId == cluster) {
    // Same association: a new epoch only needs a fresh declaration. Rebuilding
    // the mirror here would create a periodic data-plane gap without changing
    // any clustering decision.
    m_paperStateEpoch = m_paperEpoch;
    auto target = m_clusterManager->GetCluster(cluster);
    if (target && !target->ExistMember(MyNodeId())) {
      m_clusterManager->JoinCluster(cluster,c);
    }
    PaperSendControl(PaperControlHeader::Kind::HELLO);
    PaperUpdateBackbone();
    PaperCommitSemanticRole();
    return;
  }
  const bool wasHead = m_paperHeadId == MyNodeId();
  PaperLeaveCluster();
  m_paperHeadId = head; m_paperClusterId = cluster;
  m_paperStateEpoch = m_paperEpoch;
  c->SetRole(Member); c->SetClusterId(cluster);
  auto target = m_clusterManager->GetCluster(cluster);
  // Use the manager's bookkeeping path so historical peak size follows the
  // mirror's membership, including clusters that shrink before dissolution.
  if (target) { m_clusterManager->JoinCluster(cluster,c); }
  m_routingTable.Clear();
  if (wasHead) { ++m_paperAuditCounters.headDemoted; }
  PaperSendControl(PaperControlHeader::Kind::HELLO);
  PaperUpdateBackbone();
  PaperCommitSemanticRole();
}

void RoutingProtocol::PaperFinishFormation()
{
  if (!m_paperClustering) { return; }
  m_paperJoinEvent.Cancel();
  m_pendingJoinHead = 0xffff; m_pendingJoinAttempts = 0; m_pendingJoinAddress = Ipv4Address();
  m_pendingJoinTransaction = 0;
  if (m_paperRoundHead == 0xffff) {
    // Algorithm 1 lines 17--19: no joined cluster leaves the node as a CH.
    m_paperRoundHead = MyNodeId(); m_paperRoundCluster = MyNodeId();
    PaperBecomeHead(PaperHeadReason::Deadline);
  } else if (m_paperRoundHead != MyNodeId() && m_paperHeadId != m_paperRoundHead) {
    PaperCommitAsMember(m_paperRoundHead,m_paperRoundCluster);
  }
  m_paperClustering = false;
  m_paperUtilityFrozen = false;
  m_paperRoundKind = PaperRoundKind::None;
  m_paperRoundHead = 0xffff; m_paperRoundCluster = 0xffff;
  PaperUpdateBackbone();
  PaperCommitSemanticRole();
  PaperFlushQueue();
  if (m_hasPendingPolicy) { PaperApplyPolicy(); }
}

void RoutingProtocol::PaperCommitSemanticRole()
{
  if (m_paperClustering || !m_participates || m_paperHeadId == 0xffff) { return; }
  uint8_t role = m_paperHeadId == MyNodeId() ? 1 : (m_paperForwarder ? 2 : 3);
  if (m_semanticRole && role != m_semanticRole) {
    ++m_lastRoleChangeCount;
    m_lastRoleChangeTime = Simulator::Now();
  }
  m_semanticRole = role; m_semanticHead = m_paperHeadId;
}

void RoutingProtocol::PaperUpdateBackbone()
{
  if (!m_participates || m_paperHeadId == 0xffff || m_paperHeadId == MyNodeId()) { return; }
  auto head = m_paperNeighbors.find(m_paperHeadId);
  if (head == m_paperNeighbors.end() || !PaperNeighborUsable(head->second) || head->second.role != Head) {
    m_paperForwarder = false;
    return;
  }
  bool foreign = false;
  for (const auto& item : m_paperNeighbors) {
    const auto& n = item.second;
    // The excluded benchmark BS is a data endpoint, not a cluster participant.
    // A neighboring CM can therefore ask its CH for IFN status to reach it.
    bool externalEndpoint = m_excludedNodeId != 0xffff && n.nodeId == m_excludedNodeId;
    if (PaperNeighborUsable(n) && (externalEndpoint ||
        (n.headId != 0xffff && n.headId != m_paperHeadId && n.role != Isolate))) {
      foreign = true; break;
    }
  }
  if (!foreign) {
    if (m_ifnRequested || m_paperForwarder) {
      PaperSendControl(PaperControlHeader::Kind::IFN_REQUEST,head->second.address,m_paperHeadId,false);
    }
    m_ifnRequested = m_paperForwarder = false;
    PaperCommitSemanticRole();
    return;
  }
  if (m_paperForwarder && Simulator::Now()-m_lastIfnAck > m_paperNeighborTimeout) {
    m_paperForwarder = false; PaperCommitSemanticRole();
  }
  if (!m_ifnRequested || Simulator::Now()-m_lastIfnRequest >= m_paperHelloInterval) {
    m_ifnRequested = true; m_lastIfnRequest = Simulator::Now();
    PaperSendControl(PaperControlHeader::Kind::IFN_REQUEST,head->second.address,m_paperHeadId,true);
  }
}

void RoutingProtocol::PaperSendControl(PaperControlHeader::Kind kind, Ipv4Address destination,
                                      uint16_t targetHead, bool forwarder, uint16_t transaction)
{
  if (!m_Ipv4 || m_socketAddrs.empty()) { return; }
  PaperSyncLocalState();
  auto c = GetObject<Node>()->GetObject<ClusterNode>();
  PaperControlHeader header;
  header.kind = kind; header.nodeId = MyNodeId(); header.epoch = m_paperEpoch;
  header.headId = targetHead == 0xffff ? m_paperHeadId : targetHead;
  header.clusterId = m_paperClusterId; header.role = c->GetRoleType();
  header.stateEpoch = m_paperStateEpoch;
  header.transaction = kind == PaperControlHeader::Kind::ACCEPT ? transaction
    : (kind == PaperControlHeader::Kind::JOIN ? m_pendingJoinTransaction : 0);
  header.forwarder = (kind == PaperControlHeader::Kind::IFN_REQUEST || kind == PaperControlHeader::Kind::IFN_ACK)
    ? forwarder : m_paperForwarder;
  header.position = c->GetPosition(); header.velocity = c->GetVelocity();
  header.originEnergy = std::max(0.0,c->GetOriginEnergy());
  header.residualEnergy = std::max(0.0,c->GetResidualEnergy());
  // III-C.2/3 compares the published W_i in W_table/CH_table. A CH can
  // finish before its prospective members; later HELLOs must repeat that
  // declaration's value instead of replacing it with a different-time score.
  // Motion/energy above and GS report factors remain live observations.
  header.utility = m_paperUtilityPublished ? m_paperRoundUtility : c->GetFitnessValue();
  header.timestamp = Simulator::Now().GetSeconds();
  if (!header.IsValid()) { return; }
  if (kind == PaperControlHeader::Kind::JOIN) { ++m_paperAuditCounters.joinAttempts; }
  for (const auto& item : m_socketAddrs) {
    auto packet = Create<Packet>(); packet->AddHeader(header);
    packet->AddHeader(TypeHeader(ICRA_PAPER_CONTROL));
    packet->AddPacketTag(PaperControlTag());
    ++m_paperControlPackets; m_paperControlBytes += packet->GetSize();
    switch (kind) {
      case PaperControlHeader::Kind::HELLO: ++m_helloPacketCount; break;
      case PaperControlHeader::Kind::CALL: ++m_chCallPacketCount; break;
      case PaperControlHeader::Kind::JOIN: ++m_joinReqPacketCount; break;
      case PaperControlHeader::Kind::ACCEPT: ++m_joinRespPacketCount; break;
      case PaperControlHeader::Kind::UTILITY: ++m_utilityPacketCount; break;
      default: break;
    }
    Ipv4Address target = destination.IsBroadcast() ? item.second.GetBroadcast() : destination;
    item.first->SendTo(packet,0,InetSocketAddress(target,ICRA_PORT));
  }
}

void RoutingProtocol::PaperReceiveControl(Ptr<Packet> packet, Ipv4Address sender)
{
  PaperControlHeader h;
  if (packet->GetSize() < h.GetSerializedSize() || !packet->RemoveHeader(h) || !h.IsValid()
      || (h.epoch != m_paperEpoch && m_participates && h.role != Isolate) || h.nodeId == MyNodeId()
      || h.timestamp > Simulator::Now().GetSeconds()+1e-6) { return; }
  PaperNeighbor n;
  n.nodeId = h.nodeId; n.headId = h.headId; n.clusterId = h.clusterId;
  n.epoch = h.epoch; n.stateEpoch = h.stateEpoch;
  n.role = h.role; n.forwarder = h.forwarder;
  n.position = h.position; n.velocity = h.velocity; n.utility = h.utility;
  n.originEnergy = h.originEnergy; n.energy = h.residualEnergy;
  n.updated = Seconds(h.timestamp); n.address = sender;
  if (!PaperNeighborUsable(n)) { return; }
  auto previous = m_paperNeighbors.find(h.nodeId);
  if (previous != m_paperNeighbors.end() && previous->second.updated > n.updated) { return; }
  if (h.kind == PaperControlHeader::Kind::IFN_REQUEST) {
    // The request flag expresses the desired state; only an ensuing Hello
    // advertises that the member has actually received its IFN acceptance.
    n.forwarder = previous != m_paperNeighbors.end() && previous->second.forwarder;
  }
  m_paperNeighbors[h.nodeId] = n;
  if (m_neighbors && h.role != Isolate) {
    m_neighbors->Purge();
    Neighbors::NeighborInfo shared(n.nodeId,n.address,n.clusterId,PaperNeighborPosition(n),n.velocity,
                                   n.energy,n.utility,n.role,Simulator::Now(),m_paperNeighborTimeout);
    m_neighbors->AddNeighbor(shared);
  }
  // A fresh, physically received control packet permits normal ARP discovery again.
  if (m_paperArpCache) {
    auto entry = m_paperArpCache->Lookup(sender);
    if (entry && entry->IsDead()) { m_paperArpCache->Remove(entry); }
  }
  auto c = GetObject<Node>()->GetObject<ClusterNode>();
  const bool confirmedHead = m_paperHeadId == MyNodeId() && m_paperStateEpoch == m_paperEpoch;
  if (confirmedHead && h.headId == MyNodeId() && h.role == Member
      && h.clusterId == m_paperClusterId) {
    // A Member announcement in the current epoch closes the exchange: it proves
    // the member received ACCEPT. JOIN alone never adds a confirmed member.
    m_paperPendingMembers.erase(h.nodeId);
    m_paperMembers[h.nodeId] = Simulator::Now();
    auto cluster = m_clusterManager->GetCluster(m_paperClusterId);
    if (cluster) {
      if (!cluster->ExistMember(h.nodeId)) {
        // The local member map is authoritative; this only restores the shared
        // metrics mirror when the cluster object was recreated by re-clustering.
        auto remote = m_paperProtocols.find(h.nodeId);
        if (remote != m_paperProtocols.end() && remote->second->GetObject<Node>()) {
          auto mc = remote->second->GetObject<Node>()->GetObject<ClusterNode>();
          if (mc) { m_clusterManager->JoinCluster(m_paperClusterId,mc); }
        }
      } else {
        cluster->UpdateMemberTime(h.nodeId,Simulator::Now()); cluster->UpdateLatestTime(Simulator::Now());
      }
    }
  } else if (confirmedHead && m_paperMembers.count(h.nodeId)
             && (h.kind == PaperControlHeader::Kind::HELLO || h.kind == PaperControlHeader::Kind::CALL)) {
    PaperRemoveMemberMirror(h.nodeId);
    m_paperMembers.erase(h.nodeId); m_paperForwarders.erase(h.nodeId);
  }
  if (h.kind == PaperControlHeader::Kind::JOIN) {
    NS_LOG_DEBUG("t=" << Simulator::Now().GetSeconds() << " JOIN arrival from=" << h.nodeId
                 << " tx=" << h.transaction << " stateEpoch=" << h.stateEpoch
                 << " mineStateEpoch=" << m_paperStateEpoch << " epoch=" << m_paperEpoch
                 << " clustering=" << m_paperClustering << " roundHead=" << m_paperRoundHead
                 << " headId=" << m_paperHeadId << " valid=" << PaperValidHeadForJoin());
  }
  if (h.kind == PaperControlHeader::Kind::JOIN
      && PaperValidHeadForJoin() && h.headId == MyNodeId()) {
    NS_LOG_DEBUG("t=" << Simulator::Now().GetSeconds() << " paper receives JOIN from=" << h.nodeId
                 << " transaction=" << h.transaction);
    m_paperPendingMembers[h.nodeId] = Simulator::Now();
    PaperSendControl(PaperControlHeader::Kind::ACCEPT,sender,MyNodeId(),false,h.transaction);
  } else if (h.kind == PaperControlHeader::Kind::ACCEPT && h.role == Head
             && h.stateEpoch == m_paperEpoch && h.transaction == m_pendingJoinTransaction
             && h.transaction != 0 && m_pendingJoinHead != 0xffff
             && h.nodeId == m_pendingJoinHead && h.headId == h.nodeId && h.clusterId != 0xffff
             && m_paperClustering && m_paperRoundHead == 0xffff) {
    ++m_paperAuditCounters.joinAccepts;
    NS_LOG_DEBUG("t=" << Simulator::Now().GetSeconds() << " paper ACCEPT from=" << h.nodeId
                 << " transaction=" << h.transaction);
    m_paperJoinEvent.Cancel();
    m_pendingJoinHead = 0xffff; m_pendingJoinAttempts = 0;
    m_pendingJoinAddress = Ipv4Address(); m_pendingJoinTransaction = 0;
    m_paperRoundHead = h.nodeId; m_paperRoundCluster = h.clusterId;
    PaperCommitAsMember(h.nodeId,h.clusterId);
    PaperFinishFormation();
  } else if (h.kind == PaperControlHeader::Kind::IFN_REQUEST && confirmedHead
             && h.headId == MyNodeId() && m_paperMembers.count(h.nodeId) && h.role == Member) {
    if (h.forwarder) { m_paperForwarders[h.nodeId] = Simulator::Now(); }
    else { m_paperForwarders.erase(h.nodeId); }
    PaperSendControl(PaperControlHeader::Kind::IFN_ACK,sender,MyNodeId(),h.forwarder);
  } else if (h.kind == PaperControlHeader::Kind::IFN_ACK && h.nodeId == m_paperHeadId
             && h.role == Head && h.headId == m_paperHeadId && m_ifnRequested) {
    m_paperForwarder = h.forwarder; m_lastIfnAck = Simulator::Now();
    PaperCommitSemanticRole();
    PaperSendControl(PaperControlHeader::Kind::HELLO);
  }
}

Ipv4Address RoutingProtocol::PaperNextHop(Ipv4Address destination)
{
  PaperPruneNeighbors();
  // Section III-E Step 1 applies before any role-specific decision.
  for (const auto& item : m_paperNeighbors) {
    if (item.second.address == destination) { return destination; }
  }
  if (!m_participates || m_paperHeadId == 0xffff) { return Ipv4Address(); }
  if (m_paperHeadId != MyNodeId() && !m_paperForwarder) {
    auto head = m_paperNeighbors.find(m_paperHeadId);
    return head != m_paperNeighbors.end() && head->second.role == Head
      ? head->second.address : Ipv4Address();
  }
  // The paper does not describe a destination-location service. Retain the explicit
  // ideal destination location assumption; never look up a remote cluster or CH.
  Vector position = GetPositionFromIp(destination);
  double bestDistance = std::numeric_limits<double>::infinity();
  uint16_t bestId = 0xffff;
  Ipv4Address best;
  for (const auto& item : m_paperNeighbors) {
    const auto& n = item.second;
    bool candidate = m_paperHeadId == MyNodeId()
      ? ((n.role == Head && n.headId == n.nodeId) ||
         (n.headId == MyNodeId() && n.forwarder && m_paperMembers.count(n.nodeId) && m_paperForwarders.count(n.nodeId)))
      : (n.nodeId == m_paperHeadId || (n.headId != 0xffff && n.headId != m_paperHeadId && n.role != Isolate));
    if (!candidate) { continue; }
    double distance = CalculateDistance(PaperNeighborPosition(n),position);
    if (distance < bestDistance || (distance == bestDistance && n.nodeId < bestId)) {
      bestDistance = distance; bestId = n.nodeId; best = n.address;
    }
  }
  return best;
}

Ptr<Ipv4Route> RoutingProtocol::PaperMakeRoute(Ipv4Address destination, Ipv4Address nextHop,
                                             Ipv4Address source)
{
  if (!m_Ipv4 || nextHop == Ipv4Address()) { return nullptr; }
  int32_t index = source == Ipv4Address() ? -1 : m_Ipv4->GetInterfaceForAddress(source);
  if (index < 0) {
    for (uint32_t i = 1; i < m_Ipv4->GetNInterfaces(); ++i) {
      if (m_Ipv4->IsUp(i) && m_Ipv4->GetNAddresses(i)) {
        index = i; source = m_Ipv4->GetAddress(i,0).GetLocal(); break;
      }
    }
  }
  if (index < 0) { return nullptr; }
  auto route = Create<Ipv4Route>();
  route->SetDestination(destination); route->SetSource(source);
  route->SetGateway(nextHop); route->SetOutputDevice(m_Ipv4->GetNetDevice(index));
  return route;
}

Ptr<Ipv4Route> RoutingProtocol::PaperRouteOutput(Ptr<Packet> packet, const Ipv4Header& header,
                                               Ptr<NetDevice> oif, Socket::SocketErrno& error)
{
  Ipv4Address destination = header.GetDestination();
  bool broadcast = destination.IsBroadcast();
  for (const auto& item : m_socketAddrs) { broadcast |= destination == item.second.GetBroadcast(); }
  PaperControlTag control;
  bool isControl = packet && packet->PeekPacketTag(control);
  Ipv4Address next = broadcast ? Ipv4Address::GetBroadcast() : PaperNextHop(destination);
  // All paper control is explicitly one-hop; it may not use another node as relay.
  if (isControl && !broadcast && next != destination) { next = Ipv4Address(); }
  auto route = PaperMakeRoute(destination,next,header.GetSource());
  if (!route || (oif && oif != route->GetOutputDevice())) {
    if (m_paperClustering && !isControl && !broadcast && !m_socketAddrs.empty()) {
      bool validInterface = !oif;
      for (const auto& item : m_socketAddrs) {
        int32_t index = m_Ipv4->GetInterfaceForAddress(item.second.GetLocal());
        validInterface |= index >= 0 && m_Ipv4->GetNetDevice(index) == oif;
      }
      if (validInterface) {
        if (packet) {
          DeferredRouteOutputTag deferred(oif ? m_Ipv4->GetInterfaceForDevice(oif) : -1);
          packet->ReplacePacketTag(deferred);
        }
        error = Socket::ERROR_NOTERROR;
        return LoopbackRoute(header,oif);
      }
    }
    error = Socket::ERROR_NOROUTETOHOST; return nullptr;
  }
  if (packet && !broadcast) {
    PaperLinkTag tag; tag.owner = MyNodeId();
    for (const auto& item : m_paperNeighbors) { if (item.second.address == next) { tag.next = item.first; break; } }
    packet->ReplacePacketTag(tag);
  }
  if (!isControl && !broadcast) { ++m_paperRouteCount; }
  error = Socket::ERROR_NOTERROR;
  return route;
}

void RoutingProtocol::PaperEnqueue(Ptr<const Packet> packet, const Ipv4Header& header,
                                   const UnicastForwardCallback& ucb, const ErrorCallback& ecb,
                                   int32_t outputInterface)
{
  PaperExpireQueue();
  if (!m_paperClustering || !packet || ucb.IsNull() || !m_maxQueueLen || m_maxQueueTime <= Time()) {
    if (!ecb.IsNull()) { ecb(packet,header,Socket::ERROR_NOROUTETOHOST); }
    return;
  }
  for (const auto& entry : m_paperQueue) {
    if (entry.packet->GetUid() == packet->GetUid() && entry.header.GetSource() == header.GetSource()
        && entry.header.GetDestination() == header.GetDestination()) { return; }
  }
  while (m_paperQueue.size() >= m_maxQueueLen) {
    auto oldest = m_paperQueue.front(); m_paperQueue.pop_front();
    if (!oldest.ecb.IsNull()) { oldest.ecb(oldest.packet,oldest.header,Socket::ERROR_NOROUTETOHOST); }
  }
  PaperQueuedPacket entry;
  entry.packet = packet; entry.header = header; entry.ucb = ucb; entry.ecb = ecb;
  entry.expires = Simulator::Now()+std::min(m_maxQueueTime,Seconds(1));
  entry.outputInterface = outputInterface;
  m_paperQueue.push_back(entry);
  PaperExpireQueue();
}

void RoutingProtocol::PaperExpireQueue()
{
  m_paperQueueEvent.Cancel();
  for (auto it = m_paperQueue.begin(); it != m_paperQueue.end();) {
    if (it->expires <= Simulator::Now()) {
      auto expired = *it; it = m_paperQueue.erase(it);
      if (!expired.ecb.IsNull()) { expired.ecb(expired.packet,expired.header,Socket::ERROR_NOROUTETOHOST); }
    } else { ++it; }
  }
  if (!m_paperQueue.empty()) {
    Time earliest = m_paperQueue.front().expires;
    for (const auto& entry : m_paperQueue) { earliest = std::min(earliest,entry.expires); }
    m_paperQueueEvent = Simulator::Schedule(earliest-Simulator::Now(),&RoutingProtocol::PaperExpireQueue,this);
  }
}

void RoutingProtocol::PaperFlushQueue()
{
  PaperExpireQueue();
  m_paperQueueEvent.Cancel();
  auto queued = std::move(m_paperQueue);
  m_paperQueue.clear();
  for (const auto& entry : queued) {
    auto route = PaperMakeRoute(entry.header.GetDestination(),PaperNextHop(entry.header.GetDestination()));
    if (entry.outputInterface >= 0 && route &&
        m_Ipv4->GetInterfaceForDevice(route->GetOutputDevice()) != entry.outputInterface) {
      if (!entry.ecb.IsNull()) { entry.ecb(entry.packet,entry.header,Socket::ERROR_NOROUTETOHOST); }
    } else { Forwarding(entry.packet,entry.header,entry.ucb,entry.ecb); }
  }
}

void RoutingProtocol::PaperSendReport()
{
  if (!m_Ipv4 || !m_participates) { return; }
  PaperSyncLocalState(); PaperCommitSemanticRole();
  auto c = GetObject<Node>()->GetObject<ClusterNode>();
  GsReportHeader report;
  report.nodeId = MyNodeId(); report.epoch = m_paperEpoch;
  report.factors = QLearningWeightOptimizer::GetUtilityFactors(GetObject<Node>(),m_maxRange,50.0);
  report.originEnergy = std::max(0.0,c->GetOriginEnergy());
  report.residualEnergy = std::max(0.0,c->GetResidualEnergy());
  report.energyChange = report.originEnergy > 0 ? (report.originEnergy-report.residualEnergy)/report.originEnergy : 0;
  report.roleChanges = m_lastRoleChangeCount;
  report.timestampNs = Simulator::Now().GetNanoSeconds();
  report.position = c->GetPosition(); report.velocity = c->GetVelocity();
  report.degree = std::count_if(m_paperNeighbors.begin(),m_paperNeighbors.end(),
                               [](const auto& item) { return item.second.role != Isolate; });
  report.role = c->GetRoleType();
  if (report.IsValid() && m_paperController) {
    auto packet = Create<Packet>(); packet->AddHeader(report);
    packet->AddHeader(TypeHeader(ICRA_GS_REPORT));
    ++m_gsReportCount; m_gsControlBytes += packet->GetSize();
    // An explicitly ideal, out-of-band ground channel, not an A2A UDP delivery.
    Simulator::Schedule(m_groundControlDelay,&RoutingProtocol::PaperReceiveReport,
                        Ptr<RoutingProtocol>(m_paperController),packet);
  }
  m_paperReportEvent = Simulator::Schedule(m_episodeInterval,&RoutingProtocol::PaperSendReport,this);
}

void RoutingProtocol::PaperReceiveReport(Ptr<Packet> packet)
{
  if (!m_Ipv4 || !m_isQLearningMaster || !m_paperMode) { return; }
  TypeHeader type; GsReportHeader report;
  if (!packet->RemoveHeader(type) || !type.IsValid() || type.Get() != ICRA_GS_REPORT
      || packet->GetSize() != report.GetSerializedSize() || !packet->RemoveHeader(report) || !report.IsValid()) { return; }
  Time timestamp = NanoSeconds(report.timestampNs);
  auto source = m_paperProtocols.find(report.nodeId);
  if (source == m_paperProtocols.end() || !source->second->m_participates
      || report.epoch > m_gsEpoch ||
      (m_lastGsReportEpochs.count(report.nodeId) && report.epoch < m_lastGsReportEpochs[report.nodeId])) { return; }
  if (timestamp > Simulator::Now() ||
      (m_lastGsReports.count(report.nodeId) && timestamp <= m_lastGsReports[report.nodeId])) { return; }
  m_lastGsReports[report.nodeId] = timestamp;
  m_lastGsReportEpochs[report.nodeId] = report.epoch;
  m_reportedNodes.insert(report.nodeId);
  QLearningWeightOptimizer::NodeStatus status;
  status.nodeId = report.nodeId; status.energy = report.residualEnergy;
  status.originEnergy = report.originEnergy; status.factors = report.factors;
  status.speed = report.velocity.GetLength(); status.degree = report.degree;
  status.position = report.position; status.role = report.role;
  status.timestamp = timestamp; status.lastRoleChange = timestamp;
  status.active = report.originEnergy > 0 && report.residualEnergy > 0;
  if (!status.active) { m_reportedNodes.erase(report.nodeId); }
  m_qLearningOptimizer->CollectNodeStatus(report.nodeId,status);
  QLearningWeightOptimizer::Feedback feedback;
  feedback.nodeId = report.nodeId; feedback.energyChange = report.energyChange;
  feedback.roleChanges = report.roleChanges; feedback.timestamp = timestamp;
  m_qLearningOptimizer->CollectFeedback(report.nodeId,feedback);
}

void RoutingProtocol::PaperGroundControlTick()
{
  if (!m_Ipv4 || m_paperController != this) { return; }
  for (const auto& entry : m_lastGsReports) {
    if (Simulator::Now()-entry.second > m_episodeInterval*2 + m_groundControlDelay) {
      QLearningWeightOptimizer::NodeStatus expired;
      expired.nodeId = entry.first; expired.active = false; expired.timestamp = Simulator::Now();
      m_qLearningOptimizer->CollectNodeStatus(entry.first,expired);
      m_reportedNodes.erase(entry.first);
    }
  }
  if (m_qLearningOptimizer->IsEpisodeActive()) { m_qLearningOptimizer->EndEpisode(); }
  if (!m_reportedNodes.empty()) {
    m_qLearningOptimizer->StartEpisode();
    auto weights = m_qLearningOptimizer->GetCurrentWeights();
    GsPolicyHeader policy;
    policy.weights = {{weights.energyWeight,weights.degreeWeight,weights.velocityWeight,weights.distanceWeight}};
    policy.epoch = ++m_gsEpoch; policy.timestampNs = Simulator::Now().GetNanoSeconds();
    for (uint16_t id : m_reportedNodes) {
      auto target = m_paperProtocols.find(id);
      if (target == m_paperProtocols.end() || !target->second->m_participates) { continue; }
      auto packet = Create<Packet>(); packet->AddHeader(policy);
      packet->AddHeader(TypeHeader(ICRA_GS_POLICY));
      ++m_gsPolicyCount; m_gsControlBytes += packet->GetSize();
      Simulator::Schedule(m_groundControlDelay,&RoutingProtocol::PaperReceivePolicy,
                          Ptr<RoutingProtocol>(target->second),packet);
    }
  }
  m_qlearningEpisodeTimer.Schedule(m_episodeInterval);
}

void RoutingProtocol::PaperApplyPolicy()
{
  if (!m_hasPendingPolicy) { return; }
  m_hasPendingPolicy = false;
  const std::array<double,4> old = {{m_weights.energyWeight,m_weights.degreeWeight,
                                     m_weights.velocityWeight,m_weights.distanceWeight}};
  bool changed = false;
  for (uint32_t i = 0; i < 4; ++i) {
    changed |= std::abs(old[i]-m_pendingPolicyWeights[i]) > 1e-12;
  }
  m_weights.energyWeight = m_pendingPolicyWeights[0];
  m_weights.degreeWeight = m_pendingPolicyWeights[1];
  m_weights.velocityWeight = m_pendingPolicyWeights[2];
  m_weights.distanceWeight = m_pendingPolicyWeights[3];
  m_paperEpoch = m_pendingPolicyEpoch;
  ++m_appliedStrategyCount;
  m_routingTable.Clear();
  if (m_paperRebuildEveryPolicy || changed) {
    // III-C.5.5: the update message itself returns every node to Step 1. Equal
    // weights do not imply an unchanged topology after movement.
    ++m_paperAuditCounters.policyRebuilds;
    PaperStartClustering();
  } else {
    // Legacy diagnostic: retain the committed topology across an equal-weight
    // epoch. A changed action still reconstructs, as in the audited baseline.
    m_paperStateEpoch = m_paperEpoch;
    PaperSendControl(PaperControlHeader::Kind::HELLO);
  }
}

void RoutingProtocol::PaperReceivePolicy(Ptr<Packet> packet)
{
  if (!m_Ipv4 || !m_paperMode || !m_participates) { return; }
  TypeHeader type; GsPolicyHeader policy;
  if (!packet->RemoveHeader(type) || !type.IsValid() || type.Get() != ICRA_GS_POLICY
      || packet->GetSize() != policy.GetSerializedSize() || !packet->RemoveHeader(policy)
      || !policy.IsValid() || policy.epoch <= m_paperEpoch
      || (m_hasPendingPolicy && policy.epoch < m_pendingPolicyEpoch)
      || NanoSeconds(policy.timestampNs) > Simulator::Now()) { return; }
  m_hasPendingPolicy = true;
  m_pendingPolicyEpoch = policy.epoch;
  for (uint32_t i = 0; i < 4; ++i) { m_pendingPolicyWeights[i] = policy.weights[i]; }
  // Never change the control epoch in the middle of a round: in-flight
  // JOIN/ACCEPT/CALL packets carry the old epoch and would be rejected. A round
  // is bounded below the policy period, so the latest policy applies at its end.
  if (!m_paperClustering) { PaperApplyPolicy(); }
}

void RoutingProtocol::SaveQLearningModel(const std::string& filename)
{
  NS_LOG_FUNCTION(this << filename);
  if (m_isQLearningMaster && m_qLearningOptimizer) {
    NS_LOG_INFO("Finalizing last episode before saving model");
    m_qLearningOptimizer->EndEpisode();  // 强制结束最后一个episode
  }
  m_qLearningOptimizer->SaveQTable(filename);
}

void RoutingProtocol::LoadQLearningModel(const std::string& filename)
{
  NS_LOG_FUNCTION(this << filename);
  m_qLearningOptimizer->LoadQTable(filename);
}











} // namespace icra
} // ns3
