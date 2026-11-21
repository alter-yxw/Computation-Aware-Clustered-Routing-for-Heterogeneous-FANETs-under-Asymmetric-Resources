/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "fanet-clustermanager.h"
// #include "ns3/cafbcr-routing.h"
#include "ns3/log.h"
#include "ns3/ptr.h"
#include "ns3/core-module.h"
#include <cmath>
#include "ns3/cafbcr-module.h"
#include "ns3/icra-module.h"
#include "ns3/internet-module.h"
namespace ns3{

NS_LOG_COMPONENT_DEFINE("ClusterManager");


NS_OBJECT_ENSURE_REGISTERED(Cluster);


TypeId Cluster::GetTypeId()
{
  static TypeId tid = TypeId("ns3::Cluster")
  .SetParent<Object> ()
  .AddConstructor<Cluster> ();
  return tid;
}

Cluster::Cluster()
  :  m_InitializeTime(0.0)
{
  
}

void Cluster::UpdateMemberTime(uint16_t NodeId, Time NowTime)
{
  if(  m_ClusterMembers.find(NodeId) != m_ClusterMembers.end() && m_MembersUpdateTime.find(NodeId) != m_MembersUpdateTime.end()){
      m_MembersUpdateTime[NodeId] = NowTime;
  }
  else{
      NS_LOG_ERROR("UpdateMemberTime: Member(NodeId) "<<NodeId<<"does not exist in cluster!");\
  }
}

Time Cluster::GetMemberUpdateTime(uint16_t NodeId) const
{
  for (auto mem=m_MembersUpdateTime.begin();mem!=m_MembersUpdateTime.end();++mem)
  {
    if (mem->first == NodeId)
    {
      return mem->second;
    }
  }
  NS_LOG_ERROR("GetMemberUpdateTime: NodeId " << NodeId << " not found in cluster!");
  return Seconds(0);
}



// 网关表管理方法实现
void Cluster::AddGateway(const GatewayInfo& gwInfo)
{
  NS_LOG_FUNCTION(this << gwInfo.gatewayAddr << gwInfo.linkedClusterId);
  m_gatewayTable[gwInfo.gatewayAddr].push_back(gwInfo);
  NS_LOG_DEBUG("Gateway " << gwInfo.gatewayNodeId << " added to cluster " << m_ClusterId 
               << ", links to cluster " << gwInfo.linkedClusterId);
}

void Cluster::UpdateGateway(Ipv4Address gwAddr, uint16_t linkedClusterId, double linkQuality, double distance)
{
  NS_LOG_FUNCTION(this << gwAddr << linkedClusterId << linkQuality << distance);
  
  if (m_gatewayTable.find(gwAddr) == m_gatewayTable.end()) {
    NS_LOG_WARN("Gateway " << gwAddr << " not found in cluster " << m_ClusterId);
    return;
  }
  
  for (auto& gw : m_gatewayTable[gwAddr]) {
    if (gw.linkedClusterId == linkedClusterId) {
      gw.linkQuality = linkQuality;
      gw.distance = distance;
      gw.lastUpdate = Simulator::Now();
      gw.isActive = true;
      NS_LOG_DEBUG("Gateway " << gwAddr << " updated in cluster " << m_ClusterId);
      return;
    }
  }
}

void Cluster::RemoveGateway(Ipv4Address gwAddr, uint16_t linkedClusterId)
{
  NS_LOG_FUNCTION(this << gwAddr << linkedClusterId);
  
  if (m_gatewayTable.find(gwAddr) == m_gatewayTable.end()) {
    return;
  }
  
  auto& gateways = m_gatewayTable[gwAddr];
  for (auto it = gateways.begin(); it != gateways.end(); ++it) {
    if (it->linkedClusterId == linkedClusterId) {
      gateways.erase(it);
      NS_LOG_DEBUG("Gateway " << gwAddr << " removed from cluster " << m_ClusterId);
      break;
    }
  }
  
  if (gateways.empty()) {
    m_gatewayTable.erase(gwAddr);
  }
}

void Cluster::CleanExpiredGateways(Time timeout)
{
  NS_LOG_FUNCTION(this << timeout.GetSeconds());
  
  for (auto it = m_gatewayTable.begin(); it != m_gatewayTable.end(); ) {
    auto& gateways = it->second;
    for (auto gwIt = gateways.begin(); gwIt != gateways.end(); ) {
      if (Simulator::Now() - gwIt->lastUpdate > timeout) {
        gwIt->isActive = false;
        NS_LOG_DEBUG("Gateway " << gwIt->gatewayAddr << " expired in cluster " << m_ClusterId);
        gwIt = gateways.erase(gwIt);
      } else {
        ++gwIt;
      }
    }
    
    if (gateways.empty()) {
      it = m_gatewayTable.erase(it);
    } else {
      ++it;
    }
  }
}

std::vector<GatewayInfo> Cluster::GetGatewaysToCluster(uint16_t targetClusterId) const
{
  NS_LOG_FUNCTION(this << targetClusterId);
  std::vector<GatewayInfo> result;
  
  for (const auto& entry : m_gatewayTable) {
    for (const auto& gw : entry.second) {
      if (gw.linkedClusterId == targetClusterId && gw.isActive) {
        result.push_back(gw);
      }
    }
  }
  
  NS_LOG_DEBUG("Found " << result.size() << " gateways to cluster " << targetClusterId);
  return result;
}

bool Cluster::HasGatewayToCluster(uint16_t targetClusterId) const
{
  for (const auto& entry : m_gatewayTable) {
    for (const auto& gw : entry.second) {
      if (gw.linkedClusterId == targetClusterId && gw.isActive) {
        return true;
      }
    }
  }
  return false;
}













NS_OBJECT_ENSURE_REGISTERED(ClusterManager);


TypeId ClusterManager::GetTypeId()
{
  static TypeId tid = TypeId("ns3::ClusterManager")
    .SetParent<Object>()
    .SetGroupName("cafbcr")
    .AddConstructor<ClusterManager>();
  return tid;
}

ClusterManager::ClusterManager()
  :   m_rand(CreateObject<UniformRandomVariable>()),
      GlobalClusterId(0)
{

}

int64_t ClusterManager::AssignStreams(int64_t stream)
{
  NS_LOG_FUNCTION(this << stream);
  m_rand->SetStream(stream);
  return 1;
}

Ptr<ClusterManager> ClusterManager::m_instance = nullptr;

Ptr<ClusterManager> ClusterManager::GetInstance()
{
  if (m_instance == nullptr)
  {
    m_instance = CreateObject<ClusterManager>();
  }
  return m_instance;
}

void ClusterManager::CreateCluster(uint16_t clusterId, Ptr<ClusterNode> ChNode)
{
  if(m_clusters.find(clusterId) != m_clusters.end())
  {
    NS_LOG_WARN("Cluster " << clusterId << " already exists");
    return;
  }
  Ptr<Cluster> newCluster = CreateObject<Cluster>();
  newCluster->SetClusterId(clusterId);
  newCluster->RecordInitializeTime(Simulator::Now());
  if(ChNode!=nullptr)
  {
    newCluster -> SetClusterHead(ChNode);
    ChNode->SetClusterId(clusterId);
  }
  m_clusters[clusterId] = newCluster;
  ClusterIdIncrease();
  RecordClusterCreation(clusterId, ChNode ? ChNode->GetNodeId() : 0xFFFF);
  NS_LOG_INFO("Created cluster " << clusterId);
}

Ptr<Cluster> ClusterManager::GetCluster(uint16_t clusterId){
  for (auto c = m_clusters.begin(); c!=m_clusters.end(); ++c){
    if(c->second->GetClusterId() == clusterId){
      return c->second;
    }
  }
  return nullptr;
}

std::map<uint16_t, Ptr<Cluster>>& ClusterManager::GetAllClusters(){
  return m_clusters;
}
 
void ClusterManager::RemoveCluster(uint16_t clusterId)
{
  if(GetCluster(clusterId) == nullptr)
  {
    return;
  }
  RecordClusterDissolution(clusterId);
  if(GetCluster(clusterId)->GetAllMembers().size() > 0)
  {
    auto members = GetCluster(clusterId)->GetAllMembers();
    for (auto member = members.begin(); member!=members.end();++member)
    {
      Ptr<ClusterNode> cNode = member->second;
      GetCluster(clusterId)->RemoveMember(cNode);
    }
  }
  m_clusters.erase(clusterId);
}

void ClusterManager::LeaveCluster(uint16_t NodeId, uint16_t ClusterId)
{
  if(ClusterId>ArrangeClusterId()){return;}

  Ptr<ClusterNode> clusternode = NodeList::GetNode(NodeId)->GetObject<ClusterNode>();
  m_clusters[ClusterId]->RemoveMember(clusternode);
  UpdateClusterSize(ClusterId);
}

void ClusterManager::JoinCluster(uint16_t clusterId, Ptr<ClusterNode> Node)
{
  m_clusters[clusterId]->AddClusterMember(Node,Simulator::Now());
  m_clusters[clusterId]->UpdateLatestTime(Simulator::Now());
  UpdateClusterSize(clusterId);
  NS_LOG_INFO("Node "<<Node->GetNodeId()<<" join in cluster "<<clusterId);

}

bool ClusterManager::ExistMember(uint16_t NodeId){
  for(auto cluster = m_clusters.begin();cluster!= m_clusters.end();++cluster){
    if(cluster->second->ExistMember(NodeId) || cluster->second->GetClusterHeadId() == NodeId){
      return true;
    }
  }
  return false;
}

bool ClusterManager::ExistHead(uint16_t ChNodeId)
{
  bool exist = false;
  if (!ExistMember(ChNodeId)) {
    return exist;
  }

  for (auto cluster=m_clusters.begin();cluster!=m_clusters.end();++cluster){
    if(cluster->second->GetClusterHeadId()==ChNodeId)  {
      exist = true;
      break;
    }
  }
  return exist;
}

uint16_t ClusterManager::GetClusterId(uint16_t NodeId)
{
  for(auto cluster = m_clusters.begin(); cluster != m_clusters.end(); ++cluster)
  {
    if(cluster->second->ExistMember(NodeId) || cluster->second->GetClusterHeadId() == NodeId)
    {
      return cluster->second->GetClusterId();
    }
  }  
  return 0xFFFF;
}

uint16_t ClusterManager::GetClusterHeadId(uint16_t clusterId)
{
    if(m_clusters.find(clusterId) == m_clusters.end())
    {
      return 0xFFFF;
    }
    return m_clusters.at(clusterId)->GetClusterHeadId();
}

Ptr<Cluster> ClusterManager::FindNodeCluster(uint16_t nodeId)
{
  uint16_t clusterId = GetClusterId(nodeId);
  if(clusterId > ArrangeClusterId()) {
    NS_LOG_WARN("Node "<< nodeId << " not in any cluster yet.");
    return nullptr;
  }
  else{
    return m_clusters.at(clusterId);
  }
}

void ClusterManager::CheckClusteringStatus(uint16_t NodeId){
  NS_LOG_FUNCTION(this<<NodeId);
  if (NodeId > NodeList::GetNNodes()) {
    NS_LOG_ERROR("Invalid NodeId: " << NodeId);
    // std::cout<<"Invalid NodeId: " << NodeId<<std::endl;
    return;
  }
  Ptr<Node> node = NodeList::GetNode(NodeId);
  Ptr<ClusterNode> clusternode = node->GetObject<ClusterNode>();
  TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
  /*How to Get the socketAddr*/
  bool InSomeCluster = false;
  for(auto cluster = m_clusters.begin();cluster!=m_clusters.end();++cluster)
  {
    if(cluster->second->ExistMember(NodeId) || cluster->second->GetClusterHeadId() == NodeId) 
    {
      InSomeCluster = true;
      break;
    }
  }
  if(!InSomeCluster){
    NS_LOG_INFO("Node "<<NodeId<<" is not in any cluster now, "<<Simulator::Now().GetSeconds()<<" s.");
    bool loserHead = false;
    if(clusternode->GetRoleType() == BackupHead){
      loserHead = true;  // in initial stage
    }
    if(!loserHead){// does not pre-CH
      // Do not leave the node in Isolate: every join path accepts only Idle,
      // so an Isolate node would never rejoin. Reset to Idle so the periodic
      // ProactivelyJoinCluster path can adopt it again.
      clusternode->SetRole(Idle);
      clusternode->SetClusterId(0xFFFF);
    }
    uint16_t MaxFitnessIndex = 0xFFFF;
    uint16_t tempIndex = 0xFFFF; 
    double MaxFit = 0;
    bool hasNeighborCH = false;
    auto neighbors = clusternode->GetNeighborTable()->GetAll();
    for (auto neighbor = neighbors.begin();neighbor!=neighbors.end(); ++neighbor){
      if(neighbor->second.Role == BackupHead || neighbor->second.Role == Head){
        if(neighbor->second.UpdateTime.GetSeconds() == 0) {continue;}
        hasNeighborCH = true;
        tempIndex = neighbor->second.NodeId;
        if(neighbor->second.FitnessValue>MaxFit)
        {
          MaxFit = neighbor->second.FitnessValue;
          MaxFitnessIndex = neighbor->second.NodeId;
        }
      }
    }
    if(!hasNeighborCH){ // does not find CH in [nodeId]'s neighbor
      if(clusternode->GetFitnessValue()>1)
      {std::cout<<"Node "<<clusternode->GetNodeId()<<": "<<clusternode->GetFitnessValue()<<" -FitValueError"<<std::endl;}

      // 优化：区分初始分簇和维护阶段（参考cafbcr的TryToJoinCluster）
      double currentTime = Simulator::Now().GetSeconds();
      bool isInitialPhase = (currentTime < 10.0);  // 前10秒为初始分簇阶段（更宽松）

      if(isInitialPhase) {
        // 初始分簇阶段：允许创建簇（但仍需检查邻居条件）
        uint32_t neighborCount = neighbors.size();
        uint32_t idleNeighborCount = 0;
        for(auto it = neighbors.begin(); it != neighbors.end(); ++it) {
          if(it->second.Role == Idle) {
            idleNeighborCount++;
          }
        }

        if(neighborCount == 0 || (neighborCount >= 5 && idleNeighborCount >= 3)) {
          Time timer = Seconds(1-clusternode->GetFitnessValue());
          Simulator::Schedule(timer, &ClusterManager::SendClusterHeadCall, this, NodeId);
          NS_LOG_INFO("Node " << NodeId << " creating cluster in initial phase (time=" << currentTime
                      << ", neighbors=" << neighborCount << ", idle=" << idleNeighborCount << ")");
        }
        else {
          clusternode->SetRole(Idle);
          NS_LOG_DEBUG("Node " << NodeId << " staying Idle (initial phase, insufficient neighbors="
                       << neighborCount << ", idle=" << idleNeighborCount << ")");
        }
      }
      else {
        // 维护阶段（t>=10s）：完全禁止创建新簇，保持Idle状态
        // 参考cafbcr的TryToJoinCluster - 只加入现有簇，不创建新簇
        clusternode->SetRole(Idle);
        NS_LOG_INFO("Node " << NodeId << " staying Idle in maintenance phase (time=" << currentTime
                    << "), no cluster creation allowed");

        // 可选：定期尝试重新加入（通过CheckClusteringStatus会在下次BeaconTimer时再次尝试）
      }
    }
    else{
      // if(neighbors.find(MaxFitnessIndex) == neighbors.end()){
      //   NS_LOG_ERROR("ERROR MaxFitValue out of range. ");
      //   std::cout<<"Error Here, Value: "<<MaxFitnessIndex<<std::endl;
      // }
      Ipv4Address dst;
      if(MaxFitnessIndex < NodeList::GetNNodes())
      {
        dst = neighbors.at(MaxFitnessIndex).ipAddress;
      }
      else{
        dst = neighbors.at(tempIndex).ipAddress;
        // std::cout<<"SenderId: "<<NodeId<<", below error"<<", dst: "<<dst<<", Time: "<<Simulator::Now().GetSeconds()<<" s, "<<std::endl;
      }

      double timer = NodeId + m_rand->GetValue(0.0, 20.0);
      // std::cout<<"SenderId: "<<NodeId<<", MaxFitindex: "<<MaxFitnessIndex<<", dst: "<<dst<<std::endl;
      Simulator::Schedule(MilliSeconds(timer), &ClusterManager::SendJoinRequest,this, NodeId, dst);
    }

  } // end of if not insomecluster

}


bool ClusterManager::IsInCluster(uint16_t nodeId, uint16_t clusterId)
{
  if (clusterId >= ArrangeClusterId()) {  return false; }
  
  Ptr<Cluster> cluster = GetCluster(clusterId);
  if (!cluster) {  return false; }
  
  auto members = cluster->GetAllMembers();
  for (auto it = members.begin(); it != members.end(); ++it) {
    if (it->second->GetNodeId() == nodeId) {
      return true;
    }
  }
  
  return false;
}



void ClusterManager::RecordClusterCreation(uint16_t clusterId, uint16_t chNodeId)
{
  ClusterStatistics stats;
  stats.clusterId = clusterId;
  stats.creationTime = Simulator::Now();
  stats.clusterHeadId = chNodeId;
  stats.maxSize = 1; // 初始只有簇头  
  m_clusterHistory.push_back(stats);  // 添加到历史记录(解散时会更新)  
  m_clusterSizeTracker[clusterId] = 1;  // 初始化规模跟踪
}


void ClusterManager::RecordClusterDissolution(uint16_t clusterId)
{
  Ptr<Cluster> cluster = GetCluster(clusterId);
  if(cluster == nullptr) { return; }  

  for(auto& stats : m_clusterHistory)  // find the cluster in history and update the dissolve event
  {
    if(stats.clusterId == clusterId && stats.dissolutionTime == Seconds(0))
    {
      stats.dissolutionTime = Simulator::Now();
      stats.lifetime = (stats.dissolutionTime - stats.creationTime).GetSeconds();
      stats.dissolutionSize = cluster->GetAllMembers().size(); // include the cluster head
      // Update maxSize to the size at dissolution so lifetime statistics are not zero.
      if(stats.dissolutionSize > stats.maxSize) {
        stats.maxSize = stats.dissolutionSize;
      }
      NS_LOG_DEBUG("Cluster " << clusterId << " dissolved after " << stats.lifetime << "s with final size " << stats.dissolutionSize);
      break;
    }
  }  
  m_clusterSizeTracker.erase(clusterId);  // no need to chase to scale of the cluster
}


void ClusterManager::UpdateClusterSize(uint16_t clusterId)
{
  Ptr<Cluster> cluster = GetCluster(clusterId);
  if(cluster == nullptr) { return; }  
  uint32_t currentSize = cluster->GetAllMembers().size(); // include the cluster head
  m_clusterSizeTracker[clusterId] = currentSize;  

  for(auto& stats : m_clusterHistory)
  {
    if(stats.clusterId == clusterId && stats.dissolutionTime == Seconds(0))
    {
      if(currentSize > stats.maxSize)  {  stats.maxSize = currentSize;  }  // update the max scale in history
      break;
    }
  }
}


void ClusterManager::GetClusteringStatistics(uint32_t& totalClusters,  double& avgLifetime, double& avgSize) const
{
  totalClusters = m_clusterHistory.size();  
  if(totalClusters == 0)
  {
    avgLifetime = 0.0;
    avgSize = 0.0;
    return;
  }  
  double totalLifetime = 0.0;
  double totalSize = 0.0;
  uint32_t dissolvedCount = 0;  
  for(const auto& stats : m_clusterHistory)
  {
    if(/*stats.dissolutionTime != Seconds(0) &&*/ stats.maxSize > 1) // 只统计已解散的簇
    {
      totalLifetime += stats.lifetime;
      totalSize += stats.maxSize;
      dissolvedCount++;
    }
  }  
  avgLifetime = (dissolvedCount > 0) ? (totalLifetime / dissolvedCount) : 0.0;
  avgSize = (dissolvedCount > 0) ? (totalSize / dissolvedCount) : 0.0;
}


void ClusterManager::GetActiveClusterStats(uint32_t& count, double& avgSize, double &AvgSizePro) const
{
  count = m_clusters.size();
  if(count == 0)
  {
    avgSize = 0.0;
    AvgSizePro = 0.0;
    return;
  }
  uint16_t countPro = 0;
  uint32_t totalSize = 0;
  uint32_t SumSize = 0;
  for(const auto& cluster : m_clusters)
  {
    totalSize += cluster.second->GetAllMembers().size(); // 包括簇头
    for(auto status : m_clusterHistory)
    {
      if(status.clusterId == cluster.second->GetClusterId() && status.maxSize > 1) 
      {
        SumSize += cluster.second->GetAllMembers().size(); // 统计有效簇（非孤立的簇）
        countPro ++;
        break;
      }
    }
  }
  avgSize = static_cast<double>(totalSize) / count;
  // AvgSizePro averages only valid clusters with maxSize > 1.
  AvgSizePro = (countPro > 0) ? static_cast<double>(SumSize) / countPro : 0.0;
}


void ClusterManager::ResetStatistics()
{
  m_clusterHistory.clear();
  m_clusterSizeTracker.clear();
}

void ClusterManager::SendClusterHeadCall(uint16_t NodeId)
{
  Ptr<Node> node = NodeList::GetNode(NodeId);
  Ptr<ClusterNode> clusternode = node->GetObject<ClusterNode>();
  /*Catch the routingprotocol*/
  Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
  Ptr<Ipv4RoutingProtocol> routing = ipv4->GetRoutingProtocol();
  if (DynamicCast<cafbcr::RoutingProtocol>(routing)){
    auto myRouteProto = DynamicCast<cafbcr::RoutingProtocol>(routing);
    myRouteProto->SendClusterHeadAnnounceMent();
  } else if (DynamicCast<icra::RoutingProtocol>(routing)){
    auto myRouteProto = DynamicCast<icra::RoutingProtocol>(routing);
    myRouteProto->SendClusterHeadAnnounceMent();
  };
  return;
}


void ClusterManager::SendJoinRequest(uint16_t NodeId, Ipv4Address dst)
{
  Ptr<Node> node = NodeList::GetNode(NodeId);
  Ptr<ClusterNode> clusternode = node->GetObject<ClusterNode>();
  Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
  Ptr<Ipv4RoutingProtocol> routing = ipv4->GetRoutingProtocol();
  if (DynamicCast<cafbcr::RoutingProtocol>(routing)){
    auto myRouteProto = DynamicCast<cafbcr::RoutingProtocol>(routing);
    myRouteProto->SendClusterJoinRequest(dst);
  } else if (DynamicCast<icra::RoutingProtocol>(routing)){
    auto myRouteProto = DynamicCast<icra::RoutingProtocol>(routing);
    myRouteProto->SendClusterJoinRequest(dst);
  };
  return;
}




} // namespace ns3
