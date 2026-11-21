/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "cafbcr-helper.h"
#include "ns3/cafbcr-routing.h"
#include "ns3/cafbcr-election.h"
#include "ns3/node-list.h"
#include "ns3/names.h"
#include "ns3/ipv4-list-routing.h"
#include "ns3/log.h"
#include "ns3/fanet-clustermanager.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("cafbcrHelper");

cafbcrHelper::cafbcrHelper()
  : m_helloInterval(Seconds(0.25)),
    m_beaconInterval(Seconds(0.75)),
    m_maxRange(500.0),
    m_wLHTP(0.2),
    m_wEnergy(0.2),
    m_wDegree(0.2),
    m_wDistance(0.2),
    m_wCompute(0.2),
    m_helloMethod("fixed")
{
  m_agentFactory.SetTypeId("ns3::cafbcr::RoutingProtocol");
}

cafbcrHelper::~cafbcrHelper()
{
}

cafbcrHelper* cafbcrHelper::Copy() const
{
  return new cafbcrHelper(*this);
}

Ptr<Ipv4RoutingProtocol> cafbcrHelper::Create(Ptr<Node> node) const
{
  NS_LOG_FUNCTION(this << node);
  
  // 创建路由协议实例
  Ptr<cafbcr::RoutingProtocol> agent = m_agentFactory.Create<cafbcr::RoutingProtocol>();
  
  // 设置参数
  agent->SetHelloInterval(m_helloInterval);
  agent->SetBeaconInterval(m_beaconInterval);
  agent->SetMaxRange(m_maxRange);
  agent->SetFitnessWeights(m_wLHTP, m_wEnergy, m_wDegree, m_wDistance, m_wCompute);

  if(m_helloMethod == "fuzzy"){
    agent->SetHelloMode(cafbcr::FUZZY_HELLO);
  } else if(m_helloMethod == "qlearning"){
    agent->SetHelloMode(cafbcr::QLEARNING_HELLO);
  } else{
    agent->SetHelloMode(cafbcr::FIXED_HELLO);
  }
  
  
  // 获取或创建ClusterNode
  Ptr<ClusterNode> clusterNode = node->GetObject<ClusterNode>();
  if (!clusterNode) {
    NS_LOG_ERROR("ClusterNode not found on node " << node->GetId() <<
                 " - it should be aggregated before installing routing");
  } else {
    // 为ClusterNode创建并配置Neighbors对象（邻居表）
    clusterNode->SetFitnessMode(m_fitnessMode);
    Ptr<Neighbors> neighbors = clusterNode->GetObject<Neighbors>();
    if (!neighbors) {
      neighbors = CreateObject<Neighbors>(node->GetId());
      clusterNode->AggregateObject(neighbors);
      NS_LOG_DEBUG("Created Neighbors for node " << node->GetId());
    }
  }

  // 创建并配置cafbcrElection (使用ABC算法)
  Ptr<cafbcr::cafbcrElection> cafbcrElection = CreateObject<cafbcr::cafbcrElection>();

  // Pass the five fitness weights to the election module.
  double weightArray[5] = {m_wLHTP, m_wEnergy, m_wDegree, m_wDistance, m_wCompute};
  cafbcrElection->SetWeightTable(weightArray);

  // 聚合到节点
  node->AggregateObject(cafbcrElection);
  node->AggregateObject(agent);

  NS_LOG_INFO("Created cafbcr routing protocol on node " << node->GetId()
              << " with K=" << cafbcrElection->GetMaxClusterNum()
              << " (TotalNodes=" << NodeList::GetNNodes() << ")");
  
  return agent;
}

void cafbcrHelper::Set(std::string name, const AttributeValue& value)
{
  m_agentFactory.Set(name, value);
}

void cafbcrHelper::SetHelloInterval(Time interval)
{
  NS_LOG_FUNCTION(this << interval);
  m_helloInterval = interval;
}

void cafbcrHelper::SetBeaconInterval(Time interval)
{
  NS_LOG_FUNCTION(this << interval);
  m_beaconInterval = interval;
}

void cafbcrHelper::SetMaxRange(double range)
{
  NS_LOG_FUNCTION(this << range);
  NS_ASSERT_MSG(range > 0, "Max range must be positive");
  m_maxRange = range;
}

void cafbcrHelper::SetFitnessWeights(double wLHTP, double wEnergy, double wDegree, double wDistance, double wCompute)
{
  NS_LOG_FUNCTION(this << wLHTP << wEnergy << wDegree << wDistance << wCompute);

  double sum = wLHTP + wEnergy + wDegree + wDistance + wCompute;
  NS_ASSERT_MSG(std::abs(sum - 1.0) < 0.001, "Fitness weights should sum to 1.0");

  m_wLHTP = wLHTP;
  m_wEnergy = wEnergy;
  m_wDegree = wDegree;
  m_wDistance = wDistance;
  m_wCompute = wCompute;

  NS_LOG_INFO("Set Fitness weights: LHTP=" << m_wLHTP << " Energy=" << m_wEnergy
              << " Degree=" << m_wDegree << " Distance=" << m_wDistance
              << " Compute=" << m_wCompute);
}

int64_t cafbcrHelper::AssignStreams(NodeContainer c, int64_t stream)
{
  NS_LOG_FUNCTION(this << stream);
  
  int64_t currentStream = stream;
  Ptr<Node> node;
  
  for (NodeContainer::Iterator i = c.Begin(); i != c.End(); ++i) {
    node = (*i);
    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
    NS_ASSERT_MSG(ipv4, "Ipv4 not installed on node");
    
    Ptr<Ipv4RoutingProtocol> proto = ipv4->GetRoutingProtocol();
    NS_ASSERT_MSG(proto, "Ipv4 routing not installed on node");
    
    // 尝试直接获取cafbcr协议
    Ptr<cafbcr::RoutingProtocol> cafbcr = DynamicCast<cafbcr::RoutingProtocol>(proto);
    
    if (cafbcr) {
      currentStream += cafbcr->AssignStreams(currentStream);
      continue;
    }
    
    // 如果是列表路由，遍历查找
    Ptr<Ipv4ListRouting> list = DynamicCast<Ipv4ListRouting>(proto);
    if (list) {
      int16_t priority;
      Ptr<Ipv4RoutingProtocol> listProto;
      Ptr<cafbcr::RoutingProtocol> listcafbcr;
      
      for (uint32_t j = 0; j < list->GetNRoutingProtocols(); j++) {
        listProto = list->GetRoutingProtocol(j, priority);
        listcafbcr = DynamicCast<cafbcr::RoutingProtocol>(listProto);
        
        if (listcafbcr) {
          currentStream += listcafbcr->AssignStreams(currentStream);
          break;
        }
      }
    }
  }
  
  if (auto manager = ClusterManager::GetInstance()) {
    currentStream += manager->AssignStreams(currentStream);
  }
  return (currentStream - stream);
}


} // namespace ns3
