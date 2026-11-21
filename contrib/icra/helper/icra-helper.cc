/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "icra-helper.h"
#include "ns3/icra-routing.h"
// #include "ns3/icra-routing.h"
#include "ns3/icra-Qweight.h"
#include "ns3/ipv4-list-routing.h"
#include "ns3/names.h"
#include "ns3/log.h"
#include "ns3/ptr.h"
#include "ns3/node-list.h"
#include <cmath>
#include <stdexcept>

namespace ns3{
NS_LOG_COMPONENT_DEFINE("IcraHelper");

icraHelper::icraHelper() :Ipv4RoutingHelper()
{  
  m_agentFactory.SetTypeId("ns3::icra::RoutingProtocol");
}

icraHelper::~icraHelper()
{
}

icraHelper* icraHelper::Copy() const
{
  return new icraHelper(*this);
}


Ptr<Ipv4RoutingProtocol> icraHelper::Create(Ptr<Node> node ) const
{ 
  Ptr<icra::RoutingProtocol> agent = m_agentFactory.Create<icra::RoutingProtocol>();  
  agent->Configure(m_preset == "paper", m_range, m_episodeInterval,
                   m_lhtpThreshold, m_roleChangeThreshold, m_groundStationDelay,
                   m_excludedNodeId);

  Ptr<icra::QLearningWeightOptimizer> optimizer = icra::QLearningWeightOptimizer::GetInstance();
  if (m_configuredOptimizer != optimizer) {
    optimizer->SetLegacyMode(m_preset == "legacy");
    optimizer->SetLearningRate(m_alpha);
    optimizer->SetStabilityWeight(m_lambda);
    optimizer->SetExplorationRate(m_epsilon);
    optimizer->SetMetricParameters(m_range);
    optimizer->SetEvaluationPeriod(m_episodeInterval);
    optimizer->SetRoleChangeThreshold(m_roleChangeThreshold);
    if (!m_modelFile.empty()) {
      optimizer->LoadQTable(m_modelFile);
      NS_LOG_INFO("Loading ICRA model once: " << m_modelFile);
    }
    m_configuredOptimizer = optimizer;
  }

  node->AggregateObject(agent);
  return agent;
}


void icraHelper::Set(std::string name, const AttributeValue& value){  
  m_agentFactory.Set(name, value);
}



int64_t icraHelper::AssignStream(NodeContainer c, int64_t stream){
  int64_t currentStream = stream;
  Ptr<Node> node;
  for(auto i = c.Begin() ;i != c.End(); ++i){
    node = (*i);
    Ptr<Ipv4>ipv4 = node->GetObject<Ipv4>();
    NS_ASSERT_MSG (ipv4, "Ipv4 not installed on node");
    Ptr<Ipv4RoutingProtocol> proto = ipv4->GetRoutingProtocol ();
    NS_ASSERT_MSG (proto, "Ipv4 routing not installed on node");
    Ptr<icra::RoutingProtocol> icra = DynamicCast<icra::RoutingProtocol>(proto);
    if(icra){
      currentStream += icra -> AssignStreams(currentStream);
      continue;
    }
    Ptr<Ipv4ListRouting> list = DynamicCast<Ipv4ListRouting> (proto);

    if(list){

      int16_t priority;
      Ptr<Ipv4RoutingProtocol> listProto;
      Ptr<icra::RoutingProtocol> listicra;
      for (uint32_t i = 0; i < list->GetNRoutingProtocols (); i++) {
        listProto = list->GetRoutingProtocol (i, priority);
        listicra = DynamicCast<icra::RoutingProtocol> (listProto);
        if (listicra)
        {
          currentStream += listicra->AssignStreams (currentStream);
          break;
        }
      }
    }
  }
  currentStream += icra::QLearningWeightOptimizer::GetInstance()->AssignStreams(currentStream);
  return (currentStream - stream);
}

void icraHelper::SetPreset(const std::string& preset)
{
  if (preset != "paper" && preset != "legacy") {
    throw std::invalid_argument("ICRA preset must be paper or legacy");
  }
  m_preset = preset;
  const bool paper = preset == "paper";
  m_alpha = paper ? 0.8 : 0.1;
  m_lambda = 0.5;
  m_epsilon = 0.1;
  m_episodeInterval = Seconds(paper ? 2.0 : 10.0);
  m_lhtpThreshold = paper ? 0.1 : 2.0;
  m_roleChangeThreshold = paper ? 2 : 3;
  m_configuredOptimizer = nullptr;
}

void icraHelper::SetMaxRange(double meters)
{
  if (!std::isfinite(meters) || meters <= 0.0) {
    throw std::invalid_argument("ICRA range must be finite positive meters");
  }
  m_range = meters;
  m_configuredOptimizer = nullptr;
}

void icraHelper::SetGroundStationDelay(Time delay)
{
  if (delay.IsNegative()) {
    throw std::invalid_argument("ICRA ideal GS delay cannot be negative");
  }
  m_groundStationDelay = delay;
}

void icraHelper::SetExcludedNodeId(uint16_t nodeId)
{
  m_excludedNodeId = nodeId;
}


void icraHelper::SetQLearningParameters(double alpha, double lambda, double epsilon)
{
  if (!std::isfinite(alpha) || !std::isfinite(lambda) || !std::isfinite(epsilon) ||
      alpha < 0 || alpha > 1 || lambda < 0 || lambda > 1 || epsilon < 0 || epsilon > 1) {
    throw std::invalid_argument("ICRA learning parameters must be finite values in [0,1]");
  }
  m_alpha = alpha;
  m_lambda = lambda;
  m_epsilon = epsilon;
  m_configuredOptimizer = nullptr;
}

void icraHelper::SetQLearningModelFile(const std::string& filename)
{
  m_modelFile = filename;
  m_configuredOptimizer = nullptr;
}

} // namespace ns3
