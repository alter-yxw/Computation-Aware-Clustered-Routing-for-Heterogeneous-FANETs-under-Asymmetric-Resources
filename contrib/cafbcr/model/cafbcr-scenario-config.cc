/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "cafbcr-scenario-config.h"
#include "ns3/log.h"
#include <cmath>

namespace ns3 {
namespace cafbcr {

NS_LOG_COMPONENT_DEFINE("CafbcrScenarioConfig");
NS_OBJECT_ENSURE_REGISTERED(ScenarioConfig);

// ========== UavTypeConfig 实现 ==========

std::string UavTypeConfig::GetNodeTypeString() const
{
  switch (nodeType) {
    case COMMAND:
      return "COMMAND";
    case RELAY:
      return "RELAY";
    case TASK:
      return "TASK";
    case SWARM:
      return "SWARM";
    default:
      return "UNKNOWN";
  }
}

UavTypeConfig UavTypeConfig::CreateCommandNode()
{
  UavTypeConfig config;
  config.nodeType = COMMAND;
  config.name = "COMMAND";

  // 计算能力 - 最强
  config.computeCapacity = 80.0;           // GFLOP/s
  config.taskQueueCapacity = 100;          // 增加队列容量，避免计算任务和UDP数据包竞争丢弃

  // 能量
  config.batteryCapacity = 50000.0;        // 焦耳
  config.idlePowerConsumption = 10.0;      // 瓦
  config.transmitPowerPerGhz = 2.0;
  config.movementPower = 50.0;             // 瓦

  // 通信 - 最强通信能力（卫星链接）
  config.maxCommunicationRange = 1500.0;   // 1.5km 通信范围最大
  config.dataRate = 300e6;                 // 300 Mbps (高速链接，如卫星)

  // 应用 - 既生成任务又处理任务（大规模场景调优）
  config.generateTasks = true;
  config.processTasks = true;
  config.taskGenerationInterval = 10.0;   // 增加为10秒（大规模节点时降低任务频率）
  config.minTaskWorkload = 50.0;
  config.maxTaskWorkload = 200.0;
  config.minTaskDataSize = 1024;
  config.maxTaskDataSize = 1400;          // 不超过 MTU(1500)，避免 IP 分片重组缓存堆积
  config.minTaskDeadline = 20.0;          // 增加截止时间容限
  config.maxTaskDeadline = 80.0;

  return config;
}

UavTypeConfig UavTypeConfig::CreateRelayNode()
{
  UavTypeConfig config;
  config.nodeType = RELAY;
  config.name = "RELAY";

  // 计算能力 - 中等
  config.computeCapacity = 20.0;           // GFLOP/s
  config.taskQueueCapacity = 80;           // 增加队列容量，避免计算任务和UDP数据包竞争丢弃

  // 能量
  config.batteryCapacity = 30000.0;
  config.idlePowerConsumption = 8.0;
  config.transmitPowerPerGhz = 1.5;
  config.movementPower = 40.0;

  // 通信 - 中等通信能力（WiFi 5GHz）
  config.maxCommunicationRange = 600.0;    // 600m 中等范围
  config.dataRate = 150e6;                 // 150 Mbps (WiFi 5GHz)

  // 应用 - 既生成任务又处理任务（大规模场景调优）
  config.generateTasks = true;
  config.processTasks = true;
  config.taskGenerationInterval = 20.0;  // 增加为20秒（大规模节点）
  config.minTaskWorkload = 10.0;
  config.maxTaskWorkload = 100.0;
  config.minTaskDataSize = 512;
  config.maxTaskDataSize = 1400;         // 不超过 MTU(1500)，避免 IP 分片重组缓存堆积
  config.minTaskDeadline = 30.0;         // 增加截止时间
  config.maxTaskDeadline = 90.0;

  return config;
}

UavTypeConfig UavTypeConfig::CreateTaskNode()
{
  UavTypeConfig config;
  config.nodeType = TASK;
  config.name = "TASK";

  // 计算能力 - 低
  config.computeCapacity = 5.0;            // GFLOP/s
  config.taskQueueCapacity = 50;           // 增加队列容量，避免计算任务和UDP数据包竞争丢弃

  // 能量
  config.batteryCapacity = 20000.0;
  config.idlePowerConsumption = 5.0;
  config.transmitPowerPerGhz = 1.0;
  config.movementPower = 30.0;

  // 通信 - 较弱通信能力（WiFi 2.4GHz）
  config.maxCommunicationRange = 400.0;    // 400m 较短范围
  config.dataRate = 54e6;                  // 54 Mbps (WiFi 2.4GHz)

  // 应用 - 既生成任务又处理任务（大规模场景调优）
  config.generateTasks = true;
  config.processTasks = true;
  config.taskGenerationInterval = 30.0;  // 增加为30秒（大规模节点）
  config.minTaskWorkload = 5.0;
  config.maxTaskWorkload = 50.0;
  config.minTaskDataSize = 256;
  config.maxTaskDataSize = 1024;
  config.minTaskDeadline = 40.0;         // 增加截止时间
  config.maxTaskDeadline = 150.0;

  return config;
}

UavTypeConfig UavTypeConfig::CreateSwarmNode()
{
  UavTypeConfig config;
  config.nodeType = SWARM;
  config.name = "SWARM";

  // 计算能力 - 最弱但仍有处理能力
  config.computeCapacity = 1.0;            // GFLOP/s (改进：从0.5提升到1.0)
  config.taskQueueCapacity = 30;           // 增加队列容量，避免计算任务和UDP数据包竞争丢弃

  // 能量
  config.batteryCapacity = 15000.0;
  config.idlePowerConsumption = 3.0;
  config.transmitPowerPerGhz = 0.8;
  config.movementPower = 25.0;

  // 通信 - 最弱通信能力（BLE/ZigBee）
  config.maxCommunicationRange = 250.0;    // 250m 最短范围
  config.dataRate = 24e6;                  // 24 Mbps (BLE/ZigBee)

  // 应用 - 既生成任务也处理任务（大规模场景调优）
  config.generateTasks = true;
  config.processTasks = true;  // 支持处理
  config.taskGenerationInterval = 40.0;  // 增加为40秒（大规模节点）
  config.minTaskWorkload = 1.0;
  config.maxTaskWorkload = 20.0;
  config.minTaskDataSize = 128;
  config.maxTaskDataSize = 512;
  config.minTaskDeadline = 60.0;
  config.maxTaskDeadline = 200.0;

  return config;
}

// ========== ScenarioConfig 实现 ==========

TypeId ScenarioConfig::GetTypeId()
{
  static TypeId tid = TypeId("ns3::cafbcr::ScenarioConfig")
    .SetParent<Object>()
    .SetGroupName("Cafbcr")
    .AddConstructor<ScenarioConfig>();
  return tid;
}

ScenarioConfig::ScenarioConfig()
  : m_simulationArea(1000.0, 1000.0, 500.0),  // 1000m x 1000m x 500m
    m_simulationTime(300.0),                  // 300秒
    m_helloInterval(2.0),                     // 2秒
    m_beaconInterval(1.5),                    // 1.5秒
    m_clusteringTime(1.0),                    // 1秒
    m_maxCommRange(500.0),                    // 500米
    // 适应度权重默认等权（顺序对齐 ComputeFitness: LHTP, Energy, Degree, Distance, Compute）。
    // 实验中由 ca-fbcr-test 通过 SetFitnessWeights 显式覆盖。
    m_wLHTP(0.20),
    m_wEnergy(0.20),
    m_wDegree(0.20),
    m_wDistance(0.20),
    m_wCompute(0.20),
    m_mobilityModel("ns3::RandomWaypointMobilityModel"),
    m_maxVelocity(20.0),                       // 20 m/s
    m_minVelocity(5.0),                        // 5 m/s
    m_energyModel("ns3::EnergySourceContainer")
{
}

ScenarioConfig::~ScenarioConfig()
{
}

void ScenarioConfig::AddNodeType(uint32_t count, const UavTypeConfig& config)
{
  NodeTypeInfo info;
  info.count = count;
  info.config = config;
  m_nodeTypes.push_back(info);

  NS_LOG_INFO("Added " << count << " " << config.name << " nodes");
}

uint32_t ScenarioConfig::GetNodeCount() const
{
  uint32_t total = 0;
  for (const auto& info : m_nodeTypes) {
    total += info.count;
  }
  return total;
}

const std::vector<UavTypeConfig>& ScenarioConfig::GetNodeConfigs() const
{
  // 返回所有配置信息
  static std::vector<UavTypeConfig> configs;
  configs.clear();
  for (const auto& info : m_nodeTypes) {
    for (uint32_t i = 0; i < info.count; ++i) {
      configs.push_back(info.config);
    }
  }
  return configs;
}

uint32_t ScenarioConfig::GetNodeCountByType(UavNodeType type) const
{
  uint32_t count = 0;
  for (const auto& info : m_nodeTypes) {
    if (info.config.nodeType == type) {
      count += info.count;
    }
  }
  return count;
}

const UavTypeConfig& ScenarioConfig::GetNodeConfigByType(UavNodeType type) const
{
  for (const auto& info : m_nodeTypes) {
    if (info.config.nodeType == type) {
      return info.config;
    }
  }
  // 如果没找到，返回第一个（这是一个简化）
  if (!m_nodeTypes.empty()) {
    return m_nodeTypes[0].config;
  }
  static UavTypeConfig dummy = UavTypeConfig::CreateSwarmNode();
  return dummy;
}

void ScenarioConfig::SetSimulationArea(Vector size)
{
  m_simulationArea = size;
}

void ScenarioConfig::SetSimulationTime(double seconds)
{
  m_simulationTime = seconds;
}

void ScenarioConfig::SetHelloInterval(double seconds)
{
  m_helloInterval = seconds;
}

void ScenarioConfig::SetBeaconInterval(double seconds)
{
  m_beaconInterval = seconds;
}

void ScenarioConfig::SetClusteringTime(double seconds)
{
  m_clusteringTime = seconds;
}

void ScenarioConfig::SetMaxCommunicationRange(double meters)
{
  m_maxCommRange = meters;
}

void ScenarioConfig::SetFitnessWeights(double wLHTP, double wEnergy,
                                       double wDegree, double wDistance, double wCompute)
{
  m_wLHTP = wLHTP;
  m_wEnergy = wEnergy;
  m_wDegree = wDegree;
  m_wDistance = wDistance;
  m_wCompute = wCompute;
}

void ScenarioConfig::GetFitnessWeights(double& wLHTP, double& wEnergy,
                                       double& wDegree, double& wDistance, double& wCompute) const
{
  wLHTP = m_wLHTP;
  wEnergy = m_wEnergy;
  wDegree = m_wDegree;
  wDistance = m_wDistance;
  wCompute = m_wCompute;
}

void ScenarioConfig::SetMobilityModel(const std::string& model)
{
  m_mobilityModel = model;
}

void ScenarioConfig::SetMaxVelocity(double mps)
{
  m_maxVelocity = mps;
}

void ScenarioConfig::SetMinVelocity(double mps)
{
  m_minVelocity = mps;
}

void ScenarioConfig::SetEnergyModel(const std::string& model)
{
  m_energyModel = model;
}

bool ScenarioConfig::Validate() const
{
  // 检查节点配置
  if (m_nodeTypes.empty()) {
    NS_LOG_ERROR("No node types configured");
    return false;
  }

  // 检查权重和
  double weightSum = m_wLHTP + m_wEnergy + m_wDegree + m_wDistance + m_wCompute;
  if (std::abs(weightSum - 1.0) > 0.01 && weightSum > 0.0) {
    NS_LOG_WARN("Fitness weights don't sum to 1.0 (sum=" << weightSum << ")");
  }

  // 检查仿真时间
  if (m_simulationTime <= 0.0) {
    NS_LOG_ERROR("Simulation time must be positive");
    return false;
  }

  // 检查仿真区域
  if (m_simulationArea.x <= 0 || m_simulationArea.y <= 0 || m_simulationArea.z <= 0) {
    NS_LOG_ERROR("Simulation area dimensions must be positive");
    return false;
  }

  NS_LOG_INFO("Scenario configuration validated successfully");
  return true;
}

void ScenarioConfig::PrintSummary() const
{
  NS_LOG_INFO("========== Scenario Configuration Summary ==========");
  NS_LOG_INFO("Total Nodes: " << GetNodeCount());

  // 按类型统计节点
  for (const auto& info : m_nodeTypes) {
    NS_LOG_INFO("  " << info.config.name << " nodes: " << info.count
               << " (compute=" << info.config.computeCapacity << " GFLOP/s)");
  }

  NS_LOG_INFO("Simulation Area: " << m_simulationArea.x << "m x "
             << m_simulationArea.y << "m x " << m_simulationArea.z << "m");
  NS_LOG_INFO("Simulation Time: " << m_simulationTime << "s");

  NS_LOG_INFO("CAFBCR Protocol Parameters:");
  NS_LOG_INFO("  Hello Interval: " << m_helloInterval << "s");
  NS_LOG_INFO("  Beacon Interval: " << m_beaconInterval << "s");
  NS_LOG_INFO("  Clustering Time: " << m_clusteringTime << "s");
  NS_LOG_INFO("  Max Range: " << m_maxCommRange << "m");

  NS_LOG_INFO("Fitness Weights:");
  NS_LOG_INFO("  LHTP: " << m_wLHTP << " Energy: " << m_wEnergy
             << " Degree: " << m_wDegree << " Distance: " << m_wDistance
             << " Compute: " << m_wCompute);

  NS_LOG_INFO("Mobility: " << m_mobilityModel
             << " (velocity: " << m_minVelocity << "-" << m_maxVelocity << " m/s)");
  NS_LOG_INFO("====================================================");
}

} // namespace cafbcr
} // namespace ns3
