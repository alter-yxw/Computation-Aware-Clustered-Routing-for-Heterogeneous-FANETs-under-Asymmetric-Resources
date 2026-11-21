/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#define NS_LOG_APPEND_CONTEXT                                             \
  if(m_nodeId!=0xFFFF)                                                              \
  {                                                                       \
    std::clog<<"[node " << (uint16_t)m_nodeId<<"]";     \
  }

#include "cafbcr-fuzzy-hello.h"
#include "ns3/fanet-neighbor.h"
#include "ns3/fanet-node.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/node-list.h"
#include "ns3/mobility-model.h"
#include "ns3/energy-module.h"
#include <cmath>
#include <algorithm>

namespace ns3 {
namespace cafbcr {

NS_LOG_COMPONENT_DEFINE("CafbcrFuzzyHelloManager");
NS_OBJECT_ENSURE_REGISTERED(FuzzyHelloManager);

TypeId FuzzyHelloManager::GetTypeId()
{
  static TypeId tid = TypeId("ns3::cafbcr::FuzzyHelloManager")
    .SetParent<Object>()
    .SetGroupName("cafbcr")
    .AddConstructor<FuzzyHelloManager>();
  return tid;
}

FuzzyHelloManager::FuzzyHelloManager()
  : m_nodeId(0xFFFF),
    m_neighbors(nullptr),
    m_maxRange(500.0),
    m_maxNeighbors(80),
    m_initialized(false)
{
  NS_LOG_FUNCTION(this);
}

FuzzyHelloManager::~FuzzyHelloManager()
{
  NS_LOG_FUNCTION(this);
}

void FuzzyHelloManager::InitializeFuzzyEngine()
{
  NS_LOG_FUNCTION(this);

  if (m_initialized) { return; }

  InitializeInputVariables();
  InitializeOutputVariables();
  InitializeFuzzyRules();

  m_initialized = true;
  NS_LOG_INFO("FuzzyHelloManager initialized for node " << m_nodeId);
}

void FuzzyHelloManager::InitializeInputVariables()
{
  NS_LOG_FUNCTION(this);

  // 输入变量1: 平均相对速度 (m/s)
  FuzzyVariable relativeSpeed("relativeSpeed", 0.0, 100.0, INPUT_VARIABLE);
  relativeSpeed.AddFuzzySet(FuzzySet("LOW", std::make_shared<TrapezoidalMF>(0.0, 0.0, 5.0, 15.0)));
  relativeSpeed.AddFuzzySet(FuzzySet("MEDIUM", std::make_shared<TriangularMF>(10.0, 30.0, 50.0)));
  relativeSpeed.AddFuzzySet(FuzzySet("HIGH", std::make_shared<TrapezoidalMF>(40.0, 60.0, 100.0, 100.0)));
  m_fuzzyEngine.AddInputVariable(relativeSpeed);

  // 输入变量2: 归一化邻居密度 [0, 1]
  FuzzyVariable neighborDensity("neighborDensity", 0.0, 1.0, INPUT_VARIABLE);
  neighborDensity.AddFuzzySet(FuzzySet("LOW", std::make_shared<TrapezoidalMF>(0.0, 0.0, 0.1, 0.25)));
  neighborDensity.AddFuzzySet(FuzzySet("MEDIUM", std::make_shared<TriangularMF>(0.15, 0.4, 0.65)));
  neighborDensity.AddFuzzySet(FuzzySet("HIGH", std::make_shared<TrapezoidalMF>(0.55, 0.75, 1.0, 1.0)));
  m_fuzzyEngine.AddInputVariable(neighborDensity);

  // 输入变量3: 链路稳定性 (预测生存时间, 秒)
  FuzzyVariable linkStability("linkStability", 0.0, 10.0, INPUT_VARIABLE);
  linkStability.AddFuzzySet(FuzzySet("UNSTABLE", std::make_shared<TrapezoidalMF>(0.0, 0.0, 1.0, 2.0)));
  linkStability.AddFuzzySet(FuzzySet("MODERATE", std::make_shared<TriangularMF>(1.5, 3.0, 5.0)));
  linkStability.AddFuzzySet(FuzzySet("STABLE", std::make_shared<TrapezoidalMF>(4.0, 6.0, 10.0, 10.0)));
  m_fuzzyEngine.AddInputVariable(linkStability);

  NS_LOG_INFO("Initialized 3 input variables");
}

void FuzzyHelloManager::InitializeOutputVariables()
{
  NS_LOG_FUNCTION(this);

  // 输出变量: Hello间隔 (秒)
  FuzzyVariable helloInterval("helloInterval", 0.25, 2.0, OUTPUT_VARIABLE);
  helloInterval.AddFuzzySet(FuzzySet("VERY_SHORT", std::make_shared<TrapezoidalMF>(0.25, 0.25, 0.35, 0.45)));
  helloInterval.AddFuzzySet(FuzzySet("SHORT", std::make_shared<TriangularMF>(0.3, 0.5, 0.7)));
  helloInterval.AddFuzzySet(FuzzySet("MEDIUM", std::make_shared<TriangularMF>(0.6, 0.8, 1.0)));
  helloInterval.AddFuzzySet(FuzzySet("LONG", std::make_shared<TriangularMF>(0.9, 1.2, 1.5)));
  helloInterval.AddFuzzySet(FuzzySet("VERY_LONG", std::make_shared<TrapezoidalMF>(1.4, 1.7, 2.0, 2.0)));
  m_fuzzyEngine.AddOutputVariable(helloInterval);

  NS_LOG_INFO("Initialized output variable: helloInterval");
}

void FuzzyHelloManager::InitializeFuzzyRules()
{
  NS_LOG_FUNCTION(this);

  // ==================== 第一优先级：高动态场景（PDR关键） ====================

  // 规则1: 高速 + 不稳定链路 → 极短间隔（最高优先级：快速拓扑变化）
  FuzzyRule rule1;
  rule1.AddAntecedent("relativeSpeed", "HIGH");
  rule1.AddAntecedent("linkStability", "UNSTABLE");
  rule1.AddConsequent("helloInterval", "VERY_SHORT");
  m_fuzzyEngine.AddRule(rule1);

  // 规则2: 中等速度 + 不稳定链路 → 短间隔（需快速调整拓扑）
  FuzzyRule rule2;
  rule2.AddAntecedent("relativeSpeed", "MEDIUM");
  rule2.AddAntecedent("linkStability", "UNSTABLE");
  rule2.AddConsequent("helloInterval", "SHORT");
  m_fuzzyEngine.AddRule(rule2);

  // 规则3: 低速但链路不稳定 → 短间隔（震荡/障碍物遮挡场景）
  FuzzyRule rule3;
  rule3.AddAntecedent("relativeSpeed", "LOW");
  rule3.AddAntecedent("linkStability", "UNSTABLE");
  rule3.AddConsequent("helloInterval", "SHORT");
  m_fuzzyEngine.AddRule(rule3);

  // ==================== 第二优先级：低密度场景（邻居发现关键） ====================

  // 规则4: 低密度 + 不稳定链路 → 短间隔（积极发现和维护邻居）
  FuzzyRule rule4;
  rule4.AddAntecedent("neighborDensity", "LOW");
  rule4.AddAntecedent("linkStability", "UNSTABLE");
  rule4.AddConsequent("helloInterval", "SHORT");
  m_fuzzyEngine.AddRule(rule4);

  // 规则5: 低密度 + 高速移动 → 短间隔（快速发现新邻居）
  FuzzyRule rule5;
  rule5.AddAntecedent("neighborDensity", "LOW");
  rule5.AddAntecedent("relativeSpeed", "HIGH");
  rule5.AddConsequent("helloInterval", "SHORT");
  m_fuzzyEngine.AddRule(rule5);

  // 规则6: 低密度 + 中等稳定性 → 中等间隔（平衡策略）
  FuzzyRule rule6;
  rule6.AddAntecedent("neighborDensity", "LOW");
  rule6.AddAntecedent("linkStability", "MODERATE");
  rule6.AddConsequent("helloInterval", "MEDIUM");
  m_fuzzyEngine.AddRule(rule6);

  // 规则7: 低密度 + 稳定链路 → 长间隔（稀疏但稳定网络）
  FuzzyRule rule7;
  rule7.AddAntecedent("neighborDensity", "LOW");
  rule7.AddAntecedent("linkStability", "STABLE");
  rule7.AddConsequent("helloInterval", "LONG");
  m_fuzzyEngine.AddRule(rule7);

  // ==================== 第三优先级：高密度场景（冲突避免+动态平衡） ====================

  // 规则8: 高密度 + 高速 + 不稳定 → 短间隔（极端动态场景，PDR优先）
  FuzzyRule rule8;
  rule8.AddAntecedent("neighborDensity", "HIGH");
  rule8.AddAntecedent("relativeSpeed", "HIGH");
  rule8.AddAntecedent("linkStability", "UNSTABLE");
  rule8.AddConsequent("helloInterval", "SHORT");
  m_fuzzyEngine.AddRule(rule8);

  // 规则9: 高密度 + 高速 + 稳定 → 中等间隔（编队飞行，平衡冲突和维护）
  FuzzyRule rule9;
  rule9.AddAntecedent("neighborDensity", "HIGH");
  rule9.AddAntecedent("relativeSpeed", "HIGH");
  rule9.AddAntecedent("linkStability", "STABLE");
  rule9.AddConsequent("helloInterval", "MEDIUM");
  m_fuzzyEngine.AddRule(rule9);

  // 规则10: 高密度 + 低速 + 稳定 → 长间隔（拥塞但稳定，最大程度减少开销）
  FuzzyRule rule10;
  rule10.AddAntecedent("neighborDensity", "HIGH");
  rule10.AddAntecedent("relativeSpeed", "LOW");
  rule10.AddAntecedent("linkStability", "STABLE");
  rule10.AddConsequent("helloInterval", "LONG");
  m_fuzzyEngine.AddRule(rule10);

  // 规则11: 高密度 + 不稳定链路 → 中等偏短间隔（二元规则补充）
  FuzzyRule rule11;
  rule11.AddAntecedent("neighborDensity", "HIGH");
  rule11.AddAntecedent("linkStability", "UNSTABLE");
  rule11.AddConsequent("helloInterval", "SHORT");
  m_fuzzyEngine.AddRule(rule11);

  // 规则12: 高密度 + 低速移动 → 长间隔（拥塞但慢速，减少冲突）
  FuzzyRule rule12;
  rule12.AddAntecedent("neighborDensity", "HIGH");
  rule12.AddAntecedent("relativeSpeed", "LOW");
  rule12.AddConsequent("helloInterval", "LONG");
  m_fuzzyEngine.AddRule(rule12);

  // 规则13: 高密度 + 高速 + 中等稳定性 → 短间隔（拥塞+动态，PDR优先）
  FuzzyRule rule13;
  rule13.AddAntecedent("neighborDensity", "HIGH");
  rule13.AddAntecedent("relativeSpeed", "HIGH");
  rule13.AddAntecedent("linkStability", "MODERATE");
  rule13.AddConsequent("helloInterval", "SHORT");
  m_fuzzyEngine.AddRule(rule13);

  // 规则14: 高密度 + 中等速度 + 中等稳定性 → 中等间隔（高密度标准场景）
  FuzzyRule rule14;
  rule14.AddAntecedent("neighborDensity", "HIGH");
  rule14.AddAntecedent("relativeSpeed", "MEDIUM");
  rule14.AddAntecedent("linkStability", "MODERATE");
  rule14.AddConsequent("helloInterval", "MEDIUM");
  m_fuzzyEngine.AddRule(rule14);

  // 规则15: 高密度 + 稳定链路 → 长间隔（高密度但稳定，减少冲突）
  FuzzyRule rule15;
  rule15.AddAntecedent("neighborDensity", "HIGH");
  rule15.AddAntecedent("linkStability", "STABLE");
  rule15.AddConsequent("helloInterval", "LONG");
  m_fuzzyEngine.AddRule(rule15);

  // ==================== 第四优先级：中等密度场景 ====================

  // 规则16: 中等密度 + 不稳定链路 → 短间隔（需快速更新）
  FuzzyRule rule16;
  rule16.AddAntecedent("neighborDensity", "MEDIUM");
  rule16.AddAntecedent("linkStability", "UNSTABLE");
  rule16.AddConsequent("helloInterval", "SHORT");
  m_fuzzyEngine.AddRule(rule16);

  // 规则17: 中等密度 + 中等速度 + 中等稳定性 → 中等间隔（最常见场景）
  FuzzyRule rule17;
  rule17.AddAntecedent("neighborDensity", "MEDIUM");
  rule17.AddAntecedent("relativeSpeed", "MEDIUM");
  rule17.AddAntecedent("linkStability", "MODERATE");
  rule17.AddConsequent("helloInterval", "MEDIUM");
  m_fuzzyEngine.AddRule(rule17);

  // 规则18: 中等密度 + 稳定链路 → 长间隔（中等拥塞但稳定）
  FuzzyRule rule18;
  rule18.AddAntecedent("neighborDensity", "MEDIUM");
  rule18.AddAntecedent("linkStability", "STABLE");
  rule18.AddConsequent("helloInterval", "LONG");
  m_fuzzyEngine.AddRule(rule18);

  // Rule 19: medium density + high speed + medium stability -> short interval
  FuzzyRule rule19;
  rule19.AddAntecedent("neighborDensity", "MEDIUM");
  rule19.AddAntecedent("relativeSpeed", "HIGH");
  rule19.AddAntecedent("linkStability", "MODERATE");
  rule19.AddConsequent("helloInterval", "SHORT");
  m_fuzzyEngine.AddRule(rule19);

  // 规则20: 中等密度 + 低速 + 中等稳定性 → 长间隔（补充低速场景）
  FuzzyRule rule20;
  rule20.AddAntecedent("neighborDensity", "MEDIUM");
  rule20.AddAntecedent("relativeSpeed", "LOW");
  rule20.AddAntecedent("linkStability", "MODERATE");
  rule20.AddConsequent("helloInterval", "LONG");
  m_fuzzyEngine.AddRule(rule20);

  // 规则21: 中等密度 + 高速 + 稳定 → 中等间隔（补充高速稳定场景）
  FuzzyRule rule21;
  rule21.AddAntecedent("neighborDensity", "MEDIUM");
  rule21.AddAntecedent("relativeSpeed", "HIGH");
  rule21.AddAntecedent("linkStability", "STABLE");
  rule21.AddConsequent("helloInterval", "MEDIUM");
  m_fuzzyEngine.AddRule(rule21);

  // ==================== 第五优先级：稳定场景（节能优化） ====================

  // 规则22: 低速 + 稳定链路 → 极长间隔（最稳定场景，最大程度节能）
  FuzzyRule rule22;
  rule22.AddAntecedent("relativeSpeed", "LOW");
  rule22.AddAntecedent("linkStability", "STABLE");
  rule22.AddConsequent("helloInterval", "VERY_LONG");
  m_fuzzyEngine.AddRule(rule22);

  // 规则23: 高速 + 稳定链路 → 中等间隔（编队飞行场景）
  FuzzyRule rule23;
  rule23.AddAntecedent("relativeSpeed", "HIGH");
  rule23.AddAntecedent("linkStability", "STABLE");
  rule23.AddConsequent("helloInterval", "MEDIUM");
  m_fuzzyEngine.AddRule(rule23);

  // 规则24: 中等速度 + 稳定链路 → 长间隔（平稳运动场景）
  FuzzyRule rule24;
  rule24.AddAntecedent("relativeSpeed", "MEDIUM");
  rule24.AddAntecedent("linkStability", "STABLE");
  rule24.AddConsequent("helloInterval", "LONG");
  m_fuzzyEngine.AddRule(rule24);

  // 规则25: 中等速度 + 中等稳定性 → 中等间隔（通用中等场景）
  FuzzyRule rule25;
  rule25.AddAntecedent("relativeSpeed", "MEDIUM");
  rule25.AddAntecedent("linkStability", "MODERATE");
  rule25.AddConsequent("helloInterval", "MEDIUM");
  m_fuzzyEngine.AddRule(rule25);

  NS_LOG_INFO("Initialized 25 fuzzy rules for Hello interval optimization (3 input variables, PDR-optimized)");
}

Time FuzzyHelloManager::GetOptimalHelloInterval()
{
  NS_LOG_FUNCTION(this);
  if (!m_initialized) { InitializeFuzzyEngine(); }  // 确保模糊引擎已初始化
  std::map<std::string, double> inputs;  // 准备输入数据

  // 1. 评估链路稳定性
  LinkStability stability = EvaluateLinkStability();
  inputs["relativeSpeed"] = stability.relativeVelocity;
  inputs["linkStability"] = stability.predictedLifetime;

  // 2. 邻居密度
  uint16_t neighborCount = m_neighbors ? m_neighbors->GetAll().size() : 0;
  inputs["neighborDensity"] = NormalizeNeighborCount(neighborCount);

  NS_LOG_DEBUG("Fuzzy inputs - RelSpeed: " << inputs["relativeSpeed"]
               << ", Density: " << inputs["neighborDensity"]
               << ", Stability: " << inputs["linkStability"]);

  // 执行模糊推理
  std::map<std::string, double> outputs = m_fuzzyEngine.Infer(inputs);
  double intervalValue = outputs["helloInterval"];

  // 确保间隔在有效范围内
  if (intervalValue < 0.25) intervalValue = 0.25;
  if (intervalValue > 2.0) intervalValue = 2.0;

  Time interval = Seconds(intervalValue);
  NS_LOG_DEBUG("Fuzzy-based Hello interval: " << interval.GetSeconds() << "s");

  return interval;
}


FuzzyHelloManager::LinkStability FuzzyHelloManager::EvaluateLinkStability()
{
  LinkStability stability;

  if (!m_neighbors || m_neighbors->GetAll().empty()) {
    // 没有邻居，返回默认不稳定状态
    stability.relativeVelocity = 50.0;
    stability.directionDivergence = 1.0;
    stability.distanceRisk = 1.0;
    stability.predictedLifetime = 0.5;
    return stability;
  }
  Ptr<Node> node = NodeList::GetNode(m_nodeId);  // 获取当前节点信息
  Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
  Vector MyVel = mobility->GetVelocity();

  double totalRelativeSpeed = 0.0;
  double minLifetime = 10.0;
  uint16_t validNeighbors = 0;

  // 遍历所有邻居
  for (const auto& neighbor : m_neighbors->GetAll()) {
    uint16_t neighborId = neighbor.first;
    Ptr<Node> neighborNode = NodeList::GetNode(neighborId);

    if (!neighborNode) continue;

    Ptr<MobilityModel> neighborMobility = neighborNode->GetObject<MobilityModel>();
    Vector NeibrVel = neighborMobility->GetVelocity();

    double relSpeed = CalculateRelativeVelocity(MyVel, NeibrVel);    // 计算相对速度
    totalRelativeSpeed += relSpeed;
    double lifetime = EstimateLinkLifetime(neighborId);    // 估计链路生存时间
    if (lifetime < minLifetime) {   minLifetime = lifetime;}
    validNeighbors++;
  }

  if (validNeighbors > 0) {
    stability.relativeVelocity = totalRelativeSpeed / validNeighbors;
    stability.predictedLifetime = minLifetime;
  } else {
    stability.relativeVelocity = 50.0;
    stability.predictedLifetime = 0.5;
  }

  stability.directionDivergence = CalculateDirectionDivergence();
  stability.distanceRisk = (stability.predictedLifetime < 2.0) ? 0.8 : 0.2;

  return stability;
}

double FuzzyHelloManager::CalculateRelativeVelocity(Vector v1, Vector v2)
{
  Vector RelVel = v1 - v2;
  return std::sqrt(RelVel.x * RelVel.x +  RelVel.y * RelVel.y +  RelVel.z * RelVel.z);
}

double FuzzyHelloManager::CalculateDirectionDivergence()
{
  if (!m_neighbors || m_neighbors->GetAll().size() < 2) {  return 0.5;}

  Ptr<Node> node = NodeList::GetNode(m_nodeId);
  Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
  Vector MyVel = mobility->GetVelocity();
  double mySpeed = std::sqrt(MyVel.x * MyVel.x + MyVel.y * MyVel.y + MyVel.z * MyVel.z);

  if (mySpeed < 1.0) return 0.0;

  double divergenceSum = 0.0;
  uint16_t validNeighbors = 0;

  for (const auto& neighbor : m_neighbors->GetAll()) {
    uint16_t neighborId = neighbor.first;
    Ptr<Node> neighborNode = NodeList::GetNode(neighborId);

    if (!neighborNode) continue;

    Ptr<MobilityModel> neighborMobility = neighborNode->GetObject<MobilityModel>();
    Vector NeibrVel = neighborMobility->GetVelocity();
    double neighborSpeed = std::sqrt(NeibrVel.x * NeibrVel.x + NeibrVel.y * NeibrVel.y + NeibrVel.z * NeibrVel.z);

    if (neighborSpeed < 1.0) continue;

    double dotProduct = MyVel.x * NeibrVel.x + MyVel.y * NeibrVel.y + MyVel.z * NeibrVel.z;    // 计算方向余弦（归一化点积）
    double cosSimilarity = dotProduct / (mySpeed * neighborSpeed);

    double divergence = (1.0 - cosSimilarity) / 2.0;    // 发散度 = 1 - cos(theta) 范围 [0, 2]，归一化到 [0, 1]
    divergenceSum += divergence;
    validNeighbors++;
  }

  return (validNeighbors > 0) ? (divergenceSum / validNeighbors) : 0.5;
}

double FuzzyHelloManager::EstimateLinkLifetime(uint16_t neighborId)
{
  if (m_nodeId >= NodeList::GetNNodes() || neighborId >= NodeList::GetNNodes()) {
    return 1.0;
  }
  Ptr<Node> node = NodeList::GetNode(m_nodeId);
  Ptr<Node> neighborNode = NodeList::GetNode(neighborId);

  if (!node || !neighborNode) {  return 1.0; }

  Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
  Ptr<MobilityModel> neighborMobility = neighborNode->GetObject<MobilityModel>();
  if (!mobility || !neighborMobility) { return 1.0; }

  Vector myPosition = mobility->GetPosition();
  Vector MyVel = mobility->GetVelocity();
  Vector neighborPosition = neighborMobility->GetPosition();
  Vector nbrVel = neighborMobility->GetVelocity();

  // 相对位置和速度
  Vector RelPos = neighborPosition - myPosition;
  Vector RelaVel = nbrVel - MyVel;

  const double distanceSquared = RelPos.x * RelPos.x + RelPos.y * RelPos.y + RelPos.z * RelPos.z;
  const double speedSquared = RelaVel.x * RelaVel.x + RelaVel.y * RelaVel.y + RelaVel.z * RelaVel.z;
  double dotProduct = RelPos.x * RelaVel.x +  RelPos.y * RelaVel.y + RelPos.z * RelaVel.z;
  const double rangeSquared = m_maxRange * m_maxRange;
  if (!(m_maxRange > 0.0) || !std::isfinite(rangeSquared) ||
      !std::isfinite(distanceSquared) || !std::isfinite(speedSquared) ||
      !std::isfinite(dotProduct)) {
    return 1.0; // 与缺失节点／运动信息时相同的有限回退值。
  }
  if (distanceSquared > rangeSquared) { return 0.0; } // 当前已在名义范围外。
  if (speedSquared == 0.0) { return 10.0; }

  // r = 邻居位置 - 本机位置，v = 邻居速度 - 本机速度。
  // r·v > 0 表示远离。解 |r + v*t|² = R² 的离开根，
  // 同时覆盖靠近后穿过对侧边界、切向运动及恰在边界上的情况。
  const double margin = rangeSquared - distanceSquared;
  const double root = std::sqrt(dotProduct * dotProduct + speedSquared * margin);
  const double timeToBreak = dotProduct > 0.0
    ? margin / (dotProduct + root) // 避免远离且接近边界时相减损失精度。
    : (root - dotProduct) / speedSquared;
  return std::min(10.0, std::max(0.0, timeToBreak));
}

Vector FuzzyHelloManager::PredictPositionWithBeacons(uint16_t nodeId, Time targetTime)
{
  if (m_beaconHistory.find(nodeId) == m_beaconHistory.end() || m_beaconHistory[nodeId].empty()) {
    return Vector(0, 0, 0);
  }

  const auto& recent = m_beaconHistory[nodeId].back();
  Time predTime = targetTime - recent.timestamp;

  Vector predictedPos;
  predictedPos.x = recent.position.x + recent.velocity.x * predTime.GetSeconds();
  predictedPos.y = recent.position.y + recent.velocity.y * predTime.GetSeconds();
  predictedPos.z = recent.position.z + recent.velocity.z * predTime.GetSeconds();

  return predictedPos;
}

void FuzzyHelloManager::ProcessHelloInfo(uint16_t senderId, Vector position, Vector velocity, Time timestamp)
{
  BeaconCache cache;
  cache.position = position;
  cache.velocity = velocity;
  cache.timestamp = timestamp;

  m_beaconHistory[senderId].push_back(cache);

  // 保留最近10条记录
  if (m_beaconHistory[senderId].size() > 10) {  m_beaconHistory[senderId].pop_front(); }
}

double FuzzyHelloManager::NormalizeNeighborCount(uint16_t neighborCount)
{
  if (neighborCount >= m_maxNeighbors) {  return 1.0; }
  return static_cast<double>(neighborCount) / m_maxNeighbors;
}

double FuzzyHelloManager::GetEnergyRatio()
{
  Ptr<Node> node = NodeList::GetNode(m_nodeId);
  Ptr<ClusterNode> clusterNode = node->GetObject<ClusterNode>();

  if (!clusterNode) {   return 0.5;  }// 默认中等能量

  double residualEnergy = clusterNode->GetResidualEnergy();
  double originEnergy = clusterNode->GetOriginEnergy();

  if (originEnergy <= 0) {   return 0.5;}

  return residualEnergy / originEnergy;
}

} // namespace cafbcr
} // namespace ns3
