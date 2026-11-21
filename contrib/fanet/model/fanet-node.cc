/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "fanet-node.h"

#include "ns3/log.h"
#include "ns3/uinteger.h"
#include "ns3/double.h"
#include "ns3/boolean.h"
#include "ns3/enum.h"
#include "ns3/mobility-module.h"
#include "ns3/simulator.h"
// #include "ns3/application.h" /* Add This Line at 2025-01-02*/
#include <cmath>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("ClusterNode");
NS_OBJECT_ENSURE_REGISTERED(ClusterNode);

TypeId ClusterNode::GetTypeId()
{
  static TypeId tid = TypeId("ns3::ClusterNode")
    .SetParent<Object>()
    .SetGroupName("fanet")
    .AddConstructor<ClusterNode>()
    .AddTraceSource("ClusterState", "Role or cluster ID changed; individual setter transitions",
                    MakeTraceSourceAccessor(&ClusterNode::m_clusterState),
                    "ns3::ClusterNode::ClusterStateCallback")
    .AddAttribute("FitnessMode", "Homogeneous four-factor or heterogeneous compute fitness",
                    EnumValue(HETEROGENEOUS_COMPUTE),
                    MakeEnumAccessor(&ClusterNode::SetFitnessMode, &ClusterNode::GetFitnessMode),
                    MakeEnumChecker(HOMOGENEOUS, "Homogeneous",
                                    HETEROGENEOUS_COMPUTE, "HeterogeneousCompute"))
    .AddAttribute("NodeId:", "The Id of the Node", UintegerValue(0),
                    MakeUintegerAccessor (&ClusterNode::m_NodeId),
                    MakeUintegerChecker<uint16_t>())
    .AddAttribute("ClusterId:", "The Id of the cluster", UintegerValue(0),
                    MakeUintegerAccessor (&ClusterNode::m_ClusterId),
                    MakeUintegerChecker<uint16_t>())
    .AddAttribute("NetworkScale:", "UAV Nodes number", UintegerValue(0),
                    MakeUintegerAccessor (&ClusterNode::m_NodesQuantities),
                    MakeUintegerChecker<uint16_t>())
    .AddAttribute("NeighborDegree:", "The Neighbors Degree of the Node", 
                    UintegerValue(0),
                    MakeUintegerAccessor (&ClusterNode::m_NeighborDegree),
                    MakeUintegerChecker<uint16_t>())
    .AddAttribute("OriginEnergy", "The Origin Energy of the Node",
                    DoubleValue (100.0),
                    MakeDoubleAccessor(&ClusterNode::m_OriginEnergy),
                    MakeDoubleChecker<double>())
    .AddAttribute("ResidualEnergy", "The Residual Energy of the Node",
                    DoubleValue (100.0),
                    MakeDoubleAccessor(&ClusterNode::m_ResidualEnergy),
                    MakeDoubleChecker<double>())
    .AddAttribute("FitnessValue" , "The Cluster Head Fitness Value of the Node",
                    DoubleValue(0.0),
                    MakeDoubleAccessor(&ClusterNode::m_FitnessValue),
                    MakeDoubleChecker<double>())
    .AddAttribute("Velocity", "The Velocity of the Node",
                    VectorValue(Vector(0.0, 0.0, 0.0)),
                    MakeVectorAccessor(&ClusterNode::m_velocity),
                    MakeVectorChecker())
    .AddAttribute("Position", "The Position of the Node",
                    VectorValue(Vector(0.0, 0.0, 0.0)),
                    MakeVectorAccessor(&ClusterNode::m_position),
                    MakeVectorChecker());
  // Attribute Neihbor could add by the function AddNeighbor
  return tid;
}


ClusterNode::ClusterNode()
  : m_ClusterId(0xFFFF),
    m_NodesQuantities(NodeList::GetNNodes()),
    m_NeighborDegree(0),
    m_OriginEnergy(100.0),
    m_ResidualEnergy(100.0),  // 初始化残余能量
    m_FitnessValue(1.0),      // 初始fitness设为1.0（满能量状态）
    m_role(Idle),
    m_LastUpdateTime(0.0),    // 初始化最后更新时间
    m_computeCapacity(1.0),   // 默认 1 GFLOPS (MCU 级别)
    m_taskQueueCapacity(10),  // 默认队列容量
    m_currentQueueLen(0),     // 队列初始为空
    m_currentLoad(0.0),       // 初始负载为 0
    m_avgProcessingDelay(0.0),// 初始延迟为 0
    m_maxRange(500.0),        // 默认与 bcr-routing m_maxRange 一致，scratch 注入 per-type 值
    m_lhtpUseMaxRange(false),
    m_fitnessMode(HETEROGENEOUS_COMPUTE)
{
  m_Neighbors = nullptr; // 等设置了 NodeId 后再创建neighbor
}

ClusterNode:: ~ClusterNode(){};

void     ClusterNode::updateResidualEnergy(double energy)   { m_ResidualEnergy = energy;}
void     ClusterNode::SetOriginEnergy(double energy)        { m_OriginEnergy = energy;}
double   ClusterNode::GetResidualEnergy() const             { return m_ResidualEnergy;}
double   ClusterNode::GetOriginEnergy() const               { return m_OriginEnergy;}

void     ClusterNode::SetVelocity(Vector vel)               { m_velocity = vel;}
Vector   ClusterNode::GetVelocity() const                   { return m_velocity;}

void     ClusterNode::SetPosition (Vector pos)              { m_position = pos;}
Vector   ClusterNode::GetPosition() const                   { return m_position;}

void ClusterNode::SetNodeId(uint16_t NodeId)
{ 
  m_NodeId = NodeId; 
  if (!m_Neighbors) {
    Ptr<Neighbors> neighbor = CreateObject<Neighbors>(NodeId);
    this->AggregateObject(neighbor);
    m_Neighbors = this->GetObject<Neighbors>();
  }
}

uint16_t ClusterNode::GetNodeId() const                    { return m_NodeId; }

uint16_t ClusterNode::GetClusterHeadId()
{
  if(m_role == Head){
    return m_NodeId;
  }
  else {
    if(m_role == Member) {
      for(auto neighbor = m_Neighbors->GetAll().begin(); neighbor != m_Neighbors->GetAll().end() ; ++neighbor) {
        if(neighbor->second.Role == Head && neighbor->second.clusterId){
          return neighbor->first;
        }
      }
      // NS_LOG_DEBUG("Node["<<m_NodeId<<"], may lost conact with clusterHead");
      return 0xFFFF;
    }
    else{
      return 0xFFFF;
    }
  }
}

double GetEuclideanDistance(Vector A,Vector B)
{
  return std::sqrt(std::pow((A.x-B.x),2)+ std::pow((A.y-B.y),2)+std::pow((A.z-B.z),2));
}

/**
 * \brief 计算两节点间的链路保持时间(LHTP)
 * 用于ABC算法中替代速度相似性
 * \param pos1 节点1位置
 * \param vel1 节点1速度
 * \param pos2 节点2位置
 * \param vel2 节点2速度
 * \param range 通信范围
 * \return LHTP值(秒)，限制在[0, 200]范围内
 */
double CalculateLHTPassistant(Vector pos1, Vector vel1, Vector pos2, Vector vel2, double range)
{
  const double MAX_LHTP_VALUE = 200.0;  // 仿真时长

  Vector dPos = pos2 - pos1;
  Vector dVel = vel2 - vel1;

  auto dot = [](Vector u, Vector v) {
    return u.x * v.x + u.y * v.y + u.z * v.z;
  };

  double d0 = dot(dPos, dPos);
  double r2 = range * range;

  if (d0 > r2) { return 0.0; }  // 已经超出通信范围

  double a = dot(dVel, dVel);
  double b = 2 * dot(dPos, dVel);
  double c = d0 - r2;

  if (a == 0.0) { return MAX_LHTP_VALUE; }  // 相对静止

  double discriminant = b*b - 4*a*c;
  if (discriminant < 0) { return MAX_LHTP_VALUE; }  // 永不超出范围

  double sqrtD = std::sqrt(discriminant);
  double t1 = (-b - sqrtD) / (2*a);
  double t2 = (-b + sqrtD) / (2*a);

  double result = MAX_LHTP_VALUE;
  if (t1 > 0) result = t1;
  if (t2 > 0 && t2 < result) result = t2;

  if (result == MAX_LHTP_VALUE)
    return 0.0;  // 两个根都不是正数

  return std::min(result, MAX_LHTP_VALUE);  // 限制在200以内
}


const char* ClusterNode::FitnessModeName(FitnessMode mode)
{
  return mode == HOMOGENEOUS ? "homogeneous" : "heterogeneous-compute";
}

bool ClusterNode::ParseFitnessMode(const std::string& name, FitnessMode& mode)
{
  if (name == "homogeneous") { mode = HOMOGENEOUS; return true; }
  if (name == "heterogeneous-compute") { mode = HETEROGENEOUS_COMPUTE; return true; }
  return false;
}

double ClusterNode::ComputeFitness(float WeightLHTP, float WeightEnergyLevel, float WeightDegree, float WeightDistance, float WeightCompute)
{
  const bool heterogeneousCompute = m_fitnessMode == HETEROGENEOUS_COMPUTE;
  if (!heterogeneousCompute) { WeightCompute = 0.0f; }
  const double MAX_LHTP_VALUE = 200.0;  // 仿真时长
  const double REF_LHTP_TIME = 50.0;     // LHTP归一化参考值
  const double COMM_RANGE = m_lhtpUseMaxRange ? m_maxRange : 250.0; // C13：可切换到场景范围

  double result = 0.0;

  // 更新邻接信息
  if(Simulator::Now().GetSeconds() > 1.0) { UpdateNeighborDegree(); }

  // 无邻接或无能量时返回0
  if (m_NeighborDegree == 0 || m_ResidualEnergy <= 0) {  return 0.0; }

  uint32_t validNeighbors = m_Neighbors->GetAll().size();

  // ========== 1. Compute the LHTP factor ==========
  // LHTP衡量与邻接的链路稳定性，值越大链路保持时间越长，簇头越稳定
  double LHTPSum = 0.0;

  for (const auto &neighbor : m_Neighbors->GetAll())
  {
    if (neighbor.second.NodeId != m_NodeId)
    {
      double lhtp = CalculateLHTPassistant(m_position, m_velocity,
                                           neighbor.second.Position, neighbor.second.Velocity,
                                           COMM_RANGE);
      LHTPSum += lhtp;
    }
  }

  double lhtp_factor = 0.0;
  if (validNeighbors > 0)
  {
    double avgLHTP = LHTPSum / validNeighbors;

    // LHTP特殊值处理：如果接近200（无穷大情况），取1.0
    if (avgLHTP >= MAX_LHTP_VALUE * 0.99)
    {
      lhtp_factor = 1.0;
    }
    else
    {
      // 正常归一化：avgLHTP / REF_LHTP_TIME
      // 当avgLHTP=50时，lhtp_factor=1.0（饱和）
      // 当avgLHTP=0时，lhtp_factor=0.0
      lhtp_factor = std::min(1.0, avgLHTP / REF_LHTP_TIME);
    }
  }

  // ========== 2. 计算Energy因子 ==========
  double energy_factor = m_ResidualEnergy / m_OriginEnergy;
  energy_factor = std::min(1.0, std::max(0.0, energy_factor));

  // ========== 3. 计算Degree因子 ==========
  double degree_factor = 0.0;
  if (!heterogeneousCompute)
  {
    // Use all valid radio neighbors; no extra 250 m density filter.
    if (m_NodesQuantities > 1)
      degree_factor = static_cast<double>(validNeighbors) / (m_NodesQuantities - 1);
  }
  else
  {
    // [异构对称化] 用"对称服务半径内的邻居数"代替"全网邻居数"。
    // 异构 FANET 下 COMMAND TX=600m 能听到 60+ 邻居但反向链路是 SWARM
    // 的 ~190m 瓶颈——其"实际可双向服务的邻居"反而少。SWARM 在密集
    // SWARM 区域 250m 内 ~25 个对称邻居，做 CH 时 unicast 双向都通。
    // 用 SWARM 级 TX 半径作为统一对称服务标准，奖励"局部密集中心"，
    // 让 SWARM 在密集区也能竞选 CH。
    static constexpr double SYMMETRIC_RADIUS = 250.0;
    static constexpr double SYMMETRIC_R2 = SYMMETRIC_RADIUS * SYMMETRIC_RADIUS;
    static constexpr double TARGET_CLUSTER_SIZE = 10.0;  // 期望簇规模 = N/K = 200/20
    uint32_t symNeighbors = 0;
    for (const auto& kv : m_Neighbors->GetAll()) {
      if (kv.second.NodeId == m_NodeId) continue;
      double dx = kv.second.Position.x - m_position.x;
      double dy = kv.second.Position.y - m_position.y;
      double dz = kv.second.Position.z - m_position.z;
      if (dx * dx + dy * dy + dz * dz <= SYMMETRIC_R2) symNeighbors++;
    }
    degree_factor = static_cast<double>(symNeighbors) / TARGET_CLUSTER_SIZE;
  }
  degree_factor = std::min(1.0, std::max(0.0, degree_factor));

  // ========== 4. 计算Distance因子 ==========
  // 与邻接距离越近，通信延迟越小，簇内连接越紧密
  double PositionSimularity = 0.0;
  for (const auto &neighbor : m_Neighbors->GetAll())
  {
    if (neighbor.second.NodeId != m_NodeId)
    {
      double posSim = 1.0 / (1.0 + GetEuclideanDistance(neighbor.second.Position, m_position));
      PositionSimularity += posSim;
    }
  }

  double distance_factor = 0.0;
  if (validNeighbors > 0)
  {
    distance_factor = PositionSimularity / validNeighbors;
  }
  distance_factor = std::min(1.0, std::max(0.0, distance_factor));

  // ========== 5. 计算Compute因子（新增）==========
  // 计算能力越强，可处理更多任务；负载越低，可用计算资源越多
  double compute_factor = 0.0;
  if (WeightCompute > 0.0f)
  {
    // 找邻居中的最大计算能力作为归一化基准
    double maxCompute = m_computeCapacity;
    const auto& neighbors = m_Neighbors->GetAll();
    for (const auto& kv : neighbors)
    {
      if (m_Neighbors->HasUsableComputeState(kv.first) && kv.second.computeCapacity > maxCompute)
        maxCompute = kv.second.computeCapacity;
    }

    // [异构对称化] 计算能力和负载综合，但加 0.3 基底避免 SWARM (0.5GF/s)
    // 因 compute 单维度被"硬否决"。SWARM/RELAY 比 = 0.5/20 = 0.025 太悬殊，
    // 没有基底会让 fitness 公式退化成"算力优先"，SWARM 永远进不了 CH 候选。
    // 0.3 基底让 compute 在 fitness 里仍能区分但不再决定性。
    double computeNorm = (maxCompute > 0.0)
      ? m_computeCapacity / maxCompute
      : 0.0;
    double loadFactor = 1.0 - m_currentLoad;
    compute_factor = 0.3 + 0.7 * std::min(1.0, computeNorm * loadFactor);
  }

  // ========== 6. 组合Fitness ==========
  // 权重应该满足 WeightLHTP + WeightEnergyLevel + WeightDegree + WeightDistance + WeightCompute = 1.0
  double weightSum = WeightLHTP + WeightEnergyLevel + WeightDegree + WeightDistance + WeightCompute;

  if (weightSum <= 0)
  {
    // Preserve the heterogeneous fallback; homogeneous has only four factors.
    const double equalWeight = heterogeneousCompute ? 0.2 : 0.25;
    result = equalWeight * lhtp_factor
           + equalWeight * energy_factor
           + equalWeight * degree_factor
           + equalWeight * distance_factor
           + (heterogeneousCompute ? equalWeight * compute_factor : 0.0);
  }
  else if (std::abs(weightSum - 1.0) > 0.01)
  {
    // 权重和不为1，需要归一化
    double w_lhtp = WeightLHTP / weightSum;
    double w_eng = WeightEnergyLevel / weightSum;
    double w_deg = WeightDegree / weightSum;
    double w_dis = WeightDistance / weightSum;
    double w_comp = WeightCompute / weightSum;

    result = w_lhtp * lhtp_factor
           + w_eng * energy_factor
           + w_deg * degree_factor
           + w_dis * distance_factor
           + w_comp * compute_factor;
  }
  else
  {
    // 权重正常，直接使用
    result = WeightLHTP * lhtp_factor
           + WeightEnergyLevel * energy_factor
           + WeightDegree * degree_factor
           + WeightDistance * distance_factor
           + WeightCompute * compute_factor;
  }

  // 最终确保result在[0,1]范围内
  result = std::min(1.0, std::max(0.0, result));

  NS_LOG_DEBUG("NodeId: " << m_NodeId << "\tFitness = " << result
               << " (LHTP:" << lhtp_factor << " Energy:" << energy_factor
               << " Degree:" << degree_factor << " Distance:" << distance_factor
               << " Compute:" << compute_factor << ") at " << Simulator::Now().GetSeconds() << "s");

  return result;
}


void ClusterNode::PrintNodeInfo()
{
  NS_LOG_INFO("Node Id: " << m_NodeId);
  NS_LOG_INFO("Residual Energy: " << m_ResidualEnergy);
  NS_LOG_INFO("Velocity: ("<< m_velocity.x<<", "<<m_velocity.y<<", "<<m_velocity.z<<")");
  NS_LOG_INFO("Neighbors: ");
  for (auto neighbor :m_Neighbors->GetAll())
  {
    auto info = neighbor.second;
    NS_LOG_INFO("-  Neighbor NodeId: " << info.NodeId);
    NS_LOG_INFO("   Energy: " << info.ResEnergy);
    NS_LOG_INFO("   Position: ("<<info.Position.x<<", "<<info.Position.y<<", "<<info.Position.z<<")");
    NS_LOG_INFO("   Velocity: ("<<info.Velocity.x<<", "<<info.Velocity.y<<", "<<info.Velocity.z<<")");
  }
}

// ========== 计算能力相关方法实现（新增） ==========

void ClusterNode::SetComputeCapacity(double gflops)
{
  m_computeCapacity = gflops;
}

double ClusterNode::GetComputeCapacity() const
{
  return m_computeCapacity;
}

void ClusterNode::SetTaskQueueCapacity(uint32_t cap)
{
  m_taskQueueCapacity = cap;
}

uint32_t ClusterNode::GetTaskQueueCapacity() const
{
  return m_taskQueueCapacity;
}

bool ClusterNode::SetTaskQueueSnapshot(uint32_t waiting, uint32_t capacity)
{
  if (capacity == 0 || waiting > capacity) return false;
  m_currentQueueLen = waiting;
  m_taskQueueCapacity = capacity;
  m_currentLoad = static_cast<double>(waiting) / capacity;
  return true;
}

void ClusterNode::EnqueueTask()
{
  if (m_currentQueueLen < m_taskQueueCapacity)
  {
    m_currentQueueLen++;
  }
  m_currentLoad = (m_taskQueueCapacity > 0)
    ? static_cast<double>(m_currentQueueLen) / m_taskQueueCapacity
    : 1.0;
}

void ClusterNode::DequeueTask(double processingTime)
{
  if (m_currentQueueLen > 0)
  {
    m_currentQueueLen--;
  }
  m_currentLoad = (m_taskQueueCapacity > 0)
    ? static_cast<double>(m_currentQueueLen) / m_taskQueueCapacity
    : 0.0;

  // 移动平均延迟 (alpha = 0.3 为新值权重, 0.7 为旧值权重)
  m_avgProcessingDelay = 0.7 * m_avgProcessingDelay + 0.3 * processingTime;
}

double ClusterNode::GetCurrentLoad() const
{
  return m_currentLoad;
}

double ClusterNode::GetAvailableCompute() const
{
  return m_computeCapacity * (1.0 - m_currentLoad);
}

double ClusterNode::ComputeServiceScore() const
{
  if (m_computeCapacity <= 0.0)
    return 0.0;  // MCU 级节点，无计算服务能力

  // 三个维度综合评分
  double capScore    = std::min(m_computeCapacity / 100.0, 1.0); // 100 GFLOPS 为当一上限
  double loadScore   = 1.0 - m_currentLoad;                      // 空闲程度
  double energyScore = (m_OriginEnergy > 0)
    ? m_ResidualEnergy / m_OriginEnergy
    : 0.0;

  return 0.4 * capScore + 0.4 * loadScore + 0.2 * energyScore;
}

} // namespace ns3
