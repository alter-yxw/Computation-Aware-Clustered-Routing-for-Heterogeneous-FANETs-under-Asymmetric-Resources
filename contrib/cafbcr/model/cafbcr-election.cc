/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "cafbcr-election.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/node-list.h"
#include "ns3/core-module.h"
#include "ns3/boolean.h"
#include "ns3/enum.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <set>

#define AxisX_MAX 2000
#define AxisX_MIN 0
#define AxisY_MAX 2000
#define AxisY_MIN 0
#define AxisZ_MAX 500
#define AxisZ_MIN 0

double JudgeBound(double para, double Max, double Min)
{
    if (para > Max) { para = Max;}
    if (para < Min) { para = Min;}
    return para;
}

namespace ns3 {
namespace cafbcr {

NS_LOG_COMPONENT_DEFINE("cafbcrElection");
NS_OBJECT_ENSURE_REGISTERED(cafbcrElection);

class cafbcrElection::CostScope
{
public:
  explicit CostScope(cafbcrElection& owner)
    : m_owner(owner), m_previous(owner.m_activeCost), m_enabled(owner.m_observeCost)
  {
    if (!m_enabled) return;
    m_begin = Clock::now();
    m_record.invocation = owner.m_costRecords.size() + 1;
    m_record.initializedBefore = owner.m_abcInitialized;
    m_record.selector = owner.GetInitialSelectorName();
    m_record.simulationStartNs = Simulator::Now().GetNanoSeconds();
    owner.m_activeCost = &m_record;
  }
  ~CostScope()
  {
    if (!m_enabled) return;
    FinishPhase();
    m_record.hostWallNs = m_boundaryNs;
    m_record.simulationEndNs = Simulator::Now().GetNanoSeconds();
    m_record.nodes = m_owner.GetNodesNumber();
    m_record.requestedHeads = m_owner.GetMaxClusterNum();
    m_owner.m_activeCost = m_previous;
    m_owner.m_costRecords.push_back(m_record);
  }
  void Search() { if (m_enabled) { FinishPhase(); m_phase = &CostRecord::searchHostNs; } }
  void Commit() { if (m_enabled) { FinishPhase(); m_phase = &CostRecord::commitHostNs; } }
  void Outcome(const char* outcome) { if (m_enabled) m_record.outcome = outcome; }
private:
  using Clock = std::chrono::steady_clock;
  cafbcrElection& m_owner;
  CostRecord* m_previous;
  bool m_enabled;
  CostRecord m_record;
  Clock::time_point m_begin;
  uint64_t m_boundaryNs = 0;
  uint64_t CostRecord::*m_phase = &CostRecord::setupHostNs;
  void FinishPhase()
  {
    const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - m_begin).count();
    m_record.*m_phase += static_cast<uint64_t>(elapsed) - m_boundaryNs;
    m_boundaryNs = static_cast<uint64_t>(elapsed);
  }
};

TypeId cafbcrElection::GetTypeId()
{
  static TypeId tid = TypeId("ns3::cafbcr::cafbcrElection")
    .SetParent<Object>()
    .SetGroupName("cafbcr")
    .AddConstructor<cafbcrElection>()
    .AddAttribute("InitialSelector", "Initial candidate selection; subsequent cluster maintenance is unchanged",
                  EnumValue(ABC), MakeEnumAccessor(&cafbcrElection::m_initialSelector),
                  MakeEnumChecker(ABC, "abc", TOPK, "topk"))
    .AddAttribute("ObserveInitialSelection", "Capture initial inputs and the first existing scoring pass; no extra fitness calls",
                  BooleanValue(false), MakeBooleanAccessor(&cafbcrElection::m_observeInitialSelection),
                  MakeBooleanChecker())
    .AddAttribute("ObserveCost", "Record StartElection work and host time without changing simulation events",
                  BooleanValue(false), MakeBooleanAccessor(&cafbcrElection::m_observeCost),
                  MakeBooleanChecker());
  return tid;
}

cafbcrElection::cafbcrElection()
 :  m_rand(CreateObject<UniformRandomVariable>()),
    m_MaxClusterNumber(0),
    m_NodesNumber(0),
    m_MAX_CYCLES(10),
    m_abcInitialized(false)
{
    NS_LOG_FUNCTION(this);

    // 默认权重：LHTP 0.2, 速度 0.2, 距离 0.2, 能量 0.2, 计算能力 0.2
    // 实际权重由 cafbcrHelper 从 ScenarioConfig 传入并通过 SetWeightTable 覆盖
    double array[5] = {0.2, 0.2, 0.2, 0.2, 0.2};
    SetWeightTable(array);

    ClusterHeadCount = 0;
    global_best_value = 0.0;
    global_best_index = 0;
}

int64_t cafbcrElection::AssignStreams(int64_t stream)
{
  m_rand->SetStream(stream);
  return 1;
}

void cafbcrElection::Initialize()
{
  NS_LOG_FUNCTION(this);
  CountCost(&CostRecord::initializationCalls);

  if (m_abcInitialized) {
    NS_LOG_DEBUG("cafbcrElection already initialized");
    return;
  }

  m_NodesNumber = NodeList::GetNNodes();
  m_MaxClusterNumber = m_NodesNumber / 10;

  if (m_NodesNumber == 0) {
    NS_LOG_WARN("No nodes available for election");
    return;
  }

  m_clusternodes.resize(m_NodesNumber);
  for(uint32_t i = 0; i < m_NodesNumber; ++i)
  {
    CountCost(&CostRecord::initializationCandidateVisits);
    m_clusternodes[i] = NodeList::GetNode(i)->GetObject<ClusterNode>();
    if (!m_clusternodes[i]) {
      NS_LOG_ERROR("Node " << i << " does not have ClusterNode object");
    }
  }

  Fitness_Value.resize(m_clusternodes.size(), 0.0);
  Probabilitys.resize(m_clusternodes.size(), 0.0);
  trail_count.resize(m_clusternodes.size(), 0);
  m_ClusterHeadsId.resize(m_MaxClusterNumber, 0xFFFF);

  // 初始化边界
  if (m_clusternodes[0]) {
    DegreeBound[0] = m_clusternodes[0]->GetNeighborDegree();
    DegreeBound[1] = m_clusternodes[0]->GetNeighborDegree();
    MaxVel = m_clusternodes[0]->GetVelocity();
    MinVel = m_clusternodes[0]->GetVelocity();
  }

  for(size_t i = 0; i < m_clusternodes.size(); ++i)
  {
    CountCost(&CostRecord::initializationStateVisits);
    if (!m_clusternodes[i]) continue;

    Fitness_Value[i] = m_clusternodes[i]->GetFitnessValue();

    // 获取邻居度边界
    if(m_clusternodes[i]->GetNeighborDegree() > DegreeBound[1])
    {
      DegreeBound[1] = m_clusternodes[i]->GetNeighborDegree();
    }
    else if (m_clusternodes[i]->GetNeighborDegree() < DegreeBound[0])
    {
      DegreeBound[0] = m_clusternodes[i]->GetNeighborDegree();
    }

    // 获取速度边界
    Vector vel = m_clusternodes[i]->GetVelocity();
    double speed = sqrt(pow(vel.x, 2) + pow(vel.y, 2) + pow(vel.z, 2));
    double Maxspeed = sqrt(pow(MaxVel.x,2)+pow(MaxVel.y,2)+pow(MaxVel.z,2));
    double Minspeed = sqrt(pow(MinVel.x,2)+pow(MinVel.y,2)+pow(MinVel.z,2));
    if (speed > Maxspeed)
    {MaxVel = vel;}
    else if (speed < Minspeed)
    {MinVel = vel;}
  }

  m_abcInitialized = true;
  NS_LOG_INFO("cafbcrElection initialized with " << m_NodesNumber << " nodes, K=" << m_MaxClusterNumber);
}

void cafbcrElection::StartElection()
{
  NS_LOG_FUNCTION(this);
  CostScope cost(*this);

  // 延迟初始化
  if (!m_abcInitialized) {
    Initialize();
  }

  if (m_NodesNumber == 0) {
    cost.Outcome("no_nodes");
    NS_LOG_WARN("No nodes available for election");
    return;
  }

  // 检查是否已有簇头
  for(uint32_t i = 0; i < NodeList::GetNNodes(); i++)
  {
    CountCost(&CostRecord::backupScanVisits);
    Ptr<ClusterNode> cn = NodeList::GetNode(i)->GetObject<ClusterNode>();
    if(cn && cn->GetRoleType() == BackupHead)
    {
      cost.Outcome("skipped_existing_backup");
      NS_LOG_INFO("Cluster heads already exist, skipping election");
      return;
    }
  }

  if (m_observeInitialSelection) CaptureInitialSelection();

  // Only initial candidate generation varies. Both paths use the commit below.
  cost.Search();
  if (m_initialSelector == TOPK) {
    SelectTopK();
  } else {
    for (uint16_t i = 0; i < GetMaxClusterNum(); i++) {
      InitPopulation();
    }
  }

  cost.Commit();
  for(int m = 0; m < ClusterHeadCount; m++)
  {
    bool RepeatClusterHead = false;
    for (int a = 0; a < m ; a++)
    {
      CountCost(&CostRecord::commitDuplicateVisits);
      if (m_ClusterHeadsId[m] == m_ClusterHeadsId[a])
      {
        RepeatClusterHead = true;
        break;
      }
    }

    if (!RepeatClusterHead)
    {
      for(size_t i=0; i < m_clusternodes.size(); ++i)
      {
        CountCost(&CostRecord::commitCandidateVisits);
        if(m_clusternodes[i]->GetNodeId() == m_ClusterHeadsId[m])
        {
          m_clusternodes[i]->SetRole(BackupHead);
          BackUpClusterHeads.push_back(m_clusternodes[i]);
          CountCost(&CostRecord::committedHeads);

          NS_LOG_INFO("Node " << m_clusternodes[i]->GetNodeId()
                      << " elected as BackupHead (CH candidate) at time "
                      << Simulator::Now().GetSeconds() << "s");
          break;  // 找到后跳出内层循环
        }
      }
    }
  }

  NS_LOG_INFO("Cluster Head Election completed! " << BackUpClusterHeads.size()
              << " nodes elected as BackupHead at time "
              << Simulator::Now().GetSeconds() << "s");
  if (m_activeInitialSelection) {
    for (auto& candidate : m_activeInitialSelection->candidates) {
      for (int i = 0; i < ClusterHeadCount; ++i) {
        if (candidate.nodeId == m_ClusterHeadsId[i]) candidate.selected = true;
      }
    }
    m_activeInitialSelection = nullptr;
  }
}

void cafbcrElection::CaptureInitialSelection()
{
  InitialSelectionRecord record;
  record.simulationNs = Simulator::Now().GetNanoSeconds();
  record.nodes = GetNodesNumber();
  record.requestedHeads = GetMaxClusterNum();
  std::copy(std::begin(WeightTable), std::end(WeightTable), record.weights.begin());
  std::set<uint16_t> identities;
  uint32_t eligible = 0;
  for (const auto& node : m_clusternodes) {
    NS_ABORT_MSG_IF(!node, "Initial selection requires a ClusterNode for every candidate");
    InitialCandidate c;
    c.nodeId = node->GetNodeId();
    NS_ABORT_MSG_IF(!identities.insert(c.nodeId).second, "Duplicate initial candidate identity");
    c.role = node->GetRoleType();
    c.eligible = !node->IsClusterHead();
    eligible += c.eligible;
    c.networkScale = node->GetNetworkScale();
    c.degree = node->GetNeighborDegree();
    c.fitnessMode = node->GetFitnessMode();
    c.cachedFitness = node->GetFitnessValue();
    c.energy = node->GetResidualEnergy();
    c.originEnergy = node->GetOriginEnergy();
    c.computeCapacity = node->GetComputeCapacity();
    c.load = node->GetCurrentLoad();
    c.maxRange = node->GetMaxRange();
    c.lhtpUseMaxRange = node->GetLhtpUseMaxRange();
    c.position = node->GetPosition();
    c.velocity = node->GetVelocity();
    auto neighbors = node->GetNeighborTable();
    NS_ABORT_MSG_IF(!neighbors, "Initial candidate has no neighbor cache");
    // PeekAll and GetComputeObservation do not purge, reschedule, or score.
    for (const auto& entry : neighbors->PeekAll()) {
      c.neighbors.push_back({entry.first, entry.second, neighbors->GetComputeObservation(entry.first)});
    }
    record.candidates.push_back(std::move(c));
  }
  NS_ABORT_MSG_IF(eligible < record.requestedHeads, "Fewer eligible initial candidates than K");
  m_initialSelections.push_back(std::move(record));
  m_activeInitialSelection = &m_initialSelections.back();
}

void cafbcrElection::SelectTopK()
{
  if (GetMaxClusterNum() == 0) return;
  std::vector<uint16_t> indices;
  std::vector<uint16_t> savedDegrees;
  std::set<uint16_t> identities;
  for (uint16_t i = 0; i < GetNodesNumber(); ++i) {
    CountCost(&CostRecord::topkCandidateVisits);
    NS_ABORT_MSG_IF(!m_clusternodes[i], "TopK candidate is missing its ClusterNode");
    NS_ABORT_MSG_IF(!identities.insert(m_clusternodes[i]->GetNodeId()).second,
                    "Duplicate TopK candidate identity");
    if (!m_clusternodes[i]->IsClusterHead()) indices.push_back(i);
    savedDegrees.push_back(m_clusternodes[i]->GetNeighborDegree());
    CountCost(&CostRecord::stateSaveVisits);
  }
  NS_ABORT_MSG_IF(indices.size() < GetMaxClusterNum(), "Fewer eligible TopK candidates than K");
  // Reuse the same scoring pass as ABC's first InitPopulation. ComputeFitness
  // may purge expired advertisements and update degree; it does not change F0
  // into a cached score or use another normalization for TopK.
  for (uint16_t i = 0; i < GetNodesNumber(); ++i) InitBee(i);
  for (auto i : indices) {
    NS_ABORT_MSG_IF(!std::isfinite(Fitness_Value[i]), "Non-finite TopK initial fitness");
  }
  std::sort(indices.begin(), indices.end(), [this](uint16_t a, uint16_t b) {
    CountCost(&CostRecord::topkComparisons);
    if (Fitness_Value[a] != Fitness_Value[b]) return Fitness_Value[a] > Fitness_Value[b];
    return m_clusternodes[a]->GetNodeId() < m_clusternodes[b]->GetNodeId();
  });
  m_ClusterHeadsId.resize(GetMaxClusterNum(), 0xFFFF);
  for (uint16_t i = 0; i < GetMaxClusterNum(); ++i) {
    const auto index = indices[i];
    m_ClusterHeadsId[ClusterHeadCount++] = m_clusternodes[index]->GetNodeId();
    m_clusternodes[index]->SetRole(Head); // Same temporary exclusion marker as ABC.
  }
  // ABC restores the pre-pass node attributes after selection. Only degree can
  // be changed by this scoring-only path; preserve its original cache as well.
  for (uint16_t i = 0; i < GetNodesNumber(); ++i) {
    m_clusternodes[i]->SetNeighborDegree(savedDegrees[i]);
    CountCost(&CostRecord::stateRestoreVisits);
  }
}

void cafbcrElection::InitPopulation()
{
    CountCost(&CostRecord::selectionRounds);
    // 保存所有节点的原始状态，防止ABC迭代过程中破坏真实节点属性
    struct NodeState {
        double energy;
        Vector position;
        Vector velocity;
        uint16_t degree;
    };
    std::vector<NodeState> savedStates(GetNodesNumber());
    for (uint16_t i = 0; i < GetNodesNumber(); i++) {
        CountCost(&CostRecord::stateSaveVisits);
        if (m_clusternodes[i]) {
            savedStates[i].energy = m_clusternodes[i]->GetResidualEnergy();
            savedStates[i].position = m_clusternodes[i]->GetPosition();
            savedStates[i].velocity = m_clusternodes[i]->GetVelocity();
            savedStates[i].degree = m_clusternodes[i]->GetNeighborDegree();
        }
    }

    // 初始化所有蜜蜂
    for (uint16_t i = 0; i < GetNodesNumber(); i++)
    {
        InitBee(i);
    }

    // 设置初始全局最优
    global_best_value = Fitness_Value[0];
    global_best_index = 0;
    GetGlobalBest();

    // ABC迭代循环
    uint16_t cycle = 0;
    while (cycle < m_MAX_CYCLES)
    {
        CountCost(&CostRecord::cycleIterations);
        EmpolyBeePhase();
        GetProbability();
        OnlookerBeePhase();
        ScoutBeePhase();
        GetGlobalBest();
        cycle++;
    }

    // 记录本轮选出的簇头（存储节点ID而非数组索引）
    m_ClusterHeadsId[ClusterHeadCount] = m_clusternodes[global_best_index]->GetNodeId();
    m_clusternodes[global_best_index]->SetRole(Head);  // 标记为已选中，后续轮次会跳过
    ClusterHeadCount++;

    NS_LOG_DEBUG("ABC round " << ClusterHeadCount << ": selected node "
                 << m_clusternodes[global_best_index]->GetNodeId()
                 << " (index=" << global_best_index
                 << " fitness=" << global_best_value << ")");

    // 恢复所有节点的原始属性（保留Head角色标记不恢复）
    for (uint16_t i = 0; i < GetNodesNumber(); i++) {
        CountCost(&CostRecord::stateRestoreVisits);
        if (m_clusternodes[i]) {
            m_clusternodes[i]->updateResidualEnergy(savedStates[i].energy);
            m_clusternodes[i]->SetPosition(savedStates[i].position);
            m_clusternodes[i]->SetVelocity(savedStates[i].velocity);
            m_clusternodes[i]->SetNeighborDegree(savedStates[i].degree);
        }
    }
}

void cafbcrElection::InitBee(uint16_t NodeIndex)
{
    CountCost(&CostRecord::initBeeVisits);
    if (m_clusternodes[NodeIndex]->IsClusterHead())
    {
        // 已经是簇头的节点直接设为0（ComputeFitness会归一化权重，小权重无法惩罚）
        Fitness_Value[NodeIndex] = 0.0;
        trail_count[NodeIndex] = 0;
    }
    else
    {
        // 计算节点的Fitness值
        CountCost(&CostRecord::initBeeFitnessCalls);
        Fitness_Value[NodeIndex] =
            m_clusternodes[NodeIndex]->ComputeFitness(WeightTable[0], WeightTable[1],
                                                      WeightTable[2], WeightTable[3], WeightTable[4]);
        trail_count[NodeIndex] = 0;
    }
    if (m_activeInitialSelection) {
      auto& candidate = m_activeInitialSelection->candidates[NodeIndex];
      if (candidate.eligible && !candidate.scored) {
        candidate.fitness0 = Fitness_Value[NodeIndex];
        candidate.scored = true;
      }
    }
}

void cafbcrElection::GetGlobalBest()
{
    CountCost(&CostRecord::bestScanCalls);
    // 先找到第一个非簇头节点作为初始值
    bool initialized = false;
    for (int i = 0; i < GetNodesNumber(); i++)
    {
        CountCost(&CostRecord::bestScanVisits);
        if (m_clusternodes[i] && !m_clusternodes[i]->IsClusterHead())
        {
            if (!initialized)
            {
                global_best_value = Fitness_Value[i];
                global_best_index = i;
                initialized = true;
            }
            else if (Fitness_Value[i] > global_best_value)
            {
                global_best_value = Fitness_Value[i];
                global_best_index = i;
            }
        }
    }
}

void cafbcrElection::GetProbability()
{
    CountCost(&CostRecord::probabilityCalls);
    // 标准ABC轮盘赌选择：概率 = 适应度 / 总适应度
    double SumFitness = 0.0;
    for(int i = 0; i < GetNodesNumber(); i++)
    {
        CountCost(&CostRecord::probabilityVisits);
        SumFitness += Fitness_Value[i];
    }

    // 避免除零错误
    if (SumFitness < 1e-10)
    {
        // 如果总适应度接近0，给所有节点相同概率
        for(int i = 0; i < GetNodesNumber(); i++)
        {
            CountCost(&CostRecord::probabilityVisits);
            Probabilitys[i] = 1.0 / GetNodesNumber();
        }
    }
    else
    {
        for(int i = 0; i < GetNodesNumber(); i++)
        {
            CountCost(&CostRecord::probabilityVisits);
            Probabilitys[i] = Fitness_Value[i] / SumFitness;
        }
    }
}

void cafbcrElection::EmpolyBeePhase()
{
    for(int i = 0; i < GetNodesNumber(); i++)
    {
        CountCost(&CostRecord::employedVisits);
        if (m_clusternodes[i]->IsClusterHead()) {continue;}

        // 获取随机邻居
        std::vector<uint16_t> keys;
        auto neighborTable = m_clusternodes[i]->GetNeighborTable()->GetAll();
        for (const auto & kv: neighborTable)
        {
            CountCost(&CostRecord::employedNeighborVisits);
            keys.push_back(kv.first);
        }

        uint16_t neighbor_index;

        if (keys.empty())
        {
            CountCost(&CostRecord::employedNoNeighborFallbacks);
            NS_LOG_DEBUG("In EmployeeBee Phase, Node " << i << " has no neighbors");
            if (GetNodesNumber() < 2) continue;
            // Uniformly draw a different node without a rejection loop. The old
            // loop reused a single R forever when that draw selected this node.
            neighbor_index = m_rand->GetInteger(0, GetNodesNumber() - 2);
            if (neighbor_index >= i) ++neighbor_index;
        }
        else
        {
            uint16_t rand_index = m_rand->GetInteger(0, keys.size() - 1);
            neighbor_index = keys[rand_index];
        }

        Ptr<ClusterNode> NewSolution = m_clusternodes[i];

        // 标准ABC邻域搜索公式: X_ij^new = X_ij^old + φ * (X_ij^old - X_kj^neighbor)
        int8_t Modi_param_index = m_rand->GetInteger(0, 3);
        double phi = static_cast<double>(m_rand->GetInteger(0, 199)) / 100.0 - 1.0;

        double NewEnergy;
        Vector NewPosition;
        Vector NewVelocity;
        uint16_t NewDegree;
        double speed;
        double Maxspeed = sqrt(pow(MaxVel.x, 2) + pow(MaxVel.y, 2) + pow(MaxVel.z, 2));
        double Minspeed = sqrt(pow(MinVel.x, 2) + pow(MinVel.y, 2) + pow(MinVel.z, 2));

        switch (Modi_param_index)
        {
        case 0: // Energy
            NewEnergy = m_clusternodes[i]->GetResidualEnergy()
                      + phi * (m_clusternodes[i]->GetResidualEnergy()
                             - m_clusternodes[neighbor_index]->GetResidualEnergy());
            NewEnergy = JudgeBound(NewEnergy, NewSolution->GetOriginEnergy(), 0);
            NewSolution->updateResidualEnergy(NewEnergy);
            break;

        case 1: // Position
            NewPosition.x = m_clusternodes[i]->GetPosition().x
                          + phi * (m_clusternodes[i]->GetPosition().x
                                 - m_clusternodes[neighbor_index]->GetPosition().x);
            NewPosition.y = m_clusternodes[i]->GetPosition().y
                          + phi * (m_clusternodes[i]->GetPosition().y
                                 - m_clusternodes[neighbor_index]->GetPosition().y);
            NewPosition.z = m_clusternodes[i]->GetPosition().z
                          + phi * (m_clusternodes[i]->GetPosition().z
                                 - m_clusternodes[neighbor_index]->GetPosition().z);
            NewPosition.x = JudgeBound(NewPosition.x, AxisX_MAX, AxisX_MIN);
            NewPosition.y = JudgeBound(NewPosition.y, AxisY_MAX, AxisY_MIN);
            NewPosition.z = JudgeBound(NewPosition.z, AxisZ_MAX, AxisZ_MIN);
            NewSolution->SetPosition(NewPosition);
            break;

        case 2: // Velocity
            NewVelocity.x = m_clusternodes[i]->GetVelocity().x
                          + phi * (m_clusternodes[i]->GetVelocity().x
                                 - m_clusternodes[neighbor_index]->GetVelocity().x);
            NewVelocity.y = m_clusternodes[i]->GetVelocity().y
                          + phi * (m_clusternodes[i]->GetVelocity().y
                                 - m_clusternodes[neighbor_index]->GetVelocity().y);
            NewVelocity.z = m_clusternodes[i]->GetVelocity().z
                          + phi * (m_clusternodes[i]->GetVelocity().z
                                 - m_clusternodes[neighbor_index]->GetVelocity().z);
            speed = sqrt(pow(NewVelocity.x,2) + pow(NewVelocity.y,2) + pow(NewVelocity.z,2));

            if (speed > Maxspeed)
            {
                NewVelocity = MaxVel;
            }
            else if (speed < Minspeed)
            {
                NewVelocity = MinVel;
            }
            NewSolution->SetVelocity(NewVelocity);
            break;

        case 3: // Neighbor Degree
            NewDegree = m_clusternodes[i]->GetNeighborDegree()
                      + (uint16_t)(phi * (m_clusternodes[i]->GetNeighborDegree()
                                        - m_clusternodes[neighbor_index]->GetNeighborDegree()));
            NewDegree = JudgeBound(NewDegree, DegreeBound[1], DegreeBound[0]);
            NewSolution->SetNeighborDegree(NewDegree);
            break;
        }

        CountCost(&CostRecord::employedFitnessCalls);
        double NewFitness = NewSolution->ComputeFitness(WeightTable[0], WeightTable[1],
                                                        WeightTable[2], WeightTable[3], WeightTable[4]);
        if (NewFitness > Fitness_Value[i])
        {
            trail_count[i] = 0;
            m_clusternodes[i] = NewSolution;
            Fitness_Value[i] = NewFitness;
        }
        else
        {
            trail_count[i] += 1;
        }
    }
}

void cafbcrElection::OnlookerBeePhase()
{
    Ptr<ClusterNode> NewSolution;

    for (int p = 0; p < GetNodesNumber(); p++)
    {
        CountCost(&CostRecord::onlookerVisits);
        if (m_clusternodes[p]->IsClusterHead()) {continue;}

        CountCost(&CostRecord::onlookerTrials);
        double m = m_rand->GetValue(0.0, 1.0);

        if (m < Probabilitys[p])
        {
            CountCost(&CostRecord::onlookerSelectedTrials);
            NewSolution = m_clusternodes[p];

            if (GetNodesNumber() < 2) continue;
            uint16_t param_modi_index = m_rand->GetInteger(0, 3);
            double phi = static_cast<double>(m_rand->GetInteger(0, 199)) / 100.0 - 1.0;

            // 随机选择一个参考蜜源用于局部搜索
            uint16_t refIndex = m_rand->GetInteger(0, GetNodesNumber() - 2);
            if (refIndex >= p) ++refIndex;

            double NewEnergy;
            Vector NewPosition;
            Vector NewVelocity;
            double speed;
            uint16_t NewDegree;
            double Maxspeed = sqrt(pow(MaxVel.x, 2) + pow(MaxVel.y, 2) + pow(MaxVel.z, 2));
            double Minspeed = sqrt(pow(MinVel.x, 2) + pow(MinVel.y, 2) + pow(MinVel.z, 2));

            switch (param_modi_index)
            {
            case 0: // Energy
                NewEnergy = m_clusternodes[p]->GetResidualEnergy()
                          + phi * (m_clusternodes[p]->GetResidualEnergy()
                                 - m_clusternodes[refIndex]->GetResidualEnergy());
                NewEnergy = JudgeBound(NewEnergy, NewSolution->GetOriginEnergy(), 0);
                NewSolution->updateResidualEnergy(NewEnergy);
                break;

            case 1: // Position
                NewPosition.x = m_clusternodes[p]->GetPosition().x
                              + phi * (m_clusternodes[p]->GetPosition().x
                                     - m_clusternodes[refIndex]->GetPosition().x);
                NewPosition.y = m_clusternodes[p]->GetPosition().y
                              + phi * (m_clusternodes[p]->GetPosition().y
                                     - m_clusternodes[refIndex]->GetPosition().y);
                NewPosition.z = m_clusternodes[p]->GetPosition().z
                              + phi * (m_clusternodes[p]->GetPosition().z
                                     - m_clusternodes[refIndex]->GetPosition().z);
                NewPosition.x = JudgeBound(NewPosition.x, AxisX_MAX, AxisX_MIN);
                NewPosition.y = JudgeBound(NewPosition.y, AxisY_MAX, AxisY_MIN);
                NewPosition.z = JudgeBound(NewPosition.z, AxisZ_MAX, AxisZ_MIN);
                NewSolution->SetPosition(NewPosition);
                break;

            case 2: // Velocity
                NewVelocity.x = m_clusternodes[p]->GetVelocity().x
                              + phi * (m_clusternodes[p]->GetVelocity().x
                                     - m_clusternodes[refIndex]->GetVelocity().x);
                NewVelocity.y = m_clusternodes[p]->GetVelocity().y
                              + phi * (m_clusternodes[p]->GetVelocity().y
                                     - m_clusternodes[refIndex]->GetVelocity().y);
                NewVelocity.z = m_clusternodes[p]->GetVelocity().z
                              + phi * (m_clusternodes[p]->GetVelocity().z
                                     - m_clusternodes[refIndex]->GetVelocity().z);
                speed = sqrt(pow(NewVelocity.x,2) + pow(NewVelocity.y,2) + pow(NewVelocity.z,2));

                if (speed > Maxspeed)
                {
                    NewVelocity = MaxVel;
                }
                else if (speed < Minspeed)
                {
                    NewVelocity = MinVel;
                }
                NewSolution->SetVelocity(NewVelocity);
                break;

            case 3: // Neighbor Degree
                NewDegree = m_clusternodes[p]->GetNeighborDegree()
                          + (uint16_t)(phi * (m_clusternodes[p]->GetNeighborDegree()
                                           - m_clusternodes[refIndex]->GetNeighborDegree()));
                NewDegree = JudgeBound(NewDegree, DegreeBound[1], DegreeBound[0]);
                NewSolution->SetNeighborDegree(NewDegree);
                break;
            }

            CountCost(&CostRecord::onlookerFitnessCalls);
            double NewFitness = NewSolution->ComputeFitness(WeightTable[0], WeightTable[1],
                                                            WeightTable[2], WeightTable[3], WeightTable[4]);
            if (NewFitness > Fitness_Value[p])
            {
                trail_count[p] = 0;
                m_clusternodes[p] = NewSolution;
                Fitness_Value[p] = NewFitness;
            }
            else
            {
                trail_count[p] += 1;
            }
        }
    }
}

void cafbcrElection::ScoutBeePhase()
{
    int16_t max_index = 0;
    for (uint16_t i = 0; i < GetNodesNumber(); i++)
    {
        CountCost(&CostRecord::scoutScanVisits);
        if (trail_count[i] > trail_count[max_index])
        {
            max_index = i;
        }
    }

    if (trail_count[max_index] >= (GetNodesNumber() * 4))
    {
        CountCost(&CostRecord::scoutReinitializations);
        InitBee(max_index);
    }
}

} // namespace cafbcr
} // namespace ns3
