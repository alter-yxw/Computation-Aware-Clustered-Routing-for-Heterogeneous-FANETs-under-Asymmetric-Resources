/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "fanet-neighbor.h"
#include "ns3/log.h"
#include "ns3/vector.h"
#include <cmath>
namespace ns3{
  
NS_LOG_COMPONENT_DEFINE("Neighbors");
NS_OBJECT_ENSURE_REGISTERED(Neighbors);

// Optional, node-local metadata; absent for protocols that have not opted in.
class NeighborComputeHistory : public Object
{
public:
  static TypeId GetTypeId()
  {
    static TypeId tid = TypeId("ns3::NeighborComputeHistory")
      .SetParent<Object>().AddConstructor<NeighborComputeHistory>();
    return tid;
  }
  struct Entry { Time receivedAt; Time expiresAt; bool valid; };
  std::map<uint16_t, Entry> entries;
};
NS_OBJECT_ENSURE_REGISTERED(NeighborComputeHistory);

const char* Neighbors::ComputeObservation::StatusName() const
{
  switch (status) {
    case UNTRACKED: return "untracked";
    case UNKNOWN: return "unknown";
    case INVALID: return "invalid";
    case EXPIRED: return "expired";
    case FRESH: return "fresh";
  }
  return "unknown";
}

void Neighbors::EnableComputeFreshness()
{
  if (!GetObject<NeighborComputeHistory>()) {
    AggregateObject(CreateObject<NeighborComputeHistory>());
  }
}

void Neighbors::RecordComputeAdvertisement(uint16_t nodeId, Time lifetime)
{
  auto history = GetObject<NeighborComputeHistory>();
  auto it = m_neighborTable.find(nodeId);
  NS_ABORT_MSG_IF(!history || it == m_neighborTable.end() || lifetime <= Time(0),
                  "Compute advertisement requires an enabled, present neighbor and positive lifetime");
  const auto& info = it->second;
  const bool valid = std::isfinite(info.computeCapacity) && info.computeCapacity > 0.0 &&
                     std::isfinite(info.currentLoad) && info.currentLoad >= 0.0f &&
                     info.currentLoad <= 1.0f;
  history->entries[nodeId] = {Simulator::Now(), Simulator::Now() + lifetime, valid};
}

Neighbors::ComputeObservation Neighbors::GetComputeObservation(uint16_t nodeId) const
{
  ComputeObservation result;
  auto history = GetObject<NeighborComputeHistory>();
  if (!history) return result;
  result.status = ComputeObservation::UNKNOWN;
  auto neighbor = m_neighborTable.find(nodeId);
  if (neighbor == m_neighborTable.end() ||
      neighbor->second.UpdateTime + neighbor->second.NeighborLifeLength <= Simulator::Now()) {
    return result;
  }
  auto it = history->entries.find(nodeId);
  if (it == history->entries.end()) return result;
  const auto& entry = it->second;
  result.receivedAt = entry.receivedAt;
  result.expiresAt = entry.expiresAt;
  result.status = !entry.valid ? ComputeObservation::INVALID
                : Simulator::Now() >= entry.expiresAt ? ComputeObservation::EXPIRED
                : ComputeObservation::FRESH;
  return result;
}

bool Neighbors::HasUsableComputeState(uint16_t nodeId) const
{
  return GetComputeObservation(nodeId).IsUsable();
}

Neighbors::Neighbors()
  : m_timer(Timer::CANCEL_ON_DESTROY),
    m_NodeId(0xFFFF)  
{
  m_txErrorCallback = MakeCallback(&Neighbors::ProcessTxError, this);
  m_timer.SetDelay(Seconds(2.0));
  m_timer.SetFunction(&Neighbors::Purge, this);
}

Neighbors::Neighbors(uint16_t myId)
  :   m_timer(Timer::CANCEL_ON_DESTROY),
      m_NodeId(myId)
{
  m_txErrorCallback = MakeCallback(&Neighbors::ProcessTxError, this);
  m_timer.SetDelay(Seconds(2.0));
  m_timer.SetFunction(&Neighbors::Purge,this);
}



TypeId Neighbors::GetTypeId(){
  static TypeId tid = TypeId ("ns3::Neighbors")
    .SetParent<Object>()
    // .SetGroupName("fanet")
    .AddConstructor<Neighbors>();
    return tid;
}


bool Neighbors::IsNeighbor(uint16_t NeighborId)
{
  Purge();
  for(auto i = m_neighborTable.begin(); i != m_neighborTable.end(); ++i)
  {
    if(i->first == NeighborId) {   return true; }
  }
  return false;
}


void Neighbors::AddNeighbor(NeighborInfo NewNeighbor)
{
  uint16_t NodeId = NewNeighbor.NodeId;
  if(NewNeighbor.UpdateTime.GetSeconds() != 0.0)
  {
    NewNeighbor.UpdateTime = Simulator::Now();
  }
  Purge();
  // Purge may erase the previous entry for this ID; only look it up afterwards.
  auto i = m_neighborTable.find(NodeId);
  if(i != m_neighborTable.end()) // || i->first == NodeId)
  {
    // m_neighborTable.erase(NodeId);
    i -> second = NewNeighbor;
    NS_LOG_DEBUG(m_NodeId<<" Updated neighbor " << NodeId <<", neighborIp: "<<NewNeighbor.ipAddress<<", Time: "<<Simulator::Now().GetSeconds());
  }
  else{
    m_neighborTable.insert(std::make_pair(NodeId, NewNeighbor));
    NS_LOG_DEBUG(m_NodeId<<" Added new neighbor " << NodeId <<", neighborIp: "<<NewNeighbor.ipAddress<<", Time: "<<Simulator::Now().GetSeconds());
    // NS_LOG_DEBUG(m_NodeId<<" Added new neighbor " << NodeId << ", degree: " << m_neighborTable.size()
    //                     <<", Time: "<<Simulator::Now().GetSeconds());
  }
  
  if(!m_timer.IsRunning() && !m_neighborTable.empty())
  {
    m_timer.Schedule();
  }

  // 第一次添加邻居时启动定时器
  // if(m_neighborTable.size() == 1 && !m_timer.IsRunning())
  // {
  //   m_timer.Schedule();
  // }
}


void Neighbors::DeleteNeighbor(uint16_t NodeId)
{
  if(!IsNeighbor(NodeId))
  {
    NS_LOG_INFO("Neighbor Node "<<NodeId<<" not exist yet.");
    return;
  }
  m_neighborTable.erase(NodeId);
  if (auto history = GetObject<NeighborComputeHistory>()) history->entries.erase(NodeId);
}


void Neighbors::Purge()
{
  if(m_neighborTable.empty()) { return; }
  
  std::list<uint16_t> toErase;
  for(auto i = m_neighborTable.begin(); i !=m_neighborTable.end(); i++) {
    if(i->second.NeighborLifeLength + i->second.UpdateTime <= Simulator::Now()) {
      toErase.insert(toErase.begin(),i->first);
      NS_LOG_DEBUG(m_NodeId<<" Delete neighbor NodeId: "<<i->first<<", Time: "<<Simulator::Now().GetSeconds());
    }
  }
  toErase.unique();

  for(auto it = toErase.begin(); it != toErase.end(); it++){
    m_neighborTable.erase(*it);
    if (auto history = GetObject<NeighborComputeHistory>()) history->entries.erase(*it);
  }
  // 只在还有邻居时才重新调度定时器
  if(!m_neighborTable.empty() && !m_timer.IsRunning())
  {
    m_timer.Cancel();
    m_timer.Schedule();
  }
}


void Neighbors::Clear() {
  m_neighborTable.clear();
  if (auto history = GetObject<NeighborComputeHistory>()) history->entries.clear();
  m_timer.Cancel();
}
   

const std::map<uint16_t, Neighbors::NeighborInfo>&  Neighbors::GetAll()  
{
  if (!m_neighborTable.empty()) {
    Purge();
  }
  return m_neighborTable;
}

uint16_t Neighbors::GreedyNeighbor(Vector dstPos, Vector myPos, uint16_t prevHopId /* = 0xFFFF */,
                                   Vector myVel, bool linkAware, double maxRange, uint16_t myClusterId,
                                   bool predictPosition)
{
  NS_LOG_FUNCTION(this << dstPos << myPos << prevHopId << linkAware << predictPosition);
  double myDistance = CalculateDistance(myPos, dstPos);
  int16_t bestNeighborNodeId = 0xFFFF;
  double MaxProgress = 0;
  Purge();

  for(auto neighbor = m_neighborTable.begin(); neighbor!=m_neighborTable.end(); ++neighbor)
  {
    // 跳过前一跳节点，避免路由环路
    if (neighbor->first == prevHopId) {
      NS_LOG_DEBUG("Skipping previous hop " << prevHopId << " to avoid routing loop");
      continue;
    }

    Vector neighborPos = neighbor->second.Position;
    if (predictPosition) {
      // 用 Hello 携带的 Velocity 和 UpdateTime 推算当前时刻位置，
      // 补偿较长 Hello 间隔/邻居 TTL 带来的位置过期。
      double age = (Simulator::Now() - neighbor->second.UpdateTime).GetSeconds();
      if (age < 0.0) { age = 0.0; }
      if (age > 2.0) { age = 2.0; }
      neighborPos = Vector(neighborPos.x + neighbor->second.Velocity.x * age,
                           neighborPos.y + neighbor->second.Velocity.y * age,
                           neighborPos.z + neighbor->second.Velocity.z * age);
    }
    double neighborDistance = CalculateDistance(neighborPos, dstPos);
    double progress = myDistance - neighborDistance;
    if(progress > 0)
    {
      double weigh = 1.0;

      if(neighbor->second.Role == Head)   { weigh *= 1.5;}

      weigh *= (neighbor->second.ResEnergy / 100);
      Time timeSinceUpdate = Simulator::Now() - neighbor->second.UpdateTime;

      if(timeSinceUpdate < Seconds(1.0))  { weigh *= 1.2;}

      if (linkAware) {
        // Estimate link lifetime from the position and velocity carried in
        // Hello messages, and mildly penalize neighbours near the
        // communication edge as well as cross-cluster neighbours.
        Vector relPos = neighborPos - myPos;
        double dx = relPos.x;
        double dy = relPos.y;
        double dz = relPos.z;
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        Vector relVel = neighbor->second.Velocity - myVel;
        double radial = (dist > 1e-6)
            ? (relPos.x * relVel.x + relPos.y * relVel.y + relPos.z * relVel.z) / dist
            : 0.0;
        double lifetime = 10.0;
        if (radial > 0.1) {  // 正在远离：预计 (range-distance)/radial 后断链
          lifetime = (maxRange - dist) / radial;
          if (lifetime < 0.0) { lifetime = 0.0; }
          if (lifetime > 10.0) { lifetime = 10.0; }
        }
        // 链路稳定性权重 [0.5, 1.0]，2 s 以上视为充分稳定
        weigh *= (0.5 + 0.5 * std::min(1.0, lifetime / 2.0));
        // 距离质量：避免选中 500 m 边缘的弱链路
        if (maxRange > 0.0) {
          weigh *= (1.0 - 0.25 * std::min(1.0, dist / maxRange));
        }
        // Intra-cluster cohesion: prefer neighbours in the same cluster.
        if (myClusterId != 0xFFFF && neighbor->second.clusterId == myClusterId) {
          weigh *= 1.1;
        }
      }

      // === 新增：链路质量 + 负载因子 ===
      // 链路质量：距离越近（<=500m）质量越高，采用指数衰减模型
      double dx = neighborPos.x - myPos.x;
      double dy = neighborPos.y - myPos.y;
      double dz = neighborPos.z - myPos.z;
      double distKm = std::sqrt(dx*dx + dy*dy + dz*dz) / 1000.0;
      double linkScore = (distKm <= 0.5) ? 1.0 : (distKm > 5.0) ? 0.0 : std::exp(-distKm);

      // 负载感知：当前负载越低越好（负载反向权重）
      double loadScore = 1.0 - static_cast<double>(neighbor->second.currentLoad);

      // 综合链路质量和负载：链路质量70%权重，负载30%权重
      weigh *= (0.7 * linkScore +
                (HasUsableComputeState(neighbor->first) ? 0.3 * loadScore : 0.0));

      double weightedProgress = progress * weigh;
      if (weightedProgress > MaxProgress)
      {
        MaxProgress = weightedProgress;
        bestNeighborNodeId = neighbor->first;
      }
    }
  }
  NS_LOG_DEBUG("Greedy next hop: " << bestNeighborNodeId << " with progress: " << MaxProgress);
  return bestNeighborNodeId;
}

uint16_t Neighbors::PerimeterNextHop(Vector dstPos, Vector myPos, Vector & LastHopPos)
{ // 周边转发实现（当贪婪转发失败时）
  NS_LOG_FUNCTION(this<< dstPos<<myPos<<LastHopPos);
  std::vector<uint16_t> candidates;

  // Use all neighbors without filtering to preserve the GPSR planar-graph
  // property; preferring cluster heads would break the perimeter guarantee.
  for (auto &neighbor : m_neighborTable) {
    candidates.push_back(neighbor.first);
  }

  if (!candidates.empty())  { return RightHandRule(candidates, dstPos, myPos); }

  return 0xFFFF;
}

uint16_t Neighbors::PerimeterNextHopWithinRange(Vector dstPos, Vector myPos,
                                                Vector & LastHopPos, double myMaxRange)
{
  NS_LOG_FUNCTION(this << dstPos << myPos << LastHopPos << myMaxRange);
  (void)LastHopPos;  // 保留接口对齐 PerimeterNextHop，参数留给未来扩展

  std::vector<uint16_t> candidates;
  double r2 = (myMaxRange > 0.0) ? (myMaxRange * myMaxRange) : -1.0;

  // [2.A] 候选池按 myMaxRange 预过滤：异构链路下右手规则候选池里
  // 包含本节点打不到的"接收侧 asymmetric 邻居"会让 perimeter 选出
  // 不可达 next-hop，导致包要么 MAC 死要么走更远绕路。预过滤保证
  // RightHandRule 在物理可达邻居池上做拓扑决策。
  for (auto & nb : m_neighborTable) {
    if (r2 > 0.0) {
      double dx = nb.second.Position.x - myPos.x;
      double dy = nb.second.Position.y - myPos.y;
      double dz = nb.second.Position.z - myPos.z;
      if ((dx * dx + dy * dy + dz * dz) > r2) continue;
    }
    candidates.push_back(nb.first);
  }

  if (!candidates.empty()) { return RightHandRule(candidates, dstPos, myPos); }
  return 0xFFFF;
}

bool Neighbors::IsLocalMaximum(Vector dstPos, Vector myPos)
{
  double myDistance = CalculateDistance(myPos, dstPos);

  for(auto neighbor = m_neighborTable.begin(); neighbor!=m_neighborTable.end();++neighbor)
  {
    double dis = CalculateDistance(neighbor->second.Position,dstPos);
    if(dis < myDistance)  {return false;}
  }
  return true;  // 所有邻居都更远，是局部最大值
}

uint16_t Neighbors::GetNearestClusterHead(Vector dstPos)
{ // 获取最近的邻居簇头
  double minDistance = std::numeric_limits<double>::max();
  uint16_t BestChNoderId = 0xFFFF;
  for(auto neighbor = m_neighborTable.begin(); neighbor!=m_neighborTable.end(); ++neighbor)
  {
    if(neighbor->second.Role == Head)
    {
      double dis = CalculateDistance(neighbor->second.Position, dstPos);
      if(dis < minDistance)
      {
        minDistance = dis;
        BestChNoderId = neighbor->first;
      }
    }
  }

  return BestChNoderId;
}


uint16_t Neighbors::RightHandRule(std::vector<uint16_t> candidates, Vector dstPos, Vector myPos)
{ // 右手规则实现
  if (candidates.empty()) return -1;
  
  // 计算目的地方向向量
  Vector toDst = dstPos - myPos;
  
  double minAngle = 2 * M_PI;
  uint16_t selectedNeighbor = candidates[0];
  
  for (uint16_t nodeId : candidates)
  {
    if (m_neighborTable.find(nodeId) != m_neighborTable.end())
    {
      Vector toNeighbor = m_neighborTable.at(nodeId).Position - myPos;
      double angle = GetAngle(toDst, toNeighbor);  // 计算顺时针角度      
      if (angle < minAngle)
      {
        minAngle = angle;
        selectedNeighbor = nodeId;
      }
    }
  }
    
  return selectedNeighbor;
}

double Neighbors::GetAngle(Vector v1, Vector v2)
{
  // Project the right-hand-rule angle onto the XY plane.
  double dot = v1.x*v2.x + v1.y*v2.y;
  double det = v1.x*v2.y - v1.y*v2.x;  // 2D平面上的叉积
  double angle = std::atan2(det, dot);  
  if (angle < 0) { angle += 2 * M_PI;  /* 转换为0-2π范围*/ }
  
  return angle;
}


void Neighbors::ProcessTxError(WifiMacHeader const & hdr)
{
    // Mac48Address Macaddr = hdr.GetAddr1();
    // for (auto i = m_neighborTable.begin(); i != m_neighborTable.end(); ++i)
    // {
    //     if(i->second.HardwareAddress == Macaddr)
    //     {
    //         i->second.NeighborLifeLength = Seconds(0.0);
    //     }
    // }
    // Purge();
}


uint16_t Neighbors::NearestNeighbor(Vector DstPos, Vector myPos)
{
  double initialDistance = CalculateDistance(myPos, DstPos);
  uint16_t BestNeighborId = m_neighborTable.begin()->first;
  double   BestNeighborDis = CalculateDistance(m_neighborTable.begin()->second.Position, DstPos);
  for(auto i = m_neighborTable.begin();i!=m_neighborTable.end();++i) {
    double dis = CalculateDistance(i->second.Position, DstPos);
    if(BestNeighborDis > dis){
      BestNeighborDis = dis;
      BestNeighborId = i->first;
    }
  }

  if(BestNeighborDis>initialDistance){  return BestNeighborId; }
  else{  return 0xFFFF;}
}

} // namespace ns3
