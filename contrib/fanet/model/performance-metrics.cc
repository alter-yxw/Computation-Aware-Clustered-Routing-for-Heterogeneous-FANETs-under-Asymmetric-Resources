/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "performance-metrics.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/node-list.h"
#include "ns3/energy-module.h"
#include "fanet-clustermanager.h"
#include <cmath>
#include <iomanip>
#include <sstream>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("PerformanceMetrics");
NS_OBJECT_ENSURE_REGISTERED(PerformanceMetrics);

TypeId PerformanceMetrics::GetTypeId()
{
  static TypeId tid = TypeId("ns3::PerformanceMetrics")
  .SetParent<Object>()
  .SetGroupName("Applications")
  .AddConstructor<PerformanceMetrics>();
  return tid;
}


PerformanceMetrics::PerformanceMetrics()
{
  // m_routingStats.totalDataPackets = 0;
  // m_routingStats.totalControlPackets = 0;
  // m_routingStats.totalHops = 0;
  // m_routingStats.totalReceivedPackets = 0;
  // m_routingStats.totalDelay = 0;
  m_clusteringStats.totalClustersCreated = 0;
}


void PerformanceMetrics::SetFlowMonitor(Ptr<FlowMonitor> monitor, Ptr<Ipv4FlowClassifier> classifier)
{
  m_flowMonitor = monitor;
  m_classifier  = classifier;
}


void PerformanceMetrics::SetTestFlows(const std::vector<std::tuple<uint16_t, uint16_t, uint16_t, Ipv4Address, Ipv4Address>>& flows)
{
  m_testFlows = flows;
}


void PerformanceMetrics::CollectClusteringMetrics()
{ // ========== 数据收集 ==========
  NS_LOG_FUNCTION(this);
  Ptr<ClusterManager> clusterMgr = ClusterManager::GetInstance();
  auto clusters = clusterMgr->GetAllClusters();  // 统计当前簇数量和规模

  // 周期性统计：当前存在的簇个数
  uint32_t activeClusterCount = clusters.size();
  m_clusteringStats.periodicClusterCount.push_back(activeClusterCount);

  uint32_t totalMembers = 0;
  uint32_t totalClusterSize = 0; // 所有簇的总规模

  for (auto& cluster : clusters) {
    uint16_t clusterId = cluster.first;
    uint32_t size = cluster.second->GetAllMembers().size();
    totalMembers += size;
    totalClusterSize += size;

    m_clusteringStats.clusterSizeHistory.push_back(size);
    // 记录簇创建时间（如果是新簇）
    if (m_clusteringStats.clusterCreationTime.find(clusterId) ==  m_clusteringStats.clusterCreationTime.end()) {
      m_clusteringStats.clusterCreationTime[clusterId] = Seconds(cluster.second->GetInitializeTime());
      m_clusteringStats.totalClustersCreated++;
    }
  }

  // 周期性统计：平均簇规模（统计所有簇）
  double avgClusterSize = (activeClusterCount > 0) ? ((double)totalClusterSize / activeClusterCount) : 0.0;
  m_clusteringStats.periodicAvgClusterSize.push_back(avgClusterSize);

  // 周期性统计：节点入簇比率
  uint32_t clusteredNodes = 0;
  for (uint32_t i = 0; i < m_nodes.GetN(); i++) {
    Ptr<ClusterNode> cNode = m_nodes.Get(i)->GetObject<ClusterNode>();
    if (!cNode) continue;

    if (cNode->GetClusterId() != 0xFFFF) {
      clusteredNodes++;
    }
  }
  double clusteringRatio = (m_nodes.GetN() > 0) ? ((double)clusteredNodes / m_nodes.GetN()) : 0.0;
  m_clusteringStats.periodicClusteringRatio.push_back(clusteringRatio);

  for (uint32_t i = 0; i < m_nodes.GetN(); i++) {  // 检测节点换簇和角色变化
    Ptr<ClusterNode> cNode = m_nodes.Get(i)->GetObject<ClusterNode>();
    if (!cNode) continue;

    uint16_t nodeId = cNode->GetNodeId();
    uint16_t currentCluster = cNode->GetClusterId();
    uint8_t currentRole = cNode->GetRoleType();
    // 检测换簇
    if (m_lastNodeCluster.find(nodeId) != m_lastNodeCluster.end()) {
      if (m_lastNodeCluster[nodeId] != currentCluster &&
        currentCluster != 0xFFFF && m_lastNodeCluster[nodeId] != 0xFFFF) {
        m_clusteringStats.nodeClusterSwitchCount[nodeId]++;
      }
    }
    m_lastNodeCluster[nodeId] = currentCluster;
    // 检测角色变化
    if (m_lastNodeRole.find(nodeId) != m_lastNodeRole.end()) {
      if (m_lastNodeRole[nodeId] != currentRole) {
        m_clusteringStats.nodeRoleChangeCount[nodeId]++;
      }
    }
    m_lastNodeRole[nodeId] = currentRole;
  }
}

void PerformanceMetrics::CollectLinkMetrics()
{
  NS_LOG_FUNCTION(this);
  Time now = Simulator::Now();
  // 遍历所有节点的邻居表
  for (uint32_t i = 0; i < m_nodes.GetN(); i++) {
    Ptr<ClusterNode> cNode = m_nodes.Get(i)->GetObject<ClusterNode>();
    if (!cNode) continue;

    Ptr<Neighbors> neighbors = cNode->GetObject<Neighbors>();
    if (!neighbors) continue;

    uint16_t nodeId = cNode->GetNodeId();
    auto neighborMap = neighbors->GetAll();

    for (auto& n : neighborMap) {
      uint16_t neighborId = n.first;
      std::pair<uint16_t, uint16_t> linkPair = std::make_pair(
                                       std::min(nodeId, neighborId), std::max(nodeId, neighborId));
    
      if (m_linkHistory.find(linkPair) == m_linkHistory.end())   // 新链路
      {        
        LinkInfo info;
        info.creationTime = now;
        info.lastSeenTime = now;
        info.isAlive = true;
        m_linkHistory[linkPair] = info;
      } 
      else {  // 更新现有链路

        m_linkHistory[linkPair].lastSeenTime = now;
        m_linkHistory[linkPair].isAlive = true;
      }
    }
  }

  for (auto& link : m_linkHistory) {  // 检查链路断开
    if (link.second.isAlive && (now - link.second.lastSeenTime) > Seconds(3.0)) {
      link.second.isAlive = false;    // 链路断开
      double lifetime = (link.second.lastSeenTime - link.second.creationTime).GetSeconds();
      m_linkLifetimes.push_back(lifetime);
    }
  }
}


double PerformanceMetrics::CalculateAverageLinkLifetime()
{
  if (m_linkLifetimes.empty()) return 0.0;
  double sum = 0.0;
  for (double lifetime : m_linkLifetimes) { sum += lifetime; }
  
  return sum / m_linkLifetimes.size();
}


uint32_t PerformanceMetrics::GetClusterCount()
{
  Ptr<ClusterManager> clusterMgr = ClusterManager::GetInstance();

  return clusterMgr->GetAllClusters().size();
}


double PerformanceMetrics::GetAverageClusterSize()
{
  Ptr<ClusterManager> clusterMgr = ClusterManager::GetInstance();  
  uint32_t totalClusters;
  double avgLifetime, avgSize;
  clusterMgr->GetClusteringStatistics(totalClusters, avgLifetime, avgSize);
  
  return avgSize;
}

double PerformanceMetrics::GetNodeClusteringRatio()
{
  uint32_t clusteredNodes = 0;
  for (uint32_t i = 0; i < m_nodes.GetN(); i++) {
    Ptr<ClusterNode> cNode = m_nodes.Get(i)->GetObject<ClusterNode>();
    if (cNode && cNode->GetClusterId() != 0xFFFF) {  clusteredNodes++; }
  }

  return (double)clusteredNodes / m_nodes.GetN();
}


double PerformanceMetrics::GetAverageClusterLifetime()
{
  Ptr<ClusterManager> clusterMgr = ClusterManager::GetInstance();
  uint32_t totalClusters;
  double avgLifetime, avgSize;
  clusterMgr->GetClusteringStatistics(totalClusters, avgLifetime, avgSize);
  
  return avgLifetime;
}


double PerformanceMetrics::GetClusterSwitchRate()
{
  uint32_t totalSwitches = 0;
  for (auto& entry : m_clusteringStats.nodeClusterSwitchCount) {
    totalSwitches += entry.second;
  }

  return (double)totalSwitches / m_nodes.GetN();
}


double PerformanceMetrics::GetRoleChangeRate()
{
  uint32_t totalChanges = 0;
  for (auto& entry : m_clusteringStats.nodeRoleChangeCount) { totalChanges += entry.second;}

  return (double)totalChanges / m_nodes.GetN();
}


void PerformanceMetrics::PrintClusteringMetrics(std::ostream& os)
{
  os << "\n^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ 分簇效率性能指标 ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n";

  Ptr<ClusterManager> clusterMgr = ClusterManager::GetInstance();
  uint32_t totalClusters, activeCount;
  double avgLifetime, avgSize, activeAvgSize, ActiveAvgSizePro;
  clusterMgr->GetClusteringStatistics(totalClusters, avgLifetime, avgSize);
  clusterMgr->GetActiveClusterStats(activeCount, activeAvgSize, ActiveAvgSizePro);

  // 计算周期性统计的平均值
  double avgPeriodicClusterNum = 0.0;
  double avgPeriodicClusteringRatio = 0.0;
  double avgPeriodicClusterSize = 0.0;

  if (!m_clusteringStats.periodicClusterCount.empty()) {
    for (auto count : m_clusteringStats.periodicClusterCount) {
      avgPeriodicClusterNum += count;
    }
    avgPeriodicClusterNum /= m_clusteringStats.periodicClusterCount.size();
  }

  if (!m_clusteringStats.periodicClusteringRatio.empty()) {
    for (auto ratio : m_clusteringStats.periodicClusteringRatio) {
      avgPeriodicClusteringRatio += ratio;
    }
    avgPeriodicClusteringRatio /= m_clusteringStats.periodicClusteringRatio.size();
  }

  if (!m_clusteringStats.periodicAvgClusterSize.empty()) {
    for (auto size : m_clusteringStats.periodicAvgClusterSize) {
      avgPeriodicClusterSize += size;
    }
    avgPeriodicClusterSize /= m_clusteringStats.periodicAvgClusterSize.size();
  }

  std::ostringstream line1;
  line1 << std::fixed << std::setprecision(2);
  line1 << std::left << std::setw(30) << ("总创建簇数: " + std::to_string(totalClusters) + " 个") << " | "
        << std::left << std::setw(32) << ("平均有效簇数: " + std::to_string(avgPeriodicClusterNum).substr(0, 5) + " 个") << " | "
        << std::left << std::setw(30) << ("平均入簇比率: " + std::to_string(avgPeriodicClusteringRatio * 100.0).substr(0, 5) + " %");
  os << line1.str() << "\n";

  std::ostringstream line2;
  line2 << std::fixed << std::setprecision(3);
  std::ostringstream lifetime; lifetime << std::fixed << std::setprecision(3) << avgLifetime;
  std::ostringstream avgClusterSize; avgClusterSize << std::fixed << std::setprecision(2) << avgPeriodicClusterSize;

  std::ostringstream switchRate;
  switchRate << std::fixed << std::setprecision(2) << GetClusterSwitchRate();

  line2 << std::left << std::setw(30) << ("平均簇寿命: " + lifetime.str() + " s") << " | "
        << std::left << std::setw(32) << ("平均簇规模: " + avgClusterSize.str() + " 个") << " | "
        << std::left << std::setw(30) << ("平均换簇次数: " + switchRate.str() + " 次");
  os << line2.str() << "\n";

  std::ostringstream line3;
  line3 << std::fixed << std::setprecision(2);

  std::ostringstream roleChange;
  roleChange << std::fixed << std::setprecision(2) << GetRoleChangeRate();
  line3 << std::left << std::setw(30) << ("平均角色变换: " + roleChange.str() + " 次");
  os << line3.str() << "\n";
}

void PerformanceMetrics::PrintComprehensiveReport(std::ostream& os, const std::string& algorithm, uint16_t nodeCount, double simTime)
{
  PrintComprehensiveReport(os, algorithm, nodeCount, simTime, "kbps");
}

void PerformanceMetrics::PrintComprehensiveReport(std::ostream& os, const std::string& algorithm, uint16_t nodeCount, double simTime, const std::string& controlBandwidthUnit)
{
  os << "++++++++++++++++++++++ 性能测试结果 | 算法: " << algorithm << " +++++++++++++++++++++++\n";
  os << "节点数: " << std::setw(5) << nodeCount  << " | 仿真时长: " << std::setw(8) << std::fixed << std::setprecision(2) << simTime << " s\n";
  os << "=============================== 通用性能指标 =================================\n";
  std::ostringstream lineA;
  lineA << std::fixed;
  std::ostringstream throughput; throughput << std::fixed << std::setprecision(3) << m_commonMetrics.throughput;
  std::ostringstream pdr; pdr << std::fixed << std::setprecision(2) << m_commonMetrics.pdr;
  std::ostringstream delay; delay << std::fixed << std::setprecision(2) << m_commonMetrics.avgDelay;  
  lineA << std::left << std::setw(25) << ("吞吐量: " + throughput.str() + " kbps") <<  " | "
        << std::left << std::setw(23) << ("PDR: " + pdr.str() + " %")  <<  " | "
        << std::left << std::setw(30) << ("平均时延: " + delay.str() + " ms");
  os << lineA.str() << "\n";

  std::ostringstream lineB;
  lineB << std::fixed << std::setprecision(2);
  std::ostringstream hops; hops << std::fixed << std::setprecision(2) << m_commonMetrics.avgHops;
  
  // std::ostringstream flows;
  // flows << m_commonMetrics.validFlows << "/" << m_testFlows.size();
  std::ostringstream AvergeLinkTime;
  if(m_commonMetrics.AvergeLinkLength > 0) {AvergeLinkTime << m_commonMetrics.AvergeLinkLength << " s";}
  else{  AvergeLinkTime << CalculateAverageLinkLifetime() << " s";}
  lineB << std::left << std::setw(26) << ("平均跳数: " + hops.str())    <<  " | "
        // << std::left << std::setw(27) << ("有效流: " + flows.str() + " 对") <<  " | "
        
        << std::left << std::setw(30) << ("平均链路寿命: " + AvergeLinkTime.str());
  os << lineB.str() << "\n";

  std::ostringstream lineC;
  lineC << std::fixed << std::setprecision(3);
  std::ostringstream conThp; conThp << std::fixed <<std::setprecision(3) << m_commonMetrics.ControlPacketTHP;
  std::ostringstream overheadRatio; overheadRatio << std::fixed << std::setprecision(3) << m_commonMetrics.controlOverheadRatio;
  std::ostringstream helloRatio; helloRatio<<std::fixed<<std::setprecision(3) << (double)m_commonMetrics.helloPacketCount / m_commonMetrics.controlPacketNum;
  lineC << std::left << std::setw(26) << ("控制带宽: " + conThp.str() + " " + controlBandwidthUnit) << " | "
        << std::left << std::setw(26) << ("Hello包占比: " + helloRatio.str())    << " | "
        << std::left << std::setw(30) << ("控制开销比: " + overheadRatio.str());
        
  os << lineC.str() << "\n";

  std::ostringstream lineD;
  lineD << std::fixed << std::setprecision(2);  
  std::ostringstream totalEnergy; totalEnergy << std::fixed << std::setprecision(2) << m_commonMetrics.totalEnergyConsumed;
  std::ostringstream avgEnergy; avgEnergy << std::fixed << std::setprecision(2) << m_commonMetrics.avgEnergyPerNode;
  std::ostringstream deadNode;
  deadNode << m_commonMetrics.deadNodes << "/" << nodeCount;  
  lineD << std::left << std::setw(25) << ("总能耗: " + totalEnergy.str() + " J") << " | "
        << std::left << std::setw(27) << ("平均能耗: " + avgEnergy.str() + " J") << " | "
        << std::left << std::setw(30) << ("殆尽节点: " + deadNode.str());
  os << lineD.str() << "\n";
  if (IsClusteringAlgorithm(algorithm)) { PrintClusteringMetrics(os); }
}


void PerformanceMetrics::SaveToCSV(const std::string& filename, const std::string& algorithm, uint32_t seed, uint16_t nodeCount, double minSpeed, double maxSpeed)
{
  SaveToCSV(filename, algorithm, seed, nodeCount, minSpeed, maxSpeed, true);
}

void PerformanceMetrics::SaveToCSV(const std::string& filename, const std::string& algorithm, uint32_t seed, uint16_t nodeCount, double minSpeed, double maxSpeed, bool includeComputeTasks)
{
  std::ofstream csvFile;
  bool writeHeader = !std::ifstream(filename).good();
  csvFile.open(filename, std::ios::app);

  if (!csvFile.is_open()) {
    NS_LOG_ERROR("Failed to open CSV file: " << filename);
    return;
  }

  if (writeHeader) {  // 写表头（仅第一次）
    csvFile << "Algorithm,Nodes,Speed(m/s),Seed,PDR(%),AvgDelay(ms),AvgHops,ValidFlows,"
            << "Throughput(kbps),OverHeadRatio,ControlThroughput,HelloRatio,AvgLinkLen(s),TotalEnergy(J),AvgEnergy(J),"
            << "DeadNodes,IsClusteringAlgo,TotalClusterNum,"
            << "AvgClusterNum,AvgClusteringRatio(%),AvgClusterSize,"
            << "AvgClusterLifetime(s),AvgClusterSwitchCount(/node),AvgRoleChangeCount(/node)";
    if (includeComputeTasks) {
      csvFile << ",TasksGenerated,TasksCompleted,TaskCompletionRate(%),"
              << "AvgResponseTime(ms),AvgProcessingDelay(ms),AvgQueueWaitTime(ms),DeadlineMissRate(%)";
    }
    csvFile << "\n";
  }
  // 通用字段
  double AvgLinkLength = 0.0;
  double HelloRatio = 0.0;
  HelloRatio = (double)m_commonMetrics.helloPacketCount / (double)m_commonMetrics.controlPacketNum;
  if (m_commonMetrics.AvergeLinkLength > 0)
  {
    AvgLinkLength = m_commonMetrics.AvergeLinkLength;
  }
  else{  AvgLinkLength= CalculateAverageLinkLifetime(); }

  // 构建速度范围字符串
  std::string speedRange;
  if (minSpeed > 0 || maxSpeed > 0) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(0) << minSpeed << "~" << maxSpeed;
    speedRange = oss.str();
  } else {
    speedRange = "N/A";
  }

  csvFile << algorithm << "," << nodeCount << "," << speedRange << "," << seed <<","
          << std::fixed << std::setprecision(2) 
          << m_commonMetrics.pdr << ","
          << m_commonMetrics.avgDelay << ","
          << m_commonMetrics.avgHops << ","
          << m_commonMetrics.validFlows << ","
          << m_commonMetrics.throughput << ","
          << std::setprecision(6) << m_commonMetrics.controlOverheadRatio << ","
          << std::setprecision(6) << m_commonMetrics.ControlPacketTHP<<","
          << std::setprecision(4) << HelloRatio << ","
          << std::setprecision(4) << AvgLinkLength <<","
          << std::setprecision(2) << m_commonMetrics.totalEnergyConsumed << ","
          << m_commonMetrics.avgEnergyPerNode << ","
          << m_commonMetrics.deadNodes << ",";  
  // 分簇算法字段
  if (IsClusteringAlgorithm(algorithm)) {
    csvFile << "Yes,";
    Ptr<ClusterManager> clusterMgr = ClusterManager::GetInstance();
    uint32_t totalClusters, activeCount;
    double avgLifetime, avgSize, activeAvgSize, activeAvgSizePro;
    clusterMgr->GetClusteringStatistics(totalClusters, avgLifetime, avgSize);
    clusterMgr->GetActiveClusterStats(activeCount, activeAvgSize, activeAvgSizePro);

    // 计算周期性统计的平均值
    double avgPeriodicClusterNum = 0.0;
    double avgPeriodicClusteringRatio = 0.0;
    double avgPeriodicClusterSize = 0.0;

    if (!m_clusteringStats.periodicClusterCount.empty()) {
      for (auto count : m_clusteringStats.periodicClusterCount) {
        avgPeriodicClusterNum += count;
      }
      avgPeriodicClusterNum /= m_clusteringStats.periodicClusterCount.size();
    }

    if (!m_clusteringStats.periodicClusteringRatio.empty()) {
      for (auto ratio : m_clusteringStats.periodicClusteringRatio) {
        avgPeriodicClusteringRatio += ratio;
      }
      avgPeriodicClusteringRatio /= m_clusteringStats.periodicClusteringRatio.size();
    }

    if (!m_clusteringStats.periodicAvgClusterSize.empty()) {
      for (auto size : m_clusteringStats.periodicAvgClusterSize) {
        avgPeriodicClusterSize += size;
      }
      avgPeriodicClusterSize /= m_clusteringStats.periodicAvgClusterSize.size();
    }

    csvFile << totalClusters << ","
            << std::setprecision(2) << avgPeriodicClusterNum << ","
            << avgPeriodicClusteringRatio * 100.0 << ","
            << avgPeriodicClusterSize << ","
            << avgLifetime << ","
            << GetClusterSwitchRate() << ","
            << GetRoleChangeRate();
  } else {
    csvFile << "No,N/A,N/A,N/A,N/A,N/A,N/A,N/A";
  }

  if (includeComputeTasks) {
    csvFile << "," << m_commonMetrics.tasksGenerated << ","
            << m_commonMetrics.tasksCompleted << ","
            << std::fixed << std::setprecision(2) << m_commonMetrics.taskCompletionRate << ","
            << m_commonMetrics.avgResponseTime << ","
            << m_commonMetrics.avgProcessingDelay << ","
            << m_commonMetrics.avgQueueWaitTime << ","
            << m_commonMetrics.deadlineMissRate;
  }
  csvFile << "\n";

  csvFile.close();
}


bool PerformanceMetrics::IsClusteringAlgorithm(const std::string& algorithm) const
{
  return (algorithm == "icra"    ||
          algorithm == "cafbcr"  || algorithm == "ca-bcr"  ||
          algorithm == "ca-fbcr" || algorithm == "ca-qbcr");
}



} // namespace ns3
