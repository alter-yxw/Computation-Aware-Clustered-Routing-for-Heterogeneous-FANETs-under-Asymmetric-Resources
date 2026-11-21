/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef PERFORMANCE_METRICS_H
#define PERFORMANCE_METRICS_H

#include "ns3/object.h"
#include "ns3/nstime.h"
#include "ns3/node-container.h"
#include "ns3/flow-monitor.h"
#include "ns3/ipv4-flow-classifier.h"
#include <map>
#include <vector>
#include <fstream>

namespace ns3 {

class PerformanceMetrics : public Object
{
public:
  static TypeId GetTypeId();
  PerformanceMetrics();
  virtual ~PerformanceMetrics() {}
  // ===== Data collection =====
  void SetNodes(NodeContainer nodes) { m_nodes = nodes; }
  void SetFlowMonitor(Ptr<FlowMonitor> monitor, Ptr<Ipv4FlowClassifier> classifier);
  void SetTestFlows(const std::vector<std::tuple<uint16_t, uint16_t, uint16_t, Ipv4Address, Ipv4Address>>& flows);

  void CollectClusteringMetrics();  // Periodic clustering snapshot.
  void CollectLinkMetrics();
  // ===== Metric computation =====
  // Routing and transmission metrics.
  double    CalculateAverageLinkLifetime();   // Average link lifetime in seconds.
  // Clustering metrics.
  uint32_t  GetClusterCount();                // Number of clusters.
  double    GetAverageClusterSize();          // Mean cluster size.
  double    GetNodeClusteringRatio();         // Fraction of clustered nodes.
  double    GetAverageClusterLifetime();      // Mean cluster lifetime in seconds.
  double    GetClusterSwitchRate();           // Cluster switches per node.
  double    GetRoleChangeRate();              // Role changes per node.
  // ===== Reporting =====
  void PrintClusteringMetrics(std::ostream& os);
  void PrintComprehensiveReport(std::ostream& os, const std::string& algorithm, uint16_t nodeCount, double simTime);
  void PrintComprehensiveReport(std::ostream& os, const std::string& algorithm, uint16_t nodeCount, double simTime, const std::string& controlBandwidthUnit);
  void SaveToCSV(const std::string& filename, const std::string& algorithm, uint32_t seed, uint16_t nodeCount, double minSpeed = 0.0, double maxSpeed = 0.0);
  // Legacy calls keep the compute-task columns enabled by default.
  void SaveToCSV(const std::string& filename, const std::string& algorithm, uint32_t seed, uint16_t nodeCount, double minSpeed, double maxSpeed, bool includeComputeTasks);


  struct CommonMetrics {
    double    pdr;
    double    avgDelay;
    uint64_t  avgHops;
    uint32_t  validFlows;
    double    totalEnergyConsumed;
    double    avgEnergyPerNode;
    uint16_t  deadNodes;
    double    throughput;
    uint64_t  controlPacketNum;
    uint64_t  helloPacketCount;
    double    ControlPacketTHP;
    double    controlOverheadRatio;
    double    AvergeLinkLength;

    // End-to-end compute-task metrics.
    uint32_t  tasksGenerated      = 0;
    uint32_t  tasksCompleted      = 0;
    double    taskCompletionRate  = 0.0;  // Percent.
    double    avgResponseTime     = 0.0;  // Milliseconds (network round trip + queue + service).
    double    avgProcessingDelay  = 0.0;  // Milliseconds (workload / computeCapacity).
    double    avgQueueWaitTime    = 0.0;  // Milliseconds.
    double    deadlineMissRate    = 0.0;  // Percent.
  };
  void SetCommonMetrics(const CommonMetrics& metrics) { m_commonMetrics = metrics;}
  bool IsClusteringAlgorithm(const std::string& algorithm) const;

private:
  NodeContainer             m_nodes;
  Ptr<FlowMonitor>          m_flowMonitor;
  Ptr<Ipv4FlowClassifier>   m_classifier;
  std::vector<std::tuple<uint16_t, uint16_t, uint16_t, Ipv4Address, Ipv4Address>> m_testFlows;

  struct LinkInfo {  // Link statistics.
    uint16_t  sourceNodeId;      // Source node ID.
    uint16_t  targetNodeId;      // Neighbor node ID.
    Time      creationTime;
    Time      lastSeenTime;
    bool      isAlive;
  };
  std::map<std::pair<uint16_t, uint16_t>, LinkInfo> m_linkHistory;

  std::vector<double> m_linkLifetimes;
  struct TransmissionStats {
    uint64_t  totalTxPackets;
    uint64_t  totalRxPackets;
    uint64_t  totalRxBytes;
    uint32_t  totalHops;
    uint64_t  totalControlPackets;
    double    transmissionDuration;
  } m_transmissionStats;

  struct ClusteringStats {  // Clustering statistics.
    std::map<uint32_t, Time>     clusterCreationTime;         // Cluster creation time.
    std::map<uint32_t, Time>     clusterDestructionTime;      // Cluster dissolution time.
    std::map<uint32_t, uint32_t> nodeClusterSwitchCount;      // Per-node cluster-switch count.
    std::map<uint32_t, uint32_t> nodeRoleChangeCount;         // Per-node role-change count.
    std::vector<uint32_t>        clusterSizeHistory;          // Cluster-size history.
    uint32_t                     totalClustersCreated;

    // Periodic samples recorded by CollectClusteringMetrics.
    std::vector<uint32_t>        periodicClusterCount;        // Periodic cluster-count samples.
    std::vector<double>          periodicClusteringRatio;     // Periodic clustering-ratio samples.
    std::vector<double>          periodicAvgClusterSize;      // Periodic mean-cluster-size samples.
  };

  ClusteringStats                m_clusteringStats;
  // Previous sample, used to detect changes.
  std::map<uint16_t, uint16_t>   m_lastNodeCluster;          // Previous cluster of each node.
  std::map<uint16_t, uint8_t>    m_lastNodeRole;             // Previous role of each node.

  CommonMetrics                  m_commonMetrics;
};

} // namespace ns3

#endif // PERFORMANCE_METRICS_H