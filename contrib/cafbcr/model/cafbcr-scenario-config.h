/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef CAFBCR_SCENARIO_CONFIG_H
#define CAFBCR_SCENARIO_CONFIG_H

#include "ns3/object.h"
#include "ns3/vector.h"
#include "ns3/ipv4-address.h"
#include <string>
#include <vector>
#include <map>
#include <cstdint>

namespace ns3 {
namespace cafbcr {

/**
 * \brief UAV node types.
 */
enum UavNodeType {
  COMMAND = 0,   ///< Command node with the highest compute capacity (80 GFLOP/s).
  RELAY   = 1,   ///< Relay node with medium compute capacity (20 GFLOP/s).
  TASK    = 2,   ///< Task node with low compute capacity (5 GFLOP/s); generates and processes tasks.
  SWARM   = 3    ///< Swarm node with the lowest compute capacity (0.5 GFLOP/s); generates tasks only.
};

/**
 * \brief Per-type UAV configuration.
 *
 * Compute capacity, radio parameters and task profile for one node type.
 */
struct UavTypeConfig {
  // Identity.
  UavNodeType nodeType;
  std::string name;

  // Compute capacity.
  double computeCapacity;         ///< GFLOP/s
  uint32_t taskQueueCapacity;     ///< Maximum task-queue length.

  // Energy.
  double batteryCapacity;         ///< Joules.
  double idlePowerConsumption;    ///< Idle power in watts.
  double transmitPowerPerGhz;     ///< TX power in watts per GHz.
  double movementPower;           ///< Mobility power in watts.

  // Radio.
  double maxCommunicationRange;   ///< Maximum communication range in meters.
  double dataRate;                ///< Data rate in bits per second.

  // Application profile.
  bool generateTasks;             ///< Whether the node generates tasks.
  bool processTasks;              ///< Whether the node processes tasks.
  double taskGenerationInterval;  ///< Task-generation interval in seconds.
  double minTaskWorkload;         ///< Minimum task workload in GFLOP.
  double maxTaskWorkload;         ///< Maximum task workload in GFLOP.
  uint32_t minTaskDataSize;       ///< Minimum task data size in bytes.
  uint32_t maxTaskDataSize;       ///< Maximum task data size in bytes.
  double minTaskDeadline;         ///< Minimum task deadline in seconds.
  double maxTaskDeadline;         ///< Maximum task deadline in seconds.

  /**
   * \brief Return the node type as a string.
   */
  std::string GetNodeTypeString() const;

  /**
   * \brief Create the predefined configuration for each node type.
   */
  static UavTypeConfig CreateCommandNode();
  static UavTypeConfig CreateRelayNode();
  static UavTypeConfig CreateTaskNode();
  static UavTypeConfig CreateSwarmNode();
};

/**
 * \brief Simulation scenario configuration.
 *
 * Covers the network topology (node counts and distribution), CA-FBCR protocol
 * parameters, application parameters and simulation settings.
 */
class ScenarioConfig : public Object {
public:
  static TypeId GetTypeId();

  ScenarioConfig();
  virtual ~ScenarioConfig();

  /**
   * Network configuration.
   */
  // Node configuration.
  void AddNodeType(uint32_t count, const UavTypeConfig& config);
  uint32_t GetNodeCount() const;
  const std::vector<UavTypeConfig>& GetNodeConfigs() const;
  uint32_t GetNodeCountByType(UavNodeType type) const;
  const UavTypeConfig& GetNodeConfigByType(UavNodeType type) const;

  // Network parameters.
  void SetSimulationArea(Vector size);
  Vector GetSimulationArea() const { return m_simulationArea; }

  void SetSimulationTime(double seconds);
  double GetSimulationTime() const { return m_simulationTime; }

  /**
   * CA-FBCR routing protocol configuration.
   */
  void SetHelloInterval(double seconds);
  double GetHelloInterval() const { return m_helloInterval; }

  void SetBeaconInterval(double seconds);
  double GetBeaconInterval() const { return m_beaconInterval; }

  void SetClusteringTime(double seconds);
  double GetClusteringTime() const { return m_clusteringTime; }

  void SetMaxCommunicationRange(double meters);
  double GetMaxCommunicationRange() const { return m_maxCommRange; }

  // Fitness weights, in the same order as ClusterNode::ComputeFitness:
  // LHTP, Energy, Degree, Distance, Compute.
  void SetFitnessWeights(double wLHTP, double wEnergy,
                        double wDegree, double wDistance, double wCompute);
  void GetFitnessWeights(double& wLHTP, double& wEnergy,
                        double& wDegree, double& wDistance, double& wCompute) const;

  /**
   * Mobility configuration.
   */
  void SetMobilityModel(const std::string& model);
  std::string GetMobilityModel() const { return m_mobilityModel; }

  void SetMaxVelocity(double mps);
  double GetMaxVelocity() const { return m_maxVelocity; }

  void SetMinVelocity(double mps);
  double GetMinVelocity() const { return m_minVelocity; }

  /**
   * Energy-model configuration.
   */
  void SetEnergyModel(const std::string& model);
  std::string GetEnergyModel() const { return m_energyModel; }

  /**
   * \brief Validate the configuration.
   */
  bool Validate() const;

  /**
   * \brief Print a configuration summary.
   */
  void PrintSummary() const;

private:
  /**
   * Node-type count and configuration.
   */
  struct NodeTypeInfo {
    uint32_t count;
    UavTypeConfig config;
  };

  // Network configuration.
  std::vector<NodeTypeInfo> m_nodeTypes;
  Vector m_simulationArea;
  double m_simulationTime;

  // CA-FBCR protocol configuration.
  double m_helloInterval;
  double m_beaconInterval;
  double m_clusteringTime;
  double m_maxCommRange;

  // Fitness weights (ComputeFitness order: LHTP, Energy, Degree, Distance, Compute).
  double m_wLHTP;
  double m_wEnergy;
  double m_wDegree;
  double m_wDistance;
  double m_wCompute;

  // Mobility configuration.
  std::string m_mobilityModel;
  double m_maxVelocity;
  double m_minVelocity;

  // Energy-model configuration.
  std::string m_energyModel;
};

} // namespace cafbcr
} // namespace ns3

#endif // CAFBCR_SCENARIO_CONFIG_H
