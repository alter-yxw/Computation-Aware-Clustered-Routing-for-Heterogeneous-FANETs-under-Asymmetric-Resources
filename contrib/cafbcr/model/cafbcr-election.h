/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef CAFBCR_ELECTION_H
#define CAFBCR_ELECTION_H

#include "ns3/object.h"
#include "ns3/node.h"
#include "ns3/vector.h"
#include "ns3/nstime.h"
#include "ns3/fanet-node.h"
#include "ns3/fanet-clustermanager.h"
#include "ns3/random-variable-stream.h"
#include <vector>
#include <map>
#include <array>

namespace ns3 {
namespace cafbcr {

/**
 * \brief CA-FBCR cluster-head election based on artificial bee colony (ABC).
 *
 * Uses the three ABC phases (employed, onlooker and scout bees) to select the
 * cluster-head set iteratively. Candidate fitness combines LHTP, energy,
 * degree, distance and compute-capacity weights.
 */
class cafbcrElection : public Object
{
public:
  enum InitialSelector { ABC, TOPK };
  const char* GetInitialSelectorName() const { return m_initialSelector == TOPK ? "topk" : "abc"; }
  struct InitialNeighbor
  {
    uint16_t key;
    Neighbors::NeighborInfo state;
    Neighbors::ComputeObservation compute;
  };
  struct InitialCandidate
  {
    uint16_t nodeId = 0, networkScale = 0, degree = 0;
    NodeRole role = Idle;
    ClusterNode::FitnessMode fitnessMode = ClusterNode::HETEROGENEOUS_COMPUTE;
    bool eligible = false, scored = false, selected = false, lhtpUseMaxRange = false;
    double fitness0 = 0, cachedFitness = 0, energy = 0, originEnergy = 0;
    double computeCapacity = 0, load = 0, maxRange = 0;
    Vector position, velocity;
    std::vector<InitialNeighbor> neighbors;
  };
  struct InitialSelectionRecord
  {
    int64_t simulationNs = 0;
    uint16_t nodes = 0, requestedHeads = 0;
    std::array<double, 5> weights{};
    std::vector<InitialCandidate> candidates;
  };
  bool IsInitialSelectionObserved() const { return m_observeInitialSelection; }
  const std::vector<InitialSelectionRecord>& GetInitialSelectionRecords() const { return m_initialSelections; }

  // Counts cover this object's StartElection calls only. They do not model
  // CPU energy or information-exchange traffic, and do not advance simulation time.
  struct CostRecord
  {
    const char* selector = "abc";
    uint64_t invocation = 0;
    uint32_t nodes = 0, requestedHeads = 0;
    int64_t simulationStartNs = 0, simulationEndNs = 0;
    bool initializedBefore = false;
    const char* outcome = "completed";
    uint64_t hostWallNs = 0, setupHostNs = 0, searchHostNs = 0, commitHostNs = 0;
    uint64_t initializationCalls = 0, initializationCandidateVisits = 0, initializationStateVisits = 0;
    uint64_t backupScanVisits = 0, selectionRounds = 0;
    uint64_t stateSaveVisits = 0, stateRestoreVisits = 0;
    uint64_t initBeeVisits = 0, initBeeFitnessCalls = 0;
    uint64_t employedVisits = 0, employedNeighborVisits = 0, employedNoNeighborFallbacks = 0;
    uint64_t employedFitnessCalls = 0;
    uint64_t onlookerVisits = 0, onlookerTrials = 0, onlookerSelectedTrials = 0, onlookerFitnessCalls = 0;
    uint64_t probabilityCalls = 0, probabilityVisits = 0;
    uint64_t bestScanCalls = 0, bestScanVisits = 0;
    uint64_t scoutScanVisits = 0, scoutReinitializations = 0, cycleIterations = 0;
    uint64_t commitDuplicateVisits = 0, commitCandidateVisits = 0, committedHeads = 0;
    uint64_t topkCandidateVisits = 0, topkComparisons = 0;
    uint64_t FitnessCalls() const
    {
      return initBeeFitnessCalls + employedFitnessCalls + onlookerFitnessCalls;
    }
  };
  bool IsCostObservationEnabled() const { return m_observeCost; }
  const std::vector<CostRecord>& GetCostRecords() const { return m_costRecords; }

private:
  friend class CafbcrHardfixTestCase;
  Ptr<UniformRandomVariable> m_rand;
  uint16_t                        m_MaxClusterNumber;     ///< Number of clusters, K = total nodes / 10.
  uint16_t                        m_NodesNumber;          ///< Total number of nodes.
  std::vector<uint16_t>           m_ClusterHeadsId;       ///< Cluster-head node IDs.
  uint16_t                        m_MAX_CYCLES;           ///< Maximum ABC iterations (default 10).
  double                          WeightTable[5];         ///< Weights: LHTP, energy, degree, distance, compute (ComputeFitness order).
  std::vector<Ptr<ClusterNode>>   m_clusternodes;         ///< All nodes.
  std::vector<Ptr<ClusterNode>>   BackUpClusterHeads;     ///< Selected cluster heads.
  std::vector<double>             Fitness_Value;          ///< Per-node fitness values.
  std::vector<double>             Probabilitys;           ///< Onlooker selection probabilities.
  std::vector<uint16_t>           trail_count;            ///< No-improvement counters.
  bool                            m_abcInitialized;       ///< Whether ABC state has been initialized.

public:
  double                  global_best_value;      ///< Global best fitness.
  uint16_t                global_best_index;      ///< Global best node index.
  uint16_t                ClusterHeadCount;       ///< Current cluster-head count.
  double                  EnergyBound[2];         ///< Energy bounds [min, max].
  uint16_t                DegreeBound[2];         ///< Degree bounds [min, max].
  Vector                  MaxVel, MinVel;         ///< Velocity bounds.
  Vector                  MaxPos, MinPos;         ///< Position bounds.

  static TypeId GetTypeId();

  cafbcrElection();
  virtual ~cafbcrElection() {}
  int64_t AssignStreams(int64_t stream);

  /**
   * \brief Start the cluster-head election.
   *
   * Runs the ABC iterations and selects K cluster heads.
   */
  void StartElection();

  /**
   * \brief Initialize the ABC data structures.
   *
   * Lazy initialization, performed on the first StartElection call.
   */
  void Initialize();

  /**
   * \brief ABC: find the global best solution.
   *
   * Scan all nodes and keep the one with the highest fitness.
   */
  void GetGlobalBest();

  /**
   * \brief ABC: employed-bee phase.
   *
   * Search the neighborhood using X_new = X_old + phi * (X_old - X_neighbor).
   */
  void EmpolyBeePhase();

  /**
   * \brief ABC: compute onlooker selection probabilities from fitness.
   */
  void GetProbability();

  /**
   * \brief ABC: onlooker-bee phase.
   *
   * Select a promising source by probability and search its neighborhood.
   */
  void OnlookerBeePhase();

  /**
   * \brief ABC: scout-bee phase.
   *
   * Reinitialize sources that have shown no improvement for too long.
   */
  void ScoutBeePhase();

  /**
   * \brief Initialize the population and run one ABC iteration to select a head.
   */
  void InitPopulation();

  /**
   * \brief Initialize the fitness of one bee (node).
   * \param NodeIndex Node index.
   */
  void InitBee(uint16_t NodeIndex);

  /**
   * \brief Set the maximum number of clusters.
   */
  void SetMaxClusterNum(uint16_t Num) { m_MaxClusterNumber = Num; }

  /**
   * \brief Return the maximum number of clusters.
   */
  uint16_t GetMaxClusterNum() const { return m_MaxClusterNumber; }

  /**
   * \brief Return the total number of nodes.
   */
  uint16_t GetNodesNumber() const { return m_NodesNumber; }

  /**
   * \brief Return the selected cluster heads.
   */
  std::vector<Ptr<ClusterNode>> GetClusterHeads() { return BackUpClusterHeads; }

  /**
   * \brief Set the weight table.
   * \param Weight Array of five weights: LHTP, energy, degree, distance, compute.
   */
  void SetWeightTable(double *Weight)
  {
    for (size_t i = 0; i < sizeof(WeightTable) / sizeof(WeightTable[0]); i++)
    {
      WeightTable[i] = Weight[i];
    }
  }

private:
  class CostScope;
  InitialSelector m_initialSelector = ABC;
  bool m_observeInitialSelection = false;
  std::vector<InitialSelectionRecord> m_initialSelections;
  InitialSelectionRecord* m_activeInitialSelection = nullptr;
  void CaptureInitialSelection();
  void SelectTopK();
  bool m_observeCost = false;
  CostRecord* m_activeCost = nullptr;
  std::vector<CostRecord> m_costRecords;
  void CountCost(uint64_t CostRecord::*field, uint64_t amount = 1)
  {
    if (m_activeCost) (m_activeCost->*field) += amount;
  }
};

} // namespace cafbcr
} // namespace ns3

#endif /* CAFBCR_ELECTION_H */
