/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef CAFBCR_FUZZY_HELLO_H
#define CAFBCR_FUZZY_HELLO_H

#include "ns3/object.h"
#include "ns3/nstime.h"
#include "ns3/vector.h"
#include "cafbcr-fuzzy-logic.h"
#include "ns3/fanet-neighbor.h"
#include <map>
#include <deque>

namespace ns3 {
namespace cafbcr {

/**
 * \brief Fuzzy-logic Hello-interval manager.
 *
 * Uses a Mamdani inference engine to adapt the Hello interval:
 * 1. collect local context (speed, neighbours, energy, link stability);
 * 2. fuzzify the input metrics;
 * 3. evaluate the fuzzy rules;
 * 4. defuzzify to the optimal Hello interval.
 */
class FuzzyHelloManager : public Object
{
public:
  static TypeId GetTypeId();
  FuzzyHelloManager();
  virtual ~FuzzyHelloManager();

  /**
   * \brief Link-stability metrics.
   */
  struct LinkStability {
    double relativeVelocity;    ///< Mean relative velocity.
    double directionDivergence; ///< Direction divergence.
    double distanceRisk;        ///< Distance-risk factor.
    double predictedLifetime;   ///< Predicted link lifetime in seconds.
  };

  /**
   * \brief Cached beacon sample.
   */
  struct BeaconCache {
    Vector position;        ///< Node position.
    Vector velocity;        ///< Node velocity.
    Time timestamp;         ///< Timestamp.
  };

  /**
   * \brief Return the current optimal Hello interval.
   *
   * \return Hello interval in the range 0.25-2.0 s.
   */
  Time GetOptimalHelloInterval();

  /**
   * \brief Set the node ID.
   * \param id Node ID.
   */
  void SetNodeId(uint16_t id) { m_nodeId = id; }

  /**
   * \brief Set the neighbor table.
   * \param neighbors Neighbor table.
   */
  void SetNeighborTable(Ptr<Neighbors> neighbors) { m_neighbors = neighbors; }

  /**
   * \brief Process a received Hello.
   *
   * \param senderId Sender ID.
   * \param position Sender position.
   * \param velocity Sender velocity.
   * \param timestamp Reception timestamp.
   */
  void ProcessHelloInfo(uint16_t senderId, Vector position, Vector velocity, Time timestamp);

  /**
   * \brief Set the maximum communication range.
   * \param range Range in meters.
   */
  void SetMaxRange(double range) { m_maxRange = range; }

private:
  friend class CafbcrFuzzyLifetimeTestCase;

  /**
   * \brief Initialize the fuzzy inference engine.
   */
  void InitializeFuzzyEngine();

  /**
   * \brief Initialize the input variables.
   */
  void InitializeInputVariables();

  /**
   * \brief Initialize the output variables.
   */
  void InitializeOutputVariables();

  /**
   * \brief Initialize the fuzzy rules.
   */
  void InitializeFuzzyRules();

  /**
   * \brief Evaluate link stability.
   * \return Link-stability metrics.
   */
  LinkStability EvaluateLinkStability();

  /**
   * \brief Compute the relative speed.
   * \param v1 First velocity.
   * \param v2 Second velocity.
   * \return Relative speed magnitude.
   */
  double CalculateRelativeVelocity(Vector v1, Vector v2);

  /**
   * \brief Compute direction divergence.
   * \return Divergence in [0, 1].
   */
  double CalculateDirectionDivergence();

  /**
   * \brief Estimate time to leave the nominal communication radius at constant
   *        relative velocity, capped at 10 s.
   * \param neighborId Neighbor node ID.
   * \return Predicted link lifetime in seconds.
   */
  double EstimateLinkLifetime(uint16_t neighborId);

  /**
   * \brief Predict a neighbor position from cached beacons.
   * \param nodeId Neighbor node ID.
   * \param targetTime Target time.
   * \return Predicted position.
   */
  Vector PredictPositionWithBeacons(uint16_t nodeId, Time targetTime);

  /**
   * \brief Normalize the neighbor count.
   * \param neighborCount Number of neighbors.
   * \return Normalized value in [0, 1].
   */
  double NormalizeNeighborCount(uint16_t neighborCount);

  /**
   * \brief Return the remaining energy ratio.
   * \return Ratio in [0, 1].
   */
  double GetEnergyRatio();

  // Member variables.
  uint16_t                                    m_nodeId;           ///< Node ID.
  Ptr<Neighbors>                              m_neighbors;        ///< Neighbor table.
  MamdaniFuzzyEngine                          m_fuzzyEngine;      ///< Fuzzy inference engine.
  std::map<uint16_t, std::deque<BeaconCache>> m_beaconHistory;    ///< Beacon history.

  // Configuration parameters.
  double                                      m_maxRange;         ///< Maximum communication range in meters.
  uint16_t                                    m_maxNeighbors;     ///< Maximum neighbor count used for normalization.

  bool                                        m_initialized;      ///< Initialization flag.
};

} // namespace cafbcr
} // namespace ns3

#endif /* CAFBCR_FUZZY_HELLO_H */
