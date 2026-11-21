/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef CAFBCR_HELPER_H
#define CAFBCR_HELPER_H

#include "ns3/ipv4-routing-helper.h"
#include "ns3/node.h"
#include "ns3/node-container.h"
#include "ns3/object-factory.h"
#include "ns3/fanet-node.h"

namespace ns3 {

/**
 * \brief Helper that creates and installs the CA-FBCR routing protocol.
 *
 * Wraps protocol installation and the most common configuration knobs.
 */
class cafbcrHelper : public Ipv4RoutingHelper
{
public:
  /**
   * \brief Constructor.
   *
   * Defaults:
   * - K: NodeList::GetNNodes() / 10 (computed dynamically);
   * - HelloInterval: 1.0 s;
   * - BeaconInterval: 0.75 s;
   * - fitness weights: 0.25 each (LHTP, Velocity, Distance, Energy).
   */
  cafbcrHelper();

  /**
   * \brief Destructor.
   */
  virtual ~cafbcrHelper();

  /**
   * \brief Copy constructor.
   * \return A new cafbcrHelper instance.
   */
  cafbcrHelper* Copy() const;

  /**
   * \brief Create a CA-FBCR routing protocol instance on a node.
   *
   * \param node Node on which to install the protocol.
   * \return The created routing protocol.
   */
  virtual Ptr<Ipv4RoutingProtocol> Create(Ptr<Node> node) const;

  /**
   * \brief Set a routing protocol attribute.
   *
   * \param name Attribute name.
   * \param value Attribute value.
   *
   * Supported attributes include "HelloInterval", "BeaconInterval" and
   * "MaxRange".
   */
  void Set(std::string name, const AttributeValue& value);

  /**
   * \brief Set the Hello interval.
   * \param interval Hello interval (default 1.0 s).
   */
  void SetHelloInterval(Time interval);

  /**
   * \brief Select the Hello-interval optimization method.
   *
   * \param method One of "fixed" (default), "fuzzy" or "qlearning".
   */
  void SetHelloIntervalMethod(const std::string &method) {m_helloMethod = method;}

  /**
   * \brief Set the beacon interval.
   * \param interval Beacon interval (default 0.75 s).
   */
  void SetBeaconInterval(Time interval);

  /**
   * \brief Set the maximum communication range.
   * \param range Range in meters.
   */
  void SetMaxRange(double range);
  void SetFitnessMode(ClusterNode::FitnessMode mode) { m_fitnessMode = mode; }

  /**
   * \brief Set the fitness weights.
   *
   * \param wLHTP LHTP weight (default 0.25).
   * \param wEnergy Energy weight (default 0.25).
   * \param wDegree Degree weight (default 0.25).
   * \param wDistance Distance weight (default 0.25).
   * \param wCompute Compute-capacity weight (default 0.0).
   *
   * The weights should sum to 1.0 and follow the ComputeFitness order:
   * LHTP, Energy, Degree, Distance, Compute.
   */
  void SetFitnessWeights(double wLHTP, double wEnergy, double wDegree, double wDistance, double wCompute = 0.0);

  /**
   * \brief Assign RNG streams to the installed protocols.
   *
   * \param c Node container.
   * \param stream First stream number.
   * \return Number of streams consumed.
   */
  int64_t AssignStreams(NodeContainer c, int64_t stream);

private:
  ObjectFactory m_agentFactory;    ///< Object factory.
  ClusterNode::FitnessMode m_fitnessMode{ClusterNode::HETEROGENEOUS_COMPUTE};
  Time m_helloInterval;            ///< Hello interval.
  Time m_beaconInterval;           ///< Beacon interval.
  double m_maxRange;               ///< Maximum communication range.
  double m_wLHTP;                  ///< LHTP weight.
  double m_wEnergy;                ///< Energy weight.
  double m_wDegree;                ///< Degree weight.
  double m_wDistance;              ///< Distance weight.
  double m_wCompute;               ///< Compute-capacity weight.
  std::string m_helloMethod;
};

} // namespace ns3

#endif /* CAFBCR_HELPER_H */
