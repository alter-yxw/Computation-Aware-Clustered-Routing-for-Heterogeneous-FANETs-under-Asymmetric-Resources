/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef CAFBCR_QHELLO_H
#define CAFBCR_QHELLO_H

#include "ns3/fanet-neighbor.h"
#include "ns3/nstime.h"
#include "ns3/object.h"
#include "ns3/random-variable-stream.h"
#include "ns3/vector.h"

#include <array>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace ns3 {
class Packet;
class Ipv4Header;
namespace cafbcr {

/** Online, node-local Hello scheduling. No evaluator PDR or remote live mobility is used. */
class QLearningHelloManager : public Object
{
public:
  static TypeId GetTypeId();
  QLearningHelloManager();

  /** Per-window observations. NaN denotes an unavailable measurement, not zero loss. */
  struct Environment {
    double routingFailureRatio = std::numeric_limits<double>::quiet_NaN();
    double neighborLossRate = std::numeric_limits<double>::quiet_NaN(); // fraction/s, capped at 1
    double motionPredictionError = std::numeric_limits<double>::quiet_NaN(); // normalized by 0.1 R
    double linkAgeRisk = std::numeric_limits<double>::quiet_NaN(); // interval / start-of-window LET
    double normalizedHelloRate = std::numeric_limits<double>::quiet_NaN(); // 0.25 s / interval
    uint64_t routingSamples = 0; // UDP business route queries, NOT deliveries or MAC acknowledgements
  };

  struct LinkStability {
    double relativeVelocity;
    double directionDivergence;
    double distanceRisk;
    double predictedLifetime; // minimum cached-neighbor 3-D LET, bounded to [0, 60] seconds
  };

  struct State {
    uint8_t stabilityLevel;
    uint8_t densityLevel;
    uint8_t lifelengthLevel;
    uint8_t energyLevel;

    uint32_t ToIndex() const {
      return stabilityLevel * 1000 + densityLevel * 100 + lifelengthLevel * 10 + energyLevel;
    }
    bool operator<(const State& other) const { return ToIndex() < other.ToIndex(); }
  };

  enum Action {
    HELLO_VERY_FAST = 0, // 0.25 s
    HELLO_FAST = 1,      // 0.5 s
    HELLO_NORMAL = 2,    // 0.75 s
    HELLO_SLOW = 3,      // 1 s
    HELLO_VERY_SLOW = 4, // 2 s
    NUM_ACTIONS = 5
  };

  struct Statistics {
    uint64_t decisions = 0;
    uint64_t feedbackWindows = 0;
    uint64_t updates = 0;
    uint64_t exploratoryDecisions = 0;
    uint64_t learnedDecisions = 0;
    uint64_t receivedHellos = 0;
    uint64_t routingObservations = 0;
    std::array<uint64_t, NUM_ACTIONS> actions{};
    double lastReward = 0;
    double lastExplorationRate = 0;
  };

  /** Settle a completed action BEFORE selecting its successor; early polls preserve the action. */
  Time GetOptimalHelloInterval();
  /** Consume a pending, completed action at most once. Normally called by the method above. */
  void UpdateQLearning(const Environment& env);
  void ObserveRoutingResult(bool usableRoute);
  void ProcessHelloInfo(uint16_t senderId, Vector position, Vector velocity, Time timestamp);

  void SetNodeId(uint16_t id) { m_nodeId = id; }
  void SetNeighborTable(Ptr<Neighbors> neighbors) { m_neighbors = neighbors; }
  void SetMaxRange(double range) { m_maxRange = range; }
  /** Receiver's existing neighbor TTL minus a reception margin; applies only to this controller. */
  void SetIntervalLimit(Time limit);
  int64_t AssignStreams(int64_t stream);

  const Statistics& GetStatistics() const { return m_statistics; }
  const Environment& GetLastEnvironment() const { return m_lastEnvironment; }
  State GetLastState() const { return m_currentState; }
  Action GetLastAction() const { return m_lastAction; }
  double GetQValue(const State& state, Action action) const;
  static Time GetActionInterval(Action action);
  /** Only for received IPv4 payloads; source queries use CA's existing control-send scope. */
  static bool IsReceivedControlPacket(Ptr<const Packet> packet, const Ipv4Header& header);
  /** Zero-wire-byte origin marker also survives deferred source route queries. */
  static void MarkOwnControlPacket(Ptr<Packet> packet);
  static bool IsOwnControlPacket(Ptr<const Packet> packet);
  /** This CA experiment's business is UDP. ICMP errors never become business rewards. */
  static bool IsBusinessPacket(Ptr<const Packet> packet, const Ipv4Header& header, bool received);
  /** Effective TypeId defaults, without constructing another RNG or node. */
  static std::string ConfigurationJson();
  /** Exact constant-velocity exit from a sphere, using locally cached relative kinematics. */
  static double EstimateLinkLifetime(Vector relativePosition, Vector relativeVelocity, double range);

private:
  friend class CafbcrHardfixTestCase;
  // Retain the original hardfix RNG test's unrestricted action draw API.
  Action SelectAction(const State& state) {
    return SelectAction(state, {HELLO_VERY_FAST, HELLO_FAST, HELLO_NORMAL, HELLO_SLOW, HELLO_VERY_SLOW});
  }
  std::map<uint16_t, Neighbors::NeighborInfo> LiveNeighbors() const;
  bool ReadKinematics(Vector& position, Vector& velocity) const;
  double ReadEnergyRatio() const;
  LinkStability EvaluateLinkStability();
  State GetCurrentState(const LinkStability& stability);
  std::vector<Action> GetAllowedActions(const LinkStability& stability);
  Action SelectAction(const State& state, const std::vector<Action>& allowed);
  void BeginObservation(const LinkStability& stability);
  Environment CollectEnvironment();
  double CalculateReward(const Environment& env) const;
  void UpdateQValue(const Environment& env, const State& nextState,
                    const std::vector<Action>& allowed);
  uint8_t QuantifyLinkDynamics(const LinkStability& stability) const;
  uint8_t QuantifyDensity(uint16_t count) const;
  uint8_t QuantifyStability(double lifetime) const;
  uint8_t QuantifyEnergy(double ratio) const;

  std::map<State, std::vector<double>> m_qTable;
  std::map<State, std::array<uint64_t, NUM_ACTIONS>> m_visits;
  double m_alpha;
  double m_gamma; // discount per 0.25 s, not per Hello decision
  double m_epsilon;
  double m_minEpsilon;
  bool m_learningEnabled;
  Time m_maxInterval;
  Time m_intervalLimit;

  State m_currentState;
  Action m_lastAction;
  Time m_lastActionTime;
  bool m_pendingAction;
  uint16_t m_nodeId;
  Ptr<Neighbors> m_neighbors;
  double m_maxRange;
  Ptr<UniformRandomVariable> m_rand;

  std::set<uint16_t> m_startNeighbors;
  Vector m_startPosition;
  Vector m_startVelocity;
  bool m_startKinematicsValid;
  double m_startLifetime;
  uint64_t m_windowRouteFailures;
  uint64_t m_windowRouteSamples;
  double m_windowPredictionError;
  uint64_t m_windowPredictionSamples;

  struct HelloCache {
    Vector position;
    Vector velocity;
    Time timestamp;
  };
  std::map<uint16_t, HelloCache> m_helloHistory;
  Statistics m_statistics;
  Environment m_lastEnvironment;
};

} // namespace cafbcr
} // namespace ns3

#endif /* CAFBCR_QHELLO_H */
