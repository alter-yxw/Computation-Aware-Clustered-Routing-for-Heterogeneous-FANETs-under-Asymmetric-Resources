/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#define NS_LOG_APPEND_CONTEXT \
  if (m_nodeId != 0xFFFF) { std::clog << "[node " << m_nodeId << "]"; }

#include "cafbcr-qhello.h"
#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/energy-module.h"
#include "ns3/log.h"
#include "ns3/ipv4-header.h"
#include "ns3/mobility-model.h"
#include "ns3/node-list.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/tag.h"
#include "ns3/udp-header.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <sstream>

namespace ns3 {
namespace cafbcr {

NS_LOG_COMPONENT_DEFINE("QLearningHelloManagerCAFBCR");
NS_OBJECT_ENSURE_REGISTERED(QLearningHelloManager);

namespace {
// Classification only: no topology, delivery status or protocol-visible truth.
class CaQHelloControlTag : public Tag {
public:
  static TypeId GetTypeId() {
    static TypeId tid = TypeId("ns3::cafbcr::QHelloControlTag")
      .SetParent<Tag>().AddConstructor<CaQHelloControlTag>();
    return tid;
  }
  TypeId GetInstanceTypeId() const override { return GetTypeId(); }
  uint32_t GetSerializedSize() const override { return 0; }
  void Serialize(TagBuffer) const override {}
  void Deserialize(TagBuffer) override {}
  void Print(std::ostream& os) const override { os << "CA-QBCR local control origin"; }
};

constexpr double REFERENCE_INTERVAL = 0.25;
constexpr double LIFETIME_HORIZON = 60.0;
constexpr double PI = 3.14159265358979323846;

double Unit(double x) { return std::clamp(x, 0.0, 1.0); }
double Norm(Vector v) { return std::hypot(v.x, v.y, v.z); }
double Dot(Vector a, Vector b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
bool Finite(Vector v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }
Vector Advance(Vector p, Vector v, double t) {
  return Vector(p.x + v.x * t, p.y + v.y * t, p.z + v.z * t);
}
Vector Difference(Vector a, Vector b) { return Vector(a.x - b.x, a.y - b.y, a.z - b.z); }
} // namespace

TypeId QLearningHelloManager::GetTypeId()
{
  static TypeId tid = TypeId("ns3::cafbcr::QLearningHelloManager")
    .SetParent<Object>()
    .SetGroupName("cafbcr")
    .AddConstructor<QLearningHelloManager>()
    .AddAttribute("LearningEnabled", "Update the node-local Q table; false is a frozen-table control.",
                  BooleanValue(true), MakeBooleanAccessor(&QLearningHelloManager::m_learningEnabled),
                  MakeBooleanChecker())
    .AddAttribute("LearningRate", "Q-learning step size.", DoubleValue(0.1),
                  MakeDoubleAccessor(&QLearningHelloManager::m_alpha), MakeDoubleChecker<double>(0, 1))
    .AddAttribute("DiscountFactor", "Discount per 0.25 seconds of simulated time.", DoubleValue(0.9),
                  MakeDoubleAccessor(&QLearningHelloManager::m_gamma), MakeDoubleChecker<double>(0, 1))
    .AddAttribute("ExplorationRate", "Initial epsilon, decayed by completed visits to each state.",
                  DoubleValue(0.1), MakeDoubleAccessor(&QLearningHelloManager::m_epsilon),
                  MakeDoubleChecker<double>(0, 1))
    .AddAttribute("MinExplorationRate", "Exploration floor (never above ExplorationRate).",
                  DoubleValue(0.02), MakeDoubleAccessor(&QLearningHelloManager::m_minEpsilon),
                  MakeDoubleChecker<double>(0, 1))
    .AddAttribute("MaxHelloInterval", "Controller ceiling, additionally restricted by TTL and LET.",
                  TimeValue(Seconds(2)), MakeTimeAccessor(&QLearningHelloManager::m_maxInterval),
                  MakeTimeChecker(Seconds(0.25), Seconds(2)));
  return tid;
}

QLearningHelloManager::QLearningHelloManager()
  : m_alpha(0.1), m_gamma(0.9), m_epsilon(0.1), m_minEpsilon(0.02),
    m_learningEnabled(true), m_maxInterval(Seconds(2)), m_intervalLimit(Seconds(3.95)),
    m_currentState{3, 0, 0, 1}, m_lastAction(HELLO_VERY_FAST), m_lastActionTime(Seconds(0)),
    m_pendingAction(false), m_nodeId(0xFFFF), m_neighbors(nullptr), m_maxRange(500),
    m_rand(CreateObject<UniformRandomVariable>()), m_startKinematicsValid(false),
    m_startLifetime(0), m_windowRouteFailures(0), m_windowRouteSamples(0),
    m_windowPredictionError(0), m_windowPredictionSamples(0)
{
  // Zero is an optimistic value for a bounded, non-positive cost reward. It lets
  // untried admissible actions compete after the conservative first decision.
  for (uint8_t s = 0; s < 4; ++s) {
    for (uint8_t d = 0; d < 3; ++d) {
      for (uint8_t l = 0; l < 3; ++l) {
        for (uint8_t e = 0; e < 3; ++e) {
          m_qTable[{s, d, l, e}] = std::vector<double>(NUM_ACTIONS, 0.0);
          m_visits[{s, d, l, e}] = {};
        }
      }
    }
  }
}

void QLearningHelloManager::SetIntervalLimit(Time limit)
{
  NS_ABORT_MSG_IF(limit < Seconds(REFERENCE_INTERVAL),
                  "QBCR neighbor TTL must exceed the minimum Hello interval plus reception margin");
  m_intervalLimit = limit;
}

int64_t QLearningHelloManager::AssignStreams(int64_t stream)
{
  m_rand->SetStream(stream);
  return 1;
}

Time QLearningHelloManager::GetActionInterval(Action action)
{
  static const double intervals[NUM_ACTIONS] = {0.25, 0.5, 0.75, 1.0, 2.0};
  NS_ABORT_MSG_IF(action < HELLO_VERY_FAST || action >= NUM_ACTIONS, "Invalid QBCR action");
  return Seconds(intervals[action]);
}

bool QLearningHelloManager::IsReceivedControlPacket(Ptr<const Packet> packet, const Ipv4Header& header)
{
  if (!packet || header.GetProtocol() != 17 || packet->GetSize() < 8) return false;
  UdpHeader udp;
  return packet->PeekHeader(udp) > 0 && udp.GetDestinationPort() == 889;
}

void QLearningHelloManager::MarkOwnControlPacket(Ptr<Packet> packet)
{
  CaQHelloControlTag tag;
  if (packet && !packet->PeekPacketTag(tag)) packet->AddPacketTag(tag);
}

bool QLearningHelloManager::IsOwnControlPacket(Ptr<const Packet> packet)
{
  CaQHelloControlTag tag;
  return packet && packet->PeekPacketTag(tag);
}

bool QLearningHelloManager::IsBusinessPacket(Ptr<const Packet> packet,
                                             const Ipv4Header& header, bool received)
{
  if (!packet || header.GetProtocol() != 17 || IsOwnControlPacket(packet)) return false;
  // A source-side UDP route query may still contain only application payload.
  // Only received IP payloads are guaranteed to contain the UDP header.
  return !received || !IsReceivedControlPacket(packet, header);
}

std::string QLearningHelloManager::ConfigurationJson()
{
  std::ostringstream out;
  out << "{\"schema\":\"cafbcr-qhello-local-v1\"";
  const auto tid = GetTypeId();
  for (const auto name : {"LearningEnabled", "LearningRate", "DiscountFactor",
                          "ExplorationRate", "MinExplorationRate", "MaxHelloInterval"}) {
    TypeId::AttributeInformation info;
    NS_ABORT_MSG_IF(!tid.LookupAttributeByName(name, &info), "Missing CA-QHello attribute");
    out << ",\"" << name << "\":\"" << info.initialValue->SerializeToString(info.checker) << "\"";
  }
  out << ",\"reward\":\"local-cost-rate-0.50-UDP-route-0.15-loss-0.15-motion-0.10-age-0.10-hello\""
      << ",\"interval_limit\":\"twice-CA-base-Hello-minus-50ms\""
      << ",\"lifetime\":\"cached-3D-sphere-exit-capped-60s\""
      << ",\"reference_seconds\":0.25,\"LET_fraction\":0.25}"
      ;
  return out.str();
}

std::map<uint16_t, Neighbors::NeighborInfo> QLearningHelloManager::LiveNeighbors() const
{
  std::map<uint16_t, Neighbors::NeighborInfo> result;
  if (m_neighbors) {
    for (const auto& [id, info] : m_neighbors->PeekAll()) {
      if (info.UpdateTime + info.NeighborLifeLength > Simulator::Now()) result.emplace(id, info);
    }
  }
  return result;
}

Time QLearningHelloManager::GetOptimalHelloInterval()
{
  if (m_pendingAction) {
    const Time end = m_lastActionTime + GetActionInterval(m_lastAction);
    if (Simulator::Now() < end) {
      return end - Simulator::Now(); // do not overwrite an action whose feedback is still pending
    }
    UpdateQLearning(CollectEnvironment());
  }

  const auto stability = EvaluateLinkStability();
  m_currentState = GetCurrentState(stability);
  m_lastAction = SelectAction(m_currentState, GetAllowedActions(stability));
  m_lastActionTime = Simulator::Now();
  m_pendingAction = true;
  BeginObservation(stability);
  ++m_statistics.decisions;
  ++m_statistics.actions[m_lastAction];
  const Time interval = GetActionInterval(m_lastAction);
  NS_LOG_DEBUG("Q-Learning selected interval: " << interval.GetSeconds() << "s, "
               << "State: stability=" << unsigned(m_currentState.stabilityLevel)
               << " density=" << unsigned(m_currentState.densityLevel)
               << " lifetime=" << unsigned(m_currentState.lifelengthLevel)
               << " energy=" << unsigned(m_currentState.energyLevel));
  return interval;
}

bool QLearningHelloManager::ReadKinematics(Vector& position, Vector& velocity) const
{
  if (m_nodeId == 0xFFFF || m_nodeId >= NodeList::GetNNodes()) return false;
  auto mobility = NodeList::GetNode(m_nodeId)->GetObject<MobilityModel>();
  if (!mobility) return false;
  position = mobility->GetPosition();
  velocity = mobility->GetVelocity();
  return Finite(position) && Finite(velocity);
}

double QLearningHelloManager::ReadEnergyRatio() const
{
  if (m_nodeId != 0xFFFF && m_nodeId < NodeList::GetNNodes()) {
    auto sources = NodeList::GetNode(m_nodeId)->GetObject<EnergySourceContainer>();
    if (sources && sources->GetN() > 0) {
      auto source = DynamicCast<BasicEnergySource>(sources->Get(0));
      if (source && source->GetInitialEnergy() > 0) {
        return Unit(source->GetRemainingEnergy() / source->GetInitialEnergy());
      }
    }
  }
  return std::numeric_limits<double>::quiet_NaN();
}

double QLearningHelloManager::EstimateLinkLifetime(Vector p, Vector v, double range)
{
  if (!Finite(p) || !Finite(v) || !std::isfinite(range) || range <= 0) return 0;
  const double c = Dot(p, p) - range * range;
  if (c >= 0) return 0; // a cached node outside radio range is not a long-lived link
  const double a = Dot(v, v);
  if (a < 1e-12) return LIFETIME_HORIZON;
  const double b = 2 * Dot(p, v);
  const double root = std::sqrt(std::max(0.0, b * b - 4 * a * c));
  // Rationalize the positive root when b >= 0 to avoid cancellation.
  const double t = b >= 0 ? (-2 * c) / (root + b) : (root - b) / (2 * a);
  return std::isfinite(t) ? std::clamp(t, 0.0, LIFETIME_HORIZON) : 0.0;
}

QLearningHelloManager::LinkStability QLearningHelloManager::EvaluateLinkStability()
{
  LinkStability result{50.0, PI, 1.0, 0.0}; // unknown/isolated is conservatively unstable
  Vector position, velocity;
  if (!m_neighbors || !ReadKinematics(position, velocity) ||
      !std::isfinite(m_maxRange) || m_maxRange <= 0) return result;
  const auto neighbors = LiveNeighbors();
  if (neighbors.empty()) return result;

  double relSpeed = 0, divergence = 0, risk = 0, lifetime = LIFETIME_HORIZON;
  for (const auto& item : neighbors) {
    const auto& n = item.second;
    const double age = std::max(0.0, (Simulator::Now() - n.UpdateTime).GetSeconds());
    // Only advertised position/velocity and locally observed age are available.
    const Vector p = Difference(Advance(n.Position, n.Velocity, age), position);
    const Vector v = Difference(n.Velocity, velocity);
    if (!Finite(p) || !Finite(v)) return result;
    relSpeed += Norm(v);
    const double speeds = Norm(velocity) * Norm(n.Velocity);
    divergence += speeds > 1e-9 ? std::acos(std::clamp(Dot(velocity, n.Velocity) / speeds, -1.0, 1.0)) : 0;
    risk += std::min(2.0, Norm(p) / m_maxRange);
    lifetime = std::min(lifetime, EstimateLinkLifetime(p, v, m_maxRange));
  }
  const double count = neighbors.size();
  result = {relSpeed / count, divergence / count, risk / count, lifetime};
  NS_LOG_DEBUG("Link Stability - RelVel: " << result.relativeVelocity
               << " DirDiv: " << result.directionDivergence << " DistRisk: " << result.distanceRisk
               << " Lifetime: " << result.predictedLifetime);
  return result;
}

QLearningHelloManager::State QLearningHelloManager::GetCurrentState(const LinkStability& stability)
{
  return {QuantifyLinkDynamics(stability),
          QuantifyDensity(m_neighbors ? LiveNeighbors().size() : 0),
          QuantifyStability(stability.predictedLifetime), QuantifyEnergy(ReadEnergyRatio())};
}

std::vector<QLearningHelloManager::Action>
QLearningHelloManager::GetAllowedActions(const LinkStability& stability)
{
  // Use at most a quarter of the shortest predicted remaining link lifetime.
  // The minimum Hello action remains available for discovery/urgent repair.
  double limit = std::min(m_maxInterval.GetSeconds(), m_intervalLimit.GetSeconds());
  limit = std::min(limit, std::max(REFERENCE_INTERVAL, 0.25 * stability.predictedLifetime));
  std::vector<Action> allowed;
  for (int i = 0; i < NUM_ACTIONS; ++i) {
    auto action = static_cast<Action>(i);
    if (GetActionInterval(action).GetSeconds() <= limit) allowed.push_back(action);
  }
  return allowed;
}

QLearningHelloManager::Action
QLearningHelloManager::SelectAction(const State& state, const std::vector<Action>& allowed)
{
  NS_ABORT_MSG_IF(allowed.empty(), "QBCR has no admissible Hello action");
  const auto& visits = m_visits.at(state);
  const uint64_t total = std::accumulate(visits.begin(), visits.end(), uint64_t{0});
  const double floor = std::min(m_minEpsilon, m_epsilon);
  const double epsilon = floor + (m_epsilon - floor) * std::pow(0.995, total);
  m_statistics.lastExplorationRate = epsilon;
  if (m_rand->GetValue() < epsilon) {
    ++m_statistics.exploratoryDecisions;
    return allowed[m_rand->GetInteger(0, allowed.size() - 1)];
  }
  // On a genuine tie prefer the shortest admissible action; negative cost
  // updates and independent exploration remove the old all-zero-table lock.
  Action best = allowed.front();
  for (auto action : allowed) {
    if (m_qTable.at(state)[action] > m_qTable.at(state)[best]) best = action;
  }
  if (visits[best] > 0 && m_qTable.at(state)[best] != 0) ++m_statistics.learnedDecisions;
  return best;
}

void QLearningHelloManager::BeginObservation(const LinkStability& stability)
{
  m_startNeighbors.clear();
  if (m_neighbors) {
    for (const auto& item : LiveNeighbors()) m_startNeighbors.insert(item.first);
  }
  m_startKinematicsValid = ReadKinematics(m_startPosition, m_startVelocity);
  m_startLifetime = stability.predictedLifetime;
  m_windowRouteFailures = m_windowRouteSamples = m_windowPredictionSamples = 0;
  m_windowPredictionError = 0;
}

void QLearningHelloManager::ObserveRoutingResult(bool usableRoute)
{
  if (!m_pendingAction) return;
  ++m_windowRouteSamples;
  if (!usableRoute) ++m_windowRouteFailures;
  ++m_statistics.routingObservations;
}

void QLearningHelloManager::ProcessHelloInfo(uint16_t senderId, Vector position, Vector velocity,
                                             Time timestamp)
{
  if (!Finite(position) || !Finite(velocity) || timestamp > Simulator::Now()) return;
  auto previous = m_helloHistory.find(senderId);
  if (previous != m_helloHistory.end()) {
    if (timestamp <= previous->second.timestamp) return;
    const double age = (timestamp - previous->second.timestamp).GetSeconds();
    // Do not compare unrelated observations after a prolonged absence.
    if (m_pendingAction && age <= LIFETIME_HORIZON && m_maxRange > 0) {
      const auto predicted = Advance(previous->second.position, previous->second.velocity, age);
      m_windowPredictionError += Unit(Norm(Difference(position, predicted)) / (0.1 * m_maxRange));
      ++m_windowPredictionSamples;
    }
  }
  m_helloHistory[senderId] = {position, velocity, timestamp};
  ++m_statistics.receivedHellos;
}

QLearningHelloManager::Environment QLearningHelloManager::CollectEnvironment()
{
  Environment env;
  const double seconds = (Simulator::Now() - m_lastActionTime).GetSeconds();
  env.routingSamples = m_windowRouteSamples;
  if (m_windowRouteSamples) env.routingFailureRatio = double(m_windowRouteFailures) / m_windowRouteSamples;
  if (m_neighbors && !m_startNeighbors.empty()) {
    const auto current = LiveNeighbors();
    size_t lost = 0;
    for (auto id : m_startNeighbors) if (!current.count(id)) ++lost;
    env.neighborLossRate = Unit(double(lost) / m_startNeighbors.size() / seconds);
  }
  Vector position, velocity;
  const bool ownValid = m_startKinematicsValid && ReadKinematics(position, velocity) && m_maxRange > 0;
  if (ownValid) {
    env.motionPredictionError = Unit(Norm(Difference(position,
        Advance(m_startPosition, m_startVelocity, seconds))) / (0.1 * m_maxRange));
  }
  if (m_windowPredictionSamples) {
    const double neighborError = m_windowPredictionError / m_windowPredictionSamples;
    env.motionPredictionError = ownValid ? 0.5 * (env.motionPredictionError + neighborError) : neighborError;
  }
  if (!m_startNeighbors.empty()) {
    env.linkAgeRisk = Unit(seconds / std::max(REFERENCE_INTERVAL, m_startLifetime));
  }
  env.normalizedHelloRate = Unit(REFERENCE_INTERVAL / seconds);
  return env;
}

double QLearningHelloManager::CalculateReward(const Environment& env) const
{
  // A bounded utility RATE. Unobserved terms remain unknown and contribute no
  // invented success. Delivery PDR, ground-truth neighbors and delay are absent.
  double cost = 0;
  const auto add = [&cost](double weight, double value) {
    if (std::isfinite(value)) cost += weight * Unit(value);
  };
  if (env.routingSamples) add(0.50, env.routingFailureRatio);
  add(0.15, env.neighborLossRate);
  add(0.15, env.motionPredictionError);
  add(0.10, env.linkAgeRisk);
  add(0.10, env.normalizedHelloRate);
  return -cost;
}

void QLearningHelloManager::UpdateQLearning(const Environment& env)
{
  if (!m_pendingAction || Simulator::Now() < m_lastActionTime + GetActionInterval(m_lastAction)) return;
  m_lastEnvironment = env;
  m_statistics.lastReward = CalculateReward(env);
  ++m_statistics.feedbackWindows;
  if (m_learningEnabled) {
    const auto stability = EvaluateLinkStability();
    UpdateQValue(env, GetCurrentState(stability), GetAllowedActions(stability));
    ++m_statistics.updates;
  }
  // Frozen-table controls retain the same visit-based exploration schedule.
  ++m_visits.at(m_currentState)[m_lastAction];
  m_pendingAction = false; // one action can be settled only once; stop-time partial windows stay pending
  NS_LOG_DEBUG("Q-Learning updated - Reward: " << m_statistics.lastReward
               << " Updates: " << m_statistics.updates << " RouteSamples: " << env.routingSamples
               << " RouteFailureRatio: " << env.routingFailureRatio
               << " NeighborLossRate: " << env.neighborLossRate
               << " MotionError: " << env.motionPredictionError);
}

void QLearningHelloManager::UpdateQValue(const Environment& env, const State& nextState,
                                         const std::vector<Action>& allowed)
{
  double future = -std::numeric_limits<double>::infinity();
  for (auto action : allowed) future = std::max(future, m_qTable.at(nextState)[action]);
  const double periods = (Simulator::Now() - m_lastActionTime).GetSeconds() / REFERENCE_INTERVAL;
  const double discount = std::pow(m_gamma, periods);
  const double accumulation = m_gamma < 1 ? (1 - discount) / (1 - m_gamma) : periods;
  double& q = m_qTable.at(m_currentState)[m_lastAction];
  const double old = q;
  q += m_alpha * (CalculateReward(env) * accumulation + discount * future - q);
  NS_LOG_DEBUG("Updated Q-value for action " << unsigned(m_lastAction) << ": " << old << " -> " << q);
}

double QLearningHelloManager::GetQValue(const State& state, Action action) const
{
  const auto found = m_qTable.find(state);
  return found != m_qTable.end() && action >= 0 && action < NUM_ACTIONS ? found->second[action] : 0;
}

uint8_t QLearningHelloManager::QuantifyLinkDynamics(const LinkStability& s) const
{
  if (s.predictedLifetime <= 0) return 3;
  double score = s.relativeVelocity < 5 ? 0.4 : (s.relativeVelocity < 15 ? 0.24 : (s.relativeVelocity < 30 ? 0.12 : 0));
  score += s.directionDivergence < 0.52 ? 0.3 : (s.directionDivergence < 1.05 ? 0.15 : 0);
  score += s.distanceRisk < 0.5 ? 0.3 : (s.distanceRisk < 0.7 ? 0.15 : 0);
  return score > 0.75 ? 0 : (score > 0.5 ? 1 : (score > 0.25 ? 2 : 3));
}
uint8_t QLearningHelloManager::QuantifyDensity(uint16_t n) const { return n < 3 ? 0 : (n < 8 ? 1 : 2); }
uint8_t QLearningHelloManager::QuantifyStability(double t) const { return t > 10 ? 2 : (t > 5 ? 1 : 0); }
uint8_t QLearningHelloManager::QuantifyEnergy(double r) const {
  return !std::isfinite(r) ? 1 : (r < 0.3 ? 0 : (r < 0.7 ? 1 : 2));
}

} // namespace cafbcr
} // namespace ns3
