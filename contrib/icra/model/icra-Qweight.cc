/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "icra-Qweight.h"

#include "ns3/abort.h"
#include "ns3/double.h"
#include "ns3/energy-source-container.h"
#include "ns3/fanet-node.h"
#include "ns3/integer.h"
#include "ns3/log.h"
#include "ns3/mobility-model.h"
#include "ns3/node-list.h"
#include "ns3/object-factory.h"
#include "ns3/simulator.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <random>
#include <sstream>
#include <tuple>

namespace ns3
{
namespace icra
{

NS_LOG_COMPONENT_DEFINE("IcraQweightOptimizer");
NS_OBJECT_ENSURE_REGISTERED(QLearningWeightOptimizer);
Ptr<QLearningWeightOptimizer> QLearningWeightOptimizer::m_instance;

namespace
{

double
ClampUnit(double value)
{
    return std::isfinite(value) ? std::clamp(value, 0.0, 1.0) : 0.0;
}

double
RewardTransform(double r)
{
    // Eq. 22 really has exp(+3r) in its denominator. Do not replace with tanh.
    if (r == 1.0)
    {
        return 1.0;
    }
    if (r == -1.0)
    {
        return -1.0;
    }
    return (1.0 - std::exp(-3.0 * r)) / (1.0 + std::exp(3.0 * r));
}

std::vector<std::string>
Split(const std::string& value, char delimiter)
{
    std::vector<std::string> result;
    std::istringstream input(value);
    std::string part;
    while (std::getline(input, part, delimiter))
    {
        result.push_back(part);
    }
    return result;
}

bool
ParseNumber(const std::string& text, double& value)
{
    try
    {
        std::size_t consumed = 0;
        value = std::stod(text, &consumed);
        return consumed == text.size() && std::isfinite(value);
    }
    catch (const std::exception&)
    {
        return false;
    }
}

bool
CanonicalAction(const std::array<double, 4>& weights,
                QLearningWeightOptimizer::Action& action)
{
    std::array<int, 4> integers;
    int sum = 0;
    for (std::size_t i = 0; i < weights.size(); ++i)
    {
        if (!std::isfinite(weights[i]) || weights[i] < -1e-9 || weights[i] > 1.0 + 1e-9)
        {
            return false;
        }
        integers[i] = static_cast<int>(std::lround(weights[i] * 20.0));
        if (std::abs(weights[i] - integers[i] / 20.0) > 1e-9)
        {
            return false;
        }
        sum += integers[i];
    }
    if (sum != 20)
    {
        return false;
    }
    action = {integers[0] / 20.0, integers[1] / 20.0,
              integers[2] / 20.0, integers[3] / 20.0};
    return true;
}

} // namespace

bool QLearningWeightOptimizer::State::operator<(const State& other) const
{
    return std::tie(s1, s2, s3, s4) < std::tie(other.s1, other.s2, other.s3, other.s4);
}
bool QLearningWeightOptimizer::State::operator==(const State& other) const
{
    return std::tie(s1, s2, s3, s4) == std::tie(other.s1, other.s2, other.s3, other.s4);
}
bool QLearningWeightOptimizer::Action::operator<(const Action& other) const
{
    return std::tie(w1, w2, w3, w4) < std::tie(other.w1, other.w2, other.w3, other.w4);
}
bool QLearningWeightOptimizer::Action::operator==(const Action& other) const
{
    return std::tie(w1, w2, w3, w4) == std::tie(other.w1, other.w2, other.w3, other.w4);
}

TypeId
QLearningWeightOptimizer::GetTypeId()
{
    static TypeId tid = TypeId("ns3::icra::QLearningWeightOptimizer")
        .SetParent<Object>()
        .SetGroupName("icra")
        .AddConstructor<QLearningWeightOptimizer>()
        .AddAttribute("LearningRate", "Q-learning rate", DoubleValue(0.8),
                      MakeDoubleAccessor(&QLearningWeightOptimizer::m_alpha),
                      MakeDoubleChecker<double>(0.0, 1.0))
        .AddAttribute("StabilityWeight", "Eq. 23 cluster stability weight", DoubleValue(0.5),
                      MakeDoubleAccessor(&QLearningWeightOptimizer::m_lambda),
                      MakeDoubleChecker<double>(0.0, 1.0))
        .AddAttribute("ExplorationRate", "Legacy-only epsilon-greedy exploration", DoubleValue(0.1),
                      MakeDoubleAccessor(&QLearningWeightOptimizer::m_epsilon),
                      MakeDoubleChecker<double>(0.0, 1.0))
        .AddAttribute("EvaluationPeriod", "Strategy evaluation interval", TimeValue(Seconds(2)),
                      MakeTimeAccessor(&QLearningWeightOptimizer::m_evaluationPeriod),
                      MakeTimeChecker());
    return tid;
}

QLearningWeightOptimizer::QLearningWeightOptimizer()
    : m_evaluationPeriod(Seconds(2))
{
    Initialize();
}

QLearningWeightOptimizer::~QLearningWeightOptimizer() = default;

Ptr<QLearningWeightOptimizer>
QLearningWeightOptimizer::GetInstance()
{
    if (!m_instance)
    {
        m_instance = CreateObject<QLearningWeightOptimizer>();
        Simulator::ScheduleDestroy(&QLearningWeightOptimizer::ReleaseInstance);
    }
    return m_instance;
}

void QLearningWeightOptimizer::ReleaseInstance()
{
    if (m_instance)
    {
        m_instance->Reset();
        m_instance = nullptr;
    }
}

void QLearningWeightOptimizer::Initialize()
{
    GenerateActionSpace();
    Reset();
}

void QLearningWeightOptimizer::GenerateActionSpace()
{
    m_actionSpace.clear();
    // Integer simplex: all C(23,3)=1771 valid action grid points.
    // Floating loop bounds previously dropped legal zero-weight endpoints.
    for (int k1 = 0; k1 <= 20; ++k1)
    {
        for (int k2 = 0; k2 <= 20 - k1; ++k2)
        {
            for (int k3 = 0; k3 <= 20 - k1 - k2; ++k3)
            {
                const int k4 = 20 - k1 - k2 - k3;
                m_actionSpace.push_back({k1 / 20.0, k2 / 20.0, k3 / 20.0, k4 / 20.0});
            }
        }
    }
}

void QLearningWeightOptimizer::Reset(bool keepQTable)
{
    if (!keepQTable)
    {
        m_qTable.clear();
        m_rTable.clear();
        m_visitCount.clear();
    }
    m_currentState = {};
    m_currentAction = {};
    m_episodeActive = false;
    m_nodeStatusMap.clear();
    m_feedbackHistory.clear();
    m_roleChangeCount.clear();
    m_initialEnergy.clear();
    m_initialRoleChanges.clear();
    m_episodeParticipants.clear();
    m_episodeCount = 0;
    m_cumulativeReward = 0;
    m_episodeStartTime = Time();
    m_lastReward = 0;
    m_lastFeedbackCount = 0;
}

void QLearningWeightOptimizer::SetLearningRate(double alpha)
{
    NS_ABORT_MSG_IF(!std::isfinite(alpha) || alpha < 0 || alpha > 1, "Invalid ICRA alpha");
    m_alpha = alpha;
}
void QLearningWeightOptimizer::SetStabilityWeight(double lambda)
{
    NS_ABORT_MSG_IF(!std::isfinite(lambda) || lambda < 0 || lambda > 1, "Invalid ICRA lambda");
    m_lambda = lambda;
}
void QLearningWeightOptimizer::SetExplorationRate(double epsilon)
{
    NS_ABORT_MSG_IF(!std::isfinite(epsilon) || epsilon < 0 || epsilon > 1, "Invalid ICRA epsilon");
    m_epsilon = epsilon;
}
void QLearningWeightOptimizer::SetEvaluationPeriod(Time period)
{
    NS_ABORT_MSG_IF(period <= Time(), "ICRA evaluation period must be positive");
    m_evaluationPeriod = period;
}
void QLearningWeightOptimizer::SetRoleChangeThreshold(uint32_t threshold)
{
    m_roleChangeThreshold = threshold;
}
void QLearningWeightOptimizer::SetLegacyMode(bool legacy)
{
    if (m_legacyMode != legacy)
    {
        Reset();
        m_legacyMode = legacy;
    }
}
bool QLearningWeightOptimizer::IsLegacyMode() const { return m_legacyMode; }
void QLearningWeightOptimizer::SetMetricParameters(double range, double normalization)
{
    NS_ABORT_MSG_IF(!std::isfinite(range) || range <= 0 ||
                        !std::isfinite(normalization) || normalization <= 0,
                    "Invalid ICRA metric range or LHTP normalization");
    m_range = range;
    m_lhtpNormalizationSeconds = normalization;
}
int64_t QLearningWeightOptimizer::AssignStreams(int64_t stream)
{
    // Construct with the explicit stream so no automatic stream is consumed
    // first. Merely installing an unused legacy learner must not shift ARP,
    // mobility, or other protocols' automatic random streams.
    m_random = CreateObjectWithAttributes<UniformRandomVariable>("Stream", IntegerValue(stream));
    return 1;
}
bool QLearningWeightOptimizer::IsEpisodeActive() const { return m_episodeActive; }
uint32_t QLearningWeightOptimizer::GetEpisodeCount() const { return m_episodeCount; }
Time QLearningWeightOptimizer::GetEvaluationPeriod() const { return m_evaluationPeriod; }

double
QLearningWeightOptimizer::LinkHoldingTime(Vector ownPosition, Vector ownVelocity,
                                          Vector otherPosition, Vector otherVelocity, double range)
{
    // Eq. 13--18 are planar. Read current own motion in GetUtilityFactors.
    if (!std::isfinite(range) || range <= 0)
    {
        return 0;
    }
    const double a = ownVelocity.x - otherVelocity.x;
    const double b = ownPosition.x - otherPosition.x;
    const double c = ownVelocity.y - otherVelocity.y;
    const double d = ownPosition.y - otherPosition.y;
    const double speedSquared = a * a + c * c;
    const double distanceSquared = b * b + d * d;
    if (distanceSquared > range * range)
    {
        return 0.0;
    }
    if (speedSquared <= 1e-20)
    {
        return std::numeric_limits<double>::infinity();
    }
    const double cross = a * d - b * c;
    const double discriminant = speedSquared * range * range - cross * cross;
    if (discriminant < 0.0)
    {
        return 0.0;
    }
    return std::max(0.0, (-(a * b + c * d) + std::sqrt(discriminant)) / speedSquared);
}

std::array<double, 4>
QLearningWeightOptimizer::GetUtilityFactors(Ptr<Node> node, double range, double normalization)
{
    std::array<double, 4> factors{{0, 0, 0, 0}};
    if (!node || !std::isfinite(range) || range <= 0 ||
        !std::isfinite(normalization) || normalization <= 0)
    {
        return factors;
    }
    auto cluster = node->GetObject<ClusterNode>();
    auto mobility = node->GetObject<MobilityModel>();
    if (!cluster || !mobility)
    {
        return factors;
    }
    double remaining = cluster->GetResidualEnergy();
    double original = cluster->GetOriginEnergy();
    auto sources = node->GetObject<EnergySourceContainer>();
    if (sources && sources->GetN() > 0)
    {
        remaining = sources->Get(0)->GetRemainingEnergy();
        original = sources->Get(0)->GetInitialEnergy();
    }
    factors[0] = original > 0 ? ClampUnit(remaining / original) : 0;
    auto neighbors = cluster->GetNeighborTable();
    if (!neighbors)
    {
        return factors;
    }
    const Vector position = mobility->GetPosition();
    const Vector velocity = mobility->GetVelocity();
    const double speed = std::hypot(velocity.x, velocity.y);
    const double direction = std::atan2(velocity.y, velocity.x);
    std::vector<double> similarities;
    double holdingSum = 0;
    for (const auto& entry : neighbors->GetAll())
    {
        const auto& neighbor = entry.second;
        // Isolate also denotes an explicitly excluded external BS. It is not
        // a participant in the paper's UAV clustering utility distribution.
        if (neighbor.NodeId == cluster->GetNodeId() || neighbor.Role == Isolate)
        {
            continue;
        }
        const double otherSpeed = std::hypot(neighbor.Velocity.x, neighbor.Velocity.y);
        // Resolve the unspecified +/-pi branch without replacing the variance.
        const double angleDifference = std::remainder(
            direction - std::atan2(neighbor.Velocity.y, neighbor.Velocity.x),
            2.0 * std::acos(-1.0));
        similarities.push_back(1.0 / (1.0 + std::hypot(speed - otherSpeed, angleDifference)));
        const double age = std::max(0.0, (Simulator::Now() - neighbor.UpdateTime).GetSeconds());
        const Vector neighborPosition(neighbor.Position.x + age * neighbor.Velocity.x,
                                      neighbor.Position.y + age * neighbor.Velocity.y,
                                      neighbor.Position.z + age * neighbor.Velocity.z);
        const double holding = LinkHoldingTime(position, velocity, neighborPosition,
                                               neighbor.Velocity, range);
        holdingSum += std::isfinite(holding) ? std::min(holding, normalization) : normalization;
    }
    const std::size_t count = similarities.size();
    if (count == 0)
    {
        return factors;
    }
    const uint16_t scale = cluster->GetNetworkScale();
    factors[1] = scale > 1 ? ClampUnit(static_cast<double>(count) / (scale - 1)) : 0;
    double mean = 0;
    for (double similarity : similarities)
    {
        mean += similarity;
    }
    mean /= count;
    for (double similarity : similarities)
    {
        factors[2] += (similarity - mean) * (similarity - mean);
    }
    factors[2] /= count; // literal Eq. 12 variance, naturally <= 1/4
    factors[3] = ClampUnit(holdingSum / (count * normalization));
    return factors;
}

void QLearningWeightOptimizer::StartEpisode()
{
    if (m_episodeActive)
    {
        NS_LOG_WARN("Ignoring duplicate ICRA StartEpisode");
        return;
    }
    m_currentState = CalculateScenarioState();
    m_currentAction = SelectAction(m_currentState);
    m_episodeActive = true;
    m_episodeStartTime = Simulator::Now();
    ++m_episodeCount;
    m_roleChangeCount.clear();
    m_initialEnergy.clear();
    m_initialRoleChanges.clear();
    m_episodeParticipants.clear();
    if (m_legacyMode)
    {
        for (uint32_t i = 0; i < NodeList::GetNNodes(); ++i)
        {
            auto cluster = NodeList::GetNode(i)->GetObject<ClusterNode>();
            if (cluster)
            {
                m_initialEnergy[i] = cluster->GetResidualEnergy();
                m_episodeParticipants.insert(i);
            }
        }
        return;
    }
    // Baselines and participants are reports actually received at this boundary.
    // A node first seen mid-window participates in the next window.
    for (const auto& [id, status] : m_nodeStatusMap)
    {
        const auto feedback = m_feedbackHistory.find(id);
        if (!status.active || feedback == m_feedbackHistory.end() || feedback->second.empty())
        {
            continue;
        }
        const auto& latest = feedback->second.back();
        m_episodeParticipants.insert(id);
        m_initialEnergy[id] = latest.energyChange;
        m_initialRoleChanges[id] = latest.roleChanges;
    }
}

void QLearningWeightOptimizer::EndEpisode()
{
    if (!m_episodeActive)
    {
        return;
    }
    m_episodeActive = false;
    const double reward = CalculateAverageReward();
    if (m_lastFeedbackCount == 0)
    {
        return; // Lost/no feedback is not an invented zero or negative outcome.
    }
    m_lastReward = reward;
    m_cumulativeReward += reward;
    ++m_visitCount[m_currentState][m_currentAction];
    if (m_legacyMode)
    {
        const auto count = m_visitCount[m_currentState][m_currentAction];
        const double old = m_rTable[m_currentState][m_currentAction];
        m_rTable[m_currentState][m_currentAction] = (old * (count - 1) + reward) / count;
    }
    else
    {
        // Text of Algorithm 2: average feedback received in this period T.
        // Historical smoothing belongs to Q, not a second cumulative R mean.
        m_rTable[m_currentState][m_currentAction] = reward;
    }
    UpdateQValue(m_currentState, m_currentAction);
}

QLearningWeightOptimizer::State QLearningWeightOptimizer::CalculateScenarioState()
{
    if (m_legacyMode)
    {
        return CalculateLegacyScenarioState();
    }
    std::array<std::vector<double>, 4> values;
    for (const auto& [id, status] : m_nodeStatusMap)
    {
        (void)id;
        if (!status.active)
        {
            continue;
        }
        for (std::size_t i = 0; i < values.size(); ++i)
        {
            values[i].push_back(status.factors[i]);
        }
    }
    return {DiscretizeEntropyValue(CalculateEntropy(values[0])),
            DiscretizeEntropyValue(CalculateEntropy(values[1])),
            DiscretizeEntropyValue(CalculateEntropy(values[2])),
            DiscretizeEntropyValue(CalculateEntropy(values[3]))};
}

QLearningWeightOptimizer::State QLearningWeightOptimizer::CalculateLegacyScenarioState()
{
    std::array<std::vector<double>, 4> values;
    for (uint32_t i = 0; i < NodeList::GetNNodes(); ++i)
    {
        auto cluster = NodeList::GetNode(i)->GetObject<ClusterNode>();
        if (!cluster)
        {
            continue;
        }
        values[0].push_back(cluster->GetOriginEnergy() > 0
                                ? cluster->GetResidualEnergy() / cluster->GetOriginEnergy() : 0);
        const auto velocity = cluster->GetVelocity();
        values[1].push_back(std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y +
                                      velocity.z * velocity.z) / 60.0);
        values[2].push_back(NodeList::GetNNodes() > 1
                                ? static_cast<double>(cluster->GetNeighborDegree()) /
                                      (NodeList::GetNNodes() - 1) : 0);
        const auto neighbors = cluster->GetNeighborTable();
        double sum = 0;
        std::size_t count = 0;
        if (neighbors)
        {
            for (const auto& entry : neighbors->GetAll())
            {
                sum += CalculateDistance(cluster->GetPosition(), entry.second.Position);
                ++count;
            }
        }
        values[3].push_back(count ? 1.0 / (1.0 + sum / count / 100.0) : 0.5);
    }
    return {DiscretizeEntropyValue(CalculateEntropy(values[0])),
            DiscretizeEntropyValue(CalculateEntropy(values[1])),
            DiscretizeEntropyValue(CalculateEntropy(values[2])),
            DiscretizeEntropyValue(CalculateEntropy(values[3]))};
}

double QLearningWeightOptimizer::CalculateEntropy(const std::vector<double>& values)
{
    // Explicit completion of Eq. 20/21's ambiguous floor(H mod .1):
    // eleven floor bins of [0,1] factors, Shannon entropy / log2(11), then
    // floor(10H). This is not claimed to be a literal modulus operation.
    std::array<std::size_t, 11> histogram{};
    std::size_t total = 0;
    for (double value : values)
    {
        if (!std::isfinite(value))
        {
            continue;
        }
        const auto bin = static_cast<std::size_t>(std::floor(ClampUnit(value) * 10.0));
        ++histogram[bin];
        ++total;
    }
    if (total == 0)
    {
        return 0;
    }
    double entropy = 0;
    for (const auto count : histogram)
    {
        if (count > 0)
        {
            const double probability = static_cast<double>(count) / total;
            entropy -= probability * std::log2(probability);
        }
    }
    return ClampUnit(entropy / std::log2(11.0));
}
int QLearningWeightOptimizer::DiscretizeEntropyValue(double entropy)
{
    return static_cast<int>(std::floor(ClampUnit(entropy) * 10.0 + 1e-12));
}

void QLearningWeightOptimizer::CollectNodeStatus(uint16_t id, const NodeStatus& status)
{
    if (status.timestamp > Simulator::Now() ||
        std::any_of(status.factors.begin(), status.factors.end(),
                    [](double value) { return !std::isfinite(value); }))
    {
        return;
    }
    const auto existing = m_nodeStatusMap.find(id);
    if (existing != m_nodeStatusMap.end())
    {
        if (!m_legacyMode && status.timestamp < existing->second.timestamp)
        {
            return;
        }
        if (m_legacyMode && existing->second.role != status.role)
        {
            ++m_roleChangeCount[id];
        }
    }
    m_nodeStatusMap[id] = status;
}
void QLearningWeightOptimizer::CollectFeedback(uint16_t id, const Feedback& feedback)
{
    if (!std::isfinite(feedback.energyChange) || feedback.energyChange < 0 ||
        feedback.energyChange > 1.0 + 1e-9 || feedback.timestamp > Simulator::Now())
    {
        return;
    }
    auto& history = m_feedbackHistory[id];
    if (!history.empty() && feedback.timestamp < history.back().timestamp)
    {
        return;
    }
    // Only latest received cumulative counters are necessary.
    history.assign(1, feedback);
}

double QLearningWeightOptimizer::ComputeNodeReward(uint32_t changes, double depletion,
                                                   double lambda, uint32_t threshold)
{
    const double stability = changes < threshold ? 1.0 : -1.0; // Eq. 24, strict <
    const double efficiency = 1.0 - 2.0 * ClampUnit(depletion); // Eq. 25/26
    const double weight = ClampUnit(lambda);
    return RewardTransform(weight * stability + (1.0 - weight) * efficiency);
}

double QLearningWeightOptimizer::CalculateClusterStability(uint16_t id)
{
    if (!m_legacyMode)
    {
        const auto history = m_feedbackHistory.find(id);
        const auto initial = m_initialRoleChanges.find(id);
        if (history == m_feedbackHistory.end() || history->second.empty() ||
            initial == m_initialRoleChanges.end())
        {
            return 0;
        }
        const auto latest = history->second.back().roleChanges;
        const auto count = latest >= initial->second ? latest - initial->second : 0;
        return count < m_roleChangeThreshold ? 1.0 : -1.0;
    }
    uint32_t unstable = 0;
    for (const auto& pair : m_roleChangeCount)
    {
        unstable += pair.second > m_roleChangeThreshold;
    }
    return unstable < NodeList::GetNNodes() / 2 ? 1.0 : -1.0;
}

double QLearningWeightOptimizer::CalculateEnergyEfficiency(uint16_t id)
{
    if (!m_legacyMode)
    {
        const auto history = m_feedbackHistory.find(id);
        const auto initial = m_initialEnergy.find(id);
        if (history == m_feedbackHistory.end() || history->second.empty() ||
            initial == m_initialEnergy.end())
        {
            return 0;
        }
        return 1.0 - 2.0 * ClampUnit(history->second.back().energyChange - initial->second);
    }
    double total = 0;
    std::size_t count = 0;
    for (uint32_t i = 0; i < NodeList::GetNNodes(); ++i)
    {
        const auto cluster = NodeList::GetNode(i)->GetObject<ClusterNode>();
        const auto initial = m_initialEnergy.find(i);
        if (cluster && initial != m_initialEnergy.end() && cluster->GetOriginEnergy() > 0)
        {
            total += (initial->second - cluster->GetResidualEnergy()) / cluster->GetOriginEnergy();
            ++count;
        }
    }
    return count ? std::clamp(1.0 - 2.0 * total / count, -1.0, 1.0) : 0.0;
}

double QLearningWeightOptimizer::CalculateNodeReward(uint16_t id)
{
    const double r = m_lambda * CalculateClusterStability(id) +
                     (1.0 - m_lambda) * CalculateEnergyEfficiency(id);
    return RewardTransform(r);
}
double QLearningWeightOptimizer::CalculateAverageReward()
{
    double sum = 0;
    m_lastFeedbackCount = 0;
    for (const auto id : m_episodeParticipants)
    {
        if (!m_legacyMode)
        {
            const auto feedback = m_feedbackHistory.find(id);
            if (feedback == m_feedbackHistory.end() || feedback->second.empty() ||
                feedback->second.back().timestamp <= m_episodeStartTime)
            {
                continue;
            }
        }
        sum += CalculateNodeReward(id);
        ++m_lastFeedbackCount;
    }
    return m_lastFeedbackCount ? sum / m_lastFeedbackCount : 0;
}

void QLearningWeightOptimizer::UpdateQValue(const State& state, const Action& action)
{
    // Algorithm 2 explicitly sets gamma=0: no next-state bootstrap.
    const double reward = m_rTable[state][action];
    SetQValue(state, action, m_alpha * reward + (1.0 - m_alpha) * GetQValue(state, action));
}
QLearningWeightOptimizer::Action QLearningWeightOptimizer::SelectAction(const State& state)
{
    if (m_legacyMode && DrawUnitRandom() < m_epsilon)
    {
        return GetRandomAction();
    }
    return GetBestAction(state);
}
QLearningWeightOptimizer::Action QLearningWeightOptimizer::GetBestAction(const State& state)
{
    double maximum = -std::numeric_limits<double>::infinity();
    std::vector<std::size_t> best;
    for (std::size_t i = 0; i < m_actionSpace.size(); ++i)
    {
        const double q = GetQValue(state, m_actionSpace[i]);
        if (q > maximum)
        {
            maximum = q;
            best.assign(1, i);
        }
        else if (!m_legacyMode && q == maximum)
        {
            best.push_back(i);
        }
    }
    const auto chosen = best.size() > 1 ? DrawRandomIndex(best.size() - 1) : 0;
    return m_actionSpace[best[chosen]];
}
QLearningWeightOptimizer::Action QLearningWeightOptimizer::GetRandomAction()
{
    return m_actionSpace[DrawRandomIndex(m_actionSpace.size() - 1)];
}
double QLearningWeightOptimizer::DrawUnitRandom()
{
    if (!m_random && m_legacyMode)
    {
        static std::mt19937 generator(std::random_device{}());
        return std::uniform_real_distribution<double>(0.0, 1.0)(generator);
    }
    if (!m_random)
    {
        m_random = CreateObject<UniformRandomVariable>();
    }
    return m_random->GetValue();
}
uint32_t QLearningWeightOptimizer::DrawRandomIndex(uint32_t maximum)
{
    if (!m_random && m_legacyMode)
    {
        static std::mt19937 generator(std::random_device{}());
        return std::uniform_int_distribution<uint32_t>(0, maximum)(generator);
    }
    if (!m_random)
    {
        m_random = CreateObject<UniformRandomVariable>();
    }
    return m_random->GetInteger(0, maximum);
}
double QLearningWeightOptimizer::GetQValue(const State& state, const Action& action) const
{
    const auto row = m_qTable.find(state);
    if (row != m_qTable.end())
    {
        const auto value = row->second.find(action);
        if (value != row->second.end())
        {
            return value->second;
        }
    }
    return action == Action{} ? 1.0 : 0.0;
}
void QLearningWeightOptimizer::SetQValue(const State& state, const Action& action, double value)
{
    m_qTable[state][action] = value;
}
QLearningWeightOptimizer::Weights QLearningWeightOptimizer::ActionToWeights(const Action& action)
{
    Weights weights;
    if (m_legacyMode)
    {
        weights.velocityWeight = action.w1;
        weights.energyWeight = action.w2;
        weights.degreeWeight = action.w3;
        weights.distanceWeight = action.w4;
    }
    else
    {
        weights.energyWeight = action.w1;
        weights.degreeWeight = action.w2;
        weights.velocityWeight = action.w3;
        weights.distanceWeight = action.w4;
    }
    return weights;
}
QLearningWeightOptimizer::Weights QLearningWeightOptimizer::GetOptimalWeights()
{
    return ActionToWeights(m_episodeActive ? m_currentAction : GetBestAction(CalculateScenarioState()));
}
QLearningWeightOptimizer::Weights QLearningWeightOptimizer::GetCurrentWeights()
{
    return ActionToWeights(m_currentAction);
}

void QLearningWeightOptimizer::SaveQTable(const std::string& filename)
{
    std::ofstream output(filename);
    if (!output)
    {
        NS_LOG_ERROR("Cannot save ICRA Q table: " << filename);
        return;
    }
    output << "# ICRA-QWEIGHT 2 " << (m_legacyMode ? "legacy" : "paper") << '\n'
           << "s1,s2,s3,s4,w1,w2,w3,w4,Q,R,Visits\n"
           << std::setprecision(std::numeric_limits<double>::max_digits10);
    for (const auto& [state, row] : m_qTable)
    {
        for (const auto& [action, q] : row)
        {
            double reward = 0;
            uint32_t visits = 0;
            const auto rRow = m_rTable.find(state);
            if (rRow != m_rTable.end())
            {
                const auto found = rRow->second.find(action);
                if (found != rRow->second.end()) reward = found->second;
            }
            const auto vRow = m_visitCount.find(state);
            if (vRow != m_visitCount.end())
            {
                const auto found = vRow->second.find(action);
                if (found != vRow->second.end()) visits = found->second;
            }
            // Explicit zero entries can override the equal-action prior of one.
            output << state.s1 << ',' << state.s2 << ',' << state.s3 << ',' << state.s4 << ','
                   << action.w1 << ',' << action.w2 << ',' << action.w3 << ',' << action.w4 << ','
                   << q << ',' << reward << ',' << visits << '\n';
        }
    }
}

void QLearningWeightOptimizer::LoadQTable(const std::string& filename)
{
    std::ifstream input(filename);
    if (!input)
    {
        NS_LOG_WARN("Cannot load ICRA Q table: " << filename);
        return;
    }
    decltype(m_qTable) qTable;
    decltype(m_rTable) rTable;
    decltype(m_visitCount) visitCount;
    bool recognized = false;
    bool versioned = false;
    bool malformed = false;
    std::size_t loaded = 0;
    std::string line;
    const std::string prefix = "# ICRA-QWEIGHT 2 ";
    while (std::getline(input, line))
    {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty() || line == "s1,s2,s3,s4,w1,w2,w3,w4,Q,R,Visits" ||
            line.rfind("StateIndex,ActionIndex,", 0) == 0)
        {
            continue;
        }
        if (line.rfind(prefix, 0) == 0)
        {
            const auto preset = line.substr(prefix.size());
            const bool legacy = preset == "legacy";
            if ((preset != "paper" && !legacy) || legacy != m_legacyMode)
            {
                NS_LOG_WARN("ICRA model preset differs from the configured preset");
                return;
            }
            recognized = versioned = true;
            continue;
        }
        const auto fields = Split(line, ',');
        std::vector<std::string> flat;
        if (fields.size() == 7)
        {
            // Historic files describe different features; allow only legacy.
            if (!m_legacyMode)
            {
                malformed = true;
                continue;
            }
            const auto stateParts = Split(fields[2], ':');
            const auto actionParts = Split(fields[3], ':');
            if (stateParts.size() != 4 || actionParts.size() != 4)
            {
                malformed = true;
                continue;
            }
            flat.insert(flat.end(), stateParts.begin(), stateParts.end());
            flat.insert(flat.end(), actionParts.begin(), actionParts.end());
            flat.insert(flat.end(), fields.begin() + 4, fields.end());
        }
        else if (fields.size() == 11 && (versioned || m_legacyMode))
        {
            flat = fields;
        }
        else
        {
            malformed = true;
            continue;
        }
        std::array<double, 11> numbers;
        bool valid = true;
        for (std::size_t i = 0; i < numbers.size(); ++i)
        {
            valid = ParseNumber(flat[i], numbers[i]) && valid;
        }
        if (!valid)
        {
            malformed = true;
            continue;
        }
        State state;
        std::array<int*, 4> states{{&state.s1, &state.s2, &state.s3, &state.s4}};
        for (std::size_t i = 0; i < states.size(); ++i)
        {
            if (numbers[i] < 0 || numbers[i] > 10 || numbers[i] != std::floor(numbers[i]))
            {
                valid = false;
                break;
            }
            *states[i] = static_cast<int>(numbers[i]);
        }
        Action action;
        valid = CanonicalAction({{numbers[4], numbers[5], numbers[6], numbers[7]}}, action) && valid;
        valid = valid && numbers[10] >= 0 &&
                numbers[10] <= std::numeric_limits<uint32_t>::max() &&
                numbers[10] == std::floor(numbers[10]);
        if (!valid)
        {
            malformed = true;
            continue;
        }
        qTable[state][action] = numbers[8];
        rTable[state][action] = numbers[9];
        visitCount[state][action] = static_cast<uint32_t>(numbers[10]);
        recognized = true;
        ++loaded;
    }
    if (!recognized || malformed)
    {
        NS_LOG_WARN("Unrecognized ICRA Q model; keeping the existing table");
        return;
    }
    Reset();
    m_qTable = std::move(qTable);
    m_rTable = std::move(rTable);
    m_visitCount = std::move(visitCount);
    NS_LOG_INFO("Loaded " << loaded << " sparse ICRA Q entries");
}

void QLearningWeightOptimizer::PrintQTable() const
{
    NS_LOG_INFO("ICRA Q table: " << m_qTable.size() << " visited states, "
                << GetStoredQEntryCount() << " stored pairs, "
                << m_actionSpace.size() << " possible actions");
}
void QLearningWeightOptimizer::TestQLearning() { PrintQTable(); }
const std::vector<QLearningWeightOptimizer::Action>& QLearningWeightOptimizer::GetActionSpace() const
{
    return m_actionSpace;
}
QLearningWeightOptimizer::State QLearningWeightOptimizer::GetCurrentState() const
{
    return m_currentState;
}
QLearningWeightOptimizer::Action QLearningWeightOptimizer::GetCurrentAction() const
{
    return m_currentAction;
}
double QLearningWeightOptimizer::GetLastReward() const { return m_lastReward; }
uint32_t QLearningWeightOptimizer::GetLastFeedbackCount() const { return m_lastFeedbackCount; }
std::size_t QLearningWeightOptimizer::GetStoredQEntryCount() const
{
    std::size_t entries = 0;
    for (const auto& row : m_qTable) entries += row.second.size();
    return entries;
}

} // namespace icra
} // namespace ns3
