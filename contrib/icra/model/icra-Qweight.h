/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef ICRA_QWEIGHT_H
#define ICRA_QWEIGHT_H

#include "icra-packet.h"
#include "ns3/nstime.h"
#include "ns3/object.h"
#include "ns3/random-variable-stream.h"

#include <array>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace ns3
{
namespace icra
{

class IcraFidelityTestCase;

/** GS-side strategy learner. Paper mode consumes received reports only. */
class QLearningWeightOptimizer : public Object
{
  public:
    static TypeId GetTypeId();
    QLearningWeightOptimizer();
    ~QLearningWeightOptimizer() override;
    static Ptr<QLearningWeightOptimizer> GetInstance();

    struct State
    {
        int s1{0}; // entropy of residual-energy ratio
        int s2{0}; // entropy of degree centrality
        int s3{0}; // entropy of velocity-similarity variance
        int s4{0}; // entropy of normalized mean link holding time
        bool operator<(const State& other) const;
        bool operator==(const State& other) const;
    };

    struct Action
    {
        double w1{0.25}; // energy, Eq. 7
        double w2{0.25}; // degree, Eq. 8
        double w3{0.25}; // velocity variance, Eq. 12
        double w4{0.25}; // link holding time, Eq. 14
        bool operator<(const Action& other) const;
        bool operator==(const Action& other) const;
    };

    // Historical field names retained for source compatibility. In paper mode,
    // velocityWeight weights the variance, and distanceWeight weights LHTP.
    struct Weights
    {
        double velocityWeight{0.25};
        double energyWeight{0.25};
        double degreeWeight{0.25};
        double distanceWeight{0.25};
    };

    struct NodeStatus
    {
        uint16_t nodeId{0};
        double energy{0};
        double speed{0};
        uint16_t degree{0};
        Vector position;
        NodeRole role{Idle};
        Time lastRoleChange;
        std::array<double, 4> factors{{0, 0, 0, 0}};
        double originEnergy{0};
        bool active{true};
        Time timestamp;
    };

    struct Feedback
    {
        uint16_t nodeId{0};
        // Cumulative depletion / original energy and cumulative role changes.
        // The learner subtracts the values received at episode start.
        double energyChange{0};
        uint32_t roleChanges{0};
        Time timestamp;
    };

    void StartEpisode();
    void EndEpisode();
    void CollectNodeStatus(uint16_t nodeId, const NodeStatus& status);
    void CollectFeedback(uint16_t nodeId, const Feedback& feedback);
    Weights GetOptimalWeights();
    Weights GetCurrentWeights();
    void SaveQTable(const std::string& filename);
    void LoadQTable(const std::string& filename);
    void PrintQTable() const;
    void TestQLearning(); // compatibility: never fabricates feedback

    void SetLearningRate(double alpha);
    void SetStabilityWeight(double lambda);
    void SetExplorationRate(double epsilon);
    void SetEvaluationPeriod(Time period);
    void SetRoleChangeThreshold(uint32_t threshold);
    void SetLegacyMode(bool legacy);
    bool IsLegacyMode() const;
    void SetMetricParameters(double range, double lhtpNormalizationSeconds = 50.0);
    int64_t AssignStreams(int64_t stream);
    void Reset(bool keepQTable = false);
    bool IsEpisodeActive() const;
    uint32_t GetEpisodeCount() const;
    Time GetEvaluationPeriod() const;

    /** Local Eq. 7/8/12/14 factors in paper order. Own motion and energy come
     * from live models; neighbors use received cache records. Each LHTP is
     * truncated at the explicit horizon before taking its normalized mean.
     */
    static std::array<double, 4> GetUtilityFactors(Ptr<Node> node,
                                                double range,
                                                double lhtpNormalizationSeconds = 50.0);
    static double LinkHoldingTime(Vector ownPosition,
                                  Vector ownVelocity,
                                  Vector otherPosition,
                                  Vector otherVelocity,
                                  double range);
    static double ComputeNodeReward(uint32_t roleChanges,
                                    double energyChange,
                                    double lambda,
                                    uint32_t threshold);

    const std::vector<Action>& GetActionSpace() const;
    State GetCurrentState() const;
    Action GetCurrentAction() const;
    double GetQValue(const State& state, const Action& action) const;
    double GetLastReward() const;
    uint32_t GetLastFeedbackCount() const;
    std::size_t GetStoredQEntryCount() const;

  private:
    friend class IcraFidelityTestCase;
    static Ptr<QLearningWeightOptimizer> m_instance;
    static void ReleaseInstance();

    // Missing pairs have the implicit prior: equal weights = 1, others = 0.
    std::map<State, std::map<Action, double>> m_qTable;
    std::map<State, std::map<Action, double>> m_rTable;
    std::map<State, std::map<Action, uint32_t>> m_visitCount;
    std::vector<Action> m_actionSpace;
    State m_currentState;
    Action m_currentAction;
    bool m_episodeActive{false};
    bool m_legacyMode{false};
    double m_alpha{0.8};
    double m_lambda{0.5};
    double m_epsilon{0.1}; // legacy only; paper Eq. 27 randomizes maximum ties
    double m_gamma{0.0};
    Time m_evaluationPeriod;
    uint32_t m_roleChangeThreshold{2};
    double m_range{500};
    double m_lhtpNormalizationSeconds{50};
    Ptr<UniformRandomVariable> m_random;

    std::map<uint16_t, NodeStatus> m_nodeStatusMap;
    std::map<uint16_t, std::vector<Feedback>> m_feedbackHistory;
    std::map<uint16_t, uint32_t> m_roleChangeCount; // legacy sampled roles
    std::map<uint16_t, double> m_initialEnergy;
    std::map<uint16_t, uint32_t> m_initialRoleChanges;
    std::set<uint16_t> m_episodeParticipants;
    uint32_t m_episodeCount{0};
    double m_cumulativeReward{0};
    Time m_episodeStartTime;
    double m_lastReward{0};
    uint32_t m_lastFeedbackCount{0};

    void Initialize();
    void GenerateActionSpace();
    State CalculateScenarioState();
    State CalculateLegacyScenarioState();
    double CalculateEntropy(const std::vector<double>& values);
    int DiscretizeEntropyValue(double entropy);
    double CalculateAverageReward();
    double CalculateNodeReward(uint16_t nodeId);
    double CalculateClusterStability(uint16_t nodeId);
    double CalculateEnergyEfficiency(uint16_t nodeId);
    void UpdateQValue(const State& state, const Action& action);
    Action SelectAction(const State& state);
    Action GetBestAction(const State& state);
    Action GetRandomAction();
    double DrawUnitRandom();
    uint32_t DrawRandomIndex(uint32_t maximum);
    void SetQValue(const State& state, const Action& action, double value);
    Weights ActionToWeights(const Action& action);
};

} // namespace icra
} // namespace ns3

#endif // ICRA_QWEIGHT_H
