/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "qrcf-learning.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace ns3::qrcf
{
std::vector<CandidateScore> Learning::Evaluate(Vector self, const Motion& motion,
                                               Vector destination,
                                               const std::vector<Candidate>& filtered,
                                               const Components& weights,
                                               NormalizationPolicy degenerate)
{
    // Validate the caller-supplied weights even for an empty state set.
    Model::Reward({0,0,0,0},weights);
    std::vector<CandidateScore> result;
    for (const auto& c : filtered)
    {
        if (!std::isfinite(c.energy) || c.energy < 0)
        { throw std::invalid_argument("Invalid QRCF candidate energy"); }
        result.push_back({c.id,{Model::RelativeSpeed(motion,c.motion), c.connectionTime,
                               c.energy, Model::MovementAngle(self,c.position,destination)}, {}, {}, {}});
    }
    for (std::size_t component = 0; component < 4; ++component)
    {
        double minimum = std::numeric_limits<double>::infinity();
        double maximum = -std::numeric_limits<double>::infinity();
        bool complete = true;
        for (const auto& c : result)
        {
            if (!c.raw[component]) { complete = false; break; }
            minimum = std::min(minimum,*c.raw[component]);
            maximum = std::max(maximum,*c.raw[component]);
        }
        // Never silently drop an undefined candidate from the min/max domain.
        if (!complete) { continue; }
        for (auto& c : result)
        {
            const double raw = *c.raw[component];
            c.normalized[component] = Model::Normalize(raw,minimum,maximum);
            if (!c.normalized[component] && degenerate)
            { c.normalized[component] = degenerate(component,raw,minimum,maximum); }
        }
    }
    for (auto& c : result)
    {
        if (c.normalized[1]) { c.learningRate = Model::LearningRate(*c.normalized[1]); }
        if (std::all_of(c.normalized.begin(),c.normalized.end(),[](const Scalar& x) { return x.has_value(); }))
        {
            Components normalized{};
            for (std::size_t i = 0; i < 4; ++i) { normalized[i] = *c.normalized[i]; }
            c.ordinaryReward = Model::Reward(normalized,weights);
        }
    }
    return result;
}
double QTable::Value(uint32_t action) const
{
    const auto i = m_values.find(action);
    return i == m_values.end() ? 0 : i->second;
}
Scalar QTable::Maximum(const std::vector<uint32_t>& allowed) const
{
    const auto best = BestActions(allowed);
    return best.empty() ? Scalar{} : Scalar{Value(best.front())};
}
std::vector<uint32_t> QTable::BestActions(const std::vector<uint32_t>& allowed) const
{
    std::vector<uint32_t> best;
    double maximum = -std::numeric_limits<double>::infinity();
    for (uint32_t action : allowed)
    {
        const double value = Value(action);
        if (value > maximum) { best.clear(); maximum = value; }
        if (value == maximum && std::find(best.begin(),best.end(),action) == best.end())
        { best.push_back(action); }
    }
    return best; // caller explicitly chooses tie policy
}
double QTable::Update(uint32_t action, double reward, double alpha, double gamma, double next)
{
    const double value = Model::UpdateQ(Value(action),reward,alpha,gamma,next);
    m_values[action] = value;
    ++m_updates;
    return value;
}
} // namespace ns3::qrcf
