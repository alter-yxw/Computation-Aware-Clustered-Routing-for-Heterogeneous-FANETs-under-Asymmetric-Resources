/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef QRCF_LEARNING_H
#define QRCF_LEARNING_H
#include "qrcf-model.h"
#include <functional>
#include <map>
#include <vector>

namespace ns3::qrcf
{
struct Candidate
{
    uint32_t id;
    Vector position;
    Motion motion;
    double energy;
    Scalar connectionTime; // caller resolves CT coefficients/zero relative speed
};
struct CandidateScore
{
    uint32_t id;
    std::array<Scalar,4> raw;
    std::array<Scalar,4> normalized;
    Scalar ordinaryReward;
    Scalar learningRate;
};
class Learning
{
  public:
    // Called only when literal min-max normalization is undefined. A missing
    // callback leaves that component undefined, without substituting 0/1/eps.
    using NormalizationPolicy = std::function<Scalar(std::size_t, double, double, double)>;
    static std::vector<CandidateScore> Evaluate(Vector self, const Motion& motion,
                                               Vector destination,
                                               const std::vector<Candidate>& filtered,
                                               const Components& weights,
                                               NormalizationPolicy degenerate = {});
};

// One table for an explicitly selected routing context. The network adapter
// decides whether contexts represent destinations; the paper does not say.
class QTable
{
  public:
    double Value(uint32_t action) const; // unvisited actions start at zero
    Scalar Maximum(const std::vector<uint32_t>& allowed) const;
    std::vector<uint32_t> BestActions(const std::vector<uint32_t>& allowed) const;
    double Update(uint32_t action, double reward, double alpha,
                  double gamma, double maximumNextQ);
    const std::map<uint32_t,double>& Values() const { return m_values; }
    uint64_t Updates() const { return m_updates; }
  private:
    std::map<uint32_t,double> m_values;
    uint64_t m_updates = 0;
};
} // namespace ns3::qrcf
#endif
