/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef CFR_POLICY_H
#define CFR_POLICY_H

#include "ns3/vector.h"
#include <cstdint>
#include <string>
#include <vector>

namespace ns3::cfr
{
// Fixed identity slots preserve the paper's 6N+7 / 7N+7 dimensions.
struct Observation
{
    uint16_t slot = 0;
    Vector position;
    double ghz = 0;
    uint8_t hops = 0; // 0=self, 1=direct, 2=advertised by a direct neighbor
};

struct TaskState
{
    double originalCycles = 0;
    double remainingCycles = 0;
    uint32_t originalBytes = 0;
};

struct Action
{
    int next = -1;
    double fraction = 0;
    double q = 0;
    // Retain the policy request when routing maps a physical no-op to zero.
    double requestedFraction = 0;
};

class Policy
{
  public:
    void Load(const std::string& path);
    uint32_t Nodes() const { return m_nodes; }
    bool IsPaperReference() const { return m_paperReference; }
    bool BitGranularity() const { return m_bitGranularity; }
    bool OriginalPartition() const { return m_originalPartition; }
    bool IsEndpointDiagnostic() const { return m_endpointCoding; }
    double ResultRatio() const { return m_resultRatio; }
    double CompletionTolerance() const { return m_completionTolerance; }
    std::vector<double> Encode(const std::vector<Observation>& view, Vector here,
                               Vector destination, double ghz, const TaskState& task) const;
    Action Select(const std::vector<double>& state, const std::vector<bool>& valid,
                  int directDestination = -1) const;
    std::vector<double> Parameters(const std::vector<double>& state) const;
    std::vector<double> Values(const std::vector<double>& state,
                               const std::vector<double>& parameters,
                               const std::vector<bool>& valid = {}) const;
    static double ComputeSeconds(double cycles, double ghz);
    static double ComputeJoules(double cycles, double ghz, double k = 1e-28);
    static uint64_t ProcessedBits(double fraction, uint64_t remainingBits);
    static double Reward(bool arrived, bool complete, double hopSum, double hop,
                         double compute, double energy);

  private:
    struct Layer
    {
        uint32_t input = 0, output = 0;
        std::vector<double> weights, bias;
    };
    static std::vector<double> Forward(std::vector<double> x, const std::vector<Layer>& net,
                                       bool float32 = false);
    uint32_t m_nodes = 0;
    bool m_paperActor = false;
    bool m_maskPadding = false;
    bool m_normalizeNoop = false;
    bool m_encodeMask = false;
    bool m_logTail = false;
    bool m_paperReference = false;
    bool m_bitGranularity = false;
    bool m_originalPartition = false;
    bool m_endpointCoding = false;
    bool m_terminalCandidates = false;
    double m_resultRatio = .05, m_completionTolerance = 1e-3;
    std::vector<Layer> m_actor, m_critic;
};
}
#endif
