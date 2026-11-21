/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "cfr-policy.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>

namespace ns3::cfr
{
void Policy::Load(const std::string& path)
{
    std::ifstream in(path);
    std::string magic;
    uint32_t nodes;
    if (!(in >> magic >> nodes) || (magic != "CFR_PDDQN_V1" && magic != "CFR_PDDQN_V2" && magic != "CFR_PDDQN_V3" && magic != "CFR_PDDQN_V4" && magic != "CFR_PDDQN_V5" && magic != "CFR_PDDQN_V6" && magic != "CFR_PAPER_V1" && magic != "CFR_PAPER_V2" && magic != "CFR_PAPER_V3" && magic != "CFR_DIAG_ENDPOINT_V1") || nodes < 2 || nodes > 4096)
        throw std::runtime_error("Invalid CFR policy: " + path +
                                 "; generate it with contrib/cfr/tools/train.py --nodes N");
    m_nodes = nodes;
    m_endpointCoding = magic == "CFR_DIAG_ENDPOINT_V1";
    m_terminalCandidates = false;
    // The named diagnostic shares V2's numeric/task execution path. Its
    // distinct header must never be interpreted as an original-paper model.
    m_paperReference = magic == "CFR_PAPER_V1" || magic == "CFR_PAPER_V2" || magic == "CFR_PAPER_V3" || m_endpointCoding;
    // V2/V3 are separately named integer-bit reconstructions. Their header
    // keeps the same result-ratio/tolerance fields; half-up bit projection is
    // the declared granularity rule, not the author's completion rule.
    // V3 removes each hop's partition from the ORIGINAL task; V2 from the
    // remaining task. The two readings are never silently interchanged.
    m_bitGranularity = magic == "CFR_PAPER_V2" || magic == "CFR_PAPER_V3" || m_endpointCoding;
    m_originalPartition = magic == "CFR_PAPER_V3";
    m_resultRatio = .05; m_completionTolerance = 1e-3;
    if (m_paperReference && (!(in >> m_resultRatio >> m_completionTolerance) ||
        !std::isfinite(m_resultRatio) || m_resultRatio < 0 || m_resultRatio > 1 ||
        !std::isfinite(m_completionTolerance) || m_completionTolerance < 0 || m_completionTolerance > .01))
        throw std::runtime_error("Invalid CFR reference task semantics");
    if (m_bitGranularity && m_completionTolerance != 0)
        throw std::runtime_error("Integer-bit reference model must declare zero completion tolerance");
    if (m_endpointCoding)
    {
        int selector;
        if (!(in >> selector) || (selector != 0 && selector != 1))
            throw std::runtime_error("Invalid endpoint diagnostic selector");
        m_terminalCandidates = selector == 1;
    }
    m_paperActor = magic != "CFR_PDDQN_V1";
    m_logTail = magic == "CFR_PDDQN_V6";
    m_encodeMask = magic == "CFR_PDDQN_V5" || m_logTail;
    m_maskPadding = magic == "CFR_PDDQN_V3" || magic == "CFR_PDDQN_V4" || m_encodeMask || m_paperReference;
    m_normalizeNoop = magic == "CFR_PDDQN_V4" || m_encodeMask;
    auto read = [&](std::vector<Layer>& net, uint32_t input, uint32_t output) {
        uint32_t count;
        if (!(in >> count) || count < 2 || count > 8)
            throw std::runtime_error("Invalid CFR network depth");
        net.clear();
        for (uint32_t l = 0; l < count; ++l)
        {
            Layer layer;
            if (!(in >> layer.input >> layer.output) || layer.input != input ||
                layer.output == 0 || layer.output > 16384)
                throw std::runtime_error("Invalid CFR layer dimensions");
            layer.weights.resize(layer.input * layer.output);
            layer.bias.resize(layer.output);
            for (auto& w : layer.weights)
                if (!(in >> w) || !std::isfinite(w)) throw std::runtime_error("Invalid CFR weight");
            for (auto& b : layer.bias)
                if (!(in >> b) || !std::isfinite(b)) throw std::runtime_error("Invalid CFR bias");
            input = layer.output;
            net.push_back(std::move(layer));
        }
        if (input != output) throw std::runtime_error("Invalid CFR output dimensions");
    };
    read(m_actor, 6 * m_nodes + 7, m_nodes);
    read(m_critic, 7 * m_nodes + 7, m_nodes + 1);
    if (in >> magic) throw std::runtime_error("Trailing data in CFR model");
}

std::vector<double> Policy::Forward(std::vector<double> x, const std::vector<Layer>& net, bool float32)
{
    for (size_t l = 0; l < net.size(); ++l)
    {
        const auto& layer = net[l];
        if (x.size() != layer.input) throw std::runtime_error("CFR input dimension mismatch");
        std::vector<double> y(layer.output);
        for (uint32_t j = 0; j < layer.output; ++j)
        {
            if (float32)
            {
                // Match the reference PyTorch dtype, including |tanh|'s exact
                // endpoint rounding. Legacy formats retain double inference.
                float v = static_cast<float>(layer.bias[j]);
                for (uint32_t i = 0; i < layer.input; ++i)
                    v += static_cast<float>(layer.weights[j * layer.input + i]) * static_cast<float>(x[i]);
                y[j] = l + 1 == net.size() ? v : std::max(0.0f, v);
                continue;
            }
            double v = layer.bias[j];
            for (uint32_t i = 0; i < layer.input; ++i) v += layer.weights[j * layer.input + i] * x[i];
            y[j] = l + 1 == net.size() ? v : std::max(0.0, v);
        }
        x = std::move(y);
    }
    return x;
}

std::vector<double> Policy::Encode(const std::vector<Observation>& view, Vector here,
                                   Vector destination, double ghz, const TaskState& task) const
{
    std::vector<double> s(6 * m_nodes + 7, 0);
    for (const auto& n : view)
    {
        if (n.slot >= m_nodes) throw std::runtime_error("CFR slot exceeds trained model size");
        size_t p = 6 * n.slot;
        s[p] = (n.position.x - here.x) / 1000;
        s[p + 1] = (n.position.y - here.y) / 1000;
        s[p + 2] = (n.position.z - here.z) / 300;
        s[p + 3] = n.ghz / 5;
        s[p + 4] = n.hops / 2.0;
        s[p + 5] = 1;
    }
    size_t p = 6 * m_nodes;
    s[p] = (destination.x - here.x) / 1000;
    s[p + 1] = (destination.y - here.y) / 1000;
    s[p + 2] = (destination.z - here.z) / 300;
    s[p + 3] = task.originalBytes / 1500000.0;
    s[p + 4] = task.remainingCycles / 1.2e10;
    s[p + 5] = task.originalCycles > 0 ? task.remainingCycles / task.originalCycles : 0;
    s[p + 6] = ghz / 5;
    return s;
}

std::vector<double> Policy::Parameters(const std::vector<double>& state) const
{
    auto a = Forward(state, m_actor, m_paperReference);
    // Section 5.1 specifies |tanh|. V1 retains the first reproduction's sigmoid
    // so existing checkpoints never silently change their executed policy.
    for (auto& v : a)
        v = m_paperReference ? std::abs(std::tanh(static_cast<float>(v))) :
            m_paperActor ? std::abs(std::tanh(v)) : std::clamp(1.02 / (1 + std::exp(-v)) - .01, 0.0, 1.0);
    return a;
}

std::vector<double> Policy::Values(const std::vector<double>& state,
                                  const std::vector<double>& parameters,
                                  const std::vector<bool>& valid) const
{
    if (state.size() != 6 * m_nodes + 7 || parameters.size() != m_nodes || (!valid.empty() && valid.size() != m_nodes))
        throw std::runtime_error("CFR critic parameter/mask dimension mismatch");
    auto x = state;
    bool noop = m_normalizeNoop && (state[state.size() - 3] <= 0 || state.back() <= 0);
    for (uint32_t i = 0; i < m_nodes; ++i)
    {
        double parameter = noop ? 0 : parameters[i];
        // Invertible experimental critic coding: the exact full-work action
        // is represented by 2. Execution still receives the original [0,1].
        if (m_endpointCoding && static_cast<float>(parameter) == 1.0f) parameter = 2;
        // V6 extends V5 with the experimental critic's continuous input
        // transform. The actor's executed fraction remains unchanged.
        if (m_logTail) parameter = -std::log(std::max(1.0 - parameter, 1e-6)) / std::log(1e6);
        if (m_encodeMask) parameter += 1;
        x.push_back(m_maskPadding && !valid.empty() && !valid[i] ? 0 : parameter);
    }
    auto out = Forward(x, m_critic, m_paperReference);
    if (m_paperReference)
    {
        float mean = 0; uint32_t count = 0;
        for (uint32_t i = 0; i < m_nodes; ++i)
            if (valid.empty() || valid[i]) { mean += static_cast<float>(out[i + 1]); ++count; }
        mean /= std::max(1u, count);
        std::vector<double> q(m_nodes);
        for (uint32_t i = 0; i < m_nodes; ++i)
            q[i] = static_cast<float>(static_cast<float>(out[0]) + static_cast<float>(out[i + 1])) - mean;
        return q;
    }
    double mean = 0; uint32_t count = 0;
    for (uint32_t i = 0; i < m_nodes; ++i)
        if (!m_maskPadding || valid.empty() || valid[i]) { mean += out[i + 1]; ++count; }
    mean /= std::max(1u, count);
    std::vector<double> q(m_nodes);
    for (uint32_t i = 0; i < m_nodes; ++i) q[i] = out[0] + out[i + 1] - mean;
    return q;
}

Action Policy::Select(const std::vector<double>& s, const std::vector<bool>& valid,
                      int directDestination) const
{
    if (valid.size() != m_nodes) throw std::runtime_error("CFR mask dimension mismatch");
    auto a = Parameters(s);
    auto q = Values(s, a, valid);
    Action best;
    best.q = -std::numeric_limits<double>::infinity();
    for (uint32_t i = 0; i < m_nodes; ++i)
        if (valid[i] && std::isfinite(q[i]) && (best.next < 0 || q[i] > best.q))
            best = {static_cast<int>(i), a[i], q[i]};
    // Algorithm 2 lines 14-15 take precedence over the Q ranking, but not RP.
    if (directDestination >= 0 && static_cast<size_t>(directDestination) < valid.size() &&
        valid[directDestination])
    {
        best = {directDestination, a[directDestination], q[directDestination]};
        if (m_terminalCandidates)
        {
            // Same order/tie rule as Python: actor, zero, full. Only the
            // direct-destination coordinate changes; other proposals stay put.
            auto candidate = a;
            for (double fraction : {0.0, 1.0})
            {
                candidate[directDestination] = fraction;
                double value = Values(s, candidate, valid)[directDestination];
                if (value > best.q) best = {directDestination, fraction, value};
            }
        }
    }
    best.requestedFraction = best.fraction;
    return best;
}

double Policy::ComputeSeconds(double cycles, double ghz)
{
    return cycles <= 0 ? 0 : ghz > 0 ? cycles / (ghz * 1e9) : std::numeric_limits<double>::infinity();
}
double Policy::ComputeJoules(double cycles, double ghz, double k)
{
    return k * cycles * ghz * ghz * 1e18;
}
uint64_t Policy::ProcessedBits(double fraction, uint64_t remainingBits)
{
    // Match the Python reference: use the float32 actor value, then the
    // declared nearest-integer (half-up) projection. Completing one bit needs
    // fraction >= .5; for R bits the implied endpoint band is .5/R wide.
    double requested = static_cast<double>(static_cast<float>(fraction));
    if (!(requested >= 0 && requested <= 1))
        throw std::runtime_error("Offloading fraction must be in [0,1]");
    if (remainingBits == 0) return 0;
    double scaled = requested * static_cast<double>(remainingBits) + .5;
    if (scaled >= static_cast<double>(remainingBits)) return remainingBits;
    return static_cast<uint64_t>(std::floor(scaled));
}
double Policy::Reward(bool arrived, bool complete, double hopSum, double hop,
                      double compute, double energy)
{
    if (arrived && complete) return .08 * hopSum;
    return -.08 * (compute + (arrived ? 0 : hop)) - .92 * energy;
}
}
