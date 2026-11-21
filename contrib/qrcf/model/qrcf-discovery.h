/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef QRCF_DISCOVERY_H
#define QRCF_DISCOVERY_H
#include "qrcf-settings.h"
#include "qrcf-packet.h"
#include "ns3/nstime.h"
#include <vector>

namespace ns3::qrcf
{
// Adaptive Hello state machine; transport and CT classification are separate.
// The approved sampling convention is actual elapsed time between own Hellos.
class Discovery
{
  public:
    explicit Discovery(Settings settings);
    HelloHeader MakeHello(uint32_t nodeId, uint64_t messageId, Vector position,
                          const Motion& motion, double energy) const;
    // Invoke after broadcasting. At the first sample, retain the initial HT.
    // Empty connections means no in-range CT samples (keep interval).
    // currentNeighbors includes all still-valid entries for gamma history.
    // An unresolved CT is an error result, never an implicitly skipped neighbor.
    bool AfterHello(Vector position, const Motion& motion, Time now,
                    const std::vector<Scalar>& connectionTimes,
                    const std::set<uint32_t>& currentNeighbors);
    double Interval() const { return m_interval; }
    double Gamma() const { return m_gamma; }
    Scalar LastPredictionError() const { return m_predictionError; }
  private:
    Settings m_settings;
    double m_interval;
    double m_gamma = 0;
    Scalar m_predictionError;
    struct Sample { Vector position; Motion motion; Time time; };
    std::optional<Sample> m_previous;
    std::optional<std::set<uint32_t>> m_previousNeighbors;
};
} // namespace ns3::qrcf
#endif
