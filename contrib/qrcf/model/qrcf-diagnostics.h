/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef QRCF_DIAGNOSTICS_H
#define QRCF_DIAGNOSTICS_H
#include "ns3/nstime.h"
#include <cstdint>
#include <map>
#include <optional>
#include <ostream>
#include <set>

namespace ns3::qrcf
{
// Protocol-local observations, deliberately independent of FANET ClusterNode.
// Directed links follow this UAV's received Hellos, not omniscient connectivity.
class Diagnostics
{
  public:
    uint64_t helloTx = 0, helloRx = 0, duplicateHello = 0, invalidControl = 0;
    uint64_t controlTx = 0, controlBytes = 0, feedbackTx = 0, feedbackRx = 0;
    uint64_t controlL3Packets = 0;
    uint64_t qUpdates = 0, explorations = 0, exploitations = 0;
    uint64_t routeRequests = 0, noRoute = 0, localMinima = 0, linkFailures = 0;
    uint64_t undefinedFormulas = 0;
    uint64_t outOfRangeCtExclusions = 0;
    uint64_t valueTx = 0, valueRx = 0, bootstrapUpdates = 0;
    uint64_t outcomeTimeouts = 0, arpFailures = 0;

    void ObserveCandidates(std::size_t neighbors, std::size_t filtered);
    void ObserveHelloInterval(Time interval);
    void ObserveNeighbors(const std::set<uint32_t>& neighbors, Time now);
    double AverageNeighborCount() const;
    double AverageFilteredCount() const;
    double AverageHelloSeconds() const;
    uint64_t CompletedLinks() const { return m_completedLinks; }
    std::size_t OpenLinks() const { return m_linkStarts.size(); }
    double AverageCompletedLinkSeconds() const;
    static void WriteColumns(std::ostream& os);
    void WriteValues(std::ostream& os) const;
  private:
    uint64_t m_candidateSamples = 0, m_neighborSum = 0, m_filteredSum = 0;
    uint64_t m_intervalSamples = 0, m_completedLinks = 0;
    double m_intervalSum = 0, m_linkSeconds = 0;
    std::map<uint32_t,Time> m_linkStarts;
};
} // namespace ns3::qrcf
#endif
