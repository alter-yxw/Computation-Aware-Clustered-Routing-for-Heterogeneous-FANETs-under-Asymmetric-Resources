/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef QRCF_ROUTE_PLANNER_H
#define QRCF_ROUTE_PLANNER_H
#include "qrcf-learning.h"
#include "qrcf-neighbor-table.h"
#include "qrcf-settings.h"

namespace ns3::qrcf
{
struct RoutePlan
{
    enum class Kind { DIRECT, CANDIDATES, LOCAL_MINIMUM };
    Kind kind;
    Ipv4Address direct;
    std::vector<Ipv4Address> addresses; // same order as scores
    std::vector<CandidateScore> scores;
    std::size_t neighborCount = 0;
    uint64_t undefinedFilters = 0;
};
class RoutePlanner
{
  public:
    // Only observes received neighbor state and the supplied destination
    // coordinate. No cross-node Q, energy, velocity, or neighbor-table reads.
    // Input table is explicitly the caller's current, already-expired table.
    static RoutePlan Prepare(const Settings& settings, Vector self, const Motion& motion,
                             Ipv4Address destination, Vector destinationPosition,
                             const NeighborTable& table,
                             const std::map<Ipv4Address,double>& previousDistances,
                             const std::set<Ipv4Address>& unavailable = {});
};
} // namespace ns3::qrcf
#endif
