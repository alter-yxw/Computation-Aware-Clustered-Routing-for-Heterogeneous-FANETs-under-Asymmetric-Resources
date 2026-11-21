/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "qrcf-route-planner.h"

namespace ns3::qrcf
{
RoutePlan RoutePlanner::Prepare(const Settings& settings, Vector self, const Motion& motion,
                                Ipv4Address destination, Vector target, const NeighborTable& table,
                                const std::map<Ipv4Address,double>& previousDistances,
                                const std::set<Ipv4Address>& unavailable)
{
    settings.Validate();
    RoutePlan plan{RoutePlan::Kind::LOCAL_MINIMUM,Ipv4Address::GetAny(),{},{},table.Entries().size(),0};
    const auto direct = table.Entries().find(destination);
    // The user explicitly authorized bypassing filtering for an in-range
    // destination known through a still-valid received neighbor entry.
    if (!unavailable.count(destination) && direct != table.Entries().end() &&
        CalculateDistance(self,direct->second.hello.position) <= settings.range)
    {
        plan.kind = RoutePlan::Kind::DIRECT; plan.direct = destination;
        return plan;
    }
    std::vector<Candidate> candidates;
    bool hasProgress = false;
    const double distanceToTarget = CalculateDistance(self,target);
    for (const auto& [address,entry] : table.Entries())
    {
        if (unavailable.count(address)) { continue; }
        const auto& peer = entry.hello;
        // The cylinder's input domain is the communication sphere, not the
        // full abstract cylinder, whose corners extend outside the sphere.
        if (CalculateDistance(self,peer.position) > settings.range) { continue; }
        const auto keep = settings.Filter(self,peer.position,target);
        if (!keep) { ++plan.undefinedFilters; continue; }
        if (!*keep) { continue; }
        const auto previous = previousDistances.find(address);
        const Scalar oldDistance = previous == previousDistances.end() ? Scalar{} : Scalar{previous->second};
        candidates.push_back({peer.nodeId,peer.position,peer.motion,peer.energy,
                              settings.Connection(self,motion,peer.position,peer.motion,oldDistance)});
        plan.addresses.push_back(address);
        hasProgress = hasProgress || CalculateDistance(peer.position,target) < distanceToTarget;
    }
    plan.scores = Learning::Evaluate(self,motion,target,candidates,settings.weights,
        [&settings](std::size_t c,double v,double lo,double hi) { return settings.Normalize(c,v,lo,hi); });
    if (hasProgress) { plan.kind = RoutePlan::Kind::CANDIDATES; }
    return plan;
}
} // namespace ns3::qrcf
