/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef CFR_ENG_LOCAL_ADAPTER_H
#define CFR_ENG_LOCAL_ADAPTER_H

#include "ns3/cafbcr-compute-task-app.h"
#include "ns3/cafbcr-scenario-config.h"
#include "ns3/cfr-eng-task-application.h"
#include "ns3/ipv4-interface-container.h"
#include "ns3/node-container.h"

namespace ns3::cfreng
{
// Adapt the explicit CFR profile to local demand, capacity and observations.
// Other protocols continue to use their existing executors and selectors.
class LocalAdapter
{
  public:
    using Observer = std::function<void(const cafbcr::ComputeTaskEvent&)>;
    void Install(const NodeContainer& nodes, const Ipv4InterfaceContainer& interfaces,
                 const std::vector<cafbcr::UavTypeConfig>& configs, Time cutoff,
                 const std::string& strategy, Observer observer);
    void BindClient(uint32_t index, Ptr<cafbcr::ComputeTaskClientApp> client);
    static constexpr double FLOPS_PER_CYCLE = 16.0;
  private:
    std::vector<Ptr<TaskApplication>> m_apps;
    std::shared_ptr<TaskStatistics> m_statistics;
};
}
#endif
