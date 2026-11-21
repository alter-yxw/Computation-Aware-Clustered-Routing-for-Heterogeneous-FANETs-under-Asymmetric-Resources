/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "cfr-eng-local-adapter.h"
#include "ns3/abort.h"
#include "ns3/mobility-model.h"
#include "ns3/simulator.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-phy.h"
#include <cmath>
#include <limits>

namespace ns3::cfreng
{
void LocalAdapter::Install(const NodeContainer& nodes, const Ipv4InterfaceContainer& interfaces,
    const std::vector<cafbcr::UavTypeConfig>& configs, Time cutoff,
    const std::string& strategy, Observer observer)
{
    NS_ABORT_MSG_IF(!m_apps.empty() || nodes.GetN() != configs.size() || nodes.GetN() > 65535,
                    "Invalid CFR local installation");
    m_statistics = std::make_shared<TaskStatistics>();
    auto locations = std::make_shared<std::map<Ipv4Address, Ptr<MobilityModel>>>();
    auto identities = std::make_shared<std::map<Ipv4Address, uint32_t>>();
    for (uint32_t i = 0; i < nodes.GetN(); ++i) {
        (*locations)[interfaces.GetAddress(i)] = nodes.Get(i)->GetObject<MobilityModel>();
        (*identities)[interfaces.GetAddress(i)] = nodes.Get(i)->GetId();
        m_statistics->RegisterAddress(interfaces.GetAddress(i), i);
    }
    for (uint32_t i = 0; i < nodes.GetN(); ++i) {
        const auto& config = configs[i];
        auto node = nodes.Get(i);
        auto routing = node->GetObject<RoutingProtocol>();
        NS_ABORT_MSG_IF(!routing, "CFR-ENG routing is missing");
        routing->Configure(i, config.processTasks ? config.computeCapacity / FLOPS_PER_CYCLE : 0,
            nodes.GetN(), [locations](Ipv4Address destination) {
                auto it = locations->find(destination);
                NS_ABORT_MSG_IF(it == locations->end(), "Unknown CFR destination position");
                return it->second->GetPosition();
            });
        double watts = 0;
        for (uint32_t j = 0; j < node->GetNDevices(); ++j) {
            auto wifi = DynamicCast<WifiNetDevice>(node->GetDevice(j));
            if (wifi) watts = std::pow(10.0, (wifi->GetPhy()->GetTxPowerStart() - 30) / 10);
        }
        NS_ABORT_MSG_IF(config.taskQueueCapacity == std::numeric_limits<uint32_t>::max(),
                        "CFR waiting capacity overflow");
        auto app = CreateObject<TaskApplication>();
        // Local capacity counts waiting tasks; CFR reservations include service.
        app->Configure(routing, m_statistics, watts, config.taskQueueCapacity + 1);
        app->SetStrategy(strategy);
        app->SetEvaluationDeadlineOnly(true);
        const uint32_t nodeId = node->GetId();
        app->SetExecutionObserver([observer, identities, nodeId](const std::string& event, const TaskHeader& h) {
            const auto source = identities->find(h.source);
            NS_ABORT_MSG_IF(source == identities->end(), "Unknown CFR source identity");
            observer(cafbcr::ComputeTaskEvent{event, source->second, h.taskId, nodeId,
                Simulator::Now(), h.originalCycles * FLOPS_PER_CYCLE / 1e9,
                h.originalBytes, NanoSeconds(h.deadlineNs - h.createdNs), h.destination, "fixed_assignment"});
        });
        node->AddApplication(app);
        app->SetStartTime(Seconds(0));
        app->SetStopTime(cutoff + TimeStep(1));
        m_apps.push_back(app);
    }
}

void LocalAdapter::BindClient(uint32_t index, Ptr<cafbcr::ComputeTaskClientApp> client)
{
    NS_ABORT_MSG_IF(index >= m_apps.size(), "Unknown CFR task source");
    auto app = m_apps[index];
    client->SetTaskTransport([app](const cafbcr::ComputeTaskRecord& task, std::function<void()> complete) {
        // Divide workload and capacity by the same constant: service time is
        // exactly workload_gflop / capacity_gflops. No demand is resampled.
        app->Submit(task.taskId, task.workload * 1e9 / FLOPS_PER_CYCLE, task.dataSize,
                    task.selectedServer, task.deadline,
                    [complete](uint32_t) { complete(); });
    });
}
}
