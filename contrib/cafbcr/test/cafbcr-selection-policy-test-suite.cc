/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "ns3/cafbcr-compute-task-app.h"
#include "ns3/fanet-node.h"
#include "ns3/fanet-neighbor.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/test.h"
#include <sstream>
#include <iomanip>

namespace ns3::cafbcr {
class SelectionPolicyCase : public TestCase
{
public:
  enum Kind { FIXED_LIVE, FIXED_UNKNOWN, NO_TARGET, DEMAND_ISOLATION, NATIVE_DEFAULT };
  SelectionPolicyCase(const char* name, Kind kind) : TestCase(name), m_kind(kind) {}
private:
  using Records = std::map<uint32_t, ComputeTaskRecord>;
  Records Trial(bool fixed, bool tracked, bool noTarget = false)
  {
    RngSeedManager::SetSeed(2025); RngSeedManager::SetRun(7);
    auto node = CreateObject<Node>();
    auto cluster = CreateObject<ClusterNode>();
    cluster->SetNodeId(node->GetId());
    node->AggregateObject(cluster);
    InternetStackHelper stack; stack.Install(node);
    auto neighbors = cluster->GetNeighborTable();
    if (tracked) neighbors->EnableComputeFreshness();
    neighbors->AddNeighbor(Neighbors::NeighborInfo(1, Ipv4Address("10.0.0.2"), 1,
      Vector(), Vector(), 100, 1, Head, Seconds(0), MilliSeconds(250), 60, 0));
    auto app = CreateObject<ComputeTaskClientApp>();
    app->SetRemoteAddress(noTarget ? Ipv4Address::GetAny() : Ipv4Address::GetLoopback());
    if (fixed) app->SetServerSelectionPolicy(ComputeTaskClientApp::ServerSelectionPolicy::FIXED_ASSIGNMENT);
    app->SetTaskInterval(MilliSeconds(100));
    app->SetTaskWorkload(1, 90); app->SetTaskDataSize(64, 1024);
    app->SetTaskDeadline(Seconds(1), Seconds(10)); app->AssignStreams(3000000);
    app->SetObservationWindow(MilliSeconds(100), MilliSeconds(350), MilliSeconds(400));
    node->AddApplication(app); app->SetStartTime(Seconds(0)); app->SetStopTime(MilliSeconds(400));
    Simulator::Stop(MilliSeconds(400) + TimeStep(1)); Simulator::Run();
    Records result = app->GetTaskRecords();
    if (fixed) {
      NS_TEST_EXPECT_MSG_EQ(neighbors->PeekAll().count(1), 1u,
                            "Fixed selection must not call a purging neighbor query");
      for (const auto& [id, task] : result) {
        NS_TEST_EXPECT_MSG_EQ(task.selectedServer,
            (noTarget ? Ipv4Address::GetAny() : Ipv4Address::GetLoopback()), "Always use configured server");
        NS_TEST_EXPECT_MSG_EQ(task.selectionPolicy, "fixed_assignment", "Explicit assignment identity");
        NS_TEST_EXPECT_MSG_EQ(task.visibleStateStatus, "fixed_assignment", "No online-state claim");
        NS_TEST_EXPECT_MSG_EQ(task.visibleStateAge.has_value(), false, "No fabricated receive age");
      }
    }
    Simulator::Destroy();
    return result;
  }
  static std::string Demand(const Records& records)
  {
    std::ostringstream out; out << std::setprecision(17);
    for (const auto& [id, task] : records)
      out << task.sourceNodeId << ',' << id << ',' << task.generatedAt.GetNanoSeconds() << ','
          << task.workload << ',' << task.dataSize << ',' << task.deadline.GetNanoSeconds() << '\n';
    return out.str();
  }
  void DoRun() override
  {
    if (m_kind == DEMAND_ISOLATION) {
      auto native = Trial(false, false); auto fixed = Trial(true, false);
      NS_TEST_EXPECT_MSG_EQ((Demand(native) == Demand(fixed)), true, "Policy change retains every demand field");
      NS_TEST_EXPECT_MSG_EQ(native.at(0).selectedServer, Ipv4Address("10.0.0.2"), "Native path really differs");
    } else {
      auto records = Trial(m_kind != NATIVE_DEFAULT, m_kind == FIXED_UNKNOWN, m_kind == NO_TARGET);
      NS_TEST_ASSERT_MSG_EQ(records.size(), 3u, "Three demands generated even without a usable server");
      if (m_kind == NATIVE_DEFAULT) {
        NS_TEST_EXPECT_MSG_EQ(records.at(0).selectedServer, Ipv4Address("10.0.0.2"), "Native remains default");
        NS_TEST_EXPECT_MSG_EQ(records.at(0).selectionPolicy, "neighbor_tier_power_of_two", "Native policy retained");
        NS_TEST_EXPECT_MSG_EQ(records.at(2).selectionPolicy, "fixed_fallback", "Native still purges expired candidates");
      }
    }
  }
  Kind m_kind;
};
class SelectionPolicySuite : public TestSuite
{
public:
  SelectionPolicySuite() : TestSuite("cafbcr-selection-policy", UNIT)
  {
    AddTestCase(new SelectionPolicyCase("fixed ignores live neighbor candidates", SelectionPolicyCase::FIXED_LIVE), TestCase::QUICK);
    AddTestCase(new SelectionPolicyCase("fixed ignores unknown compute state", SelectionPolicyCase::FIXED_UNKNOWN), TestCase::QUICK);
    AddTestCase(new SelectionPolicyCase("unassigned demands remain in denominator", SelectionPolicyCase::NO_TARGET), TestCase::QUICK);
    AddTestCase(new SelectionPolicyCase("fixed and native demand fields match", SelectionPolicyCase::DEMAND_ISOLATION), TestCase::QUICK);
    AddTestCase(new SelectionPolicyCase("native remains default with expiry fallback", SelectionPolicyCase::NATIVE_DEFAULT), TestCase::QUICK);
  }
} g_selectionPolicySuite;
} // namespace ns3::cafbcr
