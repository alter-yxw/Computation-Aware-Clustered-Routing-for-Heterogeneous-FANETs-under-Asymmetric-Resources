/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "ns3/cafbcr-compute-task-app.h"
#include "ns3/cafbcr-scenario-config.h"
#include "ns3/fanet-node.h"
#include "ns3/fanet-neighbor.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/test.h"
#include "../../../scratch/ca-fbcr-common-demand.h"
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>

namespace ns3::cafbcr {

class CafbcrCommonDemandCase : public TestCase
{
public:
  enum Kind { ISOLATION, REPRODUCIBILITY, PAIRS, PAIR_REPRODUCIBILITY, BOUNDARIES };
  CafbcrCommonDemandCase(const char* name, Kind kind) : TestCase(name), m_kind(kind) {}

private:
  using Records = std::map<uint32_t, ComputeTaskRecord>;

  static void Noise(uint32_t count)
  {
    for (uint32_t i = 0; i < count; ++i) {
      auto random = CreateObject<UniformRandomVariable>();
      for (uint32_t j = 0; j < 5; ++j) random->GetValue();
    }
  }

  Records Trial(const std::string& name, uint32_t candidates, uint32_t noise = 0,
                uint32_t seed = 2025, uint64_t run = 7, uint32_t streamIndex = 1)
  {
    RngSeedManager::SetSeed(seed);
    RngSeedManager::SetRun(run);
    Noise(noise);
    NodeContainer nodes;
    nodes.Create(2);
    auto node = nodes.Get(1); // nonzero source identity in every trial
    InternetStackHelper stack;
    stack.Install(node);
    auto cluster = CreateObject<ClusterNode>();
    cluster->SetNodeId(node->GetId());
    node->AggregateObject(cluster);
    auto neighbors = cluster->GetObject<Neighbors>();
    for (uint32_t i = 0; i < candidates; ++i) {
      neighbors->AddNeighbor(Neighbors::NeighborInfo(i + 2, Ipv4Address(0x0a000001 + i),
        0xffff, Vector(), Vector(), 100, 1, Idle, Seconds(0), Seconds(10), 20, 0));
    }
    auto app = CreateObject<ComputeTaskClientApp>();
    app->SetRemoteAddress(Ipv4Address::GetLoopback());
    app->SetRemotePort(889);
    app->SetTaskWorkload(1.0, 100.0);
    app->SetTaskDataSize(64, 2048);
    app->SetTaskDeadline(MilliSeconds(1), Seconds(20));
    app->SetTaskInterval(MilliSeconds(10));
    app->SetObservationWindow(MilliSeconds(100), MilliSeconds(740), Seconds(1));
    app->AssignStreams(demand::TaskStream(streamIndex));
    node->AddApplication(app);
    app->SetStartTime(Seconds(0));
    app->SetStopTime(Seconds(1) + TimeStep(1));
    Simulator::Stop(Seconds(1) + TimeStep(2));
    Simulator::Run();
    Records result = app->GetTaskRecords();
    if (const char* parent = std::getenv("CAFBCR_DEMAND_TEST_OUTPUT")) {
      auto path = std::filesystem::path(parent) / name;
      NS_ABORT_MSG_IF(!std::filesystem::create_directories(path), "Trial output already exists");
      std::ofstream meta(path / "run.json");
      meta << "{\"seed\":" << seed << ",\"run\":" << run << ",\"candidates\":" << candidates
           << ",\"noise_objects\":" << noise << ",\"demand_stream\":" << demand::TaskStream(streamIndex)
           << ",\"demand_schema\":\"" << demand::SCHEMA << "\",\"formal_eligible\":false}\n";
      std::ofstream rows(path / "tasks.jsonl");
      rows << std::setprecision(17);
      for (const auto& [id, task] : result) {
        rows << "{\"source_node\":" << task.sourceNodeId << ",\"task_sequence\":" << task.taskId
             << ",\"generated_ns\":" << task.generatedAt.GetNanoSeconds()
             << ",\"workload_gflop\":" << task.workload << ",\"payload_bytes\":" << task.dataSize
             << ",\"relative_deadline_ns\":" << task.deadline.GetNanoSeconds()
             << ",\"selected_server\":\"" << task.selectedServer << "\"}\n";
      }
      NS_ABORT_MSG_IF(!meta || !rows, "Cannot write demand trial evidence");
    }
    Simulator::Destroy();
    return result;
  }

  static std::string DemandKey(const Records& records)
  {
    std::ostringstream out;
    out << std::setprecision(17);
    for (const auto& [id, task] : records) {
      out << task.sourceNodeId << ',' << task.taskId << ',' << task.generatedAt.GetNanoSeconds()
          << ',' << task.workload << ',' << task.dataSize << ',' << task.deadline.GetNanoSeconds() << '\n';
    }
    return out.str();
  }

  std::vector<std::pair<uint16_t, uint16_t>> Pairs(uint32_t nodes, uint32_t flows,
      uint32_t seed = 2025, uint64_t run = 7, uint32_t noise = 0)
  {
    RngSeedManager::SetSeed(seed);
    RngSeedManager::SetRun(run);
    Noise(noise);
    auto random = CreateObject<UniformRandomVariable>();
    random->SetStream(demand::UDP_STREAM);
    return demand::UdpPairs(nodes, flows, random);
  }

  void DoRun() override
  {
    if (m_kind == ISOLATION) {
      auto none = Trial("candidates-0", 0);
      auto one = Trial("candidates-1", 1);
      auto two = Trial("candidates-2", 2);
      auto seven = Trial("candidates-7", 7);
      auto noisy = Trial("candidates-7-noise", 7, 50);
      NS_TEST_EXPECT_MSG_EQ(none.size(), 64, "fixed generation schedule");
      for (const auto* records : {&one, &two, &seven, &noisy}) {
        NS_TEST_EXPECT_MSG_EQ((DemandKey(none) == DemandKey(*records)), true,
                              "all task demand fields match despite changed selection draws");
      }
      std::set<uint32_t> selected;
      for (const auto& [id, task] : seven) selected.insert(task.selectedServer.Get());
      NS_TEST_EXPECT_MSG_GT(selected.size(), 1, "the dynamic selector actually exercised random choices");
      NS_TEST_EXPECT_MSG_NE(none.at(0).selectedServer, one.at(0).selectedServer,
                            "fallback and dynamic decisions actually differ");
    } else if (m_kind == REPRODUCIBILITY) {
      auto baseline = DemandKey(Trial("repeat-base", 7));
      auto repeat = DemandKey(Trial("repeat-identical", 7));
      auto seed = DemandKey(Trial("changed-seed", 7, 0, 2026));
      auto run = DemandKey(Trial("changed-run", 7, 0, 2025, 8));
      auto stream = DemandKey(Trial("changed-stream", 7, 0, 2025, 7, 2));
      NS_TEST_EXPECT_MSG_EQ(baseline, repeat, "seed/run/stream replay");
      NS_TEST_EXPECT_MSG_NE(baseline, seed, "seed changes inputs");
      NS_TEST_EXPECT_MSG_NE(baseline, run, "run changes inputs");
      NS_TEST_EXPECT_MSG_NE(baseline, stream, "node stream changes inputs");
    } else if (m_kind == PAIRS) {
      NS_TEST_EXPECT_MSG_EQ(Pairs(1, 0).empty(), true, "one node supports zero flows");
      for (uint32_t n : {2U, 3U, 5U}) {
        auto pairs = Pairs(n, n * (n - 1));
        std::set<std::pair<uint16_t, uint16_t>> unique(pairs.begin(), pairs.end());
        NS_TEST_EXPECT_MSG_EQ(unique.size(), n * (n - 1), "all directed pairs sampled exactly once");
        for (uint32_t src = 0; src < n; ++src) {
          for (uint32_t dst = 0; dst < n; ++dst) {
            NS_TEST_EXPECT_MSG_EQ(unique.count({src, dst}), (src == dst ? 0 : 1),
                                  "includes last node, excludes self pairs");
          }
        }
      }
      auto large = Pairs(demand::MAX_NODES, demand::MAX_UDP_FLOWS);
      std::set<std::pair<uint16_t, uint16_t>> unique(large.begin(), large.end());
      NS_TEST_EXPECT_MSG_EQ(unique.size(), demand::MAX_UDP_FLOWS, "bounded memory at maximum supported input");
      for (auto [src, dst] : large) {
        NS_TEST_EXPECT_MSG_LT(src, demand::MAX_NODES, "valid source");
        NS_TEST_EXPECT_MSG_LT(dst, demand::MAX_NODES, "valid destination");
        NS_TEST_EXPECT_MSG_NE(src, dst, "no self pair");
      }
    } else if (m_kind == PAIR_REPRODUCIBILITY) {
      auto baseline = Pairs(20, 50);
      auto repeat = Pairs(20, 50, 2025, 7, 100);
      auto seed = Pairs(20, 50, 2026);
      auto run = Pairs(20, 50, 2025, 8);
      NS_TEST_EXPECT_MSG_EQ((baseline == repeat), true, "UDP stream unaffected by other RNGs");
      NS_TEST_EXPECT_MSG_EQ((baseline == seed), false, "UDP seed sensitivity");
      NS_TEST_EXPECT_MSG_EQ((baseline == run), false, "UDP run sensitivity");
    } else {
      NS_TEST_EXPECT_MSG_EQ(demand::Validate(1, 0).empty(), true, "empty UDP demand allowed");
      for (auto [n, f] : {std::pair{0U, 0U}, {1U, 1U}, {2U, 3U},
                           {demand::MAX_NODES + 1, 0U}, {1000U, demand::MAX_UDP_FLOWS + 1}}) {
        NS_TEST_EXPECT_MSG_EQ(demand::Validate(n, f).empty(), false, "reject impossible request before sampling");
        bool threw = false;
        try { Pairs(n, f); } catch (const std::invalid_argument&) { threw = true; }
        NS_TEST_EXPECT_MSG_EQ(threw, true, "sampler also guards invalid direct callers");
      }
      NS_TEST_EXPECT_MSG_EQ(demand::UdpPort(998), 9998, "port before reserved compute port");
      NS_TEST_EXPECT_MSG_EQ(demand::UdpPort(999), 10000, "skip compute port 9999");
      NS_TEST_EXPECT_MSG_EQ(demand::UdpPort(demand::MAX_UDP_FLOWS - 1), 65535, "last legal port");
      NS_TEST_EXPECT_MSG_EQ(demand::TaskStream(1) - demand::TaskStream(0), 2, "two streams per node");
      NS_TEST_EXPECT_MSG_LT(demand::TaskStream(demand::MAX_NODES - 1) + 1, demand::UDP_STREAM, "no reserved overlap");
      NS_TEST_EXPECT_MSG_EQ(demand::TaskStart(Seconds(1), 13).GetNanoSeconds(), 2300000000LL,
                            "node phase is an exact integer time");
    }
  }
  Kind m_kind;
};

class CafbcrBoundedTaskStartCase : public TestCase
{
public:
  CafbcrBoundedTaskStartCase()
      : TestCase("bounded starts keep all 500 task sources active before the cutoff") {}

private:
  void DoRun() override
  {
    for (uint32_t n : {1U, 100U, 200U, 300U, 400U, 500U, demand::MAX_NODES}) {
      NS_TEST_EXPECT_MSG_EQ(demand::BoundedTaskStart(Seconds(1), 0, n), Seconds(1), "first source");
      NS_TEST_EXPECT_MSG_LT(demand::BoundedTaskStart(Seconds(1), n - 1, n), Seconds(2), "last source stays inside one second");
    }
    for (auto [index, nodes] : {std::pair{0U, 0U}, {1U, 1U}, {0U, demand::MAX_NODES + 1}}) {
      bool threw = false;
      try { demand::BoundedTaskStart(Seconds(1), index, nodes); }
      catch (const std::invalid_argument&) { threw = true; }
      NS_TEST_EXPECT_MSG_EQ(threw, true, "invalid index/count rejected");
    }
    NS_TEST_EXPECT_MSG_EQ(demand::TaskStart(Seconds(1), 499), MilliSeconds(50900), "legacy schedule preserved");

    // Exercise actual client scheduling, without routing or packet traces.
    // Node types follow the current N=500 composition: 2 / 30 / 80 / 388.
    const auto types = std::vector<UavTypeConfig>{UavTypeConfig::CreateCommandNode(),
      UavTypeConfig::CreateRelayNode(), UavTypeConfig::CreateTaskNode(), UavTypeConfig::CreateSwarmNode()};
    for (auto [windowStart, stop] : {std::pair{1U, 40U}, {1U, 60U}, {30U, 150U}}) {
      NodeContainer nodes;
      nodes.Create(500);
      InternetStackHelper stack;
      stack.Install(nodes);
      std::vector<Ptr<ComputeTaskClientApp>> clients;
      for (uint32_t i = 0; i < nodes.GetN(); ++i) {
        const auto& type = types[i < 2 ? 0 : i < 32 ? 1 : i < 112 ? 2 : 3];
        auto app = CreateObject<ComputeTaskClientApp>();
        app->SetServerSelectionPolicy(ComputeTaskClientApp::ServerSelectionPolicy::FIXED_ASSIGNMENT);
        app->SetRemoteAddress(Ipv4Address::GetLoopback());
        app->SetTaskTransport([](const ComputeTaskRecord&, std::function<void()>) {});
        app->SetTaskInterval(Seconds(type.taskGenerationInterval));
        app->AssignStreams(demand::TaskStream(i));
        const Time start = demand::BoundedTaskStart(Seconds(windowStart), i, nodes.GetN());
        app->SetObservationWindow(start, Seconds(stop), Seconds(stop + 200));
        nodes.Get(i)->AddApplication(app);
        app->SetStartTime(start);
        app->SetStopTime(Seconds(stop) + TimeStep(1));
        clients.push_back(app);
      }
      Simulator::Stop(Seconds(stop) + TimeStep(2));
      Simulator::Run();
      uint64_t total = 0;
      for (uint32_t i = 0; i < clients.size(); ++i) {
        const auto& records = clients[i]->GetTaskRecords();
        const uint32_t expected = stop == 40 ? (i < 2 ? 4 : i < 112 ? 2 : 1)
            : stop == 60 ? (i < 2 ? 6 : i < 32 ? 3 : 2)
                         : (i < 2 ? 12 : i < 32 ? 6 : i < 112 ? 4 : 3);
        NS_TEST_EXPECT_MSG_EQ(records.size(), expected, "every source generates its complete task schedule");
        total += records.size();
        for (const auto& [id, task] : records)
          NS_TEST_EXPECT_MSG_LT(task.generatedAt, Seconds(stop), "exclusive generation cutoff");
      }
      NS_TEST_EXPECT_MSG_EQ(total, (stop == 40 ? 616U : stop == 60 ? 1038U : 1688U), "N500 demand denominator");
      Simulator::Destroy();
    }
  }
};

class CafbcrCommonDemandSuite : public TestSuite
{
public:
  CafbcrCommonDemandSuite() : TestSuite("cafbcr-common-demand", UNIT)
  {
    AddTestCase(new CafbcrCommonDemandCase("task inputs survive candidate and unrelated RNG changes", CafbcrCommonDemandCase::ISOLATION), TestCase::QUICK);
    AddTestCase(new CafbcrCommonDemandCase("task replay and seed-run-stream controls", CafbcrCommonDemandCase::REPRODUCIBILITY), TestCase::QUICK);
    AddTestCase(new CafbcrCommonDemandCase("finite complete and maximum-size UDP pair sampling", CafbcrCommonDemandCase::PAIRS), TestCase::QUICK);
    AddTestCase(new CafbcrCommonDemandCase("UDP pairs replay independently of unrelated RNGs", CafbcrCommonDemandCase::PAIR_REPRODUCIBILITY), TestCase::QUICK);
    AddTestCase(new CafbcrCommonDemandCase("input limits ports and node phases", CafbcrCommonDemandCase::BOUNDARIES), TestCase::QUICK);
    AddTestCase(new CafbcrBoundedTaskStartCase(), TestCase::QUICK);
  }
} g_cafbcrCommonDemandSuite;
} // namespace ns3::cafbcr
