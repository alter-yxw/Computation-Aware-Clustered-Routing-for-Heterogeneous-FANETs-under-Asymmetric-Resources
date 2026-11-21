/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "ns3/cafbcr-compute-task-app.h"
#include "ns3/fanet-node.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/inet-socket-address.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/test.h"
#include "../../../scratch/ca-fbcr-queue-observer.h"
#include <chrono>
#include <cstdlib>

namespace ns3::cafbcr {

class CafbcrQueueLoadCase : public TestCase
{
public:
  enum Kind { LIFECYCLE, CANCEL, CUTOFF, DISABLED };
  CafbcrQueueLoadCase(const char* name, Kind kind) : TestCase(name), m_kind(kind) {}

private:
  void Observe(const ComputeQueueState& state)
  {
    ++m_snapshots;
    if (state.event == "execution_cancelled" || state.event == "waiting_cancelled") ++m_cancelled;
    NS_TEST_EXPECT_MSG_EQ(state.waiting, m_server->GetCurrentQueueLength(), "trace reads the real queue");
    NS_TEST_EXPECT_MSG_EQ(state.busy, m_server->IsProcessing(), "busy is distinct from waiting");
    NS_TEST_EXPECT_MSG_EQ(m_server->GetTasksReceived(),
      m_server->GetTasksFinished() + state.waiting + uint32_t(state.busy) + m_server->GetTasksDropped() + m_cancelled,
      "actual received tasks partition into finished, busy, waiting, rejected and cancelled");
    if (m_kind == DISABLED) {
      NS_TEST_EXPECT_MSG_EQ(state.localLoad.has_value(), false, "unsynchronized is unknown, not zero");
      NS_TEST_EXPECT_MSG_EQ(m_local->GetCurrentTaskQueueLength(), 1, "legacy owner's queue untouched");
      NS_TEST_EXPECT_MSG_EQ(m_local->GetTaskQueueCapacity(), 4, "legacy owner's capacity untouched");
      NS_TEST_EXPECT_MSG_EQ(m_local->GetCurrentLoad(), .25, "legacy owner's load untouched");
    } else {
      NS_TEST_ASSERT_MSG_EQ(state.localLoad.has_value(), true, "opt-in snapshot is present");
      NS_TEST_EXPECT_MSG_EQ(*state.localWaiting, state.waiting, "local queue count equals actual waiting");
      NS_TEST_EXPECT_MSG_EQ(*state.localCapacity, 2, "actual waiting capacity replaces initial metadata");
      NS_TEST_EXPECT_MSG_EQ(*state.localLoad, state.waiting / 2.0, "load is Q_wait / Q_max");
    }
    NS_TEST_EXPECT_MSG_EQ(m_remote->PeekAll().at(0).currentLoad, .125f,
                          "local publication never edits a remote neighbor view");
  }

  void Check(uint32_t waiting, bool busy, uint32_t dropped, uint32_t finished)
  {
    NS_TEST_EXPECT_MSG_EQ(m_server->GetCurrentQueueLength(), waiting, "expected actual queue");
    NS_TEST_EXPECT_MSG_EQ(m_server->IsProcessing(), busy, "expected processor state");
    NS_TEST_EXPECT_MSG_EQ(m_server->GetTasksDropped(), dropped, "expected rejection count");
    NS_TEST_EXPECT_MSG_EQ(m_server->GetTasksFinished(), finished, "actual completions only");
    if (m_kind != DISABLED) NS_TEST_EXPECT_MSG_EQ(m_local->GetCurrentLoad(), waiting / 2.0, "expected published fraction");
  }

  void Duplicate(Ptr<Node> node)
  {
    auto socket = Socket::CreateSocket(node, UdpSocketFactory::GetTypeId());
    socket->Bind();
    CafbcrComputeTaskTag tag;
    tag.SetSourceNodeId(node->GetId()); tag.SetTaskId(1);
    tag.SetWorkload(1); tag.SetDataSize(64); tag.SetDeadline(5);
    auto packet = Create<Packet>(64); packet->AddPacketTag(tag);
    socket->SendTo(packet, 0, InetSocketAddress(Ipv4Address::GetLoopback(), 889));
    socket->Close();
  }

  void DoRun() override
  {
    NodeContainer nodes; nodes.Create(2);
    InternetStackHelper stack; stack.Install(nodes.Get(0));
    m_local = CreateObject<ClusterNode>();
    m_local->SetNodeId(0); m_local->SetComputeCapacity(1); m_local->SetTaskQueueCapacity(4);
    nodes.Get(0)->AggregateObject(m_local);
    if (m_kind == DISABLED) m_local->EnqueueTask();
    auto remote = CreateObject<ClusterNode>(); remote->SetNodeId(1);
    nodes.Get(1)->AggregateObject(remote); m_remote = remote->GetNeighborTable();
    m_remote->AddNeighbor(Neighbors::NeighborInfo(0, Ipv4Address("10.0.0.1"), 0xffff,
      Vector(), Vector(), 100, 1, Idle, Seconds(0), Seconds(100), 1, .125f));

    Time cutoff = m_kind == CUTOFF ? MilliSeconds(450) : Seconds(4);
    auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    const char* parent = std::getenv("CAFBCR_QUEUE_TEST_OUTPUT");
    auto path = parent ? std::filesystem::path(parent) / ("case-" + std::to_string(m_kind))
                       : std::filesystem::temp_directory_path() / ("cafbcr-queue-" + std::to_string(stamp));
    std::string metadata = "{\"queue_load_schema\":\"cafbcr-queue-load-v1\",\"queue_load_sync\":";
    metadata += m_kind == DISABLED ? "false" : "true";
    metadata += ",\"formal_eligible\":false}";
    TaskObservationWriter tasks(path.string(), cutoff, metadata);
    QueueLoadWriter queues(path.string(), cutoff);

    m_server = CreateObject<ComputeTaskServerApp>();
    m_server->SetLocalPort(889); m_server->SetComputeCapacity(1); m_server->SetQueueCapacity(2);
    // DISABLED deliberately uses the unchanged default, without calling the setter.
    if (m_kind != DISABLED) m_server->SetSynchronizeQueueLoad(true);
    m_server->SetObservationStop(cutoff);
    tasks.Attach(m_server); queues.Attach(m_server);
    m_server->TraceConnectWithoutContext("QueueState", MakeCallback(&CafbcrQueueLoadCase::Observe, this));
    nodes.Get(0)->AddApplication(m_server); m_server->SetStartTime(Seconds(0));
    m_server->SetStopTime(m_kind == CUTOFF ? cutoff + TimeStep(1) :
                         m_kind == CANCEL ? MilliSeconds(500) : MilliSeconds(3500));

    auto client = CreateObject<ComputeTaskClientApp>();
    client->AssignStreams(3000000); client->SetRemoteAddress(Ipv4Address::GetLoopback()); client->SetRemotePort(889);
    client->SetTaskWorkload(1, 1); client->SetTaskDataSize(64, 64); client->SetTaskDeadline(Seconds(5), Seconds(5));
    client->SetTaskInterval(MilliSeconds(100));
    client->SetObservationWindow(MilliSeconds(100), std::min(MilliSeconds(500), cutoff), cutoff);
    tasks.Attach(client); nodes.Get(0)->AddApplication(client);
    client->SetStartTime(Seconds(0)); client->SetStopTime(cutoff + TimeStep(1));

    Simulator::Schedule(MilliSeconds(50), &CafbcrQueueLoadCase::Check, this, 0, false, 0, 0);
    Simulator::Schedule(MilliSeconds(150), &CafbcrQueueLoadCase::Check, this, 0, true, 0, 0);
    Simulator::Schedule(MilliSeconds(250), &CafbcrQueueLoadCase::Check, this, 1, true, 0, 0);
    Simulator::Schedule(MilliSeconds(350), &CafbcrQueueLoadCase::Duplicate, this, nodes.Get(0));
    Simulator::Schedule(MilliSeconds(440), &CafbcrQueueLoadCase::Check, this, 2, true, 1, 0);
    if (m_kind == LIFECYCLE || m_kind == DISABLED) {
      Simulator::Schedule(MilliSeconds(1200), &CafbcrQueueLoadCase::Check, this, 1, true, 1, 1);
      Simulator::Schedule(MilliSeconds(2200), &CafbcrQueueLoadCase::Check, this, 0, true, 1, 2);
      Simulator::Schedule(MilliSeconds(3200), &CafbcrQueueLoadCase::Check, this, 0, false, 1, 3);
    }
    Simulator::Stop(cutoff + TimeStep(2)); Simulator::Run();
    tasks.Finalize(); queues.Finalize();
    Check(0, false, 1, (m_kind == LIFECYCLE || m_kind == DISABLED) ? 3 : 0);
    NS_TEST_EXPECT_MSG_EQ(m_server->GetTasksReceived(), 4, "duplicate request does not enter the real queue");
    NS_TEST_EXPECT_MSG_EQ(client->GetTasksGenerated(), 4, "fixed demand count");
    NS_TEST_EXPECT_MSG_EQ(m_cancelled, ((m_kind == CANCEL || m_kind == CUTOFF) ? 3 : 0), "all unfinished work cancelled at stop");
    NS_TEST_EXPECT_MSG_GT(m_snapshots, 10, "actual queue transitions were observed");
    m_server->TraceDisconnectWithoutContext("QueueState", MakeCallback(&CafbcrQueueLoadCase::Observe, this));
    Simulator::Destroy();
    m_server = nullptr; m_local = nullptr; m_remote = nullptr;
  }

  Kind m_kind;
  Ptr<ComputeTaskServerApp> m_server;
  Ptr<ClusterNode> m_local;
  Ptr<Neighbors> m_remote;
  uint32_t m_snapshots = 0;
  uint32_t m_cancelled = 0;
};

class CafbcrQueueLoadSuite : public TestSuite
{
public:
  CafbcrQueueLoadSuite() : TestSuite("cafbcr-queue-load", UNIT)
  {
    AddTestCase(new CafbcrQueueLoadCase("real queue lifecycle and unchanged remote view", CafbcrQueueLoadCase::LIFECYCLE), TestCase::QUICK);
    AddTestCase(new CafbcrQueueLoadCase("stop cancels active and waiting work", CafbcrQueueLoadCase::CANCEL), TestCase::QUICK);
    AddTestCase(new CafbcrQueueLoadCase("cutoff preserves observations before cleanup", CafbcrQueueLoadCase::CUTOFF), TestCase::QUICK);
    AddTestCase(new CafbcrQueueLoadCase("default disabled keeps existing owner state", CafbcrQueueLoadCase::DISABLED), TestCase::QUICK);
  }
} g_cafbcrQueueLoadSuite;
} // namespace ns3::cafbcr
