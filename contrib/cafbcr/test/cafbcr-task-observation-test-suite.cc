/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "ns3/cafbcr-compute-task-app.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/inet-socket-address.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/test.h"
#include "ns3/string.h"
#include "../../../scratch/ca-fbcr-task-observer.h"
#include <chrono>
#include <cmath>
#include <cstdlib>

namespace ns3::cafbcr {
class CafbcrTaskObservationCase : public TestCase
{
public:
  enum Kind { ON_TIME, LATE, REQUEST_UNREACHED, FINISHED_UNACKNOWLEDGED,
              UNFINISHED, QUEUE_REJECTED, DEADLINE_BOUNDARY, OBSERVATION_BOUNDARY,
              AFTER_OBSERVATION, ZERO_TASKS, DUPLICATE_REPLY, NO_SERVER, SEND_FAILED,
              SAME_LOCAL_ID, STOP_GENERATING, GENERATION_EXCLUSIVE, DUPLICATE_REQUEST };
  CafbcrTaskObservationCase(const char* name, Kind kind) : TestCase(name), m_kind(kind) {}

private:
  Ptr<ComputeTaskClientApp> Client(Ptr<Node> node, Ipv4Address destination, Time start,
                                  Time stop, Time observation, double workload, Time deadline,
                                  Time interval = Seconds(5))
  {
    auto app = CreateObject<ComputeTaskClientApp>();
    app->SetRemoteAddress(destination);
    app->SetRemotePort(889);
    app->SetTaskWorkload(workload, workload);
    app->SetTaskDataSize(m_kind == SEND_FAILED ? 70000 : 64, m_kind == SEND_FAILED ? 70000 : 64);
    app->SetTaskDeadline(deadline, deadline);
    app->SetTaskInterval(interval);
    app->SetObservationWindow(start, stop, observation);
    node->AddApplication(app);
    app->SetStartTime(Seconds(0));
    // Leave the socket alive beyond the cutoff to also test its timestamp guard.
    app->SetStopTime(observation + TimeStep(3));
    m_writer->Attach(app);
    return app;
  }

  Ptr<ComputeTaskServerApp> Server(Ptr<Node> node, Time observation, uint32_t queue = 10)
  {
    auto app = CreateObject<ComputeTaskServerApp>();
    app->SetLocalPort(889);
    app->SetComputeCapacity(1.0);
    app->SetQueueCapacity(queue);
    app->SetObservationStop(observation);
    node->AddApplication(app);
    app->SetStartTime(Seconds(0));
    app->SetStopTime(observation + TimeStep(1));
    m_writer->Attach(app);
    return app;
  }

  void Reply(CafbcrComputeTaskTag request, Address destination, bool wrongSource)
  {
    request.SetWorkload(0);
    request.SetDataSize(0);
    if (wrongSource) request.SetSourceNodeId(request.GetSourceNodeId() + 1);
    auto packet = Create<Packet>(8);
    packet->AddPacketTag(request);
    m_responder->SendTo(packet, 0, destination);
  }

  void ReceiveRequest(Ptr<Socket> socket)
  {
    Address from;
    while (auto packet = socket->RecvFrom(from)) {
      CafbcrComputeTaskTag tag;
      NS_TEST_ASSERT_MSG_EQ(packet->PeekPacketTag(tag), true, "real request carries its identity");
      Time replyTime = Seconds(.2);
      if (m_kind == AFTER_OBSERVATION) replyTime = m_observation + TimeStep(1);
      if (m_kind == DUPLICATE_REPLY) {
        Simulator::Schedule(Seconds(.05), &CafbcrTaskObservationCase::Reply, this, tag, from, true);
        Simulator::Schedule(replyTime - Simulator::Now() + TimeStep(1),
                            &CafbcrTaskObservationCase::Reply, this, tag, from, false);
      }
      Simulator::Schedule(replyTime - Simulator::Now(),
                          &CafbcrTaskObservationCase::Reply, this, tag, from, false);
    }
  }

  void SendDuplicate(Ptr<Node> node)
  {
    auto socket = Socket::CreateSocket(node, UdpSocketFactory::GetTypeId());
    socket->Bind();
    CafbcrComputeTaskTag tag;
    tag.SetSourceNodeId(node->GetId()); tag.SetTaskId(0);
    tag.SetWorkload(.01); tag.SetDataSize(64); tag.SetDeadline(.3);
    auto packet = Create<Packet>(64);
    packet->AddPacketTag(tag);
    socket->SendTo(packet, 0, InetSocketAddress(Ipv4Address::GetLoopback(), 889));
    socket->Close();
  }

  void CheckCounts(Ptr<ComputeTaskClientApp> app, uint64_t g, uint64_t s,
                   uint64_t l, uint64_t due, uint64_t future)
  {
    auto v = app->GetTaskSummary();
    NS_TEST_EXPECT_MSG_EQ(v.generated, g, "generated demand count");
    NS_TEST_EXPECT_MSG_EQ(v.onTime, s, "on-time count");
    NS_TEST_EXPECT_MSG_EQ(v.late, l, "late count");
    NS_TEST_EXPECT_MSG_EQ(v.unconfirmedDue, due, "expired unconfirmed count");
    NS_TEST_EXPECT_MSG_EQ(v.unconfirmedFuture, future, "unexpired unconfirmed count");
    NS_TEST_EXPECT_MSG_EQ(v.generated, v.onTime + v.late + v.unconfirmedDue + v.unconfirmedFuture,
                          "exact partition of generated tasks");
    NS_TEST_EXPECT_MSG_EQ(app->GetTasksCompleted(), s + l, "late completions retained");
    NS_TEST_EXPECT_MSG_EQ(app->GetTasksMissedDeadline(), l + due, "no double counting of late tasks");
    NS_TEST_EXPECT_MSG_EQ(app->GetTasksTimeout(), due, "timeout getter excludes late acknowledgements");
  }

  void CheckTwoSources()
  {
    NodeContainer nodes;
    nodes.Create(3);
    InternetStackHelper stack;
    stack.Install(nodes);
    PointToPointHelper link;
    link.SetDeviceAttribute("DataRate", StringValue("100Mbps"));
    link.SetChannelAttribute("Delay", TimeValue(MicroSeconds(1)));
    auto dev0 = link.Install(NodeContainer(nodes.Get(0), nodes.Get(2)));
    auto dev1 = link.Install(NodeContainer(nodes.Get(1), nodes.Get(2)));
    Ipv4AddressHelper addresses;
    addresses.SetBase("10.1.0.0", "255.255.255.0");
    auto ip0 = addresses.Assign(dev0);
    addresses.SetBase("10.2.0.0", "255.255.255.0");
    auto ip1 = addresses.Assign(dev1);
    auto server = Server(nodes.Get(2), m_observation);
    auto first = Client(nodes.Get(0), ip0.GetAddress(1), Seconds(.1), Seconds(.2), m_observation, .01, Seconds(.3));
    auto second = Client(nodes.Get(1), ip1.GetAddress(1), Seconds(.1), Seconds(.2), m_observation, .01, Seconds(.3));
    Simulator::Stop(m_observation + TimeStep(4));
    Simulator::Run();
    CheckCounts(first, 1, 1, 0, 0, 0);
    CheckCounts(second, 1, 1, 0, 0, 0);
    NS_TEST_EXPECT_MSG_EQ(first->GetTaskRecords().begin()->first, 0, "first source sequence");
    NS_TEST_EXPECT_MSG_EQ(second->GetTaskRecords().begin()->first, 0, "same sequence at second source");
    NS_TEST_EXPECT_MSG_EQ(server->GetTasksFinished(), 2, "same local IDs remain separate at one server");
  }

  void DoRun() override
  {
    const char* output = std::getenv("CAFBCR_TASK_TEST_OUTPUT");
    std::string base = output ? output : "/tmp/cafbcr-task-observation-tests-" +
      std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
    m_observation = (m_kind == OBSERVATION_BOUNDARY || m_kind == AFTER_OBSERVATION)
      ? Seconds(.2) : Seconds(1);
    m_writer = std::make_unique<TaskObservationWriter>(base + "/case-" + std::to_string(m_kind),
      m_observation, "{\"evidence_kind\":\"finite_functional_test\",\"formal_eligible\":false}");
    if (m_kind == SAME_LOCAL_ID) {
      CheckTwoSources();
    } else {
      NodeContainer nodes;
      nodes.Create(2); // Use source ID 1 so a missing/default response source cannot pass.
      InternetStackHelper stack;
      stack.Install(nodes);
      auto node = nodes.Get(1);
      Time deadline = Seconds(.3), stop = Seconds(.15), interval = Seconds(5);
      double workload = .01;
      bool fake = m_kind == DEADLINE_BOUNDARY || m_kind == OBSERVATION_BOUNDARY ||
                  m_kind == AFTER_OBSERVATION || m_kind == DUPLICATE_REPLY;
      if (m_kind == LATE || m_kind == FINISHED_UNACKNOWLEDGED) { deadline = Seconds(.1); workload = .5; }
      if (m_kind == DEADLINE_BOUNDARY || m_kind == OBSERVATION_BOUNDARY) deadline = Seconds(.1);
      if (m_kind == UNFINISHED || m_kind == QUEUE_REJECTED) { workload = 2; deadline = Seconds(2); }
      if (m_kind == ZERO_TASKS) stop = Seconds(.1);
      if (m_kind == QUEUE_REJECTED) { stop = Seconds(.5); interval = Seconds(.1); }
      if (m_kind == GENERATION_EXCLUSIVE) { stop = Seconds(.3); interval = Seconds(.1); }
      if (m_kind == STOP_GENERATING) { stop = Seconds(.9); interval = Seconds(.1); }
      Ptr<ComputeTaskServerApp> server;
      if (fake) {
        m_responder = Socket::CreateSocket(node, UdpSocketFactory::GetTypeId());
        m_responder->Bind(InetSocketAddress(Ipv4Address::GetLoopback(), 889));
        m_responder->SetRecvCallback(MakeCallback(&CafbcrTaskObservationCase::ReceiveRequest, this));
      } else if (m_kind != REQUEST_UNREACHED && m_kind != NO_SERVER && m_kind != SEND_FAILED) {
        server = Server(node, m_observation, m_kind == QUEUE_REJECTED ? 1 : 10);
      }
      auto client = Client(node, m_kind == NO_SERVER ? Ipv4Address() : Ipv4Address::GetLoopback(),
                            Seconds(.1), stop, m_observation, workload, deadline, interval);
      if (m_kind == FINISHED_UNACKNOWLEDGED) client->SetStopTime(Seconds(.2));
      if (m_kind == STOP_GENERATING) Simulator::Schedule(Seconds(.25), &ComputeTaskClientApp::StopGenerating, client);
      if (m_kind == DUPLICATE_REQUEST) Simulator::Schedule(Seconds(.2), &CafbcrTaskObservationCase::SendDuplicate, this, node);
      // An independent deadline-time snapshot is scheduled before the reply's event.
      // Its order cannot affect the final inclusive classification.
      if (m_kind == DEADLINE_BOUNDARY) Simulator::Schedule(Seconds(.2), [client]() { client->GetTaskSummary(); });
      Simulator::Stop(m_observation + TimeStep(4));
      Simulator::Run();
      switch (m_kind) {
        case LATE: CheckCounts(client, 1, 0, 1, 0, 0); break;
        case REQUEST_UNREACHED: case FINISHED_UNACKNOWLEDGED: case NO_SERVER: case SEND_FAILED:
          CheckCounts(client, 1, 0, 0, 1, 0); break;
        case UNFINISHED: case AFTER_OBSERVATION: CheckCounts(client, 1, 0, 0, 0, 1); break;
        case QUEUE_REJECTED: CheckCounts(client, 4, 0, 0, 0, 4); break;
        case ZERO_TASKS:
          CheckCounts(client, 0, 0, 0, 0, 0);
          NS_TEST_EXPECT_MSG_EQ(std::isnan(client->GetAverageResponseTime()), true, "no response sample is NA");
          NS_TEST_EXPECT_MSG_EQ(std::isnan(client->GetDeadlineMissRate()), true, "empty denominator is NA");
          break;
        case GENERATION_EXCLUSIVE: case STOP_GENERATING: CheckCounts(client, 2, 2, 0, 0, 0); break;
        default: CheckCounts(client, 1, 1, 0, 0, 0); break;
      }
      if (m_kind == DEADLINE_BOUNDARY || m_kind == OBSERVATION_BOUNDARY) {
        const auto& task = client->GetTaskRecords().begin()->second;
        NS_TEST_EXPECT_MSG_EQ(task.acknowledgedAt.has_value(), true, "boundary response accepted");
        if (task.acknowledgedAt) NS_TEST_EXPECT_MSG_EQ(*task.acknowledgedAt, task.generatedAt + task.deadline, "exact deadline equality");
      }
      if (m_kind == UNFINISHED || m_kind == QUEUE_REJECTED) {
        NS_TEST_EXPECT_MSG_EQ(server->GetTasksStarted(), 1, "task actually started");
        NS_TEST_EXPECT_MSG_EQ(server->GetTasksProcessed(), 0, "unfinished execution is not processed");
        NS_TEST_EXPECT_MSG_EQ(std::isnan(server->GetAverageProcessingDelay()), true, "no completed execution sample");
      }
      if (m_kind == FINISHED_UNACKNOWLEDGED) NS_TEST_EXPECT_MSG_EQ(server->GetTasksFinished(), 1, "server finish is independent of source acknowledgement");
      if (m_kind == QUEUE_REJECTED) NS_TEST_EXPECT_MSG_EQ(server->GetTasksDropped(), 2, "explicit queue rejection evidence");
      if (m_kind == DUPLICATE_REQUEST) NS_TEST_EXPECT_MSG_EQ(server->GetTasksFinished(), 1, "duplicate request does not execute twice");
    }
    m_writer->Finalize();
    if (m_responder) m_responder->Close();
    Simulator::Destroy();
    m_responder = nullptr;
    m_writer.reset();
  }
  Kind m_kind;
  Time m_observation;
  Ptr<Socket> m_responder;
  std::unique_ptr<TaskObservationWriter> m_writer;
};

class CafbcrTaskObservationSuite : public TestSuite
{
public:
  CafbcrTaskObservationSuite() : TestSuite("cafbcr-task-observation", UNIT)
  {
    const char* names[] = {"on time with nonzero source identity", "late acknowledgement retained",
      "unreached request remains in denominator", "execution finished without acknowledgement",
      "running execution at cutoff is unfinished", "queue rejection and censoring separated",
      "inclusive deadline independent of snapshot order", "inclusive observation cutoff",
      "reply after observation excluded", "zero samples are NA", "invalid and duplicate replies ignored",
      "no server still counts generated demand", "local socket send failure counts demand",
      "two sources with same sequence use one server", "stop generation keeps receiver alive",
      "generation stop is exclusive", "duplicate request does not reexecute"};
    for (int i = 0; i <= CafbcrTaskObservationCase::DUPLICATE_REQUEST; ++i)
      AddTestCase(new CafbcrTaskObservationCase(names[i], static_cast<CafbcrTaskObservationCase::Kind>(i)), TestCase::QUICK);
  }
};
static CafbcrTaskObservationSuite g_cafbcrTaskObservationSuite;
} // namespace ns3::cafbcr
