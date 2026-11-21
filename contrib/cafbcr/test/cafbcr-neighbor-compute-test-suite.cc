/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "ns3/cafbcr-routing.h"
#include "ns3/cafbcr-compute-task-app.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/test.h"
#include "../../../scratch/ca-fbcr-neighbor-observer.h"
#include "../../../scratch/ca-fbcr-queue-observer.h"
#include <chrono>
#include <cstdlib>

namespace ns3::cafbcr {

class CafbcrNeighborComputeTestCase : public TestCase
{
public:
  enum Kind { MERGE, UNKNOWN, INVALID, EXPIRY_REINSERT, SELECTION, CONSUMERS, WIRE };
  CafbcrNeighborComputeTestCase(const char* name, Kind kind) : TestCase(name), m_kind(kind) {}
private:
  using Observation = Neighbors::ComputeObservation;
  static Ipv4Address PeerIp(uint16_t id) { return Ipv4Address(0x0a000000 + id); }
  template<class HeaderType> Ptr<Packet> PacketWith(const HeaderType& header)
  {
    auto packet = Create<Packet>(); packet->AddHeader(header); return packet;
  }
  void Hello(uint16_t id = 7, double capacity = 20.5, float load = .75f, double energy = 80)
  {
    HelloHeader header(id, 42, PeerIp(id), Vector(id == 8 ? 90 : 100, 0, 0),
                       Vector(), energy, .6, Head, Simulator::Now().GetSeconds());
    header.SetComputeCapacity(capacity); header.SetCurrentLoad(load);
    m_route->RecvHello(PacketWith(header), PeerIp(0), PeerIp(id));
  }
  void Partial(MessageType type, uint16_t id = 7)
  {
    auto time = Simulator::Now().GetSeconds();
    Vector pos(id == 8 ? 90 : 100, 0, 0);
    if (type == CAFBCR_CH_CALL) {
      m_route->RecvClusterHeadAnnouncement(PacketWith(
        ClusterHeadAnnounceHeader(id, 42, PeerIp(id), pos, Vector(), 70, .7, time)), PeerIp(0), PeerIp(id));
    } else if (type == CAFBCR_JOIN_REQ || type == CAFBCR_PROXY_JOIN_REQ) {
      m_local->SetRole(Member); // Observe the update without creating a cluster.
      auto packet = PacketWith(JoinClusterRequestHeader(id, PeerIp(id), pos, Vector(), 60, .8, Member, time));
      if (type == CAFBCR_JOIN_REQ) m_route->RecvJoinRequest(packet, PeerIp(0), PeerIp(id));
      else m_route->RecvProxyJoinRequest(packet, PeerIp(0), PeerIp(id));
      m_local->SetRole(Head);
    } else if (type == CAFBCR_CH_LIVE) {
      m_route->RecvClusterHeadBeacon(PacketWith(
        ClusterHeadBeaconHeader(id, 42, PeerIp(id), pos, Vector(), .9, time)), PeerIp(0), PeerIp(id));
    } else if (type == CAFBCR_CM_LIVE) {
      m_route->RecvClusterMemberBeacon(PacketWith(
        ClusterMemberBeaconHeader(id, 42, PeerIp(id), pos, Vector(), .5, time)), PeerIp(0), PeerIp(id));
    } else if (type == CAFBCR_JOIN_RESP) {
      m_local->SetRole(Member); m_local->SetClusterId(42);
      m_route->RecvJoinResponse(PacketWith(ClusterHeadResponseHeader(id, 42, PeerIp(id), time)), PeerIp(0), PeerIp(id));
      m_local->SetRole(Head);
    }
  }
  void Check(uint16_t id, Observation::Status status, double capacity, double load,
             std::optional<Time> received, double energy = -1)
  {
    const auto& table = m_neighbors->PeekAll();
    auto it = table.find(id);
    NS_TEST_ASSERT_MSG_EQ((it != table.end()), true, "neighbor exists");
    auto observation = m_neighbors->GetComputeObservation(id);
    NS_TEST_EXPECT_MSG_EQ(observation.status, status, "independent compute state");
    NS_TEST_EXPECT_MSG_EQ((observation.receivedAt == received), true, "receipt timestamp unchanged by partial messages");
    NS_TEST_EXPECT_MSG_EQ_TOL(it->second.computeCapacity, capacity, 1.1e-6, "capacity retained");
    NS_TEST_EXPECT_MSG_EQ_TOL(it->second.currentLoad, load, 1.1e-6, "load retained");
    if (energy >= 0) NS_TEST_EXPECT_MSG_EQ_TOL(it->second.ResEnergy, energy, 1.1e-6, "only energy-bearing messages update energy");
  }
  void MergeChecks()
  {
    Simulator::Schedule(MilliSeconds(100), [this] { Hello(); });
    const MessageType types[] = {CAFBCR_CH_CALL, CAFBCR_JOIN_REQ, CAFBCR_CH_LIVE, CAFBCR_CM_LIVE, CAFBCR_JOIN_RESP};
    for (uint32_t i = 0; i < 5; ++i) {
      Simulator::Schedule(MilliSeconds(200 + 100 * i), [this, type=types[i], i] {
        Partial(type); Check(7, Observation::FRESH, 20.5, .75, MilliSeconds(100), i == 0 ? 70 : 60);
      });
    }
    Simulator::Schedule(MilliSeconds(2100) - TimeStep(1), [this] {
      Check(7, Observation::FRESH, 20.5, .75, MilliSeconds(100), 60);
    });
    Simulator::Schedule(MilliSeconds(2100), [this] {
      Partial(CAFBCR_CH_LIVE); Check(7, Observation::EXPIRED, 20.5, .75, MilliSeconds(100), 60);
      NS_TEST_EXPECT_MSG_EQ(m_neighbors->IsNeighbor(7), true, "live neighbor can have expired compute fields");
    });
    Simulator::Schedule(MilliSeconds(2200), [this] {
      Hello(7, 80, .25f, 40); Check(7, Observation::FRESH, 80, .25, MilliSeconds(2200), 40);
    });
  }
  void UnknownChecks()
  {
    const MessageType types[] = {CAFBCR_CH_LIVE, CAFBCR_CH_CALL, CAFBCR_JOIN_REQ, CAFBCR_CM_LIVE};
    for (uint32_t i = 0; i < 4; ++i) {
      Simulator::Schedule(MilliSeconds(100 + i * 100), [this, type=types[i], i] {
        uint16_t id = 7 + i;
        Partial(type, id);
        Check(id, Observation::UNKNOWN, 1, 0, std::nullopt);
        NS_TEST_EXPECT_MSG_EQ(m_neighbors->HasUsableComputeState(id), false, "constructor defaults are unknown");
      });
    }
    Simulator::Schedule(MilliSeconds(500), [this] {
      Partial(CAFBCR_JOIN_RESP, 7); Check(7, Observation::UNKNOWN, 1, 0, std::nullopt, 0);
      Partial(CAFBCR_PROXY_JOIN_REQ, 55);
      NS_TEST_EXPECT_MSG_EQ(m_neighbors->IsNeighbor(55), false, "proxy origin is not a one-hop neighbor");
    });
  }
  void InvalidChecks()
  {
    Simulator::Schedule(MilliSeconds(100), [this] {
      Hello(7, -2, .25f); Check(7, Observation::INVALID, -2, .25, MilliSeconds(100));
    });
    Simulator::Schedule(MilliSeconds(200), [this] {
      Partial(CAFBCR_CH_LIVE); Check(7, Observation::INVALID, -2, .25, MilliSeconds(100));
      Hello(8, 0, 0); Check(8, Observation::INVALID, 0, 0, MilliSeconds(200));
    });
    Simulator::Schedule(MilliSeconds(300), [this] {
      Hello(7, 20, 1.5f); Check(7, Observation::INVALID, 20, 1.5, MilliSeconds(300));
    });
    Simulator::Schedule(MilliSeconds(400), [this] {
      Hello(7, 20, 1); Check(7, Observation::FRESH, 20, 1, MilliSeconds(400));
      NS_TEST_EXPECT_MSG_EQ(m_neighbors->HasUsableComputeState(7), true, "full waiting queue is valid information");
    });
  }
  void ExpiryChecks()
  {
    Simulator::Schedule(MilliSeconds(100), [this] { Hello(); });
    Simulator::Schedule(MilliSeconds(2100), [this] {
      // The periodic purge ran at 2.1? This callback was scheduled first: CA must
      // also handle an expired entry still present when the receipt starts.
      Hello(7, 40, .5f); Check(7, Observation::FRESH, 40, .5, MilliSeconds(2100));
      m_neighbors->DeleteNeighbor(7); Partial(CAFBCR_CH_LIVE);
      Check(7, Observation::UNKNOWN, 1, 0, std::nullopt, 0);
      Hello(); m_neighbors->Clear(); Partial(CAFBCR_CH_LIVE);
      Check(7, Observation::UNKNOWN, 1, 0, std::nullopt, 0);
    });
    Simulator::Schedule(MilliSeconds(2200), [this] {
      Hello(); m_route->SetHelloInterval(Seconds(10)); Partial(CAFBCR_CH_LIVE);
    });
    Simulator::Schedule(MilliSeconds(4200), [this] {
      Partial(CAFBCR_CH_LIVE); Check(7, Observation::EXPIRED, 20.5, .75, MilliSeconds(2200));
    });
    m_stop = Seconds(4.3);
  }
  void SelectionChecks()
  {
    auto app = CreateObject<ComputeTaskClientApp>();
    app->AssignStreams(3000000); app->SetRemoteAddress(Ipv4Address::GetLoopback());
    app->SetTaskWorkload(1, 1); app->SetTaskDataSize(64, 64); app->SetTaskDeadline(Seconds(10), Seconds(10));
    app->SetTaskInterval(MilliSeconds(500));
    app->SetObservationWindow(MilliSeconds(50), MilliSeconds(3560), Seconds(4));
    m_tasks->Attach(app); m_nodes.Get(0)->AddApplication(app);
    app->SetStartTime(Seconds(0)); app->SetStopTime(Seconds(4));
    Simulator::Schedule(MilliSeconds(10), [this] { Partial(CAFBCR_CH_LIVE); });
    Simulator::Schedule(MilliSeconds(100), [this] { Hello(); });
    Simulator::Schedule(MilliSeconds(1900), [this] { Partial(CAFBCR_CH_LIVE); });
    Simulator::Schedule(Seconds(3), [this] { Hello(); });
    Simulator::Schedule(MilliSeconds(3600), [this, app] {
      NS_TEST_ASSERT_MSG_EQ(app->GetTaskRecords().size(), 8, "all generated demands retained");
      uint32_t i = 0;
      for (const auto& [id, record] : app->GetTaskRecords()) {
        bool fallback = i == 0 || i == 5;
        NS_TEST_EXPECT_MSG_EQ(record.selectedServer, (fallback ? Ipv4Address::GetLoopback() : PeerIp(7)), "unknown/expired candidates fall back");
        NS_TEST_EXPECT_MSG_EQ(record.visibleStateStatus, (fallback ? "fixed_fallback" : "fresh"), "age has explicit availability");
        NS_TEST_EXPECT_MSG_EQ(record.visibleStateAge.has_value(), !fallback, "no fake zero age");
        if (!fallback) {
          Time received = i < 5 ? MilliSeconds(100) : Seconds(3);
          NS_TEST_EXPECT_MSG_EQ(*record.visibleStateAge, record.generatedAt - received, "receive age at selection");
        }
        ++i;
      }
    });
    m_stop = Seconds(4);
  }
  void ConsumerChecks()
  {
    m_local->SetFitnessMode(ClusterNode::HETEROGENEOUS_COMPUTE);
    m_local->SetComputeCapacity(5); m_local->SetOriginEnergy(100); m_local->updateResidualEnergy(100);
    Simulator::Schedule(MilliSeconds(100), [this] { Hello(7, 20, 0, 100); });
    Simulator::Schedule(MilliSeconds(500), [this] { Hello(8, 10, 0, 100); });
    Simulator::Schedule(MilliSeconds(600), [this] {
      m_local->UpdateNeighborDegree();
      NS_TEST_EXPECT_MSG_EQ_TOL(m_local->ComputeFitness(0,0,0,0,1), .475, 1e-9, "fresh neighbor defines normalization");
      NS_TEST_EXPECT_MSG_EQ(m_neighbors->GreedyNeighbor(Vector(200,0,0), Vector()), 7, "shared greedy uses fresh load");
      NS_TEST_EXPECT_MSG_EQ(m_route->GreedyReachableNextHop(Vector(200,0,0)), 7, "CA greedy uses fresh load");
    });
    Simulator::Schedule(MilliSeconds(800), [this] { Partial(CAFBCR_CH_LIVE, 7); Partial(CAFBCR_CH_LIVE, 8); });
    Simulator::Schedule(MilliSeconds(2100), [this] {
      CafbcrComputeTaskTag task;
      NS_TEST_EXPECT_MSG_EQ_TOL(m_local->ComputeFitness(0,0,0,0,1), .65, 1e-9, "expired capacity omitted from normalization");
      NS_TEST_EXPECT_MSG_EQ(m_neighbors->GreedyNeighbor(Vector(200,0,0), Vector()), 8, "unknown load receives no bonus");
      NS_TEST_EXPECT_MSG_EQ(m_route->GreedyReachableNextHop(Vector(200,0,0)), 8, "CA next hop changes on expiry");
      NS_TEST_EXPECT_MSG_EQ(m_route->GreedyComputeForward(task), PeerIp(8), "compute forwarding excludes expired");
      NS_TEST_EXPECT_MSG_EQ(m_route->ScheduleWithinCluster(task), PeerIp(8), "cluster computation excludes expired");
    });
    Simulator::Schedule(MilliSeconds(2500), [this] {
      CafbcrComputeTaskTag task;
      NS_TEST_EXPECT_MSG_EQ_TOL(m_local->ComputeFitness(0,0,0,0,1), 1.0, 1e-9, "own capacity remains known");
      NS_TEST_EXPECT_MSG_EQ(m_route->GreedyComputeForward(task), Ipv4Address::GetAny(), "no fresh compute candidate");
      NS_TEST_EXPECT_MSG_EQ(m_route->ScheduleWithinCluster(task), Ipv4Address::GetAny(), "no fresh cluster candidate");
    });
    // Untracked callers retain the original shared behavior, including default fields.
    auto legacy = CreateObject<Neighbors>();
    legacy->AddNeighbor(Neighbors::NeighborInfo(7, PeerIp(7), 42, Vector(100,0,0), Vector(),
      100, 1, Head, Seconds(0), Seconds(100), 20, 0));
    NS_TEST_EXPECT_MSG_EQ(legacy->GetComputeObservation(7).status, Observation::UNTRACKED, "freshness is opt-in");
    NS_TEST_EXPECT_MSG_EQ(legacy->GreedyNeighbor(Vector(200,0,0), Vector()), 7, "legacy calculation unchanged");
  }
  void WireChecks()
  {
    auto remote = m_nodes.Get(1)->GetObject<ClusterNode>();
    auto sender = m_nodes.Get(1)->GetObject<RoutingProtocol>();
    remote->SetComputeCapacity(20);
    PointToPointHelper link; link.SetDeviceAttribute("DataRate", StringValue("10Mbps"));
    link.SetChannelAttribute("Delay", TimeValue(MilliSeconds(1)));
    auto devices = link.Install(m_nodes);
    Ipv4AddressHelper addresses; addresses.SetBase("10.1.1.0", "255.255.255.0");
    auto interfaces = addresses.Assign(devices);
    m_route->NotifyInterfaceUp(1); sender->NotifyInterfaceUp(1);
    Simulator::Schedule(TimeStep(1), [this, sender] {
      for (auto route : {m_route, sender}) {
        route->m_helloTimer.Cancel(); route->m_clusteringTimer.Cancel(); route->m_beaconTimer.Cancel();
      }
    });
    auto server = CreateObject<ComputeTaskServerApp>();
    server->SetLocalPort(9999); server->SetComputeCapacity(20); server->SetQueueCapacity(2);
    server->SetSynchronizeQueueLoad(true); server->SetObservationStop(Seconds(3));
    m_tasks->Attach(server); m_queues->Attach(server);
    m_nodes.Get(1)->AddApplication(server); server->SetStartTime(Seconds(0)); server->SetStopTime(Seconds(3));
    auto client = CreateObject<ComputeTaskClientApp>();
    client->AssignStreams(3000000); client->SetRemoteAddress(interfaces.GetAddress(1)); client->SetRemotePort(9999);
    client->SetTaskWorkload(200, 200); client->SetTaskDataSize(64, 64);
    client->SetTaskDeadline(Seconds(10), Seconds(10)); client->SetTaskInterval(MilliSeconds(100));
    client->SetObservationWindow(MilliSeconds(20), MilliSeconds(230), Seconds(3));
    m_tasks->Attach(client); m_nodes.Get(0)->AddApplication(client);
    client->SetStartTime(Seconds(0)); client->SetStopTime(Seconds(3));
    Simulator::Schedule(MilliSeconds(50), [this, remote] {
      NS_TEST_EXPECT_MSG_EQ_TOL(remote->GetCurrentLoad(), 0, 1e-9, "busy with no waiting task has zero queue load");
      NS_TEST_EXPECT_MSG_EQ(m_neighbors->IsNeighbor(1), false, "no advertisement means no remote view");
    });
    Simulator::Schedule(MilliSeconds(140), [this, remote] {
      NS_TEST_EXPECT_MSG_EQ_TOL(remote->GetCurrentLoad(), .5, 1e-9, "actual queue publishes locally");
      NS_TEST_EXPECT_MSG_EQ(m_neighbors->IsNeighbor(1), false, "local write cannot update remote view");
    });
    Simulator::Schedule(MilliSeconds(150), &RoutingProtocol::SendHello, sender);
    Simulator::Schedule(MilliSeconds(180), [this] {
      auto state = m_neighbors->GetComputeObservation(1); m_wireReceipt = state.receivedAt;
      NS_TEST_ASSERT_MSG_EQ(m_wireReceipt.has_value(), true, "real HELLO arrived over UDP");
      NS_TEST_EXPECT_MSG_EQ((*m_wireReceipt > MilliSeconds(150)), true, "receipt age excludes channel transit");
      Check(1, Observation::FRESH, 20, .5, m_wireReceipt);
    });
    Simulator::Schedule(MilliSeconds(250), [this, remote] {
      NS_TEST_EXPECT_MSG_EQ_TOL(remote->GetCurrentLoad(), 1, 1e-9, "remote queue changed");
      Check(1, Observation::FRESH, 20, .5, m_wireReceipt);
    });
    Simulator::Schedule(MilliSeconds(300), &RoutingProtocol::SendClusterHeadBeacon, sender);
    Simulator::Schedule(MilliSeconds(350), [this] { Check(1, Observation::FRESH, 20, .5, m_wireReceipt); });
    Simulator::Schedule(Seconds(2), &RoutingProtocol::SendClusterHeadBeacon, sender);
    Simulator::Schedule(MilliSeconds(2200), [this] { Check(1, Observation::EXPIRED, 20, .5, m_wireReceipt); });
    Simulator::Schedule(MilliSeconds(2300), &RoutingProtocol::SendHello, sender);
    Simulator::Schedule(MilliSeconds(2400), [this] {
      auto state = m_neighbors->GetComputeObservation(1);
      NS_TEST_EXPECT_MSG_EQ(state.status, Observation::FRESH, "next real advertisement refreshes load");
      NS_TEST_EXPECT_MSG_EQ_TOL(m_neighbors->PeekAll().at(1).currentLoad, 1, 1e-6, "new queue state propagated");
      NS_TEST_EXPECT_MSG_EQ((*state.receivedAt > MilliSeconds(2300)), true, "new receive timestamp");
    });
  }
  void DoRun() override
  {
    m_stop = m_kind == EXPIRY_REINSERT ? Seconds(4.3) : m_kind == SELECTION ? Seconds(4) : Seconds(3);
    m_nodes.Create(m_kind == WIRE ? 2 : 1);
    InternetStackHelper stack; stack.Install(m_nodes);
    for (auto it = m_nodes.Begin(); it != m_nodes.End(); ++it) {
      auto local = CreateObject<ClusterNode>(); local->SetNodeId((*it)->GetId());
      (*it)->AggregateObject(local); local->SetRole(Head); local->SetClusterId(42);
      auto motion = CreateObject<ConstantPositionMobilityModel>(); (*it)->AggregateObject(motion);
      auto route = CreateObject<RoutingProtocol>(); route->SetHelloInterval(Seconds(1));
      route->SetIpv4((*it)->GetObject<Ipv4>()); (*it)->AggregateObject(route);
    }
    m_local = m_nodes.Get(0)->GetObject<ClusterNode>();
    m_route = m_nodes.Get(0)->GetObject<RoutingProtocol>(); m_neighbors = m_local->GetNeighborTable();
    auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    const char* parent = std::getenv("CAFBCR_NEIGHBOR_TEST_OUTPUT");
    auto path = parent ? std::filesystem::path(parent) / ("case-" + std::to_string(m_kind))
      : std::filesystem::temp_directory_path() / ("cafbcr-neighbor-" + std::to_string(stamp));
    m_tasks = std::make_unique<TaskObservationWriter>(path.string(), m_stop,
      "{\"neighbor_compute_tracking\":true,\"queue_load_schema\":\"cafbcr-queue-load-v1\","
      "\"queue_load_sync\":true,\"formal_eligible\":false}");
    m_writer = std::make_unique<NeighborComputeWriter>(path.string(), m_stop); m_writer->Attach(m_route);
    m_queues = std::make_unique<QueueLoadWriter>(path.string(), m_stop);
    switch (m_kind) {
      case MERGE: MergeChecks(); break;
      case UNKNOWN: UnknownChecks(); break;
      case INVALID: InvalidChecks(); break;
      case EXPIRY_REINSERT: ExpiryChecks(); break;
      case SELECTION: SelectionChecks(); break;
      case CONSUMERS: ConsumerChecks(); break;
      case WIRE: WireChecks(); break;
    }
    Simulator::Stop(m_stop + TimeStep(2)); Simulator::Run();
    m_tasks->Finalize(); m_writer->Finalize(); m_queues->Finalize();
    m_writer.reset(); m_tasks.reset(); m_queues.reset();
    Simulator::Destroy(); m_route = nullptr; m_neighbors = nullptr; m_local = nullptr;
  }
  Kind m_kind;
  NodeContainer m_nodes;
  Ptr<RoutingProtocol> m_route;
  Ptr<ClusterNode> m_local;
  Ptr<Neighbors> m_neighbors;
  Time m_stop = Seconds(3);
  std::optional<Time> m_wireReceipt;
  std::unique_ptr<TaskObservationWriter> m_tasks;
  std::unique_ptr<NeighborComputeWriter> m_writer;
  std::unique_ptr<QueueLoadWriter> m_queues;
};

class CafbcrNeighborComputeSuite : public TestSuite
{
public:
  CafbcrNeighborComputeSuite() : TestSuite("cafbcr-neighbor-compute", Type::UNIT)
  {
    using C = CafbcrNeighborComputeTestCase;
    AddTestCase(new C("partial control messages preserve advertised fields and receipt time", C::MERGE), TestCase::QUICK);
    AddTestCase(new C("partial-only neighbors remain unknown", C::UNKNOWN), TestCase::QUICK);
    AddTestCase(new C("invalid computation is quarantined until a valid HELLO", C::INVALID), TestCase::QUICK);
    AddTestCase(new C("exact expiry reinsertion deletion clearing and captured TTL", C::EXPIRY_REINSERT), TestCase::QUICK);
    AddTestCase(new C("actual client selects fresh state and records receive age", C::SELECTION), TestCase::QUICK);
    AddTestCase(new C("fitness and forwarding consumers honor freshness", C::CONSUMERS), TestCase::QUICK);
    AddTestCase(new C("actual queue propagates only through received UDP HELLO", C::WIRE), TestCase::QUICK);
  }
} g_cafbcrNeighborComputeSuite;
} // namespace ns3::cafbcr
