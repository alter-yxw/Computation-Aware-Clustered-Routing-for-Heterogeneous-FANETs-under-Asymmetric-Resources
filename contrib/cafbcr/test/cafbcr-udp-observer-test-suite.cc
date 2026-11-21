/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "../../../scratch/ca-fbcr-udp-observer.h"
#include "ns3/applications-module.h"
#include "ns3/error-model.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/pointer.h"
#include "ns3/string.h"
#include "ns3/test.h"
#include <cstdlib>

namespace ns3::cafbcr {

class CafbcrUdpObserverCase : public TestCase
{
public:
  enum Kind { CONNECTED, NO_ROUTE, DROPPED, DUPLICATE, TX_EDGE, RX_EDGE,
              EMPTY_WINDOW, NO_FLOWS, UNMATCHED, INACTIVE, EARLY_STOP };
  CafbcrUdpObserverCase(const char* name, const char* artifact, Kind kind)
    : TestCase(name), m_artifact(artifact), m_kind(kind) {}

private:
  void DuplicateFrame(Ptr<const Packet> frame)
  {
    if (m_duplicated) return;
    m_duplicated = true;
    Simulator::Schedule(TimeStep(1), &PointToPointNetDevice::Receive, m_receiver, frame->Copy());
  }

  static void ExtraDatagram(Ptr<Socket> socket)
  {
    auto packet = Create<Packet>(500);
    SeqTsHeader header;
    header.SetSeq(100);
    packet->AddHeader(header);
    socket->Send(packet);
  }

  void DoRun() override
  {
    const char* output = std::getenv("CAFBCR_UDP_TEST_OUTPUT");
    auto folder = std::filesystem::path(output ? output : "/tmp/cafbcr-udp-observer-tests") / m_artifact;
    NS_TEST_ASSERT_MSG_EQ(std::filesystem::exists(folder), false, "retain earlier test artifacts");
    std::filesystem::create_directories(folder);
    Time begin = Seconds(1), end = Seconds(4.2);
    if (m_kind == EMPTY_WINDOW) end = begin;
    if (m_kind == TX_EDGE) end = Seconds(2);
    // 512 application + 8 UDP + 20 IPv4 + 2 PPP bytes at 1 Mbps,
    // plus 1 ms propagation: last packet arrives exactly at the upper edge.
    if (m_kind == RX_EDGE) end = NanoSeconds(2005336000LL);
    UdpObservationWriter writer(folder.string(), begin, end);
    NodeContainer nodes;
    nodes.Create(2);
    InternetStackHelper internet;
    internet.Install(nodes);
    Ipv4Address source("10.7.0.1"), destination("10.7.0.2");
    NetDeviceContainer devices;
    if (m_kind != NO_ROUTE) {
      PointToPointHelper p2p;
      p2p.SetDeviceAttribute("DataRate", StringValue("1Mbps"));
      p2p.SetChannelAttribute("Delay", TimeValue(MilliSeconds(1)));
      devices = p2p.Install(nodes);
      Ipv4AddressHelper addresses;
      addresses.SetBase("10.7.0.0", "255.255.255.0");
      addresses.Assign(devices);
      m_receiver = DynamicCast<PointToPointNetDevice>(devices.Get(1));
      if (m_kind == DUPLICATE) {
        m_receiver->TraceConnectWithoutContext("PhyRxEnd", MakeCallback(&CafbcrUdpObserverCase::DuplicateFrame, this));
      }
      if (m_kind == DROPPED) {
        auto error = CreateObject<RateErrorModel>();
        error->SetAttribute("ErrorRate", DoubleValue(1));
        m_receiver->SetAttribute("ReceiveErrorModel", PointerValue(error));
      }
    }
    uint32_t cap = (m_kind == CONNECTED || m_kind == NO_ROUTE) ? 2 : 3;
    if (m_kind == DUPLICATE || m_kind == UNMATCHED) cap = 1;
    const Time start = m_kind == INACTIVE ? Seconds(5) : Seconds(1);
    Ptr<Socket> extra;
    if (m_kind != NO_FLOWS && m_kind != EARLY_STOP) {
      writer.AddFlow(0, nodes.Get(0)->GetId(), nodes.Get(1)->GetId(), source, destination,
                     9000, start, Seconds(4), MilliSeconds(500), 512, cap);
      if (m_kind != INACTIVE) {
        UdpServerHelper server(9000);
        auto servers = server.Install(nodes.Get(1));
        servers.Start(Seconds(0));
        servers.Stop(Seconds(4.1));
        UdpClientHelper client(destination, 9000);
        client.SetAttribute("MaxPackets", UintegerValue(cap));
        client.SetAttribute("Interval", TimeValue(MilliSeconds(500)));
        client.SetAttribute("PacketSize", UintegerValue(512));
        auto clients = client.Install(nodes.Get(0));
        clients.Start(start);
        clients.Stop(Seconds(4));
        writer.Attach(0, DynamicCast<UdpClient>(clients.Get(0)), DynamicCast<UdpServer>(servers.Get(0)));
        if (m_kind == UNMATCHED) {
          extra = Socket::CreateSocket(nodes.Get(0), UdpSocketFactory::GetTypeId());
          extra->Connect(InetSocketAddress(destination, 9000));
          Simulator::Schedule(Seconds(1.25), &ExtraDatagram, extra);
        }
      }
    }
    Simulator::Stop(Seconds(m_kind == EARLY_STOP ? 2.1 : 4.2));
    Simulator::Run();
    writer.Finalize();
    auto c = writer.GetCounts();
    uint64_t attempts = (m_kind == NO_FLOWS || m_kind == INACTIVE || m_kind == EARLY_STOP) ? 0 :
      (m_kind == NO_ROUTE ? 6 : cap);
    uint64_t windowAttempts = m_kind == EMPTY_WINDOW ? 0 : (m_kind == TX_EDGE ? 2 : attempts);
    uint64_t received = m_kind == NO_ROUTE || m_kind == DROPPED ? 0 : attempts;
    uint64_t unique = m_kind == EMPTY_WINDOW ? 0 :
      (m_kind == TX_EDGE || m_kind == RX_EDGE ? 2 : received);
    NS_TEST_EXPECT_MSG_EQ(c.attempts, attempts, "attempts include socket submission failures");
    NS_TEST_EXPECT_MSG_EQ(c.windowAttempts, windowAttempts, "half-open generation cohort");
    NS_TEST_EXPECT_MSG_EQ(c.unique, unique, "first eligible delivery per attempt");
    NS_TEST_EXPECT_MSG_EQ(c.accepted, (m_kind == NO_ROUTE ? 0 : attempts), "whole-app successful submissions");
    NS_TEST_EXPECT_MSG_EQ(c.failed, (m_kind == NO_ROUTE ? 6 : 0), "failed submissions remain observable");
    NS_TEST_EXPECT_MSG_EQ(c.receives, received + (m_kind == DUPLICATE || m_kind == UNMATCHED ? 1 : 0), "raw server receipts include extra copies");
    NS_TEST_EXPECT_MSG_EQ(c.duplicates, (m_kind == DUPLICATE ? 1 : 0), "duplicate never adds a unique delivery");
    NS_TEST_EXPECT_MSG_EQ(c.invalidReceives, (m_kind == UNMATCHED ? 1 : 0), "unmatched input retained and flagged");
    NS_TEST_EXPECT_MSG_EQ(c.errors, (m_kind == UNMATCHED || m_kind == EARLY_STOP ? 1 : 0), "measurement validity is separate from loss");
    NS_TEST_EXPECT_MSG_EQ(c.datagramBytes, unique * 512, "datagram bytes include existing sequence header");
    NS_TEST_EXPECT_MSG_EQ(c.payloadBytes, unique * 500, "useful bytes exclude existing sequence header");
    NS_TEST_EXPECT_MSG_EQ(c.delayNs, unique * 5336000, "delivery-weighted exact latency without integer-ms truncation");
    if (extra) extra->Close();
    Simulator::Destroy();
    m_receiver = nullptr;
  }

  std::string m_artifact;
  Kind m_kind;
  Ptr<PointToPointNetDevice> m_receiver;
  bool m_duplicated = false;
};

class CafbcrUdpObserverSuite : public TestSuite
{
public:
  CafbcrUdpObserverSuite() : TestSuite("cafbcr-udp-observer", UNIT)
  {
    using C = CafbcrUdpObserverCase;
    AddTestCase(new C("successful submissions stop the legacy client before its time limit", "connected", C::CONNECTED), TestCase::QUICK);
    AddTestCase(new C("missing route preserves all failed send attempts in the denominator", "no-route", C::NO_ROUTE), TestCase::QUICK);
    AddTestCase(new C("network loss is distinct from failed socket submission", "dropped", C::DROPPED), TestCase::QUICK);
    AddTestCase(new C("actual duplicate delivery counts once", "duplicate", C::DUPLICATE), TestCase::QUICK);
    AddTestCase(new C("generation includes lower edge and excludes upper edge", "tx-edge", C::TX_EDGE), TestCase::QUICK);
    AddTestCase(new C("reception exactly at the upper edge remains outside", "rx-edge", C::RX_EDGE), TestCase::QUICK);
    AddTestCase(new C("empty observation window retains null metric semantics", "empty-window", C::EMPTY_WINDOW), TestCase::QUICK);
    AddTestCase(new C("positive window with no flows has no PDR denominator", "no-flows", C::NO_FLOWS), TestCase::QUICK);
    AddTestCase(new C("unmatched datagram is retained as invalid measurement", "unmatched", C::UNMATCHED), TestCase::QUICK);
    AddTestCase(new C("inactive declared flow has zero attempts", "inactive", C::INACTIVE), TestCase::QUICK);
    AddTestCase(new C("early capture termination cannot claim a complete window", "early-stop", C::EARLY_STOP), TestCase::QUICK);
  }
} g_cafbcrUdpObserverSuite;

class CafbcrFixedUdpCase : public TestCase
{
public:
  enum Kind { CONNECTED, NO_ROUTE, RECOVERY, DROPPED, CLIPPED, EXACT_EDGE,
              ONE_TICK_LATER, INACTIVE, ZERO_BUDGET, WINDOW_EDGE, EARLY_CLIENT };
  CafbcrFixedUdpCase(const char* name, const char* artifact, Kind kind)
    : TestCase(name), m_artifact(artifact), m_kind(kind) {}

private:
  void DoRun() override
  {
    const char* output = std::getenv("CAFBCR_UDP_TEST_OUTPUT");
    auto folder = std::filesystem::path(output ? output : "/tmp/cafbcr-udp-observer-tests") / m_artifact;
    NS_TEST_ASSERT_MSG_EQ(std::filesystem::exists(folder), false, "retain previous artifacts");
    std::filesystem::create_directories(folder);
    const bool shortBudget = m_kind == CONNECTED || m_kind == NO_ROUTE ||
                             m_kind == INACTIVE || m_kind == EARLY_CLIENT;
    uint32_t budget = shortBudget ? 2 : 3;
    if (m_kind == CLIPPED) budget = 20;
    if (m_kind == ZERO_BUDGET) budget = 0;
    Time start = m_kind == INACTIVE ? Seconds(5) : Seconds(1);
    Time latest = Seconds(4);
    if (m_kind == CLIPPED) latest = Seconds(1.75);
    if (m_kind == EXACT_EDGE) latest = Seconds(2);
    if (m_kind == ONE_TICK_LATER) latest = Seconds(2) + TimeStep(1);
    const auto plan = udp_demand::FixedPlan(start, latest, MilliSeconds(500), budget);
    const Time observationEnd = m_kind == WINDOW_EDGE ? Seconds(2) : Seconds(4.2);
    UdpObservationWriter writer(folder.string(), Seconds(1), observationEnd, true);
    NodeContainer nodes;
    nodes.Create(2);
    InternetStackHelper internet;
    internet.Install(nodes);
    const Ipv4Address source("10.8.0.1"), destination("10.8.0.2");
    if (m_kind != NO_ROUTE) {
      PointToPointHelper p2p;
      p2p.SetDeviceAttribute("DataRate", StringValue("1Mbps"));
      p2p.SetChannelAttribute("Delay", TimeValue(MilliSeconds(1)));
      auto devices = p2p.Install(nodes);
      Ipv4AddressHelper addresses;
      addresses.SetBase("10.8.0.0", "255.255.255.0");
      addresses.Assign(devices);
      if (m_kind == DROPPED) {
        auto error = CreateObject<RateErrorModel>();
        error->SetAttribute("ErrorRate", DoubleValue(1));
        devices.Get(1)->SetAttribute("ReceiveErrorModel", PointerValue(error));
      }
      if (m_kind == RECOVERY) {
        auto ip = nodes.Get(0)->GetObject<Ipv4>();
        ip->SetDown(1);
        Simulator::Schedule(Seconds(1.25), &Ipv4::SetUp, ip, 1U);
      }
    }
    writer.AddFlow(0, 0, 1, source, destination, 9000, start, plan.stop,
                   MilliSeconds(500), 512, 0, plan.attempts);
    std::ofstream demand(folder / "udp-demand.csv");
    demand << "schema,flow_index,attempt_index,source_node,destination_node,source_ip,destination_ip,"
              "port,generation_ns,packet_size_bytes,useful_payload_bytes\n";
    for (uint64_t i = 0; i < plan.attempts; ++i) {
      demand << udp_demand::SCHEMA << ",0," << i << ",0,1," << source << ',' << destination
             << ",9000," << start.GetNanoSeconds() + static_cast<int64_t>(i) * 500000000LL << ",512,500\n";
    }
    demand.close();
    if (plan.attempts) {
      UdpServerHelper server(9000);
      auto servers = server.Install(nodes.Get(1));
      servers.Start(Seconds(0));
      servers.Stop(Seconds(4.1));
      UdpClientHelper client(destination, 9000);
      // EARLY_CLIENT deliberately violates the plan. The planned denominator
      // must survive; invalid demand is not silently reduced to one packet.
      client.SetAttribute("MaxPackets", UintegerValue(m_kind == EARLY_CLIENT ? 1 : 0));
      client.SetAttribute("PacketSize", UintegerValue(512));
      client.SetAttribute("Interval", TimeValue(MilliSeconds(500)));
      auto clients = client.Install(nodes.Get(0));
      clients.Start(start);
      clients.Stop(plan.stop);
      writer.Attach(0, DynamicCast<UdpClient>(clients.Get(0)), DynamicCast<UdpServer>(servers.Get(0)));
    }
    Simulator::Stop(Seconds(4.2));
    Simulator::Run();
    writer.Finalize();
    auto c = writer.GetCounts();
    // Expected counts are fixed by the fixture, independently of socket outcome.
    uint64_t expected = 3;
    if (shortBudget || m_kind == CLIPPED || m_kind == EXACT_EDGE) expected = 2;
    if (m_kind == INACTIVE || m_kind == ZERO_BUDGET) expected = 0;
    uint64_t actual = m_kind == EARLY_CLIENT ? 1 : expected;
    uint64_t accepted = m_kind == NO_ROUTE ? 0 : (m_kind == RECOVERY ? 2 : actual);
    uint64_t unique = m_kind == NO_ROUTE || m_kind == DROPPED ? 0 : accepted;
    if (m_kind == WINDOW_EDGE) unique = 2;
    NS_TEST_EXPECT_MSG_EQ(c.planned, expected, "predeclared packet demand");
    NS_TEST_EXPECT_MSG_EQ(c.attempts, actual, "socket failures never extend the attempt budget");
    NS_TEST_EXPECT_MSG_EQ(c.windowPlanned, (m_kind == WINDOW_EDGE ? 2 : expected), "fixed denominator respects the observation cohort");
    NS_TEST_EXPECT_MSG_EQ(c.windowAttempts, (m_kind == WINDOW_EDGE ? 2 : actual), "observed attempts are checked against the plan");
    NS_TEST_EXPECT_MSG_EQ(c.accepted, accepted, "accepted submissions differ without changing planned demand");
    NS_TEST_EXPECT_MSG_EQ(c.failed, actual - accepted, "failed submissions consume their own planned opportunity");
    NS_TEST_EXPECT_MSG_EQ(c.unique, unique, "delivery count remains independent of demand identity");
    NS_TEST_EXPECT_MSG_EQ(c.delayNs, unique * 5336000, "header timestamp follows the actual attempt even after failed sequence zero");
    NS_TEST_EXPECT_MSG_EQ(c.errors, (m_kind == EARLY_CLIENT ? 2 : 0), "missing attempts invalidate measurement without shrinking the plan");
    Simulator::Destroy();
  }
  std::string m_artifact;
  Kind m_kind;
};

class CafbcrExplicitUdpWindowCase : public TestCase
{
public:
  CafbcrExplicitUdpWindowCase() : TestCase("50 explicit-window flows retain 12000 attempts with different observation tails") {}
private:
  void DoRun() override
  {
    for (uint32_t observationStop : {350U, 450U}) {
      UdpObservationWriter writer("", Seconds(0), Seconds(observationStop), true, false);
      NodeContainer nodes;
      nodes.Create(2);
      InternetStackHelper internet;
      internet.Install(nodes);
      PointToPointHelper link;
      link.SetDeviceAttribute("DataRate", StringValue("100Mbps"));
      link.SetChannelAttribute("Delay", TimeValue(MilliSeconds(1)));
      auto devices = link.Install(nodes);
      Ipv4AddressHelper addresses;
      addresses.SetBase("10.9.0.0", "255.255.255.0");
      const auto ips = addresses.Assign(devices);
      for (uint32_t i = 0; i < 50; ++i) {
        const Time start = udp_demand::WindowStart(Seconds(30), MilliSeconds(500), i, 50);
        const auto plan = udp_demand::WindowPlan(start, Seconds(150), MilliSeconds(500));
        NS_TEST_EXPECT_MSG_EQ(plan.attempts, 240, "same per-flow budget despite staggered starts");
        NS_TEST_EXPECT_MSG_LT(start + MilliSeconds(500 * 239), Seconds(150), "last send before cutoff");
        const uint16_t port = 9000 + i;
        UdpServerHelper server(port);
        auto servers = server.Install(nodes.Get(1));
        servers.Start(Seconds(5));
        servers.Stop(Seconds(observationStop));
        UdpClientHelper client(ips.GetAddress(1), port);
        client.SetAttribute("MaxPackets", UintegerValue(0));
        client.SetAttribute("PacketSize", UintegerValue(512));
        client.SetAttribute("Interval", TimeValue(MilliSeconds(500)));
        auto clients = client.Install(nodes.Get(0));
        clients.Start(start);
        clients.Stop(plan.stop);
        writer.AddFlow(i, 0, 1, ips.GetAddress(0), ips.GetAddress(1), port,
                       start, plan.stop, MilliSeconds(500), 512, 0, plan.attempts);
        writer.Attach(i, DynamicCast<UdpClient>(clients.Get(0)), DynamicCast<UdpServer>(servers.Get(0)));
      }
      Simulator::Stop(Seconds(observationStop));
      Simulator::Run();
      writer.Finalize();
      auto counts = writer.GetCounts();
      NS_TEST_EXPECT_MSG_EQ(counts.planned, 12000, "fixed demand denominator");
      NS_TEST_EXPECT_MSG_EQ(counts.attempts, 12000, "actual application sends");
      NS_TEST_EXPECT_MSG_EQ(counts.unique, 12000, "all packets received on this controlled link");
      NS_TEST_EXPECT_MSG_EQ(counts.errors, 0, "observer agrees with declared schedule");
      Simulator::Destroy();
    }
  }
};

class CafbcrFixedUdpSuite : public TestSuite
{
public:
  CafbcrFixedUdpSuite() : TestSuite("cafbcr-fixed-udp-demand", UNIT)
  {
    using C = CafbcrFixedUdpCase;
    AddTestCase(new CafbcrExplicitUdpWindowCase(), TestCase::QUICK);
    AddTestCase(new C("connected client consumes exactly two planned opportunities", "fixed-connected", C::CONNECTED), TestCase::QUICK);
    AddTestCase(new C("failed socket submissions consume exactly the same two opportunities", "fixed-no-route", C::NO_ROUTE), TestCase::QUICK);
    AddTestCase(new C("route recovery preserves three identities despite sequence reuse", "fixed-recovery", C::RECOVERY), TestCase::QUICK);
    AddTestCase(new C("network loss cannot extend the fixed attempt schedule", "fixed-dropped", C::DROPPED), TestCase::QUICK);
    AddTestCase(new C("observation tail clips a larger nominal attempt budget", "fixed-clipped", C::CLIPPED), TestCase::QUICK);
    AddTestCase(new C("preinstalled stop excludes a send exactly at its upper edge", "fixed-exact-edge", C::EXACT_EDGE), TestCase::QUICK);
    AddTestCase(new C("one extra time tick admits the boundary send", "fixed-one-tick", C::ONE_TICK_LATER), TestCase::QUICK);
    AddTestCase(new C("late flow has no planned opportunities", "fixed-inactive", C::INACTIVE), TestCase::QUICK);
    AddTestCase(new C("zero budget installs no client", "fixed-zero-budget", C::ZERO_BUDGET), TestCase::QUICK);
    AddTestCase(new C("planned denominator respects an earlier observation upper edge", "fixed-window-edge", C::WINDOW_EDGE), TestCase::QUICK);
    AddTestCase(new C("premature client completion cannot shrink the planned denominator", "fixed-early-client", C::EARLY_CLIENT), TestCase::QUICK);
  }
} g_cafbcrFixedUdpSuite;

} // namespace ns3::cafbcr
