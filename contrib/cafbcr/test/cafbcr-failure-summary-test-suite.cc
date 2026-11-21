/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "../../../scratch/ca-fbcr-failure-summary.h"
#include "ns3/applications-module.h"
#include "ns3/internet-module.h"
#include "ns3/llc-snap-header.h"
#include "ns3/mobility-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/test.h"
#include "ns3/string.h"
#include "ns3/wifi-module.h"

namespace ns3::cafbcr {

class CafbcrFailureSummaryTestCase : public TestCase
{
public:
  enum Kind { DEDUP, PHASES, RECOVERED, SOURCES, APPLICATION, CUTOFF, UDP_UID, FRAMING };
  CafbcrFailureSummaryTestCase(const char* name, Kind kind) : TestCase(name), m_kind(kind) {}
private:
  Ptr<Packet> Task(uint32_t source, uint32_t sequence, bool reply = false)
  {
    auto p = Create<Packet>(64);
    CafbcrComputeTaskTag tag; tag.SetSourceNodeId(source); tag.SetTaskId(sequence);
    tag.SetWorkload(reply ? 0 : 1); p->AddPacketTag(tag); return p;
  }
  Ipv4Header Header(uint8_t protocol = 17)
  { Ipv4Header h; h.SetProtocol(protocol); h.SetTtl(1); return h; }
  void Frame(Ptr<Packet> p, bool llc)
  {
    auto h = Header(); h.SetPayloadSize(p->GetSize()); p->AddHeader(h);
    if (llc) { LlcSnapHeader l; l.SetType(0x0800); p->AddHeader(l); }
  }
  void DoRun() override
  {
    using F = FailureSummary;
    F f(Seconds(1)); auto p = Task(4, 1);
    if (m_kind == DEDUP) {
      for (unsigned i = 0; i < 3; ++i) f.IpDrop(Header(), p, Ipv4L3Protocol::DROP_TTL_EXPIRED, nullptr, 0);
      f.IncludeTask({4,1}, false, false, false, "");
      NS_TEST_EXPECT_MSG_EQ(f.GetCounts(F::REQUEST).dropEvents, 3, "three native callbacks remain three events");
      NS_TEST_EXPECT_MSG_EQ(f.GetCounts(F::REQUEST).evidence[F::TTL], 1, "one missing request, not three failures");
      NS_TEST_EXPECT_MSG_EQ(f.GetCounts(F::REQUEST).multiple, 0, "repeated same reason is not multiple reasons");
    } else if (m_kind == PHASES) {
      auto response = Task(4, 2, true);
      f.IpDrop(Header(), p, Ipv4L3Protocol::DROP_NO_ROUTE, nullptr, 0);
      f.IpDrop(Header(), response, Ipv4L3Protocol::DROP_TTL_EXPIRED, nullptr, 0);
      f.IncludeTask({4,1}, false, false, false, "");
      f.IncludeTask({4,2}, true, true, false, "");
      NS_TEST_EXPECT_MSG_EQ(f.GetCounts(F::REQUEST).evidence[F::NO_ROUTE], 1, "request phase");
      NS_TEST_EXPECT_MSG_EQ(f.GetCounts(F::REQUEST).evidence[F::TTL], 0, "response evidence cannot contaminate request");
      NS_TEST_EXPECT_MSG_EQ(f.GetCounts(F::REPLY).evidence[F::TTL], 1, "zero-work completion tag identifies reply");
    } else if (m_kind == RECOVERED) {
      f.IpDrop(Header(), p, Ipv4L3Protocol::DROP_TTL_EXPIRED, nullptr, 0);
      f.IncludeTask({4,1}, true, true, true, "");
      NS_TEST_EXPECT_MSG_EQ(f.GetCounts(F::REQUEST).dropEvents, 1, "drop is still observed");
      NS_TEST_EXPECT_MSG_EQ(f.GetCounts(F::REQUEST).missing, 0, "delivered request is not a final missing request");
      NS_TEST_EXPECT_MSG_EQ(f.GetCounts(F::REQUEST).evidence[F::TTL], 0, "successful recovery is excluded from missing evidence");
    } else if (m_kind == SOURCES) {
      f.IpDrop(Header(), p, Ipv4L3Protocol::DROP_NO_ROUTE, nullptr, 0);
      f.IncludeTask({4,1}, false, false, false, "");
      f.IncludeTask({5,1}, false, false, false, "");
      NS_TEST_EXPECT_MSG_EQ(f.GetCounts(F::REQUEST).missing, 2, "same sequence from different sources stays distinct");
      NS_TEST_EXPECT_MSG_EQ(f.GetCounts(F::REQUEST).unobserved, 1, "no drop evidence is not fabricated");
    } else if (m_kind == APPLICATION) {
      f.IncludeTask({4,1}, false, false, false, "send_failed");
      f.IncludeTask({4,2}, false, false, false, "no_server");
      f.IncludeTask({4,3}, true, true, false, "response_send_failed");
      NS_TEST_EXPECT_MSG_EQ(f.GetCounts(F::REQUEST).evidence[F::APP_SEND], 2, "known application send failure is separate from no route");
      NS_TEST_EXPECT_MSG_EQ(f.GetCounts(F::REPLY).evidence[F::APP_SEND], 1, "response send failure");
      NS_TEST_EXPECT_MSG_EQ(f.GetCounts(F::REQUEST).dropEvents, 0, "application events are not network drop callbacks");
    } else if (m_kind == CUTOFF) {
      Simulator::Schedule(Seconds(1)-TimeStep(1), [&] { f.IpDrop(Header(), p, Ipv4L3Protocol::DROP_TTL_EXPIRED, nullptr, 0); });
      Simulator::Schedule(Seconds(1), [&] { f.IpDrop(Header(), p, Ipv4L3Protocol::DROP_NO_ROUTE, nullptr, 0); });
      Simulator::Schedule(Seconds(1)+TimeStep(1), [&] { f.Drop(p,F::MAC_RETRY,true); });
      Simulator::Stop(Seconds(1)+TimeStep(2)); Simulator::Run();
      f.IncludeTask({4,1}, false, false, false, "");
      NS_TEST_EXPECT_MSG_EQ(f.GetCounts(F::REQUEST).dropEvents, 2, "inclusive task observation cutoff, no post-cutoff event");
      NS_TEST_EXPECT_MSG_EQ(f.GetCounts(F::REQUEST).multiple, 1, "one missing object can carry two evidence categories");
      NS_TEST_EXPECT_MSG_EQ(f.GetCounts(F::REQUEST).evidence[F::MAC_RETRY], 0, "post-cutoff retry not charged");
    } else if (m_kind == UDP_UID) {
      auto datagram = Create<Packet>(32); auto copy = datagram->Copy();
      f.m_udp.emplace(datagram->GetUid(), 0);
      f.IpDrop(Header(), copy, Ipv4L3Protocol::DROP_NO_ROUTE, nullptr, 0);
      f.IpDrop(Header(), copy, Ipv4L3Protocol::DROP_NO_ROUTE, nullptr, 0);
      f.IpDrop(Header(), Create<Packet>(32), Ipv4L3Protocol::DROP_NO_ROUTE, nullptr, 0);
      f.Missing(f.m_counts[F::UDP], f.m_udp.at(datagram->GetUid()));
      NS_TEST_EXPECT_MSG_EQ(f.GetCounts(F::UDP).dropEvents, 2, "registered native copies preserve attempt identity");
      NS_TEST_EXPECT_MSG_EQ(f.GetCounts(F::UDP).evidence[F::NO_ROUTE], 1, "one unique UDP attempt");
      NS_TEST_EXPECT_MSG_EQ(f.GetUnmatchedDropEvents(), 1, "unregistered/control traffic is not a test UDP loss");
    } else {
      auto framed = Task(4, 2); Frame(framed, true);
      WifiMacHeader h; h.SetType(WIFI_MAC_DATA);
      auto mpdu = Create<WifiMpdu>(framed, h);
      f.MpduDrop(WIFI_MAC_DROP_REACHED_RETRY_LIMIT, mpdu);
      auto arpPending = Task(4, 2); Frame(arpPending, false); f.ArpDrop(arpPending);
      f.IpDrop(Header(), p, Ipv4L3Protocol::DROP_ROUTE_ERROR, nullptr, 0);
      f.IpDrop(Header(1), p, Ipv4L3Protocol::DROP_NO_ROUTE, nullptr, 0);
      f.IncludeTask({4,1}, false, false, false, "");
      f.IncludeTask({4,2}, false, false, false, "");
      NS_TEST_EXPECT_MSG_EQ(f.GetCounts(F::REQUEST).evidence[F::MAC_RETRY], 1, "LLC data retry");
      NS_TEST_EXPECT_MSG_EQ(f.GetCounts(F::REQUEST).evidence[F::ARP], 1, "native ARP pending packet has IP header");
      NS_TEST_EXPECT_MSG_EQ(f.GetCounts(F::REQUEST).evidence[F::NO_ROUTE], 0, "route error and ICMP are not UDP no-route evidence");
      NS_TEST_EXPECT_MSG_EQ(f.GetCounts(F::REQUEST).evidence[F::OTHER], 1, "route error kept as other");
    }
    // None of the observation paths may allocate another ns-3 Packet UID.
    auto anchor = Create<Packet>(4);
    f.IpDrop(Header(), p, Ipv4L3Protocol::DROP_TTL_EXPIRED, nullptr, 0);
    auto after = Create<Packet>(4);
    NS_TEST_EXPECT_MSG_EQ(after->GetUid(), anchor->GetUid()+1, "observer does not allocate packets");
    Simulator::Destroy();
  }
  Kind m_kind;
};

class CafbcrFailureNativeCase : public TestCase
{
public:
  enum Kind { SOURCE_NO_ROUTE, FORWARD_NO_ROUTE, TTL_EXPIRED, WIFI_RETRY, ARP_TIMEOUT, REPLY_SEND_FAILED };
  CafbcrFailureNativeCase(const char* name, Kind kind) : TestCase(name), m_kind(kind) {}
private:
  void DoRun() override
  {
    const bool wireless = m_kind == WIFI_RETRY || m_kind == ARP_TIMEOUT;
    const Time end = m_kind == ARP_TIMEOUT ? Seconds(5) : Seconds(.5);
    NodeContainer nodes; nodes.Create(2); InternetStackHelper internet; internet.Install(nodes);
    NetDeviceContainer devices;
    if (wireless) {
      MobilityHelper mobility; auto positions = CreateObject<ListPositionAllocator>();
      positions->Add(Vector()); positions->Add(Vector(1000,0,0));
      mobility.SetPositionAllocator(positions); mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
      mobility.Install(nodes);
      YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
      channel.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange", DoubleValue(100));
      YansWifiPhyHelper phy; phy.SetChannel(channel.Create());
      WifiHelper wifi; wifi.SetStandard(WIFI_STANDARD_80211g);
      wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode", StringValue("ErpOfdmRate6Mbps"),
                                   "ControlMode", StringValue("ErpOfdmRate6Mbps"));
      WifiMacHelper mac; mac.SetType("ns3::AdhocWifiMac"); devices = wifi.Install(phy,mac,nodes);
    } else {
      PointToPointHelper link; link.SetDeviceAttribute("DataRate", StringValue("1Mbps"));
      link.SetChannelAttribute("Delay", TimeValue(MilliSeconds(1))); devices = link.Install(nodes);
    }
    Ipv4AddressHelper addresses; addresses.SetBase("10.71.0.0", "255.255.255.0");
    auto ips = addresses.Assign(devices);
    if (m_kind == FORWARD_NO_ROUTE || m_kind == TTL_EXPIRED) {
      Ipv4StaticRoutingHelper helper;
      helper.GetStaticRouting(nodes.Get(0)->GetObject<Ipv4>())->SetDefaultRoute(ips.GetAddress(1),1);
    }
    if (m_kind == TTL_EXPIRED)
      nodes.Get(0)->GetObject<Ipv4L3Protocol>()->SetAttribute("DefaultTtl", UintegerValue(1));
    if (m_kind == TTL_EXPIRED) {
      // A next hop must exist before native IpForward checks the TTL. Without
      // this route the fixture correctly reports NO_ROUTE instead of TTL.
      Ipv4StaticRoutingHelper helper;
      helper.GetStaticRouting(nodes.Get(1)->GetObject<Ipv4>())->SetDefaultRoute(ips.GetAddress(0),1);
    }
    if (m_kind == WIFI_RETRY) {
      auto cache = nodes.Get(0)->GetObject<Ipv4L3Protocol>()->GetInterface(1)->GetArpCache();
      auto entry = cache->Add(ips.GetAddress(1)); entry->SetMacAddress(devices.Get(1)->GetAddress());
      entry->MarkPermanent();
    }
    TaskObservationWriter tasks("",end,"{}",false);
    UdpObservationWriter udp("",Seconds(0),end,true,false);
    FailureSummary summary(end); summary.AttachUdp(udp);
    for (uint32_t i = 0; i < 2; ++i) summary.AttachNode(nodes.Get(i));
    auto client = CreateObject<ComputeTaskClientApp>();
    client->SetRemoteAddress((m_kind <= TTL_EXPIRED) ? Ipv4Address("10.99.0.2") : ips.GetAddress(1));
    client->SetRemotePort(9999); client->SetTaskWorkload(1,1); client->SetTaskDataSize(64,64);
    client->SetTaskDeadline(Seconds(.2),Seconds(.2)); client->SetTaskInterval(Seconds(10));
    client->SetUseTaskHeader(true); client->SetServerSelectionPolicy(ComputeTaskClientApp::ServerSelectionPolicy::FIXED_ASSIGNMENT);
    client->SetObservationWindow(Seconds(.1),Seconds(.11),end); client->AssignStreams(5000);
    nodes.Get(0)->AddApplication(client); client->SetStartTime(Seconds(.1));
    client->SetStopTime(end+TimeStep(1)); tasks.Attach(client);
    if (m_kind == REPLY_SEND_FAILED) {
      auto server = CreateObject<ComputeTaskServerApp>(); server->SetLocalPort(9999);
      server->SetComputeCapacity(100); server->SetUseTaskHeader(true); server->SetObservationStop(end);
      nodes.Get(1)->AddApplication(server); server->SetStartTime(Seconds(0)); server->SetStopTime(end+TimeStep(1));
      tasks.Attach(server);
      Simulator::Schedule(Seconds(.105), &Ipv4::SetDown, nodes.Get(1)->GetObject<Ipv4>(), 1);
    }
    Simulator::Stop(end+TimeStep(2)); Simulator::Run(); tasks.Finalize(); udp.Finalize(); summary.Finalize(tasks,udp);
    const auto& req = summary.GetCounts(FailureSummary::REQUEST);
    const auto& reply = summary.GetCounts(FailureSummary::REPLY);
    NS_TEST_EXPECT_MSG_EQ(req.missing, (m_kind == REPLY_SEND_FAILED ? 0 : 1), "actual application reception determines request missing");
    if (m_kind == SOURCE_NO_ROUTE) NS_TEST_EXPECT_MSG_EQ(req.evidence[FailureSummary::APP_SEND],1,"actual socket submission fails without inventing an IPv4 no-route trace");
    if (m_kind == FORWARD_NO_ROUTE) NS_TEST_EXPECT_MSG_EQ(req.evidence[FailureSummary::NO_ROUTE],1,"actual forwarding no route");
    if (m_kind == TTL_EXPIRED) NS_TEST_EXPECT_MSG_EQ(req.evidence[FailureSummary::TTL],1,"actual IPv4 TTL expiry");
    if (m_kind == WIFI_RETRY) NS_TEST_EXPECT_MSG_EQ(req.evidence[FailureSummary::MAC_RETRY],1,"actual native Wi-Fi retry exhaustion");
    if (m_kind == ARP_TIMEOUT) NS_TEST_EXPECT_MSG_EQ(req.evidence[FailureSummary::ARP],1,"actual native ARP pending timeout");
    if (m_kind == REPLY_SEND_FAILED) {
      NS_TEST_EXPECT_MSG_EQ(reply.missing,1,"request processed, source has no completion");
      NS_TEST_EXPECT_MSG_EQ(reply.evidence[FailureSummary::APP_SEND],1,"actual response submission failure");
    }
    NS_TEST_EXPECT_MSG_EQ((summary.CsvHeader().find("TaskReqNoRouteEvidence")!=std::string::npos),true,"explicit CSV evidence labels");
    Simulator::Destroy();
  }
  Kind m_kind;
};

class CafbcrFailureSummarySuite : public TestSuite
{
public:
  CafbcrFailureSummarySuite() : TestSuite("cafbcr-failure-summary",UNIT)
  {
    using S = CafbcrFailureSummaryTestCase;
    AddTestCase(new S("repeat callbacks are events, not new failed tasks",S::DEDUP),TestCase::QUICK);
    AddTestCase(new S("request and completion phases remain distinct",S::PHASES),TestCase::QUICK);
    AddTestCase(new S("drop followed by delivery is not charged as missing",S::RECOVERED),TestCase::QUICK);
    AddTestCase(new S("source and sequence form the complete task key",S::SOURCES),TestCase::QUICK);
    AddTestCase(new S("application failures do not pretend to be network drops",S::APPLICATION),TestCase::QUICK);
    AddTestCase(new S("cutoff and overlapping reasons are explicit",S::CUTOFF),TestCase::QUICK);
    AddTestCase(new S("only registered UDP attempts and their copies match",S::UDP_UID),TestCase::QUICK);
    AddTestCase(new S("native IP and LLC framing and typed reasons",S::FRAMING),TestCase::QUICK);
    using N = CafbcrFailureNativeCase;
    AddTestCase(new N("native task socket has no source route",N::SOURCE_NO_ROUTE),TestCase::QUICK);
    AddTestCase(new N("native relay has no route",N::FORWARD_NO_ROUTE),TestCase::QUICK);
    AddTestCase(new N("native relay expires the task TTL",N::TTL_EXPIRED),TestCase::QUICK);
    AddTestCase(new N("native Wi-Fi exhausts data retries",N::WIFI_RETRY),TestCase::QUICK);
    AddTestCase(new N("native ARP pending packet expires",N::ARP_TIMEOUT),TestCase::QUICK);
    AddTestCase(new N("native response socket fails after server processing",N::REPLY_SEND_FAILED),TestCase::QUICK);
  }
} g_cafbcrFailureSummarySuite;
} // namespace ns3::cafbcr
