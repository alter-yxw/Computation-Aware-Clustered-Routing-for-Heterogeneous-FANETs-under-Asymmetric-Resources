/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "../../../scratch/ca-fbcr-network-observer.h"
#include "ns3/applications-module.h"
#include "ns3/error-model.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/llc-snap-header.h"
#include "ns3/point-to-point-module.h"
#include "ns3/test.h"
#include "ns3/string.h"
#include "ns3/wifi-module.h"
#include <cstdlib>

namespace ns3::cafbcr {
static std::filesystem::path NetworkFolder(const char* name)
{
  const char* output = std::getenv("CAFBCR_NETWORK_TEST_OUTPUT");
  auto folder = std::filesystem::path(output ? output : "/tmp/cafbcr-network-tests") / name;
  NS_ABORT_MSG_IF(std::filesystem::exists(folder), "Retain previous network test artifacts");
  std::filesystem::create_directories(folder);
  return folder;
}

class NetworkHopCase : public TestCase
{
public:
  enum Kind { SELF, ONE, TWO, THREE, LOSS, DUPLICATE, LOOP, FRAGMENT, MISSING, EMPTY, EDGE, INCOMPLETE, MIXED };
  NetworkHopCase(const char* name, const char* artifact, Kind kind) : TestCase(name), m_artifact(artifact), m_kind(kind) {}
private:
  void Duplicate(Ptr<const Packet> p)
  {
    if (m_duplicated) return;
    m_duplicated = true;
    Simulator::Schedule(TimeStep(1), &PointToPointNetDevice::Receive, m_receiver, p->Copy());
  }
  static void ChangeRoute(Ptr<Ipv4StaticRouting> route, Ipv4Address destination, Ipv4Address gateway, uint32_t interface)
  {
    for (uint32_t i = route->GetNRoutes(); i-- > 0; )
      if (route->GetRoute(i).GetDest() == destination) route->RemoveRoute(i);
    route->AddHostRouteTo(destination, gateway, interface);
  }
  void DoRun() override
  {
    auto folder = NetworkFolder(m_artifact);
    uint32_t n = m_kind == SELF ? 1 : (m_kind == THREE || m_kind == LOOP || m_kind == MIXED ? 4 : m_kind == TWO || m_kind == LOSS ? 3 : 2);
    NodeContainer nodes; nodes.Create(n);
    InternetStackHelper internet; internet.Install(nodes);
    Ipv4StaticRoutingHelper routing;
    std::vector<Ipv4InterfaceContainer> links;
    std::vector<NetDeviceContainer> devices;
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", StringValue("1Mbps"));
    p2p.SetChannelAttribute("Delay", TimeValue(MilliSeconds(1)));
    if (m_kind == FRAGMENT) p2p.SetDeviceAttribute("Mtu", UintegerValue(400));
    for (uint32_t j = 0; j + 1 < n; ++j) {
      uint32_t a = m_kind == LOOP && j == 1 ? 0 : j;
      devices.push_back(p2p.Install(nodes.Get(a), nodes.Get(j + 1)));
      Ipv4AddressHelper addresses;
      addresses.SetBase(Ipv4Address(0x0a140000 + (j << 8)), Ipv4Mask("255.255.255.0"));
      links.push_back(addresses.Assign(devices.back()));
    }
    Ipv4Address source = n == 1 ? Ipv4Address::GetLoopback() : links[0].GetAddress(0);
    Ipv4Address destination = n == 1 ? source : links.back().GetAddress(1);
    for (uint32_t j = 0; j + 1 < n; ++j) {
      auto ip = nodes.Get(j)->GetObject<Ipv4>();
      auto r = routing.GetStaticRouting(ip);
      uint32_t link = m_kind == LOOP && j == 1 ? 0 : j;
      uint32_t deviceIndex = m_kind == LOOP && j == 1 ? 1 : 0;
      auto gateway = m_kind == LOOP && j == 1 ? links[0].GetAddress(0) : links[link].GetAddress(1);
      r->AddHostRouteTo(destination, gateway, ip->GetInterfaceForDevice(devices[link].Get(deviceIndex)));
    }
    if (m_kind == LOOP) {
      auto ip = nodes.Get(0)->GetObject<Ipv4>();
      Simulator::Schedule(MilliSeconds(109), &ChangeRoute, routing.GetStaticRouting(ip), destination,
                          links[1].GetAddress(1), uint32_t(ip->GetInterfaceForDevice(devices[1].Get(0))));
    }
    if (n > 1) m_receiver = DynamicCast<PointToPointNetDevice>(devices.back().Get(1));
    if (m_kind == DUPLICATE)
      m_receiver->TraceConnectWithoutContext("PhyRxEnd", MakeCallback(&NetworkHopCase::Duplicate, this));
    if (m_kind == LOSS) {
      auto error = CreateObject<RateErrorModel>(); error->SetRate(1);
      m_receiver->SetReceiveErrorModel(error);
    }
    // Different defaults at source/destination disallow assuming a fixed TTL=64.
    nodes.Get(0)->GetObject<Ipv4L3Protocol>()->SetAttribute("DefaultTtl", UintegerValue(37));
    nodes.Get(n - 1)->GetObject<Ipv4L3Protocol>()->SetAttribute("DefaultTtl", UintegerValue(5));
    Time end = m_kind == EMPTY ? Seconds(0) : m_kind == EDGE ? MilliSeconds(100) : Seconds(.5);
    NetworkObservationWriter observer(folder.string(), Seconds(0), end, 889, true);
    for (uint32_t j = 0; j < n; ++j) if (!(m_kind == MISSING && j == 0)) observer.AttachIp(j, nodes.Get(j));
    UdpServerHelper server(9000);
    auto servers = server.Install(nodes.Get(n - 1)); servers.Start(Seconds(0)); servers.Stop(Seconds(.5));
    UdpClientHelper client(destination, 9000);
    client.SetAttribute("MaxPackets", UintegerValue(m_kind == MIXED ? 2 : 1));
    client.SetAttribute("Interval", TimeValue(MilliSeconds(50)));
    client.SetAttribute("PacketSize", UintegerValue(m_kind == FRAGMENT ? 5000 : 512));
    auto clients = client.Install(nodes.Get(0)); clients.Start(Seconds(.1)); clients.Stop(Seconds(.5));
    observer.AttachUdp(0, 0, n - 1, source, destination, 9000,
                       DynamicCast<UdpClient>(clients.Get(0)), DynamicCast<UdpServer>(servers.Get(0)));
    if (m_kind == MIXED) {
      UdpServerHelper secondServer(9001);
      auto secondServers = secondServer.Install(nodes.Get(3)); secondServers.Start(Seconds(0)); secondServers.Stop(Seconds(.5));
      UdpClientHelper secondClient(destination, 9001);
      secondClient.SetAttribute("MaxPackets", UintegerValue(1)); secondClient.SetAttribute("PacketSize", UintegerValue(512));
      auto secondClients = secondClient.Install(nodes.Get(2)); secondClients.Start(Seconds(.2)); secondClients.Stop(Seconds(.5));
      observer.AttachUdp(1, 2, 3, links[2].GetAddress(0), destination, 9001,
                         DynamicCast<UdpClient>(secondClients.Get(0)), DynamicCast<UdpServer>(secondServers.Get(0)));
    }
    Simulator::Stop(m_kind == INCOMPLETE ? Seconds(.2) : Seconds(.5));
    Simulator::Run(); observer.Finalize();
    auto count = observer.GetHopCounts();
    uint64_t delivered = m_kind == LOSS || m_kind == EMPTY || m_kind == EDGE ? 0 : m_kind == MIXED ? 3 : 1;
    uint64_t unknown = m_kind == FRAGMENT || m_kind == MISSING ? 1 : 0;
    uint64_t hops = m_kind == LOOP ? 4 : m_kind == MIXED ? 7 : n - 1;
    NS_TEST_EXPECT_MSG_EQ(count.delivered, delivered, "only first eligible app delivery contributes");
    NS_TEST_EXPECT_MSG_EQ(count.observed, delivered - unknown, "missing or fragmented path is not a zero-hop delivery");
    NS_TEST_EXPECT_MSG_EQ(count.unknown, unknown, "unknown paths remain explicit");
    NS_TEST_EXPECT_MSG_EQ(count.sum, (delivered && !unknown ? hops : 0), "actual path includes first link and repeated nodes");
    NS_TEST_EXPECT_MSG_EQ(count.duplicates, (m_kind == DUPLICATE ? 1 : 0), "duplicate receipt does not change denominator");
    std::ofstream expected(folder / "fixture.json");
    expected << "{\"kind\":" << m_kind << ",\"nodes\":" << n << ",\"delivered\":" << delivered
             << ",\"unknown\":" << unknown << ",\"hop_sum\":" << (delivered && !unknown ? hops : 0) << "}\n";
    expected.close();
    Simulator::Destroy(); m_receiver = nullptr;
  }
  const char* m_artifact;
  Kind m_kind;
  Ptr<PointToPointNetDevice> m_receiver;
  bool m_duplicated = false;
};

class NetworkWifiCase : public TestCase
{
public:
  NetworkWifiCase(const char* name, const char* artifact, bool retry, bool completion=false)
    : TestCase(name), m_artifact(artifact), m_retry(retry), m_completion(completion) {}
private:
  static void Send(Ptr<Socket> socket) {
    uint8_t bytes[12] = {1}; auto packet = Create<Packet>(bytes, 12); socket->Send(packet);
  }
  void DoRun() override
  {
    auto folder = NetworkFolder(m_artifact);
    NodeContainer nodes; nodes.Create(2);
    MobilityHelper mobility; mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel"); mobility.Install(nodes);
    nodes.Get(1)->GetObject<MobilityModel>()->SetPosition(Vector(10, 0, 0));
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper phy; phy.SetChannel(channel.Create());
    WifiHelper wifi; wifi.SetStandard(WIFI_STANDARD_80211a);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode", StringValue("OfdmRate6Mbps"),
                                "ControlMode", StringValue("OfdmRate6Mbps"), "NonUnicastMode", StringValue("OfdmRate6Mbps"));
    WifiMacHelper mac; mac.SetType("ns3::AdhocWifiMac");
    auto devices = wifi.Install(phy, mac, nodes);
    InternetStackHelper internet; internet.Install(nodes);
    Ipv4AddressHelper addresses; addresses.SetBase("10.21.0.0", "255.255.255.0");
    auto ips = addresses.Assign(devices);
    NetworkObservationWriter observer(folder.string(), Seconds(0), Seconds(.5), 889, true, m_completion);
    for (uint32_t i = 0; i < 2; ++i) { observer.AttachIp(i, nodes.Get(i)); observer.AttachWifi(i, nodes.Get(i)); }
    auto receiver = Socket::CreateSocket(nodes.Get(1), UdpSocketFactory::GetTypeId());
    receiver->Bind(InetSocketAddress(Ipv4Address::GetAny(), 889));
    auto sender = Socket::CreateSocket(nodes.Get(0), UdpSocketFactory::GetTypeId());
    sender->Bind(InetSocketAddress(Ipv4Address::GetAny(), 889)); sender->SetAllowBroadcast(true);
    sender->Connect(InetSocketAddress(m_retry ? ips.GetAddress(1) : Ipv4Address("10.21.0.255"), 889));
    Simulator::Schedule(Seconds(.1), &Send, sender);
    if (m_retry) {
      auto receiverPhy = DynamicCast<WifiNetDevice>(devices.Get(1))->GetPhy();
      Simulator::Schedule(Seconds(.25), &WifiPhy::SetOffMode, receiverPhy);
      Simulator::Schedule(Seconds(.3), &Send, sender);
    }
    Simulator::Stop(Seconds(.5)); Simulator::Run(); observer.Finalize();
    if (m_completion) {
      auto c=observer.GetPhyCompletionCounts();
      NS_TEST_EXPECT_MSG_GT(c.candidates,1,"real MAC retry fixture has multiple local PHY attempts");
      NS_TEST_EXPECT_MSG_EQ(c.completed,c.candidates,"a failed wireless delivery can still have a completed local transmission");
      NS_TEST_EXPECT_MSG_EQ(c.errors,0,"native end events match each local retry attempt");
    }
    std::ofstream expected(folder / "fixture.json");
    expected << "{\"wifi\":true,\"retry\":" << (m_retry ? "true" : "false")
             << ",\"control_app_bytes\":12,\"control_ip_bytes\":40,\"control_mpdu_bytes\":76,\"nominal_control_ns\":128000}\n";
    expected.close(); sender->Close(); receiver->Close(); Simulator::Destroy();
  }
  const char* m_artifact;
  bool m_retry;
  bool m_completion;
};

class CompletionInjection : public WifiPhyListener
{
public:
  Ptr<WifiPhy> phy;bool synchronous=false,armed=false,fired=false;
  void NotifyTxStart(Time,double) override {
    if (!armed || fired) return;
    fired=true;
    if (synchronous) phy->SetOffMode();
    else Simulator::Schedule(MicroSeconds(40),&WifiPhy::SetOffMode,phy);
  }
  void NotifyRxStart(Time) override {}
  void NotifyRxEndOk() override {}
  void NotifyRxEndError() override {}
  void NotifyCcaBusyStart(Time,WifiChannelListType,const std::vector<Time>&) override {}
  void NotifySwitchingStart(Time) override {}
  void NotifySleep() override {}
  void NotifyOff() override {}
  void NotifyWakeup() override {}
  void NotifyOn() override {}
};

class NetworkPhyCompletionCase : public TestCase
{
public:
  NetworkPhyCompletionCase(const char* name,const char* artifact,unsigned kind)
    :TestCase(name),m_artifact(artifact),m_kind(kind) {}
private:
  const char* m_artifact;unsigned m_kind;
  void DoRun() override
  {
    const auto folder=NetworkFolder(m_artifact);
    NodeContainer nodes;nodes.Create(2);
    MobilityHelper mobility;mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");mobility.Install(nodes);
    nodes.Get(1)->GetObject<MobilityModel>()->SetPosition(Vector(10,0,0));
    YansWifiPhyHelper helper;helper.SetChannel(YansWifiChannelHelper::Default().Create());
    WifiHelper wifi;wifi.SetStandard(m_kind==9?WIFI_STANDARD_80211n:WIFI_STANDARD_80211a);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager","DataMode",StringValue("OfdmRate6Mbps"),
      "ControlMode",StringValue("OfdmRate6Mbps"),"NonUnicastMode",StringValue("OfdmRate6Mbps"));
    WifiMacHelper mac;mac.SetType("ns3::AdhocWifiMac");
    auto devices=wifi.Install(helper,mac,nodes);
    auto phy=DynamicCast<WifiNetDevice>(devices.Get(0))->GetPhy();
    CompletionInjection injection;injection.phy=phy;injection.synchronous=m_kind!=3 && m_kind!=13;
    if (m_kind==1) phy->RegisterListener(&injection);
    const auto start=MilliSeconds(100),nominal=MicroSeconds(128);
    const auto cut=m_kind==13?start+MicroSeconds(20):m_kind==4 || m_kind==6?
      start+MicroSeconds(64):m_kind==5?start+nominal:MilliSeconds(200);
    const auto begin=m_kind==11?start+MicroSeconds(64):Time();
    const auto stop=m_kind==13?start+MicroSeconds(50):m_kind==6?start+nominal+NanoSeconds(2):
      m_kind==7?start+MicroSeconds(64):cut+NanoSeconds(2);
    NetworkObservationWriter observer(folder.string(),begin,cut,889,true,true);
    observer.AttachWifi(0,nodes.Get(0));
    if (m_kind==2 || m_kind==3 || m_kind==13) phy->RegisterListener(&injection);
    uint8_t bytes[12]={1};auto packet=Create<Packet>(bytes,sizeof(bytes));
    UdpHeader udp;udp.SetSourcePort(889);udp.SetDestinationPort(889);packet->AddHeader(udp);
    Ipv4Header ip;ip.SetSource(Ipv4Address("10.53.1.1"));ip.SetDestination(Ipv4Address("10.53.1.255"));
    ip.SetProtocol(17);ip.SetTtl(1);ip.SetPayloadSize(packet->GetSize());packet->AddHeader(ip);
    LlcSnapHeader llc;llc.SetType(0x0800);packet->AddHeader(llc);
    WifiMacHeader header;header.SetType(m_kind==9?WIFI_MAC_QOSDATA:WIFI_MAC_DATA);
    header.SetAddr1(Mac48Address::GetBroadcast());header.SetAddr2(Mac48Address::ConvertFrom(devices.Get(0)->GetAddress()));
    header.SetAddr3(Mac48Address::GetBroadcast());
    if (m_kind==9) {header.SetQosTid(0);header.SetQosAckPolicy(WifiMacHeader::NO_ACK);}
    auto psdu=Create<WifiPsdu>(packet,header);
    WifiTxVector vector;vector.SetMode(WifiMode(m_kind==9?"HtMcs0":"OfdmRate6Mbps"));
    vector.SetPreambleType(m_kind==9?WIFI_PREAMBLE_HT_MF:WIFI_PREAMBLE_LONG);
    vector.SetChannelWidth(20);vector.SetGuardInterval(800);vector.SetNss(1);vector.SetNTx(1);
    if (m_kind==9) {
      auto second=Create<Packet>(20);
      psdu=Create<WifiPsdu>(std::vector<Ptr<WifiMpdu>>{Create<WifiMpdu>(packet,header),Create<WifiMpdu>(second,header)});
      vector.SetAggregation(true);
    }
    Simulator::Schedule(start,[&,psdu,vector] {
      injection.armed=true;
      phy->Send(psdu,vector);
    });
    if (m_kind==8) Simulator::Schedule(start+MicroSeconds(200),[&,vector] {
      auto retryHeader=header;retryHeader.SetRetry();
      // Same original UID, separate local attempt; native retries are tested
      // separately below, so this fixture does not claim a MAC retry decision.
      phy->Send(Create<WifiPsdu>(packet,retryHeader),vector);
    });
    Simulator::Stop(stop);Simulator::Run();observer.Finalize();
    auto c=observer.GetPhyCompletionCounts();
    const uint64_t expected=m_kind==8?2:1;
    NS_TEST_EXPECT_MSG_EQ(c.candidates,expected,"each local attempt is retained even for the same packet UID");
    NS_TEST_EXPECT_MSG_EQ(c.errors,0,"observed native callbacks match their own local attempt");
    if (m_kind==1 || m_kind==2 || m_kind==3) {
      NS_TEST_EXPECT_MSG_EQ(c.interrupted,1,"OFF is a retained interruption, not completion");
      NS_TEST_EXPECT_MSG_EQ(c.completedBytes,0,"interrupted frame bytes are not fractionally invented");
      NS_TEST_EXPECT_MSG_EQ(c.localNs,(m_kind==3?40000:0),"local TX interval ends at observed OFF");
    } else if (m_kind==4 || m_kind==6 || m_kind==7 || m_kind==13) {
      NS_TEST_EXPECT_MSG_EQ(c.open,1,"cutoff retains an open attempt even if End is captured later");
      NS_TEST_EXPECT_MSG_EQ(c.localNs,(m_kind==13?20000u:64000u),"only the observed interval up to cutoff contributes");
      NS_TEST_EXPECT_MSG_EQ(c.completedBytes,0,"completion after cutoff is not retroactive completion");
    } else {
      NS_TEST_EXPECT_MSG_EQ(c.completed,expected,"all expected MPDU End callbacks complete one local attempt");
      if (m_kind!=9) NS_TEST_EXPECT_MSG_EQ(c.localNs,128000*expected,"known Ofdm frame duration is independently fixed");
    }
    std::ofstream fixture(folder/"completion-fixture.json");
    fixture<<"{\"kind\":"<<m_kind<<",\"cutoff_ns\":"<<cut.GetNanoSeconds()<<",\"capture_ns\":"
      <<stop.GetNanoSeconds()<<",\"expected_candidates\":"<<expected<<",\"real_radio\":true,"
      "\"interruption_is_local_phy_activity_not_receiver_outage_proof\":true}\n";
    if (m_kind==1 || m_kind==2 || m_kind==3 || m_kind==13) phy->UnregisterListener(&injection);
    fixture.close();Simulator::Destroy();
  }
};

class NetworkPhyMissingEndCase : public TestCase
{
public:
  NetworkPhyMissingEndCase():TestCase("missing completion is retained as indeterminate, never zero airtime") {}
private:
  void DoRun() override {
    const auto folder=NetworkFolder("phy-missing-end");
    auto phy=CreateObject<YansWifiPhy>();
    PhyCompletionObserver observer(folder.string(),Time(),MilliSeconds(200));observer.Attach(0,0,phy);
    WifiMacHeader header;header.SetType(WIFI_MAC_DATA);
    auto psdu=Create<WifiPsdu>(Create<Packet>(12),header);
    Simulator::Schedule(MilliSeconds(100),[&] {
      observer.Candidate(0,0,0,{{SU_STA_ID,psdu}},128000,"unclassified_data",0,psdu->GetSize(),false);
    });
    Simulator::Stop(MilliSeconds(200));Simulator::Run();observer.Finalize();
    auto c=observer.GetCounts();
    NS_TEST_EXPECT_MSG_EQ(c.candidates,1,"constructed missing-event observation is retained");
    NS_TEST_EXPECT_MSG_EQ(c.indeterminate,1,"absence of End does not establish OFF or completion");
    NS_TEST_EXPECT_MSG_EQ(c.unknownIntervals,1,"unknown duration is not a measured zero");
    NS_TEST_EXPECT_MSG_EQ(c.completedBytes,0,"no invented completed bytes");
    std::ofstream fixture(folder/"completion-fixture.json");
    fixture<<"{\"real_radio\":false,\"constructed_missing_event\":true,\"expected_candidates\":1}\n";
    fixture.close();Simulator::Destroy();
  }
};

class CafbcrPhyCompletionSuite : public TestSuite
{
public:
  CafbcrPhyCompletionSuite():TestSuite("cafbcr-phy-completion",UNIT) {
    const char* names[]={"completed broadcast matches actual End","synchronous OFF before observer TX notification",
      "synchronous OFF after observer TX notification","mid-frame OFF preserves local interval and unknown bytes",
      "cutoff during TX preserves right censoring","End exactly at cutoff is observed completion",
      "End after cutoff does not retroactively complete frame","early capture cannot certify full window",
      "same packet UID has separate local attempt identities","single-PSDU aggregation completes once after all MPDU ends",
      "ordinary independent completion control","frame crossing lower window boundary clips only its interval"};
    const char* folders[]={"phy-normal","phy-sync-before","phy-sync-after","phy-mid-off","phy-cutoff",
      "phy-end-boundary","phy-end-after","phy-early","phy-reused-uid","phy-aggregate","phy-control","phy-lower-boundary"};
    for (unsigned i=0;i<12;++i) if (i!=10)
      AddTestCase(new NetworkPhyCompletionCase(names[i],folders[i],i),TestCase::QUICK);
    AddTestCase(new NetworkPhyMissingEndCase(),TestCase::QUICK);
    AddTestCase(new NetworkPhyCompletionCase("OFF after cutoff preserves as-of censoring","phy-off-after",13),TestCase::QUICK);
    AddTestCase(new NetworkWifiCase("real MAC retries complete locally despite receive failure","phy-native-retry",true,true),TestCase::QUICK);
  }
} g_cafbcrPhyCompletionSuite;

class CafbcrNetworkObserverSuite : public TestSuite
{
public:
  CafbcrNetworkObserverSuite() : TestSuite("cafbcr-network-observer", UNIT) {
    AddTestCase(new NetworkHopCase("self delivery is zero logical links", "self", NetworkHopCase::SELF), TestCase::QUICK);
    AddTestCase(new NetworkHopCase("direct delivery is one link with arbitrary source TTL", "one", NetworkHopCase::ONE), TestCase::QUICK);
    AddTestCase(new NetworkHopCase("two-link delivery counts forwarding plus first link", "two", NetworkHopCase::TWO), TestCase::QUICK);
    AddTestCase(new NetworkHopCase("three-link delivery retains a nontruncated hop sum", "three", NetworkHopCase::THREE), TestCase::QUICK);
    AddTestCase(new NetworkHopCase("lost packet forwarding never enters delivered hop mean", "loss", NetworkHopCase::LOSS), TestCase::QUICK);
    AddTestCase(new NetworkHopCase("a real duplicated frame is counted only once", "duplicate", NetworkHopCase::DUPLICATE), TestCase::QUICK);
    AddTestCase(new NetworkHopCase("a delivered routing loop counts repeated-node traversals", "loop", NetworkHopCase::LOOP), TestCase::QUICK);
    AddTestCase(new NetworkHopCase("fragment paths remain unavailable after reassembly", "fragment", NetworkHopCase::FRAGMENT), TestCase::QUICK);
    AddTestCase(new NetworkHopCase("an unattached source remains missing", "missing", NetworkHopCase::MISSING), TestCase::QUICK);
    AddTestCase(new NetworkHopCase("empty window has null hop mean", "empty", NetworkHopCase::EMPTY), TestCase::QUICK);
    AddTestCase(new NetworkHopCase("upper-bound attempt is excluded", "edge", NetworkHopCase::EDGE), TestCase::QUICK);
    AddTestCase(new NetworkHopCase("early stop cannot certify window completion", "incomplete", NetworkHopCase::INCOMPLETE), TestCase::QUICK);
    AddTestCase(new NetworkHopCase("unequal flow cohorts keep a packet weighted fractional mean", "mixed", NetworkHopCase::MIXED), TestCase::QUICK);
    AddTestCase(new NetworkWifiCase("actual broadcast headers have independently known bytes and nominal duration", "wifi-broadcast", false), TestCase::QUICK);
    AddTestCase(new NetworkWifiCase("Wi-Fi retries ARP and ACK remain distinct from IP send count", "wifi-retry", true), TestCase::QUICK);
  }
} g_cafbcrNetworkObserverSuite;
} // namespace ns3::cafbcr
