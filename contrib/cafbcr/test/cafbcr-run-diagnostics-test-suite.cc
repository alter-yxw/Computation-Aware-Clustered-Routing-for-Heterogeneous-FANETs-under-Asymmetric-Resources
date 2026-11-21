/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "../../../scratch/ca-fbcr-run-diagnostics.h"
#include "../../../scratch/ca-fbcr-control-meter.h"
#include "ns3/applications-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/string.h"
#include "ns3/test.h"
#include "ns3/udp-header.h"

namespace ns3::cafbcr {
class CafbcrRunDiagnosticsCase : public TestCase
{
public:
  enum Kind { STRICT_TTL, KEYS, PHASES, REGISTERED, CUTOFF, TTL_DROP, OUTCOMES, SCALARS,
              CONTROL_BASE, CONTROL_TAILS, CONTROL_UNKNOWN_TAIL, CONTROL_FILTERS,
              CONTROL_CUTOFF, CONTROL_CSV, CONTROL_NATIVE };
  CafbcrRunDiagnosticsCase(const char* name, Kind kind) : TestCase(name), m_kind(kind) {}
private:
  Ptr<Packet> Task(uint32_t source=4, uint32_t sequence=1, bool reply=false)
  {
    auto p=Create<Packet>(64); CafbcrComputeTaskTag tag;
    tag.SetSourceNodeId(source); tag.SetTaskId(sequence); tag.SetWorkload(reply ? 0 : 1);
    p->AddPacketTag(tag); return p;
  }
  Ipv4Header Header(uint8_t ttl=64)
  {
    Ipv4Header h; h.SetSource(Ipv4Address("10.1.0.1")); h.SetDestination(Ipv4Address("10.1.0.9"));
    h.SetProtocol(17); h.SetTtl(ttl); return h;
  }
  Ptr<Packet> Control(MessageType type, uint8_t records=0, uint32_t rawTail=0)
  {
    auto p=Create<Packet>(rawTail);
    if (records) {
      DirectMaintenanceHeader tail(500000);
      for (uint8_t i=0;i<records;++i) tail.AddRecord(i,100000);
      p->AddHeader(tail);
    }
    if (type==CAFBCR_CH_LIVE) p->AddHeader(ClusterHeadBeaconHeader());
    p->AddHeader(TypeHeader(type));
    return p;
  }
  Ptr<Packet> Wire(Ptr<Packet> payload, Ipv4Header h, uint16_t port=889)
  {
    UdpHeader u; u.SetSourcePort(port); u.SetDestinationPort(889);
    payload->AddHeader(u);
    h.SetPayloadSize(payload->GetSize()); payload->AddHeader(h); return payload;
  }
  void ControlChecks(RunDiagnostics& d)
  {
    using D=RunDiagnostics;
    NodeContainer nodes; nodes.Create(2); InternetStackHelper internet; internet.Install(nodes);
    PointToPointHelper link; auto devices=link.Install(nodes);
    Ipv4AddressHelper addresses; addresses.SetBase("10.1.0.0","255.255.255.0");
    auto assigned=addresses.Assign(devices);
    auto ip=nodes.Get(0)->GetObject<Ipv4L3Protocol>();
    auto h=Header();
    auto tx=[&](Ptr<const Packet> p,uint32_t interface=1){D::IpTx(&d,0,889,p,ip,interface);};
    if (m_kind==CONTROL_BASE) {
      auto bare=Wire(Control(CAFBCR_CH_LIVE),h);
      auto hello=Wire(Control(CAFBCR_HELLO),h);
      auto cm=Wire(Control(CAFBCR_CM_LIVE),h);
      auto anchor=Create<Packet>(); tx(bare); tx(hello); tx(cm); auto next=Create<Packet>();
      NS_TEST_EXPECT_MSG_EQ(next->GetUid(),anchor->GetUid()+1,"IP byte observation preserves Packet UIDs");
      NS_TEST_EXPECT_MSG_EQ(d.m_control,3,"all originated control types count");
      NS_TEST_EXPECT_MSG_EQ(d.m_controlBytes,175,"IPv4 and UDP headers included exactly once");
      NS_TEST_EXPECT_MSG_EQ(d.m_chLive,1,"CH is a subset of all control");
      NS_TEST_EXPECT_MSG_EQ(d.m_chLiveBytes,117,"bare CH includes type, 88-byte CH, UDP and IP");
      NS_TEST_EXPECT_MSG_EQ(d.m_chLiveTailBytes,0,"grant slots belong to the base, not feedback");
      NS_TEST_EXPECT_MSG_EQ(d.m_hello,1,"existing HELLO count unchanged");
      NS_TEST_EXPECT_MSG_EQ(d.m_cm,1,"existing CM count unchanged");
    } else if (m_kind==CONTROL_TAILS) {
      tx(Wire(Control(CAFBCR_CH_LIVE,1),h)); tx(Wire(Control(CAFBCR_CH_LIVE,8),h));
      NS_TEST_EXPECT_MSG_EQ(d.m_chLive,2,"one and eight records remain two IP sends");
      NS_TEST_EXPECT_MSG_EQ(d.m_chLiveTailBytes,110,"20 and 90 real serialized tail bytes charged");
      NS_TEST_EXPECT_MSG_EQ(d.m_chLiveBytes,344,"tails included in CH IP bytes");
      NS_TEST_EXPECT_MSG_EQ(d.m_controlBytes,344,"subset bytes are not added twice to total");
    } else if (m_kind==CONTROL_UNKNOWN_TAIL) {
      tx(Wire(Control(CAFBCR_CH_LIVE,0,3),h));
      auto truncated=Create<Packet>(); truncated->AddHeader(TypeHeader(CAFBCR_CH_LIVE));
      tx(Wire(truncated,h));
      NS_TEST_EXPECT_MSG_EQ(d.m_chLive,2,"type identifies cost even for a short CH payload");
      NS_TEST_EXPECT_MSG_EQ(d.m_chLiveTailBytes,3,"invalid tails still cost bytes; short frames cannot underflow");
      NS_TEST_EXPECT_MSG_EQ(d.m_chLiveBytes,149,"malformed cost is retained without inventing valid feedback");
    } else if (m_kind==CONTROL_FILTERS) {
      tx(Wire(Control(CAFBCR_CH_LIVE,1),h),0);
      tx(Wire(Control(CAFBCR_CH_LIVE,1),h,9000));
      auto foreign=h; foreign.SetSource(Ipv4Address("10.9.0.1"));
      tx(Wire(Control(CAFBCR_CH_LIVE,1),foreign));
      auto tcp=h; tcp.SetProtocol(6); tx(Wire(Control(CAFBCR_CH_LIVE,1),tcp));
      auto fragment=h; fragment.SetMoreFragments(); tx(Wire(Control(CAFBCR_CH_LIVE,1),fragment));
      tx(Wire(Control(CAFBCR_CH_LIVE,1),h));
      NS_TEST_EXPECT_MSG_EQ(d.m_control,1,"loopback, other ports, forwarding, non-UDP and fragments excluded");
      NS_TEST_EXPECT_MSG_EQ(d.m_controlBytes,137,"only eligible originated bytes charged");
      NS_TEST_EXPECT_MSG_EQ(d.m_fragments,1,"fragment exclusion stays visible");
    } else if (m_kind==CONTROL_CUTOFF) {
      auto p=Wire(Control(CAFBCR_CH_LIVE,1),h);
      Simulator::Schedule(Seconds(31),[&]{tx(p);});
      Simulator::Schedule(Seconds(31)+TimeStep(1),[&]{tx(p);});
      Simulator::Stop(Seconds(31)+TimeStep(2)); Simulator::Run();
      NS_TEST_EXPECT_MSG_EQ(d.m_controlBytes,137,"byte capture includes cutoff and excludes cleanup");
      NS_TEST_EXPECT_MSG_EQ(d.m_chLiveTailBytes,20,"tail capture uses the same cutoff");
    } else if (m_kind==CONTROL_CSV) {
      tx(Wire(Control(CAFBCR_CH_LIVE,1),h));
      d.m_finalized=true;
      auto header=D::CsvHeader(); auto ca=d.CsvValues();
      NS_TEST_EXPECT_MSG_EQ(std::count(header.begin(),header.end(),','),46,"47 diagnostic columns");
      NS_TEST_EXPECT_MSG_EQ(std::count(ca.begin(),ca.end(),','),46,"header and CA values align");
      auto split=[](const std::string& text){std::vector<std::string> fields; std::istringstream s(text);
        for (std::string f;std::getline(s,f,',');) fields.push_back(f); return fields;};
      auto fields=split(ca);
      NS_TEST_EXPECT_MSG_EQ(fields[43],"137","all-control bytes serialized as an integer");
      NS_TEST_EXPECT_MSG_EQ(fields[44],"1","CH transmission count serialized");
      NS_TEST_EXPECT_MSG_EQ(fields[45],"137","CH total bytes serialized");
      NS_TEST_EXPECT_MSG_EQ(fields[46],"20","raw CH tail bytes serialized");
      D other(Seconds(31),false,"N/A","N/A",false,D::Now());
      D::IpTx(&other,0,889,Wire(Control(CAFBCR_CH_LIVE,1),h),ip,1); other.m_finalized=true;
      fields=split(other.CsvValues());
      NS_TEST_EXPECT_MSG_EQ(fields.size(),47,"non-CA width remains aligned");
      NS_TEST_EXPECT_MSG_EQ(fields[43],"137","common IP byte meter applies to non-CA");
      for (auto i : {44,45,46}) NS_TEST_EXPECT_MSG_EQ(fields[i],"N/A","CA-only costs stay N/A for non-CA");
      NS_TEST_EXPECT_MSG_EQ(other.m_chLive,0,"a non-CA first byte is not a CA type");
    } else if (m_kind==CONTROL_NATIVE) {
      auto callback=MakeBoundCallback(&D::IpTx,&d,0u,uint16_t(889));
      NS_TEST_ASSERT_MSG_EQ(ip->TraceConnectWithoutContext("Tx",callback),true,"attach actual IPv4 Tx");
      ControlCostMeter reference(889); reference.Attach(nodes.Get(0));
      auto source=Socket::CreateSocket(nodes.Get(0),UdpSocketFactory::GetTypeId());
      auto sink=Socket::CreateSocket(nodes.Get(1),UdpSocketFactory::GetTypeId());
      NS_TEST_ASSERT_MSG_EQ(source->Bind(InetSocketAddress(assigned.GetAddress(0),889)),0,"bind source control port");
      NS_TEST_ASSERT_MSG_EQ(sink->Bind(InetSocketAddress(Ipv4Address::GetAny(),889)),0,"bind sink control port");
      Simulator::Schedule(Seconds(.1),[&]{for (auto p : {Control(CAFBCR_CH_LIVE),Control(CAFBCR_CH_LIVE,1),
          Control(CAFBCR_CH_LIVE,8),Control(CAFBCR_HELLO)}) source->SendTo(p,0,InetSocketAddress(assigned.GetAddress(1),889));});
      Simulator::Stop(Seconds(.3)); Simulator::Run();
      NS_TEST_EXPECT_MSG_EQ(reference.Packets(),4,"four actual originated IP datagrams");
      NS_TEST_EXPECT_MSG_EQ(d.m_control,reference.Packets(),"packet count reconciles with independent existing meter");
      NS_TEST_EXPECT_MSG_EQ(d.m_controlBytes,reference.Bytes(),"byte count reconciles with independent existing meter");
      NS_TEST_EXPECT_MSG_EQ(d.m_controlBytes,490,"exact native IP bytes, including tails");
      NS_TEST_EXPECT_MSG_EQ(d.m_chLive,3,"native CH subset count");
      NS_TEST_EXPECT_MSG_EQ(d.m_chLiveBytes,461,"native CH bytes exclude the HELLO");
      NS_TEST_EXPECT_MSG_EQ(d.m_chLiveTailBytes,110,"native optional tails cost 20 plus 90 bytes");
      ip->TraceDisconnectWithoutContext("Tx",callback); source->Close(); sink->Close();
    }
  }
  void DoRun() override
  {
    using D=RunDiagnostics;
    D d(Seconds(31),true,"off","eager",true,D::Now());
    auto p=Task(); auto h=Header();
    if (m_kind==STRICT_TTL) {
      d.TrackTx(2,h,p); d.TrackTx(2,h,p);
      h.SetTtl(65); d.TrackTx(2,h,p);
      h.SetTtl(63); d.TrackTx(3,h,p);
      h.SetTtl(62); d.TrackTx(2,h,p->Copy());
      NS_TEST_EXPECT_MSG_EQ(d.GetCounts(D::REQUEST).ipTx,5,"all IP transmissions counted");
      NS_TEST_EXPECT_MSG_EQ(d.GetCounts(D::REQUEST).revisitTx,1,"only lower TTL at the same node proves a repeat");
    } else if (m_kind==KEYS) {
      d.TrackTx(2,h,p); h.SetTtl(63); h.SetDestination(Ipv4Address("10.1.0.8"));
      d.TrackTx(2,h,p); h.SetDestination(Ipv4Address("10.1.0.9"));
      h.SetSource(Ipv4Address("10.1.0.2")); d.TrackTx(2,h,p);
      auto q=Task(); h=Header(63); d.TrackTx(2,h,q);
      NS_TEST_EXPECT_MSG_EQ(d.GetCounts(D::REQUEST).revisitTx,0,"different IP tuple or UID is not a revisit");
    } else if (m_kind==PHASES) {
      d.TrackTx(2,h,p); d.TrackTx(2,h,Task(5)); d.TrackTx(2,h,Task(4,1,true));
      NS_TEST_EXPECT_MSG_EQ(d.GetCounts(D::REQUEST).ipTx,2,"request sources stay distinct");
      NS_TEST_EXPECT_MSG_EQ(d.GetCounts(D::REPLY).ipTx,1,"zero workload denotes completion");
      std::set<D::TaskKey> keys;
      for (const auto& [key,r] : d.m_records) keys.insert(r.task);
      NS_TEST_EXPECT_MSG_EQ(keys.size(),3,"task source, sequence and phase are retained");
    } else if (m_kind==REGISTERED) {
      auto q=Create<Packet>(64); d.TrackTx(2,h,q);
      NS_TEST_EXPECT_MSG_EQ(d.GetCounts(D::UDP).ipTx,0,"unregistered UDP is not business traffic");
      d.m_udpAttempts.insert(q->GetUid()); d.TrackTx(2,h,q);
      h.SetTtl(63); d.TrackTx(2,h,q->Copy());
      NS_TEST_EXPECT_MSG_EQ(d.GetCounts(D::UDP).revisitTx,1,"registered attempt and native copies correlate");
      h.SetMoreFragments(); d.TrackTx(2,h,q);
      NS_TEST_EXPECT_MSG_EQ(d.GetCounts(D::UDP).ipTx,2,"fragments excluded from datagram claims");
    } else if (m_kind==CUTOFF) {
      Simulator::Schedule(Seconds(31),[&]{d.TrackTx(2,h,p);});
      Simulator::Schedule(Seconds(31)+TimeStep(1),[&]{d.TrackTx(3,h,p);});
      Simulator::Stop(Seconds(31)+TimeStep(2)); Simulator::Run();
      NS_TEST_EXPECT_MSG_EQ(d.GetCounts(D::REQUEST).ipTx,1,"capture includes cutoff but excludes later cleanup");
    } else if (m_kind==TTL_DROP) {
      d.TrackTx(2,h,p); h.SetTtl(63); d.TrackTx(2,h,p);
      d.Drop(h,p,Ipv4L3Protocol::DROP_NO_ROUTE,nullptr,0);
      NS_TEST_EXPECT_MSG_EQ(d.m_records.begin()->second.ttl,false,"no route is not TTL expiry");
      d.Drop(h,p,Ipv4L3Protocol::DROP_TTL_EXPIRED,nullptr,0);
      NS_TEST_EXPECT_MSG_EQ(d.m_records.begin()->second.ttl,true,"native TTL reason joins the same datagram");
    } else if (m_kind==OUTCOMES) {
      D::Counts c; D::Outcome(c,{true,true},false); D::Outcome(c,{true,true},true);
      D::Outcome(c,{},false);
      NS_TEST_EXPECT_MSG_EQ(c.missingRevisited,1,"only missing objects enter the missing subset");
      NS_TEST_EXPECT_MSG_EQ(c.missingTtlRevisited,1,"TTL and revisit evidence belong to the same missing object");
      NS_TEST_EXPECT_MSG_EQ(c.deliveredRevisited,1,"successful repeats remain separate");
    } else if (m_kind==SCALARS) {
      auto anchor=Create<Packet>();
      d.TrackTx(2,h,p); d.PhyTx({},WifiTxVector(),1); d.PhyRx(WifiTxVector(),Seconds(.1));
      d.ResponseTimeout(0,nullptr,WifiTxVector());
      auto next=Create<Packet>();
      NS_TEST_EXPECT_MSG_EQ(next->GetUid(),anchor->GetUid()+1,"diagnostics never allocate a Packet UID");
      NS_TEST_EXPECT_MSG_EQ(d.m_ppduTx,1,"one transmit trace is one PPDU attempt");
      NS_TEST_EXPECT_MSG_EQ(d.m_payloadRx,1,"receive payload begins do not imply delivery");
      NS_TEST_EXPECT_MSG_EQ(d.m_timeouts,1,"response timeout is not a final MAC drop");
    } else ControlChecks(d);
    Simulator::Destroy();
  }
  Kind m_kind;
};

class CafbcrRunDiagnosticsNative : public TestCase
{
public:
  CafbcrRunDiagnosticsNative(bool loop) : TestCase(loop ? "native static loop repeats nodes then expires TTL" :
    "native linear path delivers without a revisit"),m_loop(loop) {}
private:
  void DoRun() override
  {
    NodeContainer nodes; nodes.Create(3); InternetStackHelper internet; internet.Install(nodes);
    PointToPointHelper link; link.SetDeviceAttribute("DataRate",StringValue("1Mbps"));
    link.SetChannelAttribute("Delay",TimeValue(MilliSeconds(1)));
    auto a=link.Install(nodes.Get(0),nodes.Get(1)); auto b=link.Install(nodes.Get(1),nodes.Get(2));
    Ipv4AddressHelper address; address.SetBase("10.81.0.0","255.255.255.252"); auto ia=address.Assign(a);
    address.SetBase("10.82.0.0","255.255.255.252"); auto ib=address.Assign(b);
    Ipv4StaticRoutingHelper routing;
    routing.GetStaticRouting(nodes.Get(0)->GetObject<Ipv4>())->SetDefaultRoute(ia.GetAddress(1),1);
    routing.GetStaticRouting(nodes.Get(1)->GetObject<Ipv4>())->SetDefaultRoute(ib.GetAddress(1),2);
    routing.GetStaticRouting(nodes.Get(2)->GetObject<Ipv4>())->SetDefaultRoute(ib.GetAddress(0),1);
    nodes.Get(0)->GetObject<Ipv4>()->SetAttribute("DefaultTtl",UintegerValue(6));
    auto dst=m_loop ? Ipv4Address("10.99.0.1") : ib.GetAddress(1);
    UdpServerHelper server(9000); auto sink=server.Install(nodes.Get(2)); sink.Start(Seconds(0));
    UdpClientHelper client(dst,9000); client.SetAttribute("MaxPackets",UintegerValue(1));
    client.SetAttribute("PacketSize",UintegerValue(64)); client.SetAttribute("Interval",TimeValue(Seconds(.5)));
    auto source=client.Install(nodes.Get(0)); source.Start(Seconds(30.1)); source.Stop(Seconds(30.2));
    TaskObservationWriter tasks("",Seconds(31),"{}",false);
    UdpObservationWriter udp("",Seconds(30),Seconds(31),true,false);
    udp.AddFlow(0,nodes.Get(0)->GetId(),nodes.Get(2)->GetId(),ia.GetAddress(0),dst,9000,
                Seconds(30.1),Seconds(30.2),Seconds(.5),64,1,1);
    udp.Attach(0,DynamicCast<UdpClient>(source.Get(0)),DynamicCast<UdpServer>(sink.Get(0)));
    RunDiagnostics d(Seconds(31),false,"N/A","N/A",false,RunDiagnostics::Now());
    for (uint32_t i=0;i<3;++i) d.AttachNode(nodes.Get(i),889);
    d.AttachUdp(DynamicCast<UdpClient>(source.Get(0)));
    Simulator::Stop(Seconds(31)+TimeStep(2)); d.BeginRun(); Simulator::Run(); d.EndRun();
    tasks.Finalize(); udp.Finalize(); d.Finalize(tasks,udp);
    const auto& c=d.GetCounts(RunDiagnostics::UDP);
    NS_TEST_EXPECT_MSG_EQ(c.ipTx,(m_loop ? 6 : 2),"actual per-node native IP transmissions");
    NS_TEST_EXPECT_MSG_EQ(c.revisitTx,(m_loop ? 3 : 0),"only actual decreasing-TTL node revisits count");
    NS_TEST_EXPECT_MSG_EQ(c.revisited,(m_loop ? 1 : 0),"unique datagrams deduplicated");
    NS_TEST_EXPECT_MSG_EQ(c.missingTtlRevisited,(m_loop ? 1 : 0),"native TTL expiry joined to final missing UDP");
    NS_TEST_EXPECT_MSG_EQ(udp.GetCounts().unique,(m_loop ? 0 : 1),"endpoint delivery defines the outcome");
    NS_TEST_EXPECT_MSG_EQ((d.CsvValues().find(RunDiagnostics::SCHEMA)==0),true,"final CSV snapshot available");
    Simulator::Destroy();
  }
  bool m_loop;
};

class CafbcrRunDiagnosticsSuite : public TestSuite
{
public:
  CafbcrRunDiagnosticsSuite() : TestSuite("cafbcr-run-diagnostics",UNIT)
  {
    using C=CafbcrRunDiagnosticsCase;
    AddTestCase(new C("only a lower TTL at the same node is a revisit",C::STRICT_TTL),TestCase::QUICK);
    AddTestCase(new C("datagram identity includes UID and IP endpoints",C::KEYS),TestCase::QUICK);
    AddTestCase(new C("task source and request versus reply are separate",C::PHASES),TestCase::QUICK);
    AddTestCase(new C("UDP needs a registered native attempt and excludes fragments",C::REGISTERED),TestCase::QUICK);
    AddTestCase(new C("cutoff excludes later cleanup events",C::CUTOFF),TestCase::QUICK);
    AddTestCase(new C("TTL reason is typed and joined to the same datagram",C::TTL_DROP),TestCase::QUICK);
    AddTestCase(new C("delivered and missing repeated objects remain separate",C::OUTCOMES),TestCase::QUICK);
    AddTestCase(new C("PHY counters use scalars and preserve Packet UIDs",C::SCALARS),TestCase::QUICK);
    AddTestCase(new C("IP costs include headers and retain original counters and UIDs",C::CONTROL_BASE),TestCase::QUICK);
    AddTestCase(new C("one and eight serialized feedback records charge their exact tail bytes",C::CONTROL_TAILS),TestCase::QUICK);
    AddTestCase(new C("unknown tails retain cost and short CH payloads cannot underflow",C::CONTROL_UNKNOWN_TAIL),TestCase::QUICK);
    AddTestCase(new C("IP cost excludes loopback forwarding other traffic and fragments",C::CONTROL_FILTERS),TestCase::QUICK);
    AddTestCase(new C("IP byte costs share the capture cutoff",C::CONTROL_CUTOFF),TestCase::QUICK);
    AddTestCase(new C("diagnostic v2 CSV aligns CA values and non-CA NAs",C::CONTROL_CSV),TestCase::QUICK);
    AddTestCase(new C("native UDP control costs reconcile with the independent L3 meter",C::CONTROL_NATIVE),TestCase::QUICK);
    AddTestCase(new CafbcrRunDiagnosticsNative(true),TestCase::QUICK);
    AddTestCase(new CafbcrRunDiagnosticsNative(false),TestCase::QUICK);
  }
} g_cafbcrRunDiagnosticsSuite;
} // namespace ns3::cafbcr
