/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "ns3/cafbcr-routing.h"
#include "ns3/boolean.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/csma-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/node-container.h"
#include "ns3/test.h"
#include "ns3/udp-socket-factory.h"
#include <array>
#include <set>

namespace ns3::cafbcr {

class CafbcrDirectMaintenanceCase : public TestCase
{
public:
  enum Kind {
    WIRE, MALFORMED, VALID, WRONG_ORIGIN, WRONG_HEAD, WRONG_CLUSTER, WRONG_IP,
    FUTURE, NEGATIVE, PRE_ASSOCIATION, EXPIRED, WRONG_ROLE, ORDINARY,
    PROXY_OFF, RENEWAL_OFF, MASTER_OFF, CANDIDATE_OFF, REPLAY, NEWER,
    MIGRATION, PROMOTION, DISPOSE, CH_SHORT_WINDOW, SOURCE_SHORT_WINDOW,
    HEAD_DIRECT, HEAD_PROXY, HEAD_WRONG_CLUSTER, HEAD_OLD_CM, HEAD_MOVED_MEMBER,
    HEAD_CHANGED_OWNER, HEAD_EXPIRED_RECORD, HEAD_REMOVED_MEMBER, ROTATION,
    COMBINED_VALID, COMBINED_NO_ACK, COMBINED_OLDER_ACK, COMBINED_FOREIGN_ACK,
    COMBINED_RENEWAL_OFF, COMBINED_PROXY_OFF, COMBINED_EXPIRES, COMBINED_COORD_OFF,
    COMBINED_STALE_HEAD
  };
  CafbcrDirectMaintenanceCase(const char* name, Kind kind) : TestCase(name), m_kind(kind) {}
private:
  Kind m_kind;
  std::vector<ProxyRenewalEvent> m_events;
  void Observe(const ProxyRenewalEvent& event) { m_events.push_back(event); }
  static DirectMaintenanceHeader Decode(const DirectMaintenanceHeader& sent)
  {
    auto p = Create<Packet>(); p->AddHeader(sent);
    DirectMaintenanceHeader received; p->RemoveHeader(received); return received;
  }
  void Wire()
  {
    DirectMaintenanceHeader sent(500000);
    for (uint16_t i=0; i<8; ++i)
      NS_TEST_EXPECT_MSG_EQ(sent.AddRecord(0x1200+i, INT64_C(0x0102030405060708)+i), true, "bounded records");
    NS_TEST_EXPECT_MSG_EQ(sent.AddRecord(99, 1), false, "ninth record cannot enter the wire");
    ClusterHeadBeaconHeader head(17,42,Ipv4Address("10.53.1.2"),Vector(-12.25,0,0),Vector(),.8,.5);
    for (uint8_t i=0; i<8; ++i) head.SetGrantedMember(i,30+i);
    auto p=Create<Packet>(); p->AddHeader(sent); p->AddHeader(head); p->AddHeader(TypeHeader(CAFBCR_CH_LIVE));
    NS_TEST_EXPECT_MSG_EQ(p->GetSize(),179,"one type byte, original 88-byte header, 90-byte maximum tail");
    TypeHeader type; ClusterHeadBeaconHeader receivedHead; DirectMaintenanceHeader received;
    p->RemoveHeader(type); p->RemoveHeader(receivedHead); p->RemoveHeader(received);
    NS_TEST_EXPECT_MSG_EQ(receivedHead.GetPosition().x,-12.25,"existing signed fields stay intact");
    for (uint8_t i=0; i<8; ++i) {
      NS_TEST_EXPECT_MSG_EQ(receivedHead.GetGrantedMember(i),30+i,"all original grant slots remain independent");
      NS_TEST_EXPECT_MSG_EQ(received.GetRecords()[i].origin,0x1200+i,"member identity is serialized");
      NS_TEST_EXPECT_MSG_EQ(received.GetRecords()[i].sourceUs,INT64_C(0x0102030405060708)+i,"exact integer source time");
    }
    NS_TEST_EXPECT_MSG_EQ(received.IsValid(),true,"extension decoded");
    NS_TEST_EXPECT_MSG_EQ(received.GetWindowUs(),500000,"CH half-lease window is real control data");
    NS_TEST_EXPECT_MSG_EQ(p->GetSize(),0,"exact consumption");
    auto legacy=Create<Packet>(); legacy->AddHeader(head); legacy->AddHeader(TypeHeader(CAFBCR_CH_LIVE));
    NS_TEST_EXPECT_MSG_EQ(legacy->GetSize(),89,"no optional tail is needed for the default policy");
  }
  void Malformed()
  {
    DirectMaintenanceHeader sent(500000); sent.AddRecord(0,400000);
    auto packet=Create<Packet>(); packet->AddHeader(sent);
    std::vector<uint8_t> valid(packet->GetSize()); packet->CopyData(valid.data(),valid.size());
    std::vector<std::vector<uint8_t>> corrupt;
    for (size_t length : {size_t(0),size_t(1),size_t(9),size_t(19)})
      corrupt.emplace_back(valid.begin(),valid.begin()+length);
    auto bad=valid; bad[0]=0; corrupt.push_back(bad);
    bad=valid; bad[1]=0; corrupt.push_back(bad);
    bad=valid; bad[1]=9; corrupt.push_back(bad);
    bad=valid; bad.push_back(0); corrupt.push_back(bad);
    bad=valid; std::fill(bad.begin()+2,bad.begin()+10,0); corrupt.push_back(bad);
    for (const auto& bytes : corrupt) {
      auto p=Create<Packet>(bytes.data(),bytes.size()); DirectMaintenanceHeader received;
      NS_TEST_EXPECT_MSG_EQ(p->RemoveHeader(received),0,"malformed extension is rejected without reading past it");
      NS_TEST_EXPECT_MSG_EQ(received.IsValid(),false,"invalid bytes cannot grant coverage");
    }
  }
  void Source()
  {
    auto manager=ClusterManager::GetInstance();
    NS_TEST_ASSERT_MSG_EQ(manager->GetAllClusters().empty(),true,"source has no global membership facts");
    NodeContainer nodes; nodes.Create(3); InternetStackHelper stack; stack.Install(nodes);
    auto state=CreateObject<ClusterNode>(); state->SetNodeId(0); state->SetRole(Member); state->SetClusterId(42);
    nodes.Get(0)->AggregateObject(state); nodes.Get(0)->AggregateObject(CreateObject<ConstantPositionMobilityModel>());
    auto route=CreateObject<RoutingProtocol>(); route->SetIpv4(nodes.Get(0)->GetObject<Ipv4>());
    route->SetBeaconInterval(Seconds(m_kind==SOURCE_SHORT_WINDOW?.02:.1));
    route->SetAttribute("DirectMaintenanceFeedback",BooleanValue(m_kind!=CANDIDATE_OFF));
    route->m_proxyMembership=std::make_pair(1,42); route->m_proxyMembershipSince=Seconds(.1);
    route->TraceConnectWithoutContext("ProxyRenewal",MakeCallback(&CafbcrDirectMaintenanceCase::Observe,this));
    const bool accepted = m_kind==VALID || m_kind==REPLAY || m_kind==NEWER || m_kind==MIGRATION ||
      m_kind==PROMOTION || m_kind==DISPOSE || m_kind==CH_SHORT_WINDOW || m_kind==SOURCE_SHORT_WINDOW;
    const auto address=Ipv4Address("10.53.1.2");
    auto receive=[&](int64_t stamp) {
      DirectMaintenanceHeader feedback(m_kind==CH_SHORT_WINDOW?100000:500000);
      feedback.AddRecord(m_kind==WRONG_ORIGIN?2:0,stamp);
      ClusterHeadBeaconHeader head(m_kind==WRONG_HEAD?2:1,m_kind==WRONG_CLUSTER?43:42,
        address,Vector(),Vector(),.8,Simulator::Now().GetSeconds());
      route->ApplyDirectMaintenance(head,Decode(feedback),m_kind==WRONG_IP?Ipv4Address("10.53.1.9"):address,77);
    };
    Simulator::Schedule(Seconds(.5),[&] {
      if (m_kind==WRONG_ROLE) state->SetRole(Head);
      if (m_kind==ORDINARY) route->m_proxyMembership.reset();
      if (m_kind==PROXY_OFF) route->SetAttribute("EnableProxyJoin",BooleanValue(false));
      if (m_kind==RENEWAL_OFF) route->SetAttribute("EnableProxyRenewal",BooleanValue(false));
      if (m_kind==MASTER_OFF) route->SetAttribute("EnableAsymHandling",BooleanValue(false));
      if (m_kind==EXPIRED) route->m_proxyMembershipSince=Seconds(0);
      const int64_t stamp=m_kind==FUTURE?600000:m_kind==NEGATIVE?-1:m_kind==PRE_ASSOCIATION?50000:
        m_kind==EXPIRED?0:(m_kind==CH_SHORT_WINDOW || m_kind==SOURCE_SHORT_WINDOW)?450000:400000;
      receive(stamp);
      NS_TEST_EXPECT_MSG_EQ(route->HasDirectMaintenanceCoverage(),accepted,"only current real feedback suppresses proxy requests");
      NS_TEST_EXPECT_MSG_EQ(manager->GetAllClusters().empty(),true,"source never consults CH membership state");
    });
    Simulator::Schedule(Seconds(.7),[&] {
      if (m_kind==REPLAY) { receive(400000); receive(350000); }
      if (m_kind==NEWER) receive(650000);
      if (m_kind==MIGRATION) { state->SetClusterId(43); route->m_proxyMembership=std::make_pair(2,43); }
      if (m_kind==PROMOTION) state->SetRole(Head);
      if (m_kind==DISPOSE) route->Dispose();
    });
    Simulator::Schedule(Seconds(.95),[&] {
      if (m_kind==REPLAY) {
        receive(400000);
        NS_TEST_EXPECT_MSG_EQ(route->HasDirectMaintenanceCoverage(),false,"late replay cannot restart an expired suppression");
      }
      if (m_kind==NEWER)
        NS_TEST_EXPECT_MSG_EQ(route->HasDirectMaintenanceCoverage(),true,"newer received CM coverage replaces the old timer");
    });
    Simulator::Stop(Seconds(1.2)); Simulator::Run();
    std::vector<ProxyRenewalEvent> resumed;
    for (const auto& e:m_events) if (e.event=="source_proxy_resumed_no_feedback") resumed.push_back(e);
    const bool resume=accepted && m_kind!=MIGRATION && m_kind!=PROMOTION && m_kind!=DISPOSE;
    NS_TEST_EXPECT_MSG_EQ(resumed.size(),resume?1:0,"one bounded fallback, no stale-role or stale-association request");
    if (resume) {
      const auto expected=Seconds(m_kind==NEWER?1.15:
        (m_kind==CH_SHORT_WINDOW || m_kind==SOURCE_SHORT_WINDOW)?.55:.9);
      NS_TEST_EXPECT_MSG_EQ(resumed.front().time,expected,"expiry is anchored to CM source time, not feedback arrival");
    }
    NS_TEST_EXPECT_MSG_EQ(route->m_directMaintenanceCoverage.has_value(),false,"coverage does not survive its bounded validity");
    route->Dispose(); Simulator::Destroy(); m_events.clear();
  }
  void RunHead()
  {
    auto manager=ClusterManager::GetInstance();
    NS_TEST_ASSERT_MSG_EQ(manager->GetAllClusters().empty(),true,"clean registry");
    NodeContainer nodes; nodes.Create(m_kind==ROTATION?10:3); InternetStackHelper stack; stack.Install(nodes);
    const uint16_t headId=nodes.GetN()-1;
    std::vector<Ptr<ClusterNode>> states;
    for (uint32_t i=0;i<nodes.GetN();++i) {
      auto state=CreateObject<ClusterNode>(); state->SetNodeId(i); state->SetRole(i==headId?Head:Member);
      state->SetClusterId(42); nodes.Get(i)->AggregateObject(state);
      nodes.Get(i)->AggregateObject(CreateObject<ConstantPositionMobilityModel>()); states.push_back(state);
    }
    manager->CreateCluster(42,states[headId]);
    for (uint16_t i=0;i<headId;++i) { manager->JoinCluster(42,states[i]); states[i]->SetRole(Member); }
    auto cluster=manager->GetCluster(42);
    auto route=CreateObject<RoutingProtocol>(); route->SetIpv4(nodes.Get(headId)->GetObject<Ipv4>());
    route->SetBeaconInterval(Seconds(.1)); route->SetAttribute("DirectMaintenanceFeedback",BooleanValue(true));
    route->m_recentGranted.push_back(0);
    Simulator::Schedule(Seconds(.5),[&] {
      for (uint16_t i=0;i<(m_kind==ROTATION?headId:1);++i) {
        if (m_kind==HEAD_MOVED_MEMBER) states[i]->SetClusterId(43);
        const auto ip=Ipv4Address(0x0a350101+i);
        ClusterMemberBeaconHeader cm(i,m_kind==HEAD_WRONG_CLUSTER?43:42,ip,Vector(),Vector(),.8,
          m_kind==HEAD_OLD_CM?0:.4);
        auto p=Create<Packet>(); p->AddHeader(cm);
        route->RecvClusterMemberBeacon(p,Ipv4Address("10.53.1.99"),m_kind==HEAD_PROXY?Ipv4Address("10.53.1.98"):ip);
      }
      if (m_kind==HEAD_CHANGED_OWNER) states[headId]->SetRole(Member);
      if (m_kind==HEAD_REMOVED_MEMBER) cluster->RemoveMemberRecord(0);
      auto feedback=route->SelectDirectMaintenanceRecords();
      const bool evidence=m_kind==HEAD_DIRECT || m_kind==HEAD_EXPIRED_RECORD || m_kind==ROTATION;
      NS_TEST_EXPECT_MSG_EQ(feedback.GetRecords().size(),evidence?(m_kind==ROTATION?8:1):0,
        "only a received direct CM for a current owned member can be advertised");
      if (m_kind==ROTATION) {
        std::set<uint16_t> represented;
        for (const auto& r:feedback.GetRecords()) represented.insert(r.origin);
        const auto next=route->SelectDirectMaintenanceRecords();
        for (const auto& r:next.GetRecords()) represented.insert(r.origin);
        NS_TEST_EXPECT_MSG_EQ(represented.size(),9,"rotation reaches members beyond eight without extra frames");
      }
    });
    if (m_kind==HEAD_EXPIRED_RECORD) Simulator::Schedule(Seconds(.91),[&] {
      NS_TEST_EXPECT_MSG_EQ(route->SelectDirectMaintenanceRecords().GetRecords().empty(),true,
        "a CH cannot renew its evidence by rebroadcasting the same old observation");
    });
    Simulator::Stop(Seconds(1)); Simulator::Run();
    while (!manager->GetAllClusters().empty()) manager->RemoveCluster(manager->GetAllClusters().begin()->first);
    route->Dispose(); Simulator::Destroy();
  }
  void Combined()
  {
    // Independent source/relay local state, no CH registry. ACKs below are
    // serialized fixture inputs; native N12 checks cover the radio integration.
    auto manager=ClusterManager::GetInstance();
    NS_TEST_ASSERT_MSG_EQ(manager->GetAllClusters().empty(),true,"no remote membership facts");
    NodeContainer nodes; nodes.Create(4); InternetStackHelper stack; stack.Install(nodes);
    std::vector<Ptr<ClusterNode>> states;
    for (uint16_t i=0;i<4;++i) {
      auto state=CreateObject<ClusterNode>(); state->SetNodeId(i);
      state->SetRole(i==1?Head:Member); state->SetClusterId(42);
      nodes.Get(i)->AggregateObject(state);
      nodes.Get(i)->AggregateObject(CreateObject<ConstantPositionMobilityModel>());
      states.push_back(state);
    }
    auto source=CreateObject<RoutingProtocol>(); source->SetIpv4(nodes.Get(0)->GetObject<Ipv4>());
    source->SetBeaconInterval(Seconds(.1));
    source->SetAttribute("DirectMaintenanceFeedback",BooleanValue(true));
    source->SetAttribute("CoordinateProxyRenewal",BooleanValue(true));
    source->m_proxyMembership=std::make_pair(1,42); source->m_proxyMembershipSince=Seconds(.1);
    source->TraceConnectWithoutContext("ProxyRenewal",MakeCallback(&CafbcrDirectMaintenanceCase::Observe,this));
    std::vector<Ptr<RoutingProtocol>> relays;
    for (uint16_t i:{2,3}) {
      auto route=CreateObject<RoutingProtocol>(); route->SetIpv4(nodes.Get(i)->GetObject<Ipv4>());
      route->SetBeaconInterval(Seconds(.1)); route->SetMaxRange(100);
      route->SetAttribute("DirectMaintenanceFeedback",BooleanValue(true));
      route->SetAttribute("CoordinateProxyRenewal",BooleanValue(m_kind!=COMBINED_COORD_OFF));
      route->TraceConnectWithoutContext("ProxyRenewal",MakeCallback(&CafbcrDirectMaintenanceCase::Observe,this));
      relays.push_back(route);
    }
    CsmaHelper csma; auto devices=csma.Install(nodes);
    Ipv4AddressHelper addresses; addresses.SetBase("10.57.1.0","255.255.255.0");
    auto ips=addresses.Assign(devices);
    Simulator::Schedule(Seconds(.2),[&] {
      for (uint16_t i:{0,2,3}) {
        auto route=i==0?source:relays[i-2]; route->m_mainAddress=ips.GetAddress(i);
        auto socket=Socket::CreateSocket(nodes.Get(i),UdpSocketFactory::GetTypeId());
        socket->SetAllowBroadcast(true);
        NS_TEST_ASSERT_MSG_EQ(socket->Bind(InetSocketAddress(ips.GetAddress(i),0)),0,"real local UDP socket");
        route->m_socketAddrs[socket]=Ipv4InterfaceAddress(ips.GetAddress(i),Ipv4Mask("255.255.255.0"));
      }
    });
    Simulator::Schedule(Seconds(.5),[&] {
      DirectMaintenanceHeader coverage(500000); coverage.AddRecord(0,400000);
      ClusterHeadBeaconHeader head(1,42,ips.GetAddress(1),Vector(10,0,0),Vector(),.8,.5);
      source->ApplyDirectMaintenance(head,Decode(coverage),ips.GetAddress(1),77);
      NS_TEST_EXPECT_MSG_EQ(source->HasDirectMaintenanceCoverage(),true,"relay coordination does not disable source direct coverage");
      double stamp=m_kind==COMBINED_OLDER_ACK?.45:.4;
      if (m_kind==COMBINED_STALE_HEAD) {
        while (std::any_of(relays.begin(),relays.end(),[&](auto route) {
          return route->RenewalHoldoff({0,1,42,int64_t(stamp*1e6)})<MilliSeconds(2);
        })) stamp+=.000001;
      }
      for (auto route:relays) {
        route->m_neighbors->AddNeighbor(Neighbors::NeighborInfo(1,ips.GetAddress(1),42,Vector(10,0,0),
          Vector(),100,.8,Head,Simulator::Now(),m_kind==COMBINED_STALE_HEAD?MilliSeconds(1):Seconds(10)));
        HelloHeader hello(0,42,ips.GetAddress(0),Vector(),Vector(),100,.8,Member,stamp);
        hello.SetIntentJoinChId(1); route->RelayProxyMemberHello(hello,ips.GetAddress(0),123);
        NS_TEST_EXPECT_MSG_EQ(route->m_pendingRenewals.size(),(m_kind==COMBINED_COORD_OFF?0:1),"requests queue only under explicit relay coordination");
        if (m_kind==COMBINED_RENEWAL_OFF) route->SetAttribute("EnableProxyRenewal",BooleanValue(false));
        if (m_kind==COMBINED_PROXY_OFF) route->SetAttribute("EnableProxyJoin",BooleanValue(false));
        if (m_kind==COMBINED_VALID || m_kind==COMBINED_OLDER_ACK || m_kind==COMBINED_FOREIGN_ACK ||
            m_kind==COMBINED_RENEWAL_OFF || m_kind==COMBINED_PROXY_OFF || m_kind==COMBINED_COORD_OFF) {
          auto packet=Create<Packet>(); packet->AddHeader(RenewalAckHeader(0,1,42,400000));
          route->RecvRenewalAck(packet,m_kind==COMBINED_FOREIGN_ACK?Ipv4Address("10.57.1.99"):ips.GetAddress(1));
        }
        NS_TEST_EXPECT_MSG_EQ(route->m_pendingRenewals.empty(),(m_kind==COMBINED_VALID || m_kind==COMBINED_COORD_OFF),
          "only matching received CH coverage cancels this observation");
      }
      NS_TEST_EXPECT_MSG_EQ(source->HasDirectMaintenanceCoverage(),true,"relay feedback cannot extend or clear independent direct coverage");
      NS_TEST_EXPECT_MSG_EQ(manager->GetAllClusters().empty(),true,"no remote registry used to suppress sends");
    });
    Simulator::Stop(Seconds(m_kind==COMBINED_EXPIRES?1.05:.8)); Simulator::Run();
    const bool noTransmit=m_kind==COMBINED_VALID || m_kind==COMBINED_RENEWAL_OFF ||
      m_kind==COMBINED_PROXY_OFF || m_kind==COMBINED_STALE_HEAD;
    for (auto route:relays) {
      NS_TEST_EXPECT_MSG_EQ(route->GetCmBeaconCount(),(noTransmit?0:1),"lost or invalid ACK retains a real relay attempt; permission and freshness guards survive");
      NS_TEST_EXPECT_MSG_EQ(route->m_pendingRenewals.empty(),true,"bounded callbacks finish or cancel");
      route->Dispose();
    }
    auto resumed=std::count_if(m_events.begin(),m_events.end(),[](const auto& e) {
      return e.event=="source_proxy_resumed_no_feedback";
    });
    NS_TEST_EXPECT_MSG_EQ(resumed,(m_kind==COMBINED_EXPIRES?1:0),"source coverage still expires and resumes requests independently");
    NS_TEST_EXPECT_MSG_EQ(manager->GetAllClusters().empty(),true,"combination does not create membership or leases");
    source->Dispose(); Simulator::Destroy(); m_events.clear();
  }
  void DoRun() override
  {
    if (m_kind==WIRE) { Wire(); return; }
    if (m_kind==MALFORMED) { Malformed(); return; }
    if (m_kind>=COMBINED_VALID) Combined();
    else if (m_kind>=HEAD_DIRECT) RunHead(); else Source();
  }
};

class CafbcrDirectMaintenanceSuite : public TestSuite
{
public:
  CafbcrDirectMaintenanceSuite() : TestSuite("cafbcr-direct-maintenance",Type::UNIT)
  {
    const char* names[]={"bounded CH_LIVE tail and independent grant bytes","malformed feedback tails",
      "current direct coverage and timed fallback","foreign member feedback","foreign head feedback",
      "foreign cluster feedback","foreign source address","future CM time","negative CM time",
      "CM predates current association","coverage already expired","head cannot consume member feedback",
      "ordinary association does not opt in","proxy disabled","renewal disabled","asymmetry disabled",
      "candidate disabled","replay cannot move deadline","newer coverage cancels prior timer",
      "migration cancels old fallback","promotion cancels old fallback","dispose cancels fallback",
      "shorter CH window","shorter source window","actual direct CM evidence","proxy CM is not direct evidence",
      "wrong-cluster CM is not direct evidence","old CM is not fresh evidence","migrated member excluded",
      "ownership change retires evidence","CH record ages from CM time","removed member excluded",
      "bounded fair rotation beyond eight members",
      "combined source coverage and matching relay ACK", "combined lost ACK retains both relays",
      "combined older ACK preserves newer observations", "combined foreign ACK cannot suppress",
      "combined renewal disabled during holdoff", "combined proxy disabled during holdoff",
      "combined direct coverage expires independently", "direct feedback without relay coordination stays eager",
      "combined relay rechecks expired head view"};
    for (unsigned i=0;i<std::size(names);++i)
      AddTestCase(new CafbcrDirectMaintenanceCase(names[i],static_cast<CafbcrDirectMaintenanceCase::Kind>(i)),TestCase::QUICK);
  }
} g_cafbcrDirectMaintenanceSuite;
} // namespace ns3::cafbcr
