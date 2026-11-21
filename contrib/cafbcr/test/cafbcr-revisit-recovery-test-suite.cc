/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "ns3/cafbcr-routing.h"
#include "../model/cafbcr-revisit-recovery.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/csma-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/test.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/double.h"
#include "ns3/uinteger.h"
#include "../../../scratch/ca-fbcr-recovery-observer.h"
#include <cstdlib>

namespace ns3::cafbcr {
class CafbcrRecoveryStateCase : public TestCase
{
public:
  enum Kind { WIRE, DISTINCT, TTL, EXPIRY, ENTRIES, BYTES, UNTRACKED, NO_COMMIT, LIFECYCLE };
  CafbcrRecoveryStateCase(const char* name, Kind kind) : TestCase(name), m_kind(kind) {}
private:
  void DoRun() override {
    auto history = CreateObject<RevisitRecovery>();
    uint8_t bytes[] = {1, 2, 3, 4}; auto p = Create<Packet>(bytes, 4);
    Ipv4Header h; h.SetSource(Ipv4Address("10.1.1.1")); h.SetDestination(Ipv4Address("10.1.1.8"));
    h.SetProtocol(17); h.SetIdentification(7); h.SetTtl(15);
    Ipv4Address first("10.1.1.3"), second("10.1.1.4");
    auto keep = [&](Ipv4Header header, Ipv4Address next) { history->Remember(history->Inspect(p, header), next); };
    if (m_kind == LIFECYCLE) {
      auto route = CreateObject<RoutingProtocol>();
      NS_TEST_EXPECT_MSG_EQ(route->GetAlternateRepeatedForward(), false, "default allocates no recovery");
      route->SetAlternateRepeatedForward(true); auto saved = route->GetRevisitRecovery();
      saved->Remember(saved->Inspect(p, h), first);
      route->SetTerminateRepeatedForward(true);
      NS_TEST_EXPECT_MSG_EQ(bool(route->GetRevisitRecovery()), false, "termination and alternatives are mutually exclusive");
      NS_TEST_EXPECT_MSG_EQ(saved->GetStats().entries, 0, "switch releases stored bytes");
      route->SetAlternateRepeatedForward(true);
      NS_TEST_EXPECT_MSG_EQ(route->GetTerminateRepeatedForward(), false, "switch back removes termination history");
      route->Dispose(); NS_TEST_EXPECT_MSG_EQ(bool(route->GetRevisitRecovery()), false, "Dispose erases registry ownership");
      route = nullptr;
    } else if (m_kind == ENTRIES || m_kind == BYTES) {
      if (m_kind == ENTRIES) history->m_entryLimit = 2;
      else history->m_payloadLimit = 6;
      for (uint16_t i = 0; i < 3; ++i) { h.SetIdentification(i); keep(h, first); }
      auto s = history->GetStats();
      NS_TEST_EXPECT_MSG_EQ(s.entries, (m_kind == ENTRIES ? 2 : 1), "entries are bounded");
      NS_TEST_EXPECT_MSG_EQ(s.payloadBytes, (m_kind == ENTRIES ? 8 : 4), "payload accounting matches retained entries");
      NS_TEST_EXPECT_MSG_EQ(s.hopBytes, (m_kind == ENTRIES ? 8 : 4), "hop byte accounting follows eviction");
      h.SetIdentification(0); h.SetTtl(13);
      NS_TEST_EXPECT_MSG_EQ(history->Inspect(p, h).revisit, false, "eviction loses the observation explicitly");
    } else if (m_kind == UNTRACKED) {
      auto fragment = h; fragment.SetMoreFragments();
      NS_TEST_EXPECT_MSG_EQ(history->Inspect(p, fragment).track, false, "first fragment is untracked");
      fragment = h; fragment.SetFragmentOffset(8);
      NS_TEST_EXPECT_MSG_EQ(history->Inspect(p, fragment).track, false, "later fragment is untracked");
      h.SetTtl(1); NS_TEST_EXPECT_MSG_EQ(history->Inspect(p, h).track, false, "TTL one belongs to IPv4");
      h.SetTtl(15); history->m_payloadLimit = 2;
      keep(h, first); NS_TEST_EXPECT_MSG_EQ(history->GetStats().entries, 0, "oversized payload is not hidden state");
      NS_TEST_EXPECT_MSG_EQ(history->GetStats().copiedPayloadBytes, 0, "untracked bytes are not copied");
    } else if (m_kind == NO_COMMIT) {
      auto unused = history->Inspect(p, h); h.SetTtl(13);
      NS_TEST_EXPECT_MSG_EQ(history->Inspect(p, h).revisit, false, "no unicast callback means no tried hop");
    } else {
      keep(h, first); h.SetTtl(13);
      if (m_kind == WIRE) {
        auto independent = Create<Packet>(bytes, 4);
        NS_TEST_EXPECT_MSG_NE(independent->GetUid(), p->GetUid(), "independent ns-3 identities");
        auto c = history->Inspect(independent, h);
        NS_TEST_EXPECT_MSG_EQ(c.revisit, true, "wire bytes, not UID, identify a return");
        NS_TEST_EXPECT_MSG_EQ(c.tried.size(), 1, "previous next hop is available");
        NS_TEST_EXPECT_MSG_EQ(c.tried.front(), first, "native next hop is excluded from recovery");
        history->Remember(std::move(c), second); h.SetTtl(11);
        c = history->Inspect(p, h);
        NS_TEST_EXPECT_MSG_EQ(c.tried.size(), 2, "multiple returns retain all attempted distinct hops");
        NS_TEST_EXPECT_MSG_EQ(c.tried[1], second, "alternate next hop is retained");
      } else if (m_kind == DISTINCT) {
        bytes[0] = 9; auto other = Create<Packet>(bytes, 4);
        NS_TEST_EXPECT_MSG_EQ(history->Inspect(other, h).revisit, false, "different payload is not merged");
        for (int field = 0; field < 4; ++field) {
          auto otherHeader = h;
          if (field == 0) otherHeader.SetSource(Ipv4Address("10.1.1.9"));
          if (field == 1) otherHeader.SetDestination(Ipv4Address("10.1.1.9"));
          if (field == 2) otherHeader.SetProtocol(1);
          if (field == 3) otherHeader.SetIdentification(8);
          NS_TEST_EXPECT_MSG_EQ(history->Inspect(p, otherHeader).revisit, false, "header tuple fields are distinct");
        }
      } else if (m_kind == TTL) {
        h.SetTtl(15); auto equal = history->Inspect(p, h);
        NS_TEST_EXPECT_MSG_EQ(equal.revisit, false, "equal TTL duplicate is not a loop proof");
        history->Remember(std::move(equal), first);
        NS_TEST_EXPECT_MSG_EQ(history->GetStats().hopBytes, 4, "duplicate hop is not appended");
        h.SetTtl(16); keep(h, second); h.SetTtl(14);
        auto c = history->Inspect(p, h);
        NS_TEST_EXPECT_MSG_EQ(c.tried.size(), 1, "higher TTL starts fresh identity reuse state");
        NS_TEST_EXPECT_MSG_EQ(c.tried[0], second, "old next hop does not survive reuse");
      } else if (m_kind == EXPIRY) {
        Simulator::Schedule(MilliSeconds(500), [&, h] { keep(h, second); });
        Simulator::Schedule(Seconds(1) - TimeStep(1), [&, h] {
          NS_TEST_EXPECT_MSG_EQ(history->Inspect(p, h).revisit, true, "return does not renew initial lifetime");
        });
        Simulator::Schedule(Seconds(1), [&, h] {
          NS_TEST_EXPECT_MSG_EQ(history->Inspect(p, h).revisit, false, "exact expiry releases history");
          NS_TEST_EXPECT_MSG_EQ(history->GetStats().hopBytes, 0, "expiry releases tried-hop bytes");
        });
        Simulator::Stop(Seconds(1) + TimeStep(1)); Simulator::Run();
      }
    }
    NS_TEST_EXPECT_MSG_EQ(p->GetSize(), 4, "state does not add wire headers");
    history->Dispose(); Simulator::Destroy();
  }
  Kind m_kind;
};

// Fixed advertised graph over native IPv4/ARP/CSMA, not a FANET PHY claim.
// S--A--B is a greedy trap; A--C--E--F--G--D is an available detour.
class CafbcrAlternateNetworkCase : public TestCase
{
public:
  enum Kind { VOID, TASK_VOID, DISCONNECTED, BUDGET, ACYCLIC, LOCAL, FILTERS };
  CafbcrAlternateNetworkCase(const char* name, const char* slug, int policy, Kind kind)
    : TestCase(name), m_slug(slug), m_policy(policy), m_kind(kind) {}
private:
  struct Tx { uint32_t node; uint8_t ttl; uint64_t uid; };
  static void TxBound(CafbcrAlternateNetworkCase* self, uint32_t node, Ptr<const Packet> p, Ptr<Ipv4> ip, uint32_t iface) {
    Ipv4Header h; p->PeekHeader(h);
    if (iface && h.GetProtocol() == 17 && h.GetSource() == self->m_addresses.GetAddress(0) &&
        h.GetDestination() == self->m_addresses.GetAddress(7)) self->m_tx.push_back({node,h.GetTtl(),p->GetUid()});
  }
  void Drop(const Ipv4Header& h, Ptr<const Packet> p, Ipv4L3Protocol::DropReason reason, Ptr<Ipv4> ip, uint32_t iface) {
    if (h.GetProtocol() == 17 && h.GetDestination() == m_addresses.GetAddress(7)) { ++m_drops; m_reason = reason; }
  }
  void Receive(Ptr<Socket> socket) {
    while (auto p = socket->Recv()) {
      ++m_received; NS_TEST_EXPECT_MSG_EQ(p->GetSize(), 100, "no recovery shim on wire");
      uint8_t bytes[100]; p->CopyData(bytes, 100);
      for (uint8_t i = 0; i < 100; ++i) NS_TEST_EXPECT_MSG_EQ(bytes[i], i, "complete delivered payload");
    }
  }
  void LocalReceive(Ptr<const Packet> p, const Ipv4Header& h, uint32_t iface) { ++m_received; }
  void Send() {
    for (auto route : m_routes) {
      route->m_helloTimer.Cancel(); route->m_clusteringTimer.Cancel(); route->m_beaconTimer.Cancel(); route->m_neighbors->Clear();
    }
    auto add = [&](uint32_t i, uint32_t j, bool expired = false) {
      m_routes[i]->m_neighbors->AddNeighbor(Neighbors::NeighborInfo(m_nodes.Get(j)->GetId(), m_addresses.GetAddress(j),
        0xFFFF, m_positions[j], Vector(), 100, .8, Idle,
        expired ? Seconds(-1) : Simulator::Now(), expired ? MilliSeconds(1) : Seconds(10)));
    };
    for (uint32_t i = 0; i < 8; ++i)
      for (uint32_t j = 0; j < 8; ++j)
        if (i != j && CalculateDistance(m_positions[i], m_positions[j]) <= 160) {
          if ((m_kind == DISCONNECTED || m_kind == FILTERS) && ((i == 1 && j == 3) || (i == 3 && j == 1))) continue;
          add(i, j);
        }
    if (m_kind == FILTERS) { add(1, 3, true); add(1, 7); } // expired exit and outside configured range
    if (m_kind == LOCAL) {
      auto p = Create<Packet>(100); Ipv4Header h; h.SetSource(m_addresses.GetAddress(0));
      h.SetDestination(m_addresses.GetAddress(1)); h.SetProtocol(17); h.SetIdentification(1); h.SetTtl(16);
      auto state = m_routes[1]->GetRevisitRecovery(); state->Remember(state->Inspect(p,h), m_addresses.GetAddress(2));
      h.SetTtl(14); auto checks = state->GetStats().checks;
      bool handled = m_routes[1]->RouteInput(p,h,m_nodes.Get(1)->GetDevice(1),
        Ipv4RoutingProtocol::UnicastForwardCallback(),Ipv4RoutingProtocol::MulticastForwardCallback(),
        MakeCallback(&CafbcrAlternateNetworkCase::LocalReceive,this),Ipv4RoutingProtocol::ErrorCallback());
      NS_TEST_EXPECT_MSG_EQ(handled,true,"local delivery is handled");
      NS_TEST_EXPECT_MSG_EQ(state->GetStats().checks,checks,"destination bypasses transit state"); return;
    }
    m_source = Socket::CreateSocket(m_nodes.Get(0),UdpSocketFactory::GetTypeId()); m_source->SetIpTtl(16);
    m_source->Bind(); m_source->Connect(InetSocketAddress(m_addresses.GetAddress(7),9000));
    uint8_t bytes[100]; for (uint8_t i=0;i<100;++i) bytes[i]=i;
    auto packet = Create<Packet>(bytes,100);
    if (m_kind == TASK_VOID) { CafbcrComputeTaskTag tag; packet->AddPacketTag(tag); }
    NS_TEST_EXPECT_MSG_EQ(m_source->Send(packet),100,"one submitted datagram");
  }
  void DoRun() override {
    NS_TEST_ASSERT_MSG_EQ(ClusterManager::GetInstance()->GetAllClusters().empty(),true,"no global cluster path");
    m_positions = {Vector(-100,0,0),Vector(0,0,0),Vector(100,0,0),Vector(0,150,0),
                   Vector(120,230,0),Vector(240,160,0),Vector(330,80,0),Vector(360,0,0)};
    if (m_kind == ACYCLIC) for (uint32_t i=0;i<8;++i) m_positions[i]=Vector(int(i)*100-100,0,0);
    m_nodes.Create(8); InternetStackHelper stack; stack.Install(m_nodes);
    for (uint32_t i=0;i<8;++i) {
      auto n=m_nodes.Get(i); auto cn=CreateObject<ClusterNode>(); cn->SetNodeId(n->GetId());
      cn->SetRole(Idle); cn->SetClusterId(0xFFFF); cn->SetMaxRange(160); cn->SetOriginEnergy(100); cn->updateResidualEnergy(100);
      n->AggregateObject(cn); auto mobility=CreateObject<ConstantPositionMobilityModel>(); mobility->SetPosition(m_positions[i]);
      n->AggregateObject(mobility); auto route=CreateObject<RoutingProtocol>(); route->SetAttribute("MaxRange",DoubleValue(160));
      route->SetTerminateRepeatedForward(m_policy==1); route->SetAlternateRepeatedForward(m_policy==2);
      if (m_kind==BUDGET) route->GetRevisitRecovery()->SetAttribute("NextHopLimit",UintegerValue(1));
      n->AggregateObject(route); n->GetObject<Ipv4>()->SetRoutingProtocol(route); route->AssignStreams(7000000+i*2); m_routes.push_back(route);
    }
    CsmaHelper channel; channel.SetChannelAttribute("DataRate",StringValue("100Mbps"));
    channel.SetChannelAttribute("Delay",TimeValue(MicroSeconds(1))); auto devices=channel.Install(m_nodes);
    Ipv4AddressHelper address; address.SetBase("10.2.1.0","255.255.255.0"); m_addresses=address.Assign(devices);
    const char* output=std::getenv("CAFBCR_RECOVERY_TEST_OUTPUT");
    NS_TEST_ASSERT_MSG_NE(output,nullptr,"runner supplies isolated evidence directory");
    auto directory=std::filesystem::path(output)/m_slug;
    NS_TEST_ASSERT_MSG_EQ(std::filesystem::exists(directory),false,"preserve prior evidence"); std::filesystem::create_directories(directory);
    RecoveryObservationWriter writer(directory.string(),Seconds(.5),m_policy==2);
    for (uint32_t i=0;i<8;++i) {
      writer.Attach(i,m_routes[i]); auto ip=m_nodes.Get(i)->GetObject<Ipv4L3Protocol>();
      ip->TraceConnectWithoutContext("Tx",MakeBoundCallback(&CafbcrAlternateNetworkCase::TxBound,this,i));
      ip->TraceConnectWithoutContext("Drop",MakeCallback(&CafbcrAlternateNetworkCase::Drop,this));
    }
    m_sink=Socket::CreateSocket(m_nodes.Get(7),UdpSocketFactory::GetTypeId());
    m_sink->Bind(InetSocketAddress(Ipv4Address::GetAny(),9000)); m_sink->SetRecvCallback(MakeCallback(&CafbcrAlternateNetworkCase::Receive,this));
    Simulator::Schedule(TimeStep(1),[this] { for (auto r:m_routes) { r->m_helloTimer.Cancel(); r->m_clusteringTimer.Cancel(); r->m_beaconTimer.Cancel(); } });
    Simulator::Schedule(MilliSeconds(100),&CafbcrAlternateNetworkCase::Send,this);
    Simulator::Stop(Seconds(.5)); Simulator::Run(); writer.Finalize();
    const bool reachable=m_policy==2 && (m_kind==VOID || m_kind==TASK_VOID || m_kind==ACYCLIC || m_kind==LOCAL);
    NS_TEST_EXPECT_MSG_EQ(m_received,(reachable?1:0),"delivery matches the fixed graph and policy");
    if (reachable && m_kind!=LOCAL) {
      std::vector<uint32_t> expected = m_kind==ACYCLIC ? std::vector<uint32_t>{0,1,2,3,4,5,6} : std::vector<uint32_t>{0,1,2,1,3,4,5,6};
      NS_TEST_EXPECT_MSG_EQ(m_tx.size(),expected.size(),"physical IP Tx count matches detour");
      for (size_t i=0;i<std::min(m_tx.size(),expected.size());++i) {
        NS_TEST_EXPECT_MSG_EQ(m_tx[i].node,expected[i],"actual route follows expected detour");
        NS_TEST_EXPECT_MSG_EQ(unsigned(m_tx[i].ttl),16-i,"native IPv4 decrements TTL on every edge");
      }
      NS_TEST_EXPECT_MSG_EQ(m_drops,0,"no false terminal on reachable detour");
      if (m_kind!=ACYCLIC) {
        NS_TEST_EXPECT_MSG_EQ(m_routes[1]->m_routeCache.at(m_addresses.GetAddress(7)).nextHop,m_addresses.GetAddress(2),"alternate did not overwrite destination cache");
        NS_TEST_EXPECT_MSG_EQ(m_routes[1]->GetRevisitRecovery()->GetStats().alternateForwards,1,"one return is recovered");
      }
    } else if (!reachable) {
      NS_TEST_EXPECT_MSG_EQ(m_drops,1,"native IPv4 sees one terminal data drop");
      NS_TEST_EXPECT_MSG_EQ(m_reason,(m_policy==0?Ipv4L3Protocol::DROP_TTL_EXPIRED:Ipv4L3Protocol::DROP_NO_ROUTE),"native terminal cause is explicit");
      if (m_policy==0 || m_policy==1 || m_kind==BUDGET)
        NS_TEST_EXPECT_MSG_EQ(m_tx.size(),(m_policy==0?16:3),"off, termination and budget controls differ");
      if (m_kind==DISCONNECTED || m_kind==FILTERS)
        NS_TEST_EXPECT_MSG_EQ(m_routes[1]->GetRevisitRecovery()->GetStats().exhausted,1,"all locally valid untried exits exhausted");
      if (m_kind==BUDGET) NS_TEST_EXPECT_MSG_EQ(m_routes[1]->GetRevisitRecovery()->GetStats().budgetStops,1,"attempt budget stops before trying the available exit");
    }
    std::ofstream f(directory/"fixture.json"); f<<"{\"policy\":"<<m_policy<<",\"kind\":"<<m_kind<<",\"received\":"<<m_received<<",\"drops\":"<<m_drops<<",\"drop_reason\":"<<int(m_reason)<<",\"tx\":[";
    for (size_t i=0;i<m_tx.size();++i) { if(i) f<<','; f<<"{\"node\":"<<m_tx[i].node<<",\"ttl\":"<<unsigned(m_tx[i].ttl)<<",\"uid\":"<<m_tx[i].uid<<'}'; }
    f<<"]}\n"; f.close(); Simulator::Destroy(); m_routes.clear(); m_source=nullptr; m_sink=nullptr;
  }
  const char* m_slug; int m_policy; Kind m_kind;
  NodeContainer m_nodes; Ipv4InterfaceContainer m_addresses;
  std::vector<Vector> m_positions; std::vector<Ptr<RoutingProtocol>> m_routes;
  Ptr<Socket> m_source,m_sink; std::vector<Tx> m_tx;
  uint32_t m_received=0,m_drops=0; Ipv4L3Protocol::DropReason m_reason=Ipv4L3Protocol::DROP_NO_ROUTE;
};

class CafbcrRecoverySuite : public TestSuite
{
public:
  CafbcrRecoverySuite():TestSuite("cafbcr-revisit-recovery",UNIT) {
    using S=CafbcrRecoveryStateCase;
    AddTestCase(new S("wire identity and multiple tried next hops",S::WIRE),TestCase::QUICK);
    AddTestCase(new S("distinct datagrams retain distinct history",S::DISTINCT),TestCase::QUICK);
    AddTestCase(new S("duplicate and higher TTL semantics",S::TTL),TestCase::QUICK);
    AddTestCase(new S("return does not refresh the expiry boundary",S::EXPIRY),TestCase::QUICK);
    AddTestCase(new S("entry budget releases payload and next hops",S::ENTRIES),TestCase::QUICK);
    AddTestCase(new S("payload budget releases next-hop state",S::BYTES),TestCase::QUICK);
    AddTestCase(new S("fragments oversized data and native TTL are untracked",S::UNTRACKED),TestCase::QUICK);
    AddTestCase(new S("uncommitted candidate is not an attempted hop",S::NO_COMMIT),TestCase::QUICK);
    AddTestCase(new S("off switching and disposal isolate state",S::LIFECYCLE),TestCase::QUICK);
    using N=CafbcrAlternateNetworkCase;
    AddTestCase(new N("reachable void loops under original policy","void-off",0,N::VOID),TestCase::QUICK);
    AddTestCase(new N("termination control abandons reachable detour","void-terminate",1,N::VOID),TestCase::QUICK);
    AddTestCase(new N("alternative recovers reachable geographic detour","void-alternate",2,N::VOID),TestCase::QUICK);
    AddTestCase(new N("task shortcut detour also recovers","task-alternate",2,N::TASK_VOID),TestCase::QUICK);
    AddTestCase(new N("disconnected graph exhausts alternatives","disconnected",2,N::DISCONNECTED),TestCase::QUICK);
    AddTestCase(new N("budget can terminate before an available exit","budget",2,N::BUDGET),TestCase::QUICK);
    AddTestCase(new N("acyclic route remains native","acyclic",2,N::ACYCLIC),TestCase::QUICK);
    AddTestCase(new N("local delivery bypasses alternative state","local",2,N::LOCAL),TestCase::QUICK);
    AddTestCase(new N("expired and out-of-range exits are excluded","filters",2,N::FILTERS),TestCase::QUICK);
  }
} g_cafbcrRecoverySuite;
} // namespace ns3::cafbcr
