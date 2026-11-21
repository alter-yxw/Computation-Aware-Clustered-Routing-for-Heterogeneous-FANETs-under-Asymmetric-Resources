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
#include "ns3/ipv4-l3-protocol.h"
#include "ns3/test.h"
#include "ns3/udp-header.h"
#include "ns3/udp-socket-factory.h"

namespace ns3::cafbcr {

// Actual CA RouteOutput/RouteInput and IPv4/UDP, without election. CSMA makes
// reception deterministic: these cases prove routing/TTL semantics, not PHY reach.
class ControlPathTestRouting : public RoutingProtocol
{
public:
  static TypeId GetTypeId()
  {
    static TypeId tid = TypeId("ns3::CafbcrControlPathTestRouting")
      .SetParent<RoutingProtocol>().AddConstructor<ControlPathTestRouting>();
    return tid;
  }
  Ipv4Address watch, gateway;
  Ptr<Ipv4Route> RouteOutput(Ptr<Packet> p, const Ipv4Header& h,
                           Ptr<NetDevice> dev, Socket::SocketErrno& error) override
  {
    auto route = RoutingProtocol::RouteOutput(p,h,dev,error);
    if (route && h.GetDestination() == watch) gateway = route->GetGateway();
    return route;
  }
protected:
  void DoInitialize() override { Ipv4RoutingProtocol::DoInitialize(); }
};

class CafbcrControlUnicastCase : public TestCase
{
public:
  CafbcrControlUnicastCase(const char* name, bool fixed, bool business = false,
                          double distance = 450, bool proxy = true)
    : TestCase(name), m_fixed(fixed), m_business(business), m_distance(distance), m_proxy(proxy) {}
private:
  bool m_fixed, m_business;
  double m_distance;
  bool m_proxy;
  uint32_t m_cmReceived = 0, m_ttlDrops = 0, m_controlTx = 0;
  void Join(const JoinMechanismEvent& e)
  {
    if (e.event == "cm_live_rx" && e.originNodeId == 0) ++m_cmReceived;
  }
  bool IsCm(Ptr<const Packet> p)
  {
    if (p->GetSize() < 9) return false;
    auto copy=p->Copy(); UdpHeader udp; copy->RemoveHeader(udp);
    if (udp.GetDestinationPort()!=RoutingProtocol::CAFBCR_PORT) return false;
    TypeHeader type;copy->RemoveHeader(type);return type.IsValid() && type.Get()==CAFBCR_CM_LIVE;
  }
  void Drop(const Ipv4Header& h, Ptr<const Packet> p, Ipv4L3Protocol::DropReason why,
            Ptr<Ipv4>, uint32_t)
  {
    if (h.GetProtocol()==17 && why==Ipv4L3Protocol::DROP_TTL_EXPIRED && IsCm(p)) ++m_ttlDrops;
  }
  void Tx(const Ipv4Header& h, Ptr<const Packet> p, uint32_t)
  {
    if (h.GetProtocol()==17 && IsCm(p)) {
      ++m_controlTx;
      NS_TEST_EXPECT_MSG_EQ(h.GetTtl(),1,"control TTL must stay one; no hidden extra-hop repair");
    }
  }
  void DoRun() override
  {
    auto manager=ClusterManager::GetInstance();
    NS_TEST_ASSERT_MSG_EQ(manager->GetAllClusters().empty(),true,"fresh cluster registry");
    NodeContainer nodes;nodes.Create(3);InternetStackHelper internet;internet.Install(nodes);
    std::vector<Ptr<ClusterNode>> states;std::vector<Ptr<ControlPathTestRouting>> routes;
    for (uint16_t i=0;i<3;++i) {
      auto state=CreateObject<ClusterNode>();state->SetNodeId(i);state->SetRole(i==1?Head:i==0?Member:Idle);
      state->SetClusterId(i<2?42:0xffff);state->SetOriginEnergy(1000);state->updateResidualEnergy(1000);
      nodes.Get(i)->AggregateObject(state);states.push_back(state);
      auto mobility=CreateObject<ConstantPositionMobilityModel>();
      mobility->SetPosition(Vector(i==0?0:i==1?m_distance:m_distance/2,0,0));nodes.Get(i)->AggregateObject(mobility);
      auto route=CreateObject<ControlPathTestRouting>();route->SetMaxRange(i==0?250:1000);
      route->SetAttribute("EnableProxyJoin",BooleanValue(m_proxy));
      if (m_fixed)
        NS_TEST_ASSERT_MSG_EQ(route->SetAttributeFailSafe("OneHopControlUnicast",BooleanValue(true)),true,"candidate installed");
      nodes.Get(i)->AggregateObject(route);
      nodes.Get(i)->GetObject<Ipv4>()->SetRoutingProtocol(route);routes.push_back(route);
      route->TraceConnectWithoutContext("JoinMechanism",MakeCallback(&CafbcrControlUnicastCase::Join,this));
    }
    CsmaHelper csma;auto devices=csma.Install(nodes);Ipv4AddressHelper address;
    address.SetBase("10.55.1.0","255.255.255.0");auto interfaces=address.Assign(devices);
    manager->CreateCluster(42,states[1]);manager->JoinCluster(42,states[0]);states[0]->SetRole(Member);
    nodes.Get(2)->GetObject<Ipv4L3Protocol>()->TraceConnectWithoutContext("Drop",MakeCallback(&CafbcrControlUnicastCase::Drop,this));
    nodes.Get(0)->GetObject<Ipv4L3Protocol>()->TraceConnectWithoutContext("SendOutgoing",MakeCallback(&CafbcrControlUnicastCase::Tx,this));
    // The normal initial HELLOs run; deterministic advertisements below are
    // fixture state, not evidence of wireless discovery or proxy admission.
    Simulator::Schedule(Seconds(.2),[&] {
      for (uint16_t i=0;i<3;++i) {
        states[i]->SetRole(i==1?Head:i==0?Member:Idle);states[i]->SetClusterId(i<2?42:0xffff);
        auto neighbors=states[i]->GetObject<Neighbors>();neighbors->Clear();
        for (uint16_t j=0;j<3;++j) if (i!=j) {
          Neighbors::NeighborInfo info(j,interfaces.GetAddress(j),j<2?42:0xffff,
            nodes.Get(j)->GetObject<MobilityModel>()->GetPosition(),Vector(),1000,.5,
            j==1?Head:j==0?Member:Idle,Simulator::Now(),Seconds(10));
          neighbors->AddNeighbor(info);
        }
      }
      routes[0]->watch=interfaces.GetAddress(1);
    });
    Ptr<Socket> businessSocket;
    Simulator::Schedule(Seconds(.25),[&] {
      if (m_business) {
        // Even a user datagram sent TO the CA port must not acquire the
        // protocol's private SendControlPacket context.
        businessSocket=Socket::CreateSocket(nodes.Get(0),UdpSocketFactory::GetTypeId());
        businessSocket->Bind();businessSocket->SetIpTtl(64);
        businessSocket->SendTo(Create<Packet>(16),0,InetSocketAddress(interfaces.GetAddress(1),RoutingProtocol::CAFBCR_PORT));
      } else routes[0]->SendClusterMemberBeacon();
    });
    Simulator::Stop(Seconds(.45));Simulator::Run();
    const bool direct=m_distance<=250 || (m_fixed && !m_business);
    NS_TEST_EXPECT_MSG_EQ(routes[0]->gateway,interfaces.GetAddress(direct?1:2),"selected actual gateway");
    if (!m_business) {
      NS_TEST_EXPECT_MSG_EQ(m_controlTx,1,"one real source CM IP transmission");
      NS_TEST_EXPECT_MSG_EQ(m_cmReceived,direct?1:0,"CH must actually receive the one-hop CM");
      NS_TEST_EXPECT_MSG_EQ(m_ttlDrops,direct?0:1,"legacy sends to a nonfinal gateway and expires there");
    }
    manager->RemoveCluster(42);Simulator::Destroy();
  }
};

class CafbcrControlUnicastLegacySuite : public TestSuite
{
public:
  CafbcrControlUnicastLegacySuite():TestSuite("cafbcr-control-unicast-legacy",UNIT)
  {
    AddTestCase(new CafbcrControlUnicastCase("legacy nonfinal CM gateway really expires at TTL one",false),TestCase::QUICK);
  }
} static g_controlUnicastLegacySuite;

class CafbcrControlUnicastSuite : public TestSuite
{
public:
  CafbcrControlUnicastSuite():TestSuite("cafbcr-control-unicast",UNIT)
  {
    AddTestCase(new CafbcrControlUnicastCase("one-hop CM reaches actual head without raising TTL",true),TestCase::QUICK);
    AddTestCase(new CafbcrControlUnicastCase("ordinary reachable head remains a direct destination",true,false,200),TestCase::QUICK);
    AddTestCase(new CafbcrControlUnicastCase("same fix is available when proxy is disabled",true,false,450,false),TestCase::QUICK);
    AddTestCase(new CafbcrControlUnicastCase("business TO control port still uses normal data routing",true,true),TestCase::QUICK);
    AddTestCase(new CafbcrControlUnicastCase("legacy business routing is the same",false,true),TestCase::QUICK);
  }
} static g_controlUnicastSuite;

} // namespace ns3::cafbcr
