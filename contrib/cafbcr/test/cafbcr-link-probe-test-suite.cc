/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "ns3/cafbcr-routing.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/csma-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-route.h"
#include "ns3/test.h"

namespace ns3::cafbcr {
class QuietLinkTestRouting : public RoutingProtocol
{
public:
  static TypeId GetTypeId() {
    static TypeId tid=TypeId("ns3::cafbcr::QuietLinkTestRouting").SetParent<RoutingProtocol>()
      .AddConstructor<QuietLinkTestRouting>();return tid;
  }
protected:void DoInitialize() override { Ipv4RoutingProtocol::DoInitialize(); }
};

class CafbcrLinkProbeCase : public TestCase
{
public:CafbcrLinkProbeCase(const char* name,unsigned kind):TestCase(name),m_kind(kind){}
private:
  void Setup()
  {
    m_nodes.Create(3);InternetStackHelper stack;stack.Install(m_nodes);
    for (uint32_t i=0;i<3;++i) {
      auto c=CreateObject<ClusterNode>();c->SetNodeId(m_nodes.Get(i)->GetId());c->SetClusterId(0xffff);
      c->SetRole(Idle);c->SetOriginEnergy(100);c->updateResidualEnergy(100);m_nodes.Get(i)->AggregateObject(c);
      auto pos=CreateObject<ConstantPositionMobilityModel>();pos->SetPosition(Vector(i==2?150:50*i,0,0));
      m_nodes.Get(i)->AggregateObject(pos);
    }
    CsmaHelper medium;auto dev=medium.Install(m_nodes);Ipv4AddressHelper ip;ip.SetBase("10.69.1.0","255.255.255.0");
    m_ips=ip.Assign(dev);m_route=CreateObject<QuietLinkTestRouting>();
    // Direct local setup avoids Start/election; these are state tests, not Wi-Fi evidence.
    m_route->m_ipv4=m_nodes.Get(0)->GetObject<Ipv4>();m_nodes.Get(0)->AggregateObject(m_route);
    m_route->m_mainAddress=m_ips.GetAddress(0);m_route->m_neighbors=CreateObject<Neighbors>(m_route->MyNodeId());
    m_route->m_maxRange=100;m_route->m_linkProbes=true;m_route->m_handshakeGate=true;
    for (uint32_t i=1;i<3;++i) m_route->m_neighbors->AddNeighbor(Neighbors::NeighborInfo(
      m_nodes.Get(i)->GetId(),m_ips.GetAddress(i),0xffff,Vector(i==1?50:150,0,0),Vector(),100,.8,Idle,Seconds(0),Seconds(100)));
  }
  uint16_t Peer(unsigned i=1) const { return m_nodes.Get(i)->GetId(); }
  void Confirm(unsigned i=1) {
    m_route->m_linkConfirmed.insert_or_assign(Peer(i),RoutingProtocol::LinkConfirmation{m_ips.GetAddress(i),1,Seconds(0),Seconds(0)});
  }
  void Pending(uint64_t nonce=2) {
    m_route->m_linkPending.emplace(Peer(),RoutingProtocol::PendingLinkProbe{m_ips.GetAddress(1),nonce,Seconds(0),EventId()});
  }
  void Reply(uint64_t nonce=2,Ipv4Address ip=Ipv4Address(),uint16_t origin=0xffff) {
    auto p=Create<Packet>();p->AddHeader(LinkProbeHeader(origin==0xffff?m_route->MyNodeId():origin,Peer(),nonce,0));
    m_route->RecvLinkProbe(p,ip==Ipv4Address()?m_ips.GetAddress(1):ip,true);
  }
  bool Allowed(unsigned i=1) { return m_route->DataNeighborAllowed(m_route->m_neighbors->PeekAll().at(Peer(i))); }
  void DoRun() override
  {
    if (m_kind==0) {
      auto p=Create<Packet>();p->AddHeader(LinkProbeHeader(7,13,0xfedcba9876543210ULL,1234567890123456ULL));
      NS_TEST_ASSERT_MSG_EQ(p->GetSize(),20,"real header is 20 bytes");LinkProbeHeader h;p->RemoveHeader(h);
      NS_TEST_ASSERT_MSG_EQ(h.GetOrigin(),7,"origin");NS_TEST_ASSERT_MSG_EQ(h.GetPeer(),13,"peer");
      NS_TEST_ASSERT_MSG_EQ(h.GetNonce(),0xfedcba9876543210ULL,"full integer nonce");
      NS_TEST_ASSERT_MSG_EQ(h.GetSentNs(),1234567890123456ULL,"integer local timestamp");
      for (auto type:{CAFBCR_LINK_REQUEST,CAFBCR_LINK_REPLY}) {
        p->AddHeader(TypeHeader(type));TypeHeader t;p->RemoveHeader(t);
        NS_TEST_ASSERT_MSG_EQ(t.IsValid(),true,"new type decoded");NS_TEST_ASSERT_MSG_EQ(t.Get(),type,"type preserved");
      }
      return;
    }
    Setup();
    if (m_kind==1) { NS_TEST_EXPECT_MSG_EQ(m_route->GetLinkEvidenceState(Peer()),"unknown","no invented evidence");NS_TEST_EXPECT_MSG_EQ(Allowed(),false,"unknown rejected"); }
    if (m_kind==2) {Pending();NS_TEST_EXPECT_MSG_EQ(m_route->GetLinkEvidenceState(Peer()),"pending","pending distinct");NS_TEST_EXPECT_MSG_EQ(Allowed(),false,"pending not a round trip");}
    if (m_kind==3) {Pending();Reply();NS_TEST_EXPECT_MSG_EQ(m_route->GetLinkEvidenceState(Peer()),"confirmed","matched actual handler confirms");NS_TEST_EXPECT_MSG_EQ(m_route->m_linkPending.empty(),true,"pending consumed");Confirm(2);NS_TEST_EXPECT_MSG_EQ(Allowed(2),true,"measured link can be beyond configured radius");}
    if (m_kind==4) {Confirm();Simulator::Schedule(Seconds(6),[this]{NS_TEST_EXPECT_MSG_EQ(m_route->GetLinkEvidenceState(Peer()),"expired","send-anchored exact boundary");NS_TEST_EXPECT_MSG_EQ(Allowed(),false,"expired rejected");});Simulator::Stop(Seconds(6)+NanoSeconds(1));Simulator::Run();}
    if (m_kind==5) {Confirm();auto info=m_route->m_neighbors->PeekAll().at(Peer());info.ipAddress=Ipv4Address("10.69.1.99");m_route->m_neighbors->AddNeighbor(info);NS_TEST_EXPECT_MSG_EQ(m_route->GetLinkEvidenceState(Peer()),"ip_changed","old proof cannot bind new IP");NS_TEST_EXPECT_MSG_EQ(Allowed(),false,"new IP needs evidence");}
    if (m_kind==6) {Pending();Reply(3);NS_TEST_EXPECT_MSG_EQ(m_route->m_linkConfirmed.empty(),true,"wrong nonce rejected");}
    if (m_kind==7) {Pending();Reply(2,Ipv4Address("10.69.1.99"));NS_TEST_EXPECT_MSG_EQ(m_route->m_linkConfirmed.empty(),true,"wrong source IP rejected");}
    if (m_kind==8) {Pending();Reply(2,Ipv4Address(),Peer());NS_TEST_EXPECT_MSG_EQ(m_route->m_linkConfirmed.empty(),true,"wrong echoed origin rejected");}
    if (m_kind==9) {Pending();Simulator::Schedule(MilliSeconds(250),[this]{Reply();NS_TEST_EXPECT_MSG_EQ(m_route->m_linkConfirmed.empty(),true,"deadline is exclusive");});Simulator::Stop(MilliSeconds(251));Simulator::Run();}
    if (m_kind==10) {m_route->RecvLinkProbe(Create<Packet>(19),m_ips.GetAddress(1),true);NS_TEST_EXPECT_MSG_EQ(m_route->m_linkConfirmed.empty(),true,"truncated body is rejected before decoding");}
    if (m_kind==11) {NS_TEST_EXPECT_MSG_EQ(Allowed(),false,"measured data unknown");m_route->m_controlSendDepth=1;NS_TEST_EXPECT_MSG_EQ(Allowed(),true,"local legacy control retains configured-distance rule");m_route->m_controlSendDepth=0;m_route->m_enableRangeGating=false;NS_TEST_EXPECT_MSG_EQ(Allowed(2),true,"independent R-off semantics preserved");}
    if (m_kind==12) {Confirm(2);Vector dst(300,0,0),my,previous;NS_TEST_EXPECT_MSG_EQ(m_route->GreedyReachableNextHop(dst),Peer(2),"greedy filters by proof");NS_TEST_EXPECT_MSG_EQ(m_route->NearestReachableNeighbor(dst),Peer(2),"nearest cannot bypass proof");NS_TEST_EXPECT_MSG_EQ(m_route->PerimeterReachableNextHop(dst,my,previous),Peer(2),"perimeter cannot bypass proof");NS_TEST_EXPECT_MSG_EQ(m_route->GreedyNextHop(dst,true),Peer(),"association helper keeps original distance rule");}
    if (m_kind==13) {Confirm();Pending();Simulator::Schedule(MilliSeconds(250),[this]{m_route->LinkProbeExpired(Peer(),2);NS_TEST_EXPECT_MSG_EQ(m_route->GetLinkEvidenceState(Peer()),"confirmed","one timeout does not erase a still-fresh prior observation");NS_TEST_EXPECT_MSG_EQ(m_route->m_linkConfirmed.at(Peer()).sent,Seconds(0),"timeout does not renew proof");});Simulator::Stop(MilliSeconds(251));Simulator::Run();}
    if (m_kind==14) {Pending();m_route->m_linkProbeTimer=Simulator::Schedule(Seconds(1),[]{});m_route->ClearLinkProbes();NS_TEST_EXPECT_MSG_EQ(m_route->m_linkProbeTimer.IsRunning(),false,"timer cancelled");NS_TEST_EXPECT_MSG_EQ(m_route->m_linkPending.empty(),true,"pending removed");}
    if (m_kind==15) {m_route->m_linkProbes=false;m_route->m_handshakeGate=false;NS_TEST_EXPECT_MSG_EQ(Allowed(),true,"default distance behavior");NS_TEST_EXPECT_MSG_EQ(Allowed(2),false,"default range boundary");NS_TEST_EXPECT_MSG_EQ(m_route->m_linkNonce,0,"no default probes");}
    if (m_kind==16) {
      Confirm(2);auto p=Create<Packet>(100);Ipv4Header h;h.SetDestination(m_ips.GetAddress(2));h.SetSource(m_ips.GetAddress(0));h.SetProtocol(17);h.SetTtl(64);
      Socket::SocketErrno error;auto r=m_route->RouteOutput(p,h,nullptr,error);
      NS_TEST_ASSERT_MSG_NE(r,nullptr,"native route uses fresh proof");NS_TEST_EXPECT_MSG_EQ(r->GetGateway(),m_ips.GetAddress(2),"actual next hop");
      m_route->m_routeCache[m_ips.GetAddress(2)]={m_ips.GetAddress(2),Simulator::Now(),1};m_route->m_linkConfirmed.clear();
      NS_TEST_EXPECT_MSG_EQ(m_route->Forwarding(p,h,RoutingProtocol::UnicastForwardCallback(),RoutingProtocol::ErrorCallback()),false,"cached next hop cannot bypass unknown proof");
    }
    if (m_kind==17) {Confirm();auto info=m_route->m_neighbors->PeekAll().at(Peer());info.NeighborLifeLength=Seconds(0);m_route->m_neighbors->AddNeighbor(info);NS_TEST_EXPECT_MSG_EQ(m_route->GetLinkEvidenceState(Peer()),"neighbor_missing","fresh proof does not revive an expired neighbor");}
    m_route->Dispose();Simulator::Destroy();
  }
  unsigned m_kind;NodeContainer m_nodes;Ipv4InterfaceContainer m_ips;Ptr<RoutingProtocol> m_route;
};
class CafbcrLinkProbeSuite : public TestSuite
{
public:CafbcrLinkProbeSuite():TestSuite("cafbcr-link-probe",UNIT) {
    static const char* names[]={"wire header and types","unknown","pending","confirmation and radius","proof expiry","changed peer IP","wrong nonce","wrong source","wrong origin","deadline","malformed","control and R-off","all candidate helpers","timeout retains earlier proof","cleanup","default distance","native output and cache","expired neighbor"};
    for(unsigned i=0;i<18;++i) AddTestCase(new CafbcrLinkProbeCase(names[i],i),TestCase::QUICK);
  }
} g_cafbcrLinkProbeSuite;
}
