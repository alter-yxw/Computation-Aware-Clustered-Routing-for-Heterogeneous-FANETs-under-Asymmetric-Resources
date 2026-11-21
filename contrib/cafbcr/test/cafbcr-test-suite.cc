/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "ns3/cafbcr-helper.h"
#include "ns3/cafbcr-routing.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-list-routing-helper.h"
#include "ns3/test.h"
#include "ns3/constant-velocity-mobility-model.h"
#include <cmath>
#include <set>

namespace ns3::cafbcr {
class CafbcrHardfixTestCase : public TestCase
{
public:
  enum Check { PACKET, TAGS, ABC_RNG, Q_RNG, STREAMS, EMPTY_NEIGHBORS, SINGLETON };
  CafbcrHardfixTestCase(const char* name, Check check) : TestCase(name), m_check(check) {}
private:
  template<class H> H RoundTrip(const H& sent, uint32_t expected)
  {
    auto packet = Create<Packet>(7);
    packet->AddHeader(sent);
    NS_TEST_EXPECT_MSG_EQ(sent.GetSerializedSize(), expected, "original CA header size");
    NS_TEST_EXPECT_MSG_EQ(packet->GetSize(), expected+7, "payload length");
    H received;
    NS_TEST_EXPECT_MSG_EQ(packet->RemoveHeader(received), expected, "consumed header length");
    NS_TEST_EXPECT_MSG_EQ(packet->GetSize(), 7, "payload preserved");
    return received;
  }
  template<class H> void Motion(const H& header)
  {
    NS_TEST_ASSERT_MSG_EQ_TOL(header.GetPosition().x, -10.125, 1.1e-6, "signed position x");
    NS_TEST_ASSERT_MSG_EQ_TOL(header.GetPosition().y, 20.25, 1.1e-6, "positive position y");
    NS_TEST_ASSERT_MSG_EQ_TOL(header.GetPosition().z, -30.5, 1.1e-6, "signed position z");
    NS_TEST_ASSERT_MSG_EQ_TOL(header.GetVelocity().x, -12.25, 1.1e-6, "signed velocity x");
    NS_TEST_ASSERT_MSG_EQ_TOL(header.GetVelocity().y, -8.5, 1.1e-6, "signed velocity y");
    NS_TEST_ASSERT_MSG_EQ_TOL(header.GetVelocity().z, -3.125, 1.1e-6, "signed velocity z");
    NS_TEST_ASSERT_MSG_EQ_TOL(header.GetTimeStamp(), 1.125, 1.1e-6, "timestamp");
  }
  void PacketChecks()
  {
    Vector pos(-10.125,20.25,-30.5),vel(-12.25,-8.5,-3.125);
    Ipv4Address ip("10.0.0.7");
    HelloHeader hello(7,42,ip,pos,vel,80,.625,Member,1.125);
    hello.SetComputeCapacity(20.5);hello.SetCurrentLoad(.75f);hello.SetIntentJoinChId(9);
    auto h=RoundTrip(hello,95);Motion(h);
    NS_TEST_ASSERT_MSG_EQ_TOL(h.GetFitness(),.625,1.1e-6,"fitness");
    NS_TEST_ASSERT_MSG_EQ_TOL(h.GetComputeCapacity(),20.5,1.1e-6,"compute field retained");
    NS_TEST_ASSERT_MSG_EQ_TOL(h.GetCurrentLoad(),.75,1.1e-6,"load field retained");
    NS_TEST_ASSERT_MSG_EQ(h.GetIntentJoinChId(),9,"proxy intent retained");
    Motion(RoundTrip(ClusterHeadAnnounceHeader(7,42,ip,pos,vel,80,.625,1.125),80));
    Motion(RoundTrip(JoinClusterRequestHeader(7,ip,pos,vel,80,.625,Member,1.125),79));
    auto response=RoundTrip(ClusterHeadResponseHeader(7,42,ip,1.125),16);
    NS_TEST_ASSERT_MSG_EQ_TOL(response.GetTimeStamp(),1.125,1.1e-6,"response timestamp");
    ClusterHeadBeaconHeader beacon(7,42,ip,pos,vel,.625,1.125);
    for(uint8_t i=0;i<8;++i)beacon.SetGrantedMember(i,100+i);
    auto head=RoundTrip(beacon,88);Motion(head);
    for(uint8_t i=0;i<8;++i)NS_TEST_ASSERT_MSG_EQ(head.GetGrantedMember(i),100+i,"broadcast grants retained");
    Motion(RoundTrip(ClusterMemberBeaconHeader(7,42,ip,pos,vel,.625,1.125),72));
    NS_TEST_ASSERT_MSG_EQ(RoundTrip(TypeHeader(CAFBCR_PROXY_JOIN_REQ),1).IsValid(),true,"proxy type retained");
  }
  void TagChecks()
  {
    auto packet=Create<Packet>(37);CafbcrComputeTaskTag task;
    task.SetWorkload(7.125);task.SetDataSize(65537);task.SetDeadline(1.23456789);
    task.SetSourceNodeId(65534);task.SetTaskId(0xffffffff);packet->AddPacketTag(task);
    CafbcrClusterHopTag hop;hop.SetClusterHeadId(42);packet->AddPacketTag(hop);
    auto copied=packet->Copy();CafbcrComputeTaskTag read;CafbcrClusterHopTag readHop;
    NS_TEST_ASSERT_MSG_EQ(copied->PeekPacketTag(read),true,"compute tag present");
    NS_TEST_ASSERT_MSG_EQ_TOL(read.GetWorkload(),7.125,0.0,"workload bits");
    NS_TEST_ASSERT_MSG_EQ_TOL(read.GetDeadline(),1.23456789,0.0,"deadline bits");
    NS_TEST_ASSERT_MSG_EQ(read.GetDataSize(),65537,"data size");
    NS_TEST_ASSERT_MSG_EQ(read.GetSourceNodeId(),65534,"source ID");
    NS_TEST_ASSERT_MSG_EQ(read.GetTaskId(),0xffffffff,"task ID");
    NS_TEST_ASSERT_MSG_EQ(copied->PeekPacketTag(readHop),true,"CH tag present");
    NS_TEST_ASSERT_MSG_EQ(readHop.GetClusterHeadId(),42,"CH ID");
    NS_TEST_ASSERT_MSG_EQ(copied->GetSize(),37,"packet tags do not add payload bytes");
  }
  std::vector<double> Draw(uint32_t seed,uint64_t run,int64_t stream,bool q)
  {
    RngSeedManager::SetSeed(seed);RngSeedManager::SetRun(run);
    std::vector<double> values;
    if(q){
      auto manager=CreateObject<QLearningHelloManager>();manager->AssignStreams(stream);
      manager->m_epsilon=1.0;QLearningHelloManager::State state{1,1,1,1};
      for(int i=0;i<128;++i)values.push_back(manager->SelectAction(state));
    }else{
      auto election=CreateObject<cafbcrElection>();election->AssignStreams(stream);
      for(int i=0;i<128;++i)values.push_back(election->m_rand->GetValue());
    }
    return values;
  }
  void RngChecks(bool q)
  {
    auto expected=Draw(2025,1,9000,q);
    NS_TEST_ASSERT_MSG_EQ((expected==Draw(2025,1,9000,q)),true,"same seed/run/stream reproduces");
    NS_TEST_ASSERT_MSG_EQ((expected!=Draw(2026,1,9000,q)),true,"seed controls protocol draws");
    NS_TEST_ASSERT_MSG_EQ((expected!=Draw(2025,2,9000,q)),true,"run controls protocol draws");
    NS_TEST_ASSERT_MSG_EQ((expected!=Draw(2025,1,9001,q)),true,"streams are distinct");
    if(q)for(auto action:expected)NS_TEST_ASSERT_MSG_EQ((action>=0&&action<5),true,"action index bounds");
  }
  void StreamChecks()
  {
    NodeContainer nodes;nodes.Create(3);
    for(auto i=nodes.Begin();i!=nodes.End();++i){
      auto metadata=CreateObject<ClusterNode>();metadata->SetNodeId((*i)->GetId());
      (*i)->AggregateObject(metadata);
    }
    cafbcrHelper ca;Ipv4ListRoutingHelper list;list.Add(ca,0);
    InternetStackHelper stack;stack.SetRoutingHelper(list);stack.Install(nodes);
    NS_TEST_ASSERT_MSG_EQ(ca.AssignStreams(nodes,2000000),10,"three streams per node plus one manager stream");
    for(uint32_t i=0;i<3;++i){
      auto election=nodes.Get(i)->GetObject<cafbcrElection>();
      NS_TEST_ASSERT_MSG_EQ(election->m_rand->GetStream(),2000001+3*i,"distinct election stream");
      auto protocols=DynamicCast<Ipv4ListRouting>(nodes.Get(i)->GetObject<Ipv4>()->GetRoutingProtocol());
      int16_t priority;auto routing=DynamicCast<RoutingProtocol>(protocols->GetRoutingProtocol(0,priority));
      NS_TEST_ASSERT_MSG_NE(routing,nullptr,"CA protocol installed");
      NS_TEST_ASSERT_MSG_EQ(routing->GetQLearningHelloManager()->m_rand->GetStream(),2000002+3*i,"distinct Q-Hello stream");
      NS_TEST_ASSERT_MSG_EQ(routing->GetHelloInterval(),Seconds(.25),"original helper Hello default");
      NS_TEST_ASSERT_MSG_EQ(routing->GetBeaconInterval(),Seconds(.75),"original helper beacon default");
    }
  }
  void ElectionChecks(bool singleton)
  {
    NodeContainer nodes;nodes.Create(singleton?1:20);
    std::vector<Ptr<ClusterNode>> metadata;
    for(uint32_t i=0;i<nodes.GetN();++i){
      auto c=CreateObject<ClusterNode>();c->SetNodeId(i);c->SetNetworkScale(nodes.GetN());
      c->SetOriginEnergy(200);c->updateResidualEnergy(100+i);
      c->SetPosition(Vector(i*10,20,5));c->SetVelocity(Vector(-double(i),2,0));
      nodes.Get(i)->AggregateObject(c);metadata.push_back(c);
    }
    auto election=CreateObject<cafbcrElection>();election->AssignStreams(500);election->Initialize();
    if(singleton){
      election->InitBee(0);election->EmpolyBeePhase();election->GetProbability();election->OnlookerBeePhase();
      NS_TEST_ASSERT_MSG_EQ(election->GetMaxClusterNum(),0,"original N/10 policy retained");
    }else{
      election->StartElection();std::set<uint16_t> heads;
      for(auto c:election->GetClusterHeads())heads.insert(c->GetNodeId());
      NS_TEST_ASSERT_MSG_EQ(heads.size(),2,"CA already excluded elected heads before this patch");
      for(auto c:metadata){auto id=c->GetNodeId();
        NS_TEST_ASSERT_MSG_EQ_TOL(c->GetResidualEnergy(),100+id,1e-12,"existing whole-round energy restoration retained");
        NS_TEST_ASSERT_MSG_EQ(c->GetPosition(),Vector(10*id,20,5),"existing position restoration retained");
        NS_TEST_ASSERT_MSG_EQ(c->GetVelocity(),Vector(-double(id),2,0),"existing velocity restoration retained");
      }
    }
  }
  void DoRun() override
  {
    switch(m_check){
      case PACKET:PacketChecks();break;case TAGS:TagChecks();break;
      case ABC_RNG:RngChecks(false);break;case Q_RNG:RngChecks(true);break;
      case STREAMS:StreamChecks();break;case EMPTY_NEIGHBORS:ElectionChecks(false);break;
      case SINGLETON:ElectionChecks(true);break;
    }
    Simulator::Destroy();RngSeedManager::SetSeed(1);RngSeedManager::SetRun(1);
  }
  Check m_check;
};
class CafbcrHardfixTestSuite : public TestSuite
{
public:
  CafbcrHardfixTestSuite():TestSuite("cafbcr",UNIT)
  {
    AddTestCase(new CafbcrHardfixTestCase("signed headers with original CA wire layout",CafbcrHardfixTestCase::PACKET),TestCase::QUICK);
    AddTestCase(new CafbcrHardfixTestCase("compute and CH packet tags",CafbcrHardfixTestCase::TAGS),TestCase::QUICK);
    AddTestCase(new CafbcrHardfixTestCase("ABC seed run and stream control",CafbcrHardfixTestCase::ABC_RNG),TestCase::QUICK);
    AddTestCase(new CafbcrHardfixTestCase("Q-Hello seed run and stream control",CafbcrHardfixTestCase::Q_RNG),TestCase::QUICK);
    AddTestCase(new CafbcrHardfixTestCase("helper stream coverage and original intervals",CafbcrHardfixTestCase::STREAMS),TestCase::QUICK);
    AddTestCase(new CafbcrHardfixTestCase("empty-neighbor sampling terminates and existing restoration holds",CafbcrHardfixTestCase::EMPTY_NEIGHBORS),TestCase::QUICK);
    AddTestCase(new CafbcrHardfixTestCase("singleton sampling terminates",CafbcrHardfixTestCase::SINGLETON),TestCase::QUICK);
  }
};
static CafbcrHardfixTestSuite g_cafbcrHardfixTestSuite;

class CafbcrFuzzyLifetimeTestCase : public TestCase
{
public:
  CafbcrFuzzyLifetimeTestCase(const char* name, Vector relativePosition,
                            Vector relativeVelocity, double expected,
                            Vector position = Vector(), Vector velocity = Vector())
    : TestCase(name), m_position(position), m_velocity(velocity),
      m_relativePosition(relativePosition), m_relativeVelocity(relativeVelocity),
      m_expected(expected) {}

private:
  void DoRun() override
  {
    NodeContainer nodes;
    nodes.Create(2);
    auto own = CreateObject<ConstantVelocityMobilityModel>();
    auto peer = CreateObject<ConstantVelocityMobilityModel>();
    nodes.Get(0)->AggregateObject(own);
    nodes.Get(1)->AggregateObject(peer);
    own->SetPosition(m_position);
    own->SetVelocity(m_velocity);
    peer->SetPosition(m_position + m_relativePosition);
    peer->SetVelocity(m_velocity + m_relativeVelocity);
    auto manager = CreateObject<FuzzyHelloManager>();
    manager->SetNodeId(nodes.Get(0)->GetId());
    manager->SetMaxRange(500.0);
    const double lifetime = manager->EstimateLinkLifetime(nodes.Get(1)->GetId());
    NS_TEST_EXPECT_MSG_EQ_TOL(lifetime, m_expected, 1e-8,
                             "time to leave the 500m sphere, capped at 10s");
    NS_TEST_EXPECT_MSG_EQ((std::isfinite(lifetime) && lifetime >= 0 && lifetime <= 10),
                          true, "bounded finite fuzzy input");
    Simulator::Destroy();
  }

  Vector m_position, m_velocity, m_relativePosition, m_relativeVelocity;
  double m_expected;
};

class CafbcrFuzzyLifetimeTestSuite : public TestSuite
{
public:
  CafbcrFuzzyLifetimeTestSuite() : TestSuite("cafbcr-fuzzy-lifetime", UNIT)
  {
    AddTestCase(new CafbcrFuzzyLifetimeTestCase("separating link expires in one second",
      Vector(450,0,0), Vector(50,0,0), 1), TestCase::QUICK);
    AddTestCase(new CafbcrFuzzyLifetimeTestCase("approaching link exits through opposite side",
      Vector(450,0,0), Vector(-50,0,0), 10), TestCase::QUICK);
    AddTestCase(new CafbcrFuzzyLifetimeTestCase("fast approaching link has finite exit time",
      Vector(450,0,0), Vector(-200,0,0), 4.75), TestCase::QUICK);
    AddTestCase(new CafbcrFuzzyLifetimeTestCase("tangential motion reaches sphere boundary",
      Vector(450,0,0), Vector(0,100,0), std::sqrt(47500.0)/100), TestCase::QUICK);
    AddTestCase(new CafbcrFuzzyLifetimeTestCase("equal velocities retain maximum lifetime",
      Vector(450,0,0), Vector(), 10, Vector(20,-30,40), Vector(60,-20,10)), TestCase::QUICK);
    AddTestCase(new CafbcrFuzzyLifetimeTestCase("coincident moving nodes have a finite exit",
      Vector(), Vector(100,0,0), 5), TestCase::QUICK);
    AddTestCase(new CafbcrFuzzyLifetimeTestCase("outward motion at boundary expires now",
      Vector(500,0,0), Vector(50,0,0), 0), TestCase::QUICK);
    AddTestCase(new CafbcrFuzzyLifetimeTestCase("inward motion at boundary remains inside",
      Vector(500,0,0), Vector(-200,0,0), 5), TestCase::QUICK);
    AddTestCase(new CafbcrFuzzyLifetimeTestCase("tangential motion at boundary expires now",
      Vector(500,0,0), Vector(0,50,0), 0), TestCase::QUICK);
    AddTestCase(new CafbcrFuzzyLifetimeTestCase("stationary boundary remains within closed range",
      Vector(500,0,0), Vector(), 10), TestCase::QUICK);
    AddTestCase(new CafbcrFuzzyLifetimeTestCase("outside link has no current remaining lifetime",
      Vector(501,0,0), Vector(-50,0,0), 0), TestCase::QUICK);
    AddTestCase(new CafbcrFuzzyLifetimeTestCase("slow outward motion still crosses nearby boundary",
      Vector(499.99,0,0), Vector(0.05,0,0), 0.2), TestCase::QUICK);
    AddTestCase(new CafbcrFuzzyLifetimeTestCase("3D translation and shared velocity preserve estimate",
      Vector(0,0,450), Vector(0,0,50), 1, Vector(-700,100,-50), Vector(80,-30,5)), TestCase::QUICK);
    AddTestCase(new CafbcrFuzzyLifetimeTestCase("near boundary outward root stays nonnegative",
      Vector(500-1e-9,0,0), Vector(50,0,0), 1e-9/50), TestCase::QUICK);
  }
};
static CafbcrFuzzyLifetimeTestSuite g_cafbcrFuzzyLifetimeTestSuite;
} // namespace ns3::cafbcr
