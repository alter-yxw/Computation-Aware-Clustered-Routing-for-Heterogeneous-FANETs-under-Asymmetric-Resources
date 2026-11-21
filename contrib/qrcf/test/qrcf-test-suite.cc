/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "ns3/qrcf-model.h"
#include "ns3/qrcf-neighbor-table.h"
#include "ns3/qrcf-learning.h"
#include "ns3/qrcf-discovery.h"
#include "ns3/qrcf-route-planner.h"
#include "ns3/qrcf-diagnostics.h"
#include "ns3/packet.h"
#include "ns3/test.h"
#include <cmath>
#include <limits>

using namespace ns3;
using namespace ns3::qrcf;
namespace
{
class FormulaTest : public TestCase
{
  public:
    FormulaTest() : TestCase("QRCF formula examples and explicitly undefined domains") {}
    void DoRun() override
    {
        const double pi = std::acos(-1.0);
        const Motion x{10,0,pi/2}, y{5,pi/2,pi/2};
        NS_TEST_ASSERT_MSG_EQ_TOL(Model::Velocity(x).x,10,1e-12,"Polar angle measured from z");
        NS_TEST_ASSERT_MSG_EQ_TOL(Model::Velocity(x).z,0,1e-12,"Horizontal velocity");
        NS_TEST_ASSERT_MSG_EQ_TOL(Model::RelativeSpeed(x,y),std::sqrt(125),1e-12,"True velocity difference");
        const Motion slower{5,0,pi/2};
        NS_TEST_ASSERT_MSG_EQ_TOL(*Model::ConnectionTime(250,50,x,slower,-1,1),60,1e-12,"Approaching CT");
        NS_TEST_ASSERT_MSG_EQ_TOL(*Model::ConnectionTime(250,50,x,slower,1,1),40,1e-12,"Departing CT");
        NS_TEST_ASSERT_MSG_EQ(Model::ConnectionTime(250,50,x,x,1,1).has_value(),false,"Zero speed is unspecified");
        NS_TEST_ASSERT_MSG_EQ_TOL(Model::Predict({0,0,0},x,2).x,20,1e-12,"Prediction");
        NS_TEST_ASSERT_MSG_EQ_TOL(Model::InitialHello(1,3),2,1e-12,"Initial midpoint");
        NS_TEST_ASSERT_MSG_EQ_TOL(*Model::AdaptHello(2,1,3,10,1,5,5,10),2*std::exp(-0.5),1e-12,"High error decreases HT");
        NS_TEST_ASSERT_MSG_EQ_TOL(*Model::AdaptHello(2,1,3,0,1,5,5,10),3,1e-12,"Low error respects upper bound");
        NS_TEST_ASSERT_MSG_EQ_TOL(*Model::AdaptHello(2,1,3,2,1,5,0,0),2,1e-12,"Middle branch independent of CT");
        NS_TEST_ASSERT_MSG_EQ(Model::AdaptHello(2,1,3,0,1,5,0,0).has_value(),false,"No implicit CT policy");
        NS_TEST_ASSERT_MSG_EQ_TOL(*Model::Normalize(7,5,9),0.5,1e-12,"Minmax");
        NS_TEST_ASSERT_MSG_EQ(Model::Normalize(5,5,5).has_value(),false,"Equal initial energies need a policy");
        NS_TEST_ASSERT_MSG_EQ(Model::Normalize(std::numeric_limits<double>::infinity(),0,10).has_value(),false,"Infinite CT needs a policy");
        NS_TEST_ASSERT_MSG_EQ_TOL(*Model::MovementAngle({0,0,0},{5,0,0},{10,0,0}),0,1e-12,"Geometric forward angle");
        NS_TEST_ASSERT_MSG_EQ_TOL(*Model::MovementAngle({0,0,0},{0,5,0},{10,0,0}),pi/2,1e-12,"Geometric right angle");
        NS_TEST_ASSERT_MSG_EQ_TOL(Model::Reward({0.2,0.6,1,0.4},{0.1,0.2,0.3,0.4}),0.74,1e-12,"Four reward terms");
        NS_TEST_ASSERT_MSG_EQ_TOL(Model::LearningRate(1),std::exp(-1.0),1e-12,"Equation 33 never reaches zero");
        NS_TEST_ASSERT_MSG_EQ_TOL(*Model::Discount({1,2},{2,3}),1.0/3,1e-12,"Neighbor-set Jaccard");
        NS_TEST_ASSERT_MSG_EQ(Model::Discount({},{}).has_value(),false,"Empty union undefined");
        NS_TEST_ASSERT_MSG_EQ_TOL(Model::UpdateQ(2,1,0.5,0.8,3),2.7,1e-12,"Bellman update");
        NS_TEST_ASSERT_MSG_EQ(*Model::AlignedFilter({0,0,0},{10,0,0},{100,0,0},250),true,"Aligned axis follows horizontal destination");
        NS_TEST_ASSERT_MSG_EQ(*Model::AlignedFilter({0,0,0},{-10,0,0},{100,0,0},250),false,"Reject behind aligned cylinder");
        NS_TEST_ASSERT_MSG_EQ(*Model::AlignedFilter({0,0,0},{0,0,-10},{0,0,-100},250),true,"Aligned axis can point down");
        NS_TEST_ASSERT_MSG_EQ(*Model::AlignedFilter({0,0,0},{0,0,-10},{0,0,100},250),false,"Aligned axis can point up");
        NS_TEST_ASSERT_MSG_EQ(*Model::AlignedFilter({0,0,0},{250,250,0},{100,0,0},250),true,"Cylinder boundary inclusive, neighbor sphere enforced by caller");
        NS_TEST_ASSERT_MSG_EQ(*Model::AlignedFilter({0,0,0},{251,0,0},{100,0,0},250),false,"Cylinder height boundary");
        NS_TEST_ASSERT_MSG_EQ(Model::AlignedFilter({0,0,0},{1,0,0},{0,0,0},250).has_value(),false,"Zero aligned axis undefined");
    }
};
class WireNeighborTest : public TestCase
{
  public:
    WireNeighborTest() : TestCase("QRCF Hello serialization, validity refresh and duplicate handling") {}
    void DoRun() override
    {
        HelloHeader hello;
        hello.messageId = (uint64_t{77} << 32) | 3; hello.nodeId = 77;
        hello.position = {1.25,-10,20}; hello.motion = {15,-0.5,1.2};
        hello.energy = 1000; hello.helloInterval = 2;
        auto p = Create<Packet>(); p->AddHeader(hello);
        HelloHeader decoded;
        NS_TEST_ASSERT_MSG_EQ(p->RemoveHeader(decoded),76,"Literal Hello wire bytes");
        NS_TEST_ASSERT_MSG_EQ(decoded.messageId,hello.messageId,"Full message ID");
        NS_TEST_ASSERT_MSG_EQ_TOL(decoded.position.y,-10,1e-12,"Signed coordinate round trip");
        NS_TEST_ASSERT_MSG_EQ_TOL(decoded.motion.polar,1.2,1e-12,"Polar angle round trip");
        NS_TEST_ASSERT_MSG_EQ(Create<Packet>(75)->RemoveHeader(decoded),0,"Reject truncation");
        auto bad = hello; bad.energy = std::numeric_limits<double>::quiet_NaN();
        auto malformed = Create<Packet>(); malformed->AddHeader(bad);
        NS_TEST_ASSERT_MSG_EQ(malformed->RemoveHeader(decoded),0,"Reject nonfinite fields");
        LearningHeader learning;
        learning.gamma = 0.75;
        auto extended = Create<Packet>(); extended->AddHeader(learning);
        MessageHeader message; extended->AddHeader(message);
        MessageHeader parsedMessage; LearningHeader parsedLearning;
        NS_TEST_ASSERT_MSG_EQ(extended->RemoveHeader(parsedMessage),4,"Versioned envelope costs real bytes");
        NS_TEST_ASSERT_MSG_EQ(extended->RemoveHeader(parsedLearning),10,"Fixed gamma-only extension");
        NS_TEST_ASSERT_MSG_EQ_TOL(parsedLearning.gamma,0.75,1e-12,"Gamma round trip");
        auto shortLearning = Create<Packet>(); shortLearning->AddHeader(learning);
        shortLearning->RemoveAtEnd(1);
        NS_TEST_ASSERT_MSG_EQ(shortLearning->RemoveHeader(parsedLearning),0,"Reject truncated gamma extension");
        uint8_t legacy[22]{}; // gamma=0, count=1, one obsolete Q entry
        legacy[9] = 1;
        auto legacyPacket = Create<Packet>(legacy,sizeof(legacy));
        NS_TEST_ASSERT_MSG_EQ(legacyPacket->RemoveHeader(parsedLearning),0,"Reject removed Q-list format");
        FeedbackHeader feedback; feedback.dataId = uint64_t{1} << 40;
        feedback.destination = Ipv4Address("10.0.0.9");
        auto report = Create<Packet>(); report->AddHeader(feedback);
        FeedbackHeader parsedFeedback;
        NS_TEST_ASSERT_MSG_EQ(report->RemoveHeader(parsedFeedback),13,"Feedback actual wire length");
        NS_TEST_ASSERT_MSG_EQ(parsedFeedback.dataId,feedback.dataId,"Full feedback packet correlation ID");
        NS_TEST_ASSERT_MSG_EQ(parsedFeedback.destination,feedback.destination,"Destination-scoped feedback");
        NeighborTable table;
        const Ipv4Address peer("10.0.0.2");
        NS_TEST_ASSERT_MSG_EQ(table.Receive(peer,hello,Seconds(1),Seconds(3)),true,"Add neighbor");
        NS_TEST_ASSERT_MSG_EQ(table.Receive(peer,hello,Seconds(2),Seconds(3)),false,"Duplicate cannot extend validity");
        NS_TEST_ASSERT_MSG_EQ(table.Entries().at(peer).expires,Seconds(4),"Expiry is upper bound, not peer HT");
        ++hello.messageId; hello.position.x = 2;
        table.Receive(peer,hello,Seconds(3),Seconds(3));
        auto replay = hello; --replay.messageId;
        NS_TEST_ASSERT_MSG_EQ(table.Receive(peer,replay,Seconds(4),Seconds(3)),false,"Older replay must not refresh latest state");
        NS_TEST_ASSERT_MSG_EQ_TOL(table.Entries().at(peer).previousHello->position.x,1.25,1e-12,"Previous observation retained");
        NS_TEST_ASSERT_MSG_EQ(table.Purge(Seconds(6),false),0,"Explicit retain-at-boundary policy");
        NS_TEST_ASSERT_MSG_EQ(table.Purge(Seconds(6),true),1,"Explicit expire-at-boundary policy");
    }
};
class LearningTest : public TestCase
{
  public:
    LearningTest() : TestCase("QRCF candidate normalization domain, Bellman learning and failure penalties") {}
    void DoRun() override
    {
        const double pi = std::acos(-1.0);
        const Motion self{10,0,pi/2};
        std::vector<Candidate> candidates{{1,{5,0,0},self,20,10},
                                         {2,{0,5,0},{0,0,pi/2},100,20}};
        auto scores = Learning::Evaluate({0,0,0},self,{10,0,0},candidates,{0.25,0.25,0.25,0.25});
        NS_TEST_ASSERT_MSG_EQ_TOL(*scores[0].ordinaryReward,0.5,1e-12,"All four normalization domains");
        NS_TEST_ASSERT_MSG_EQ_TOL(*scores[1].ordinaryReward,0.5,1e-12,"Competing reward components");
        NS_TEST_ASSERT_MSG_EQ_TOL(*scores[0].learningRate,1,1e-12,"Lowest CT learns fastest");
        NS_TEST_ASSERT_MSG_EQ_TOL(*scores[1].learningRate,std::exp(-1.0),1e-12,"Highest CT learns slower");
        candidates[0].energy = 100;
        scores = Learning::Evaluate({0,0,0},self,{10,0,0},candidates,{0.25,0.25,0.25,0.25});
        NS_TEST_ASSERT_MSG_EQ(scores[0].ordinaryReward.has_value(),false,"Equal energy does not silently normalize");
        NS_TEST_ASSERT_MSG_EQ(scores[0].learningRate.has_value(),true,"Independent CT remains usable");
        scores = Learning::Evaluate({0,0,0},self,{10,0,0},candidates,{0.25,0.25,0.25,0.25},
            [](std::size_t,double,double,double) { return Scalar{0.5}; });
        NS_TEST_ASSERT_MSG_EQ_TOL(*scores[0].ordinaryReward,0.625,1e-12,"Explicit caller policy only");
        candidates[0].connectionTime.reset();
        scores = Learning::Evaluate({0,0,0},self,{10,0,0},candidates,{0.25,0.25,0.25,0.25});
        NS_TEST_ASSERT_MSG_EQ(scores[1].learningRate.has_value(),false,"An undefined candidate must not disappear from FS minmax");
        QTable q;
        NS_TEST_ASSERT_MSG_EQ_TOL(q.Value(1),0,1e-12,"Initial Q is zero");
        NS_TEST_ASSERT_MSG_EQ(q.BestActions({1,2}).size(),2,"No arbitrary tie resolution");
        q.Update(1,-2,1,0,0); q.Update(2,-1,1,0,0);
        NS_TEST_ASSERT_MSG_EQ_TOL(*q.Maximum({1,2}),-1,1e-12,"All-negative Q must not select phantom zero");
        NS_TEST_ASSERT_MSG_EQ_TOL(*q.Maximum({1,2,3}),0,1e-12,"Unvisited admissible action starts at zero");
        NS_TEST_ASSERT_MSG_EQ(q.Maximum({}).has_value(),false,"Empty next-state domain needs terminal policy");
        q.Update(2,4,0.5,0.25,2);
        NS_TEST_ASSERT_MSG_EQ_TOL(q.Value(2),1.75,1e-12,"Known positive Bellman result");
        NS_TEST_ASSERT_MSG_EQ(q.BestActions({1,2,3}).front(),2,"Exploit greatest Q");
        q.Update(2,-4,1,0,0);
        NS_TEST_ASSERT_MSG_EQ_TOL(q.Value(2),-4,1e-12,"Explicit failure reward penalizes previous action");
        NS_TEST_ASSERT_MSG_EQ(q.Updates(),4,"Update accounting");
    }
};
class DiscoveryPlannerTest : public TestCase
{
  public:
    DiscoveryPlannerTest() : TestCase("QRCF approved adaptive-Hello conventions and geometric route preparation") {}
    void DoRun() override
    {
        Settings settings;
        settings.Validate();
        const Motion moving = Settings::FromVelocity({10,0,0});
        NS_TEST_ASSERT_MSG_EQ_TOL(Settings::FromVelocity({0,0,0}).polar,0,1e-12,"Approved stationary encoding");
        NS_TEST_ASSERT_MSG_EQ_TOL(*settings.Connection({0,0,0},moving,{50,0,0},moving,{}),60,1e-12,"Zero-denominator CT cap");
        const Motion slower = Settings::FromVelocity({9.8,0,0});
        settings.connectionCap = 10000;
        NS_TEST_ASSERT_MSG_EQ_TOL(*settings.Connection({0,0,0},moving,{50,0,0},slower,{}),1500,1e-8,"Faster behind uses R+d");
        NS_TEST_ASSERT_MSG_EQ_TOL(*settings.Connection({50,0,0},moving,{0,0,0},slower,{}),1000,1e-8,"Faster ahead uses R-d");
        settings.connectionCap = 60;
        Discovery discovery(settings);
        NS_TEST_ASSERT_MSG_EQ_TOL(discovery.Interval(),0.625,1e-12,"Approved initial Hello");
        NS_TEST_ASSERT_MSG_EQ(discovery.AfterHello({0,0,0},moving,Seconds(0),{10,20},{1,2}),true,"Initial sample");
        NS_TEST_ASSERT_MSG_EQ_TOL(discovery.Gamma(),0,1e-12,"No neighbor history");
        NS_TEST_ASSERT_MSG_EQ(discovery.AfterHello({6.25,0,0},moving,Seconds(0.625),{10,20},{2,3}),true,"Prediction follows actual elapsed time");
        const double enlarged = 0.625*(1+std::exp(-0.75));
        NS_TEST_ASSERT_MSG_EQ_TOL(discovery.Interval(),enlarged,1e-12,"Low error enlarges interval");
        NS_TEST_ASSERT_MSG_EQ_TOL(discovery.Gamma(),1.0/3,1e-12,"Successive Hello neighbor sets");
        NS_TEST_ASSERT_MSG_EQ(discovery.AfterHello({100,0,0},moving,Seconds(1.625),{10,20},{2,3}),true,"Large prediction error");
        NS_TEST_ASSERT_MSG_EQ_TOL(discovery.Interval(),enlarged*std::exp(-0.75),1e-12,"High error shrinks interval");
        const double saved = discovery.Interval();
        NS_TEST_ASSERT_MSG_EQ(discovery.AfterHello({100,0,0},moving,Seconds(2),{},{}),true,"Isolated sample");
        NS_TEST_ASSERT_MSG_EQ_TOL(discovery.Interval(),saved,1e-12,"No neighbors keeps interval");
        NS_TEST_ASSERT_MSG_EQ_TOL(discovery.Gamma(),0,1e-12,"Empty current neighbor set");
        NS_TEST_ASSERT_MSG_EQ(discovery.AfterHello({100,0,0},moving,Seconds(3),{},{}),true,"Empty union sample");
        NS_TEST_ASSERT_MSG_EQ_TOL(discovery.Gamma(),0,1e-12,"Approved empty union convention");
        NS_TEST_ASSERT_MSG_EQ(discovery.AfterHello({100,0,0},moving,Seconds(4),{Scalar{}},{2}),false,"No silent undefined-CT skip");
        NS_TEST_ASSERT_MSG_EQ(discovery.AfterHello({100,0,0},moving,Seconds(5),{0},{2}),true,"Approved all-zero CT convention");
        NS_TEST_ASSERT_MSG_EQ_TOL(discovery.Interval(),saved,1e-12,"All-zero CT keeps interval");
        NS_TEST_ASSERT_MSG_EQ(discovery.AfterHello({100,0,0},moving,Seconds(6),{},{2}),true,"Out-of-range cached peer excluded only from CT domain");
        NeighborTable table;
        HelloHeader hello; hello.nodeId = 2; hello.helloInterval = 1;
        hello.energy = 100; hello.motion = moving; hello.position = {5,0,0};
        table.Receive(Ipv4Address("10.0.0.2"),hello,Seconds(0),Seconds(1));
        auto plan = RoutePlanner::Prepare(settings,{0,0,0},moving,Ipv4Address("10.0.0.99"),{10,0,0},table,{});
        NS_TEST_ASSERT_MSG_EQ(static_cast<int>(plan.kind),static_cast<int>(RoutePlan::Kind::CANDIDATES),"Forward progress");
        NS_TEST_ASSERT_MSG_EQ(plan.scores.size(),1,"Singleton remains a candidate");
        NS_TEST_ASSERT_MSG_EQ_TOL(*plan.scores[0].ordinaryReward,0.5,1e-12,"Approved singleton normalization");
        NS_TEST_ASSERT_MSG_EQ_TOL(*plan.scores[0].learningRate,std::exp(-0.5),1e-12,"Singleton alpha");
        plan = RoutePlanner::Prepare(settings,{0,0,0},moving,Ipv4Address("10.0.0.2"),{5,0,-1},table,{});
        NS_TEST_ASSERT_MSG_EQ(static_cast<int>(plan.kind),static_cast<int>(RoutePlan::Kind::DIRECT),"Direct destination bypass approved");
        table.Clear(); hello.position = {-5,0,0};
        table.Receive(Ipv4Address("10.0.0.2"),hello,Seconds(0),Seconds(1));
        plan = RoutePlanner::Prepare(settings,{0,0,0},moving,Ipv4Address("10.0.0.99"),{10,0,0},table,{});
        NS_TEST_ASSERT_MSG_EQ(static_cast<int>(plan.kind),static_cast<int>(RoutePlan::Kind::LOCAL_MINIMUM),"No closer filtered candidates");
        Diagnostics diagnostics;
        diagnostics.ObserveNeighbors({1,2},Seconds(1));
        diagnostics.ObserveNeighbors({2,3},Seconds(3));
        NS_TEST_ASSERT_MSG_EQ(diagnostics.CompletedLinks(),1,"Only lost directed link completes");
        NS_TEST_ASSERT_MSG_EQ(diagnostics.OpenLinks(),2,"Do not close surviving links at horizon");
        NS_TEST_ASSERT_MSG_EQ_TOL(diagnostics.AverageCompletedLinkSeconds(),2,1e-12,"Observed link interval");
    }
};
class QrcfSuite : public TestSuite
{
  public:
    QrcfSuite() : TestSuite("qrcf", UNIT)
    {
        AddTestCase(new FormulaTest, TestCase::QUICK);
        AddTestCase(new WireNeighborTest, TestCase::QUICK);
        AddTestCase(new LearningTest, TestCase::QUICK);
        AddTestCase(new DiscoveryPlannerTest, TestCase::QUICK);
    }
} g_qrcfSuite;
}
