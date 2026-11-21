/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
// CA-specific adaptation of the local-feedback tests; FBCR sources remain protected.
#include "ns3/cafbcr-qhello.h"
#include "ns3/boolean.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/double.h"
#include "ns3/ipv4-header.h"
#include "ns3/node.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/test.h"
#include "ns3/udp-header.h"
#include <cmath>
#include <functional>

namespace ns3::cafbcr {
namespace {
using QHello = QLearningHelloManager;
struct QHelloFixture {
  Ptr<Node> node = CreateObject<Node>();
  Ptr<ConstantVelocityMobilityModel> mobility = CreateObject<ConstantVelocityMobilityModel>();
  Ptr<Neighbors> neighbors = CreateObject<Neighbors>();
  Ptr<QHello> controller = CreateObject<QHello>();

  explicit QHelloFixture(bool withNeighbor = true) {
    node->AggregateObject(mobility);
    controller->SetNodeId(node->GetId());
    controller->SetNeighborTable(neighbors);
    controller->SetMaxRange(500);
    controller->SetIntervalLimit(Seconds(0.7));
    controller->AssignStreams(731);
    controller->SetAttribute("ExplorationRate", DoubleValue(0));
    if (withNeighbor) {
      // No remote Node exists for this ID: state must come from advertisements.
      neighbors->AddNeighbor(Neighbors::NeighborInfo(60000, Ipv4Address("10.0.0.2"), 1,
          Vector(100, 0, 0), Vector(), 100, 1, Member, Seconds(0), Seconds(1000)));
    }
  }
};

class QHelloRegressionCase : public TestCase {
public:
  QHelloRegressionCase(unsigned kind, const std::string& name) : TestCase(name), m_kind(kind) {}

private:
  void Lifetime() {
    NS_TEST_ASSERT_MSG_EQ_TOL(QHello::EstimateLinkLifetime(Vector(400, 0, 0), Vector(10, 0, 0), 500),
                              10, 1e-10, "separating link exit");
    NS_TEST_ASSERT_MSG_EQ_TOL(QHello::EstimateLinkLifetime(Vector(400, 0, 0), Vector(-100, 0, 0), 500),
                              9, 1e-10, "approach followed by separation");
    NS_TEST_ASSERT_MSG_EQ_TOL(QHello::EstimateLinkLifetime(Vector(490, 0, 0), Vector(0, 100, 0), 500),
                              std::sqrt(0.99), 1e-10, "tangential motion has finite exit time");
    NS_TEST_ASSERT_MSG_EQ(QHello::EstimateLinkLifetime(Vector(501, 0, 0), Vector(-1, 0, 0), 500),
                          0, "outside range is unavailable even if approaching");
    NS_TEST_ASSERT_MSG_EQ(QHello::EstimateLinkLifetime(Vector(100, 0, 0), Vector(), 500),
                          60, "stationary lifetime is capped, not DBL_MAX");
    NS_TEST_ASSERT_MSG_EQ(QHello::EstimateLinkLifetime(Vector(), Vector(1, 2, 3), 0),
                          0, "invalid range is conservative");
    NS_TEST_ASSERT_MSG_EQ(QHello::EstimateLinkLifetime(Vector(NAN, 0, 0), Vector(), 500),
                          0, "invalid cached kinematics is conservative");
  }

  void Lifecycle() {
    QHelloFixture f;
    auto q = f.controller;
    NS_TEST_ASSERT_MSG_EQ(q->GetStatistics().updates, 0, "no update before first action");
    NS_TEST_ASSERT_MSG_EQ(q->GetOptimalHelloInterval(), Seconds(0.25), "conservative cold start");
    const auto state = q->GetLastState();
    q->ObserveRoutingResult(true);
    q->ObserveRoutingResult(false);
    Simulator::Schedule(Seconds(0.1), [this, q, state]() {
      NS_TEST_ASSERT_MSG_EQ(q->GetOptimalHelloInterval(), Seconds(0.15), "early poll returns remaining time");
      NS_TEST_ASSERT_MSG_EQ(q->GetStatistics().decisions, 1, "early poll cannot overwrite a pending action");
      NS_TEST_ASSERT_MSG_EQ(q->GetQValue(state, QHello::HELLO_VERY_FAST), 0, "no premature update");
    });
    Simulator::Schedule(Seconds(0.25), [this, q, state]() {
      q->GetOptimalHelloInterval();
      NS_TEST_ASSERT_MSG_EQ(q->GetStatistics().updates, 1, "completed action updates once");
      NS_TEST_ASSERT_MSG_EQ(q->GetStatistics().decisions, 2, "successor selected after settlement");
      NS_TEST_ASSERT_MSG_EQ(q->GetLastEnvironment().routingSamples, 2, "local route observations connected");
      NS_TEST_ASSERT_MSG_EQ_TOL(q->GetLastEnvironment().routingFailureRatio, 0.5, 1e-12, "attempt failure ratio");
      NS_TEST_ASSERT_MSG_LT(q->GetQValue(state, QHello::HELLO_VERY_FAST), 0, "feedback changes previous Q value");
      q->UpdateQLearning(QHello::Environment{});
      NS_TEST_ASSERT_MSG_EQ(q->GetStatistics().updates, 1, "cannot settle the new action prematurely");
    });
    Simulator::Stop(Seconds(0.3));
    Simulator::Run();
    NS_TEST_ASSERT_MSG_EQ(q->GetStatistics().updates, 1, "partial final action is not trained as failure");
  }

  void Mask() {
    QHelloFixture f, wide;
    auto q = f.controller;
    q->SetAttribute("ExplorationRate", DoubleValue(1));
    q->SetIntervalLimit(Seconds(0.7));
    wide.controller->SetIntervalLimit(Seconds(2));
    wide.controller->SetAttribute("ExplorationRate", DoubleValue(1));
    wide.controller->SetAttribute("MinExplorationRate", DoubleValue(1));
    wide.controller->SetAttribute("LearningEnabled", BooleanValue(false));
    unsigned left = 120;
    std::function<void()> tick = [&, q]() {
      const auto interval = q->GetOptimalHelloInterval();
      NS_TEST_ASSERT_MSG_LT(interval, Seconds(0.75), "exploration must obey existing TTL with margin");
      if (--left) Simulator::Schedule(interval, tick);
    };
    tick();
    unsigned wideLeft = 120;
    std::function<void()> wideTick = [&]() {
      const auto interval = wide.controller->GetOptimalHelloInterval();
      if (--wideLeft) Simulator::Schedule(interval, wideTick);
    };
    wideTick();
    Simulator::Stop(Seconds(241));
    Simulator::Run();
    NS_TEST_ASSERT_MSG_GT(q->GetStatistics().actions[QHello::HELLO_FAST], 0, "safe slower action is explored");
    NS_TEST_ASSERT_MSG_EQ(q->GetStatistics().actions[QHello::HELLO_NORMAL], 0, "TTL equality excluded");
    NS_TEST_ASSERT_MSG_EQ(q->GetStatistics().actions[QHello::HELLO_SLOW], 0, "one-second action excluded");
    NS_TEST_ASSERT_MSG_EQ(q->GetStatistics().actions[QHello::HELLO_VERY_SLOW], 0, "two-second action excluded");
    for (unsigned i = 0; i < QHello::NUM_ACTIONS; ++i) {
      NS_TEST_ASSERT_MSG_GT(wide.controller->GetStatistics().actions[i], 0,
                            "all original actions remain available with a sufficient lifetime budget");
    }
  }

  void Learning() {
    QHelloFixture active, frozen, unreliable;
    active.controller->SetAttribute("LearningRate", DoubleValue(1));
    frozen.controller->SetAttribute("LearningEnabled", BooleanValue(false));
    unreliable.controller->SetAttribute("LearningRate", DoubleValue(1));
    unsigned left = 120;
    std::function<void()> learn = [&]() {
      const auto dt = active.controller->GetOptimalHelloInterval();
      if (--left) Simulator::Schedule(dt, learn);
    };
    unsigned coldLeft = 120;
    std::function<void()> freeze = [&]() {
      const auto dt = frozen.controller->GetOptimalHelloInterval();
      if (--coldLeft) Simulator::Schedule(dt, freeze);
    };
    unsigned unreliableLeft = 120;
    std::function<void()> penalize = [&]() {
      const auto dt = unreliable.controller->GetOptimalHelloInterval();
      // A controlled environment in which the slower update loses usable routes.
      unreliable.controller->ObserveRoutingResult(dt == Seconds(0.25));
      if (--unreliableLeft) Simulator::Schedule(dt, penalize);
    };
    learn();
    freeze();
    penalize();
    Simulator::Stop(Seconds(80));
    Simulator::Run();
    const auto& a = active.controller->GetStatistics();
    const auto& b = frozen.controller->GetStatistics();
    NS_TEST_ASSERT_MSG_EQ(a.updates, 119, "online updates actually execute");
    NS_TEST_ASSERT_MSG_EQ(b.updates, 0, "frozen-table control never writes Q");
    NS_TEST_ASSERT_MSG_EQ(b.feedbackWindows, 119, "frozen control still observes completed windows");
    NS_TEST_ASSERT_MSG_GT(a.actions[QHello::HELLO_FAST], 90, "learned greedy policy favors lower cost in a stable environment");
    NS_TEST_ASSERT_MSG_EQ(b.actions[QHello::HELLO_FAST], 0, "frozen greedy policy retains its cold-start action");
    NS_TEST_ASSERT_MSG_GT(a.learnedDecisions, 0, "learned values are reused for decisions");
    NS_TEST_ASSERT_MSG_GT(unreliable.controller->GetStatistics().actions[QHello::HELLO_VERY_FAST], 90,
                          "route-failure feedback can reverse the preference for a slower interval");
    NS_TEST_ASSERT_MSG_EQ(std::isnan(active.controller->GetLastEnvironment().routingFailureRatio), true,
                          "no traffic preserves unknown routing quality");
  }

  void Isolation() {
    QHelloFixture empty(false), urgent;
    auto q = empty.controller;
    q->GetOptimalHelloInterval();
    NS_TEST_ASSERT_MSG_EQ(q->GetLastState().stabilityLevel, 3, "no neighbor is unstable");
    NS_TEST_ASSERT_MSG_EQ(q->GetLastState().lifelengthLevel, 0, "no neighbor has no usable lifetime");
    NS_TEST_ASSERT_MSG_EQ(q->GetLastState().energyLevel, 1, "missing energy source stays neutral");
    urgent.neighbors->Clear();
    urgent.neighbors->AddNeighbor(Neighbors::NeighborInfo(60000, Ipv4Address("10.0.0.2"), 1,
        Vector(499, 0, 0), Vector(100, 0, 0), 100, 1, Member, Seconds(0), Seconds(1000)));
    urgent.controller->SetAttribute("ExplorationRate", DoubleValue(1));
    NS_TEST_ASSERT_MSG_EQ(urgent.controller->GetOptimalHelloInterval(), Seconds(0.25), "urgent link masks slow exploration");
    Simulator::Schedule(Seconds(0.25), [this, q]() {
      q->GetOptimalHelloInterval();
      NS_TEST_ASSERT_MSG_EQ(std::isnan(q->GetLastEnvironment().neighborLossRate), true, "empty neighbor sample stays unknown");
      NS_TEST_ASSERT_MSG_EQ(std::isfinite(q->GetStatistics().lastReward), true, "unknown signals do not produce NaN Q values");
    });
    Simulator::Stop(Seconds(0.3));
    Simulator::Run();
  }

  void Duration() {
    QHelloFixture f, narrow;
    auto q = f.controller;
    q->SetAttribute("LearningRate", DoubleValue(1));
    q->SetAttribute("DiscountFactor", DoubleValue(0.5));
    q->GetOptimalHelloInterval();
    auto single = narrow.controller;
    single->SetIntervalLimit(Seconds(0.25));
    single->SetAttribute("LearningRate", DoubleValue(1));
    single->SetAttribute("DiscountFactor", DoubleValue(0.5));
    single->GetOptimalHelloInterval();
    const auto singleState = single->GetLastState();
    Simulator::Schedule(Seconds(0.25), [single]() {
      QHello::Environment env;
      env.normalizedHelloRate = 1;
      single->UpdateQLearning(env);
      single->GetOptimalHelloInterval();
    });
    Simulator::Schedule(Seconds(0.5), [this, single, singleState]() {
      QHello::Environment env;
      env.normalizedHelloRate = 1;
      single->UpdateQLearning(env);
      NS_TEST_ASSERT_MSG_EQ_TOL(single->GetQValue(singleState, QHello::HELLO_VERY_FAST), -0.15, 1e-12,
                                "bootstrap excludes unavailable zero-valued actions");
    });
    const auto state = q->GetLastState();
    Simulator::Schedule(Seconds(0.25), [this, q, state]() {
      QHello::Environment env;
      env.normalizedHelloRate = 1;
      q->UpdateQLearning(env);
      NS_TEST_ASSERT_MSG_EQ_TOL(q->GetQValue(state, QHello::HELLO_VERY_FAST), -0.1, 1e-12, "one reference-period cost");
      q->UpdateQLearning(env);
      NS_TEST_ASSERT_MSG_EQ(q->GetStatistics().updates, 1, "duplicate settlement is ignored");
      NS_TEST_ASSERT_MSG_EQ(q->GetOptimalHelloInterval(), Seconds(0.5), "changed Q values alter next greedy action");
    });
    Simulator::Schedule(Seconds(0.75), [this, q, state]() {
      QHello::Environment env;
      env.normalizedHelloRate = 0.5;
      q->UpdateQLearning(env);
      NS_TEST_ASSERT_MSG_EQ_TOL(q->GetQValue(state, QHello::HELLO_FAST), -0.075, 1e-12,
                                "two-period reward is accumulated with simulated-time discount");
    });
    Simulator::Stop(Seconds(0.8));
    Simulator::Run();
  }

  void Observations() {
    QHelloFixture f;
    auto q = f.controller;
    q->GetOptimalHelloInterval();
    q->ProcessHelloInfo(60000, Vector(100, 0, 0), Vector(), Seconds(0));
    Simulator::Schedule(Seconds(0.2), [q]() {
      q->ProcessHelloInfo(60000, Vector(150, 0, 0), Vector(), Simulator::Now());
      q->ProcessHelloInfo(60000, Vector(), Vector(), Seconds(0.1)); // reordered observation
    });
    Simulator::Schedule(Seconds(0.25), [this, q]() {
      q->GetOptimalHelloInterval();
      NS_TEST_ASSERT_MSG_EQ(q->GetStatistics().receivedHellos, 2, "received Hello observation path connected");
      NS_TEST_ASSERT_MSG_EQ_TOL(q->GetLastEnvironment().motionPredictionError, 0.5, 1e-12,
                                "cached Hello prediction innovation contributes to feedback");
    });
    Simulator::Stop(Seconds(0.3));
    Simulator::Run();
  }

  void ControlPackets() {
    Ipv4Header header;
    header.SetProtocol(17);
    auto packet = Create<Packet>(40);
    UdpHeader udp;
    udp.SetDestinationPort(889);
    packet->AddHeader(udp);
    NS_TEST_ASSERT_MSG_EQ(QHello::IsReceivedControlPacket(packet, header), true,
                          "received CA control UDP is excluded from business feedback");
    packet->RemoveHeader(udp);
    udp.SetDestinationPort(9999);
    packet->AddHeader(udp);
    NS_TEST_ASSERT_MSG_EQ(QHello::IsReceivedControlPacket(packet, header), false,
                          "compute task UDP remains business traffic");
    NS_TEST_ASSERT_MSG_EQ(QHello::IsReceivedControlPacket(Create<Packet>(7), header), false,
                          "short payload cannot contain a UDP header");
    header.SetProtocol(6);
    NS_TEST_ASSERT_MSG_EQ(QHello::IsReceivedControlPacket(packet, header), false,
                          "TCP traffic is not parsed as CA control UDP");
  }

  void CaExpiryAndRange() {
    QHelloFixture expiring, asymmetric;
    expiring.neighbors->Clear();
    expiring.neighbors->AddNeighbor(Neighbors::NeighborInfo(60000, Ipv4Address("10.0.0.2"), 1,
        Vector(100, 0, 0), Vector(), 100, 1, Member, Seconds(0), Seconds(0.1)));
    expiring.controller->GetOptimalHelloInterval();
    asymmetric.controller->SetMaxRange(250);
    asymmetric.controller->SetIntervalLimit(Seconds(3.95));
    asymmetric.neighbors->Clear();
    asymmetric.neighbors->AddNeighbor(Neighbors::NeighborInfo(60000, Ipv4Address("10.0.0.2"), 1,
        Vector(400, 0, 0), Vector(), 100, 1, Member, Seconds(0), Seconds(4)));
    NS_TEST_ASSERT_MSG_EQ(asymmetric.controller->GetOptimalHelloInterval(), Seconds(0.25),
                          "heard neighbor beyond this CA node's own nominal range is conservative");
    Simulator::Schedule(Seconds(0.25), [this, q=expiring.controller, n=expiring.neighbors]() {
      q->GetOptimalHelloInterval();
      NS_TEST_ASSERT_MSG_EQ(q->GetLastState().densityLevel, 0, "expired record does not enter CA state");
      NS_TEST_ASSERT_MSG_EQ_TOL(q->GetLastEnvironment().neighborLossRate, 1, 1e-12, "local expiry is observed");
      NS_TEST_ASSERT_MSG_EQ(n->PeekAll().size(), 1, "learning observation does not purge shared tables");
    });
    Simulator::Stop(Seconds(0.3));
    Simulator::Run();
  }

  void DeferredSourceControl() {
    auto packet = Create<Packet>(40);
    Ipv4Header header;
    header.SetProtocol(1);
    NS_TEST_ASSERT_MSG_EQ(QHello::IsBusinessPacket(packet, header, false), false,
                          "source ICMP TTL errors are not application business");
    NS_TEST_ASSERT_MSG_EQ(QHello::IsBusinessPacket(packet, header, true), false,
                          "forwarded ICMP is not a UDP delivery observation");
    header.SetProtocol(17);
    NS_TEST_ASSERT_MSG_EQ(QHello::IsBusinessPacket(packet, header, false), true,
                          "source UDP payload remains eligible before its UDP header is added");
    NS_TEST_ASSERT_MSG_EQ(QHello::IsOwnControlPacket(packet), false, "unmarked business is not control");
    QHello::MarkOwnControlPacket(packet);
    NS_TEST_ASSERT_MSG_EQ(QHello::IsBusinessPacket(packet, header, false), false,
                          "marked source control is excluded from the business window");
    NS_TEST_ASSERT_MSG_EQ(packet->GetSize(), 40, "marker adds no UDP/IP or PHY payload bytes");
    NS_TEST_ASSERT_MSG_EQ(QHello::IsOwnControlPacket(packet->Copy()), true,
                          "local control origin survives a deferred source route query");
    NS_TEST_ASSERT_MSG_EQ(QHello::IsOwnControlPacket(Create<Packet>(40)), false,
                          "control classification does not leak into another business packet");
  }

  void DoRun() override {
    switch (m_kind) {
      case 0: Lifetime(); break;
      case 1: Lifecycle(); break;
      case 2: Mask(); break;
      case 3: Learning(); break;
      case 4: Isolation(); break;
      case 5: Duration(); break;
      case 6: Observations(); break;
      case 7: ControlPackets(); break;
      case 8: CaExpiryAndRange(); break;
      case 9: DeferredSourceControl(); break;
    }
    Simulator::Destroy();
  }
  unsigned m_kind;
};

class QHelloTestSuite : public TestSuite {
public:
  QHelloTestSuite() : TestSuite("cafbcr-qhello", UNIT) {
    const char* names[] = {"finite 3-D link lifetime boundaries", "action and feedback ordering",
        "TTL-safe exploration", "learned policy versus frozen Q table", "unknown and isolated observations",
        "duration-aware Q update", "local Hello prediction observations", "received CA control versus business payloads",
        "CA local range and non-mutating expiry", "deferred source control origin without wire bytes"};
    for (unsigned i = 0; i < 10; ++i) AddTestCase(new QHelloRegressionCase(i, names[i]), TestCase::QUICK);
  }
} g_qHelloTestSuite;
} // namespace
} // namespace ns3::cafbcr
