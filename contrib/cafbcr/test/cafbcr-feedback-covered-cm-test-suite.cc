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
#include "ns3/test.h"
#include "ns3/udp-socket-factory.h"

namespace ns3::cafbcr {

// These cases exercise the delayed response callback, real UDP socket attempts,
// and the existing feedback expiry event; the wireless fixture covers CH loss.
class CafbcrFeedbackCoveredCmCase : public TestCase
{
public:
  enum Kind { VALID, DEFAULT_OFF, NO_FEEDBACK, PRIOR_SOCKET_SUCCESS, PRIOR_SOCKET_FAILURE,
    EXPIRES_IN_JITTER, EXACT_EXPIRY, OLD_CALLBACK, WRONG_HEAD, WRONG_CLUSTER,
    WRONG_COVERAGE, PRE_ASSOCIATION, ORDINARY, IDLE, PROMOTED, PROXY_OFF,
    RENEWAL_OFF, MASTER_OFF, FEEDBACK_OFF, PERIODIC, PROXY_CONFIRMATION,
    ORDINARY_CONFIRMATION, DISPOSED, NO_REMOTE_STATE };
  CafbcrFeedbackCoveredCmCase(const char* name, Kind kind) : TestCase(name), m_kind(kind) {}
private:
  Kind m_kind;
  std::vector<JoinMechanismEvent> m_join;
  std::vector<ProxyRenewalEvent> m_renewal;
  void Join(const JoinMechanismEvent& e) { m_join.push_back(e); }
  void Renewal(const ProxyRenewalEvent& e) { m_renewal.push_back(e); }
  void DoRun() override
  {
    auto manager = ClusterManager::GetInstance();
    NS_TEST_ASSERT_MSG_EQ(manager->GetAllClusters().empty(), true, "clean local fixture");
    NodeContainer nodes; nodes.Create(2);
    InternetStackHelper stack; stack.Install(nodes);
    std::vector<Ptr<ClusterNode>> states;
    for (uint16_t i=0; i<2; ++i) {
      auto state = CreateObject<ClusterNode>(); state->SetNodeId(i);
      state->SetRole(i ? Head : Member); state->SetClusterId(42);
      nodes.Get(i)->AggregateObject(state);
      nodes.Get(i)->AggregateObject(CreateObject<ConstantPositionMobilityModel>());
      states.push_back(state);
    }
    manager->CreateCluster(42, states[1]);
    manager->JoinCluster(42, states[0]); states[0]->SetRole(Member);
    auto route = CreateObject<RoutingProtocol>();
    route->SetIpv4(nodes.Get(0)->GetObject<Ipv4>());
    CsmaHelper csma; auto devices = csma.Install(nodes);
    Ipv4AddressHelper addresses; addresses.SetBase("10.54.1.0", "255.255.255.0");
    auto interfaces = addresses.Assign(devices);
    route->SetBeaconInterval(Seconds(.1));
    route->SetAttribute("DirectMaintenanceFeedback", BooleanValue(true));
    if (m_kind != DEFAULT_OFF)
      route->SetAttribute("FeedbackCoveredCmResponse", BooleanValue(true));
    route->m_proxyMembership = std::make_pair(1,42);
    route->m_proxyMembershipSince = Seconds(.1);
    route->m_mainAddress = interfaces.GetAddress(0);
    auto socket = Socket::CreateSocket(nodes.Get(0), UdpSocketFactory::GetTypeId());
    socket->SetAllowBroadcast(true);
    NS_TEST_ASSERT_MSG_EQ(socket->Bind(InetSocketAddress(interfaces.GetAddress(0), 0)), 0, "real source socket");
    // SetIpv4 schedules Start at time zero. Attach the test socket afterwards
    // so autonomous HELLO/clustering timers cannot contaminate this fixture.
    Simulator::Schedule(Seconds(.2),[&] {
      route->m_socketAddrs[socket] = Ipv4InterfaceAddress(interfaces.GetAddress(0), Ipv4Mask("255.255.255.0"));
    });
    route->TraceConnectWithoutContext("JoinMechanism", MakeCallback(&CafbcrFeedbackCoveredCmCase::Join,this));
    route->TraceConnectWithoutContext("ProxyRenewal", MakeCallback(&CafbcrFeedbackCoveredCmCase::Renewal,this));
    auto feedback = [&](int64_t sourceUs) {
      DirectMaintenanceHeader sent(500000); sent.AddRecord(0,sourceUs);
      auto packet = Create<Packet>(); packet->AddHeader(sent);
      DirectMaintenanceHeader received; packet->RemoveHeader(received);
      ClusterHeadBeaconHeader head(1,42,interfaces.GetAddress(1),Vector(),Vector(),.8,.4);
      route->ApplyDirectMaintenance(head,received,interfaces.GetAddress(1),77);
    };
    const bool noFeedback = m_kind==NO_FEEDBACK || m_kind==PRIOR_SOCKET_SUCCESS || m_kind==PRIOR_SOCKET_FAILURE;
    if (m_kind==PRIOR_SOCKET_SUCCESS || m_kind==PRIOR_SOCKET_FAILURE)
      Simulator::Schedule(Seconds(.35),[&] {
        if (m_kind==PRIOR_SOCKET_FAILURE) socket->ShutdownSend();
        route->SendClusterMemberBeaconFor("periodic");
      });
    // Insert this before feedback schedules its expiry, to force the dangerous
    // same-timestamp order: response first, fallback second.
    if (m_kind==EXACT_EXPIRY)
      Simulator::Schedule(Seconds(.8),[&] {
        NS_TEST_EXPECT_MSG_EQ(route->CanSuppressClusterMemberResponse(1,42,Seconds(.1)),false,"deadline is exclusive");
        NS_TEST_EXPECT_MSG_EQ(route->m_directMaintenanceFallback.IsRunning(),true,"pure query preserves pending fallback");
        route->SendClusterMemberResponse(1,42,Seconds(.1));
        NS_TEST_EXPECT_MSG_EQ(route->m_directMaintenanceFallback.IsRunning(),true,"actual callback preserves pending fallback");
      });
    Simulator::Schedule(Seconds(.4),[&] {
      if (!noFeedback) feedback(300000);
    });
    if (m_kind!=EXACT_EXPIRY)
      Simulator::Schedule(Seconds(m_kind==EXPIRES_IN_JITTER?.81:.5),[&] {
        if (m_kind==OLD_CALLBACK) {
          route->ClearDirectMaintenanceCoverage();
          route->m_proxyMembershipSince=Seconds(.45); feedback(460000);
          NS_TEST_EXPECT_MSG_EQ(route->CanSuppressClusterMemberResponse(1,42,Seconds(.45)),true,"new association has its own valid evidence");
        }
        if (m_kind==WRONG_COVERAGE) route->m_directMaintenanceCoverage=RoutingProtocol::RenewalKey{0,2,43,300000};
        if (m_kind==PRE_ASSOCIATION) route->m_directMaintenanceCoverage=RoutingProtocol::RenewalKey{0,1,42,50000};
        if (m_kind==ORDINARY) route->m_proxyMembership.reset();
        if (m_kind==IDLE) states[0]->SetRole(Idle);
        if (m_kind==PROMOTED) states[0]->SetRole(Head);
        if (m_kind==PROXY_OFF) route->SetAttribute("EnableProxyJoin",BooleanValue(false));
        if (m_kind==RENEWAL_OFF) route->SetAttribute("EnableProxyRenewal",BooleanValue(false));
        if (m_kind==MASTER_OFF) route->SetAttribute("EnableAsymHandling",BooleanValue(false));
        if (m_kind==FEEDBACK_OFF) route->SetAttribute("DirectMaintenanceFeedback",BooleanValue(false));
        if (m_kind==DISPOSED) route->Dispose();
        if (m_kind==NO_REMOTE_STATE) {
          manager->RemoveCluster(42);
          states[0]->SetClusterId(42); // retain source-local facts with no CH registry
        }
        if (m_kind==PERIODIC || m_kind==PROXY_CONFIRMATION || m_kind==ORDINARY_CONFIRMATION)
          route->SendClusterMemberBeaconFor(m_kind==PERIODIC?"periodic":m_kind==PROXY_CONFIRMATION?"proxy-confirmation":"ordinary-confirmation");
        else
          route->SendClusterMemberResponse(m_kind==WRONG_HEAD?2:1,m_kind==WRONG_CLUSTER?43:42,Seconds(.1));
      });
    Simulator::Stop(Seconds(m_kind==EXACT_EXPIRY || m_kind==EXPIRES_IN_JITTER?.82:.51));
    Simulator::Run();
    unsigned skipped=0, attempted=0, successful=0, resumed=0, hello=0;
    for (const auto& e:m_join) {
      if (e.event=="cm_live_feedback_suppressed") {
        ++skipped;
        NS_TEST_EXPECT_MSG_EQ(e.socketResult.has_value(),false,"omission is never a fabricated send");
        NS_TEST_EXPECT_MSG_EQ(std::string(e.cmTrigger),"ch-live-response","only extra response can be omitted");
      }
      if (e.event=="cm_live_tx_attempt") { ++attempted; if (e.socketResult && *e.socketResult==73) ++successful; }
    }
    for (const auto& e:m_renewal) {
      if (e.event=="source_proxy_resumed_no_feedback") ++resumed;
      if (e.event=="source_hello_tx_attempt") ++hello;
    }
    const bool suppress = m_kind==VALID || m_kind==NO_REMOTE_STATE;
    const bool noSend = suppress || m_kind==IDLE || m_kind==PROMOTED || m_kind==DISPOSED;
    const unsigned expected = noSend?0:(m_kind==PRIOR_SOCKET_SUCCESS || m_kind==PRIOR_SOCKET_FAILURE)?2:1;
    NS_TEST_EXPECT_MSG_EQ(skipped,(suppress?1:0),"only valid current received feedback covers an extra response");
    NS_TEST_EXPECT_MSG_EQ(attempted,expected,"periodic, confirmation and unacknowledged responses retain real attempts");
    NS_TEST_EXPECT_MSG_EQ(successful,(m_kind==PRIOR_SOCKET_FAILURE?0:expected),"socket failure does not create coverage");
    const bool expired=m_kind==EXACT_EXPIRY || m_kind==EXPIRES_IN_JITTER;
    NS_TEST_EXPECT_MSG_EQ(resumed,(expired?1:0),"expiry remains independent of response gating");
    NS_TEST_EXPECT_MSG_EQ(hello,(expired?1:0),"expiry emits real proxy-intent HELLO, not just a state event");
    if (m_kind==NO_REMOTE_STATE)
      NS_TEST_EXPECT_MSG_EQ(manager->GetAllClusters().empty(),true,"suppression uses no remote registry");
    route->Dispose(); manager->RemoveCluster(42); Simulator::Destroy();
  }
};

class CafbcrFeedbackCoveredCmSuite : public TestSuite
{
public:
  CafbcrFeedbackCoveredCmSuite() : TestSuite("cafbcr-feedback-covered-cm",Type::UNIT)
  {
    const char* names[]={"current received feedback covers only response", "default remains legacy",
      "no received feedback", "prior successful socket is not CH evidence", "prior failed socket is not CH evidence",
      "feedback expires during response jitter", "response at exact expiry cannot cancel HELLO fallback",
      "old callback after same-ID rejoin", "callback head changed", "callback cluster changed",
      "cached coverage belongs to another association", "cached CM predates association", "ordinary member",
      "source left cluster", "source promoted", "proxy disabled", "renewal disabled", "asym master disabled",
      "direct feedback disabled", "periodic CM retained", "proxy confirmation CM retained",
      "ordinary confirmation CM retained", "disposed callback", "no remote membership read"};
    for (unsigned i=0;i<std::size(names);++i)
      AddTestCase(new CafbcrFeedbackCoveredCmCase(names[i],static_cast<CafbcrFeedbackCoveredCmCase::Kind>(i)),TestCase::QUICK);
  }
} g_cafbcrFeedbackCoveredCmSuite;
} // namespace ns3::cafbcr
