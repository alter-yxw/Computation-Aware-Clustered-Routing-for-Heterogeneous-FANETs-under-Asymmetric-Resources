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

// Calls the actual forwarding handler with fixed packet/tag/neighbor state.
// It is a correctness fixture, not a wireless performance experiment.
class CafbcrForwardingCacheTestCase : public TestCase
{
public:
  enum Kind { OWN_RETURN, IDLE_RETURN, UNMARKED, FOREIGN_MARK, ALTERNATE_CACHE,
              HEAD_DESTINATION, SAME_CLUSTER, EXPIRED, TASK, NO_ASSOCIATION };
  CafbcrForwardingCacheTestCase(const char* name, Kind kind) : TestCase(name), m_kind(kind) {}
private:
  void Observe(Ptr<Ipv4Route> route, Ptr<const Packet> packet, const Ipv4Header& header)
  {
    ++m_callbacks;
    m_next = route->GetGateway();
    NS_TEST_EXPECT_MSG_EQ(route->GetOutputDevice(), m_nodes.Get(0)->GetObject<Ipv4>()->GetNetDevice(1),
                          "the native handler selects its real interface");
    NS_TEST_EXPECT_MSG_EQ(packet->GetUid(), m_uid, "forwarding preserves packet identity");
    NS_TEST_EXPECT_MSG_EQ(packet->GetSize(), 100, "diagnostics and guard add no wire bytes");
    NS_TEST_EXPECT_MSG_EQ(header.GetTtl(), 61, "the IPv4 layer owns TTL decrement");
    NS_TEST_EXPECT_MSG_EQ(header.GetIdentification(), 7, "IPv4 identification is preserved");
    CafbcrClusterHopTag tag;
    bool has = packet->PeekPacketTag(tag);
    NS_TEST_EXPECT_MSG_EQ(has, (m_expectedTag != 0xFFFF), "forwarding preserves the incoming marker");
    if (has) NS_TEST_EXPECT_MSG_EQ(tag.GetClusterHeadId(), m_expectedTag, "marker identity is unchanged");
  }
  Ipv4Address Ip(uint32_t i) const { return m_interfaces.GetAddress(i); }
  void DoRun() override
  {
    auto manager = ClusterManager::GetInstance();
    NS_TEST_ASSERT_MSG_EQ(manager->GetAllClusters().empty(), true, "fixture starts without clusters");
    m_nodes.Create(4);
    InternetStackHelper stack; stack.Install(m_nodes);
    for (uint32_t i = 0; i < m_nodes.GetN(); ++i) {
      auto node = m_nodes.Get(i);
      auto cn = CreateObject<ClusterNode>(); cn->SetNodeId(node->GetId());
      cn->SetRole(Idle); cn->SetClusterId(0xFFFF); cn->SetMaxRange(150);
      cn->SetOriginEnergy(100); cn->updateResidualEnergy(100); node->AggregateObject(cn);
      auto mobility = CreateObject<ConstantPositionMobilityModel>();
      mobility->SetPosition(Vector(i == 0 ? 0 : i == 1 ? -50 : i == 2 ? 100 : 300, 0, 0));
      node->AggregateObject(mobility);
    }
    auto route = CreateObject<RoutingProtocol>();
    route->SetIpv4(m_nodes.Get(0)->GetObject<Ipv4>()); m_nodes.Get(0)->AggregateObject(route);
    route->m_maxRange = 150;
    CsmaHelper medium;
    auto devices = medium.Install(m_nodes);
    Ipv4AddressHelper addresses; addresses.SetBase("10.1.1.0", "255.255.255.0");
    m_interfaces = addresses.Assign(devices);
    route->m_mainAddress = Ip(0);
    auto member = m_nodes.Get(0)->GetObject<ClusterNode>();
    auto head = m_nodes.Get(1)->GetObject<ClusterNode>();
    const uint16_t cid = manager->ArrangeClusterId();
    manager->CreateCluster(cid, head); head->SetRole(Head);
    manager->JoinCluster(cid, member); member->SetRole(Member);
    for (uint32_t i : {1u, 2u}) {
      auto peer = m_nodes.Get(i)->GetObject<ClusterNode>();
      route->m_neighbors->AddNeighbor(Neighbors::NeighborInfo(peer->GetNodeId(), Ip(i), peer->GetClusterId(),
        m_nodes.Get(i)->GetObject<MobilityModel>()->GetPosition(), Vector(), 100, .8,
        peer->GetRoleType(), Simulator::Now(), Seconds(10)));
    }
    auto packet = Create<Packet>(100); m_uid = packet->GetUid();
    Ipv4Header header; header.SetSource(Ip(2)); header.SetDestination(Ip(3));
    header.SetProtocol(17); header.SetTtl(61); header.SetIdentification(7);
    auto callback = MakeCallback(&CafbcrForwardingCacheTestCase::Observe, this);
    // Seed the cache through the actual member-inter-cluster branch.
    NS_TEST_EXPECT_MSG_EQ(route->Forwarding(packet, header, callback, RoutingProtocol::ErrorCallback()), true,
                          "the ordinary packet enters native member-to-head routing");
    NS_TEST_EXPECT_MSG_EQ(m_callbacks, 1, "one callback for the seed packet");
    NS_TEST_EXPECT_MSG_EQ(m_next, Ip(1), "the first ordinary packet uses its own head");
    NS_TEST_EXPECT_MSG_EQ(route->m_routeCache.at(Ip(3)).nextHop, Ip(1), "native forwarding writes the cache");

    auto incoming = packet->Copy();
    if (m_kind != UNMARKED) {
      m_expectedTag = m_nodes.Get(m_kind == FOREIGN_MARK ? 2 : 1)->GetId();
      CafbcrClusterHopTag marker; marker.SetClusterHeadId(m_expectedTag); incoming->AddPacketTag(marker);
    }
    Ipv4Address expected = Ip(1);
    if (m_kind == OWN_RETURN || m_kind == IDLE_RETURN || m_kind == EXPIRED || m_kind == TASK || m_kind == ALTERNATE_CACHE)
      expected = Ip(2);
    if (m_kind == IDLE_RETURN) member->SetRole(Idle); // native A3 applies to all non-Heads
    if (m_kind == SAME_CLUSTER) manager->JoinCluster(cid, m_nodes.Get(3)->GetObject<ClusterNode>());
    if (m_kind == NO_ASSOCIATION) { member->SetClusterId(0xFFFF); member->SetRole(Idle); }
    if (m_kind == EXPIRED) route->m_routeCache.at(Ip(3)).lastUpdate = Seconds(-2);
    if (m_kind == ALTERNATE_CACHE) route->m_routeCache.at(Ip(3)).nextHop = Ip(2);
    if (m_kind == HEAD_DESTINATION) {
      header.SetDestination(Ip(1));
      route->m_routeCache[Ip(1)] = {Ip(1), Simulator::Now(), 1};
    }
    if (m_kind == TASK) {
      CafbcrComputeTaskTag task; task.SetSourceNodeId(m_nodes.Get(2)->GetId()); task.SetTaskId(9);
      incoming->AddPacketTag(task);
    }
    NS_TEST_EXPECT_MSG_EQ(route->Forwarding(incoming, header, callback, RoutingProtocol::ErrorCallback()), true,
                          "the marked packet remains routable through existing fallback");
    NS_TEST_EXPECT_MSG_EQ(m_callbacks, 2, "the second packet gets exactly one callback");
    NS_TEST_EXPECT_MSG_EQ(m_next, expected, "cache reuse honors the native member-return context");
    CafbcrClusterHopTag check;
    NS_TEST_EXPECT_MSG_EQ(packet->PeekPacketTag(check), false, "caller-owned original packet stays unmodified");
    NS_TEST_EXPECT_MSG_EQ(incoming->PeekPacketTag(check), (m_expectedTag != 0xFFFF), "input copy keeps its marker");
    if (m_expectedTag != 0xFFFF)
      NS_TEST_EXPECT_MSG_EQ(check.GetClusterHeadId(), m_expectedTag, "the guard does not rewrite the input marker");
    while (!manager->GetAllClusters().empty()) manager->RemoveCluster(manager->GetAllClusters().begin()->first);
    Simulator::Destroy(); route = nullptr; manager = nullptr;
  }
  Kind m_kind;
  NodeContainer m_nodes;
  Ipv4InterfaceContainer m_interfaces;
  uint64_t m_uid = 0;
  uint16_t m_expectedTag = 0xFFFF;
  uint32_t m_callbacks = 0;
  Ipv4Address m_next;
};

class CafbcrForwardingCacheTestSuite : public TestSuite
{
public:
  CafbcrForwardingCacheTestSuite() : TestSuite("cafbcr-forwarding-cache", UNIT)
  {
    using C = CafbcrForwardingCacheTestCase;
    AddTestCase(new C("marked own-head return bypasses the old member cache", C::OWN_RETURN), TestCase::QUICK);
    AddTestCase(new C("non-Head Idle role retains the existing return guard", C::IDLE_RETURN), TestCase::QUICK);
    AddTestCase(new C("unmarked packets keep valid cache reuse", C::UNMARKED), TestCase::QUICK);
    AddTestCase(new C("foreign-head marker does not block the own-head cache", C::FOREIGN_MARK), TestCase::QUICK);
    AddTestCase(new C("marked return can reuse an alternative cached gateway", C::ALTERNATE_CACHE), TestCase::QUICK);
    AddTestCase(new C("direct delivery to the marked head stays available", C::HEAD_DESTINATION), TestCase::QUICK);
    AddTestCase(new C("same-cluster destinations retain their cache policy", C::SAME_CLUSTER), TestCase::QUICK);
    AddTestCase(new C("expired cache still takes the native fresh path", C::EXPIRED), TestCase::QUICK);
    AddTestCase(new C("task forwarding keeps its existing earlier shortcut", C::TASK), TestCase::QUICK);
    AddTestCase(new C("unassociated nodes do not invent a head relationship", C::NO_ASSOCIATION), TestCase::QUICK);
  }
} g_cafbcrForwardingCacheTestSuite;
} // namespace ns3::cafbcr
