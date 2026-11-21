/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "ns3/cafbcr-routing.h"
#include "../model/cafbcr-forwarding-history.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/csma-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/test.h"
#include "ns3/udp-socket-factory.h"
#include "../../../scratch/ca-fbcr-loop-observer.h"
#include <cstdlib>

namespace ns3::cafbcr {

class CafbcrForwardingHistoryTestCase : public TestCase
{
public:
  enum Kind { WIRE_ID, PAYLOAD, HEADER, SAME_TTL, EXPIRY, ENTRIES, BYTES, OVERSIZE,
              FRAGMENT, TTL, NO_COMMIT, LIFECYCLE };
  CafbcrForwardingHistoryTestCase(const char* name, Kind kind) : TestCase(name), m_kind(kind) {}
private:
  Ipv4Header Header() {
    Ipv4Header h; h.SetSource(Ipv4Address("10.1.1.1")); h.SetDestination(Ipv4Address("10.1.1.2"));
    h.SetProtocol(17); h.SetIdentification(7); h.SetTtl(61); return h;
  }
  Ptr<Packet> PacketWith(uint8_t first = 1) {
    uint8_t bytes[] = {first, 2, 3, 4}; return Create<Packet>(bytes, 4);
  }
  void Keep(Ptr<ForwardingHistory> history, Ptr<Packet> packet, Ipv4Header header) {
    auto c = history->Inspect(packet, header);
    NS_TEST_EXPECT_MSG_EQ(c.revisit, false, "the initial forwarding attempt is not a revisit");
    history->Remember(std::move(c));
  }
  void DoRun() override {
    auto history = CreateObject<ForwardingHistory>(); auto packet = PacketWith(); auto header = Header();
    const uint64_t uid = packet->GetUid();
    if (m_kind == LIFECYCLE) {
      auto route = CreateObject<RoutingProtocol>();
      NS_TEST_EXPECT_MSG_EQ(route->GetTerminateRepeatedForward(), false, "default remains off");
      NS_TEST_EXPECT_MSG_EQ(bool(route->GetForwardingHistory()), false, "off allocates no per-instance history");
      route->SetTerminateRepeatedForward(true); auto old = route->GetForwardingHistory();
      Keep(old, packet, header); route->SetTerminateRepeatedForward(false);
      NS_TEST_EXPECT_MSG_EQ(old->GetStats().entries, 0, "disabled history releases stored payloads");
      route->SetTerminateRepeatedForward(true);
      NS_TEST_EXPECT_MSG_EQ(route->GetForwardingHistory()->GetStats().entries, 0, "re-enable starts with fresh local state");
      route->Dispose();
      NS_TEST_EXPECT_MSG_EQ(bool(route->GetForwardingHistory()), false, "Dispose erases the instance registry entry");
      route = nullptr;
    } else if (m_kind == ENTRIES || m_kind == BYTES) {
      if (m_kind == ENTRIES) history->m_entryLimit = 2;
      else history->m_payloadLimit = 6;
      for (uint16_t i = 1; i <= (m_kind == ENTRIES ? 3 : 2); ++i) {
        header.SetIdentification(i); Keep(history, packet, header);
      }
      auto s = history->GetStats();
      NS_TEST_EXPECT_MSG_EQ(s.evicted, 1, "oldest entry is evicted at the selected budget");
      NS_TEST_EXPECT_MSG_EQ(s.entries, (m_kind == ENTRIES ? 2 : 1), "entry count stays bounded");
      NS_TEST_EXPECT_MSG_EQ(s.payloadBytes, (m_kind == ENTRIES ? 8 : 4), "exact payload byte accounting");
      header.SetIdentification(1); header.SetTtl(59);
      NS_TEST_EXPECT_MSG_EQ(history->Inspect(packet, header).revisit, false, "evicted observations do not cause drops");
    } else if (m_kind == OVERSIZE) {
      history->m_payloadLimit = 2; auto c = history->Inspect(packet, header);
      NS_TEST_EXPECT_MSG_EQ(c.track, false, "oversized payload passes without tracking");
      history->Remember(std::move(c));
      NS_TEST_EXPECT_MSG_EQ(history->GetStats().copiedPayloadBytes, 0, "oversized payload is not copied");
      NS_TEST_EXPECT_MSG_EQ(history->GetStats().entries, 0, "no hidden oversized state");
    } else if (m_kind == NO_COMMIT) {
      auto unused = history->Inspect(packet, header); header.SetTtl(59);
      auto second = history->Inspect(packet, header);
      NS_TEST_EXPECT_MSG_EQ(second.revisit, false, "no route callback means no prior forwarding record");
      NS_TEST_EXPECT_MSG_EQ(history->GetStats().entries, 0, "Inspect alone stores no payload");
      history->Remember(std::move(second)); header.SetTtl(58);
      NS_TEST_EXPECT_MSG_EQ(history->Inspect(packet, header).revisit, true, "the committed attempt is remembered");
    } else {
      Keep(history, packet, header); header.SetTtl(59);
      if (m_kind == WIRE_ID) {
        auto sameBytes = PacketWith();
        NS_TEST_EXPECT_MSG_EQ((sameBytes->GetUid() != uid), true, "fixture has independent ns-3 packet identities");
        NS_TEST_EXPECT_MSG_EQ(history->Inspect(sameBytes, header).revisit, true, "exact wire identity detects return despite different UID");
      } else if (m_kind == PAYLOAD) {
        NS_TEST_EXPECT_MSG_EQ(history->Inspect(PacketWith(9), header).revisit, false, "equal size and IP ID do not merge different bytes");
      } else if (m_kind == HEADER) {
        for (int field = 0; field < 4; ++field) {
          auto other = header;
          if (field == 0) other.SetSource(Ipv4Address("10.1.1.3"));
          if (field == 1) other.SetDestination(Ipv4Address("10.1.1.3"));
          if (field == 2) other.SetProtocol(1);
          if (field == 3) other.SetIdentification(8);
          NS_TEST_EXPECT_MSG_EQ(history->Inspect(packet, other).revisit, false, "each wire header identity field matters");
        }
      } else if (m_kind == SAME_TTL) {
        header.SetTtl(61); Keep(history, packet, header);
        header.SetTtl(63); Keep(history, packet, header); header.SetTtl(62);
        NS_TEST_EXPECT_MSG_EQ(history->Inspect(packet, header).revisit, true, "a later lower TTL is compared with the new higher-TTL observation");
        NS_TEST_EXPECT_MSG_EQ(history->GetStats().sameTtlForwards, 1, "equal TTL passes as a duplicate");
        NS_TEST_EXPECT_MSG_EQ(history->GetStats().higherTtlForwards, 1, "higher TTL is permitted as identity reuse");
      } else if (m_kind == EXPIRY) {
        Simulator::Schedule(Seconds(1) - TimeStep(1), [this, history, packet, header] {
          NS_TEST_EXPECT_MSG_EQ(history->Inspect(packet, header).revisit, true, "history remains valid immediately before expiry");
        });
        Simulator::Schedule(Seconds(1), [this, history, packet, header] {
          NS_TEST_EXPECT_MSG_EQ(history->Inspect(packet, header).revisit, false, "exact expiry is untracked rather than a false loop");
          NS_TEST_EXPECT_MSG_EQ(history->GetStats().expired, 1, "expiry is counted");
        });
        Simulator::Stop(Seconds(1) + TimeStep(1)); Simulator::Run();
      } else if (m_kind == FRAGMENT) {
        auto first = header; first.SetMoreFragments();
        auto later = header; later.SetFragmentOffset(8);
        NS_TEST_EXPECT_MSG_EQ(history->Inspect(packet, first).track, false, "first fragment is explicitly untracked");
        NS_TEST_EXPECT_MSG_EQ(history->Inspect(packet, later).track, false, "later fragment is explicitly untracked");
        NS_TEST_EXPECT_MSG_EQ(history->GetStats().fragmentsUntracked, 2, "fragment unknowns remain visible");
      } else if (m_kind == TTL) {
        header.SetTtl(1); NS_TEST_EXPECT_MSG_EQ(history->Inspect(packet, header).track, false, "native IPv4 handles TTL one");
        header.SetTtl(0); NS_TEST_EXPECT_MSG_EQ(history->Inspect(packet, header).track, false, "no synthetic TTL-zero forwarding");
        NS_TEST_EXPECT_MSG_EQ(history->GetStats().revisitDrops, 0, "native expiry is not relabeled as policy drop");
      }
    }
    uint8_t bytes[4]; packet->CopyData(bytes, 4);
    NS_TEST_EXPECT_MSG_EQ(bytes[0], 1, "policy does not alter packet bytes");
    NS_TEST_EXPECT_MSG_EQ(packet->GetUid(), uid, "policy does not replace packet identity");
    history->Dispose(); Simulator::Destroy();
  }
  Kind m_kind;
};

// Actual IPv4 routing/ARP/CSMA fixture with fixed local neighbor tables. It
// tests termination and delivery, not the physical heterogeneous FANET model.
class CafbcrRevisitNetworkCase : public TestCase
{
public:
  enum Kind { LOOP, DELIVERY, LOCAL };
  CafbcrRevisitNetworkCase(const char* name, const char* slug, bool enabled, Kind kind)
    : TestCase(name), m_slug(slug), m_enabled(enabled), m_kind(kind) {}
private:
  struct Tx { uint32_t node; uint8_t ttl; uint64_t uid; };
  void TxIp(uint32_t node, Ptr<const Packet> packet, Ptr<Ipv4> ipv4, uint32_t interface) {
    if (!interface) return;
    Ipv4Header h; packet->PeekHeader(h);
    if (h.GetProtocol() == 17 && h.GetSource() == m_interfaces.GetAddress(0) && h.GetDestination() == m_interfaces.GetAddress(3))
      m_tx.push_back({node, h.GetTtl(), packet->GetUid()});
  }
  static void TxBound(CafbcrRevisitNetworkCase* self, uint32_t node, Ptr<const Packet> p, Ptr<Ipv4> ip, uint32_t iface)
  { self->TxIp(node, p, ip, iface); }
  void Drop(const Ipv4Header& header, Ptr<const Packet> packet, Ipv4L3Protocol::DropReason reason,
            Ptr<Ipv4> ipv4, uint32_t interface) {
    if (header.GetProtocol() != 17 || header.GetDestination() != m_interfaces.GetAddress(3)) return;
    ++m_drops; m_dropReason = reason; m_dropTtl = header.GetTtl();
  }
  void Receive(Ptr<Socket> socket) {
    while (auto packet = socket->Recv()) {
      ++m_received; NS_TEST_EXPECT_MSG_EQ(packet->GetSize(), 100, "no routing shim or extra wire bytes");
      uint8_t data[100]; packet->CopyData(data, 100);
      for (uint8_t i = 0; i < 100; ++i) NS_TEST_EXPECT_MSG_EQ(data[i], i, "delivered payload is intact");
    }
  }
  void LocalReceive(Ptr<const Packet> packet, const Ipv4Header& h, uint32_t interface) { ++m_local; }
  void RunPacket() {
    for (auto route : m_routes) {
      route->m_helloTimer.Cancel(); route->m_clusteringTimer.Cancel(); route->m_beaconTimer.Cancel();
      route->m_neighbors->Clear();
    }
    auto add = [&](uint32_t local, uint32_t peer) {
      m_routes[local]->m_neighbors->AddNeighbor(Neighbors::NeighborInfo(m_nodes.Get(peer)->GetId(),
        m_interfaces.GetAddress(peer), 0xFFFF, m_nodes.Get(peer)->GetObject<MobilityModel>()->GetPosition(),
        Vector(), 100, .8, Idle, Simulator::Now(), Seconds(10)));
    };
    add(0, 1); add(1, 2); add(2, 1);
    if (m_kind == DELIVERY) add(2, 3);
    if (m_kind == LOCAL) {
      auto packet = Create<Packet>(100); Ipv4Header h; h.SetSource(m_interfaces.GetAddress(0));
      h.SetDestination(m_interfaces.GetAddress(1)); h.SetProtocol(17); h.SetIdentification(7); h.SetTtl(8);
      auto history = m_routes[1]->GetForwardingHistory();
      history->Remember(history->Inspect(packet, h)); h.SetTtl(6);
      auto checks = history->GetStats().checks;
      bool handled = m_routes[1]->RouteInput(packet, h, m_nodes.Get(1)->GetDevice(1),
        Ipv4RoutingProtocol::UnicastForwardCallback(), Ipv4RoutingProtocol::MulticastForwardCallback(),
        MakeCallback(&CafbcrRevisitNetworkCase::LocalReceive, this), Ipv4RoutingProtocol::ErrorCallback());
      NS_TEST_EXPECT_MSG_EQ(handled, true, "destination delivery precedes loop policy");
      NS_TEST_EXPECT_MSG_EQ(m_local, 1, "local delivery is not suppressed");
      NS_TEST_EXPECT_MSG_EQ(history->GetStats().checks, checks, "local delivery does not query transit history");
      return;
    }
    auto socket = Socket::CreateSocket(m_nodes.Get(0), UdpSocketFactory::GetTypeId());
    socket->SetIpTtl(8); socket->Bind(); socket->Connect(InetSocketAddress(m_interfaces.GetAddress(3), 9000));
    uint8_t bytes[100]; for (uint8_t i = 0; i < 100; ++i) bytes[i] = i;
    NS_TEST_EXPECT_MSG_EQ(socket->Send(Create<Packet>(bytes, 100)), 100, "one datagram submitted");
    m_source = socket;
  }
  void DoRun() override {
    auto manager = ClusterManager::GetInstance();
    NS_TEST_ASSERT_MSG_EQ(manager->GetAllClusters().empty(), true, "no hidden cluster state");
    m_nodes.Create(4); InternetStackHelper stack; stack.Install(m_nodes);
    for (uint32_t i = 0; i < 4; ++i) {
      auto node = m_nodes.Get(i); auto cn = CreateObject<ClusterNode>(); cn->SetNodeId(node->GetId());
      cn->SetRole(Idle); cn->SetClusterId(0xFFFF); cn->SetMaxRange(150);
      cn->SetOriginEnergy(100); cn->updateResidualEnergy(100); node->AggregateObject(cn);
      auto motion = CreateObject<ConstantPositionMobilityModel>();
      motion->SetPosition(Vector(i == 0 ? -100 : i == 1 ? 0 : i == 2 ? 100 : m_kind == DELIVERY ? 200 : 300, 0, 0));
      node->AggregateObject(motion);
      auto route = CreateObject<RoutingProtocol>(); route->SetTerminateRepeatedForward(m_enabled);
      node->AggregateObject(route); node->GetObject<Ipv4>()->SetRoutingProtocol(route);
      route->AssignStreams(6000000 + i * 2); m_routes.push_back(route);
    }
    CsmaHelper medium; medium.SetChannelAttribute("DataRate", StringValue("100Mbps"));
    medium.SetChannelAttribute("Delay", TimeValue(MicroSeconds(1)));
    auto devices = medium.Install(m_nodes); Ipv4AddressHelper addresses;
    addresses.SetBase("10.1.1.0", "255.255.255.0"); m_interfaces = addresses.Assign(devices);
    const char* base = std::getenv("CAFBCR_HISTORY_TEST_OUTPUT");
    NS_TEST_ASSERT_MSG_NE(base, nullptr, "bounded runner supplies a new output directory");
    auto directory = std::filesystem::path(base) / m_slug;
    NS_TEST_ASSERT_MSG_EQ(std::filesystem::exists(directory), false, "do not overwrite fixture evidence");
    std::filesystem::create_directories(directory);
    LoopObservationWriter writer(directory.string(), Seconds(.5), m_enabled);
    for (uint32_t i = 0; i < 4; ++i) {
      writer.Attach(i, m_routes[i]); auto ip = m_nodes.Get(i)->GetObject<Ipv4L3Protocol>();
      ip->TraceConnectWithoutContext("Tx", MakeBoundCallback(&CafbcrRevisitNetworkCase::TxBound, this, i));
      ip->TraceConnectWithoutContext("Drop", MakeCallback(&CafbcrRevisitNetworkCase::Drop, this));
    }
    m_sink = Socket::CreateSocket(m_nodes.Get(3), UdpSocketFactory::GetTypeId());
    m_sink->Bind(InetSocketAddress(Ipv4Address::GetAny(), 9000));
    m_sink->SetRecvCallback(MakeCallback(&CafbcrRevisitNetworkCase::Receive, this));
    Simulator::Schedule(TimeStep(1), [this] {
      for (auto route : m_routes) {
        route->m_helloTimer.Cancel(); route->m_clusteringTimer.Cancel(); route->m_beaconTimer.Cancel();
      }
    });
    Simulator::Schedule(MilliSeconds(100), &CafbcrRevisitNetworkCase::RunPacket, this);
    Simulator::Stop(Seconds(.5)); Simulator::Run(); writer.Finalize();
    if (m_kind == LOOP) {
      NS_TEST_EXPECT_MSG_EQ(m_received, 0, "unreachable destination is not relabeled success");
      NS_TEST_EXPECT_MSG_EQ(m_drops, 1, "native IPv4 observes exactly one data drop");
      NS_TEST_EXPECT_MSG_EQ(m_tx.size(), (m_enabled ? 3 : 8), "policy stops the revisit before the TTL limit");
      NS_TEST_EXPECT_MSG_EQ(m_dropReason, (m_enabled ? Ipv4L3Protocol::DROP_NO_ROUTE : Ipv4L3Protocol::DROP_TTL_EXPIRED), "actual native drop reason");
      NS_TEST_EXPECT_MSG_EQ(unsigned(m_dropTtl), (m_enabled ? 6 : 1), "policy and TTL expiry have different received TTLs");
    } else if (m_kind == DELIVERY) {
      NS_TEST_EXPECT_MSG_EQ(m_received, 1, "acyclic route still delivers");
      NS_TEST_EXPECT_MSG_EQ(m_tx.size(), 3, "three physical IP transmissions");
      NS_TEST_EXPECT_MSG_EQ(m_drops, 0, "no false drop on an acyclic path");
    }
    std::ofstream fixture(directory / "fixture.json");
    fixture << "{\"enabled\":" << (m_enabled ? "true" : "false") << ",\"kind\":" << m_kind
      << ",\"received\":" << m_received << ",\"drops\":" << m_drops
      << ",\"drop_reason\":" << int(m_dropReason) << ",\"drop_ttl\":" << unsigned(m_dropTtl) << ",\"tx\":[";
    for (size_t i = 0; i < m_tx.size(); ++i) { if (i) fixture << ',';
      fixture << "{\"node\":" << m_tx[i].node << ",\"ttl\":" << unsigned(m_tx[i].ttl) << ",\"uid\":" << m_tx[i].uid << '}'; }
    fixture << "]}\n"; fixture.close();
    Simulator::Destroy(); m_routes.clear(); m_source = nullptr; m_sink = nullptr;
  }
  const char* m_slug; bool m_enabled; Kind m_kind;
  NodeContainer m_nodes; Ipv4InterfaceContainer m_interfaces;
  std::vector<Ptr<RoutingProtocol>> m_routes;
  Ptr<Socket> m_source, m_sink;
  std::vector<Tx> m_tx;
  uint32_t m_received = 0, m_drops = 0, m_local = 0;
  uint8_t m_dropTtl = 0;
  Ipv4L3Protocol::DropReason m_dropReason = Ipv4L3Protocol::DROP_NO_ROUTE;
};

class CafbcrForwardingHistorySuite : public TestSuite
{
public:
  CafbcrForwardingHistorySuite() : TestSuite("cafbcr-forwarding-history", UNIT) {
    using C = CafbcrForwardingHistoryTestCase;
    AddTestCase(new C("wire identity excludes simulation UID", C::WIRE_ID), TestCase::QUICK);
    AddTestCase(new C("different full payload is not merged", C::PAYLOAD), TestCase::QUICK);
    AddTestCase(new C("wire header tuple fields remain distinct", C::HEADER), TestCase::QUICK);
    AddTestCase(new C("same and higher TTL are permitted", C::SAME_TTL), TestCase::QUICK);
    AddTestCase(new C("one-second expiry has an exact boundary", C::EXPIRY), TestCase::QUICK);
    AddTestCase(new C("entry budget evicts oldest observation", C::ENTRIES), TestCase::QUICK);
    AddTestCase(new C("payload byte budget bounds stored data", C::BYTES), TestCase::QUICK);
    AddTestCase(new C("oversized datagram stays explicitly untracked", C::OVERSIZE), TestCase::QUICK);
    AddTestCase(new C("fragment handling remains native and untracked", C::FRAGMENT), TestCase::QUICK);
    AddTestCase(new C("TTL expiry remains owned by IPv4", C::TTL), TestCase::QUICK);
    AddTestCase(new C("unsent candidate does not become forwarding history", C::NO_COMMIT), TestCase::QUICK);
    AddTestCase(new C("default off and instance disposal isolate state", C::LIFECYCLE), TestCase::QUICK);
    using N = CafbcrRevisitNetworkCase;
    AddTestCase(new N("native cyclic path terminates at TTL with policy off", "loop-off", false, N::LOOP), TestCase::QUICK);
    AddTestCase(new N("policy stops cyclic path at first repeated forwarding node", "loop-on", true, N::LOOP), TestCase::QUICK);
    AddTestCase(new N("native acyclic route delivers with policy off", "delivery-off", false, N::DELIVERY), TestCase::QUICK);
    AddTestCase(new N("acyclic route payload survives with policy on", "delivery-on", true, N::DELIVERY), TestCase::QUICK);
    AddTestCase(new N("local delivery takes precedence over transit history", "local-delivery", true, N::LOCAL), TestCase::QUICK);
  }
} g_cafbcrForwardingHistorySuite;
} // namespace ns3::cafbcr
