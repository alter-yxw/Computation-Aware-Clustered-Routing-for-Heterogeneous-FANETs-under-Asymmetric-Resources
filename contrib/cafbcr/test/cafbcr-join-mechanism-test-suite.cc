/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "ns3/cafbcr-routing.h"
#include "ns3/boolean.h"
#include "ns3/basic-energy-source-helper.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/csma-helper.h"
#include "ns3/error-model.h"
#include "ns3/ethernet-header.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/test.h"
#include "../../../scratch/ca-fbcr-join-observer.h"
#include <chrono>
#include <array>
#include <cstdlib>

namespace ns3::cafbcr {

class JoinDropErrorModel : public ErrorModel
{
public:
  static TypeId GetTypeId()
  {
    static TypeId tid = TypeId("ns3::cafbcr::JoinDropErrorModel").SetParent<ErrorModel>()
      .AddConstructor<JoinDropErrorModel>();
    return tid;
  }
  bool dropAll = false;
  std::optional<Mac48Address> blockedSource;
  uint32_t drops = 0;
private:
  bool DoCorrupt(Ptr<Packet> packet) override
  {
    bool drop = dropAll;
    if (!drop && blockedSource && packet->GetSize() >= 14) {
      EthernetHeader header(false); packet->PeekHeader(header);
      drop = header.GetSource() == *blockedSource;
    }
    if (drop) ++drops;
    return drop;
  }
  void DoReset() override { drops = 0; }
};

// Deterministic control-plane fixture using point-to-point links or a filtered
// CSMA medium. It tests UDP handlers, not heterogeneous Wi-Fi performance.
class CafbcrJoinMechanismTestCase : public TestCase
{
public:
  enum Kind { NORMAL, LOST_GRANT, DISABLED_GUARDS, RELAY, NO_RELAY, ROLLING, MAINTENANCE, GRANT_ELIGIBILITY, PROXY_QUALIFICATION, HEAD_REQUEST_RACE, HEAD_RESPONSE_RACE, EXPIRY_OWNERSHIP, OWN_CLUSTER, GRANT_ADMISSION_RACE };
  CafbcrJoinMechanismTestCase(const char* name, bool proxy, bool range,
                            bool master = true, uint32_t selector = 0, Kind kind = NORMAL)
    : TestCase(name), m_proxy(proxy), m_range(range), m_master(master), m_selector(selector), m_kind(kind) {}
private:
  void Observe(const JoinMechanismEvent& e) { m_events.push_back(e); }
  uint32_t Count(const char* event, uint32_t node) const
  {
    return std::count_if(m_events.begin(), m_events.end(), [event, node](const auto& e) {
      return e.event == event && e.observerNodeId == node;
    });
  }
  template<class H> Ptr<Packet> PacketWith(const H& header)
  { auto p = Create<Packet>(); p->AddHeader(header); return p; }
  void InjectProxy(uint16_t origin)
  {
    m_routes[1]->RecvProxyJoinRequest(PacketWith(JoinClusterRequestHeader(origin,
      Ipv4Address(0x0a010100 + origin + 1), Vector(), Vector(), 100, .8, Idle,
      Simulator::Now().GetSeconds())), m_interfaces.GetAddress(1), m_interfaces.GetAddress(0));
  }
  void RunExpiryOwnership()
  {
    // Invoke the real CA maintenance handler with deterministic registry state.
    // These are state/timeout tests, not wireless delivery experiments.
    m_manager = ClusterManager::GetInstance();
    NS_TEST_ASSERT_MSG_EQ(m_manager->GetAllClusters().empty(), true, "fixture starts without clusters");
    m_nodes.Create(4);
    InternetStackHelper stack; stack.Install(m_nodes);
    for (auto it = m_nodes.Begin(); it != m_nodes.End(); ++it) {
      auto cn = CreateObject<ClusterNode>(); cn->SetNodeId((*it)->GetId());
      cn->SetRole(Idle); cn->SetClusterId(0xFFFF);
      cn->SetOriginEnergy(100); cn->updateResidualEnergy(100); cn->SetMaxRange(100);
      (*it)->AggregateObject(cn);
      (*it)->AggregateObject(CreateObject<ConstantPositionMobilityModel>());
    }
    auto route = CreateObject<RoutingProtocol>();
    route->SetIpv4(m_nodes.Get(1)->GetObject<Ipv4>()); m_nodes.Get(1)->AggregateObject(route);
    route->SetBeaconInterval(MilliSeconds(10));
    route->m_beaconTimer.SetFunction(&RoutingProtocol::BeaconTimerExpire, PeekPointer(route));
    route->TraceConnectWithoutContext("JoinMechanism", MakeCallback(&CafbcrJoinMechanismTestCase::Observe, this));
    auto subject = m_nodes.Get(0)->GetObject<ClusterNode>();
    auto head = m_nodes.Get(1)->GetObject<ClusterNode>();
    auto keeper = m_nodes.Get(2)->GetObject<ClusterNode>();
    const auto oldId = m_manager->ArrangeClusterId();
    m_manager->CreateCluster(oldId, head);
    m_manager->JoinCluster(oldId, subject); subject->SetRole(Member);
    m_manager->JoinCluster(oldId, keeper); keeper->SetRole(Member);
    auto old = m_manager->GetCluster(oldId);
    if (m_selector == 0 || m_selector == 5 || m_selector == 6) {
      m_manager->CreateCluster(m_manager->ArrangeClusterId(), subject);
    } else if (m_selector == 1) {
      auto other = m_nodes.Get(3)->GetObject<ClusterNode>();
      const auto newId = m_manager->ArrangeClusterId();
      m_manager->CreateCluster(newId, other);
      m_manager->JoinCluster(newId, subject);
    } else if (m_selector == 2) {
      subject->SetClusterId(0xFFFF); subject->SetRole(Idle);
    }
    const auto currentId = subject->GetClusterId();
    const auto currentRole = subject->GetRoleType();
    Simulator::Schedule(MilliSeconds(250), [this, route, old, subject, keeper, oldId, currentId, currentRole] {
      old->UpdateMemberTime(keeper->GetNodeId(), Simulator::Now());
      if (m_selector == 5 || m_selector == 6) {
        // The existing API retains its exact state-mutating semantics; the
        // new API removes both maps and preserves the current association.
        if (m_selector == 5) old->RemoveMember(subject);
        else old->RemoveMemberRecord(subject->GetNodeId());
        NS_TEST_EXPECT_MSG_EQ(subject->GetClusterId(), (m_selector == 5 ? 0xFFFF : currentId),
                              "legacy and opt-in deletion contracts remain distinct");
        NS_TEST_EXPECT_MSG_EQ(subject->GetRoleType(), currentRole, "neither API changes the role");
      } else {
        if (m_selector == 4) old->UpdateMemberTime(subject->GetNodeId(), Simulator::Now());
        route->BeaconTimerExpire();
        const bool ownedExpiry = m_selector == 3;
        NS_TEST_EXPECT_MSG_EQ(subject->GetClusterId(), (ownedExpiry ? 0xFFFF : currentId),
                              "old expiry clears only its own current association");
        NS_TEST_EXPECT_MSG_EQ(subject->GetRoleType(), (ownedExpiry ? Idle : currentRole),
                              "old expiry preserves newer Head, Member, or Idle state");
        NS_TEST_EXPECT_MSG_EQ(Count("stale_member_record_expired", m_nodes.Get(1)->GetId()),
                              (m_selector < 3 ? 1 : 0), "only stale registrations use bookkeeping-only expiry");
        NS_TEST_EXPECT_MSG_EQ(Count("member_expired_at_head", m_nodes.Get(1)->GetId()),
                              (ownedExpiry ? 1 : 0), "owned membership still expires normally");
      }
      const bool retained = m_selector == 4;
      NS_TEST_EXPECT_MSG_EQ(old->ExistMember(subject->GetNodeId()), retained, "member map retires the old registration");
      NS_TEST_EXPECT_MSG_EQ(old->GetAllMembersStamp().count(subject->GetNodeId()), (retained ? 1 : 0),
                            "timestamp map retires the same registration");
      NS_TEST_EXPECT_MSG_EQ(old->ExistMember(keeper->GetNodeId()), true, "other members remain registered");
      NS_TEST_EXPECT_MSG_EQ(old->GetMemberUpdateTime(keeper->GetNodeId()), Simulator::Now(),
                            "other member timestamps remain intact");
      if (m_selector == 0 || m_selector == 1 || m_selector == 6) {
        NS_TEST_EXPECT_MSG_EQ(m_manager->GetCluster(currentId)->ExistMember(subject->GetNodeId()), true,
                              "new cluster keeps its membership");
      }
    });
    Simulator::Stop(MilliSeconds(250) + TimeStep(1)); Simulator::Run();
    while (!m_manager->GetAllClusters().empty()) m_manager->RemoveCluster(m_manager->GetAllClusters().begin()->first);
    Simulator::Destroy(); m_manager = nullptr;
  }
  void RunOwnCluster()
  {
    // Admission uses real point-to-point UDP; maintenance/recovery invokes the
    // actual timer handler at a fixed time. Neither is a Wi-Fi performance run.
    m_manager = ClusterManager::GetInstance();
    NS_TEST_ASSERT_MSG_EQ(m_manager->GetAllClusters().empty(), true, "fixture starts without clusters");
    m_nodes.Create(4);
    InternetStackHelper stack; stack.Install(m_nodes);
    for (uint32_t i = 0; i < m_nodes.GetN(); ++i) {
      auto node = m_nodes.Get(i);
      auto cn = CreateObject<ClusterNode>(); cn->SetNodeId(node->GetId());
      cn->SetRole(Idle); cn->SetClusterId(0xFFFF); cn->SetMaxRange(100);
      cn->SetOriginEnergy(100); cn->updateResidualEnergy(100); cn->UpdateFitness(.8);
      node->AggregateObject(cn);
      auto motion = CreateObject<ConstantPositionMobilityModel>();
      motion->SetPosition(Vector(i * 10, 0, 0)); node->AggregateObject(motion);
      if (i >= 2) continue;
      auto route = CreateObject<RoutingProtocol>();
      route->SetAttribute("EnableProxyJoin", BooleanValue(m_proxy));
      route->SetIpv4(node->GetObject<Ipv4>()); node->AggregateObject(route);
      route->AssignStreams(2000000 + i * 2); m_routes.push_back(route);
      route->TraceConnectWithoutContext("JoinMechanism", MakeCallback(&CafbcrJoinMechanismTestCase::Observe, this));
    }
    BasicEnergySourceHelper energy; energy.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(100)); energy.Install(m_nodes);
    PointToPointHelper link; link.SetDeviceAttribute("DataRate", StringValue("10Mbps"));
    link.SetChannelAttribute("Delay", TimeValue(MilliSeconds(1)));
    auto devices = link.Install(m_nodes.Get(0), m_nodes.Get(1));
    Ipv4AddressHelper addresses; addresses.SetBase("10.1.1.0", "255.255.255.0");
    m_interfaces = addresses.Assign(devices);
    // A malformed old response can direct automatic CM_LIVE at the foreign
    // Head. Give that bookkeeping node a real interface so the counterexample
    // reports the wrong association rather than crashing an incomplete fixture.
    auto otherDevices = link.Install(m_nodes.Get(2), m_nodes.Get(3));
    addresses.SetBase("10.1.2.0", "255.255.255.0"); addresses.Assign(otherDevices);
    for (auto route : m_routes) route->NotifyInterfaceUp(1);
    Simulator::Schedule(TimeStep(1), [this] {
      for (auto route : m_routes) {
        route->m_helloTimer.Cancel(); route->m_clusteringTimer.Cancel(); route->m_beaconTimer.Cancel();
      }
    });
    auto receiver = m_nodes.Get(1)->GetObject<ClusterNode>();
    auto keeper = m_nodes.Get(2)->GetObject<ClusterNode>();
    auto other = m_nodes.Get(3)->GetObject<ClusterNode>();
    const bool staleHead = m_selector >= 3 && m_selector <= 8;
    const auto oldId = m_manager->ArrangeClusterId();
    m_manager->CreateCluster(oldId, staleHead ? receiver : other);
    m_manager->JoinCluster(oldId, keeper); keeper->SetRole(Member);
    if (!staleHead) m_manager->JoinCluster(oldId, receiver);
    if (m_selector <= 2 || m_selector == 9) {
      m_raceClusterId = m_manager->ArrangeClusterId();
      m_manager->CreateCluster(m_raceClusterId, receiver);
      if (m_selector == 0) {
        auto subject = m_nodes.Get(0)->GetObject<ClusterNode>();
        m_manager->JoinCluster(m_raceClusterId, subject); subject->SetRole(Member);
      } else if (m_selector == 9) {
        m_manager->GetCluster(m_raceClusterId)->RemoveMemberRecord(receiver->GetNodeId());
      }
    } else if (m_selector == 5 || m_selector == 7 || m_selector == 8) {
      m_raceClusterId = m_manager->ArrangeClusterId();
      m_manager->CreateCluster(m_raceClusterId, other);
      m_manager->JoinCluster(m_raceClusterId, receiver);
      receiver->SetRole(m_selector == 5 ? Head : BackupHead);
    } else {
      m_raceClusterId = 0xFFFF;
      receiver->SetClusterId(0xFFFF);
      receiver->SetRole(m_selector == 6 ? Head : BackupHead);
    }
    const bool maintenance = m_selector == 0 || m_selector == 5 || m_selector == 6 || m_selector == 9;
    const auto oldMembers = m_manager->GetCluster(oldId)->GetAllMembers();
    const auto oldStamps = m_manager->GetCluster(oldId)->GetAllMembersStamp();
    if (maintenance) {
      m_routes[1]->SetBeaconInterval(MilliSeconds(10));
      m_routes[1]->m_beaconTimer.SetFunction(&RoutingProtocol::BeaconTimerExpire, PeekPointer(m_routes[1]));
      Simulator::Schedule(MilliSeconds(250), [this, receiver, keeper, other, oldId, oldMembers, oldStamps] {
        m_routes[1]->BeaconTimerExpire();
        NS_TEST_EXPECT_MSG_EQ((m_manager->GetCluster(oldId)->GetAllMembers() == oldMembers), true,
                              "maintenance never deletes registrations in an older cluster");
        NS_TEST_EXPECT_MSG_EQ((m_manager->GetCluster(oldId)->GetAllMembersStamp() == oldStamps), true,
                              "maintenance never edits timestamps in an older cluster");
        NS_TEST_EXPECT_MSG_EQ(keeper->GetRoleType(), Member, "the older cluster's actual member is untouched");
        NS_TEST_EXPECT_MSG_EQ(keeper->GetClusterId(), oldId, "the older member retains its association");
        if (m_selector == 0) {
          NS_TEST_EXPECT_MSG_EQ(receiver->GetRoleType(), Head, "valid current ownership remains a Head");
          NS_TEST_EXPECT_MSG_EQ(receiver->GetClusterId(), m_raceClusterId, "valid Head retains its current cluster");
          NS_TEST_EXPECT_MSG_EQ(other->GetRoleType(), Head, "old cluster's actual Head is untouched");
          auto subject = m_nodes.Get(0)->GetObject<ClusterNode>();
          NS_TEST_EXPECT_MSG_EQ(subject->GetRoleType(), Idle, "a truly expired current member is still removed");
          NS_TEST_EXPECT_MSG_EQ(subject->GetClusterId(), 0xFFFF, "normal current expiry clears the owned association");
          NS_TEST_EXPECT_MSG_EQ(m_manager->GetCluster(m_raceClusterId)->ExistMember(subject->GetNodeId()), false,
                                "current membership table is actually maintained");
        } else {
          NS_TEST_EXPECT_MSG_EQ(receiver->GetRoleType(), Idle, "invalid Head follows the existing Idle recovery policy");
          NS_TEST_EXPECT_MSG_EQ(receiver->GetClusterId(), 0xFFFF, "invalid ownership is not restored from an old registration");
          NS_TEST_EXPECT_MSG_EQ(m_routes[1]->m_isRejoinScheduled, true, "invalid ownership schedules recovery");
          NS_TEST_EXPECT_MSG_EQ(Count("head_cluster_invalid", receiver->GetNodeId()), 1, "invalid ownership is traced");
          if (m_selector == 5) {
            NS_TEST_EXPECT_MSG_EQ(other->GetRoleType(), Head, "the current foreign cluster's actual Head is untouched");
            NS_TEST_EXPECT_MSG_EQ(other->GetClusterId(), m_raceClusterId, "foreign Head keeps its association");
            NS_TEST_EXPECT_MSG_EQ(m_manager->GetCluster(m_raceClusterId)->ExistMember(receiver->GetNodeId()), true,
                                  "recovery does not dissolve or edit the foreign cluster");
          }
        }
      });
    } else {
      Simulator::Schedule(MilliSeconds(10), [this, receiver] {
        m_routes[0]->m_neighbors->AddNeighbor(Neighbors::NeighborInfo(receiver->GetNodeId(),
          m_interfaces.GetAddress(1), receiver->GetClusterId(), Vector(10,0,0), Vector(),
          100, .8, receiver->GetRoleType(), Simulator::Now(), Seconds(10)));
        if (m_proxy) {
          m_routes[0]->m_intentJoinChId = receiver->GetNodeId();
          m_routes[0]->SendProxyJoinRequest(m_nodes.Get(0)->GetId(), m_interfaces.GetAddress(0),
            Vector(), Vector(), 100, .8, m_interfaces.GetAddress(1));
        } else {
          m_routes[0]->SendClusterJoinRequest(m_interfaces.GetAddress(1));
        }
      });
      Simulator::Schedule(MilliSeconds(80), &RoutingProtocol::SendClusterHeadBeacon, m_routes[1]);
      Simulator::Schedule(MilliSeconds(220), [this, receiver, keeper, oldId, oldMembers, oldStamps] {
        auto origin = m_nodes.Get(0)->GetObject<ClusterNode>();
        const auto cid = receiver->GetClusterId();
        NS_TEST_EXPECT_MSG_EQ(receiver->GetRoleType(), Head, "successful admission establishes the receiver as Head");
        NS_TEST_EXPECT_MSG_NE(cid, 0xFFFF, "admission must establish a valid current cluster");
        NS_TEST_EXPECT_MSG_NE(cid, oldId, "an older membership/head registration is not an admission target");
        if (m_selector == 1 || m_selector == 2) {
          NS_TEST_EXPECT_MSG_EQ(cid, m_raceClusterId, "a valid current cluster is reused");
        } else {
          NS_TEST_EXPECT_MSG_NE(cid, m_raceClusterId, "BackupHead creates its own cluster instead of reusing a foreign one");
        }
        NS_TEST_EXPECT_MSG_EQ(origin->GetRoleType(), Member, "the real response/grant reaches the origin");
        NS_TEST_EXPECT_MSG_EQ(origin->GetClusterId(), cid, "response, admission and Head use the same cluster ID");
        auto current = m_manager->GetCluster(cid);
        NS_TEST_ASSERT_MSG_NE(current, nullptr, "current cluster exists");
        if (!current) return; // Test runners may continue after ASSERT failures.
        NS_TEST_EXPECT_MSG_EQ(current->GetClusterHeadId(), receiver->GetNodeId(), "the receiver actually heads the admitted cluster");
        NS_TEST_EXPECT_MSG_EQ(current->ExistMember(origin->GetNodeId()), true, "source confirmation matches CH registration");
        NS_TEST_EXPECT_MSG_EQ((m_manager->GetCluster(oldId)->GetAllMembers() == oldMembers), true,
                              "new admission leaves older cluster registration intact");
        NS_TEST_EXPECT_MSG_EQ((m_manager->GetCluster(oldId)->GetAllMembersStamp() == oldStamps), true,
                              "new admission leaves older cluster timestamps intact");
        NS_TEST_EXPECT_MSG_EQ(keeper->GetClusterId(), oldId, "an unrelated older member is not migrated");
        NS_TEST_EXPECT_MSG_EQ(Count(m_proxy ? "proxy_request_rx" : "ordinary_request_rx", receiver->GetNodeId()), 1,
                              "one request was delivered through real UDP");
        NS_TEST_EXPECT_MSG_EQ(Count(m_proxy ? "proxy_confirmed_at_source" : "ordinary_confirmed_at_source", origin->GetNodeId()), 1,
                              "one received response/grant confirms the correct cluster");
        for (const auto& e : m_events) {
          if (e.event == (m_proxy ? "proxy_admitted_at_head" : "ordinary_admitted_at_head")) {
            NS_TEST_EXPECT_MSG_EQ(e.clusterId, cid, "admission event uses the current owned cluster");
          }
        }
      });
    }
    Simulator::Stop(maintenance ? MilliSeconds(250) + TimeStep(1) : MilliSeconds(240)); Simulator::Run();
    while (!m_manager->GetAllClusters().empty()) m_manager->RemoveCluster(m_manager->GetAllClusters().begin()->first);
    Simulator::Destroy(); m_routes.clear(); m_manager = nullptr;
  }
  void RunGrantAdmissionRace()
  {
    // Handler-only reproduction of the Run3 ordering: target admission/grant,
    // a delayed admission elsewhere, then reception of the first grant. CA
    // sockets/election are not installed; the addresses merely make existing
    // scheduled CM handlers safe. No wireless or business sample is produced.
    m_manager = ClusterManager::GetInstance();
    NS_TEST_ASSERT_MSG_EQ(m_manager->GetAllClusters().empty(), true, "clean registry");
    m_nodes.Create(3);
    InternetStackHelper stack; stack.Install(m_nodes);
    for (uint32_t i = 0; i < m_nodes.GetN(); ++i) {
      auto state = CreateObject<ClusterNode>(); state->SetNodeId(m_nodes.Get(i)->GetId());
      state->SetRole(i ? Head : Idle); state->SetClusterId(0xFFFF);
      state->SetMaxRange(100); state->SetOriginEnergy(100); state->updateResidualEnergy(100);
      m_nodes.Get(i)->AggregateObject(state);
      auto motion = CreateObject<ConstantPositionMobilityModel>(); motion->SetPosition(Vector(i * 10, 0, 0));
      m_nodes.Get(i)->AggregateObject(motion);
      auto route = CreateObject<RoutingProtocol>(); route->SetIpv4(m_nodes.Get(i)->GetObject<Ipv4>());
      route->SetAttribute("DirectMaintenanceFeedback", BooleanValue(m_selector == 1));
      route->AssignStreams(2300000 + i * 2); m_routes.push_back(route);
      route->TraceConnectWithoutContext("JoinMechanism", MakeCallback(&CafbcrJoinMechanismTestCase::Observe, this));
    }
    CsmaHelper link; auto devices = link.Install(m_nodes);
    Ipv4AddressHelper addresses; addresses.SetBase("10.57.1.0", "255.255.255.0");
    m_interfaces = addresses.Assign(devices);
    const auto origin = m_nodes.Get(0)->GetObject<ClusterNode>();
    const auto oldHead = m_nodes.Get(1)->GetObject<ClusterNode>();
    const auto targetHead = m_nodes.Get(2)->GetObject<ClusterNode>();
    const auto oldId = m_manager->ArrangeClusterId(); m_manager->CreateCluster(oldId, oldHead);
    const auto targetId = m_manager->ArrangeClusterId(); m_manager->CreateCluster(targetId, targetHead);
    m_routes[0]->m_intentJoinChId = targetHead->GetNodeId();
    const char* parent = std::getenv("CAFBCR_JOIN_TEST_OUTPUT");
    const auto path = parent ? std::filesystem::path(parent) / ("grant-race-" + std::to_string(m_selector)) :
      std::filesystem::temp_directory_path() / ("cafbcr-grant-race-" + std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(path);
    m_writer = std::make_unique<JoinObservationWriter>(path.string(), MilliSeconds(10));
    for (auto route : m_routes) m_writer->Attach(route);
    auto request = [&](uint32_t head) {
      JoinClusterRequestHeader header(origin->GetNodeId(), m_interfaces.GetAddress(0), Vector(), Vector(),
        100, .8, Idle, Simulator::Now().GetSeconds());
      m_routes[head]->RecvProxyJoinRequest(PacketWith(header), m_interfaces.GetAddress(head), Ipv4Address("10.57.1.9"));
    };
    auto grant = [&] {
      ClusterHeadBeaconHeader header(targetHead->GetNodeId(), targetId, m_interfaces.GetAddress(2),
        Vector(20,0,0), Vector(), .8, Simulator::Now().GetSeconds());
      header.SetGrantedMember(0, origin->GetNodeId());
      return PacketWith(header);
    };
    Ptr<Packet> inFlight;
    Simulator::Schedule(MilliSeconds(1), [&] {
      request(2); inFlight = grant();
      NS_TEST_EXPECT_MSG_EQ(m_manager->GetCluster(targetId)->ExistMember(origin->GetNodeId()), true,
                            "target admission exists before its grant is sent");
      NS_TEST_EXPECT_MSG_EQ(origin->GetRoleType(), Idle, "admission alone cannot confirm the source");
    });
    Simulator::Schedule(MilliSeconds(2), [&] {
      if (m_selector <= 1) request(1);
      if (m_selector == 2) m_manager->LeaveCluster(origin->GetNodeId(), targetId);
    });
    Simulator::Schedule(MilliSeconds(3), [&] {
      const auto before = origin->GetClusterId();
      m_routes[0]->RecvClusterHeadBeacon(inFlight, m_interfaces.GetAddress(0), m_interfaces.GetAddress(2));
      if (m_selector == 3) {
        NS_TEST_EXPECT_MSG_EQ(origin->GetRoleType(), Member, "matching admission still confirms");
        return;
      }
      NS_TEST_EXPECT_MSG_EQ(origin->GetRoleType(), Idle,
                            "in-flight grant cannot confirm an admission that was replaced or removed");
      NS_TEST_EXPECT_MSG_EQ(origin->GetClusterId(), before,
                            "in-flight grant cannot overwrite the current local admission");
      NS_TEST_EXPECT_MSG_EQ(m_routes[0]->m_proxyMembership.has_value(), false,
                            "rejected grant cannot create a phantom proxy binding");
      NS_TEST_EXPECT_MSG_EQ(m_routes[0]->m_intentJoinChId, targetHead->GetNodeId(),
                            "the source retains its intended CH for the existing retry path");
      NS_TEST_EXPECT_MSG_EQ(Count("grant_ignored_admission", origin->GetNodeId()), 1,
                            "the local admission conflict is an explicit receive fact");
    });
    Simulator::Schedule(MilliSeconds(4), [&] {
      if (m_selector != 3) request(2);
    });
    Simulator::Schedule(MilliSeconds(5), [&] {
      if (m_selector != 3)
        m_routes[0]->RecvClusterHeadBeacon(grant(), m_interfaces.GetAddress(0), m_interfaces.GetAddress(2));
      NS_TEST_EXPECT_MSG_EQ(origin->GetRoleType(), Member, "a fresh matching retry confirms");
      NS_TEST_EXPECT_MSG_EQ(origin->GetClusterId(), targetId, "confirmed local association matches target");
      NS_TEST_EXPECT_MSG_EQ(m_manager->GetCluster(targetId)->ExistMember(origin->GetNodeId()), true,
                            "confirmation retains the real target registration");
      NS_TEST_EXPECT_MSG_EQ(m_manager->GetCluster(oldId)->ExistMember(origin->GetNodeId()), false,
                            "retry retires the competing registration");
      NS_TEST_EXPECT_MSG_EQ(Count("proxy_confirmed_at_source", origin->GetNodeId()), 1,
                            "exactly one coherent source confirmation");
    });
    Simulator::Schedule(MilliSeconds(6), [&] {
      ClusterMemberBeaconHeader cm(origin->GetNodeId(), targetId, m_interfaces.GetAddress(0),
        Vector(), Vector(), .8, .0055);
      m_routes[2]->RecvClusterMemberBeacon(PacketWith(cm), m_interfaces.GetAddress(2), Ipv4Address("10.57.1.9"));
      NS_TEST_EXPECT_MSG_EQ(Count("proxy_cm_live_refresh_at_head", targetHead->GetNodeId()), 1,
                            "coherent retry permits real proxy maintenance without weakening rejection guards");
      NS_TEST_EXPECT_MSG_EQ(m_manager->GetCluster(targetId)->GetMemberUpdateTime(origin->GetNodeId()), MicroSeconds(5500),
                            "proxy renewal advances the registered source time");
    });
    Simulator::Schedule(MilliSeconds(7), [&] {
      ClusterMemberBeaconHeader cm(origin->GetNodeId(), targetId, m_interfaces.GetAddress(0),
        Vector(), Vector(), .8, .0065);
      m_routes[2]->RecvClusterMemberBeacon(PacketWith(cm), m_interfaces.GetAddress(2), m_interfaces.GetAddress(0));
      NS_TEST_EXPECT_MSG_EQ(Count("cm_live_refresh_at_head", targetHead->GetNodeId()), 1,
                            "direct maintenance also remains available after a coherent retry");
    });
    Simulator::Stop(MilliSeconds(8)); Simulator::Run();
    m_writer->Finalize(); m_writer.reset();
    for (auto route : m_routes) route->Dispose();
    while (!m_manager->GetAllClusters().empty()) m_manager->RemoveCluster(m_manager->GetAllClusters().begin()->first);
    Simulator::Destroy(); m_routes.clear(); m_manager = nullptr;
  }
  void DoRun() override
  {
    if (m_kind == EXPIRY_OWNERSHIP) { RunExpiryOwnership(); return; }
    if (m_kind == OWN_CLUSTER) { RunOwnCluster(); return; }
    if (m_kind == GRANT_ADMISSION_RACE) { RunGrantAdmissionRace(); return; }
    m_manager = ClusterManager::GetInstance();
    NS_TEST_ASSERT_MSG_EQ(m_manager->GetAllClusters().empty(), true, "fixture starts without clusters");
    m_nodes.Create(m_kind == ROLLING ? 10 : m_kind == RELAY || m_kind == NO_RELAY || m_kind == PROXY_QUALIFICATION ? 3 : 2);
    InternetStackHelper stack; stack.Install(m_nodes);
    for (uint32_t i = 0; i < m_nodes.GetN(); ++i) {
      auto node = m_nodes.Get(i);
      auto cn = CreateObject<ClusterNode>(); cn->SetNodeId(node->GetId());
      node->AggregateObject(cn); cn->SetRole(i == 1 ? Head : Idle); cn->SetClusterId(0xFFFF);
      cn->SetMaxRange(100); cn->UpdateFitness(.8);
      cn->SetOriginEnergy(100); cn->updateResidualEnergy(100);
      auto motion = CreateObject<ConstantPositionMobilityModel>();
      node->AggregateObject(motion); motion->SetPosition(Vector(i * 10, 0, 0));
      if ((m_kind == ROLLING || m_kind == PROXY_QUALIFICATION) && i >= 2) continue;
      auto route = CreateObject<RoutingProtocol>();
      route->SetAttribute("EnableAsymHandling", BooleanValue(m_master));
      route->SetAttribute("EnableProxyJoin", BooleanValue(m_proxy));
      route->SetAttribute("EnableRangeGating", BooleanValue(m_range));
      route->SetIpv4(node->GetObject<Ipv4>()); node->AggregateObject(route);
      route->AssignStreams(2000000 + i * 2); m_routes.push_back(route);
      route->TraceConnectWithoutContext("JoinMechanism", MakeCallback(&CafbcrJoinMechanismTestCase::Observe, this));
    }
    BasicEnergySourceHelper energy; energy.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(100));
    energy.Install(m_nodes);
    NetDeviceContainer devices;
    if (m_kind == RELAY || m_kind == NO_RELAY) {
      CsmaHelper link; link.SetChannelAttribute("DataRate", StringValue("10Mbps"));
      link.SetChannelAttribute("Delay", TimeValue(MilliSeconds(1)));
      devices = link.Install(m_nodes);
      m_nodes.Get(1)->GetObject<MobilityModel>()->SetPosition(Vector(200,0,0));
      m_nodes.Get(2)->GetObject<MobilityModel>()->SetPosition(Vector(50,0,0));
      m_nodes.Get(2)->GetObject<ClusterNode>()->SetRole(Member);
      m_nodes.Get(2)->GetObject<ClusterNode>()->SetMaxRange(400);
      m_routes[2]->SetAttribute("EnableProxyJoin", BooleanValue(m_kind == RELAY));
    } else {
      PointToPointHelper link; link.SetDeviceAttribute("DataRate", StringValue("10Mbps"));
      link.SetChannelAttribute("Delay", TimeValue(MilliSeconds(1)));
      devices = link.Install(m_nodes.Get(0), m_nodes.Get(1));
    }
    m_drop = CreateObject<JoinDropErrorModel>();
    if (m_kind == RELAY || m_kind == NO_RELAY) {
      m_drop->blockedSource = Mac48Address::ConvertFrom(devices.Get(0)->GetAddress());
      devices.Get(1)->SetAttribute("ReceiveErrorModel", PointerValue(m_drop));
    } else if (m_kind == LOST_GRANT || m_kind == MAINTENANCE) {
      devices.Get(0)->SetAttribute("ReceiveErrorModel", PointerValue(m_drop));
      m_drop->dropAll = m_kind == MAINTENANCE;
    }
    Ipv4AddressHelper addresses; addresses.SetBase("10.1.1.0", "255.255.255.0");
    m_interfaces = addresses.Assign(devices);
    for (auto route : m_routes) route->NotifyInterfaceUp(1);
    auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    const char* parent = std::getenv("CAFBCR_JOIN_TEST_OUTPUT");
    auto path = parent ? std::filesystem::path(parent) / ("case-" + std::to_string(m_kind) + "-" +
      std::to_string(m_proxy) + std::to_string(m_range) + std::to_string(m_master) + "-" + std::to_string(m_selector))
      : std::filesystem::temp_directory_path() / ("cafbcr-join-" + std::to_string(stamp));
    std::filesystem::create_directories(path);
    m_writer = std::make_unique<JoinObservationWriter>(path.string(), Seconds(2));
    for (auto route : m_routes) m_writer->Attach(route);
    Simulator::Schedule(TimeStep(1), [this] {
      for (auto route : m_routes) {
        route->m_helloTimer.Cancel(); route->m_clusteringTimer.Cancel(); route->m_beaconTimer.Cancel();
      }
    });
    Simulator::Schedule(MilliSeconds(10), [this] {
      auto origin = m_routes[0];
      if (m_kind == HEAD_REQUEST_RACE || m_kind == HEAD_RESPONSE_RACE) {
        auto local = m_nodes.Get(0)->GetObject<ClusterNode>();
        if (m_kind == HEAD_REQUEST_RACE) {
          // Send while Idle; promote before the real UDP request is delivered.
          origin->m_neighbors->AddNeighbor(Neighbors::NeighborInfo(m_nodes.Get(1)->GetId(),
            m_interfaces.GetAddress(1), 0xFFFF, Vector(10,0,0), Vector(), 100, .8, Head,
            Simulator::Now(), Seconds(10)));
          if (m_proxy) {
            origin->SendProxyJoinRequest(local->GetNodeId(), m_interfaces.GetAddress(0),
              Vector(), Vector(), 100, .8, m_interfaces.GetAddress(1));
          } else {
            origin->SendClusterJoinRequest(m_interfaces.GetAddress(1));
          }
        }
        m_raceClusterId = m_manager->ArrangeClusterId();
        m_manager->CreateCluster(m_raceClusterId, local);
        if (m_kind == HEAD_REQUEST_RACE && m_selector == 1) {
          // A historical registry head ID must not block a now-Idle applicant.
          local->SetRole(Idle);
          origin->m_intentJoinChId = m_nodes.Get(1)->GetId();
        }
        if (m_kind == HEAD_RESPONSE_RACE) {
          ClusterHeadResponseHeader response(m_nodes.Get(1)->GetId(), m_raceClusterId + 1,
            m_interfaces.GetAddress(1), Simulator::Now().GetSeconds());
          origin->RecvJoinResponse(PacketWith(response), m_interfaces.GetAddress(0),
                                  m_interfaces.GetAddress(1));
        }
        return;
      }
      if (m_kind == PROXY_QUALIFICATION) {
        origin->m_neighbors->AddNeighbor(Neighbors::NeighborInfo(m_nodes.Get(1)->GetId(),
          m_interfaces.GetAddress(1), 0xFFFF, Vector(150,0,0), Vector(), 100, .8, Head,
          Simulator::Now(), Seconds(10)));
        HelloHeader intent(m_nodes.Get(2)->GetId(), 0xFFFF, Ipv4Address("10.1.1.3"),
          Vector(20,0,0), Vector(), 100, .8, Idle, Simulator::Now().GetSeconds());
        intent.SetIntentJoinChId(m_nodes.Get(1)->GetId());
        origin->RecvHello(PacketWith(intent), m_interfaces.GetAddress(0), Ipv4Address("10.1.1.3"));
        origin->SendProxyJoinRequest(m_nodes.Get(2)->GetId(), Ipv4Address("10.1.1.3"),
          Vector(20,0,0), Vector(), 100, .8, m_interfaces.GetAddress(1));
        return;
      }
      if (m_kind == GRANT_ELIGIBILITY) {
        auto local = m_nodes.Get(0)->GetObject<ClusterNode>();
        ClusterHeadBeaconHeader grant(m_nodes.Get(1)->GetId(), 42, m_interfaces.GetAddress(1),
          Vector(10,0,0), Vector(), .8, Simulator::Now().GetSeconds());
        grant.SetGrantedMember(0, local->GetNodeId());
        for (auto intent : {uint16_t(0xFFFF), uint16_t(0xFFFE)}) {
          local->SetRole(Idle); local->SetClusterId(0xFFFF); origin->m_intentJoinChId = intent;
          origin->RecvClusterHeadBeacon(PacketWith(grant), m_interfaces.GetAddress(0), m_interfaces.GetAddress(1));
          NS_TEST_EXPECT_MSG_EQ(local->GetRoleType(), Idle, "grant needs a matching current intent");
        }
        for (auto role : {Member, BackupHead, Head}) {
          local->SetRole(role); local->SetClusterId(23); origin->m_intentJoinChId = m_nodes.Get(1)->GetId();
          origin->RecvClusterHeadBeacon(PacketWith(grant), m_interfaces.GetAddress(0), m_interfaces.GetAddress(1));
          NS_TEST_EXPECT_MSG_EQ(local->GetRoleType(), role, "grant cannot move an existing member or head role");
          NS_TEST_EXPECT_MSG_EQ(local->GetClusterId(), 23, "old grant cannot change an existing association");
        }
        local->SetRole(Idle); local->SetClusterId(0xFFFF); origin->m_intentJoinChId = 0xFFFF;
        return;
      }
      if (m_kind == DISABLED_GUARDS) {
        origin->m_intentJoinChId = m_nodes.Get(1)->GetId();
        origin->SendHello();
        origin->SendProxyJoinRequest(m_nodes.Get(0)->GetId(), m_interfaces.GetAddress(0),
          Vector(), Vector(), 100, .8, m_interfaces.GetAddress(1));
        InjectProxy(m_nodes.Get(0)->GetId());
        m_routes[1]->m_recentGranted.push_back(m_nodes.Get(0)->GetId());
        ClusterHeadBeaconHeader grant(m_nodes.Get(1)->GetId(), 42, m_interfaces.GetAddress(1),
          Vector(10,0,0), Vector(), .8, Simulator::Now().GetSeconds());
        grant.SetGrantedMember(0, m_nodes.Get(0)->GetId());
        origin->RecvClusterHeadBeacon(PacketWith(grant), m_interfaces.GetAddress(0), m_interfaces.GetAddress(1));
        return;
      }
      if (m_kind == ROLLING) {
        InjectProxy(m_nodes.Get(0)->GetId());
        for (uint32_t i = 2; i < 10; ++i) InjectProxy(m_nodes.Get(i)->GetId());
        auto before = m_routes[1]->m_recentGranted;
        InjectProxy(m_nodes.Get(2)->GetId());
        NS_TEST_EXPECT_MSG_EQ((before == m_routes[1]->m_recentGranted), true, "repeated origin does not displace or reorder grants");
        return;
      }
      if (m_kind == RELAY || m_kind == NO_RELAY) {
        // Each neighbor entry is learned through an actual HELLO below.
        return;
      }
      auto table = origin->m_neighbors;
      table->AddNeighbor(Neighbors::NeighborInfo(m_nodes.Get(1)->GetId(), m_interfaces.GetAddress(1),
        0xFFFF, Vector(10,0,0), Vector(), 100, .8, Head, Simulator::Now(), Seconds(10)));
      table->AddNeighbor(Neighbors::NeighborInfo(99, Ipv4Address("10.1.1.99"),
        0xFFFF, Vector(150,0,0), Vector(), 100, .8, Member, Simulator::Now(), Seconds(10)));
      bool effectiveRange = m_master && m_range;
      NS_TEST_EXPECT_MSG_EQ(origin->WithinMyTxRange(Vector(150,0,0)), !effectiveRange, "local range gate");
      NS_TEST_EXPECT_MSG_EQ(origin->GreedyReachableNextHop(Vector(200,0,0)),
        (effectiveRange ? m_nodes.Get(1)->GetId() : 99), "forwarding candidate changes only with range control");
      // Remove the synthetic forwarding candidate before the association test.
      table->DeleteNeighbor(99);
      if (m_selector == 0) {
        origin->m_chCandidates.push_back({static_cast<uint16_t>(m_nodes.Get(1)->GetId()),
          m_interfaces.GetAddress(1), Vector(10,0,0), 10, .8});
        origin->SelectNearestClusterHeadAndJoin();
      } else if (m_selector == 1) {
        origin->ProactivelyJoinCluster();
      } else {
        origin->TrySelfPromotion();
      }
    });
    Simulator::Schedule(MilliSeconds(60), [this] {
      auto local = m_nodes.Get(0)->GetObject<ClusterNode>();
      if (m_kind == HEAD_REQUEST_RACE || m_kind == HEAD_RESPONSE_RACE) {
        if (m_kind == HEAD_REQUEST_RACE && m_selector == 1) {
          NS_TEST_EXPECT_MSG_EQ(local->GetRoleType(), (m_proxy ? Idle : Member),
                                "stale head registry does not block an Idle applicant");
          NS_TEST_EXPECT_MSG_EQ(Count(m_proxy ? "proxy_admitted_at_head" : "ordinary_admitted_at_head",
                                     m_nodes.Get(1)->GetId()), 1, "valid received request is admitted");
          NS_TEST_EXPECT_MSG_EQ(Count(m_proxy ? "proxy_request_rejected_head" : "ordinary_request_rejected_head",
                                     m_nodes.Get(1)->GetId()), 0, "guard applies only to a current Head");
          return;
        }
        NS_TEST_EXPECT_MSG_EQ(local->GetRoleType(), Head, "late join cannot demote an established head");
        NS_TEST_EXPECT_MSG_EQ(local->GetClusterId(), m_raceClusterId, "late join cannot overwrite the head cluster ID");
        NS_TEST_EXPECT_MSG_EQ(m_manager->GetCluster(m_raceClusterId)->ExistMember(local->GetNodeId()), true,
                              "head remains registered in its own cluster");
        NS_TEST_EXPECT_MSG_EQ(m_manager->GetClusterHeadId(m_raceClusterId), local->GetNodeId(),
                              "own cluster retains a consistent head");
        if (m_kind == HEAD_REQUEST_RACE) {
          NS_TEST_EXPECT_MSG_EQ(Count(m_proxy ? "proxy_request_rx" : "ordinary_request_rx",
                                     m_nodes.Get(1)->GetId()), 1, "the delayed real UDP request arrived");
          NS_TEST_EXPECT_MSG_EQ(Count(m_proxy ? "proxy_request_rejected_head" : "ordinary_request_rejected_head",
                                     m_nodes.Get(1)->GetId()), 1, "receiver rejects implicit head migration");
          NS_TEST_EXPECT_MSG_EQ(m_routes[1]->GetJoinRespCount(), 0, "rejected request produces no ordinary response");
          NS_TEST_EXPECT_MSG_EQ(m_routes[1]->m_recentGranted.empty(), true, "rejected request queues no proxy grant");
        } else {
          NS_TEST_EXPECT_MSG_EQ(Count("ordinary_response_ignored_head", local->GetNodeId()), 1,
                                "source rejects a late serialized response");
        }
        return;
      }
      if (m_kind == PROXY_QUALIFICATION) {
        NS_TEST_EXPECT_MSG_EQ(Count("intent_rx", local->GetNodeId()), 1, "origin intent reached the relay");
        NS_TEST_EXPECT_MSG_EQ(m_routes[0]->GetJoinReqCount(), 0, "proxy eligibility is independent of the data range switch");
        NS_TEST_EXPECT_MSG_EQ(Count("proxy_tx_unqualified", local->GetNodeId()), 1, "central sender also enforces local qualification");
        NS_TEST_EXPECT_MSG_EQ(m_manager->ExistMember(m_nodes.Get(2)->GetId()), false, "unqualified relay cannot create admission");
        return;
      }
      if (m_kind == GRANT_ELIGIBILITY) return;
      if (m_kind == DISABLED_GUARDS) {
        NS_TEST_EXPECT_MSG_EQ(m_manager->ExistMember(local->GetNodeId()), false, "disabled CH rejects received proxy request");
        NS_TEST_EXPECT_MSG_EQ(m_routes[0]->GetJoinReqCount(), 0, "central send guard suppresses proxy request");
        NS_TEST_EXPECT_MSG_EQ(Count("intent_rx", m_nodes.Get(1)->GetId()), 0, "disabled origin serializes no HELLO intent");
        NS_TEST_EXPECT_MSG_EQ(m_routes[1]->m_neighbors->IsNeighbor(local->GetNodeId()), true, "real HELLO arrived despite having no intent");
        return;
      }
      if (m_kind == ROLLING) {
        auto& queue = m_routes[1]->m_recentGranted;
        NS_TEST_EXPECT_MSG_EQ(queue.size(), 8, "fixed grant payload capacity");
        NS_TEST_EXPECT_MSG_EQ(queue.front(), m_nodes.Get(2)->GetId(), "oldest of nine unique origins was evicted");
        NS_TEST_EXPECT_MSG_EQ(Count("grant_enqueued", m_nodes.Get(1)->GetId()), 9, "nine unique grants");
        NS_TEST_EXPECT_MSG_EQ(Count("grant_evicted", m_nodes.Get(1)->GetId()), 1, "exactly one capacity eviction");
        return;
      }
      if (m_kind == NO_RELAY) {
        NS_TEST_EXPECT_MSG_EQ(m_manager->ExistMember(local->GetNodeId()), false, "no relay means no received request or admission");
        return;
      }
      bool proxy = m_master && m_proxy;
      NS_TEST_EXPECT_MSG_EQ(m_manager->ExistMember(local->GetNodeId()), true, "CH received the UDP request and admitted origin");
      NS_TEST_EXPECT_MSG_EQ(local->GetRoleType(), (proxy ? Idle : Member), "admission alone is not source confirmation");
      NS_TEST_EXPECT_MSG_EQ(m_routes[1]->GetJoinRespCount(), (proxy ? 0 : 1), "proxy off uses an actual ordinary response");
      NS_TEST_EXPECT_MSG_EQ(m_routes[1]->m_recentGranted.size(), (proxy ? 1 : 0), "ordinary join does not queue proxy grants");
      NS_TEST_EXPECT_MSG_EQ(m_routes[0]->m_intentJoinChId,
        (proxy ? m_nodes.Get(1)->GetId() : 0xFFFF), "disabled proxy leaves no intent");
      if (m_kind == MAINTENANCE) {
        auto head = m_routes[1]; head->m_beaconTimer.Cancel(); head->SetBeaconInterval(MilliSeconds(100));
        auto cluster = m_manager->GetCluster(local->GetClusterId());
        auto expiry = cluster->GetMemberUpdateTime(local->GetNodeId()) + Seconds(1);
        Simulator::Schedule(expiry - TimeStep(1) - Simulator::Now(), [this, head, cluster] {
          head->BeaconTimerExpire();
          NS_TEST_EXPECT_MSG_EQ(cluster->ExistMember(m_nodes.Get(0)->GetId()), true, "member retained immediately before maintenance threshold");
        });
        Simulator::Schedule(expiry + TimeStep(1) - Simulator::Now(), [this, head, cluster] {
          head->m_beaconTimer.Cancel(); head->BeaconTimerExpire();
          NS_TEST_EXPECT_MSG_EQ(cluster->ExistMember(m_nodes.Get(0)->GetId()), false, "missing CM_LIVE eventually expires admission");
          NS_TEST_EXPECT_MSG_EQ(Count("member_expired_at_head", m_nodes.Get(1)->GetId()), 1, "expiry is a CH-side fact");
          NS_TEST_EXPECT_MSG_EQ(Count("proxy_confirmed_at_source", m_nodes.Get(0)->GetId()), 0, "no grant reached the source");
          NS_TEST_EXPECT_MSG_EQ(head->m_recentGranted.empty(), true, "expired admission is no longer advertised as a grant");
        });
      }
    });
    if (m_kind == RELAY || m_kind == NO_RELAY) {
      Simulator::Schedule(MilliSeconds(3), &RoutingProtocol::SendHello, m_routes[1]);
      Simulator::Schedule(MilliSeconds(6), &RoutingProtocol::SendHello, m_routes[2]);
      Simulator::Schedule(MilliSeconds(20), [this] {
        m_routes[0]->m_chCandidates.push_back({static_cast<uint16_t>(m_nodes.Get(1)->GetId()),
          m_interfaces.GetAddress(1), Vector(200,0,0), 200, .8});
        m_routes[0]->SelectNearestClusterHeadAndJoin();
        NS_TEST_EXPECT_MSG_EQ(m_routes[0]->GetJoinReqCount(), 0, "out-of-range origin cannot self-proxy");
        m_routes[0]->SendHello();
      });
    }
    if (m_kind == LOST_GRANT) {
      Simulator::Schedule(MilliSeconds(70), [this] { m_drop->dropAll = true; });
      Simulator::Schedule(MilliSeconds(110), [this] {
        NS_TEST_EXPECT_MSG_EQ((m_drop->drops > 0), true, "a real received frame was dropped");
        NS_TEST_EXPECT_MSG_EQ(Count("grant_tx_attempt", m_nodes.Get(1)->GetId()), 1, "CH attempted a grant");
        NS_TEST_EXPECT_MSG_EQ(Count("grant_rx", m_nodes.Get(0)->GetId()), 0, "lost grant was not received");
        NS_TEST_EXPECT_MSG_EQ(m_nodes.Get(0)->GetObject<ClusterNode>()->GetRoleType(), Idle, "shared clusterId does not confirm the source");
        m_drop->dropAll = false;
      });
      Simulator::Schedule(MilliSeconds(130), &RoutingProtocol::SendClusterHeadBeacon, m_routes[1]);
    }
    if ((m_kind != HEAD_REQUEST_RACE || m_selector == 1) && m_kind != HEAD_RESPONSE_RACE)
      Simulator::Schedule(MilliSeconds(80), &RoutingProtocol::SendClusterHeadBeacon, m_routes[1]);
    if (m_kind == NORMAL) Simulator::Schedule(MilliSeconds(100), &RoutingProtocol::SendClusterHeadBeacon, m_routes[1]);
    Simulator::Schedule(MilliSeconds(220), [this] {
      auto origin = m_nodes.Get(0)->GetObject<ClusterNode>();
      if (m_kind == HEAD_REQUEST_RACE || m_kind == HEAD_RESPONSE_RACE) {
        if (m_kind == HEAD_REQUEST_RACE && m_selector == 1) {
          NS_TEST_EXPECT_MSG_EQ(origin->GetRoleType(), Member, "valid rejoin reaches actual source confirmation");
          NS_TEST_EXPECT_MSG_EQ(Count(m_proxy ? "proxy_confirmed_at_source" : "ordinary_confirmed_at_source",
                                     origin->GetNodeId()), 1, "valid source receives one confirmation");
        }
        return;
      }
      if (m_kind == PROXY_QUALIFICATION) return;
      if (m_kind == MAINTENANCE) return;
      if (m_kind == GRANT_ELIGIBILITY) {
        NS_TEST_EXPECT_MSG_EQ(Count("proxy_confirmed_at_source", origin->GetNodeId()), 0, "unrelated grant never confirms");
        NS_TEST_EXPECT_MSG_EQ(Count("grant_ignored_intent", origin->GetNodeId()), 2, "missing and different intents are distinct rejected receptions");
        NS_TEST_EXPECT_MSG_EQ(Count("grant_ignored_role", origin->GetNodeId()), 3, "stable roles are not overwritten");
        return;
      }
      if (m_kind == DISABLED_GUARDS || m_kind == NO_RELAY || m_kind == ROLLING) {
        NS_TEST_EXPECT_MSG_EQ(origin->GetRoleType(), Idle, "no source confirmation without an applicable received grant");
        NS_TEST_EXPECT_MSG_EQ(Count("proxy_confirmed_at_source", origin->GetNodeId()), 0, "admission or disabled/evicted grant is not confirmation");
        if (m_kind == DISABLED_GUARDS) {
          NS_TEST_EXPECT_MSG_EQ(Count("grant_ignored_disabled", origin->GetNodeId()), 1, "only the injected grant carries own ID; real disabled CH serializes none");
          NS_TEST_EXPECT_MSG_EQ((m_routes[0]->m_neighbors->PeekAll().at(m_nodes.Get(1)->GetId()).UpdateTime > MilliSeconds(80)), true, "real CH beacon arrived with no serialized grant");
        }
        return;
      }
      NS_TEST_EXPECT_MSG_EQ(origin->GetRoleType(), Member, "actual response or grant confirms origin");
      NS_TEST_EXPECT_MSG_EQ(m_routes[0]->GetJoinReqCount(), (m_kind == RELAY ? 0 : 1), "origin request attempt count");
      NS_TEST_EXPECT_MSG_EQ((m_routes[0]->GetCmBeaconCount() > 0), true, "confirmed member sends CM_LIVE");
      bool proxy = m_master && m_proxy;
      NS_TEST_EXPECT_MSG_EQ(Count(proxy ? "proxy_confirmed_at_source" : "ordinary_confirmed_at_source", origin->GetNodeId()), 1, "repeated grant causes one source transition");
      NS_TEST_EXPECT_MSG_EQ((Count("cm_live_refresh_at_head", m_nodes.Get(1)->GetId()) > 0), (m_kind != RELAY), "CM attempt is distinct from reception on the one-way fixture");
      if (m_kind == RELAY) {
        NS_TEST_EXPECT_MSG_EQ(m_routes[2]->GetJoinReqCount(), 1, "neighbor actually sent the proxied request");
        NS_TEST_EXPECT_MSG_EQ((m_drop->drops > 0), true, "origin-to-head frames are blocked");
        NS_TEST_EXPECT_MSG_EQ(m_routes[1]->m_recentGranted.size(), 1, "without CM reception the pending grant remains");
      } else if (proxy) {
        NS_TEST_EXPECT_MSG_EQ(m_routes[1]->m_recentGranted.empty(), true, "received CM_LIVE retires the pending grant");
        NS_TEST_EXPECT_MSG_EQ(Count("grant_retired_cm", m_nodes.Get(1)->GetId()), 1, "retirement follows actual CM reception");
      }
    });
    Simulator::Stop(m_kind == MAINTENANCE ? MilliSeconds(1020) : MilliSeconds(240)); Simulator::Run();
    m_writer->Finalize(); m_writer.reset();
    while (!m_manager->GetAllClusters().empty()) m_manager->RemoveCluster(m_manager->GetAllClusters().begin()->first);
    Simulator::Destroy(); m_routes.clear(); m_manager = nullptr;
  }
  bool m_proxy, m_range, m_master;
  uint32_t m_selector;
  uint16_t m_raceClusterId = 0xFFFF;
  Kind m_kind;
  NodeContainer m_nodes;
  Ipv4InterfaceContainer m_interfaces;
  std::vector<Ptr<RoutingProtocol>> m_routes;
  Ptr<ClusterManager> m_manager;
  Ptr<JoinDropErrorModel> m_drop;
  std::vector<JoinMechanismEvent> m_events;
  std::unique_ptr<JoinObservationWriter> m_writer;
};

// State/handler guards; real over-the-air renewal is checked by the separate
// three-node Wi-Fi fixture. No fabricated receive is claimed as wireless proof.
class CafbcrProxyRenewalCase : public TestCase
{
public:
  CafbcrProxyRenewalCase(const char* name, unsigned kind) : TestCase(name), m_kind(kind) {}
private:
  unsigned m_kind;
  std::vector<JoinMechanismEvent> m_events;
  std::vector<ProxyRenewalEvent> m_renewalEvents;
  void Observe(const JoinMechanismEvent& event) { m_events.push_back(event); }
  void ObserveRenewal(const ProxyRenewalEvent& event) { m_renewalEvents.push_back(event); }
  void DoRun() override
  {
    auto manager=ClusterManager::GetInstance();
    NS_TEST_ASSERT_MSG_EQ(manager->GetAllClusters().empty(),true,"clean registry");
    NodeContainer nodes; nodes.Create(3); InternetStackHelper stack; stack.Install(nodes);
    std::array<Ptr<ClusterNode>,3> states;
    std::array<Ptr<RoutingProtocol>,3> routes;
    for (unsigned i=0;i<3;++i) {
      auto state=CreateObject<ClusterNode>(); state->SetNodeId(nodes.Get(i)->GetId());
      state->SetClusterId(0xffff); state->SetRole(i==1?Head:Member); state->SetMaxRange(100);
      nodes.Get(i)->AggregateObject(state); states[i]=state;
      auto mobility=CreateObject<ConstantPositionMobilityModel>(); nodes.Get(i)->AggregateObject(mobility);
      mobility->SetPosition(Vector(i*10,0,0));
      auto route=CreateObject<RoutingProtocol>(); route->SetIpv4(nodes.Get(i)->GetObject<Ipv4>());
      route->SetBeaconInterval(Seconds(.1)); route->SetMaxRange(100); routes[i]=route;
      route->TraceConnectWithoutContext("JoinMechanism",MakeCallback(&CafbcrProxyRenewalCase::Observe,this));
      route->TraceConnectWithoutContext("ProxyRenewal",MakeCallback(&CafbcrProxyRenewalCase::ObserveRenewal,this));
      // Not aggregated and no interface notification: no election or periodic
      // socket traffic can alter this deliberately constructed state test.
    }
    manager->CreateCluster(42,states[1]); manager->JoinCluster(42,states[0]); states[0]->SetRole(Member);
    auto cluster=manager->GetCluster(42);
    routes[1]->m_recentGranted.push_back(states[0]->GetNodeId());
    const auto source=Ipv4Address("10.51.1.1"), head=Ipv4Address("10.51.1.2"), relay=Ipv4Address("10.51.1.3");
    const bool late = m_kind==6 || m_kind>=16;
    Simulator::Schedule(Seconds(late?2:.5), [&,this] {
      if (m_kind==12 || m_kind==13) {
        auto route=routes[2];
        Neighbors::NeighborInfo info(states[1]->GetNodeId(),head,42,Vector(m_kind==13?1000:10,0,0),
          Vector(),100,.8,Head,Seconds(0),Seconds(m_kind==12?.1:10));
        route->m_neighbors->AddNeighbor(info);
        HelloHeader hello(states[0]->GetNodeId(),42,source,Vector(),Vector(),100,.8,Member,.4);
        hello.SetIntentJoinChId(states[1]->GetNodeId());
        route->RelayProxyMemberHello(hello,source);
        NS_TEST_EXPECT_MSG_EQ(std::count_if(m_events.begin(),m_events.end(),[](const auto& e) {
          return e.event=="proxy_renewal_unqualified";
        }),1,"stale or out-of-range head view cannot generate a renewal");
        NS_TEST_EXPECT_MSG_EQ(route->GetCmBeaconCount(),0,"no relay send was attempted");
        return;
      }
      uint16_t origin=m_kind==3?99:states[0]->GetNodeId();
      double stamp=m_kind==4?0:m_kind==5?.6:m_kind==6?1.9:.4;
      if (m_kind==7) states[0]->SetRole(Head);
      if (m_kind==8) states[0]->SetClusterId(99);
      if (m_kind==9) routes[1]->SetAttribute("EnableProxyRenewal",BooleanValue(false));
      if (m_kind==10) routes[1]->SetAttribute("EnableProxyJoin",BooleanValue(false));
      if (m_kind==11) states[1]->SetClusterId(99);
      if (m_kind==16) cluster->UpdateMemberTime(states[0]->GetNodeId(),Seconds(1.9));
      if (m_kind==14) {
        ClusterMemberBeaconHeader direct(origin,42,source,Vector(),Vector(),.8,.35);
        auto packet=Create<Packet>(); packet->AddHeader(direct);
        routes[1]->RecvClusterMemberBeacon(packet,head,source);
      }
      auto receive=[&](Ipv4Address peer) {
        ClusterMemberBeaconHeader header(origin,m_kind==2?43:42,source,Vector(),Vector(),.8,stamp);
        auto packet=Create<Packet>(); packet->AddHeader(header);
        routes[1]->RecvClusterMemberBeacon(packet,head,peer);
      };
      receive(relay);
      if (m_kind==1) receive(relay);
      if (m_kind==15) receive(Ipv4Address("10.51.1.4"));
      const bool accepted = m_kind<=1 || m_kind==15;
      const Time expected = accepted ? Seconds(.4) : m_kind==14 ? Seconds(.5) :
        m_kind==16 ? Seconds(1.9) : Seconds(0);
      NS_TEST_EXPECT_MSG_EQ(cluster->GetMemberUpdateTime(states[0]->GetNodeId()),
        expected,"only a fresh current membership observation renews");
      NS_TEST_EXPECT_MSG_EQ(routes[1]->m_neighbors->IsNeighbor(states[0]->GetNodeId()),(m_kind==14),
        "only a real direct CM, never a relayed origin, creates a one-hop neighbor");
      NS_TEST_EXPECT_MSG_EQ(cluster->ExistMember(99),false,"renewal cannot admit an unknown member");
      NS_TEST_EXPECT_MSG_EQ(std::count_if(m_events.begin(),m_events.end(),[](const auto& e) {
        return e.event=="proxy_cm_live_refresh_at_head";
      }),(accepted?1:0),"duplicate and rejected messages do not refresh the lease");
      if (accepted) NS_TEST_EXPECT_MSG_EQ(routes[1]->m_recentGranted.empty(),true,
        "received renewal retires the outstanding grant");
      const unsigned expectedMask = (m_kind==1 || m_kind==4 || m_kind==14 || m_kind==15) ? 8 :
        m_kind==5 ? 4 : m_kind==6 ? 32 : m_kind==16 ? 24 : m_kind==17 ? 48 : 0;
      auto rejected = std::find_if(m_renewalEvents.begin(),m_renewalEvents.end(),[](const auto& e) {
        return e.event=="head_time_rejected";
      });
      NS_TEST_EXPECT_MSG_EQ((rejected!=m_renewalEvents.end()),(expectedMask!=0),
        "time rejection evidence is distinct from association rejection");
      if (rejected!=m_renewalEvents.end()) {
        NS_TEST_EXPECT_MSG_EQ(unsigned(rejected->timeRejections),expectedMask,
          "all applicable time guards are retained, including overlapping causes");
        NS_TEST_EXPECT_MSG_EQ(rejected->previousRefresh.has_value(),true,"record actual prior lease");
      }
    });
    Simulator::Stop(Seconds(late?2.01:.51)); Simulator::Run();
    while (!manager->GetAllClusters().empty()) manager->RemoveCluster(manager->GetAllClusters().begin()->first);
    for (auto route:routes) route->Dispose();
    Simulator::Destroy(); m_events.clear(); m_renewalEvents.clear();
  }
};

class CafbcrProxyRenewalSuite : public TestSuite
{
public:
  CafbcrProxyRenewalSuite() : TestSuite("cafbcr-proxy-renewal",Type::UNIT)
  {
    const char* names[]={"fresh relayed observation renews at source timestamp",
      "duplicate observation does not extend a lease","wrong cluster is rejected",
      "unknown member cannot be admitted by renewal","old observation is rejected",
      "future observation is rejected","fresh message cannot revive an expired lease",
      "promoted member cannot renew an old association","moved member cannot renew an old association",
      "renewal switch disables the relay receive path","proxy switch disables renewal",
      "foreign current cluster cannot be renewed","expired head view prevents relay",
      "out-of-range head view prevents relay","direct refresh precedes proxy observation",
      "two relays cannot refresh the same source observation twice",
      "old observation and not-newer evidence coexist","expired source and lease evidence coexist"};
    for (unsigned i=0;i<18;++i) AddTestCase(new CafbcrProxyRenewalCase(names[i],i),TestCase::QUICK);
  }
} g_cafbcrProxyRenewalSuite;

// Local queue/feedback guards. The separate Wi-Fi fixture supplies actual
// packets, selective PHY loss, multiple relays and membership maintenance.
class CafbcrRenewalFeedbackCase : public TestCase
{
public:
  CafbcrRenewalFeedbackCase(const char* name,unsigned kind):TestCase(name),m_kind(kind) {}
private:
  unsigned m_kind;
  std::vector<ProxyRenewalEvent> m_events;
  std::vector<JoinMechanismEvent> m_join;
  void Observe(const ProxyRenewalEvent& e) { m_events.push_back(e); }
  void Join(const JoinMechanismEvent& e) { m_join.push_back(e); }
  void DoRun() override
  {
    if (m_kind==0) {
      constexpr int64_t stamp=INT64_C(0x0102030405060708);
      RenewalAckHeader sent(0x1234,0x2345,0x3456,stamp),received;
      auto p=Create<Packet>();p->AddHeader(sent);p->AddHeader(TypeHeader(CAFBCR_RENEWAL_ACK));
      NS_TEST_EXPECT_MSG_EQ(p->GetSize(),15,"one type byte plus fourteen real feedback bytes");
      TypeHeader type;p->RemoveHeader(type);p->RemoveHeader(received);
      NS_TEST_EXPECT_MSG_EQ(type.IsValid(),true,"type eight is recognized");
      NS_TEST_EXPECT_MSG_EQ(type.Get(),CAFBCR_RENEWAL_ACK,"feedback dispatch type");
      NS_TEST_EXPECT_MSG_EQ(received.GetOrigin(),0x1234,"member identity survives wire encoding");
      NS_TEST_EXPECT_MSG_EQ(received.GetHead(),0x2345,"head identity survives wire encoding");
      NS_TEST_EXPECT_MSG_EQ(received.GetCluster(),0x3456,"cluster identity survives wire encoding");
      NS_TEST_EXPECT_MSG_EQ(received.GetSourceUs(),stamp,"integer observation never passes through double");
      for (double time:{1.000001,1.54783,126.004777}) {
        ClusterMemberBeaconHeader cm(0,42,Ipv4Address("10.51.1.1"),Vector(),Vector(),.8,time),decoded;
        auto data=Create<Packet>();data->AddHeader(cm);std::array<uint8_t,72> bytes;
        data->CopyData(bytes.data(),bytes.size());uint64_t raw=0;
        for (unsigned i=64;i<72;++i) raw=(raw<<8)|bytes[i];
        data->RemoveHeader(decoded);
        NS_TEST_EXPECT_MSG_EQ(decoded.GetReceivedTimeStampUs().has_value(),true,"raw receive integer is present");
        NS_TEST_EXPECT_MSG_EQ(*decoded.GetReceivedTimeStampUs(),int64_t(raw),"feedback uses the exact received CM integer");
      }
      Simulator::Destroy();return;
    }
    NodeContainer nodes;nodes.Create(3);InternetStackHelper stack;stack.Install(nodes);
    auto state=CreateObject<ClusterNode>();state->SetNodeId(nodes.Get(2)->GetId());
    state->SetRole(Member);state->SetClusterId(0xffff);nodes.Get(2)->AggregateObject(state);
    auto mobility=CreateObject<ConstantPositionMobilityModel>();nodes.Get(2)->AggregateObject(mobility);
    auto route=CreateObject<RoutingProtocol>();route->SetIpv4(nodes.Get(2)->GetObject<Ipv4>());
    route->SetBeaconInterval(Seconds(.1));route->SetMaxRange(100);
    route->SetAttribute("CoordinateProxyRenewal",BooleanValue(true));
    route->TraceConnectWithoutContext("ProxyRenewal",MakeCallback(&CafbcrRenewalFeedbackCase::Observe,this));
    route->TraceConnectWithoutContext("JoinMechanism",MakeCallback(&CafbcrRenewalFeedbackCase::Join,this));
    const auto origin=uint16_t(nodes.Get(0)->GetId()),headId=uint16_t(nodes.Get(1)->GetId());
    const auto source=Ipv4Address("10.51.1.1"),head=Ipv4Address("10.51.1.2");
    auto view=[&](Time updated,Time life) {
      route->m_neighbors->AddNeighbor(Neighbors::NeighborInfo(headId,head,42,Vector(10,0,0),
        Vector(),100,.8,Head,updated,life));
    };
    auto hello=[&](double time) {
      HelloHeader h(origin,42,source,Vector(),Vector(),100,.8,Member,time);
      h.SetIntentJoinChId(headId);return h;
    };
    const double now=m_kind==7?2:.5;
    Simulator::Schedule(Seconds(now),[&,this] {
      view(Seconds(0),Seconds(10));
      double observed=m_kind==7?1.9:.4;
      if (m_kind==17) {
        while (route->RenewalHoldoff({origin,headId,42,int64_t(observed*1e6)})<MilliSeconds(2)) observed+=.000001;
        const auto delay=route->RenewalHoldoff({origin,headId,42,int64_t(observed*1e6)});
        view(Simulator::Now(),delay/2);
      }
      route->RelayProxyMemberHello(hello(observed),source,123);
      NS_TEST_ASSERT_MSG_EQ(route->m_pendingRenewals.size(),1,"one local holdoff is created");
      if (m_kind==15) route->RelayProxyMemberHello(hello(observed),source,123);
      if (m_kind==9) route->RelayProxyMemberHello(hello(.45),source,124);
      if (m_kind==10) route->SetAttribute("EnableProxyRenewal",BooleanValue(false));
      if (m_kind==11) route->SetAttribute("CoordinateProxyRenewal",BooleanValue(false));
      if (m_kind==12) view(Seconds(0),Seconds(.1));
      if (m_kind==14) route->Dispose();
      if (m_kind!=14 && m_kind!=15 && m_kind!=17) {
        const int64_t stamp=m_kind==6?600000:m_kind==7?400000:m_kind==8?300000:m_kind==9?420000:400000;
        RenewalAckHeader ack(m_kind==2?origin+20:origin,m_kind==4?headId+20:headId,
          m_kind==3?43:42,stamp);
        auto packet=m_kind==13?Create<Packet>(13):Create<Packet>();
        if (m_kind!=13) packet->AddHeader(ack);
        route->RecvRenewalAck(packet,m_kind==5?Ipv4Address("10.51.1.9"):head);
      }
      if (m_kind==16) {
        route->RelayProxyMemberHello(hello(.45),source,124);
        route->RelayProxyMemberHello(hello(.35),source,125);
      }
      const size_t expected=(m_kind==1 || m_kind==14)?0:1;
      NS_TEST_EXPECT_MSG_EQ(route->m_pendingRenewals.size(),expected,"only received matching current feedback cancels covered observations");
      NS_TEST_EXPECT_MSG_EQ(ClusterManager::GetInstance()->GetAllClusters().empty(),true,
        "feedback neither reads a shared membership to decide suppression nor admits a member");
    });
    if (m_kind==16) Simulator::Schedule(Seconds(1.45),[&] {
      route->RelayProxyMemberHello(hello(1.4),source,126);
      NS_TEST_EXPECT_MSG_EQ(route->m_confirmedRenewals.empty(),true,"expiry uses source observation time, not ACK arrival time");
      NS_TEST_EXPECT_MSG_EQ(route->m_pendingRenewals.size(),1,"expired feedback cannot block a new renewal");
    });
    Simulator::Stop(Seconds(m_kind==7?2.3:m_kind==16?1.8:.8));Simulator::Run();
    if (m_kind==17) {
      const auto rejected=std::count_if(m_join.begin(),m_join.end(),[](const auto& e) {
        return e.event=="proxy_renewal_unqualified";
      });
      NS_TEST_EXPECT_MSG_EQ(rejected,1,"holdoff expiration rechecks the CH view before sending");
      NS_TEST_EXPECT_MSG_EQ(route->GetCmBeaconCount(),0,"no renewal is sent using the expired CH view");
    }
    route->Dispose();Simulator::Destroy();m_events.clear();m_join.clear();
  }
};

class CafbcrRenewalFeedbackSuite : public TestSuite
{
public:
  CafbcrRenewalFeedbackSuite():TestSuite("cafbcr-renewal-feedback",Type::UNIT)
  {
    const char* names[]={"fourteen-byte feedback and raw CM timestamp round trip",
      "matching feedback cancels pending relay","different member cannot cancel a relay",
      "different cluster cannot cancel a relay","different head cannot cancel a relay",
      "different IP sender cannot impersonate the local head view","future feedback is ignored",
      "expired observation feedback is ignored","older feedback leaves a newer pending observation",
      "cumulative feedback cancels only covered observations","renewal disabled ignores feedback",
      "eager policy ignores coordination feedback","expired head view rejects feedback",
      "malformed feedback length is ignored","dispose cancels pending callbacks",
      "duplicate HELLO queues once","feedback ages from source time and does not block new HELLO",
      "pending relay rechecks expired head view"};
    for (unsigned i=0;i<18;++i) AddTestCase(new CafbcrRenewalFeedbackCase(names[i],i),TestCase::QUICK);
  }
} g_cafbcrRenewalFeedbackSuite;

class CafbcrJoinMechanismSuite : public TestSuite
{
  void AddCase(CafbcrJoinMechanismTestCase* test)
  {
    const char* selected = std::getenv("CAFBCR_JOIN_TEST_CASE");
    if (selected && test->GetName() != selected) { delete test; return; }
    AddTestCase(test, TestCase::QUICK);
  }
public:
  CafbcrJoinMechanismSuite() : TestSuite("cafbcr-join-mechanism", Type::UNIT)
  {
    using C = CafbcrJoinMechanismTestCase;
    AddCase(new C("proxy on range on: admission awaits received grant", true, true));
    AddCase(new C("proxy on range off: forwarding gate is independent", true, false));
    AddCase(new C("proxy off range on: ordinary UDP JOIN remains available", false, true));
    AddCase(new C("proxy off range off: ordinary UDP JOIN remains available", false, false));
    AddCase(new C("compatibility master disables both mechanisms", true, true, false));
    AddCase(new C("proactive path uses ordinary JOIN when proxy is off", false, true, true, 1));
    AddCase(new C("self-promotion recheck uses ordinary JOIN when proxy is off", false, true, true, 2));
    AddCase(new C("lost real grant leaves source idle until a later grant arrives", true, true, true, 0, C::LOST_GRANT));
    AddCase(new C("disabled proxy rejects send receive intent and stale stored grants", false, true, true, 0, C::DISABLED_GUARDS));
    AddCase(new C("one-way source-to-head filter requires a real neighbor proxy", true, true, true, 0, C::RELAY));
    AddCase(new C("missing proxy leaves one-way origin unadmitted", true, true, true, 0, C::NO_RELAY));
    AddCase(new C("nine unique origins roll eight grants without fake confirmation", true, true, true, 0, C::ROLLING));
    AddCase(new C("unconfirmed admission expires when no CM_LIVE is received", true, true, true, 0, C::MAINTENANCE));
    AddCase(new C("unrelated or obsolete grant cannot change intent or stable roles", true, true, true, 0, C::GRANT_ELIGIBILITY));
    AddCase(new C("inflight grant preserves a replaced admission under eager", true, true, true, 0, C::GRANT_ADMISSION_RACE));
    AddCase(new C("inflight grant preserves a replaced admission under direct feedback", true, true, true, 1, C::GRANT_ADMISSION_RACE));
    AddCase(new C("inflight grant cannot confirm a removed admission", true, true, true, 2, C::GRANT_ADMISSION_RACE));
    AddCase(new C("matching grant still confirms and supports both maintenance paths", true, true, true, 3, C::GRANT_ADMISSION_RACE));
    AddCase(new C("proxy eligibility retains range checking with data gating on", true, true, true, 0, C::PROXY_QUALIFICATION));
    AddCase(new C("proxy eligibility retains range checking with data gating off", true, false, true, 0, C::PROXY_QUALIFICATION));
    AddCase(new C("late ordinary request cannot migrate a newly established head", false, true, true, 0, C::HEAD_REQUEST_RACE));
    AddCase(new C("late proxy request cannot migrate a newly established head", true, true, true, 0, C::HEAD_REQUEST_RACE));
    AddCase(new C("late ordinary response cannot demote an established head", false, true, true, 0, C::HEAD_RESPONSE_RACE));
    AddCase(new C("stale head registry does not block ordinary rejoin by an Idle node", false, true, true, 1, C::HEAD_REQUEST_RACE));
    AddCase(new C("stale head registry does not block proxy rejoin by an Idle node", true, true, true, 1, C::HEAD_REQUEST_RACE));
    AddCase(new C("old membership expiry preserves a newer Head association", true, true, true, 0, C::EXPIRY_OWNERSHIP));
    AddCase(new C("old membership expiry preserves a newer Member association", true, true, true, 1, C::EXPIRY_OWNERSHIP));
    AddCase(new C("old membership expiry preserves an already Idle node", true, true, true, 2, C::EXPIRY_OWNERSHIP));
    AddCase(new C("current membership still expires and resets its node", true, true, true, 3, C::EXPIRY_OWNERSHIP));
    AddCase(new C("fresh current membership remains registered", true, true, true, 4, C::EXPIRY_OWNERSHIP));
    AddCase(new C("legacy RemoveMember keeps its original clearing behavior", true, true, true, 5, C::EXPIRY_OWNERSHIP));
    AddCase(new C("record-only removal erases both maps without changing association", true, true, true, 6, C::EXPIRY_OWNERSHIP));
    AddCase(new C("Head maintains its current cluster instead of an older membership", true, true, true, 0, C::OWN_CLUSTER));
    AddCase(new C("ordinary admission and response use the same current owned cluster", false, true, true, 1, C::OWN_CLUSTER));
    AddCase(new C("proxy admission and grant use the same current owned cluster", true, true, true, 2, C::OWN_CLUSTER));
    AddCase(new C("ordinary join promotes a BackupHead despite a stale head registration", false, true, true, 3, C::OWN_CLUSTER));
    AddCase(new C("proxy join promotes a BackupHead despite a stale head registration", true, true, true, 4, C::OWN_CLUSTER));
    AddCase(new C("Head with a foreign current cluster recovers without maintaining it", true, true, true, 5, C::OWN_CLUSTER));
    AddCase(new C("Head with invalid current ID does not restore an older head registration", true, true, true, 6, C::OWN_CLUSTER));
    AddCase(new C("ordinary BackupHead admission cannot borrow a foreign cluster", false, true, true, 7, C::OWN_CLUSTER));
    AddCase(new C("proxy BackupHead admission cannot borrow a foreign cluster", true, true, true, 8, C::OWN_CLUSTER));
    AddCase(new C("Head without its own member registration follows invalid ownership recovery", true, true, true, 9, C::OWN_CLUSTER));
  }
} g_cafbcrJoinMechanismSuite;
// Finite state/codec checks, not wireless performance samples.
class CafbcrRenewalAckBatchCase : public TestCase
{
public:
  CafbcrRenewalAckBatchCase(const char* name,unsigned kind):TestCase(name),m_kind(kind) {}
private:
  unsigned m_kind;
  std::vector<ProxyRenewalEvent> m_events;
  void Observe(const ProxyRenewalEvent& e) { m_events.push_back(e); }
  void DoRun() override
  {
    NodeContainer nodes;nodes.Create(10);InternetStackHelper stack;stack.Install(nodes);
    std::vector<Ptr<ClusterNode>> states;
    for (auto it=nodes.Begin();it!=nodes.End();++it) {
      auto node=*it;
      auto s=CreateObject<ClusterNode>();s->SetNodeId(node->GetId());
      s->SetRole(Idle);s->SetClusterId(0xffff);node->AggregateObject(s);states.push_back(s);
      node->AggregateObject(CreateObject<ConstantPositionMobilityModel>());
    }
    auto manager=ClusterManager::GetInstance();manager->CreateCluster(42,states[0]);
    for (unsigned i=1;i<=8;++i) {manager->JoinCluster(42,states[i]);states[i]->SetRole(Member);}
    auto cluster=manager->GetCluster(42);
    const bool receiver=m_kind>=14;
    auto route=CreateObject<RoutingProtocol>();auto local=nodes.Get(receiver?9:0);
    route->SetIpv4(local->GetObject<Ipv4>());local->AggregateObject(route);
    route->SetBeaconInterval(Seconds(.1));route->SetMaxRange(100);
    route->SetAttribute("CoordinateProxyRenewal",BooleanValue(true));
    route->SetAttribute("CoalesceProxyRenewalAck",BooleanValue(true));
    route->TraceConnectWithoutContext("ProxyRenewal",MakeCallback(&CafbcrRenewalAckBatchCase::Observe,this));
    const auto headIp=Ipv4Address("10.52.1.1");
    auto queue=[&](unsigned i,double time,uint64_t uid) {
      ClusterMemberBeaconHeader cm(i,42,Ipv4Address(0x0a340100+i+1),Vector(),Vector(),.8,time),decoded;
      auto packet=Create<Packet>();packet->AddHeader(cm);packet->RemoveHeader(decoded);
      cluster->UpdateMemberTime(i,Seconds(time));route->SendRenewalAck(decoded,uid);
    };
    Simulator::Schedule(Seconds(2),[&,this] {
      if (!receiver) {
        // Install the send-only socket after Start has returned with no socket;
        // no periodic election/HELLO is allowed to alter this state fixture.
        auto socket=Socket::CreateSocket(local,TypeId::LookupByName("ns3::UdpSocketFactory"));
        socket->Bind();
        route->m_socketAddrs.emplace(socket,Ipv4InterfaceAddress(headIp,Ipv4Mask("255.255.255.0")));
      }
      if (receiver) {
        route->m_neighbors->AddNeighbor(Neighbors::NeighborInfo(0,headIp,42,Vector(10,0,0),
          Vector(),100,.8,Head,Seconds(2),Seconds(10)));
        const unsigned n=m_kind==21?8:2;
        for (unsigned i=1;i<=n;++i) {
          auto ip=Ipv4Address(0x0a340100+i+1);
          HelloHeader hello(i,42,ip,Vector(),Vector(),100,.8,Member,1.9);hello.SetIntentJoinChId(0);
          route->RelayProxyMemberHello(hello,ip,100+i);
        }
        NS_TEST_ASSERT_MSG_EQ(route->m_pendingRenewals.size(),n,"actual local HELLO path queues each observation");
        auto packet=Create<Packet>();
        if (m_kind==15) packet=Create<Packet>(15);
        else {
          const unsigned records=m_kind==16?9:m_kind==21?8:2;
          for (unsigned i=records;i>0;--i) {
            const unsigned origin=m_kind==18?1:(m_kind==17 && i==1?0xffff:i);
            const int64_t stamp=m_kind==20 && i==1?2100000:m_kind==22 && i==1?900000:1900000;
            packet->AddHeader(RenewalAckHeader(origin,0,42,stamp));
          }
        }
        if (m_kind==23) route->SetAttribute("EnableProxyRenewal",BooleanValue(false));
        route->RecvRenewalAck(packet,m_kind==19?Ipv4Address("10.52.1.99"):headIp);
        const unsigned remaining=m_kind==15 || m_kind==16 || m_kind==19 || m_kind==23?2:
          m_kind==17 || m_kind==18 || m_kind==20 || m_kind==22?1:0;
        NS_TEST_EXPECT_MSG_EQ(route->m_pendingRenewals.size(),remaining,"only individually valid received records cancel matching relays");
        NS_TEST_EXPECT_MSG_EQ(cluster->GetMemberUpdateTime(1),Seconds(0),"receiving feedback never renews CH membership");
        route->ClearPendingRenewals();return;
      }
      queue(1,m_kind==6?1.01:1.9,101);
      if (m_kind==1) queue(1,1.91,102);
      else if (m_kind==2) queue(1,1.9,102);
      else if (m_kind==3) for (unsigned i=2;i<=8;++i) queue(i,1.9,100+i);
      else if (m_kind!=6 && m_kind!=11) queue(2,1.9,102);
      if (m_kind==4) states[0]->SetRole(Idle);
      if (m_kind==5) states[1]->SetRole(Idle);
      if (m_kind==7) route->SetAttribute("EnableProxyRenewal",BooleanValue(false));
      if (m_kind==8) route->SetAttribute("CoordinateProxyRenewal",BooleanValue(false));
      if (m_kind==9) route->SetAttribute("CoalesceProxyRenewalAck",BooleanValue(false));
      if (m_kind==10) route->Dispose();
      if (m_kind==12) states[0]->SetClusterId(43);
      if (m_kind==13) route->SetAttribute("EnableProxyJoin",BooleanValue(false));
      if (m_kind==11) Simulator::Schedule(MilliSeconds(10),[&] {queue(2,1.9,102);});
    });
    Simulator::Stop(Seconds(2.05));Simulator::Run();
    if (!receiver) {
      std::vector<ProxyRenewalEvent> sent;
      for (const auto& e:m_events) if (e.event=="head_feedback_tx_attempt") sent.push_back(e);
      const bool discarded=m_kind==4 || (m_kind>=6 && m_kind<=10) || m_kind==12 || m_kind==13;
      const unsigned expected=discarded?0:m_kind==3?8:m_kind==1 || m_kind==2 || m_kind==5?1:2;
      NS_TEST_EXPECT_MSG_EQ(sent.size(),expected,"one accounting event per actual feedback record");
      NS_TEST_EXPECT_MSG_EQ(route->GetRenewalAckCount(),(expected?1:0),"packet count increases once per batch, not per record");
      if (!sent.empty()) {
        for (const auto& e:sent) {
          NS_TEST_EXPECT_MSG_EQ(e.time,(m_kind==3?Seconds(2):Seconds(2.02)),"first record anchors the deadline; capacity flush is immediate");
          NS_TEST_EXPECT_MSG_EQ(e.packetUid,sent.front().packetUid,"records share one actual send attempt");
        }
        if (m_kind==1) NS_TEST_EXPECT_MSG_EQ(*sent[0].observationUs,1910000,"only newer source observation survives compaction");
      }
      NS_TEST_EXPECT_MSG_EQ(route->m_pendingRenewalAcks.empty(),true,"flushed or disposed queue has no retained record");
    }
    while (!manager->GetAllClusters().empty()) manager->RemoveCluster(manager->GetAllClusters().begin()->first);
    route->Dispose();Simulator::Destroy();m_events.clear();
  }
};
class CafbcrRenewalAckBatchSuite : public TestSuite
{
public:
  CafbcrRenewalAckBatchSuite():TestSuite("cafbcr-renewal-ack-batch",Type::UNIT)
  {
    const char* names[]={"two members share one bounded feedback send","newer observation replaces pending record",
      "duplicate observation is covered without a second record","eighth member flushes immediately",
      "changed head role discards pending feedback","changed member role removes only its record",
      "source expiry during batching cannot revive membership","renewal switch rechecked at flush",
      "coordination switch rechecked at flush","batch switch rechecked at flush","dispose cancels feedback callback",
      "later member does not extend first record deadline","changed cluster invalidates queued feedback",
      "proxy permission rechecked at flush","two received records cancel two corresponding pending relays",
      "partial record rejects whole malformed datagram","nine records exceed bounded parser",
      "invalid record does not prevent a later valid record","duplicate received record cancels at most once",
      "wrong source IP rejects every record","future record leaves its pending relay intact",
      "eight received records decode and cancel individually","expired record leaves its pending relay intact",
      "disabled renewal ignores received batches"};
    for (unsigned i=0;i<24;++i) AddTestCase(new CafbcrRenewalAckBatchCase(names[i],i),TestCase::QUICK);
  }
} g_cafbcrRenewalAckBatchSuite;
} // namespace ns3::cafbcr
