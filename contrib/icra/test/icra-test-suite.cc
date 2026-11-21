/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "ns3/basic-energy-source-helper.h"
#include "ns3/arp-cache.h"
#include "ns3/boolean.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/error-model.h"
#include "ns3/fanet-clustermanager.h"
#include "ns3/fanet-node.h"
#include "ns3/icra-helper.h"
#include "ns3/icra-Qweight.h"
#include "ns3/icra-routing.h"
#include "ns3/inet-socket-address.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/mobility-helper.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/test.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/wifi-helper.h"
#include "ns3/wifi-mac-helper.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-phy.h"
#include "ns3/yans-wifi-helper.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <set>
#include <vector>

namespace ns3::icra
{
// Exercise paper invariants and concrete audit regressions. Friendship avoids
// exposing production mutation hooks solely for testing.
class IcraFidelityTestCase : public TestCase
{
  public:
    enum Case { ACTIONS, REWARD, METRICS, STATE, RANDOMNESS, PERSISTENCE, RESET,
                WIRE, ROUTING, DIRECT, MEMBERS, CLUSTERING, BACKBONE, STRATEGY,
                EXCLUSION, LEGACY, POLICY, BS_DELIVERY, RECONFIGURATION,
                ARP_RECOVERY, JOIN_BOUND, REJOIN_ONLY, SAME_POLICY, ALL_CH,
                DEMOTION, ROUND_UTILITY, CLUSTER_LIFETIME };
    IcraFidelityTestCase(Case which, const std::string& name) : TestCase(name), m_case(which) {}
  private:
    Case m_case;
    NodeContainer m_nodes;
    Ipv4InterfaceContainer m_interfaces;
    std::vector<Ptr<RoutingProtocol>> m_protocols;
    uint32_t m_received{0};
    bool m_sawQueued{false};

    void Clean()
    {
        auto manager = ClusterManager::GetInstance();
        std::vector<uint16_t> ids;
        for (const auto& [id, cluster] : manager->GetAllClusters()) ids.push_back(id);
        for (auto id : ids) manager->RemoveCluster(id);
        manager->ResetStatistics();
        manager->ClusterIdInitial();
        Simulator::Destroy();
        m_protocols.clear();
        m_nodes = NodeContainer();
    }

    Ptr<Node> MetricNode()
    {
        auto node = CreateObject<Node>();
        auto cluster = CreateObject<ClusterNode>();
        cluster->SetNodeId(node->GetId());
        cluster->BindNode(node);
        cluster->SetNetworkScale(2);
        cluster->SetOriginEnergy(100);
        cluster->updateResidualEnergy(50);
        node->AggregateObject(cluster);
        auto mobility = CreateObject<ConstantVelocityMobilityModel>();
        mobility->SetPosition({1000, 1000, 0});
        mobility->SetVelocity({10, 0, 0});
        node->AggregateObject(mobility);
        BasicEnergySourceHelper source;
        source.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(50));
        source.Install(node);
        cluster->GetNeighborTable()->AddNeighbor(Neighbors::NeighborInfo(
            1, Ipv4Address("10.8.0.2"), 1, {1100, 1000, 0}, {20, 0, 0},
            100, 0, Member, Seconds(0), Seconds(10)));
        return node;
    }

    void Network(const std::vector<Vector>& positions, bool paper = true,
                 uint16_t excluded = 0xffff)
    {
        m_nodes.Create(positions.size());
        for (uint32_t i = 0; i < m_nodes.GetN(); ++i)
        {
            auto cluster = CreateObject<ClusterNode>();
            cluster->SetNodeId(m_nodes.Get(i)->GetId());
            cluster->BindNode(m_nodes.Get(i));
            cluster->SetNetworkScale(m_nodes.GetN());
            cluster->SetOriginEnergy(100);
            m_nodes.Get(i)->AggregateObject(cluster);
        }
        MobilityHelper mobility;
        mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
        mobility.Install(m_nodes);
        for (uint32_t i = 0; i < m_nodes.GetN(); ++i)
            m_nodes.Get(i)->GetObject<MobilityModel>()->SetPosition(positions[i]);
        YansWifiChannelHelper channel;
        channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
        channel.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange", DoubleValue(500));
        YansWifiPhyHelper phy;
        phy.SetChannel(channel.Create());
        WifiHelper wifi;
        wifi.SetStandard(WIFI_STANDARD_80211b);
        wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode",
                                     StringValue("DsssRate2Mbps"), "ControlMode",
                                     StringValue("DsssRate1Mbps"));
        WifiMacHelper mac;
        mac.SetType("ns3::AdhocWifiMac");
        auto devices = wifi.Install(phy, mac, m_nodes);
        BasicEnergySourceHelper source;
        source.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(100));
        source.Install(m_nodes);
        icraHelper helper;
        helper.SetPreset(paper ? "paper" : "legacy");
        helper.SetExcludedNodeId(excluded);
        helper.SetQLearningParameters(paper ? 0.8 : 0.1, 0.5, 0.1);
        InternetStackHelper stack;
        stack.SetRoutingHelper(helper);
        stack.Install(m_nodes);
        Ipv4AddressHelper address;
        address.SetBase("10.8.0.0", "255.255.255.0");
        m_interfaces = address.Assign(devices);
        for (auto it = m_nodes.Begin(); it != m_nodes.End(); ++it)
        {
            auto protocol = (*it)->GetObject<RoutingProtocol>();
            protocol->Configure(paper, 500, Seconds(paper ? 2 : 10), paper ? 0.1 : 2,
                                paper ? 2 : 3, MilliSeconds(1), excluded);
            m_protocols.push_back(protocol);
        }
        helper.AssignStream(m_nodes, 5000);
    }

    void NeighborsFromGeometry()
    {
        for (uint32_t i = 0; i < m_nodes.GetN(); ++i)
        {
            auto protocol = m_protocols[i];
            protocol->m_neighbors->Clear();
            protocol->m_paperNeighbors.clear();
            auto pos = m_nodes.Get(i)->GetObject<MobilityModel>()->GetPosition();
            for (uint32_t j = 0; j < m_nodes.GetN(); ++j)
            {
                auto other = m_nodes.Get(j)->GetObject<ClusterNode>();
                auto otherPos = m_nodes.Get(j)->GetObject<MobilityModel>()->GetPosition();
                if (i != j && CalculateDistance(pos, otherPos) <= 500)
                {
                    protocol->m_neighbors->AddNeighbor(Neighbors::NeighborInfo(
                        other->GetNodeId(), m_interfaces.GetAddress(j), other->GetClusterId(),
                        otherPos, {}, 100, 0.5, other->GetRoleType(), Simulator::Now(), Seconds(10)));
                    RoutingProtocol::PaperNeighbor neighbor;
                    neighbor.nodeId = other->GetNodeId();
                    neighbor.headId = m_protocols[j]->m_paperHeadId;
                    neighbor.clusterId = other->GetClusterId();
                    neighbor.role = other->GetRoleType();
                    neighbor.position = otherPos;
                    neighbor.address = m_interfaces.GetAddress(j);
                    neighbor.updated = Simulator::Now();
                    neighbor.forwarder = m_protocols[j]->m_paperForwarder;
                    protocol->m_paperNeighbors.emplace(neighbor.nodeId, neighbor);
                }
            }
        }
    }

    void SetCluster(uint32_t i, uint16_t head, uint16_t cluster, bool forwarder = false)
    {
        auto protocol = m_protocols.at(i);
        protocol->m_paperHeadId = head;
        protocol->m_paperClusterId = cluster;
        protocol->m_paperForwarder = forwarder;
        auto node = m_nodes.Get(i)->GetObject<ClusterNode>();
        node->SetClusterId(cluster);
        node->SetRole(node->GetNodeId() == head ? Head : Member);
    }

    void Receive(Ptr<Socket> socket) { while (socket->Recv()) ++m_received; }
    void SendData(Ptr<Socket> socket) { socket->Send(Create<Packet>(64)); }

    void LongEpisode()
    {
        for (auto protocol : m_protocols)
            protocol->Configure(true,500,Seconds(10),0.1,2,MilliSeconds(1),
                                protocol->m_excludedNodeId);
    }

    // Isolate JOIN delivery after discovery/election: the intended CH is known,
    // but its MAC address must still be obtained through the actual ARP path.
    void StartColdJoin()
    {
        auto source=m_protocols[0];
        auto head=m_protocols[1];
        head->PaperLeaveCluster();
        head->PaperBecomeHead();
        source->PaperStartClustering();
        source->m_paperUtilityEvent.Cancel();
        source->m_paperElectionEvent.Cancel();
        source->m_paperSelectionEvent.Cancel();
        source->m_paperRoundUtility=0;
        source->m_paperUtilityFrozen=true;
        NeighborsFromGeometry();
        source->m_paperNeighbors.at(1).utility=1;
        source->m_paperArpCache->Flush();
        head->m_paperArpCache->Flush();
        source->PaperSelectHead();
    }

    void CheckArpRecovery()
    {
        for (bool aware : {false,true})
        {
            Network({{0,0,0},{100,0,0}});
            LongEpisode();
            m_protocols[0]->SetAttribute("PaperArpAwareJoin",BooleanValue(aware));
            auto loss=CreateObject<RateErrorModel>();
            loss->SetAttribute("ErrorRate",DoubleValue(1));
            loss->Disable();
            auto device=DynamicCast<WifiNetDevice>(m_nodes.Get(1)->GetDevice(0));
            device->GetPhy()->SetPostReceptionErrorModel(loss);
            auto sink=Socket::CreateSocket(m_nodes.Get(1),UdpSocketFactory::GetTypeId());
            sink->Bind(InetSocketAddress(Ipv4Address::GetAny(),9900));
            sink->SetRecvCallback(MakeCallback(&IcraFidelityTestCase::Receive,this));
            auto sender=Socket::CreateSocket(m_nodes.Get(0),UdpSocketFactory::GetTypeId());
            sender->Connect(InetSocketAddress(m_interfaces.GetAddress(1),9900));
            RoutingProtocol::PaperAuditCounters before;
            Simulator::Schedule(Seconds(1.1),[this,loss,&before]() {
                before=m_protocols[0]->GetPaperAuditCounters();
                loss->Enable();
                StartColdJoin();
            });
            // Lose the first ARP request, then restore the channel well before
            // its normal one-second retry. No ACCEPT is injected by the test.
            Simulator::Schedule(Seconds(1.15),[loss]() { loss->Disable(); });
            Simulator::Schedule(Seconds(1.6),[this,aware,&before]() {
                auto source=m_protocols[0];
                auto entry=source->m_paperArpCache->Lookup(m_interfaces.GetAddress(1));
                NS_TEST_ASSERT_MSG_NE(entry,nullptr,"Cold JOIN really initiated ARP resolution");
                if (entry)
                    NS_TEST_ASSERT_MSG_EQ(entry->IsWaitReply(),true,
                                          "The first ARP recovery opportunity is still pending");
                NS_TEST_ASSERT_MSG_EQ(source->m_paperRoundHead,0xffff,
                                      "An unresolved JOIN must not self-elect before ARP retries");
                NS_TEST_ASSERT_MSG_EQ(source->m_paperClustering,true,
                                      "The round stays open while the JOIN waits for ARP");
                NS_TEST_ASSERT_MSG_EQ((source->GetPaperAuditCounters().joinAttempts-before.joinAttempts),
                                      1,"First JOIN waits rather than filling the ARP queue with retries");
            });
            Simulator::Schedule(Seconds(2.6),[this,aware,&before]() {
                (void)aware;
                NS_TEST_ASSERT_MSG_EQ(m_protocols[0]->m_paperHeadId,1,
                                      "Only an outstanding JOIN can accept the real delayed reply");
                NS_TEST_ASSERT_MSG_EQ((m_protocols[0]->GetPaperAuditCounters().joinAccepts-before.joinAccepts),
                                      1,"An actual ACCEPT closes the recovered handshake");
            });
            m_received=0;
            Simulator::Schedule(Seconds(2.9),&IcraFidelityTestCase::SendData,this,sender);
            Simulator::Stop(Seconds(3.2)); Simulator::Run();
            NS_TEST_ASSERT_MSG_EQ(m_received,1,"Recovered control state still delivers UDP over WiFi");
            Clean();
        }
    }

    void CheckJoinBound()
    {
        Network({{0,0,0},{100,0,0}});
        LongEpisode();
        struct Expected { const char* name; bool value; } expected[] = {
            {"PaperArpAwareJoin",false}, {"PaperRejoinOnly",true},
            {"PaperRebuildEveryPolicy",true}, {"PaperAllCh",false}};
        for (const auto& item : expected)
        {
            BooleanValue value;
            m_protocols[0]->GetAttribute(item.name,value);
            NS_TEST_ASSERT_MSG_EQ(value.Get(),item.value,
                                  "Paper Step 3 recovery and policy re-clustering are the defaults");
        }
        m_protocols[0]->SetAttribute("PaperArpAwareJoin",BooleanValue(true));
        auto loss=CreateObject<RateErrorModel>();
        loss->SetAttribute("ErrorRate",DoubleValue(1));
        loss->Disable();
        DynamicCast<WifiNetDevice>(m_nodes.Get(1)->GetDevice(0))->GetPhy()
            ->SetPostReceptionErrorModel(loss);
        RoutingProtocol::PaperAuditCounters before;
        Simulator::Schedule(Seconds(1.1),[this,loss,&before]() {
            before=m_protocols[0]->GetPaperAuditCounters();
            loss->Enable();
            StartColdJoin();
        });
        Simulator::Stop(Seconds(2.95)); Simulator::Run();
        const auto after=m_protocols[0]->GetPaperAuditCounters();
        NS_TEST_ASSERT_MSG_LT((after.joinAttempts-before.joinAttempts),4,
                              "JOIN retransmission stays within its bound under persistent loss");
        NS_TEST_ASSERT_MSG_GT((after.joinAttempts-before.joinAttempts),0,
                              "A failed JOIN really was attempted");
        NS_TEST_ASSERT_MSG_EQ((after.joinAccepts-before.joinAccepts),0,"Loss cannot fabricate membership");
        NS_TEST_ASSERT_MSG_EQ((after.headJoinFailure-before.headJoinFailure),1,
                              "A failed real handshake is reported distinctly from election");
        NS_TEST_ASSERT_MSG_EQ(m_protocols[0]->m_paperClustering,false,"Formation terminates despite loss");
        NS_TEST_ASSERT_MSG_EQ(m_protocols[0]->m_paperHeadId,0,"Bounded failure falls back to a CH");
    }

    void CheckRejoinOnly()
    {
        for (bool rejoin : {false,true})
        {
            Network({{0,0,0},{100,0,0},{200,0,0}});
            LongEpisode();
            m_protocols[0]->SetAttribute("PaperRejoinOnly",BooleanValue(rejoin));
            RoutingProtocol::PaperAuditCounters before;
            Simulator::Schedule(Seconds(1.2),[this,rejoin,&before]() {
                const double publishedUtility=m_protocols[0]->m_paperRoundUtility;
                // Equal-energy peers tie at the maximum: a fresh Step 2 would
                // elect this member, while Step 3 can join the surviving CH.
                for (auto protocol : m_protocols)
                {
                    protocol->m_weights.energyWeight=1;
                    protocol->m_weights.degreeWeight=0;
                    protocol->m_weights.velocityWeight=0;
                    protocol->m_weights.distanceWeight=0;
                    protocol->m_paperUtilityFrozen=false;
                    protocol->UpdateNodeFitness();
                }
                if (rejoin)
                {
                    NS_TEST_ASSERT_MSG_EQ(m_protocols[0]->m_paperUtilityPublished,true,
                                          "Recovery starts with a utility published by the prior full round");
                    NS_TEST_ASSERT_MSG_GT(m_protocols[0]->ComputeFitnessICRA(),publishedUtility,
                                          "The current utility differs from the prior published snapshot");
                }
                m_protocols[0]->PaperLeaveCluster();
                SetCluster(0,1,1);
                m_protocols[0]->m_paperClustering=false;
                m_nodes.Get(1)->GetObject<MobilityModel>()->SetPosition({2000,0,0});
                NeighborsFromGeometry();
                m_protocols[0]->m_paperNeighbors.at(2).utility=1;
                before=m_protocols[0]->GetPaperAuditCounters();
                m_protocols[0]->PaperRecoverHead();
                if (rejoin)
                    NS_TEST_ASSERT_MSG_EQ_TOL(m_protocols[0]->m_paperRoundUtility,publishedUtility,1e-12,
                                              "Step 3 reuses its published utility without repeating Step 2");
            });
            Simulator::Stop(Seconds(2)); Simulator::Run();
            const auto after=m_protocols[0]->GetPaperAuditCounters();
            NS_TEST_ASSERT_MSG_EQ((after.formationStarts-before.formationStarts),1,
                                  "A head loss launches exactly one recovery");
            NS_TEST_ASSERT_MSG_EQ((after.rejoinStarts-before.rejoinStarts),(rejoin ? 1 : 0),
                                  "Only the treatment resumes at Step 3");
            NS_TEST_ASSERT_MSG_EQ((after.headElection-before.headElection),(rejoin ? 0 : 1),
                                  "Step 3 recovery avoids a tied local CH election");
            NS_TEST_ASSERT_MSG_EQ(m_protocols[0]->m_paperHeadId,(rejoin ? 2 : 0),
                                  "A surviving CH receives the member through JOIN and ACCEPT");
            if (rejoin)
                NS_TEST_ASSERT_MSG_EQ((after.joinAccepts-before.joinAccepts),1,
                                      "Recovery succeeds through actual network control traffic");
            Clean();
        }
    }

    void CheckSamePolicy()
    {
        Network({{0,0,0},{100,0,0}});
        LongEpisode();
        Simulator::Schedule(Seconds(1.5),[this]() {
            for (uint32_t i=0; i<2; ++i)
            {
                auto protocol=m_protocols[i];
                auto before=protocol->GetPaperAuditCounters();
                GsPolicyHeader policy;
                policy.epoch=1;
                policy.weights={0.25,0.25,0.25,0.25};
                policy.timestampNs=Simulator::Now().GetNanoSeconds();
                auto packet=Create<Packet>(); packet->AddHeader(policy);
                packet->AddHeader(TypeHeader(ICRA_GS_POLICY));
                protocol->PaperReceivePolicy(packet->Copy());
                NS_TEST_ASSERT_MSG_EQ(protocol->m_paperClustering,true,
                                      "III-C.5.5 restarts Step 1 even for identical weights");
                NS_TEST_ASSERT_MSG_EQ((protocol->GetPaperAuditCounters().formationStarts-before.formationStarts),
                                      1,"Every strategy epoch starts one round");
                NS_TEST_ASSERT_MSG_EQ((protocol->GetPaperAuditCounters().policyRebuilds-before.policyRebuilds),
                                      1,"The audit records the actual reconstruction trigger");
                protocol->PaperReceivePolicy(packet->Copy());
                NS_TEST_ASSERT_MSG_EQ((protocol->GetPaperAuditCounters().formationStarts-before.formationStarts),
                                      1,"A duplicate epoch does not restart a formation");
                policy.epoch=2; policy.weights={1,0,0,0};
                packet=Create<Packet>(); packet->AddHeader(policy);
                packet->AddHeader(TypeHeader(ICRA_GS_POLICY));
                protocol->PaperReceivePolicy(packet);
                NS_TEST_ASSERT_MSG_EQ(protocol->m_paperEpoch,1,
                                      "A new epoch waits for the open round to commit");
                NS_TEST_ASSERT_MSG_EQ(protocol->m_hasPendingPolicy,true,
                                      "The latest update is retained, not applied mid-round");
                NS_TEST_ASSERT_MSG_EQ((protocol->GetPaperAuditCounters().policyRebuilds-before.policyRebuilds),
                                      1,"An unfinished round does not multiply rebuilds");
            }
        });
        Simulator::Schedule(Seconds(4.2),[this]() {
            for (auto protocol : m_protocols)
            {
                NS_TEST_ASSERT_MSG_EQ(protocol->m_paperEpoch,2,
                                      "The deferred policy applies at the end of the round");
                NS_TEST_ASSERT_MSG_EQ(protocol->GetAppliedStrategyCount(),2,
                                      "Both strategy epochs are applied exactly once");
                NS_TEST_ASSERT_MSG_GT(protocol->GetPaperAuditCounters().policyRebuilds,1,
                                      "The deferred epoch starts its own round");
            }
        });
        Simulator::Stop(Seconds(4.25)); Simulator::Run();
    }

    void CheckClusterLifetime()
    {
        Network({{0,0,0},{100,0,0},{200,0,0}});
        LongEpisode();
        Simulator::Schedule(Seconds(1.5),[this]() {
            for (auto protocol : m_protocols) protocol->PaperLeaveCluster();
            auto manager=ClusterManager::GetInstance();
            manager->ResetStatistics();
            m_protocols[0]->PaperBecomeHead();
            m_protocols[1]->PaperCommitAsMember(0,0);
            m_protocols[2]->PaperCommitAsMember(0,0);
            // A repeated commitment must keep the same cluster incarnation.
            m_protocols[1]->PaperCommitAsMember(0,0);
            const auto& history=manager->GetClusterHistory();
            NS_TEST_ASSERT_MSG_EQ(history.size(),1,"Repeated membership does not recreate the cluster");
            NS_TEST_ASSERT_MSG_EQ(history.back().maxSize,3,"Joining updates historical peak size");
        });
        Simulator::Schedule(Seconds(2.0),[this]() {
            m_protocols[1]->PaperLeaveCluster();
            m_protocols[2]->PaperLeaveCluster();
            m_protocols[2]->PaperBecomeHead();
            m_protocols[1]->PaperCommitAsMember(2,2);
            NS_TEST_ASSERT_MSG_EQ(ClusterManager::GetInstance()->GetCluster(0)->GetAllMembers().size(),
                                  1,"The old cluster shrinks back to its head");
            NS_TEST_ASSERT_MSG_EQ(ClusterManager::GetInstance()->GetClusterHistory().front().maxSize,
                                  3,"Departures retain the historical peak size");
        });
        Simulator::Schedule(Seconds(2.5),[this]() {
            auto manager=ClusterManager::GetInstance();
            auto moved=m_nodes.Get(1)->GetObject<ClusterNode>();
            // Model an old mirror registration surviving a same-round switch.
            manager->GetCluster(0)->AddClusterMember(moved,Simulator::Now());
            moved->SetClusterId(2);
            m_protocols[0]->PaperLeaveCluster();
            const auto& history=manager->GetClusterHistory();
            NS_TEST_ASSERT_MSG_EQ_TOL(history.front().lifetime,1.0,1e-12,
                                     "Completed lifetime survives mirror cleanup");
            NS_TEST_ASSERT_MSG_EQ(history.front().maxSize,3,"A formerly multi-node cluster remains eligible");
            NS_TEST_ASSERT_MSG_EQ(history.front().dissolutionSize,1,
                                  "Dissolution records current members before clearing them");
            NS_TEST_ASSERT_MSG_EQ(moved->GetClusterId(),2,"Dissolving the old head preserves the new association");
            NS_TEST_ASSERT_MSG_EQ(m_protocols[1]->GetPaperHeadId(),2,"Protocol-local association is preserved");
            NS_TEST_ASSERT_MSG_EQ(manager->GetCluster(2)->ExistMember(1),true,
                                  "The destination cluster retains the moved member");
            uint32_t count;
            double lifetime,size;
            manager->GetClusteringStatistics(count,lifetime,size);
            NS_TEST_ASSERT_MSG_GT(lifetime,0,"A completed multi-node cluster contributes to the common metric");
        });
        Simulator::Schedule(Seconds(3.0),[this]() {
            m_protocols[0]->PaperBecomeHead();
            m_protocols[1]->PaperCommitAsMember(0,0);
        });
        Simulator::Schedule(Seconds(4.0),[this]() {
            auto manager=ClusterManager::GetInstance();
            m_protocols[0]->PaperLeaveCluster();
            std::vector<ClusterStatistics> incarnations;
            for (const auto& stats : manager->GetClusterHistory())
                if (stats.clusterId==0) incarnations.push_back(stats);
            NS_TEST_ASSERT_MSG_EQ(incarnations.size(),2,"A reused head ID has separate lifetime records");
            NS_TEST_ASSERT_MSG_EQ_TOL(incarnations[0].lifetime,1.0,1e-12,"The first lifetime remains unchanged");
            NS_TEST_ASSERT_MSG_EQ_TOL(incarnations[1].lifetime,1.0,1e-12,"The next lifetime uses its own creation time");
            NS_TEST_ASSERT_MSG_EQ(incarnations[1].maxSize,2,"The next incarnation tracks its own peak size");
            NS_TEST_ASSERT_MSG_EQ(incarnations[1].dissolutionSize,2,"Live members are counted at dissolution");
        });
        Simulator::Stop(Seconds(4.01)); Simulator::Run();
    }

    void CheckDemotion()
    {
        Network({{0,0,0},{100,0,0}});
        LongEpisode();
        Simulator::Schedule(Seconds(1.0),[this]() {
            // Round 1's equal utilities leave both nodes as CHs. A later
            // full round must let node 0 join the new higher-utility winner.
            NS_TEST_ASSERT_MSG_EQ(m_protocols[0]->m_paperHeadId,0,
                                  "The initial tied election leaves node 0 as a CH");
            NS_TEST_ASSERT_MSG_EQ(m_protocols[1]->m_paperHeadId,1,
                                  "The initial tied election leaves node 1 as a CH");
            // Give node 1 a strictly higher utility so the paper's maximum-
            // utility rule makes it the only election winner.
            m_protocols[1]->m_weights.energyWeight=1;
            m_protocols[1]->m_weights.degreeWeight=0;
            m_protocols[1]->m_weights.velocityWeight=0;
            m_protocols[1]->m_weights.distanceWeight=0;
            // A live weight change alone is not a utility declaration: both
            // peers run Step 1--3 and exchange the newly published values.
            m_protocols[1]->PaperStartClustering();
            m_protocols[0]->PaperStartClustering();
            NS_TEST_ASSERT_MSG_EQ(m_protocols[0]->m_paperHeadId,0,
                                  "Step 1 keeps the committed topology until commit");
        });
        Simulator::Stop(Seconds(2.6)); Simulator::Run();
        NS_TEST_ASSERT_MSG_EQ(m_protocols[0]->m_paperHeadId,1,
                              "A later round joins the higher-utility CH");
        NS_TEST_ASSERT_MSG_EQ(m_nodes.Get(0)->GetObject<ClusterNode>()->GetRoleType(),Member,
                              "The former CH becomes a member");
        NS_TEST_ASSERT_MSG_EQ(m_protocols[0]->GetPaperAuditCounters().headDemoted,1,
                              "Demotion is recorded separately from election fallbacks");
        NS_TEST_ASSERT_MSG_EQ(m_nodes.Get(1)->GetObject<ClusterNode>()->GetRoleType(),Head,
                              "The surviving CH keeps serving the cluster");
    }

    void CheckRoundUtility()
    {
        Network({{0,0,0},{300,0,0},{100,0,0},{750,0,0}});
        LongEpisode();
        // With a degree-only action, node 1 is the unique initial maximum:
        // degree 3/3 versus node 0's 2/3. All nodes use the same action.
        for (auto protocol : m_protocols)
        {
            protocol->m_weights.energyWeight=0;
            protocol->m_weights.degreeWeight=1;
            protocol->m_weights.velocityWeight=0;
            protocol->m_weights.distanceWeight=0;
        }
        Simulator::Schedule(Seconds(0.9),[this]() {
            auto head=m_protocols[1];
            NS_TEST_ASSERT_MSG_EQ(head->m_paperHeadId,1,
                                  "The unique utility maximum has declared itself CH");
            NS_TEST_ASSERT_MSG_EQ(head->m_paperClustering,false,
                                  "The CH finishes locally before its neighbors select a head");
            NS_TEST_ASSERT_MSG_EQ(m_protocols[0]->m_paperClustering,true,
                                  "The prospective member still awaits the normal JOIN phase");
            // The CH stays reachable from node 0, but loses its other two
            // neighbors before node 0's staggered selection. Recompute and
            // transmit through normal maintenance rather than editing W_table.
            m_nodes.Get(1)->GetObject<MobilityModel>()->SetPosition({-450,0,0});
            head->m_helloTimer.Cancel();
            head->PaperMaintenance();
        });
        Simulator::Schedule(Seconds(1.05),[this]() {
            NS_TEST_ASSERT_MSG_EQ_TOL(m_protocols[1]->ComputeFitnessICRA(),1.0/3,1e-12,
                                      "Live utility follows the CH's reduced physical neighborhood");
            auto observed=m_protocols[0]->m_paperNeighbors.find(1);
            NS_TEST_ASSERT_MSG_EQ((observed != m_protocols[0]->m_paperNeighbors.end()),true,
                                  "The member received the still-reachable CH's control traffic");
            if (observed != m_protocols[0]->m_paperNeighbors.end())
            {
                NS_TEST_ASSERT_MSG_GT(observed->second.updated,Seconds(0.84),
                                      "The assertion observes a post-election HELLO");
                NS_TEST_ASSERT_MSG_EQ_TOL(observed->second.utility,1.0,1e-12,
                                          "HELLO preserves the utility published for this formation");
            }
        });
        Simulator::Stop(Seconds(1.8)); Simulator::Run();
        NS_TEST_ASSERT_MSG_EQ(m_protocols[0]->m_paperHeadId,1,
                              "A pending member joins the CH elected from the same utility snapshot");
        NS_TEST_ASSERT_MSG_EQ(m_protocols[0]->GetPaperAuditCounters().joinAccepts,1,
                              "Membership is completed by an actual WiFi JOIN and ACCEPT");
        NS_TEST_ASSERT_MSG_EQ(m_protocols[0]->GetPaperAuditCounters().headNoCandidate,0,
                              "Live utility drift does not fabricate a singleton fallback");
    }

    void CheckAllCh()
    {
        Network({{0,0,0},{400,0,0},{800,0,0},{1200,0,0},{1600,0,0}},true,4);
        LongEpisode();
        for (auto protocol : m_protocols)
            protocol->SetAttribute("PaperAllCh",BooleanValue(true));
        auto sink=Socket::CreateSocket(m_nodes.Get(4),UdpSocketFactory::GetTypeId());
        sink->Bind(InetSocketAddress(Ipv4Address::GetAny(),9900));
        sink->SetRecvCallback(MakeCallback(&IcraFidelityTestCase::Receive,this));
        auto sender=Socket::CreateSocket(m_nodes.Get(0),UdpSocketFactory::GetTypeId());
        sender->Connect(InetSocketAddress(m_interfaces.GetAddress(4),9900));
        Simulator::Schedule(Seconds(2.5),&IcraFidelityTestCase::SendData,this,sender);
        Simulator::Stop(Seconds(3.5)); Simulator::Run();
        for (uint32_t i=0; i<4; ++i)
        {
            const auto counters=m_protocols[i]->GetPaperAuditCounters();
            NS_TEST_ASSERT_MSG_EQ(m_protocols[i]->m_paperHeadId,i,"Every participating UAV becomes CH");
            NS_TEST_ASSERT_MSG_EQ(counters.headAllCh,1,"The forced CH has a distinct audit reason");
            NS_TEST_ASSERT_MSG_EQ(counters.headElection,0,"The control does not report ordinary utility election");
            NS_TEST_ASSERT_MSG_EQ(counters.joinAttempts,0,"All-CH control needs no JOIN handshake");
        }
        NS_TEST_ASSERT_MSG_EQ(m_protocols[4]->GetPaperAuditCounters().formationStarts,0,
                              "The external BS stays outside the all-CH population");
        NS_TEST_ASSERT_MSG_EQ(m_nodes.Get(4)->GetObject<ClusterNode>()->GetRoleType(),Isolate,
                              "The external endpoint is not promoted to a UAV relay");
        NS_TEST_ASSERT_MSG_EQ(m_received,1,"The observed CH backbone delivers UDP over multiple WiFi hops");
    }

    void CheckActions()
    {
        auto q = QLearningWeightOptimizer::GetInstance();
        q->SetLegacyMode(false);
        NS_TEST_ASSERT_MSG_EQ(q->m_actionSpace.size(), 1771, "All integer compositions of 20");
        NS_TEST_ASSERT_MSG_EQ(q->m_qTable.size(), 0, "Do not allocate the Cartesian Q table");
        for (const auto& a : q->m_actionSpace)
            NS_TEST_ASSERT_MSG_EQ_TOL(a.w1+a.w2+a.w3+a.w4, 1, 1e-12, "Weights sum to one");
        QLearningWeightOptimizer::State s{0,0,0,0};
        NS_TEST_ASSERT_MSG_EQ(q->GetQValue(s, {0.25,0.25,0.25,0.25}), 1, "Algorithm 2 prior");
        NS_TEST_ASSERT_MSG_EQ(q->GetQValue(s, {1,0,0,0}), 0, "Boundary action exists");
        NS_TEST_ASSERT_MSG_EQ(q->m_qTable.size(), 0, "Queries stay sparse");
    }

    void CheckRewards()
    {
        using Q = QLearningWeightOptimizer;
        NS_TEST_ASSERT_MSG_EQ(Q::ComputeNodeReward(0,0,0.5,2), 1, "Stable and no energy loss");
        NS_TEST_ASSERT_MSG_EQ(Q::ComputeNodeReward(2,0,0.5,2), 0, "Equality with phi is unstable");
        NS_TEST_ASSERT_MSG_EQ(Q::ComputeNodeReward(2,1,0.5,2), -1, "Both objectives worst");
        double average = (Q::ComputeNodeReward(4,0,0.5,3)+3*Q::ComputeNodeReward(0,0,0.5,3))/4;
        NS_TEST_ASSERT_MSG_EQ_TOL(average, 0.75, 1e-12, "Average per-node rewards, not majority vote");
        const double expected = (1-std::exp(-1.5))/(1+std::exp(1.5));
        NS_TEST_ASSERT_MSG_EQ_TOL(Q::ComputeNodeReward(0,0.5,0.5,2), expected, 1e-12,
                                  "Keep the paper's unusual Eq22");
    }

    void CheckMetrics()
    {
        auto node = MetricNode();
        auto f = QLearningWeightOptimizer::GetUtilityFactors(node,500,50);
        NS_TEST_ASSERT_MSG_EQ_TOL(f[0],1,1e-12,"Live energy source overrides stale cluster energy");
        NS_TEST_ASSERT_MSG_EQ_TOL(f[1],1,1e-12,"Degree / (N-1)");
        NS_TEST_ASSERT_MSG_EQ_TOL(f[2],0,1e-12,"One sample has zero variance");
        NS_TEST_ASSERT_MSG_EQ_TOL(f[3],0.8,1e-12,"True motion gives LHT=40s normalized by 50");
        f = QLearningWeightOptimizer::GetUtilityFactors(node,200,50);
        NS_TEST_ASSERT_MSG_EQ_TOL(f[3],0.2,1e-12,"Configured range reaches the predictor");
        node->GetObject<ConstantVelocityMobilityModel>()->SetVelocity({20,0,0});
        f = QLearningWeightOptimizer::GetUtilityFactors(node,500,50);
        NS_TEST_ASSERT_MSG_EQ_TOL(f[3],1,1e-12,"Equal velocities have finite capped utility");
        auto c=node->GetObject<ClusterNode>();
        c->SetNetworkScale(3);
        c->GetNeighborTable()->AddNeighbor(Neighbors::NeighborInfo(
            2,Ipv4Address("10.8.0.3"),1,{1000,1100,0},{10,0,0},
            100,0,Member,Seconds(0),Seconds(10)));
        node->GetObject<ConstantVelocityMobilityModel>()->SetVelocity({10,0,0});
        f=QLearningWeightOptimizer::GetUtilityFactors(node,500,50);
        NS_TEST_ASSERT_MSG_EQ_TOL(f[2],25.0/121,1e-12,"Literal variance, not mean or inverse variance");
        NS_TEST_ASSERT_MSG_EQ_TOL(f[3],0.9,1e-12,"Cap each LHTP before averaging, including infinity");
        auto protocol=CreateObject<RoutingProtocol>();
        protocol->Configure(true,500,Seconds(2),0.1,2,MilliSeconds(1));
        NS_TEST_ASSERT_MSG_EQ_TOL(protocol->CalculateLHTP({0,0,0},{10,0,0},
                                                         {100,0,400},{20,0,10}),
                                  40,1e-12,"JOIN uses the same planar paper predictor as utility");
    }

    void CheckState()
    {
        auto q = QLearningWeightOptimizer::GetInstance();
        QLearningWeightOptimizer::NodeStatus a;
        a.nodeId=0; a.energy=100; a.originEnergy=100; a.factors={0.5,0.5,0,0.5};
        auto b=a; b.nodeId=1; b.factors[2]=1;
        q->CollectNodeStatus(0,a);
        q->CollectNodeStatus(1,b);
        q->StartEpisode();
        auto s=q->GetCurrentState();
        NS_TEST_ASSERT_MSG_EQ(s.s1,0,"Same energy distribution");
        NS_TEST_ASSERT_MSG_EQ(s.s2,0,"Same degree distribution");
        NS_TEST_ASSERT_MSG_GT(s.s3,0,"Variance feature, not raw speed, drives third state");
        NS_TEST_ASSERT_MSG_EQ(s.s4,0,"Same LHTP distribution");
        q->Reset();
        b.factors={0.5,0.5,0,1};
        q->CollectNodeStatus(0,a);
        q->CollectNodeStatus(1,b);
        q->StartEpisode();
        s=q->GetCurrentState();
        NS_TEST_ASSERT_MSG_EQ(s.s3,0,"Same velocity variance");
        NS_TEST_ASSERT_MSG_GT(s.s4,0,"Reported LHTP, not geometric distance, drives fourth state");
    }

    void CheckFeedback()
    {
        auto q=QLearningWeightOptimizer::GetInstance();
        for (uint16_t id=0; id<4; ++id)
        {
            QLearningWeightOptimizer::NodeStatus status;
            status.nodeId=id; status.energy=100; status.originEnergy=100;
            status.factors={1,0.5,0,1};
            q->CollectNodeStatus(id,status);
            q->CollectFeedback(id,{id,0,0,Seconds(0)});
        }
        q->SetRoleChangeThreshold(3);
        q->StartEpisode();
        const auto state=q->GetCurrentState();
        const auto action=q->GetCurrentAction();
        Simulator::Schedule(Seconds(2),[q]() {
            for (uint16_t id=0; id<4; ++id)
                q->CollectFeedback(id,{id,0,id==0 ? 4u : 0u,Simulator::Now()});
            q->EndEpisode();
        });
        Simulator::Stop(Seconds(2.01));
        Simulator::Run();
        NS_TEST_ASSERT_MSG_EQ(q->GetLastFeedbackCount(),4,"Received reports drive actual reward");
        NS_TEST_ASSERT_MSG_EQ_TOL(q->GetLastReward(),0.75,1e-12,"Individual feedback is averaged");
        NS_TEST_ASSERT_MSG_EQ_TOL(q->GetQValue(state,action),0.8,1e-12,"Alpha=.8 and gamma=0");
        q->StartEpisode();
        const double before=q->GetQValue(q->GetCurrentState(),q->GetCurrentAction());
        q->EndEpisode();
        NS_TEST_ASSERT_MSG_EQ(q->GetLastFeedbackCount(),0,"Old feedback is not rewarded twice");
        NS_TEST_ASSERT_MSG_EQ_TOL(q->GetQValue(q->GetCurrentState(),q->GetCurrentAction()),before,
                                  1e-12,"No new feedback means no fabricated Q update");
    }

    void CheckRandomness()
    {
        auto q=QLearningWeightOptimizer::GetInstance();
        QLearningWeightOptimizer::State state{0,0,0,0};
        q->SetQValue(state,{0.25,0.25,0.25,0.25},0);
        q->AssignStreams(731);
        std::vector<QLearningWeightOptimizer::Action> first;
        std::set<QLearningWeightOptimizer::Action> distinct;
        for (int i=0; i<40; ++i)
        {
            auto a=q->GetBestAction(state);
            first.push_back(a);
            distinct.insert(a);
        }
        NS_TEST_ASSERT_MSG_GT(distinct.size(),1,"Tied maxima must not always pick the first action");
        auto second=CreateObject<QLearningWeightOptimizer>();
        second->SetQValue(state,{0.25,0.25,0.25,0.25},0);
        second->AssignStreams(731);
        for (const auto& a:first)
            NS_TEST_ASSERT_MSG_EQ((second->GetBestAction(state)==a),true,"Seed/Run/stream reproduce ties");
    }

    void CheckPersistence()
    {
        auto q=QLearningWeightOptimizer::GetInstance();
        QLearningWeightOptimizer::State state{1,2,3,4};
        QLearningWeightOptimizer::Action action{0.1,0.2,0.3,0.4};
        q->SetQValue(state,action,0.37);
        q->SetQValue({0,0,0,0},{0.25,0.25,0.25,0.25},0);
        auto path=CreateTempDirFilename("icra-paper-q.csv");
        q->SaveQTable(path);
        auto second=CreateObject<QLearningWeightOptimizer>();
        second->LoadQTable(path);
        NS_TEST_ASSERT_MSG_EQ_TOL(second->GetQValue(state,action),0.37,1e-12,"Own file reloads");
        NS_TEST_ASSERT_MSG_EQ(second->GetStoredQEntryCount(),q->GetStoredQEntryCount(),"Sparse model roundtrip");
        NS_TEST_ASSERT_MSG_EQ(second->GetQValue({0,0,0,0},{0.25,0.25,0.25,0.25}),0,
                              "Explicit zero overrides the implicit equal-weight prior after reload");
        NS_TEST_ASSERT_MSG_EQ(second->GetQValue({9,9,9,9},{0.25,0.25,0.25,0.25}),1,
                              "Implicit prior survives roundtrip");
    }

    void CheckReset()
    {
        auto q=QLearningWeightOptimizer::GetInstance();
        q->SetQValue({0,0,0,0},{0.25,0.25,0.25,0.25},-1);
        q->StartEpisode();
        Simulator::Destroy();
        auto next=QLearningWeightOptimizer::GetInstance();
        NS_TEST_ASSERT_MSG_EQ(next->GetEpisodeCount(),0,"New simulation has no inherited episodes");
        NS_TEST_ASSERT_MSG_EQ(next->GetStoredQEntryCount(),0,"Fresh simulation has a fresh model");
        NS_TEST_ASSERT_MSG_EQ(next->GetQValue({0,0,0,0},{0.25,0.25,0.25,0.25}),1,"Fresh prior");
    }

    void CheckWire()
    {
        GsReportHeader report;
        report.nodeId=7; report.factors={0.23,0.5,0.017,0.8};
        report.energyChange=0.2; report.roleChanges=3; report.epoch=9;
        report.timestampNs=123456789; report.position={-2.125,35.75,300.5};
        report.velocity={10.25,-20.5,0.125}; report.residualEnergy=80;
        auto packet=Create<Packet>();
        packet->AddHeader(report); packet->AddHeader(TypeHeader(ICRA_GS_REPORT));
        NS_TEST_ASSERT_MSG_EQ(packet->GetSize(),126,"Report includes one-byte type");
        TypeHeader type; packet->RemoveHeader(type);
        NS_TEST_ASSERT_MSG_EQ(type.Get(),ICRA_GS_REPORT,"Report type roundtrip");
        GsReportHeader received;
        NS_TEST_ASSERT_MSG_EQ(packet->RemoveHeader(received),125,"Full report wire length");
        NS_TEST_ASSERT_MSG_EQ(received.IsValid(),true,"Valid report");
        NS_TEST_ASSERT_MSG_EQ_TOL(received.position.x,-2.125,1e-12,"Signed full-precision position");
        NS_TEST_ASSERT_MSG_EQ_TOL(received.velocity.z,0.125,1e-12,"3D velocity transport");
        NS_TEST_ASSERT_MSG_EQ(received.timestampNs,123456789,"Report timestamp");
        received.energyChange=-0.1;
        NS_TEST_ASSERT_MSG_EQ(received.IsValid(),false,"Invalid feedback cannot enter GS state");
        auto shortPacket=Create<Packet>(124);
        NS_TEST_ASSERT_MSG_EQ(shortPacket->RemoveHeader(received),0,"Truncated report rejected");
        NS_TEST_ASSERT_MSG_EQ(received.IsValid(),false,"Truncation invalidates reused object");

        GsPolicyHeader policy; policy.weights={0.1,0.2,0.3,0.4}; policy.epoch=8;
        policy.timestampNs=987654321;
        packet=Create<Packet>(); packet->AddHeader(policy);
        GsPolicyHeader policyCopy;
        NS_TEST_ASSERT_MSG_EQ(packet->RemoveHeader(policyCopy),44,"Policy wire length");
        NS_TEST_ASSERT_MSG_EQ(policyCopy.IsValid(),true,"Valid simplex policy");
        NS_TEST_ASSERT_MSG_EQ_TOL(policyCopy.weights[2],0.3,1e-12,"Correct factor order");
        policyCopy.weights={0.123,0.2,0.3,0.377};
        NS_TEST_ASSERT_MSG_EQ(policyCopy.IsValid(),false,"Only the paper action grid is valid");
        PaperControlHeader control;
        control.kind=PaperControlHeader::IFN_REQUEST; control.nodeId=3; control.headId=2;
        control.forwarder=true; control.role=Member; control.position={-500.5,0,0};
        packet=Create<Packet>(); packet->AddHeader(control);
        PaperControlHeader copy;
        NS_TEST_ASSERT_MSG_EQ(packet->RemoveHeader(copy),99,"Paper A2A wire length");
        NS_TEST_ASSERT_MSG_EQ(copy.IsValid(),true,"Explicit IFN registration control");
        NS_TEST_ASSERT_MSG_EQ(copy.forwarder,true,"IFN intent roundtrip");
        NS_TEST_ASSERT_MSG_EQ_TOL(copy.position.x,-500.5,1e-12,"Coordinates do not wrap uint16");
        copy.kind=static_cast<PaperControlHeader::Kind>(99);
        NS_TEST_ASSERT_MSG_EQ(copy.IsValid(),false,"Unknown control kind rejected");
        control.utility=std::numeric_limits<double>::quiet_NaN();
        NS_TEST_ASSERT_MSG_EQ(control.IsValid(),false,"Nonfinite utility rejected");
    }

    void CheckPolicy()
    {
        Network({{1000,1000,0},{1100,1000,0}});
        // Deliver a policy only after the IP/MAC/traffic-control objects have
        // initialized, as on the real GS path (first delivery is after 2s).
        Simulator::Schedule(Seconds(1.5),[this]() {
            double before=m_protocols[1]->UpdateNodeFitness();
            GsPolicyHeader policy; policy.epoch=1; policy.weights={1,0,0,0};
            policy.timestampNs=Simulator::Now().GetNanoSeconds();
            auto packet=Create<Packet>(); packet->AddHeader(policy);
            packet->AddHeader(TypeHeader(ICRA_GS_POLICY));
            m_protocols[1]->PaperReceivePolicy(packet);
            double after=m_protocols[1]->UpdateNodeFitness();
            NS_TEST_ASSERT_MSG_GT(after,before,"Received strategy changes actual fitness");
            NS_TEST_ASSERT_MSG_EQ_TOL(after,1,1e-12,"Energy-only action is used by utility calculation");
            NS_TEST_ASSERT_MSG_EQ(m_protocols[1]->m_paperClustering,true,"Changed policy starts reclustering");
            NS_TEST_ASSERT_MSG_EQ(m_protocols[1]->GetAppliedStrategyCount(),1,"Policy applied once");
            packet=Create<Packet>(); packet->AddHeader(policy);
            packet->AddHeader(TypeHeader(ICRA_GS_POLICY));
            m_protocols[1]->PaperReceivePolicy(packet);
            NS_TEST_ASSERT_MSG_EQ(m_protocols[1]->GetAppliedStrategyCount(),1,"Duplicate policy ignored");
        });
        Simulator::Stop(Seconds(1.55)); Simulator::Run();
    }

    void CheckExclusion()
    {
        Network({{0,0,0},{400,0,0},{200,0,0}},true,2);
        Simulator::Stop(Seconds(4.5)); Simulator::Run();
        NS_TEST_ASSERT_MSG_EQ(m_protocols[2]->m_participates,false,"External BS is not a UAV");
        NS_TEST_ASSERT_MSG_EQ(m_protocols[2]->GetGsReportCount(),0,"BS is not in learning population");
        NS_TEST_ASSERT_MSG_EQ(m_nodes.Get(2)->GetObject<ClusterNode>()->GetRoleType(),Isolate,
                              "Passive endpoint never becomes CH or IFN");
        auto q=QLearningWeightOptimizer::GetInstance();
        NS_TEST_ASSERT_MSG_EQ(q->m_nodeStatusMap.count(2),0,"GS only learns from UAV reports");
        NS_TEST_ASSERT_MSG_EQ(m_nodes.Get(0)->GetObject<ClusterNode>()->GetNetworkScale(),2,
                              "Eq8 population denominator excludes the external BS");
        NS_TEST_ASSERT_MSG_GT(m_protocols[0]->GetGsPolicyCount(),0,"Logical GS distributes policies");
        NS_TEST_ASSERT_MSG_GT(m_protocols[1]->GetAppliedStrategyCount(),0,"UAV receives policy");
    }

    void CheckLegacy()
    {
        Network({{0,0,0},{100,0,0}},false);
        NS_TEST_ASSERT_MSG_EQ(m_protocols[0]->GetPaperMode(),false,"Legacy routing selected");
        NS_TEST_ASSERT_MSG_EQ(QLearningWeightOptimizer::GetInstance()->IsLegacyMode(),true,
                              "Legacy optimizer selected");
        Simulator::Stop(Seconds(3.8)); Simulator::Run();
        NS_TEST_ASSERT_MSG_EQ(m_protocols[0]->GetAppliedStrategyCount(),0,"Legacy keeps fixed weights");
        NS_TEST_ASSERT_MSG_EQ(m_protocols[0]->GetGsReportCount(),0,"Legacy uses historical control path");
        NS_TEST_ASSERT_MSG_EQ_TOL(m_protocols[0]->m_weights.energyWeight,0.25,1e-12,"Legacy equal weights");
    }

    void CheckRoute(Case which)
    {
        // A-M loops under target-CH greed; the paper's A-S-D route succeeds.
        Network({{0,400,0},{200,50,0},{0,850,0},{600,400,0},{600,300,0},{350,800,0}});
        for (uint32_t i=0; i<3; ++i) SetCluster(i,0,10,i!=0);
        for (uint32_t i=3; i<6; ++i) SetCluster(i,3,20,i!=3);
        NeighborsFromGeometry();
        for (uint16_t i : {1,2})
        {
            m_protocols[0]->m_paperMembers.emplace(i,Simulator::Now());
            m_protocols[0]->m_paperForwarders.emplace(i,Simulator::Now());
        }
        if (which == ROUTING)
        {
            NS_TEST_ASSERT_MSG_EQ(m_protocols[0]->PaperNextHop(m_interfaces.GetAddress(5)),
                                  m_interfaces.GetAddress(2),"CH selects IFN nearest final destination");
            NS_TEST_ASSERT_MSG_EQ(m_protocols[2]->PaperNextHop(m_interfaces.GetAddress(5)),
                                  m_interfaces.GetAddress(5),"IFN completes A-S-D");
            m_protocols[0]->m_paperForwarders.erase(2);
            NS_TEST_ASSERT_MSG_EQ(m_protocols[0]->PaperNextHop(m_interfaces.GetAddress(5)),
                                  m_interfaces.GetAddress(1),"Unregistered CM is not a CH relay");
        }
        else if (which == DIRECT)
        {
            m_nodes.Get(5)->GetObject<MobilityModel>()->SetPosition({350,600,0});
            NeighborsFromGeometry();
            NS_TEST_ASSERT_MSG_EQ(m_protocols[0]->PaperNextHop(m_interfaces.GetAddress(5)),
                                  m_interfaces.GetAddress(5),"Direct destination wins across clusters");
        }
        else
        {
            m_protocols[1]->m_paperForwarder=false;
            NS_TEST_ASSERT_MSG_EQ(m_protocols[1]->PaperNextHop(m_interfaces.GetAddress(3)),
                                  m_interfaces.GetAddress(0),"Common CM forwards to its CH");
            m_protocols[1]->m_paperForwarder=true;
            NS_TEST_ASSERT_MSG_EQ(m_protocols[1]->PaperNextHop(m_interfaces.GetAddress(3)),
                                  m_interfaces.GetAddress(4),"IFN may use other-cluster neighbor");
            m_protocols[1]->m_paperForwarder=false;
            m_protocols[1]->PaperInvalidateNeighbor(0);
            NS_TEST_ASSERT_MSG_EQ(m_protocols[1]->PaperNextHop(m_interfaces.GetAddress(3)),
                                  Ipv4Address(),"Failed CH is not reused or bypassed by generic greed");
        }
    }

    void CheckClustering()
    {
        Network({{0,0,0},{400,0,0},{800,0,0}});
        Simulator::Stop(Seconds(1.95));
        Simulator::Run();
        NS_TEST_ASSERT_MSG_EQ(m_nodes.Get(1)->GetObject<ClusterNode>()->GetRoleType(),Head,
                              "Unique local utility maximum becomes CH");
        for (auto i : {0,2})
        {
            NS_TEST_ASSERT_MSG_EQ(m_nodes.Get(i)->GetObject<ClusterNode>()->GetRoleType(),Member,
                                  "Losing an election must not block CH collection");
            NS_TEST_ASSERT_MSG_EQ(m_protocols[i]->m_paperHeadId,1,"JOIN/ACCEPT completed");
        }
    }

    void CheckBsDelivery()
    {
        Network({{0,0,0},{400,0,0},{-300,200,0},{-300,-200,0},{800,0,0}},true,4);
        for (auto protocol:m_protocols)
            protocol->Configure(true,500,Seconds(10),0.1,2,MilliSeconds(1),4);
        auto sink=Socket::CreateSocket(m_nodes.Get(4),UdpSocketFactory::GetTypeId());
        sink->Bind(InetSocketAddress(Ipv4Address::GetAny(),9900));
        sink->SetRecvCallback(MakeCallback(&IcraFidelityTestCase::Receive,this));
        auto sender=Socket::CreateSocket(m_nodes.Get(0),UdpSocketFactory::GetTypeId());
        sender->Connect(InetSocketAddress(m_interfaces.GetAddress(4),9900));
        Simulator::Schedule(Seconds(3.5),&IcraFidelityTestCase::SendData,this,sender);
        Simulator::Stop(Seconds(4)); Simulator::Run();
        NS_TEST_ASSERT_MSG_EQ(m_protocols[1]->IsInterClusterForwarder(),true,
                              "CM adjacent to the explicit external sink registers as IFN");
        NS_TEST_ASSERT_MSG_EQ(m_protocols[0]->m_paperForwarders.count(1),1,
                              "The CH confirms the endpoint bridge through actual control packets");
        NS_TEST_ASSERT_MSG_EQ(m_received,1,"UDP traverses CH-IFN-external BS");
        NS_TEST_ASSERT_MSG_EQ(m_nodes.Get(4)->GetObject<ClusterNode>()->GetRoleType(),Isolate,
                              "The external sink does not become a UAV relay");
        NS_TEST_ASSERT_MSG_EQ(m_protocols[4]->GetGsReportCount(),0,
                              "The external sink remains outside the learning population");
    }

    void CheckBackbone(bool reconfigure = false)
    {
        Network({{0,0,0},{400,0,0},{-300,0,0},{-300,200,0},{-300,-200,0},
                 {1200,0,0},{800,0,0},{1500,0,0},{1500,200,0},{1500,-200,0}});
        for (auto protocol:m_protocols)
            protocol->Configure(true,500,Seconds(10),0.1,2,MilliSeconds(1));
        auto sink=Socket::CreateSocket(m_nodes.Get(5),UdpSocketFactory::GetTypeId());
        sink->Bind(InetSocketAddress(Ipv4Address::GetAny(),9900));
        sink->SetRecvCallback(MakeCallback(&IcraFidelityTestCase::Receive,this));
        auto sender=Socket::CreateSocket(m_nodes.Get(0),UdpSocketFactory::GetTypeId());
        sender->Connect(InetSocketAddress(m_interfaces.GetAddress(5),9900));
        if (reconfigure)
        {
            Simulator::Schedule(Seconds(3.3),[this]() {
                GsPolicyHeader policy; policy.epoch=1; policy.weights={0.2,0.3,0.25,0.25};
                auto packet=Create<Packet>(); packet->AddHeader(policy);
                packet->AddHeader(TypeHeader(ICRA_GS_POLICY));
                for (auto protocol:m_protocols) protocol->PaperReceivePolicy(packet->Copy());
            });
            Simulator::Schedule(Seconds(3.35),&IcraFidelityTestCase::SendData,this,sender);
            Simulator::Schedule(Seconds(3.4),[this]() {
                // Step 1 keeps the committed topology serving until the round
                // commits. In-flight data must not be queued behind an
                // unnecessary teardown.
                m_sawQueued=!m_protocols[0]->m_paperQueue.empty();
            });
        }
        else
            Simulator::Schedule(Seconds(3.5),&IcraFidelityTestCase::SendData,this,sender);
        Simulator::Stop(Seconds(reconfigure ? 4.6 : 4));
        Simulator::Run();
        NS_TEST_ASSERT_MSG_EQ(m_received,1,"UDP completes the intercluster path");
        if (reconfigure)
        {
            NS_TEST_ASSERT_MSG_EQ(m_sawQueued,false,
                                  "Reconstruction keeps the committed topology until commit");
            NS_TEST_ASSERT_MSG_EQ(m_protocols[0]->m_paperQueue.empty(),true,
                                  "No unnecessary teardown fills the bounded queue");
            NS_TEST_ASSERT_MSG_GT(m_protocols[0]->GetPaperAuditCounters().policyRebuilds,0,
                                  "The strategy update really started a new round");
            return;
        }
        NS_TEST_ASSERT_MSG_EQ(m_protocols[1]->IsInterClusterForwarder(),true,"Bridge CM is IFN");
        NS_TEST_ASSERT_MSG_EQ(m_protocols[6]->IsInterClusterForwarder(),true,"Other bridge is IFN");
        NS_TEST_ASSERT_MSG_EQ(m_protocols[0]->m_paperForwarders.count(1),1,"CH acknowledges IFN");
        m_nodes.Get(6)->GetObject<MobilityModel>()->SetPosition({3000,0,0});
        Simulator::Stop(Seconds(2.2));
        Simulator::Run();
        NS_TEST_ASSERT_MSG_EQ(m_protocols[1]->IsInterClusterForwarder(),false,
                              "Losing the foreign neighbor withdraws the IFN role");
        NS_TEST_ASSERT_MSG_EQ(m_protocols[0]->m_paperForwarders.count(1),0,
                              "The CH removes the withdrawn IFN");
    }

    void DoRun() override
    {
        RngSeedManager::SetSeed(101);
        RngSeedManager::SetRun(7);
        switch (m_case)
        {
        case ACTIONS: CheckActions(); break;
        case REWARD: CheckRewards(); break;
        case METRICS: CheckMetrics(); break;
        case STATE: CheckState(); break;
        case STRATEGY: CheckFeedback(); break;
        case RANDOMNESS: CheckRandomness(); break;
        case PERSISTENCE: CheckPersistence(); break;
        case RESET: CheckReset(); break;
        case WIRE: CheckWire(); break;
        case POLICY: CheckPolicy(); break;
        case EXCLUSION: CheckExclusion(); break;
        case BS_DELIVERY: CheckBsDelivery(); break;
        case RECONFIGURATION: CheckBackbone(true); break;
        case ARP_RECOVERY: CheckArpRecovery(); break;
        case JOIN_BOUND: CheckJoinBound(); break;
        case REJOIN_ONLY: CheckRejoinOnly(); break;
        case SAME_POLICY: CheckSamePolicy(); break;
        case ALL_CH: CheckAllCh(); break;
        case DEMOTION: CheckDemotion(); break;
        case ROUND_UTILITY: CheckRoundUtility(); break;
        case CLUSTER_LIFETIME: CheckClusterLifetime(); break;
        case LEGACY: CheckLegacy(); break;
        case ROUTING: case DIRECT: case MEMBERS: CheckRoute(m_case); break;
        case CLUSTERING: CheckClustering(); break;
        case BACKBONE: CheckBackbone(); break;
        default: break;
        }
        Clean();
    }
};

class IcraTestSuite : public TestSuite
{
  public:
    IcraTestSuite() : TestSuite("icra",UNIT)
    {
        using C=IcraFidelityTestCase;
        AddTestCase(new C(C::ACTIONS,"complete 1771-action space and sparse Algorithm 2 prior"),TestCase::QUICK);
        AddTestCase(new C(C::REWARD,"per-node Eq22-26 rewards and strict role threshold"),TestCase::QUICK);
        AddTestCase(new C(C::METRICS,"true own mobility and configured communication radius"),TestCase::QUICK);
        AddTestCase(new C(C::STATE,"GS entropy state uses reported paper features"),TestCase::QUICK);
        AddTestCase(new C(C::STRATEGY,"received per-node feedback updates Q once per observation window"),TestCase::QUICK);
        AddTestCase(new C(C::RANDOMNESS,"maximum ties and ns3 streams are reproducible"),TestCase::QUICK);
        AddTestCase(new C(C::PERSISTENCE,"sparse paper Q model saves and reloads its own format"),TestCase::QUICK);
        AddTestCase(new C(C::RESET,"Simulator Destroy clears singleton learning state"),TestCase::QUICK);
        AddTestCase(new C(C::WIRE,"serialized GS and IFN messages preserve values and reject malformed input"),TestCase::QUICK);
        AddTestCase(new C(C::ROUTING,"CH selects acknowledged IFN toward final destination"),TestCase::QUICK);
        AddTestCase(new C(C::DIRECT,"one-hop destination precedes cluster routing"),TestCase::QUICK);
        AddTestCase(new C(C::MEMBERS,"ordinary CM and IFN have different candidate sets"),TestCase::QUICK);
        AddTestCase(new C(C::CLUSTERING,"real control packets join nonwinning nodes to CH"),TestCase::QUICK);
        AddTestCase(new C(C::BACKBONE,"IFN registration and intercluster UDP delivery"),TestCase::QUICK);
        AddTestCase(new C(C::POLICY,"received new strategy changes utility and starts reclustering"),TestCase::QUICK);
        AddTestCase(new C(C::EXCLUSION,"passive BS exclusion and GS policy delivery"),TestCase::QUICK);
        AddTestCase(new C(C::BS_DELIVERY,"external BS delivery uses an acknowledged endpoint IFN"),TestCase::QUICK);
        AddTestCase(new C(C::RECONFIGURATION,"strategy reconstruction keeps service and starts a new round"),TestCase::QUICK);
        AddTestCase(new C(C::LEGACY,"runtime legacy preset retains old routing and fixed weights"),TestCase::QUICK);
        AddTestCase(new C(C::ARP_RECOVERY,"adaptive JOIN waits for real cold-ARP recovery"),TestCase::QUICK);
        AddTestCase(new C(C::JOIN_BOUND,"paper defaults and failed JOIN remains bounded"),TestCase::QUICK);
        AddTestCase(new C(C::REJOIN_ONLY,"head loss resumes Step 3 without a new tied CH election"),TestCase::QUICK);
        AddTestCase(new C(C::SAME_POLICY,"equal-weight policy rebuilds and deferred epoch application"),TestCase::QUICK);
        AddTestCase(new C(C::ALL_CH,"all-CH control forwards across WiFi hops while excluding the external BS"),TestCase::QUICK);
        AddTestCase(new C(C::DEMOTION,"a later paper round lets a fallback CH join a better CH"),TestCase::QUICK);
        AddTestCase(new C(C::ROUND_UTILITY,"declared CH utility remains comparable until neighboring members join"),TestCase::QUICK);
        AddTestCase(new C(C::CLUSTER_LIFETIME,"cluster lifetime tracks joins departures moved members and reused head IDs"),TestCase::QUICK);
    }
} g_icraTestSuite;
} // namespace ns3::icra
