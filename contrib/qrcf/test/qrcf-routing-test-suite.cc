/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "ns3/qrcf-helper.h"
#include "ns3/qrcf-routing.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/energy-module.h"
#include "ns3/wifi-module.h"
#include "ns3/test.h"

using namespace ns3;
namespace
{
class RoutingTest : public TestCase
{
  public:
    enum Scenario { CHAIN, HOLE, MAC_BREAK };
    explicit RoutingTest(Scenario scenario)
        : TestCase(scenario == CHAIN ? "QRCF real two-hop forwarding and local learning" :
                   scenario == HOLE ? "QRCF local-minimum UDP feedback penalizes the correct action" :
                                      "QRCF final MAC retry failure penalizes the transmitted action"),
          m_scenario(scenario) {}
    void Receive(Ptr<Socket> socket)
    {
        while (auto packet = socket->Recv())
        {
            ++m_delivered;
            if (m_scenario == MAC_BREAK && !m_broken)
            {
                m_broken = true;
                // Wait for the first packet's ACK, then break the actual link.
                // The next transmission uses a still-valid cached Hello and
                // warm ARP entry, so failure occurs at MAC retry exhaustion.
                Simulator::Schedule(MilliSeconds(2),[this] {
                    m_destinationMobility->SetPosition({1000,0,0});
                });
                Simulator::Schedule(MilliSeconds(4),&RoutingTest::Send,this);
            }
        }
    }
    void Send()
    {
        m_sender->SendTo(Create<Packet>(100),0,InetSocketAddress(m_destination,9000));
    }
    void ReceiveLocal(Ptr<Socket> socket)
    { while (socket->Recv()) { ++m_localDelivered; } }
    void DoRun() override
    {
        RngSeedManager::SetSeed(27); RngSeedManager::SetRun(1);
        const bool chain = m_scenario == CHAIN;
        const uint32_t count = m_scenario == MAC_BREAK ? 2 : 3;
        NodeContainer nodes; nodes.Create(count);
        MobilityHelper mobility; mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel"); mobility.Install(nodes);
        nodes.Get(0)->GetObject<MobilityModel>()->SetPosition({0,0,0});
        nodes.Get(1)->GetObject<MobilityModel>()->SetPosition({100,0,0});
        if (count == 3)
        {
            nodes.Get(2)->GetObject<MobilityModel>()->SetPosition(chain ? Vector{200,0,0} : Vector{200,200,0});
        }
        m_destinationMobility = nodes.Get(count-1)->GetObject<MobilityModel>();
        YansWifiChannelHelper channel;
        channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
        channel.AddPropagationLoss("ns3::RangePropagationLossModel","MaxRange",DoubleValue(150));
        YansWifiPhyHelper phy; phy.SetChannel(channel.Create());
        WifiMacHelper mac; mac.SetType("ns3::AdhocWifiMac");
        WifiHelper wifi; wifi.SetStandard(WIFI_STANDARD_80211g);
        wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager","DataMode",StringValue("ErpOfdmRate6Mbps"));
        const auto devices = wifi.Install(phy,mac,nodes);
        wifi.AssignStreams(devices,100);
        BasicEnergySourceHelper energy; energy.Set("BasicEnergySourceInitialEnergyJ",DoubleValue(1000));
        energy.Install(nodes);
        qrcf::Settings settings; settings.ApplyPreset("legacy-noq"); settings.range = 150;
        QrcfHelper helper; helper.SetSettings(settings);
        InternetStackHelper internet; internet.SetRoutingHelper(helper); internet.Install(nodes);
        Ipv4AddressHelper addresses; addresses.SetBase("10.0.0.0","255.255.255.0");
        const auto interfaces = addresses.Assign(devices);
        helper.AssignStreams(nodes,1000);
        for (uint32_t i = 0; i < count; ++i)
        {
            auto protocol = nodes.Get(i)->GetObject<qrcf::RoutingProtocol>();
            // Explicit test-only per-UAV settings prevent identical stationary
            // clocks from locking every periodic broadcast into a collision.
            // There is no unapproved random Hello jitter in the protocol.
            auto perNode = settings; perNode.helloUpper += i*0.125;
            protocol->Configure(nodes.Get(i),perNode);
            protocol->SetDestinationLookup([nodes,interfaces](Ipv4Address destination) -> std::optional<Vector> {
                for (uint32_t j = 0; j < nodes.GetN(); ++j)
                { if (interfaces.GetAddress(j) == destination) { return nodes.Get(j)->GetObject<MobilityModel>()->GetPosition(); } }
                return {};
            });
        }
        m_destination = interfaces.GetAddress(count-1);
        m_receiver = Socket::CreateSocket(nodes.Get(count-1),UdpSocketFactory::GetTypeId());
        m_receiver->Bind(InetSocketAddress(Ipv4Address::GetAny(),9000));
        m_receiver->SetRecvCallback(MakeCallback(&RoutingTest::Receive,this));
        m_sender = Socket::CreateSocket(nodes.Get(0),UdpSocketFactory::GetTypeId());
        m_sender->Bind();
        if (chain)
        {
            m_localReceiver = Socket::CreateSocket(nodes.Get(0),UdpSocketFactory::GetTypeId());
            m_localReceiver->Bind(InetSocketAddress(Ipv4Address::GetAny(),9001));
            m_localReceiver->SetRecvCallback(MakeCallback(&RoutingTest::ReceiveLocal,this));
            Simulator::Schedule(Seconds(2.05),[this,interfaces] {
                m_sender->SendTo(Create<Packet>(30),0,InetSocketAddress(interfaces.GetAddress(0),9001));
            });
        }
        for (uint32_t i = 0; i < 20; ++i)
        { Simulator::Schedule(Seconds(2+i*0.2),&RoutingTest::Send,this); }
        Simulator::Stop(Seconds(7)); Simulator::Run();
        const auto source = nodes.Get(0)->GetObject<qrcf::RoutingProtocol>();
        const auto relay = nodes.Get(1)->GetObject<qrcf::RoutingProtocol>();
        const auto& sourceDiagnostic = source->GetDiagnostics();
        if (chain)
        {
            NS_TEST_ASSERT_MSG_GT(m_delivered,0,"Packets must cross a real intermediate UAV");
            NS_TEST_ASSERT_MSG_EQ(m_localDelivered,1,"Own-IP packets use ordinary loopback delivery");
            NS_TEST_ASSERT_MSG_EQ(source->QTables().count(interfaces.GetAddress(0)),0,"No zero-distance self-routing Q context");
            NS_TEST_ASSERT_MSG_GT(sourceDiagnostic.qUpdates,0,"Source online Q updates");
            NS_TEST_ASSERT_MSG_GT(relay->GetDiagnostics().qUpdates,0,"Relay terminal Q updates");
            NS_TEST_ASSERT_MSG_GT(source->QTables().at(m_destination).Value(interfaces.GetAddress(1).Get()),0,"Learn the relay action");
            NS_TEST_ASSERT_MSG_GT(relay->QTables().at(m_destination).Value(m_destination.Get()),0,"Learn terminal destination");
            for (uint32_t i = 0; i < count; ++i)
            {
                const auto& d = nodes.Get(i)->GetObject<qrcf::RoutingProtocol>()->GetDiagnostics();
                NS_TEST_ASSERT_MSG_EQ(d.controlBytes,d.helloTx*118+d.feedbackTx*45,
                    "No-Q Hellos stay fixed-size even after destination Q tables are learned");
                NS_TEST_ASSERT_MSG_EQ(d.invalidControl,0,"Fixed gamma-only Hellos decode on every receiver");
            }
            const double q = source->QTables().at(m_destination).Value(interfaces.GetAddress(1).Get());
            NS_TEST_ASSERT_MSG_LT_OR_EQ(q,0.5+1e-12,
                "Singleton reward is 0.5; missing peer max Q supplies no bootstrap value");
        }
        else if (m_scenario == HOLE)
        {
            NS_TEST_ASSERT_MSG_EQ(m_delivered,0,"Physical routing hole cannot deliver");
            NS_TEST_ASSERT_MSG_GT(relay->GetDiagnostics().localMinima,0,"Relay detects no closer filtered neighbor");
            NS_TEST_ASSERT_MSG_GT(relay->GetDiagnostics().feedbackTx,0,"Feedback is actually sent");
            NS_TEST_ASSERT_MSG_GT(sourceDiagnostic.feedbackRx,0,"Predecessor receives UDP feedback");
            NS_TEST_ASSERT_MSG_LT(source->QTables().at(m_destination).Value(interfaces.GetAddress(1).Get()),0,"Failed action gets a negative Q");
            const auto& d = relay->GetDiagnostics();
            NS_TEST_ASSERT_MSG_EQ(d.controlTx,d.helloTx+d.feedbackTx,"Both control types are counted");
        }
        else
        {
            NS_TEST_ASSERT_MSG_GT(m_delivered,0,"Warm the actual ARP/data path before breaking it");
            NS_TEST_ASSERT_MSG_GT(sourceDiagnostic.linkFailures,0,"A real MAC retry-limit trace must fire");
            NS_TEST_ASSERT_MSG_LT(source->QTables().at(m_destination).Value(m_destination.Get()),0,"MAC failure penalizes that destination action");
        }
        NS_TEST_ASSERT_MSG_EQ(sourceDiagnostic.invalidControl,0,"Valid learning frames decode successfully");
        // These small control datagrams are unfragmented. Cross-check the
        // legacy payload-only metric against independently traced L3 bytes.
        NS_TEST_ASSERT_MSG_EQ_TOL(source->GetControlThroughput()*1000,
            sourceDiagnostic.controlBytes-28*sourceDiagnostic.controlTx,1e-6,
            "Legacy control metric uses payload kB, excluding UDP/IP and bit conversion");
        m_sender->Close(); m_receiver->Close();
        if (m_localReceiver) { m_localReceiver->Close(); }
        Simulator::Destroy();
        m_sender = nullptr; m_receiver = nullptr; m_localReceiver = nullptr; m_destinationMobility = nullptr;
    }
  private:
    Scenario m_scenario;
    Ptr<Socket> m_sender, m_receiver, m_localReceiver;
    Ptr<MobilityModel> m_destinationMobility;
    Ipv4Address m_destination;
    uint32_t m_delivered = 0;
    uint32_t m_localDelivered = 0;
    bool m_broken = false;
};
class RoutingSuite : public TestSuite
{
  public:
    RoutingSuite() : TestSuite("qrcf-routing",UNIT)
    {
        AddTestCase(new RoutingTest(RoutingTest::CHAIN),TestCase::QUICK);
        AddTestCase(new RoutingTest(RoutingTest::HOLE),TestCase::QUICK);
        AddTestCase(new RoutingTest(RoutingTest::MAC_BREAK),TestCase::QUICK);
    }
} g_routingSuite;
}
