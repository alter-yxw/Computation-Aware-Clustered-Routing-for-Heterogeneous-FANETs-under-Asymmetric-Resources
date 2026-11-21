/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "ns3/qrcf-helper.h"
#include "ns3/qrcf-packet.h"
#include "ns3/qrcf-routing.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/energy-module.h"
#include "ns3/wifi-module.h"
#include "ns3/test.h"
#include <algorithm>
#include <string>
#include <vector>

using namespace ns3;
namespace
{
class PriorityNetworkTest : public TestCase
{
  protected:
    explicit PriorityNetworkTest(const std::string& name) : TestCase(name) {}
    void Install(const std::string& preset,const std::vector<Vector>& positions,double connectionCap=60)
    {
        RngSeedManager::SetSeed(27); RngSeedManager::SetRun(1);
        m_nodes.Create(positions.size());
        MobilityHelper mobility; mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        mobility.Install(m_nodes);
        for (uint32_t i=0;i<m_nodes.GetN();++i) { Move(i,positions[i]); }
        YansWifiChannelHelper channel;
        channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
        channel.AddPropagationLoss("ns3::RangePropagationLossModel","MaxRange",DoubleValue(125));
        YansWifiPhyHelper phy; phy.SetChannel(channel.Create());
        WifiMacHelper mac; mac.SetType("ns3::AdhocWifiMac");
        WifiHelper wifi; wifi.SetStandard(WIFI_STANDARD_80211g);
        wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager","DataMode",StringValue("ErpOfdmRate6Mbps"));
        m_devices=wifi.Install(phy,mac,m_nodes); wifi.AssignStreams(m_devices,100);
        BasicEnergySourceHelper energy; energy.Set("BasicEnergySourceInitialEnergyJ",DoubleValue(1000));
        energy.Install(m_nodes);
        qrcf::Settings settings; settings.ApplyPreset(preset); settings.range=125;
        settings.connectionCap=connectionCap;
        QrcfHelper helper; helper.SetSettings(settings);
        InternetStackHelper internet; internet.SetRoutingHelper(helper); internet.Install(m_nodes);
        Ipv4AddressHelper addresses; addresses.SetBase("10.20.0.0","255.255.255.0");
        m_interfaces=addresses.Assign(m_devices); helper.AssignStreams(m_nodes,1000);
        for (uint32_t i=0;i<m_nodes.GetN();++i)
        {
            Protocol(i)->SetDestinationLookup([this](Ipv4Address destination)->std::optional<Vector> {
                for (uint32_t j=0;j<m_nodes.GetN();++j)
                { if (Address(j)==destination) { return m_nodes.Get(j)->GetObject<MobilityModel>()->GetPosition(); } }
                return {};
            });
            // Passive observation of real IPv4 transmissions, including their
            // actual serialized value receipts. No remote state is injected.
            m_nodes.Get(i)->GetObject<Ipv4L3Protocol>()->TraceConnectWithoutContext("Tx",
                MakeCallback(&PriorityNetworkTest::ObserveWire,this));
        }
    }
    Ptr<qrcf::RoutingProtocol> Protocol(uint32_t i) const
    { return m_nodes.Get(i)->GetObject<qrcf::RoutingProtocol>(); }
    Ipv4Address Address(uint32_t i) const { return m_interfaces.GetAddress(i); }
    void Move(uint32_t i,Vector position)
    { m_nodes.Get(i)->GetObject<MobilityModel>()->SetPosition(position); }
    Ptr<Socket> SocketAt(uint32_t i,uint16_t port=0)
    {
        auto socket=Socket::CreateSocket(m_nodes.Get(i),UdpSocketFactory::GetTypeId());
        socket->Bind(InetSocketAddress(Ipv4Address::GetAny(),port));
        m_sockets.push_back(socket); return socket;
    }
    void Clean()
    {
        for (auto socket:m_sockets) { socket->Close(); }
        Simulator::Destroy();
        m_sockets.clear(); m_nodes=NodeContainer(); m_devices=NetDeviceContainer();
        m_interfaces=Ipv4InterfaceContainer(); m_values.clear();
    }
    struct WireValue
    {
        Time time;
        Ipv4Address sender,receiver;
        qrcf::ValueHeader value;
        uint32_t bytes;
    };
    void ObserveWire(Ptr<const Packet> packet,Ptr<Ipv4>,uint32_t)
    {
        auto copy=packet->Copy(); Ipv4Header ip;
        if (!copy->RemoveHeader(ip) || ip.GetProtocol()!=17) { return; }
        UdpHeader udp;
        if (!copy->RemoveHeader(udp) || udp.GetDestinationPort()!=17930) { return; }
        qrcf::MessageHeader envelope;
        if (!copy->RemoveHeader(envelope) || envelope.kind!=qrcf::MessageHeader::Kind::VALUE) { return; }
        qrcf::ValueHeader value;
        if (copy->RemoveHeader(value) && copy->GetSize()==0)
        { m_values.push_back({Simulator::Now(),ip.GetSource(),ip.GetDestination(),value,packet->GetSize()}); }
    }
    NodeContainer m_nodes;
    NetDeviceContainer m_devices;
    Ipv4InterfaceContainer m_interfaces;
    std::vector<Ptr<Socket>> m_sockets;
    std::vector<WireValue> m_values;
};

class BootstrapWireTest : public PriorityNetworkTest
{
  public:
    BootstrapWireTest():PriorityNetworkTest("QRCF real value receipts bootstrap only the same destination and current filtered actions") {}
    void ReceiveDestination(Ptr<Socket> socket)
    {
        while (socket->Recv())
        {
            if (Simulator::Now()<Seconds(6)) { ++m_warmDelivered; }
            else { ++m_delivered; }
        }
    }
    void ReceiveOtherDestination(Ptr<Socket> socket)
    { while (socket->Recv()) { ++m_otherDelivered; } }
    void DoRun() override
    {
        double immediateQ=0,bellmanQ=0;
        for (const std::string preset:{"outcome-immediate","outcome-bellman"})
        {
            m_warmDelivered=0; m_delivered=0; m_otherDelivered=0;
            // Initially B can reach D only via A,C. Later A moves behind B,
            // leaving B's learned A action outside B's destination cylinder.
            Install(preset,{{100,0,0},{0,0,0},{200,0,0},{300,0,0}});
            auto destination=SocketAt(3,9000);
            destination->SetRecvCallback(MakeCallback(&BootstrapWireTest::ReceiveDestination,this));
            auto other=SocketAt(0,9001);
            other->SetRecvCallback(MakeCallback(&BootstrapWireTest::ReceiveOtherDestination,this));
            auto training=SocketAt(1),source=SocketAt(0);
            for (uint32_t i=0;i<20;++i)
            {
                Simulator::Schedule(Seconds(2+i*0.1),[this,training] {
                    training->SendTo(Create<Packet>(80),0,InetSocketAddress(Address(3),9000));
                });
            }
            // A second destination has a positive Q entry at B as well.
            for (uint32_t i=0;i<3;++i)
            {
                Simulator::Schedule(Seconds(4.3+i*0.15),[this,training] {
                    training->SendTo(Create<Packet>(80),0,InetSocketAddress(Address(0),9001));
                });
            }
            Simulator::Schedule(Seconds(5),[this] { Move(0,{0,0,0}); Move(1,{100,0,0}); });
            bool oldActionPositive=false,otherDestinationPositive=false,newActionUnvisited=false;
            Simulator::Schedule(Seconds(6.9),[this,&oldActionPositive,&otherDestinationPositive,&newActionUnvisited] {
                const auto& tables=Protocol(1)->QTables();
                const auto same=tables.find(Address(3)),other=tables.find(Address(0));
                oldActionPositive=same!=tables.end() && same->second.Value(Address(0).Get())>0;
                otherDestinationPositive=other!=tables.end() && other->second.Value(Address(0).Get())>0;
                newActionUnvisited=same!=tables.end() && same->second.Value(Address(2).Get())==0;
            });
            for (uint32_t i=0;i<20;++i)
            {
                Simulator::Schedule(Seconds(7+i*0.2),[this,source] {
                    source->SendTo(Create<Packet>(100),0,InetSocketAddress(Address(3),9000));
                });
            }
            Simulator::Stop(Seconds(12)); Simulator::Run();
            NS_TEST_ASSERT_MSG_GT(m_warmDelivered,0,"The excluded action must be trained by real multihop traffic");
            NS_TEST_ASSERT_MSG_GT(m_otherDelivered,0,"The foreign destination must be trained by real terminal receipts");
            NS_TEST_ASSERT_MSG_EQ(oldActionPositive,true,"Keep positive history for the now-excluded same-destination action");
            NS_TEST_ASSERT_MSG_EQ(otherDestinationPositive,true,"Keep positive history for a different destination");
            NS_TEST_ASSERT_MSG_EQ(newActionUnvisited,true,"The newly admissible action starts at Q=0");
            NS_TEST_ASSERT_MSG_GT(m_delivered,0,"Equal stationary Hello settings must discover and recover the changed chain");
            const auto first=std::find_if(m_values.begin(),m_values.end(),[this](const WireValue& v) {
                return v.time>=Seconds(7) && v.sender==Address(1) && v.receiver==Address(0) &&
                       v.value.destination==Address(3) && !v.value.IsTerminal();
            });
            NS_TEST_ASSERT_MSG_EQ((first!=m_values.end()),true,"The relay must transmit an actual destination-scoped value receipt");
            if (first!=m_values.end())
            {
                NS_TEST_ASSERT_MSG_EQ_TOL(first->value.maximumQ,0,1e-12,
                    "Exclude old out-of-filter actions and other destinations from the receipt's maximum");
            }
            bool positiveBootstrap=false,terminalReceipt=false,stableGamma=false;
            for (const auto& v:m_values)
            {
                NS_TEST_ASSERT_MSG_EQ(v.bytes,61,"Every actual value receipt costs 61 IPv4 bytes");
                terminalReceipt=terminalReceipt || v.value.IsTerminal();
                if (v.time>=Seconds(7) && v.sender==Address(1) && v.receiver==Address(0) && v.value.destination==Address(3))
                {
                    positiveBootstrap=positiveBootstrap || v.value.maximumQ>0;
                    stableGamma=stableGamma || v.value.gamma>0.9;
                }
            }
            NS_TEST_ASSERT_MSG_EQ(terminalReceipt,true,"Destination delivery must emit a real terminal receipt");
            NS_TEST_ASSERT_MSG_EQ(positiveBootstrap,true,"Later relay receipts must carry learned continuation values");
            NS_TEST_ASSERT_MSG_EQ(stableGamma,true,"Stationary common settings must provide the relay's stable-neighbor gamma");
            const auto table=Protocol(0)->QTables().find(Address(3));
            NS_TEST_ASSERT_MSG_EQ((table!=Protocol(0)->QTables().end()),true,"Source learns the final destination context");
            const double q=table==Protocol(0)->QTables().end()?0:table->second.Value(Address(1).Get());
            if (preset=="outcome-immediate")
            {
                immediateQ=q;
                NS_TEST_ASSERT_MSG_GT(q,0,"Successful receipts train the immediate-reward control");
                NS_TEST_ASSERT_MSG_LT_OR_EQ(q,0.5+1e-12,"Immediate mode ignores the positive continuation value on its wire");
            }
            else
            {
                bellmanQ=q;
                NS_TEST_ASSERT_MSG_GT(q,0.5+1e-6,"Real terminal values must propagate upstream beyond immediate reward");
            }
            for (uint32_t i=0;i<m_nodes.GetN();++i)
            { NS_TEST_ASSERT_MSG_EQ(Protocol(i)->GetDiagnostics().invalidControl,0,"All receipt and Hello frames decode"); }
            Clean();
        }
        NS_TEST_ASSERT_MSG_GT(bellmanQ,immediateQ,"Bootstrap changes learning while both outcome modes pay for the same receipt format");
    }
  private:
    uint32_t m_warmDelivered=0,m_delivered=0,m_otherDelivered=0;
};

class OutcomeFailureTest : public PriorityNetworkTest
{
  public:
    explicit OutcomeFailureTest(bool cold)
        :PriorityNetworkTest(cold?"QRCF cold ARP loss applies one negative update without optimistic training":
            "QRCF MAC failure blocks a cached direct action until a real new Hello restores it"),m_cold(cold) {}
    void ReceiveDestination(Ptr<Socket> socket)
    {
        while (socket->Recv())
        {
            ++m_delivered;
            if (!m_cold && m_delivered==1)
            {
                Simulator::Schedule(MilliSeconds(1),&OutcomeFailureTest::BreakWhenConfirmed,this);
            }
        }
    }
    void BreakWhenConfirmed()
    {
        // The reverse value receipt may itself need ARP. Observe its actual
        // learning outcome before breaking the now-warm data path.
        if (Protocol(0)->GetDiagnostics().qUpdates==0)
        {
            if (Simulator::Now()<Seconds(8)) { Simulator::Schedule(MilliSeconds(1),&OutcomeFailureTest::BreakWhenConfirmed,this); }
            return;
        }
        Simulator::Schedule(MilliSeconds(2),[this] { Move(1,{1000,0,0}); });
        Simulator::Schedule(MilliSeconds(4),&OutcomeFailureTest::SendFailure,this);
        Simulator::Schedule(MilliSeconds(200),&OutcomeFailureTest::CheckBlocked,this);
        Simulator::Schedule(MilliSeconds(350),[this] {
            Move(1,{100,0,0}); m_restoredAt=Simulator::Now(); TryRecovered();
        });
    }
    void ObserveArpDrop(Ptr<const Packet> packet)
    {
        auto copy=packet->Copy(); Ipv4Header ip; UdpHeader udp;
        if (copy->RemoveHeader(ip) && ip.GetProtocol()==17 && copy->RemoveHeader(udp) &&
            udp.GetDestinationPort()==9000) { ++m_arpDrops; }
    }
    void StartWhenFresh()
    {
        const auto entry=Protocol(0)->Neighbors().Entries().find(Address(1));
        if (entry==Protocol(0)->Neighbors().Entries().end() || entry->second.expires-Simulator::Now()<MilliSeconds(500))
        {
            if (Simulator::Now()<Seconds(8)) { Simulator::Schedule(MilliSeconds(10),&OutcomeFailureTest::StartWhenFresh,this); }
            return;
        }
        m_started=true;
        if (m_cold)
        {
            auto cache=m_nodes.Get(0)->GetObject<Ipv4L3Protocol>()->GetInterface(1)->GetArpCache();
            const auto arp=cache->Lookup(Address(1));
            m_coldAtSend=!arp || !arp->IsAlive();
            Move(1,{1000,0,0}); SendFailure();
        }
        else { m_sender->SendTo(Create<Packet>(100),0,InetSocketAddress(Address(1),9000)); }
    }
    void SendFailure()
    {
        m_beforeFailure=Protocol(0)->GetDiagnostics().qUpdates;
        const int sent=m_sender->SendTo(Create<Packet>(100),0,InetSocketAddress(Address(1),9000));
        m_failedSendAccepted=sent>=0;
        m_noOptimisticUpdate=Protocol(0)->GetDiagnostics().qUpdates==m_beforeFailure;
    }
    void CheckBlocked()
    {
        const auto protocol=Protocol(0);
        NS_TEST_ASSERT_MSG_EQ(protocol->GetDiagnostics().qUpdates,m_beforeFailure+1,
            "Exactly one real failure outcome updates the failed transmission");
        const auto context=protocol->QTables().find(Address(1));
        NS_TEST_ASSERT_MSG_EQ((context!=protocol->QTables().end()),true,"Failure creates the destination Q context");
        if (context!=protocol->QTables().end())
        { NS_TEST_ASSERT_MSG_LT(context->second.Value(Address(1).Get()),0,"A single failure leaves the direct action negative"); }
        const auto entry=protocol->Neighbors().Entries().find(Address(1));
        m_cacheValidAtBlock=entry!=protocol->Neighbors().Entries().end() && entry->second.expires>Simulator::Now();
        const auto before=protocol->GetDiagnostics().qUpdates;
        m_directBlocked=m_sender->SendTo(Create<Packet>(100),0,InetSocketAddress(Address(1),9000))<0;
        NS_TEST_ASSERT_MSG_EQ(protocol->GetDiagnostics().qUpdates,before,"A rejected direct retry must not invent a learning outcome");
    }
    void TryRecovered()
    {
        const auto entry=Protocol(0)->Neighbors().Entries().find(Address(1));
        if (entry!=Protocol(0)->Neighbors().Entries().end() && entry->second.received>m_restoredAt)
        {
            m_recoveryHello=true;
            m_sender->SendTo(Create<Packet>(100),0,InetSocketAddress(Address(1),9000)); return;
        }
        if (Simulator::Now()<Seconds(13)) { Simulator::Schedule(MilliSeconds(50),&OutcomeFailureTest::TryRecovered,this); }
    }
    void CheckColdFailureAndRestore()
    {
        const auto protocol=Protocol(0);
        m_coldFailureChecked=true;
        NS_TEST_ASSERT_MSG_GT(m_arpDrops,0,"Observe actual ARP exhaustion before restoring the link");
        NS_TEST_ASSERT_MSG_EQ(protocol->GetDiagnostics().qUpdates,1,"One cold ARP loss produces exactly one update");
        const auto context=protocol->QTables().find(Address(1));
        NS_TEST_ASSERT_MSG_EQ((context!=protocol->QTables().end()),true,"ARP loss must train its pending action");
        if (context!=protocol->QTables().end())
        { NS_TEST_ASSERT_MSG_LT(context->second.Value(Address(1).Get()),0,"Cold ARP loss is negative before restoration"); }
        NS_TEST_ASSERT_MSG_EQ(m_delivered,0,"No successful terminal outcome precedes the cold ARP failure");
        Move(1,{100,0,0}); m_restoredAt=Simulator::Now(); TryRecovered();
    }
    void DoRun() override
    {
        // A short CT cap must not expire the token before real ARP exhaustion.
        Install("outcome-bellman",{{0,0,0},{100,0,0}},m_cold?1:60);
        auto receiver=SocketAt(1,9000);
        receiver->SetRecvCallback(MakeCallback(&OutcomeFailureTest::ReceiveDestination,this));
        m_sender=SocketAt(0);
        // ns-3 traces retry exhaustion on ArpCache and immediate queue/dead
        // entry drops on ArpL3Protocol. Both are genuine stack outcomes.
        m_nodes.Get(0)->GetObject<ArpL3Protocol>()->TraceConnectWithoutContext("Drop",
            MakeCallback(&OutcomeFailureTest::ObserveArpDrop,this));
        m_nodes.Get(0)->GetObject<Ipv4L3Protocol>()->GetInterface(1)->GetArpCache()->TraceConnectWithoutContext("Drop",
            MakeCallback(&OutcomeFailureTest::ObserveArpDrop,this));
        Simulator::Schedule(Seconds(2),&OutcomeFailureTest::StartWhenFresh,this);
        if (m_cold) { Simulator::Schedule(Seconds(9),&OutcomeFailureTest::CheckColdFailureAndRestore,this); }
        Simulator::Stop(Seconds(14)); Simulator::Run();
        NS_TEST_ASSERT_MSG_EQ(m_started,true,"Common stationary Hello settings must discover the peer");
        NS_TEST_ASSERT_MSG_EQ(m_failedSendAccepted,true,"A real cached data route must reach the failing stack path");
        NS_TEST_ASSERT_MSG_EQ(m_noOptimisticUpdate,true,"Sending before any actual outcome must not train Q");
        if (m_cold)
        {
            NS_TEST_ASSERT_MSG_EQ(m_coldAtSend,true,"The failed data packet must start with cold ARP");
            NS_TEST_ASSERT_MSG_GT(m_arpDrops,0,"ARP retry exhaustion must be observed in the real stack");
            NS_TEST_ASSERT_MSG_EQ(m_coldFailureChecked,true,"Check the negative outcome before restoring the peer");
            NS_TEST_ASSERT_MSG_EQ(m_recoveryHello,true,"A new real Hello must restore discovery after cold ARP loss");
            NS_TEST_ASSERT_MSG_EQ(m_delivered,1,"The dead ARP entry must permit real resolution and delivery after new Hello");
            NS_TEST_ASSERT_MSG_EQ(Protocol(0)->GetDiagnostics().qUpdates,2,"Cold ARP failure and recovered delivery each update once");
        }
        else
        {
            NS_TEST_ASSERT_MSG_GT(Protocol(0)->GetDiagnostics().linkFailures,0,"Warm ARP loss must reach the actual MAC retry limit");
            NS_TEST_ASSERT_MSG_EQ(m_cacheValidAtBlock,true,"The direct-bypass check must run before the old Hello expires");
            NS_TEST_ASSERT_MSG_EQ(m_directBlocked,true,"A cached destination cannot bypass a recorded link failure");
            NS_TEST_ASSERT_MSG_EQ(m_recoveryHello,true,"Only a newly received Hello restores the failed neighbor");
            NS_TEST_ASSERT_MSG_GT_OR_EQ(m_delivered,2,"The restored peer must receive another real data packet");
            NS_TEST_ASSERT_MSG_EQ(Protocol(0)->GetDiagnostics().qUpdates,m_beforeFailure+2,
                "Warm success, failure, and recovery each update exactly once");
        }
        Clean(); m_sender=nullptr;
    }
  private:
    bool m_cold;
    Ptr<Socket> m_sender;
    Time m_restoredAt;
    uint64_t m_beforeFailure=0;
    uint32_t m_delivered=0,m_arpDrops=0;
    bool m_started=false,m_coldAtSend=false,m_failedSendAccepted=false,m_noOptimisticUpdate=false;
    bool m_cacheValidAtBlock=false,m_directBlocked=false,m_recoveryHello=false;
    bool m_coldFailureChecked=false;
};

class StationaryDiscoveryTest : public PriorityNetworkTest
{
  public:
    StationaryDiscoveryTest():PriorityNetworkTest("QRCF twelve truly stationary UAVs discover every peer with common Hello parameters") {}
    void ReceiveData(Ptr<Socket> socket)
    {
        for (uint32_t i=0;i<m_receivers.size();++i)
        {
            if (m_receivers[i]!=socket) { continue; }
            while (socket->Recv()) { ++m_delivered[i]; }
            break;
        }
    }
    void ObserveDiscovery()
    {
        for (uint32_t i=0;i<m_nodes.GetN();++i)
        {
            Protocol(i)->RefreshObservations();
            if (Protocol(i)->Neighbors().Entries().size()==m_nodes.GetN()-1) { m_complete[i]=true; }
        }
        if (!m_previewChecked && m_complete[0])
        {
            Ipv4Header header; header.SetDestination(Address(6)); header.SetProtocol(17);
            Socket::SocketErrno error;
            const auto before=Protocol(0)->GetDiagnostics().qUpdates;
            const auto route=Protocol(0)->RouteOutput(nullptr,header,nullptr,error);
            NS_TEST_ASSERT_MSG_EQ((route!=nullptr),true,"A preview can resolve a discovered direct peer");
            NS_TEST_ASSERT_MSG_EQ(Protocol(0)->GetDiagnostics().qUpdates,before,"A route preview cannot train Q");
            m_previewChecked=true;
        }
        if (Simulator::Now()<Seconds(11)) { Simulator::Schedule(MilliSeconds(100),&StationaryDiscoveryTest::ObserveDiscovery,this); }
    }
    void DoRun() override
    {
        std::vector<Vector> positions;
        for (uint32_t i=0;i<12;++i) { positions.push_back({15.0*(i%4),15.0*(i/4),0}); }
        Install("outcome-bellman",positions);
        m_complete.assign(12,false); m_delivered.assign(12,0);
        for (uint32_t i=0;i<12;++i)
        {
            auto receiver=SocketAt(i,9000); receiver->SetRecvCallback(MakeCallback(&StationaryDiscoveryTest::ReceiveData,this));
            m_receivers.push_back(receiver);
            auto sender=SocketAt(i);
            for (uint32_t attempt=0;attempt<3;++attempt)
            {
                Simulator::Schedule(Seconds(5+attempt*2+i*0.03),[this,sender,i] {
                    sender->SendTo(Create<Packet>(100),0,InetSocketAddress(Address((i+6)%12),9000));
                });
            }
        }
        Simulator::Schedule(Seconds(2),&StationaryDiscoveryTest::ObserveDiscovery,this);
        Simulator::Stop(Seconds(12)); Simulator::Run();
        for (uint32_t i=0;i<12;++i)
        {
            NS_TEST_ASSERT_MSG_EQ_TOL(m_nodes.Get(i)->GetObject<MobilityModel>()->GetVelocity().GetLength(),0,0,
                "Use ConstantPositionMobilityModel, not a random mobility model with zero mean speed");
            NS_TEST_ASSERT_MSG_EQ(m_complete[i],true,"Every stationary UAV must observe all eleven peers in its current valid table");
            NS_TEST_ASSERT_MSG_GT(m_delivered[i],0,"Every stationary destination must receive actual UDP traffic");
            NS_TEST_ASSERT_MSG_GT(Protocol(i)->GetDiagnostics().qUpdates,0,"Every source must process actual delivery outcomes");
        }
        NS_TEST_ASSERT_MSG_EQ(m_previewChecked,true,"Exercise a real no-training route preview");
        Clean(); m_receivers.clear();
    }
  private:
    std::vector<Ptr<Socket>> m_receivers;
    std::vector<bool> m_complete;
    std::vector<uint32_t> m_delivered;
    bool m_previewChecked=false;
};

class LostReceiptTest : public PriorityNetworkTest
{
  public:
    LostReceiptTest():PriorityNetworkTest("QRCF a delivered packet with a lost value receipt expires without positive or negative training") {}
    void ReceiveData(Ptr<Socket> socket) { while (socket->Recv()) { ++m_delivered; } }
    void Acknowledged(Ptr<const WifiMpdu> mpdu)
    {
        if (m_dropReceipts || !mpdu->GetHeader().IsData()) { return; }
        auto copy=mpdu->GetPacket()->Copy(); LlcSnapHeader llc; Ipv4Header ip; UdpHeader udp;
        if (!copy->RemoveHeader(llc) || llc.GetType()!=0x0800 || !copy->RemoveHeader(ip) ||
            ip.GetProtocol()!=17 || !copy->RemoveHeader(udp) || udp.GetDestinationPort()!=9000) { return; }
        // The actual data frame has already received its MAC ACK. Corrupt only
        // subsequent reception at the source, including the terminal receipt.
        // This must not turn missing feedback into a data-failure penalty.
        auto errors=CreateObject<RateErrorModel>(); errors->SetUnit(RateErrorModel::ERROR_UNIT_PACKET); errors->SetRate(1);
        DynamicCast<WifiNetDevice>(m_devices.Get(0))->GetPhy()->SetPostReceptionErrorModel(errors);
        m_dropReceipts=true;
    }
    void SendWhenDiscovered()
    {
        if (!Protocol(0)->Neighbors().Entries().count(Address(1)))
        {
            if (Simulator::Now()<Seconds(5)) { Simulator::Schedule(MilliSeconds(20),&LostReceiptTest::SendWhenDiscovered,this); }
            return;
        }
        m_sender->SendTo(Create<Packet>(100),0,InetSocketAddress(Address(1),9000));
        NS_TEST_ASSERT_MSG_EQ(Protocol(0)->GetDiagnostics().qUpdates,0,"Sending without an outcome does not train Q");
    }
    void DoRun() override
    {
        Install("outcome-bellman",{{0,0,0},{100,0,0}});
        auto receiver=SocketAt(1,9000); receiver->SetRecvCallback(MakeCallback(&LostReceiptTest::ReceiveData,this));
        m_sender=SocketAt(0);
        DynamicCast<WifiNetDevice>(m_devices.Get(0))->GetMac()->TraceConnectWithoutContext("AckedMpdu",
            MakeCallback(&LostReceiptTest::Acknowledged,this));
        Simulator::Schedule(Seconds(2),&LostReceiptTest::SendWhenDiscovered,this);
        Simulator::Stop(Seconds(66)); Simulator::Run();
        NS_TEST_ASSERT_MSG_EQ(m_delivered,1,"The data packet must reach the real destination");
        NS_TEST_ASSERT_MSG_EQ(m_dropReceipts,true,"Begin corruption only after actual MAC acknowledgement of the data");
        NS_TEST_ASSERT_MSG_GT(Protocol(1)->GetDiagnostics().valueTx,0,"The destination must send a real value receipt");
        NS_TEST_ASSERT_MSG_EQ(Protocol(0)->GetDiagnostics().valueRx,0,"The source cannot learn from a corrupted receipt");
        NS_TEST_ASSERT_MSG_GT(Protocol(0)->GetDiagnostics().outcomeTimeouts,0,"The unconfirmed outcome eventually expires");
        NS_TEST_ASSERT_MSG_EQ(Protocol(0)->GetDiagnostics().qUpdates,0,"A lost receipt proves neither data success nor data failure");
        NS_TEST_ASSERT_MSG_EQ(Protocol(0)->GetDiagnostics().linkFailures,0,"The acknowledged data frame must not be marked failed");
        Clean(); m_sender=nullptr;
    }
  private:
    Ptr<Socket> m_sender;
    uint32_t m_delivered=0;
    bool m_dropReceipts=false;
};
class LocalMinimumOutcomeTest : public PriorityNetworkTest
{
  public:
    LocalMinimumOutcomeTest():PriorityNetworkTest("QRCF local-minimum feedback resolves one outcome without a prior positive update") {}
    void SendWhenDiscovered()
    {
        if (!Protocol(0)->Neighbors().Entries().count(Address(1)) ||
            !Protocol(1)->Neighbors().Entries().count(Address(0)))
        {
            if (Simulator::Now()<Seconds(5))
            { Simulator::Schedule(MilliSeconds(20),&LocalMinimumOutcomeTest::SendWhenDiscovered,this); }
            return;
        }
        m_accepted=m_sender->SendTo(Create<Packet>(100),0,InetSocketAddress(Address(2),9000))>=0;
        NS_TEST_ASSERT_MSG_EQ(Protocol(0)->GetDiagnostics().qUpdates,0,"No positive action-time update precedes feedback");
    }
    void DoRun() override
    {
        Install("outcome-bellman",{{0,0,0},{100,0,0},{300,0,0}});
        m_sender=SocketAt(0);
        Simulator::Schedule(Seconds(2),&LocalMinimumOutcomeTest::SendWhenDiscovered,this);
        Simulator::Stop(Seconds(8)); Simulator::Run();
        NS_TEST_ASSERT_MSG_EQ(m_accepted,true,"A real source route must forward to the dead-end relay");
        NS_TEST_ASSERT_MSG_EQ(Protocol(0)->GetDiagnostics().feedbackRx,1,"One real failure feedback reaches the predecessor");
        NS_TEST_ASSERT_MSG_EQ(Protocol(0)->GetDiagnostics().qUpdates,1,"Feedback resolves the source action once");
        const auto table=Protocol(0)->QTables().find(Address(2));
        NS_TEST_ASSERT_MSG_EQ((table!=Protocol(0)->QTables().end()),true,"Failed destination retains learned state");
        if (table!=Protocol(0)->QTables().end())
        { NS_TEST_ASSERT_MSG_LT(table->second.Value(Address(1).Get()),0,"Local minima give Rmin without optimistic pre-training"); }
        NS_TEST_ASSERT_MSG_EQ(Protocol(1)->GetDiagnostics().valueTx,0,"The dead-end relay must not emit a successful TRANSIT receipt");
        Clean(); m_sender=nullptr;
    }
  private:
    Ptr<Socket> m_sender;
    bool m_accepted=false;
};
class RandomStreamIsolationTest : public TestCase
{
  public:
    RandomStreamIsolationTest():TestCase("QRCF explicit jitter streams do not consume automatic streams used by ARP") {}
    void DoRun() override
    {
        for (const std::string preset:{"legacy-noq","staggered-noq","outcome-immediate","outcome-bellman"})
        {
            auto node=CreateObject<Node>();
            auto protocol=CreateObject<qrcf::RoutingProtocol>();
            qrcf::Settings settings; settings.ApplyPreset(preset);
            const uint64_t before=RngSeedManager::GetNextStreamIndex();
            protocol->Configure(node,settings);
            const auto used=protocol->AssignStreams(9300);
            const uint64_t after=RngSeedManager::GetNextStreamIndex();
            NS_TEST_ASSERT_MSG_EQ(used,1,"Keep the historical action-stream stride");
            NS_TEST_ASSERT_MSG_EQ(after,before+1,"Only the two observation probes may consume an automatic stream index");
            protocol->Dispose();
        }
        Simulator::Destroy();
    }
};
class PrioritySuite : public TestSuite
{
  public:
    PrioritySuite():TestSuite("qrcf-priority",UNIT)
    {
        AddTestCase(new BootstrapWireTest,TestCase::QUICK);
        AddTestCase(new OutcomeFailureTest(false),TestCase::QUICK);
        AddTestCase(new OutcomeFailureTest(true),TestCase::QUICK);
        AddTestCase(new StationaryDiscoveryTest,TestCase::QUICK);
        AddTestCase(new LostReceiptTest,TestCase::QUICK);
        AddTestCase(new LocalMinimumOutcomeTest,TestCase::QUICK);
        AddTestCase(new RandomStreamIsolationTest,TestCase::QUICK);
    }
} g_prioritySuite;
}
