/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef QRCF_ROUTING_H
#define QRCF_ROUTING_H
#include "qrcf-diagnostics.h"
#include "qrcf-discovery.h"
#include "qrcf-route-planner.h"
#include "ns3/event-id.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/ipv4-l3-protocol.h"
#include "ns3/node.h"
#include "ns3/random-variable-stream.h"
#include "ns3/socket.h"
#include "ns3/wifi-mac.h"
#include "ns3/arp-cache.h"
#include "ns3/arp-l3-protocol.h"
#include <functional>
#include <memory>

namespace ns3::qrcf
{
class RoutingProtocol : public Ipv4RoutingProtocol
{
  public:
    static TypeId GetTypeId();
    RoutingProtocol();
    void Configure(Ptr<Node> node, Settings settings);
    int64_t AssignStreams(int64_t stream);
    using DestinationLookup = std::function<std::optional<Vector>(Ipv4Address)>;
    void SetDestinationLookup(DestinationLookup lookup) { m_destinationLookup = std::move(lookup); }
    bool DataRoutingReady() const { return bool(m_destinationLookup); }
    Ptr<Ipv4Route> RouteOutput(Ptr<Packet>,const Ipv4Header&,Ptr<NetDevice>,Socket::SocketErrno&) override;
    bool RouteInput(Ptr<const Packet>,const Ipv4Header&,Ptr<const NetDevice>,
                    const UnicastForwardCallback&,const MulticastForwardCallback&,
                    const LocalDeliverCallback&,const ErrorCallback&) override;
    void NotifyInterfaceUp(uint32_t) override;
    void NotifyInterfaceDown(uint32_t) override;
    void NotifyAddAddress(uint32_t,Ipv4InterfaceAddress) override;
    void NotifyRemoveAddress(uint32_t,Ipv4InterfaceAddress) override;
    void SetIpv4(Ptr<Ipv4>) override;
    void PrintRoutingTable(Ptr<OutputStreamWrapper>,Time::Unit = Time::S) const override;
    const Diagnostics& GetDiagnostics() const { return m_diagnostics; }
    const NeighborTable& Neighbors() const { return m_neighbors; }
    const Settings& GetSettings() const { return m_settings; }
    const std::map<Ipv4Address,QTable>& QTables() const { return m_qTables; }
    double Epsilon() const { return m_epsilon; }
    double Gamma() const { return m_discovery ? m_discovery->Gamma() : 0; }
    void RefreshObservations() { Purge(); }
    uint64_t GetControlPacketCount() const { return m_diagnostics.controlTx; }
    uint64_t GetHelloCount() const { return m_diagnostics.helloTx; }
    // Legacy benchmark convention: protocol payload kB; caller divides by
    // simulation seconds. Actual IPv4 bytes remain in Diagnostics.
    double GetControlThroughput() const { return m_controlPayloadBytes/1000.0; }
    Ipv4Address Address() const { return m_address.GetLocal(); }
  protected:
    void DoInitialize() override;
    void DoDispose() override;
  private:
    void Hello();
    void Receive(Ptr<Socket>);
    void Purge();
    void ScheduleExpiry();
    void SendFeedback(uint64_t dataId,Ipv4Address destination,Ipv4Address previous);
    void SendValue(uint64_t dataId,Ipv4Address destination,Ipv4Address previous,
                   double maximumQ,bool terminal);
    void AcceptValue(const ValueHeader&,Ipv4Address sender);
    void Penalize(uint64_t dataId,Ipv4Address destination,Ipv4Address next);
    void OnMacDrop(WifiMacDropReason,Ptr<const WifiMpdu>);
    void OnArpDrop(Ptr<const Packet>);
    void OnArpResolutionDrop(Ptr<const Packet>);
    void OnDataDrop(Ptr<const Packet>,bool unavailable,bool arp);
    void OnIpv4Tx(Ptr<const Packet>,Ptr<Ipv4>,uint32_t);
    void OnIpv4Drop(const Ipv4Header&,Ptr<const Packet>,Ipv4L3Protocol::DropReason,Ptr<Ipv4>,uint32_t);
    void OnLocalDeliver(const Ipv4Header&,Ptr<const Packet>,uint32_t);
    void ExpirePending();
    void UpdateQ(Ipv4Address destination,Ipv4Address next,double reward,double alpha,
                 double gamma = 0,double maximumQ = 0);
    Ptr<Ipv4Route> MakeRoute(Ipv4Address destination,Ipv4Address gateway) const;
    Ptr<Ipv4Route> Lookup(Ptr<Packet>,const Ipv4Header&,bool source);
    Ptr<Node> m_node;
    Ptr<Ipv4> m_ipv4;
    Ptr<Ipv4L3Protocol> m_l3;
    Ptr<Socket> m_socket;
    Settings m_settings;
    std::unique_ptr<Discovery> m_discovery;
    NeighborTable m_neighbors;
    Diagnostics m_diagnostics;
    uint64_t m_controlPayloadBytes = 0;
    std::map<Ipv4Address,double> m_previousDistances;
    Ipv4InterfaceAddress m_address;
    int32_t m_interface = -1;
    EventId m_helloEvent, m_expiryEvent;
    EventId m_pendingEvent;
    uint32_t m_sequence = 0;
    uint64_t m_actionSequence = 0;
    Ptr<UniformRandomVariable> m_random;
    Ptr<UniformRandomVariable> m_helloRandom;
    double m_epsilon = 0;
    Ptr<WifiMac> m_mac;
    Ptr<ArpL3Protocol> m_arp;
    Ptr<ArpCache> m_arpCache;
    std::set<Ipv4Address> m_unavailableNeighbors;
    std::map<Ipv4Address,QTable> m_qTables;
    struct PendingAction
    {
        Ipv4Address destination;
        Ipv4Address next;
        double alpha;
        bool penalized;
        double reward;
        Time expires;
    };
    std::map<uint64_t,PendingAction> m_pending;
    DestinationLookup m_destinationLookup;
};
} // namespace ns3::qrcf
#endif
