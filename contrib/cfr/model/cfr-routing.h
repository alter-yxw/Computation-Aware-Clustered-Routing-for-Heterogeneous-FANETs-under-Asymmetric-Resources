/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef CFR_ROUTING_H
#define CFR_ROUTING_H
#include "cfr-packet.h"
#include "cfr-policy.h"
#include "ns3/event-id.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/socket.h"
#include <functional>
#include <map>
#include <memory>

namespace ns3::cfr
{
class RoutingProtocol : public Ipv4RoutingProtocol
{
  public:
    static TypeId GetTypeId();
    void SetNode(Ptr<Node> node) { m_node = node; }
    void Configure(uint16_t slot, double ghz, std::shared_ptr<const Policy> policy,
                   std::function<Vector(Ipv4Address)> location);
    Action Decide(Ipv4Address destination, const TaskState& task,
                  const std::vector<uint16_t>& visited) const;
    std::vector<double> State(Ipv4Address destination, const TaskState& task) const;
    std::vector<double> Parameters(const std::vector<double>& state) const { return m_policy->Parameters(state); }
    std::vector<bool> Mask(const std::vector<uint16_t>& visited) const;
    Ipv4Address NeighborAddress(uint16_t slot) const;
    bool IsDirect(Ipv4Address address) const;
    uint16_t Slot() const { return m_slot; }
    double Ghz() const { return m_ghz; }
    const Policy& GetPolicy() const { return *m_policy; }
    Ipv4Address Address() const;
    uint64_t GetHelloCount() const { return m_helloCount; }
    uint64_t GetControlPacketCount() const { return m_helloCount; }
    double GetControlThroughput() const { return m_controlBytes * 8.0 / 1000; }
    double GetAverageLinkLifetime() const;
    size_t OneHopCount() const;
    size_t TwoHopCount() const;
    Ptr<Ipv4Route> RouteOutput(Ptr<Packet>, const Ipv4Header&, Ptr<NetDevice>, Socket::SocketErrno&) override;
    bool RouteInput(Ptr<const Packet>, const Ipv4Header&, Ptr<const NetDevice>,
                    const UnicastForwardCallback&, const MulticastForwardCallback&,
                    const LocalDeliverCallback&, const ErrorCallback&) override;
    void NotifyInterfaceUp(uint32_t) override;
    void NotifyInterfaceDown(uint32_t) override;
    void NotifyAddAddress(uint32_t, Ipv4InterfaceAddress) override;
    void NotifyRemoveAddress(uint32_t, Ipv4InterfaceAddress) override;
    void SetIpv4(Ptr<Ipv4>) override;
    void PrintRoutingTable(Ptr<OutputStreamWrapper>, Time::Unit = Time::S) const override;

  protected:
    void DoInitialize() override;
    void DoDispose() override;

  private:
    struct Peer
    {
        NeighborRecord record;
        Time firstSeen;
        std::vector<NeighborRecord> advertised;
    };
    bool Fresh(const NeighborRecord&) const;
    bool Usable(const Peer&) const;
    Vector Position() const;
    void Start();
    void SendHello();
    void Receive(Ptr<Socket>);
    void Purge();
    Ptr<Ipv4Route> Route(Ipv4Address destination, Ipv4Address gateway) const;
    Ptr<Ipv4> m_ipv4;
    Ptr<Node> m_node;
    Ptr<Socket> m_socket;
    uint32_t m_interface = 0;
    uint16_t m_slot = 0;
    double m_ghz = 0;
    bool m_requireBidirectional = true;
    Time m_helloInterval = Seconds(1), m_holdTime = Seconds(3);
    EventId m_helloEvent;
    uint32_t m_sequence = 0;
    std::map<uint16_t, Peer> m_peers;
    std::shared_ptr<const Policy> m_policy;
    std::function<Vector(Ipv4Address)> m_location;
    uint64_t m_helloCount = 0, m_controlBytes = 0;
    double m_closedLinkSeconds = 0;
    uint64_t m_closedLinks = 0;
};
}
#endif
