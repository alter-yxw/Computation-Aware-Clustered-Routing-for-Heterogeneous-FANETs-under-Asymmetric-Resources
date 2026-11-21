/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef MO_ROUTING_H
#define MO_ROUTING_H
#include "mo-policy.h"
#include "mo-relay-batch.h"
#include "ns3/event-id.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/random-variable-stream.h"
#include "ns3/socket.h"
#include "ns3/traced-callback.h"
#include <fstream>
#include <map>
#include <memory>

namespace ns3::mo
{
// Radio-delivered, flooded link-state adaptation of the author's static graphs.
// No reads of another node's mobility, queues, or simulator-global topology.
class RoutingProtocol : public Ipv4RoutingProtocol
{
  public:
    static TypeId GetTypeId();
    void SetNode(Ptr<Node> node) { m_node = node; }
    void Configure(uint32_t slot, uint32_t nodes, Vertex descriptor,
                   std::shared_ptr<const Policy> policy, const std::string& mode,
                   double slotSeconds, double horizonSlots, double linkRate,
                   const std::string& relayMode = "legacy");
    Ipv4Address SelectDestination(uint32_t taskId, double uploadPackets, double downloadPackets);
    void OpenDecisions(const std::string& path);
    uint64_t GetHelloCount() const { return m_hellos; }
    uint64_t GetControlPacketCount() const { return m_control; }
    double GetControlThroughput() const { return m_controlBytes * 8.0 / 1000; }
    double GetAverageLinkLifetime() const;
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
    struct Record
    {
        uint32_t sequence = 0;
        Time stamp;
        Ipv4Address address;
        Vertex vertex;
        std::vector<uint32_t> neighbors;
    };
    struct Peer { Time stamp, first; };
    bool Fresh(Time stamp) const;
    bool Direct(uint32_t slot) const;
    Ipv4Address Address() const;
    void Start();
    void Hello();
    void Receive(Ptr<Socket>);
    void ReceiveState(StateHeader, Ipv4Address sender, uint32_t bytes, bool bundled);
    void FlushRelayBatch();
    void ClearRelayBatch(const char* reason);
    void Broadcast(Ptr<Packet> packet, bool hello);
    void Refresh();
    Ptr<Ipv4Route> Route(Ipv4Address dst, Ipv4Address gateway) const;
    Ptr<Ipv4Route> LookupRoute(Ptr<Packet>, const Ipv4Header&, Ptr<NetDevice>, Socket::SocketErrno&, bool forwarding);
    void ObserveRoute(Ptr<const Packet>, const Ipv4Header&, const char* stage, const char* outcome);
    // Passive observations only: no scheduling, packet mutation or protocol input.
    TracedCallback<Ptr<const Packet>, const Ipv4Header&, std::string, std::string> m_routeDecision;
    TracedCallback<uint32_t, std::string> m_controlEvent;
    TracedCallback<uint32_t, uint32_t, uint32_t, uint32_t, uint32_t> m_graphSnapshot;
    // reason, current pending depth, record's queue waiting time in ns.
    TracedCallback<std::string, uint32_t, uint64_t> m_relayQueueEvent;
    Ptr<Ipv4> m_ipv4;
    Ptr<Node> m_node;
    Ptr<Socket> m_socket;
    Ptr<UniformRandomVariable> m_jitter;
    uint32_t m_interface = 0, m_slot = 0, m_nodes = 0, m_sequence = 0;
    Vertex m_vertex;
    std::shared_ptr<const Policy> m_policy;
    std::string m_mode;
    double m_slotSeconds = .01, m_horizon = 1000, m_linkRate = 50;
    Time m_interval = Seconds(1), m_hold = Seconds(4), m_cacheUntil;
    double m_serverHelloScale = 1.0;
    EventId m_helloEvent;
    TracedCallback<Ptr<const Packet>, std::string> m_selectedHop;
    bool m_batchRelays = false;
    Time m_relayBatchWindow = MilliSeconds(20);
    bool m_suppressDuplicateRelays = false;
    EventId m_batchEvent;
    RelayBatchQueue m_relayQueue;
    std::vector<EventId> m_events;
    std::map<uint32_t, Record> m_records;
    std::map<uint32_t, Peer> m_peers;
    std::vector<Vertex> m_view;
    Solution m_solution;
    uint64_t m_hellos = 0, m_control = 0, m_controlBytes = 0, m_closedLinks = 0;
    double m_closedSeconds = 0;
    std::ofstream m_decisions;
};
}
#endif
