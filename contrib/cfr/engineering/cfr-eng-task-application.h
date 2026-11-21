/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef CFR_ENG_TASK_APPLICATION_H
#define CFR_ENG_TASK_APPLICATION_H
#include "cfr-eng-routing.h"
#include "ns3/cfr-task-application.h"
#include "ns3/application.h"
#include <fstream>
#include <set>
#include <string>
#include <tuple>

namespace ns3::cfreng
{
using cfr::TaskHeader;
using cfr::TaskStatistics;
class TaskApplication : public Application
{
  public:
    static TypeId GetTypeId();
    void Configure(Ptr<RoutingProtocol> routing, std::shared_ptr<TaskStatistics> statistics,
                   double txWatts, uint32_t queueCapacity);
    void ConfigureEndToEnd(uint16_t slot, double ghz, Ipv4Address address,
                           std::shared_ptr<TaskStatistics> statistics, double watts, uint32_t capacity);
    void SetStrategy(const std::string& strategy);
    // Local evaluation contract: deadlines classify source receipts; they do
    // not cancel work, prune forwarding or remove late-completion callbacks.
    void SetEvaluationDeadlineOnly(bool enabled) { m_evaluationDeadlineOnly = enabled; }
    using ExecutionObserver = std::function<void(const std::string&, const TaskHeader&)>;
    void SetExecutionObserver(ExecutionObserver observer) { m_executionObserver = std::move(observer); }
    // Preserve a network protocol's task classification without changing bytes/service.
    using PacketDecorator = std::function<void(Ptr<Packet>, const TaskHeader&)>;
    void SetPacketDecorator(PacketDecorator decorate) { m_decorate = std::move(decorate); }
    void Submit(uint32_t id, double cycles, uint32_t bytes, Ipv4Address destination,
                Time deadline, std::function<void(uint32_t)> complete);
  protected:
    void StartApplication() override;
    void StopApplication() override;
    void DoDispose() override;
  private:
    using Key = std::tuple<uint32_t, uint32_t, uint16_t>;
    struct Assembly
    {
        TaskHeader task;
        std::vector<bool> received;
        uint32_t count = 0, bytes = 0;
        EventId timeout;
    };
    void Receive(Ptr<Socket> socket);
    void Process(TaskHeader h);
    void ComputeDone(TaskHeader h);
    void SendChunk(TaskHeader h, Ipv4Address next, uint32_t bytes);
    bool AwaitingArp(Ipv4Address next) const;
    void AssemblyTimeout(Key key);
    void Arrive(TaskHeader h);
    void Drop(const TaskHeader& h, const std::string& reason);
    bool Expired(const TaskHeader& h) const;
    Ipv4Address Address() const { return m_routing ? m_routing->Address() : m_address; }
    double Ghz() const { return m_routing ? m_routing->Ghz() : m_ghz; }
    uint16_t Slot() const { return m_routing ? m_routing->Slot() : m_slot; }
    bool ShouldCompute(const TaskHeader& h, double wait) const;
    uint32_t WireBytes(const TaskHeader& h) const;
    Ptr<RoutingProtocol> m_routing;
    std::shared_ptr<TaskStatistics> m_statistics;
    Ptr<Socket> m_socket;
    Time m_cpuAvailable;
    uint32_t m_queueCapacity = 64, m_cpuReservations = 0;
    double m_txWatts = 1, m_resultRatio = .05, m_k = 1e-28;
    double m_pacingBps = 6e6;
    std::string m_strategy = "joint";
    PacketDecorator m_decorate;
    Ipv4Address m_address;
    double m_ghz = 0;
    uint16_t m_slot = 0;
    uint32_t m_fragmentBytes = 900, m_maxHops = 64;
    Time m_reassemblyTimeout = Seconds(10);
    bool m_running = false;
    bool m_evaluationDeadlineOnly = false;
    ExecutionObserver m_executionObserver;
    std::map<Key, Assembly> m_assemblies;
    std::map<Key, Time> m_seen;
    std::map<uint32_t, std::function<void(uint32_t)>> m_callbacks;
    std::map<uint32_t, TaskHeader> m_submitted;
    std::vector<EventId> m_events;
};
}
#endif
