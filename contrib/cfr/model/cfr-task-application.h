/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef CFR_TASK_APPLICATION_H
#define CFR_TASK_APPLICATION_H
#include "cfr-routing.h"
#include "ns3/application.h"
#include <fstream>
#include <set>
#include <string>
#include <tuple>

namespace ns3::cfr
{
// Measurement ledger only. Protocol decisions never read this shared object.
class TaskStatistics
{
  public:
    struct Row
    {
        TaskHeader task;
        std::string status = "pending";
        double finished = 0;
        bool acknowledged = false;
        int64_t ackReceivedNs = -1;
        double sourceComputeCycles = 0;
    };
    void Generated(const TaskHeader& h);
    void Finish(const TaskHeader& h, const std::string& status);
    void Acknowledge(Ipv4Address source, uint32_t id);
    void ObserveAck(Ipv4Address source, uint32_t id);
    void SourceComputed(const TaskHeader& task);
    std::string AckStatus(const Row& row) const;
    uint32_t GeneratedCount() const { return m_rows.size(); }
    uint32_t Count(const std::string& status, bool acknowledgedOnly = false) const;
    const Row* Find(Ipv4Address source, uint32_t id) const;
    double Average(double TaskHeader::*field, bool acknowledgedOnly = false) const;
    double AverageE2e() const;
    void WriteCsv(const std::string& path) const;
    void Print(std::ostream& out) const;
    void OpenReplay(const std::string& path);
    void RegisterAddress(Ipv4Address address, uint16_t slot) { m_slots[address.Get()] = slot; }
    void BeginTransition(const TaskHeader& h, const std::vector<double>& state,
                         const std::vector<double>& parameters, const std::vector<bool>& mask, int chosen);
    void EndTransition(const TaskHeader& h, const std::vector<double>& nextState,
                       const std::vector<bool>& mask, double reward, bool done);
    void AbortTransitions(const TaskHeader& h);
    void FlushReplay() { if (m_replay.is_open()) m_replay.flush(); }
  private:
    struct Transition
    {
        std::vector<double> state, parameters;
        std::vector<bool> mask;
        int chosen = -1, destination = -1;
    };
    std::map<std::pair<uint32_t, uint32_t>, Row> m_rows;
    std::map<std::tuple<uint32_t, uint32_t, uint16_t>, Transition> m_transitions;
    std::map<uint32_t, uint16_t> m_slots;
    std::ofstream m_replay;
};

class TaskApplication : public Application
{
  public:
    static TypeId GetTypeId();
    void Configure(Ptr<RoutingProtocol> routing, std::shared_ptr<TaskStatistics> statistics,
                   double txWatts, uint32_t queueCapacity);
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
    void ComputeDone(TaskHeader h, Ipv4Address next);
    void SendChunk(TaskHeader h, Ipv4Address next, uint32_t bytes);
    void AssemblyTimeout(Key key);
    void Arrive(TaskHeader h);
    void Drop(const TaskHeader& h, const std::string& reason);
    bool Expired(const TaskHeader& h) const;
    bool BitMode() const;
    uint64_t OriginalBits(const TaskHeader& h) const;
    uint64_t RemainingBits(const TaskHeader& h) const;
    uint32_t WireBytes(const TaskHeader& h) const;
    Ptr<RoutingProtocol> m_routing;
    std::shared_ptr<TaskStatistics> m_statistics;
    Ptr<Socket> m_socket;
    Time m_cpuAvailable;
    uint32_t m_queueCapacity = 64, m_cpuReservations = 0;
    double m_txWatts = 1, m_resultRatio = .05, m_k = 1e-28;
    double m_pacingBps = 6e6, m_completionTolerance = 1e-3;
    std::string m_diagnosticCompute = "normal";
    uint32_t m_fragmentBytes = 900, m_maxHops = 64;
    Time m_reassemblyTimeout = Seconds(10);
    bool m_running = false;
    std::map<Key, Assembly> m_assemblies;
    std::map<Key, Time> m_seen;
    std::map<uint32_t, std::function<void(uint32_t)>> m_callbacks;
    std::vector<EventId> m_events;
};
}
#endif
