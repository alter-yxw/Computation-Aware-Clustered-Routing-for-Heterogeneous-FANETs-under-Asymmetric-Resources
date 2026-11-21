/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef CAFBCR_COMPUTE_TASK_APP_H
#define CAFBCR_COMPUTE_TASK_APP_H

#include "ns3/application.h"
#include "ns3/socket.h"
#include "ns3/ipv4-address.h"
#include "ns3/event-id.h"
#include "ns3/ptr.h"
#include "ns3/data-rate.h"
#include "ns3/random-variable-stream.h"
#include "ns3/traced-callback.h"
#include "cafbcr-compute-task-tag.h"
#include "cafbcr-compute-task-header.h"
#include <queue>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <functional>
#include <array>
#include <vector>

namespace ns3 {
class ClusterNode;
namespace cafbcr {

// Optional, received-information-only extension. Never insert these in Neighbors.
struct ServerDiscoveryCandidate {
  uint16_t nodeId;
  Ipv4Address address;
  double capacity;
  float load;
  uint8_t role;
  Time generatedAt, firstReceivedAt, localReceivedAt, expiresAt;
  uint8_t discoveryHops = 2;
};

struct ServerDiscoveryDecision {
  uint32_t sourceNode = 0, taskId = 0;
  Time time;
  Ipv4Address selected;
  std::array<uint32_t, 3> direct{}, additional{};
  uint32_t unknown = 0, invalid = 0, expired = 0, untracked = 0;
  std::optional<uint16_t> selectedOrigin;
  uint8_t selectedHop = 0, selectedTier = 0;
  std::optional<Time> generatedAt, firstReceivedAt, localReceivedAt, expiresAt;
  std::string reason = "fixed_fallback";
};

struct ComputeQueueState {
  uint32_t nodeId;
  Time time;
  std::string event;
  uint32_t waiting;
  uint32_t capacity;
  bool busy;
  bool running;
  std::optional<uint32_t> localWaiting;
  std::optional<uint32_t> localCapacity;
  std::optional<double> localLoad;
};

// Passive observation only: these events must not supply information to a selector.
struct ComputeTaskEvent {
  std::string event;
  uint32_t sourceNodeId;
  uint32_t taskId;
  uint32_t observerNodeId;
  Time time;
  double workload;
  uint32_t dataSize;
  Time deadline;                 // relative deadline; the client owns the exact Time
  Ipv4Address selectedServer;
  std::string selectionPolicy;
  std::optional<Time> visibleStateAge; // Receive age at selection, not sender sampling age.
  std::string visibleStateStatus = "not_applicable";
};

struct ComputeTaskRecord {
  uint32_t sourceNodeId;
  uint32_t taskId;
  Time generatedAt;
  Time deadline;                 // relative deadline
  double workload;
  uint32_t dataSize;
  Ipv4Address selectedServer;
  std::string selectionPolicy;
  std::optional<Time> acknowledgedAt;
  std::optional<Time> visibleStateAge;
  std::string visibleStateStatus = "not_applicable";
};

struct ComputeTaskSummary {
  uint64_t generated = 0;
  uint64_t onTime = 0;
  uint64_t late = 0;
  uint64_t unconfirmedDue = 0;
  uint64_t unconfirmedFuture = 0;
  double responseSeconds = 0;
};

struct ComputeTaskWireEvent {
  std::string event, format;
  uint32_t observerNodeId;
  Time time;
  Ptr<const Packet> packet;
  Address peer;
  std::optional<uint32_t> sourceNodeId, taskId, payloadBytes, metadataBytes;
  bool valid;
  std::string reason;
  std::optional<int32_t> socketResult;
};

/**
 * \brief Compute-task client application.
 *
 * Generates tasks and sends them to the network:
 * - periodic generation with configurable characteristics;
 * - response-time and completion tracking;
 * - completion-rate and deadline-satisfaction statistics.
 */
class ComputeTaskClientApp : public Application
{
public:
  static TypeId GetTypeId();

  ComputeTaskClientApp();
  virtual ~ComputeTaskClientApp();

  /**
   * Application parameters.
   */
  void SetRemoteAddress(Ipv4Address addr);
  void SetRemotePort(uint16_t port);
  void SetTaskInterval(Time interval);
  void SetTaskWorkload(double minGflops, double maxGflops);
  void SetTaskDataSize(uint32_t minBytes, uint32_t maxBytes);
  void SetTaskDeadline(Time minDeadline, Time maxDeadline);
  // Existing callers stay legacy; the CA scenario explicitly enables the header.
  void SetUseTaskHeader(bool enabled) { m_useTaskHeader = enabled; }
  enum class ServerSelectionPolicy { NATIVE_NEIGHBOR, FIXED_ASSIGNMENT };
  // Fixed assignments use only SetRemoteAddress; no neighbor state or selection RNG.
  void SetServerSelectionPolicy(ServerSelectionPolicy policy);
  using AdditionalServerCandidates = std::function<std::vector<ServerDiscoveryCandidate>()>;
  void SetAdditionalServerCandidates(AdditionalServerCandidates provider);
  // Optional scenario adapter. The actual choice is recorded before generated
  // is emitted; an empty callback preserves all existing selection behaviour.
  using TaskDestinationSelector = std::function<Ipv4Address(const ComputeTaskRecord&)>;
  void SetTaskDestinationSelector(TaskDestinationSelector selector, const std::string& policy);
  // Optional transport adapter. Generation, selection, RNG and source accounting
  // remain owned by this client. Invoke complete only on a valid source receipt.
  using TaskTransport = std::function<void(const ComputeTaskRecord&, std::function<void()>)>;
  void SetTaskTransport(TaskTransport transport);
  // Two adjacent streams: exogenous task attributes, then server-selection draws.
  // Call before starting; scenario code assigns these by stable node index.
  int64_t AssignStreams(int64_t stream);
  // Absolute times. Generate on [start, stop); accept replies through observationStop.
  void SetObservationWindow(Time start, Time stop, Time observationStop);
  void StopGenerating();
  ComputeTaskSummary GetTaskSummary() const;
  const std::map<uint32_t, ComputeTaskRecord>& GetTaskRecords() const { return m_taskMap; }

  /**
   * \brief Return task statistics.
   */
  uint32_t GetTasksGenerated() const { return m_taskMap.size(); }
  uint32_t GetTasksCompleted() const;
  uint32_t GetTasksMissedDeadline() const;
  uint32_t GetTasksTimeout() const;
  double GetAverageResponseTime() const;
  double GetDeadlineMissRate() const;

protected:
  virtual void DoDispose();

private:
  virtual void StartApplication();
  virtual void StopApplication();

  /**
   * Generate and send one task.
   */
  void GenerateTask();
  void TransportCompleted(uint32_t taskId);
  TaskTransport m_taskTransport;
  TaskDestinationSelector m_taskDestinationSelector;
  std::string m_externalSelectionPolicy;

  /**
   * Computation-aware destination selection: choose the neighbor with
   * compute capacity of at least 5 GFLOP/s and the lowest load, falling back
   * to the configured m_remoteAddress when no candidate qualifies. This is the
   * application-layer form of CA-FBCR's "routing is scheduling" design.
   */
  struct ServerSelection {
    Ipv4Address address;
    std::optional<Time> age;
    std::string status = "fixed_fallback";
  };
  ServerSelection PickDynamicServer();
  AdditionalServerCandidates m_additionalServerCandidates;
  ServerDiscoveryDecision m_discoveryDecision;
  TracedCallback<const ServerDiscoveryDecision&> m_discoveryEvent;

  /**
   * Receive optional task-completion notifications.
   */
  void HandleRead(Ptr<Socket> socket);
  void Emit(const std::string& event, const ComputeTaskRecord& record);

  // Configuration parameters.
  Ipv4Address m_remoteAddress;
  uint16_t m_remotePort;
  Time m_taskInterval;
  double m_minWorkload;
  double m_maxWorkload;
  uint32_t m_minDataSize;
  uint32_t m_maxDataSize;
  Time m_minDeadline;
  Time m_maxDeadline;
  ServerSelectionPolicy m_serverSelectionPolicy = ServerSelectionPolicy::NATIVE_NEIGHBOR;

  // Runtime state.
  Ptr<Socket> m_socket;
  EventId m_sendEvent;
  uint32_t m_nextTaskId;
  bool m_running = false;
  bool m_generating = false;
  Time m_generationStart = Seconds(0);
  Time m_generationStop = Time::Max();
  Time m_observationStop = Time::Max();

  // Retained until disposal for late replies, duplicate suppression and audit.
  // No per-task deadline timers: classification uses integer Time at the cutoff.
  std::map<uint32_t, ComputeTaskRecord> m_taskMap;
  TracedCallback<const ComputeTaskEvent&> m_taskEvent;
  TracedCallback<const ComputeTaskWireEvent&> m_wireEvent;
  bool m_useTaskHeader = false;

  // Selection may consume a variable number of draws without changing demand.
  Ptr<UniformRandomVariable> m_demandRandom;
  Ptr<UniformRandomVariable> m_selectionRandom;
};

/**
 * \brief Compute-task server application.
 *
 * Receives and processes tasks:
 * - manages the local task queue;
 * - models service time from the node compute capacity;
 * - returns results or progress updates to clients;
 * - collects processing statistics.
 */
class ComputeTaskServerApp : public Application
{
public:
  static TypeId GetTypeId();

  ComputeTaskServerApp();
  virtual ~ComputeTaskServerApp();

  /**
   * Application parameters.
   */
  void SetLocalPort(uint16_t port);
  void SetComputeCapacity(double gflops);
  void SetQueueCapacity(uint32_t maxTasks);
  // Opt in only for the single application owning this node's advertised queue.
  // The CA scenario enables it explicitly; other users keep their existing state.
  void SetSynchronizeQueueLoad(bool enabled);
  void SetObservationStop(Time stop);
  void SetUseTaskHeader(bool enabled) { m_useTaskHeader = enabled; }

  /**
   * \brief Return task statistics.
   */
  uint32_t GetTasksReceived() const { return m_tasksReceived; }
  uint32_t GetTasksStarted() const { return m_tasksStarted; }
  uint32_t GetTasksFinished() const { return m_tasksFinished; }
  uint32_t GetTasksProcessed() const { return m_tasksFinished; } // actual execution completion
  uint32_t GetTasksDropped() const { return m_tasksDropped; }
  uint32_t GetCurrentQueueLength() const { return m_taskQueue.size(); }
  bool IsProcessing() const { return m_processing; }
  double GetAverageProcessingDelay() const;
  double GetAverageQueueWaitTime() const;

protected:
  virtual void DoDispose();

private:
  virtual void StartApplication();
  virtual void StopApplication();

  /**
   * Handle a new TCP connection.
   */
  bool HandleAccept(Ptr<Socket> s, const Address& from);

  /**
   * Receive a UDP task packet.
   */
  void HandleRead(Ptr<Socket> socket);

  /**
   * Process the next queued task.
   */
  void ProcessNextTask();

  /**
   * Send a completion notification to the client.
   */
  void OnTaskComplete(uint32_t taskId, Address clientAddress);
  void Emit(const std::string& event, const CafbcrComputeTaskTag& tag);
  void PublishQueueState(const std::string& event);

  /**
   * Queued task.
   */
  struct QueuedTask {
    CafbcrComputeTaskTag taskTag;
    Time arrivalTime;
    Time processingStartTime;
    Time relativeDeadline;
  };

  // Configuration parameters.
  uint16_t m_localPort;
  double m_computeCapacity;      // GFLOP/s
  uint32_t m_maxQueueCapacity;
  bool m_syncQueueLoad = false;
  Ptr<ClusterNode> m_loadState;

  // Runtime state.
  Ptr<Socket> m_socket;
  EventId m_processingEvent;
  bool m_processing;
  bool m_running = false;
  Time m_observationStop = Time::Max();
  std::set<std::pair<uint32_t, uint32_t>> m_seenTasks;
  TracedCallback<const ComputeTaskEvent&> m_taskEvent;
  TracedCallback<const ComputeTaskWireEvent&> m_wireEvent;
  bool m_useTaskHeader = false;
  TracedCallback<const ComputeQueueState&> m_queueState;

  // Task queue.
  struct QueuedTaskWithSource {
    QueuedTask task;
    Address sourceAddress;
  };
  std::queue<QueuedTaskWithSource> m_taskQueue;

  // Statistics.
  uint32_t m_tasksReceived;
  uint32_t m_tasksStarted;
  uint32_t m_tasksFinished;
  uint32_t m_tasksDropped;
  double m_totalProcessingDelay;
  double m_totalWaitTime;

  // Task currently being processed.
  QueuedTaskWithSource m_currentTask;
  Time m_processingStartTime;
};

} // namespace cafbcr
} // namespace ns3

#endif // CAFBCR_COMPUTE_TASK_APP_H
