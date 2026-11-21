/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/energy-module.h"
#include "ns3/applications-module.h"
#include "ns3/log.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/olsr-module.h"
#include "ns3/aodv-module.h"
#include "ns3/udp-echo-helper.h"
#include <limits>
#include <iomanip>
#include <cstdlib>
#include <sstream>
#include <cmath>
#include <chrono>
#include "ca-fbcr-task-observer.h"
#include "ca-fbcr-csv-results.h"
#include "ca-fbcr-common-demand.h"
#include "ca-fbcr-queue-observer.h"
#include "ca-fbcr-neighbor-observer.h"
#include "ca-fbcr-comparison-config.h"
#include "ca-fbcr-join-observer.h"
#include "ca-fbcr-link-observer.h"
#include "ca-fbcr-udp-observer.h"
#include "ca-fbcr-task-wire-observer.h"
#include "ca-fbcr-network-observer.h"
#include "ca-fbcr-packet-loss-observer.h"
#include "ca-fbcr-loop-observer.h"
#include "ca-fbcr-recovery-observer.h"
#include "ca-fbcr-shadow-channel.h"
#include "ca-fbcr-phy-header-boundary.h"
#include "ca-fbcr-control-meter.h"
#include "ca-fbcr-cm-summary.h"
#include "ca-fbcr-mognn-adapter.h"
#include "ca-fbcr-mognn-diagnostics.h"

// CAFBCR相关头文件
#include "ns3/cafbcr-routing.h"
#include "ns3/cafbcr-helper.h"
#include "ns3/cafbcr-election.h"
#include "ns3/cafbcr-compute-task-app.h"
#include "ns3/cafbcr-scenario-config.h"
#include "ns3/performance-metrics.h"

// Baseline protocol headers
#include "ns3/icra-module.h"
#include "ns3/qrcf-module.h"
#include "ns3/cfr-eng-helper.h"
#include "ns3/cfr-eng-local-adapter.h"
#include "ns3/mo-helper.h"

using namespace ns3;
using namespace ns3::cafbcr;

NS_LOG_COMPONENT_DEFINE("CafbcrSimulation");

class CafbcrSimulation
{
public:
  struct NodeStats {
    uint32_t tasksSent;
    uint32_t tasksCompleted;
    uint32_t tasksReceived;
    uint32_t tasksProcessed;
    uint32_t tasksDropped;
    double deadlineMissRate;
    double avgProcessingDelay;

    NodeStats()
      : tasksSent(0), tasksCompleted(0), tasksReceived(0),
        tasksProcessed(0), tasksDropped(0), deadlineMissRate(0.0),
        avgProcessingDelay(0.0)
    {}
  };

  struct TestFlow {
    uint16_t srcNode;
    uint16_t dstNode;
    uint16_t port;
    Ipv4Address srcAddr;
    Ipv4Address dstAddr;
  };

  CafbcrSimulation();
  ~CafbcrSimulation();
  bool Configure(int argc, char** argv);
  bool IsDryRun() const { return dryRun; }
  const char* GetFitnessModeName() const { return ClusterNode::FitnessModeName(fitnessMode); }
  void Run();
  void Report(std::ostream& os);

private:
  // Configuration parameters
  uint32_t numCommandNodes;
  uint32_t numRelayNodes;
  uint32_t numTaskNodes;
  uint32_t numSwarmNodes;
  uint32_t totalSize;        // 总节点数（>0 时按比例覆盖上面四个）
  double minSpeed;           // SWARM 速度下限；RELAY/TASK 按 0.33/0.62 比例缩放；COMMAND 静止
  double maxSpeed;           // SWARM 速度上限
  double simulationTime;
  bool verbose;
  std::string outputDir;
  bool csvOnly = true;
  std::string caRenewalTraceDir; // Explicit opt-in sidecars; empty preserves CSV-only output.
  std::string caCmSummary; // Optional compact provenance summary, not an event log.
  std::string resultsCsv = "Result/ca-fbcr-results.csv";
  std::string csvBuildId = "manual";
  CsvResults csvResults;
  double statisticsCollectionInterval;  // 统计收集间隔
  std::string protocol;                 // cafbcr, a baseline protocol, or olsr/aodv
  std::string fanetMode = "auto";
  std::string caVariant = "hardfix";
  std::string caLoopPolicy = "off";
  wireless::ChannelConfig channelConfig;
  Ptr<wireless::CorrelatedShadowing> shadowModel;
  std::string qrcfPreset = "outcome-bellman";
  qrcf::Settings qrcfSettings;
  double olsrHelloInterval = 0.25;
  std::string taskSelection = "native";
  MognnAdapter::Settings moSettings;
  std::unique_ptr<MognnAdapter> mognnAdapter;
  std::string moSummary;
  std::string moDiagnostics;
  std::unique_ptr<MognnDiagnostics> moDiagnosticObserver;
  bool UsesMo() const { return MognnAdapter::Supports(protocol); }
  ClusterNode::FitnessMode fitnessMode = ClusterNode::HETEROGENEOUS_COMPUTE;
  bool dryRun = false;
  bool phyHeaderBoundaryGuard = true;
  bool observeAbcCost = false;
  std::string caInitialSelector = "abc";
  bool observeInitialSelection = false;
  double taskGenStart = 1.0;
  double taskGenStop = -1.0;
  std::string taskStartMode = "legacy-index";
  bool requireTaskDeadlineCoverage = false;
  bool taskDeadlineCoverage = false;
  double maximumTaskDeadline = 0.0;
  std::string taskOutputDir;
  std::string controlMeter = "legacy";
  std::unique_ptr<ControlCostMeter> controlCostMeter;
  std::string commandArguments;
  std::unique_ptr<TaskObservationWriter> taskObserver;
  std::unique_ptr<cfreng::LocalAdapter> cfrAdapter;
  std::string cfrEngCompute = "joint";
  std::unique_ptr<QueueLoadWriter> queueObserver;
  std::unique_ptr<NeighborComputeWriter> neighborObserver;
  std::unique_ptr<JoinObservationWriter> joinObserver;
  std::unique_ptr<UdpObservationWriter> udpObserver;
  std::unique_ptr<FailureSummary> failureSummary;
  std::unique_ptr<RunDiagnostics> runDiagnostics;
  std::unique_ptr<CmSummary> cmSummary;
  std::unique_ptr<TaskWireWriter> taskWireObserver;
  std::unique_ptr<NetworkObservationWriter> networkObserver;
  std::unique_ptr<PacketLossObservationWriter> packetLossObserver;
  bool observePacketLoss = true;
  bool observeCaNextHop = true;
  std::unique_ptr<LoopObservationWriter> loopObserver;
  std::unique_ptr<RecoveryObservationWriter> recoveryObserver;
  std::string taskWire = "header-v1";
  bool observeJoin = true;
  bool observePhyCompletion = true;
  bool syncQueueLoad = true;
  std::string helloMethod;              // Hello方法: "fixed", "fuzzy", "qlearning"
  bool caQHelloDiagnostics = false;     // One aggregate record; no per-node/event files
  uint32_t seed;                        // 随机种子
  uint32_t numTestFlows;                // Validate before narrowing node IDs / ports
  std::string udpDemand = "fixed-attempts";
  double udpStart = -1.0, udpStop = -1.0;
  bool csvConfiguration = false;
  bool csvScenario = false;
  bool csvUdpMetrics = false;
  std::optional<scenario::Contract> scenarioContract;
  std::string asymRangeProfile = "linked";
  bool csvFailureSummary = false;
  bool csvRunDiagnostics = false;
  bool enableComputeTask;               // Enable the compute-task application.
  bool cleanShutdown;                   // Use the full Simulator::Destroy() path on exit.
  bool traceDrops;                      // Attach IPv4/PHY drop traces (diagnostics only).
  double sweepWc;                       // <0 uses the default five-way weights; otherwise wC is capped at 0.40 and the other four share (1-wC)/4.
  bool disableAsym;                     // Ablation: disable asym handling (M1/M4); CSV label gets *-noasym.
  bool caProxyJoin = true;
  bool caProxyRenewal = true;
  std::string caProxyRenewalPolicy = "eager";
  std::string caProxyRelayPolicy = "legacy";
  std::string caDirectCmPolicy = "legacy";
  std::string caControlUnicastPolicy = "legacy";
  double caProxyRenewalWindowMs = 155;
  double caProxyAckBatchMs = 20;
  bool caRangeGating = true;
  bool caLinkProbes=false,observeLinkProbes=true;
  std::string caLinkGate="distance";
  double caLinkProbeInterval=2,caLinkProbeTimeout=.25,caLinkValidity=6;
  std::unique_ptr<LinkObservationWriter> linkObserver;
  int  asymLevel;                       // [asymmetry-stress] TX 功率差档位 0..4；<0 用默认场景
  double arpDeadReprobe;                // DEAD ARP 入口限速重探间隔（秒）；0 保持 ns-3 默认直接丢弃
  uint32_t arpMaxRetries;               // ARP 最大重传次数；3 为 ns-3 默认
  double arpWaitReplyMs;                // ARP WaitReplyTimeout（毫秒）；1000 为 ns-3 默认

  // Simulation objects
  NodeContainer allNodes;
  Ptr<ScenarioConfig> config;
  std::map<uint32_t, NodeStats> nodeStats;
  uint32_t totalTasksGenerated;
  uint32_t totalTasksCompleted;
  Ipv4InterfaceContainer interfaces;
  NetDeviceContainer devices;

  // FlowMonitor and performance metrics
  FlowMonitorHelper flowmon;
  ns3::Ptr<ns3::FlowMonitor> monitor;
  Ptr<ns3::PerformanceMetrics> m_metrics;
  std::vector<TestFlow> m_testFlows;

  // Drop tracking (when traceDrops=true)
  uint64_t m_dropTotal;
  std::map<uint32_t, uint64_t> m_dropByReason;     // ipv4 drop reason -> count
  uint64_t m_dropTestFlow;                          // ipv4 drops on UDP test-flow packets
  std::map<uint32_t, uint64_t> m_dropByNodeIdx;    // 哪些节点丢得最多 (按真实 nodeIdx)
  std::map<Ptr<Ipv4>, uint32_t> m_ipv4ToNodeIdx;   // ipv4 ptr → node 索引（避免 ClusterNode aggregate 错位）
  uint64_t m_phyTxDrop;
  uint64_t m_phyRxDrop;
  uint64_t m_macTxDrop;                             // MAC 层 drop（队列满）
  std::map<uint32_t, uint64_t> m_macDropByNodeIdx;
  uint64_t m_macMpduDrop;                            // DroppedMpdu 总数（含重传耗尽）
  std::map<uint32_t, uint64_t> m_macMpduDropByReason;
  std::map<uint32_t, uint64_t> m_macMpduDropByNodeIdx;

  void OnIpv4Drop(const Ipv4Header& header, Ptr<const Packet> p,
                  Ipv4L3Protocol::DropReason reason, Ptr<Ipv4> ipv4, uint32_t iface);
  void OnPhyTxDrop(Ptr<const Packet> p);
  void OnPhyRxDrop(Ptr<const Packet> p, WifiPhyRxfailureReason reason);
  // MAC 的 trace 没有 nodeIdx 参数，用 TraceConnect(context) 把 nodeIdx 编进 context 字符串
  void OnMacTxDropCtx(std::string context, Ptr<const Packet> p);
  // DroppedMpdu 触发场景含重传耗尽（最可能丢失数据包的环节）
  void OnDroppedMpduCtx(std::string context,
                        WifiMacDropReason reason,
                        Ptr<const WifiMpdu> mpdu);

  // Helper methods (按照正确的顺序）
  void CreateScenarioConfig();
  void CreateNodes();              // 创建节点 + 配置异构移动模型
  void CreateDevices();             // 创建异构网络设备
  void InstallEnergyModel();       // Install energy sources (required by clustering protocols)
  void InstallInternetStack();      // 安装协议栈和路由协议（仅限网络层）
  void InstallApplications();       // 安装应用层（计算任务 + UDP测试流 + FlowMonitor）
  void ConfigureStatisticsCollection();  // 配置统计收集任务
  void DoCollectStatistics();        // 实际执行统计收集的内部方法
  void FinalizeStatistics();        // 最终统计（FlowMonitor + 任务应用）
  std::string BaselineProfileJson() const;
  std::string CsvAlgorithm() const;
  scenario::Contract MakeScenarioContract() const;
  void SaveBaselineRuntime() const;
  void SavePhyHeaderBoundarySummary() const;
  void SaveAbcCostObservation() const;
  void SaveInitialSelectionObservation() const;
};

// ==================== Constructor ====================
CafbcrSimulation::CafbcrSimulation()
  : numCommandNodes(1),
    numRelayNodes(3),
    numTaskNodes(6),
    numSwarmNodes(15),
    totalSize(0),       // 0 表示不覆盖，使用上面四个独立配置
    minSpeed(15.0),     // Default SWARM mean-speed range: 15-25 m/s.
    maxSpeed(25.0),
    simulationTime(240.0),
    verbose(false),
    outputDir("./ca-fbcr-results/"),
    statisticsCollectionInterval(5.0),  // 每5秒收集一次统计信息
    protocol("cafbcr"),                 // 默认 CA-FBCR 协议
    helloMethod("fuzzy"),               // 默认fuzzy方法
    seed(2025),                         // 默认随机种子
    numTestFlows(50),                   // Default UDP test-flow count.
    enableComputeTask(true),            // 默认启用计算任务应用
    cleanShutdown(false),               // 默认 _Exit(0)，跳过 Simulator::Destroy() 的 ~30 分钟级联析构
    traceDrops(false),                  // 默认不挂 drop trace
    sweepWc(-1.0),                      // <0 表示不参与 wC 扫描，用 CreateScenarioConfig 里的默认权重
    disableAsym(false),
    asymLevel(-1),
    arpDeadReprobe(0.0),
    arpMaxRetries(3),
    arpWaitReplyMs(1000.0),
    totalTasksGenerated(0),
    totalTasksCompleted(0),
    m_dropTotal(0),
    m_dropTestFlow(0),
    m_phyTxDrop(0),
    m_phyRxDrop(0),
    m_macTxDrop(0),
    m_macMpduDrop(0)
{
}

CafbcrSimulation::~CafbcrSimulation()
{
}

// ==================== Configure ====================
bool CafbcrSimulation::Configure(int argc, char** argv)
{
  CommandLine cmd;
  channelConfig.AddOptions(cmd);
  cmd.AddValue("numCommand", "Number of COMMAND nodes", numCommandNodes);
  cmd.AddValue("numRelay", "Number of RELAY nodes", numRelayNodes);
  cmd.AddValue("numTask", "Number of TASK nodes", numTaskNodes);
  cmd.AddValue("numSwarm", "Number of SWARM nodes", numSwarmNodes);
  cmd.AddValue("size", "Total node count (overrides numCommand/Relay/Task/Swarm by FANET-realistic ratio)", totalSize);
  cmd.AddValue("MinSpeed", "Lower SWARM mean-velocity input in m/s (RELAY=0.33x, TASK=0.62x; COMMAND static)", minSpeed);
  cmd.AddValue("MaxSpeed", "Upper SWARM mean-velocity input in m/s; not an instantaneous speed bound", maxSpeed);
  cmd.AddValue("time", "Simulation time in seconds", simulationTime);
  cmd.AddValue("verbose", "Enable verbose logging", verbose);
  cmd.AddValue("output", "Output directory for results", outputDir);
  cmd.AddValue("csvOnly", "Write one result CSV only; default true. False enables legacy diagnostic artifacts", csvOnly);
  cmd.AddValue("caRenewalTraceDir", "CA CSV-only passive JOIN/ProxyRenewal sidecars in a fresh directory; empty disables", caRenewalTraceDir);
  cmd.AddValue("caCmSummary", "CA passive CM trigger summary in one fresh JSON; requires CSV scenario and run diagnostics", caCmSummary);
  cmd.AddValue("controlMeter", "Control cost meter [legacy|l3]; l3 counts non-loopback IPv4 Tx control bytes including IPv4+UDP", controlMeter);
  cmd.AddValue("resultsCsv", "Result CSV appended directly by the simulation", resultsCsv);
  cmd.AddValue("csvBuildId", "Build label stored in the result CSV", csvBuildId);
  cmd.AddValue("statsInterval", "Statistics collection interval in seconds", statisticsCollectionInterval);
  cmd.AddValue("protocol", "Routing protocol [cafbcr|qrcf|cfr-eng|mognn|mognn-baseline|mognn-local|olsr|icra|aodv]; AODV retained for historical diagnostics only", protocol);
  cmd.AddValue("fanetMode", "Fitness semantics [auto|homogeneous|heterogeneous-compute]; auto selects heterogeneous-compute", fanetMode);
  cmd.AddValue("caVariant", "CA build identity [hardfix]; packet/RNG fixes with original CA routing settings", caVariant);
  cmd.AddValue("caLoopPolicy", "CA-only experimental transit policy [off,terminate-revisit,alternate-revisit]; default off", caLoopPolicy);
  cmd.AddValue("dryRun", "Validate configuration without creating nodes or running a simulation", dryRun);
  cmd.AddValue("phyHeaderBoundaryGuard", "CA-entry PHY adaptation: process a due 802.11g header event before a same-timestamp ERP/DSSS/HR-DSSS arrival; default true", phyHeaderBoundaryGuard);
  cmd.AddValue("algorithm", "Alias for --protocol; primary comparisons use cafbcr, qrcf, icra, cfr-eng, olsr", protocol);
  cmd.AddValue("helloMethod", "CAFBCR Hello method [fixed|fuzzy|qlearning]", helloMethod);
  cmd.AddValue("caQHelloDiagnostics", "Print one aggregate local-learning summary; CA qlearning only", caQHelloDiagnostics);
  cmd.AddValue("qrcfPreset", "QRCF profile [legacy-noq|staggered-noq|outcome-immediate|outcome-bellman]; default outcome-bellman", qrcfPreset);
  cmd.AddValue("olsrHelloInterval", "OLSR HELLO interval in seconds; CA-entrypoint default 0.25", olsrHelloInterval);
  cmd.AddValue("cfrEngCompute", "CFR-ENG only: joint|source|destination; engineering heuristic, no PDDQN", cfrEngCompute);
  cmd.AddValue("moModel", "MOGNN learned text model; strict inference, no silent fallback", moSettings.model);
  cmd.AddValue("moSlotSeconds", "MOGNN policy slot in seconds", moSettings.slotSeconds);
  cmd.AddValue("moLinkBps", "MOGNN configured link-rate estimate, not measured bandwidth", moSettings.linkBps);
  cmd.AddValue("moHelloInterval", "MOGNN radio descriptor period in seconds", moSettings.helloSeconds);
  cmd.AddValue("moHoldTime", "MOGNN maximum original descriptor age in seconds", moSettings.holdSeconds);
  cmd.AddValue("moRelayMode", "MO local control transport: legacy|batch-v1 (20 ms, 1200 bytes, at most 287 nodes)", moSettings.relayMode);
  cmd.AddValue("moRelayWindowMs",
               "batch-v1 relay batch deadline in ms, 1..1000; larger windows allow more "
               "duplicate-relay suppression but add descriptor latency",
               moSettings.relayBatchWindowMs);
  cmd.AddValue("moRelaySuppressDuplicates",
               "batch-v1: cancel a pending relay when the same descriptor is received "
               "from another node before the batch flush",
               moSettings.relaySuppressDuplicates);
  cmd.AddValue("moServerHelloScale",
               "MOGNN Hello-period multiplier for COMMAND/RELAY server nodes; "
               "below 1 refreshes server descriptors faster",
               moSettings.serverHelloScale);
  cmd.AddValue("arpDeadReprobe",
               "Minimum seconds since an ARP entry was marked DEAD before a new packet may "
               "re-probe it; 0 keeps the stock ns-3 drop-until-DeadTimeout behaviour. "
               "Applies uniformly to every node and protocol in this run.",
               arpDeadReprobe);
  cmd.AddValue("arpMaxRetries",
               "ARP retransmissions before an entry is marked DEAD; 3 is the ns-3 default. "
               "Applies uniformly to every node and protocol in this run.",
               arpMaxRetries);
  cmd.AddValue("arpWaitReplyMs",
               "ARP WaitReplyTimeout in milliseconds; 1000 is the ns-3 default. "
               "Applies uniformly to every node and protocol in this run.",
               arpWaitReplyMs);
  cmd.AddValue("moSummary", "Optional compact local/remote task partition JSON; new MOGNN-only file", moSummary);
  cmd.AddValue("moDiagnostics", "Optional new MOGNN-only directory for passive route/control/MAC/task counters", moDiagnostics);
  cmd.AddValue("taskSelection", "Task server selection [native|fixed]; fixed uses a common offline assignment", taskSelection);
  cmd.AddValue("seed", "Random seed", seed);
  cmd.AddValue("numTestFlows", "Number of UDP test flows", numTestFlows);
  cmd.AddValue("udpDemand", "UDP demand [fixed-attempts|legacy-success-capped]; legacy is a diagnostic control", udpDemand);
  cmd.AddValue("udpStart", "Explicit UDP window start in seconds (>=5); set together with udpStop, -1 preserves legacy timing", udpStart);
  cmd.AddValue("udpStop", "Exclusive explicit UDP window end, independent of total observation time", udpStop);
  cmd.AddValue("csvConfiguration", "Append task/UDP window and control-meter configuration to the native CSV", csvConfiguration);
  cmd.AddValue("csvScenario", "Append versioned scenario identity/effective configuration; requires csvOnly and csvConfiguration", csvScenario);
  cmd.AddValue("csvUdpMetrics", "Append unique UDP delivery delay, sample count, sum and measurement profile; requires csvOnly and csvConfiguration", csvUdpMetrics);
  cmd.AddValue("asymRangeProfile", "CA-entry nominal range profile: linked (existing table) or fixed-default (power-only control)", asymRangeProfile);
  cmd.AddValue("csvFailureSummary", "CA-only passive drop evidence in CSV; requires csvOnly and csvConfiguration", csvFailureSummary);
  cmd.AddValue("csvRunDiagnostics", "Passive timing, event and revisit counters in CSV (CA only); requires csvOnly and csvConfiguration", csvRunDiagnostics);
  cmd.AddValue("taskWire", "Task metadata [header-v1|legacy-tag]; legacy is a diagnostic control", taskWire);
  cmd.AddValue("enableComputeTask", "Enable compute task applications [true|false]", enableComputeTask);
  cmd.AddValue("syncQueueLoad", "Publish each server's actual waiting-queue fraction to its local ClusterNode", syncQueueLoad);
  cmd.AddValue("observeJoin", "Write passive CA join events; does not alter join policy", observeJoin);
  cmd.AddValue("observePacketLoss", "Write passive IPv4/ARP/MAC packet events; CA entrypoint only", observePacketLoss);
  cmd.AddValue("observeCaNextHop", "Capture existing CA next-hop audit when packet loss observation is enabled", observeCaNextHop);
  cmd.AddValue("taskGenStart", "Task generation window start; node phases follow taskStartMode", taskGenStart);
  cmd.AddValue("taskGenStop", "Exclusive task generation cutoff; -1 uses time-1 (diagnostic window)", taskGenStop);
  cmd.AddValue("taskStartMode", "Task starts: legacy-index (default, index*0.1s) or bounded-1s (index/N seconds)", taskStartMode);
  cmd.AddValue("taskOutputDir", "New per-run directory for task-observation-v1 artifacts; must not exist", taskOutputDir);
  cmd.AddValue("requireTaskDeadlineCoverage", "Reject windows shorter than taskGenStop + maximum task deadline", requireTaskDeadlineCoverage);
  cmd.AddValue("cleanShutdown",
               "Run full Simulator::Destroy() at exit (slow on large sims). "
               "Default false uses _Exit(0) for fast termination",
               cleanShutdown);
  cmd.AddValue("traceDrops",
               "Enable ipv4 + PHY drop tracing for debugging packet loss",
               traceDrops);
  cmd.AddValue("wC",
               "CH fitness compute weight: nonnegative values are capped at 0.40; "
               "LHTP, Energy, Degree and Distance each use (1-wC)/4. "
               "Negative = default five equal weights (0.20 each).",
               sweepWc);
  cmd.AddValue("disableAsym",
               "[compatibility] Force both CA proxy join and range gating off; cafbcr only.",
               disableAsym);
  cmd.AddValue("caProxyJoin", "Enable CA proxy join; false retains ordinary JOIN_REQ/JOIN_RESP", caProxyJoin);
  cmd.AddValue("caProxyRenewal", "Renew grant-confirmed CA members through HELLO-assisted relays; false is the old maintenance control", caProxyRenewal);
  cmd.AddValue("caProxyRenewalPolicy", "eager (default), ack-stagger, ack-batch, or direct-feedback: expiring direct-CM evidence in CH_LIVE", caProxyRenewalPolicy);
  cmd.AddValue("caProxyRelayPolicy", "legacy (default), or ack-stagger: combine direct-feedback requests with received-ACK relay coordination", caProxyRelayPolicy);
  cmd.AddValue("caDirectCmPolicy", "legacy (default), or feedback-covered: omit extra CH_LIVE response only under valid received feedback", caDirectCmPolicy);
  cmd.AddValue("caControlUnicastPolicy", "legacy (default), or one-hop: own TTL-one controls address the actual peer directly", caControlUnicastPolicy);
  cmd.AddValue("caProxyRenewalWindowMs", "Maximum ACK-stagger relay window in [1,1000] ms; also bounded by source lifetime", caProxyRenewalWindowMs);
  cmd.AddValue("caLinkProbes","Enable real one-hop unicast round-trip probes; default off",caLinkProbes);
  cmd.AddValue("caLinkGate","Data candidate filter: distance (default) or handshake",caLinkGate);
  cmd.AddValue("caLinkProbeInterval","Probe interval seconds, diagnostic default 2",caLinkProbeInterval);
  cmd.AddValue("caLinkProbeTimeout","Reply timeout seconds, diagnostic default .25",caLinkProbeTimeout);
  cmd.AddValue("caLinkValidity","Evidence validity from local send time, seconds, default 6",caLinkValidity);
  cmd.AddValue("observeLinkProbes","Passive link probe/selection ledger when probes enabled",observeLinkProbes);
  cmd.AddValue("observeAbcCost", "Record CA StartElection work and host timing; default off, no simulated delay", observeAbcCost);
  cmd.AddValue("caInitialSelector", "CA initial candidate selector: abc (default) or topk; maintenance is unchanged", caInitialSelector);
  cmd.AddValue("observeInitialSelection", "Capture CA initial inputs and first-pass F0 without extra scoring; default off", observeInitialSelection);
  cmd.AddValue("observePhyCompletion", "Passive local PHY completion/interruption ledger; separate from nominal candidate costs", observePhyCompletion);
  cmd.AddValue("caProxyAckBatchMs", "ACK-batch maximum wait in [1,100] ms, first record starts the timer", caProxyAckBatchMs);
  cmd.AddValue("caRangeGating", "Enable CA data forwarding range filtering; proxy qualification and PHY stay configured", caRangeGating);
  cmd.AddValue("asymLevel",
               "[asymmetry-stress] TX-power-gap level 0..4 (lowers TASK/SWARM power and "
               "routing range together with asymRangeProfile=linked). L4 matches default; -1 keeps default.",
               asymLevel);
  cmd.Parse(argc, argv);
  if (const auto error = scenario::Validate(minSpeed, maxSpeed, asymLevel, asymRangeProfile); !error.empty()) {
    std::cerr << error << std::endl; return false;
  }
  if (!std::isfinite(arpDeadReprobe) || arpDeadReprobe < 0.0) {
    std::cerr << "arpDeadReprobe must be a finite value >= 0." << std::endl; return false;
  }
  if (arpMaxRetries < 1 || arpMaxRetries > 32) {
    std::cerr << "arpMaxRetries must be in [1,32]." << std::endl; return false;
  }
  if (!std::isfinite(arpWaitReplyMs) || arpWaitReplyMs < 100.0 || arpWaitReplyMs > 10000.0) {
    std::cerr << "arpWaitReplyMs must be a finite value in [100,10000]." << std::endl; return false;
  }
  if (arpDeadReprobe > 0.0) {
    Config::SetDefault("ns3::ArpL3Protocol::DeadReprobeInterval",
                       TimeValue(Seconds(arpDeadReprobe)));
  }
  if (arpMaxRetries != 3) {
    Config::SetDefault("ns3::ArpCache::MaxRetries", UintegerValue(arpMaxRetries));
  }
  if (arpWaitReplyMs != 1000.0) {
    Config::SetDefault("ns3::ArpCache::WaitReplyTimeout", TimeValue(MilliSeconds(arpWaitReplyMs)));
  }
  if ((csvScenario && (!csvOnly || !csvConfiguration)) ||
      (asymRangeProfile != "linked" && !csvScenario)) {
    std::cerr << "csvScenario requires csvOnly=1 and csvConfiguration=1; fixed-default requires csvScenario=1." << std::endl;
    return false;
  }
  if (csvUdpMetrics && (!csvOnly || !csvConfiguration)) {
    std::cerr << "csvUdpMetrics requires csvOnly=1 and csvConfiguration=1." << std::endl;
    return false;
  }
  if (resultsCsv.empty() || csvBuildId.empty() || csvBuildId.find_first_of(",\r\n\"") != std::string::npos) {
    std::cerr << "resultsCsv and a comma-free csvBuildId are required." << std::endl;
    return false;
  }
  if (controlMeter != "legacy" && controlMeter != "l3") {
    std::cerr << "Unknown --controlMeter; choose legacy or l3." << std::endl;
    return false;
  }
  if (csvFailureSummary && (!csvOnly || !csvConfiguration || protocol != "cafbcr")) {
    std::cerr << "csvFailureSummary requires protocol=cafbcr, csvOnly=1 and csvConfiguration=1." << std::endl;
    return false;
  }
  if (csvRunDiagnostics && (!csvOnly || !csvConfiguration || protocol != "cafbcr")) {
    std::cerr << "csvRunDiagnostics requires CA, csvOnly=1 and csvConfiguration=1." << std::endl;
    return false;
  }
  if (!caCmSummary.empty()) {
    if (protocol != "cafbcr" || !csvOnly || !csvConfiguration || !csvScenario || !csvRunDiagnostics) {
      std::cerr << "caCmSummary requires CA, csvOnly/configuration/scenario/runDiagnostics=1." << std::endl;
      return false;
    }
    try {
      const auto requested = std::filesystem::absolute(caCmSummary).lexically_normal();
      if (std::filesystem::exists(std::filesystem::symlink_status(requested))) {
        std::cerr << "caCmSummary must be a fresh file." << std::endl;
        return false;
      }
      const auto summary = std::filesystem::weakly_canonical(requested);
      const auto csv = std::filesystem::weakly_canonical(std::filesystem::absolute(resultsCsv));
      const auto within = [](const auto& child, const auto& parent) {
        const auto relative = child.lexically_relative(parent);
        return !relative.empty() && *relative.begin() != "..";
      };
      if (within(summary, csv) || within(csv, summary)) {
        std::cerr << "caCmSummary collides with resultsCsv." << std::endl;
        return false;
      }
      auto ancestor = summary.parent_path();
      while (!std::filesystem::exists(ancestor)) ancestor = ancestor.parent_path();
      if (!std::filesystem::is_directory(ancestor)) {
        std::cerr << "caCmSummary parent is not a directory." << std::endl;
        return false;
      }
      if (!caRenewalTraceDir.empty()) {
        const auto trace = std::filesystem::weakly_canonical(std::filesystem::absolute(caRenewalTraceDir));
        if (within(trace, summary) || summary == trace / "trace-run.json" ||
            summary == trace / "join-events.jsonl" || summary == trace / "proxy-renewal.jsonl") {
          std::cerr << "caCmSummary collides with renewal sidecars." << std::endl;
          return false;
        }
      }
      caCmSummary = summary.string();
    } catch (const std::filesystem::filesystem_error& error) {
      std::cerr << "Cannot validate caCmSummary: " << error.what() << std::endl;
      return false;
    }
  }
  if (!caRenewalTraceDir.empty()) {
    if (protocol != "cafbcr" || !csvOnly) {
      std::cerr << "caRenewalTraceDir requires protocol=cafbcr and csvOnly=1." << std::endl;
      return false;
    }
    // Validation/dry-run never creates output. Refuse existing directories/files,
    // including dangling symlinks, and CSV paths that collide with sidecars.
    try {
      const auto requested = std::filesystem::absolute(caRenewalTraceDir).lexically_normal();
      if (std::filesystem::exists(std::filesystem::symlink_status(requested))) {
        std::cerr << "caRenewalTraceDir must be a fresh path." << std::endl;
        return false;
      }
      const auto directory = std::filesystem::weakly_canonical(requested);
      const auto csv = std::filesystem::weakly_canonical(std::filesystem::absolute(resultsCsv));
      const auto relative = directory.lexically_relative(csv);
      if ((!relative.empty() && *relative.begin() != "..") ||
          csv == directory / "join-events.jsonl" || csv == directory / "proxy-renewal.jsonl" ||
          csv == directory / "trace-run.json") {
        std::cerr << "resultsCsv collides with caRenewalTraceDir sidecars." << std::endl;
        return false;
      }
      caRenewalTraceDir = directory.string();
    } catch (const std::filesystem::filesystem_error& error) {
      std::cerr << "Cannot validate caRenewalTraceDir: " << error.what() << std::endl;
      return false;
    }
  }
  if (csvOnly) {
    if (!taskOutputDir.empty()) {
      std::cerr << "CSV-only output uses --resultsCsv; remove --taskOutputDir or explicitly use --csvOnly=0." << std::endl;
      return false;
    }
    observeJoin = observePacketLoss = observeCaNextHop = observePhyCompletion = false;
    observeLinkProbes = observeAbcCost = observeInitialSelection = traceDrops = false;
    channelConfig.pairingAudit = false;
    channelConfig.tracePairs = 0;
  }
  if (const auto error = channelConfig.Validate(); !error.empty()) {
    std::cerr << error << std::endl;
    return false;
  }
  std::cout << "[Channel] " << channelConfig.Json() << std::endl;
  std::ostringstream arguments;
  arguments << '[';
  for (int i = 0; i < argc; ++i) {
    if (i) arguments << ',';
    arguments << TaskObservationWriter::Quote(argv[i]);
  }
  arguments << ']';
  commandArguments = arguments.str();
  if (taskGenStop == -1.0) taskGenStop = simulationTime - 1.0;
  if (taskStartMode != "legacy-index" && taskStartMode != "bounded-1s") {
    std::cerr << "taskStartMode must be legacy-index or bounded-1s." << std::endl;
    return false;
  }
  if (!std::isfinite(simulationTime) || simulationTime <= 0 ||
      !std::isfinite(taskGenStart) || !std::isfinite(taskGenStop) ||
      taskGenStart < 0 || taskGenStop < taskGenStart || taskGenStop > simulationTime ||
      !std::isfinite(statisticsCollectionInterval) || statisticsCollectionInterval <= 0) {
    std::cerr << "Invalid observation window: require 0 <= taskGenStart <= taskGenStop <= time,"
                 " positive time and statsInterval." << std::endl;
    return false;
  }

  // Normalize legacy capitalizations of the supported protocol names.
  if (protocol == "QRCF" || protocol == "Qrcf") { protocol = "qrcf"; }
  else if (protocol == "OLSR") { protocol = "olsr"; }
  else if (protocol == "ICRA") { protocol = "icra"; }

  // Only accept protocols that this entry actually installs, so a stale name
  // cannot silently produce an empty routing stack.
  if (protocol != "cafbcr" && protocol != "aodv" && protocol != "olsr" &&
      protocol != "icra" && protocol != "qrcf" &&
      protocol != "cfr-eng" && !UsesMo()) {
    std::cerr << "Unsupported routing protocol: " << protocol
              << ". Choose cafbcr, qrcf, icra, cfr-eng, mognn, "
                 "mognn-baseline, mognn-local or olsr; aodv is historical diagnostics only."
              << std::endl;
    return false;
  }

  if ((protocol == "cfr-eng" && taskSelection != "fixed") ||
      (cfrEngCompute != "joint" && cfrEngCompute != "source" && cfrEngCompute != "destination") ||
      (protocol != "cfr-eng" && cfrEngCompute != "joint")) {
    std::cerr << "CFR-ENG requires --taskSelection=fixed; --cfrEngCompute=joint|source|destination is CFR-only." << std::endl;
    return false;
  }

  if (fanetMode == "auto") fanetMode = "heterogeneous-compute";
  if (!ClusterNode::ParseFitnessMode(fanetMode, fitnessMode)) {
    std::cerr << "Unknown --fanetMode; choose auto, homogeneous or heterogeneous-compute." << std::endl;
    return false;
  }
  if (caLoopPolicy != "off" && caLoopPolicy != "terminate-revisit" && caLoopPolicy != "alternate-revisit") {
    std::cerr << "caLoopPolicy must be off or terminate-revisit or alternate-revisit" << std::endl; return false;
  }
  if (protocol != "cafbcr" && caLoopPolicy != "off") {
    std::cerr << "caLoopPolicy=" << caLoopPolicy << " is a CA-only experimental policy" << std::endl; return false;
  }
  if (caVariant != "hardfix") {
    std::cerr << "This build supports only --caVariant=hardfix; the previous optimized build is archived." << std::endl;
    return false;
  }
  if (observeAbcCost && protocol != "cafbcr") {
    std::cerr << "--observeAbcCost requires protocol=cafbcr." << std::endl;
    return false;
  }
  if (caInitialSelector != "abc" && caInitialSelector != "topk") {
    std::cerr << "Unknown --caInitialSelector; choose abc or topk." << std::endl;
    return false;
  }
  if (protocol != "cafbcr" && (caInitialSelector != "abc" || observeInitialSelection)) {
    std::cerr << "--caInitialSelector=topk and --observeInitialSelection require protocol=cafbcr." << std::endl;
    return false;
  }
  if (protocol == "cafbcr") {
    Config::SetDefault("ns3::cafbcr::cafbcrElection::ObserveCost", BooleanValue(observeAbcCost));
    Config::SetDefault("ns3::cafbcr::cafbcrElection::InitialSelector",
                       EnumValue(caInitialSelector == "topk" ? cafbcrElection::TOPK : cafbcrElection::ABC));
    Config::SetDefault("ns3::cafbcr::cafbcrElection::ObserveInitialSelection", BooleanValue(observeInitialSelection));
  }
  if (taskSelection != "native" && taskSelection != "fixed") {
    std::cerr << "Unknown --taskSelection; choose native or fixed." << std::endl;
    return false;
  }
  if (udpDemand != "fixed-attempts" && udpDemand != "legacy-success-capped") {
    std::cerr << "Unknown --udpDemand; choose fixed-attempts or legacy-success-capped." << std::endl;
    return false;
  }
  if (!std::isfinite(udpStart) || !std::isfinite(udpStop) ||
      !((udpStart == -1 && udpStop == -1) ||
        (udpStart >= 5 && udpStop > udpStart && udpStop <= simulationTime &&
         udpDemand == "fixed-attempts"))) {
    std::cerr << "Explicit UDP windows require fixed-attempts and 5 <= udpStart < udpStop <= time;"
                 " omit both options for legacy timing." << std::endl;
    return false;
  }
  if (taskWire != "header-v1" && taskWire != "legacy-tag") {
    std::cerr << "Unknown --taskWire; choose header-v1 or legacy-tag." << std::endl;
    return false;
  }
  if (caProxyRenewalPolicy!="eager" && caProxyRenewalPolicy!="ack-stagger" &&
      caProxyRenewalPolicy!="ack-batch" && caProxyRenewalPolicy!="direct-feedback") {
    std::cerr << "Unknown --caProxyRenewalPolicy; choose eager, ack-stagger, ack-batch or direct-feedback." << std::endl;
    return false;
  }
  if (!std::isfinite(caProxyRenewalWindowMs) || caProxyRenewalWindowMs<1 || caProxyRenewalWindowMs>1000) {
    std::cerr << "--caProxyRenewalWindowMs must be in [1,1000]." << std::endl;
    return false;
  }
  if (caDirectCmPolicy != "legacy" && caDirectCmPolicy != "feedback-covered") {
    std::cerr << "Unknown --caDirectCmPolicy; choose legacy or feedback-covered." << std::endl;
    return false;
  }
  if ((caControlUnicastPolicy != "legacy" && caControlUnicastPolicy != "one-hop") ||
      (caControlUnicastPolicy != "legacy" && (protocol != "cafbcr" || !csvScenario))) {
    std::cerr << "caControlUnicastPolicy requires legacy or CA one-hop with csvScenario=1." << std::endl;
    return false;
  }
  if (caDirectCmPolicy == "feedback-covered" && (protocol != "cafbcr" ||
      caProxyRenewalPolicy != "direct-feedback" || disableAsym || !caProxyJoin ||
      !caProxyRenewal || !csvScenario)) {
    std::cerr << "feedback-covered requires CA direct-feedback, asym/proxy/renewal enabled and csvScenario=1." << std::endl;
    return false;
  }
  if ((caProxyRelayPolicy != "legacy" && caProxyRelayPolicy != "ack-stagger") ||
      (caProxyRelayPolicy != "legacy" && (protocol != "cafbcr" ||
       caProxyRenewalPolicy != "direct-feedback" || disableAsym || !caProxyJoin ||
       !caProxyRenewal || !csvScenario || caDirectCmPolicy != "legacy"))) {
    std::cerr << "caProxyRelayPolicy requires legacy or CA ack-stagger with direct-feedback, "
                 "asym/proxy/renewal enabled, caDirectCmPolicy=legacy and csvScenario=1." << std::endl;
    return false;
  }
  if (!std::isfinite(caProxyAckBatchMs) || caProxyAckBatchMs<1 || caProxyAckBatchMs>100) {
    std::cerr << "--caProxyAckBatchMs must be in [1,100]." << std::endl;
    return false;
  }
  if ((caLinkGate!="distance" && caLinkGate!="handshake") ||
      !std::isfinite(caLinkProbeInterval) || caLinkProbeInterval<.01 || caLinkProbeInterval>60 ||
      !std::isfinite(caLinkProbeTimeout) || caLinkProbeTimeout<.001 || caLinkProbeTimeout>=caLinkProbeInterval ||
      !std::isfinite(caLinkValidity) || caLinkValidity<caLinkProbeInterval || caLinkValidity>120 ||
      (caLinkGate=="handshake" && (!caLinkProbes || disableAsym || !caRangeGating))) {
    std::cerr<<"Invalid CA link gate/probe windows; handshake requires probes, asym master and range gate."<<std::endl;return false;
  }
  if (protocol!="cafbcr" && (caLinkProbes || caLinkGate!="distance" || caLinkProbeInterval!=2 || caLinkProbeTimeout!=.25 || caLinkValidity!=6)) {
    std::cerr<<"Link measurement controls require --protocol=cafbcr."<<std::endl;return false;
  }
  if (protocol != "cafbcr" && (disableAsym || !caProxyJoin || !caProxyRenewal || !caRangeGating ||
      caProxyRenewalPolicy!="eager" || caDirectCmPolicy!="legacy" || caControlUnicastPolicy!="legacy" || caProxyRenewalWindowMs!=155 || caProxyAckBatchMs!=20)) {
    std::cerr << "CA mechanism switches require --protocol=cafbcr." << std::endl;
    return false;
  }
  if (helloMethod != "fixed" && helloMethod != "fuzzy" && helloMethod != "qlearning") {
    std::cerr << "Unknown --helloMethod; choose fixed, fuzzy or qlearning." << std::endl;
    return false;
  }
  if (caQHelloDiagnostics && (protocol != "cafbcr" || helloMethod != "qlearning")) {
    std::cerr << "caQHelloDiagnostics requires protocol=cafbcr and helloMethod=qlearning." << std::endl;
    return false;
  }
  std::cout << "[Comparison] schema=" << comparison::SCHEMA
            << " taskSelection=" << taskSelection << std::endl;
  if (protocol == "cafbcr") {
    std::cout << "[CA mechanisms] schema=cafbcr-asym-controls-v1 proxyJoin="
              << (!disableAsym && caProxyJoin ? "true" : "false")
              << " proxyRenewal=" << (!disableAsym && caProxyJoin && caProxyRenewal ? "true" : "false")
              << " renewalPolicy=" << caProxyRenewalPolicy
              << " renewalWindowMs=" << caProxyRenewalWindowMs
              << " rangeGating=" << (!disableAsym && caRangeGating ? "true" : "false") << std::endl;
  }

  // Set log level
  if (verbose) {
    LogComponentEnable("CafbcrSimulation", LOG_LEVEL_INFO);
    LogComponentEnable("cafbcrRoutingProtocol", LOG_LEVEL_INFO);
    LogComponentEnable("CafbcrComputeTaskApp", LOG_LEVEL_INFO);
    LogComponentEnable("CafbcrScenarioConfig", LOG_LEVEL_INFO);
  }

  CreateScenarioConfig();

  try {
    if (!std::isfinite(olsrHelloInterval) || olsrHelloInterval <= 0 ||
        olsrHelloInterval >= Time::Max().GetSeconds() / 2 ||
        !Seconds(olsrHelloInterval).IsStrictlyPositive() ||
        (protocol != "olsr" && olsrHelloInterval != 0.25))
      throw std::invalid_argument("olsrHelloInterval must be a positive representable time and is OLSR-only");
    if (protocol == "qrcf") {
      qrcfSettings.ApplyPreset(qrcfPreset);
      qrcfSettings.range = config->GetMaxCommunicationRange();
      qrcfSettings.Validate();
    } else if (qrcfPreset != "outcome-bellman") {
      throw std::invalid_argument("qrcfPreset requires protocol=qrcf");
    }
    if (UsesMo()) {
      if (!moDiagnostics.empty() && (std::filesystem::exists(moDiagnostics) ||
          std::filesystem::absolute(moDiagnostics).lexically_normal() == std::filesystem::absolute(resultsCsv).lexically_normal() ||
          (!moSummary.empty() && std::filesystem::absolute(moDiagnostics).lexically_normal() == std::filesystem::absolute(moSummary).lexically_normal())))
        throw std::invalid_argument("moDiagnostics must be a new directory distinct from CSV and moSummary");
      if (taskWire != "header-v1") throw std::invalid_argument("MOGNN CA bridge requires header-v1");
      if (!moSummary.empty() && (std::filesystem::exists(moSummary) ||
          std::filesystem::absolute(moSummary).lexically_normal() == std::filesystem::absolute(resultsCsv).lexically_normal()))
        throw std::invalid_argument("moSummary must be a new file distinct from resultsCsv");
      mognnAdapter = std::make_unique<MognnAdapter>();
      mognnAdapter->Configure(moSettings, config->GetNodeConfigs(), protocol, taskSelection,
                             taskGenStop - taskGenStart, enableComputeTask);
    } else if (!moSettings.model.empty() || moSettings.relayMode != "legacy" || !moSummary.empty() || !moDiagnostics.empty() || moSettings.slotSeconds != .01 ||
               moSettings.linkBps != 6e6 || moSettings.helloSeconds != 1 || moSettings.holdSeconds != 4 ||
               moSettings.relayBatchWindowMs != 20 || moSettings.relaySuppressDuplicates ||
               moSettings.serverHelloScale != 1.0) {
      throw std::invalid_argument("MO settings require a MOGNN protocol");
    }
  } catch (const std::exception& error) {
    std::cerr << "Invalid baseline configuration: " << error.what() << std::endl;
    return false;
  }
  std::cout << "[Baseline] profile=" << BaselineProfileJson() << std::endl;

  if (!config->Validate()) {
    std::cerr << "Configuration validation failed!" << std::endl;
    return false;
  }
  auto demandError = demand::Validate(config->GetNodeCount(), numTestFlows);
  if (!demandError.empty()) {
    std::cerr << demandError << std::endl;
    return false;
  }
  if (udpStart >= 0 && numTestFlows > 0) {
    try {
      auto lastStart = udp_demand::WindowStart(Seconds(udpStart), Seconds(.5), numTestFlows - 1, numTestFlows);
      udp_demand::WindowPlan(Seconds(udpStart), Seconds(udpStop), Seconds(.5));
      udp_demand::WindowPlan(lastStart, Seconds(udpStop), Seconds(.5));
    } catch (const std::invalid_argument& error) {
      std::cerr << error.what() << std::endl;
      return false;
    }
  }
  if (std::max(20.0, std::floor(simulationTime - 50.0)) > std::numeric_limits<uint32_t>::max()) {
    std::cerr << "UDP packet cap exceeds uint32_t" << std::endl;
    return false;
  }

  for (const auto& node : config->GetNodeConfigs()) {
    if (enableComputeTask && node.generateTasks) maximumTaskDeadline = std::max(maximumTaskDeadline, node.maxTaskDeadline);
  }
  taskDeadlineCoverage = Seconds(taskGenStop) + Seconds(maximumTaskDeadline) <= Seconds(simulationTime);
  if (requireTaskDeadlineCoverage && !taskDeadlineCoverage) {
    std::cerr << "Task window does not cover all deadlines: time must be >= taskGenStop + "
              << maximumTaskDeadline << " seconds." << std::endl;
    return false;
  }
  std::cout << "[Task observation] schema=" << TaskObservationWriter::SCHEMA
            << " generation=[" << taskGenStart << ',' << taskGenStop << ")"
            << " taskStartMode=" << taskStartMode
            << " observationStop=" << simulationTime
            << " deadlineCoverage=" << (taskDeadlineCoverage ? "true" : "false")
            << " formalEligible=false" << std::endl;

  if (csvScenario) {
    scenarioContract = MakeScenarioContract();
    std::cout << "[Scenario] " << scenarioContract->Preview() << std::endl;
  }
  return true;
}

// ==================== CreateScenarioConfig ====================
void CafbcrSimulation::CreateScenarioConfig()
{
  // 当 --size 给定（>0）时，按 FANET 现实部署比例分配节点：
  //   COMMAND ≈ ceil(N/300)   小型大型基站，几乎不增长
  //   RELAY   ≈ 6%·N          中继骨干，少量
  //   TASK    ≈ 16%·N          中等任务节点
  //   SWARM   = N − 三者       小型/廉价/可大规模部署，占大头
  if (totalSize > 0) {
    numCommandNodes = std::max(1u, (uint32_t)std::ceil((double)totalSize / 300.0));
    numRelayNodes   = (uint32_t)std::round(0.06 * totalSize);
    numTaskNodes    = (uint32_t)std::round(0.16 * totalSize);
    uint32_t allocated = numCommandNodes + numRelayNodes + numTaskNodes;
    numSwarmNodes   = (allocated < totalSize) ? (totalSize - allocated) : 0;
    std::cout << "[size=" << totalSize << "] 分配: COMMAND=" << numCommandNodes
              << " RELAY=" << numRelayNodes << " TASK=" << numTaskNodes
              << " SWARM=" << numSwarmNodes << std::endl;
  }

  config = CreateObject<ScenarioConfig>();
  config->AddNodeType(numCommandNodes, UavTypeConfig::CreateCommandNode());
  config->AddNodeType(numRelayNodes, UavTypeConfig::CreateRelayNode());
  config->AddNodeType(numTaskNodes, UavTypeConfig::CreateTaskNode());
  config->AddNodeType(numSwarmNodes, UavTypeConfig::CreateSwarmNode());
  config->SetSimulationTime(simulationTime);
  config->SetSimulationArea(Vector(2000.0, 2000.0, 500.0));
  config->SetMaxCommunicationRange(500.0);
  // 适应度权重顺序严格对齐 ClusterNode::ComputeFitness:
  //   (LHTP, Energy, Degree, Distance, Compute)
  if (sweepWc >= 0.0) {
    // 计算权重敏感性扫描：wC 为计算能力权重，其余四维(LHTP/Energy/Degree/Distance)
    // 均分剩余预算 (1 - wC)/4。wC=0.20 时退化为五维等权(与默认配置一致)。
    double wc   = std::min(0.40, sweepWc);
    double rest = (1.0 - wc) / 4.0;
    config->SetFitnessWeights(rest, rest, rest, rest, wc);
    std::cout << "[wC sweep] each-other=" << rest << " wC=" << wc << std::endl;
  } else {
    // 默认配置：五维等权
    config->SetFitnessWeights(0.20, 0.20, 0.20, 0.20, 0.20);
  }
}

// ==================== CreateNodes ====================
// 职责：创建节点 + 配置异构移动模型
void CafbcrSimulation::CreateNodes()
{
  uint32_t totalNodes = config->GetNodeCount();
  allNodes.Create(totalNodes);

  std::cout << "[1] 创建 " << totalNodes << " 个节点" << std::endl;

  // Step 1: 配置节点的计算和存储属性
  uint32_t nodeIndex = 0;
  const auto& nodeConfigs = config->GetNodeConfigs();

  for (auto it = allNodes.Begin(); it != allNodes.End(); ++it, ++nodeIndex) {
    Ptr<Node> node = *it;
    const UavTypeConfig& nodeConfig = nodeConfigs[nodeIndex];

    Ptr<ClusterNode> clusterNode = CreateObject<ClusterNode>();
    // The shared ClusterId attribute defaults to 0 and overrides the
    // constructor sentinel. Initialize only CA's unassociated nodes here.
    if (protocol == "cafbcr") clusterNode->SetClusterId(0xFFFF);
    clusterNode->SetNodeId(node->GetId());
    clusterNode->SetNetworkScale(totalNodes);
    clusterNode->SetFitnessMode(fitnessMode);
    clusterNode->BindNode(node);
    clusterNode->SetComputeCapacity(nodeConfig.computeCapacity);
    clusterNode->SetTaskQueueCapacity(nodeConfig.taskQueueCapacity);

    node->AggregateObject(clusterNode);
  }






  // Step 2: 配置异构移动模型
  // 先配置随机初始位置（所有节点）
  MobilityHelper mobility;
  mobility.SetPositionAllocator("ns3::RandomBoxPositionAllocator",
                               "X", StringValue("ns3::UniformRandomVariable[Min=0|Max=2000]"),
                               "Y", StringValue("ns3::UniformRandomVariable[Min=0|Max=2000]"),
                               "Z", StringValue("ns3::UniformRandomVariable[Min=0|Max=500]"));

  // 对不同节点类型分别配置移动模型
  // 分离处理：COMMAND节点 vs 其他节点
  NodeContainer commandNodes, otherNodes;
  nodeIndex = 0;
  for (auto it = allNodes.Begin(); it != allNodes.End(); ++it, ++nodeIndex) {
    const UavTypeConfig& nodeConfig = nodeConfigs[nodeIndex];
    if (nodeConfig.nodeType == COMMAND) {
      commandNodes.Add(*it);
    } else {
      otherNodes.Add(*it);
    }
  }

  // 为COMMAND节点配置：静止
  MobilityHelper commandMobility;
  commandMobility.SetPositionAllocator("ns3::RandomBoxPositionAllocator",
                                       "X", StringValue("ns3::UniformRandomVariable[Min=0|Max=2000]"),
                                       "Y", StringValue("ns3::UniformRandomVariable[Min=0|Max=2000]"),
                                       "Z", StringValue("ns3::UniformRandomVariable[Min=0|Max=500]"));
  commandMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  commandMobility.Install(commandNodes);

  // 为其他节点配置：移动模型
  // RELAY: 5-8 m/s, TASK: 10-15 m/s, SWARM: 15-25 m/s
  // 使用GaussMarkovMobilityModel实现异构速度
  nodeIndex = 0;
  for (auto it = otherNodes.Begin(); it != otherNodes.End(); ++it) {
    // 找到原始节点索引
    uint32_t origIndex = 0;
    for (auto origIt = allNodes.Begin(); origIt != allNodes.End(); ++origIt, ++origIndex) {
      if (*origIt == *it) break;
    }

    const UavTypeConfig& nodeConfig = nodeConfigs[origIndex];
    Ptr<Node> node = *it;

    // 为这个节点创建随机位置分配器
    MobilityHelper nodeMobility;
    nodeMobility.SetPositionAllocator("ns3::RandomBoxPositionAllocator",
                                      "X", StringValue("ns3::UniformRandomVariable[Min=0|Max=2000]"),
                                      "Y", StringValue("ns3::UniformRandomVariable[Min=0|Max=2000]"),
                                      "Z", StringValue("ns3::UniformRandomVariable[Min=0|Max=500]"));


                                      
    // 根据节点类型设置速度（异构）：
    //   SWARM = [minSpeed, maxSpeed]                    用户输入
    //   TASK  = [0.62·minSpeed, 0.62·maxSpeed]          中等机动
    //   RELAY = [0.33·minSpeed, 0.33·maxSpeed]          慢速骨干
    // 比例来自原硬编码 (RELAY 5-8, TASK 10-15, SWARM 15-25)
    double sMin, sMax;
    if (nodeConfig.nodeType == RELAY) {
      sMin = minSpeed * 0.33;
      sMax = maxSpeed * 0.33;
    } else if (nodeConfig.nodeType == TASK) {
      sMin = minSpeed * 0.62;
      sMax = maxSpeed * 0.62;
    } else { // SWARM
      sMin = minSpeed;
      sMax = maxSpeed;
    }
    std::ostringstream speedOss;
    speedOss << "ns3::UniformRandomVariable[Min=" << sMin << "|Max=" << sMax << "]";
    std::string speedStr = speedOss.str();

    nodeMobility.SetMobilityModel("ns3::GaussMarkovMobilityModel",
                                 "Bounds", BoxValue(Box(0, 2000, 0, 2000, 0, 500)),
                                 "TimeStep", TimeValue(Seconds(0.5)),
                                 "Alpha", DoubleValue(0.85),
                                 "MeanVelocity", StringValue(speedStr),
                                 "MeanDirection", StringValue("ns3::UniformRandomVariable[Min=0|Max=6.283]"),
                                 "MeanPitch", StringValue("ns3::UniformRandomVariable[Min=-0.1|Max=0.1]"));
    nodeMobility.Install(node);
  }

}

// ==================== CreateDevices ====================
// 职责：创建异构网络设备（不同节点不同通信能力）
void CafbcrSimulation::CreateDevices()
{
  std::cout << "[2] 创建异构网络设备" << std::endl;

  WifiMacHelper wifiMac;
  wifiMac.SetType("ns3::AdhocWifiMac");

  // legacy preserves Friis+600m. shadow adds movement-correlated log-normal
  // shadowing, with an independent hard-cutoff control. All protocols in this
  // CA scenario use this channel; benchmarktest does not include this helper.
  Ptr<YansWifiChannel> channel = wireless::CreateChannel(allNodes, channelConfig, shadowModel, taskOutputDir);

  WifiHelper wifi;
  wifi.SetStandard(WIFI_STANDARD_80211g);

  WifiMacHelper wifiMacInner;
  wifiMacInner.SetType("ns3::AdhocWifiMac");

  // 按节点类型逐个安装，配置异构 TxPower/RxSensitivity/DataRate
  const auto& nodeConfigs = config->GetNodeConfigs();

  for (uint32_t i = 0; i < allNodes.GetN(); i++) {
    const UavTypeConfig& nodeConfig = nodeConfigs[i];

    const auto radio = scenario::Resolve(nodeConfig.nodeType, asymLevel, asymRangeProfile,
                                         nodeConfig.maxCommunicationRange);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode", StringValue(radio.dataMode));

    CafbcrHeaderBoundaryYansWifiPhyHelper wifiPhy(phyHeaderBoundaryGuard);
    wifiPhy.SetChannel(channel);
    wifiPhy.Set("TxPowerStart", DoubleValue(radio.txDbm));
    wifiPhy.Set("TxPowerEnd", DoubleValue(radio.txDbm));
    wifiPhy.Set("RxSensitivity", DoubleValue(radio.rxDbm));

    NodeContainer singleNode(allNodes.Get(i));
    NetDeviceContainer dev = wifi.Install(wifiPhy, wifiMacInner, singleNode);
    if (csvScenario) {
      auto installed = DynamicCast<WifiNetDevice>(dev.Get(0))->GetPhy();
      // RxSensitivity is stored as watts; its dBm getter round-trips through log10.
      NS_ABORT_MSG_IF(std::abs(installed->GetTxPowerStart() - radio.txDbm) > 1e-10 ||
                      std::abs(installed->GetTxPowerEnd() - radio.txDbm) > 1e-10 ||
                      std::abs(installed->GetRxSensitivity() - radio.rxDbm) > 1e-10,
                      "Installed radio differs from scenario contract");
    }
    devices.Add(dev);
  }
}

// ==================== InstallEnergyModel ====================
// 职责：异构 BasicEnergySource + WifiRadioEnergyModel。
//
//  - BasicEnergySource: 按 UavTypeConfig.batteryCapacity 异构
//    (COMMAND 50000J / RELAY 30000J / TASK 20000J / SWARM 15000J)
//  - WifiRadioEnergyModel: 按 idlePowerConsumption 推导 idle 电流，
//    Tx/Rx/Switch 用经验比例。挂到 PHY StateHelper 上，让能耗随
//    Tx/Rx/CcaBusy 状态切换实时累积；
//    实验中可观察到：
//      - 总能耗随时间增长
//      - 转发频繁 / 簇头节点消耗更快
//      - 长仿真下出现节点掉电 → 触发簇头重选
//
// Constraint: do not call GetObject<ClusterNode> inside this function. The
// aggregate LRU reorder would make a later internet.Install match ClusterNode
// instead of Node and corrupt the device vector. Energy-field synchronization
// happens after InstallInternetStack instead.
void CafbcrSimulation::InstallEnergyModel()
{
  std::cout << "[2.5] 安装能量模型（异构）" << std::endl;

  std::vector<UavTypeConfig> nodeConfigs = config->GetNodeConfigs();
  const double voltage = 3.0;
  uint32_t perType[4] = {0, 0, 0, 0};

  for (uint32_t i = 0; i < allNodes.GetN(); ++i) {
    const UavTypeConfig& nc = nodeConfigs[i];
    perType[nc.nodeType]++;

    // 1. 异构电池容量
    BasicEnergySourceHelper energyHelper;
    energyHelper.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(nc.batteryCapacity));
    energyHelper.Set("BasicEnergySupplyVoltageV", DoubleValue(voltage));

    NodeContainer singleNode(allNodes.Get(i));
    EnergySourceContainer sources = energyHelper.Install(singleNode);

    // 2. 异构无线电功耗（由配置反推电流）
    double idleCurrent   = nc.idlePowerConsumption / voltage;
    double txCurrent     = idleCurrent * 3.0;   // 发射 ~3× 静置
    double rxCurrent     = idleCurrent * 1.5;   // 接收 ~1.5× 静置
    double sleepCurrent  = idleCurrent * 0.1;
    double switchCurrent = idleCurrent;

    WifiRadioEnergyModelHelper radioHelper;
    radioHelper.Set("IdleCurrentA",      DoubleValue(idleCurrent));
    radioHelper.Set("CcaBusyCurrentA",   DoubleValue(idleCurrent));
    radioHelper.Set("TxCurrentA",        DoubleValue(txCurrent));
    radioHelper.Set("RxCurrentA",        DoubleValue(rxCurrent));
    radioHelper.Set("SleepCurrentA",     DoubleValue(sleepCurrent));
    radioHelper.Set("SwitchingCurrentA", DoubleValue(switchCurrent));

    NetDeviceContainer singleDevice(devices.Get(i));
    radioHelper.Install(singleDevice, sources);
  }

  std::cout << "  电池/功耗按类型异构: COMMAND=" << perType[COMMAND]
            << " RELAY=" << perType[RELAY]
            << " TASK=" << perType[TASK]
            << " SWARM=" << perType[SWARM] << std::endl;
}

// ==================== InstallInternetStack ====================
// 职责：安装协议栈和路由协议（仅限网络层）
void CafbcrSimulation::InstallInternetStack()
{
  std::cout << "[3] 安装网络协议栈 (" << protocol << ")" << std::endl;

  // 安装Internet Stack
  InternetStackHelper internet;
  Ipv4ListRoutingHelper list;

  // 根据protocol参数选择路由协议
  if (protocol == "cafbcr") {
    cafbcrHelper cafbcrRouting;
    cafbcrRouting.SetFitnessMode(fitnessMode);
    cafbcrRouting.SetHelloIntervalMethod(helloMethod);
    cafbcrRouting.SetHelloInterval(Seconds(config->GetHelloInterval()));
    cafbcrRouting.SetBeaconInterval(Seconds(config->GetBeaconInterval()));
    cafbcrRouting.SetMaxRange(config->GetMaxCommunicationRange());
    cafbcrRouting.Set("EnableAsymHandling", BooleanValue(!disableAsym));
    cafbcrRouting.Set("EnableProxyJoin", BooleanValue(caProxyJoin));
    cafbcrRouting.Set("EnableProxyRenewal", BooleanValue(caProxyRenewal));
    cafbcrRouting.Set("CoordinateProxyRenewal", BooleanValue(
      caProxyRenewalPolicy=="ack-stagger" || caProxyRenewalPolicy=="ack-batch" ||
      caProxyRelayPolicy=="ack-stagger"));
    cafbcrRouting.Set("DirectMaintenanceFeedback", BooleanValue(caProxyRenewalPolicy=="direct-feedback"));
    cafbcrRouting.Set("FeedbackCoveredCmResponse", BooleanValue(caDirectCmPolicy=="feedback-covered"));
    cafbcrRouting.Set("OneHopControlUnicast", BooleanValue(caControlUnicastPolicy=="one-hop"));
    cafbcrRouting.Set("CoalesceProxyRenewalAck", BooleanValue(caProxyRenewalPolicy=="ack-batch"));
    cafbcrRouting.Set("ProxyRenewalAckBatchWindow", TimeValue(Seconds(caProxyAckBatchMs/1000)));
    cafbcrRouting.Set("ProxyRenewalWindow", TimeValue(Seconds(caProxyRenewalWindowMs/1000)));
    cafbcrRouting.Set("EnableRangeGating", BooleanValue(caRangeGating));
    cafbcrRouting.Set("EnableLinkProbes",BooleanValue(caLinkProbes));
    cafbcrRouting.Set("UseHandshakeGate",BooleanValue(caLinkGate=="handshake"));
    cafbcrRouting.Set("LinkProbeInterval",TimeValue(Seconds(caLinkProbeInterval)));
    cafbcrRouting.Set("LinkProbeTimeout",TimeValue(Seconds(caLinkProbeTimeout)));
    cafbcrRouting.Set("LinkEvidenceValidity",TimeValue(Seconds(caLinkValidity)));
    cafbcrRouting.Set("TerminateRepeatedForward", BooleanValue(caLoopPolicy == "terminate-revisit"));
    cafbcrRouting.Set("AlternateRepeatedForward", BooleanValue(caLoopPolicy == "alternate-revisit"));

    double wLHTP, wEnergy, wDegree, wDistance, wCompute;
    config->GetFitnessWeights(wLHTP, wEnergy, wDegree, wDistance, wCompute);
    cafbcrRouting.SetFitnessWeights(wLHTP, wEnergy, wDegree, wDistance, wCompute);

    list.Add(cafbcrRouting, 0);
  }

  if (protocol == "aodv") {
    AodvHelper aodv;
    list.Add(aodv, 0);
  }

  if (protocol == "olsr") {
    OlsrHelper olsr;
    olsr.Set("HelloInterval", TimeValue(Seconds(olsrHelloInterval)));
    list.Add(olsr, 0);
  }

  if (protocol == "cfr-eng") {
    CfrEngHelper helper;
    list.Add(helper, 0);
  }
  if (UsesMo()) {
    MoHelper helper;
    helper.Set("HelloInterval", TimeValue(Seconds(moSettings.helloSeconds)));
    helper.Set("HoldTime", TimeValue(Seconds(moSettings.holdSeconds)));
    list.Add(helper, 0);
  }

  if (protocol == "qrcf") {
    QrcfHelper helper;
    helper.SetSettings(qrcfSettings);
    list.Add(helper, 0);
  }
  if (protocol == "icra") {
    icraHelper icra;
    icra.SetQLearningParameters(0.1, 0.5, 0.1);  // alpha, lambda, epsilon
    list.Add(icra, 0);
  }

  internet.SetRoutingHelper(list);
  internet.Install(allNodes);
  if (protocol == "cafbcr") {
    cafbcrHelper streams;
    streams.AssignStreams(allNodes, 2000000);
    for (auto node = allNodes.Begin(); node != allNodes.End(); ++node) {
      auto neighborRoute = (*node)->GetObject<cafbcr::RoutingProtocol>();
      if (neighborObserver) neighborObserver->Attach(neighborRoute);
      if (joinObserver) joinObserver->Attach((*node)->GetObject<cafbcr::RoutingProtocol>());
      if (linkObserver) linkObserver->Attach((*node)->GetObject<cafbcr::RoutingProtocol>());
      if (loopObserver) loopObserver->Attach((*node)->GetId(), neighborRoute);
      if (recoveryObserver) recoveryObserver->Attach((*node)->GetId(), neighborRoute);
    }
  }

  // Synchronize initial energy to ClusterNode only after internet.Install.
  // ClusterNode derives from Node; GetObject<ClusterNode> triggers an aggregate
  // LRU reorder that makes a later GetObject<Node> hit ClusterNode, which
  // corrupts m_devices in Ipv4L3Protocol::SetupLoopback.
  std::vector<UavTypeConfig> nodeConfigs = config->GetNodeConfigs();
  for (uint32_t i = 0; i < allNodes.GetN() && i < nodeConfigs.size(); ++i) {
    Ptr<ClusterNode> clusterNode = allNodes.Get(i)->GetObject<ClusterNode>();
    if (clusterNode) {
      clusterNode->SetOriginEnergy(nodeConfigs[i].batteryCapacity);
      clusterNode->updateResidualEnergy(nodeConfigs[i].batteryCapacity);
      // Protocol nominal range; stochastic PHY reception is evaluated independently.
      const auto radio = scenario::Resolve(nodeConfigs[i].nodeType, asymLevel, asymRangeProfile,
                                           nodeConfigs[i].maxCommunicationRange);
      clusterNode->SetMaxRange(radio.range);
    }
  }

  // 配置IP地址
  Ipv4AddressHelper address;
  address.SetBase("10.0.0.0", "255.255.0.0");  // /16子网支持65K节点
  interfaces = address.Assign(devices);
  if (mognnAdapter) mognnAdapter->Install(allNodes, interfaces);
  if (!moDiagnostics.empty()) {
    moDiagnosticObserver = std::make_unique<MognnDiagnostics>(moDiagnostics, Seconds(taskGenStart),
                                                           Seconds(taskGenStop), Seconds(simulationTime));
    for (uint32_t i = 0; i < allNodes.GetN(); ++i) moDiagnosticObserver->Attach(allNodes.Get(i));
  }

  if (protocol == "cfr-eng") {
    cfrAdapter = std::make_unique<cfreng::LocalAdapter>();
    cfrAdapter->Install(allNodes, interfaces, nodeConfigs, Seconds(simulationTime), cfrEngCompute,
        [this](const ComputeTaskEvent& event) { taskObserver->ObserveExternalExecution(event); });
  }

  if (protocol == "qrcf") {
    for (uint32_t i = 0; i < allNodes.GetN(); ++i) {
      auto routing = allNodes.Get(i)->GetObject<qrcf::RoutingProtocol>();
      NS_ABORT_MSG_IF(!routing, "QRCF routing protocol is not installed");
      // Imported QRCF's ideal destination-position service, without remote Q-table access.
      // This lookup is simulation information, not a measured radio location service.
      routing->SetDestinationLookup([this](Ipv4Address destination) -> std::optional<Vector> {
        for (uint32_t j = 0; j < allNodes.GetN(); ++j)
          if (interfaces.GetAddress(j) == destination)
            return allNodes.Get(j)->GetObject<MobilityModel>()->GetPosition();
        return {};
      });
    }
    QrcfHelper helper;
    helper.AssignStreams(allNodes, 1000);
  }
  // 可选：挂 ipv4 / MAC / PHY drop trace 用于定位丢包源
  // 必须在 internet.Install 之后才能拿到 Ipv4L3Protocol
  if (traceDrops) {
    std::cout << "[3.5] 挂载 ipv4/MAC/PHY drop trace" << std::endl;
    for (uint32_t i = 0; i < allNodes.GetN(); ++i) {
      Ptr<Node> n = allNodes.Get(i);
      Ptr<Ipv4> ip = n->GetObject<Ipv4>();
      Ptr<Ipv4L3Protocol> ipL3 = DynamicCast<Ipv4L3Protocol>(ip);
      if (ipL3) {
        m_ipv4ToNodeIdx[ip] = i;       // 同时建 ipv4→nodeIdx 映射
        ipL3->TraceConnectWithoutContext(
            "Drop", MakeCallback(&CafbcrSimulation::OnIpv4Drop, this));
      }
    }
    // PHY/MAC drop trace：按 device 索引找到节点，然后绑 nodeIdx
    for (uint32_t i = 0; i < devices.GetN(); ++i) {
      Ptr<WifiNetDevice> wdev = DynamicCast<WifiNetDevice>(devices.Get(i));
      if (!wdev) continue;

      // PHY drops（全局计数即可）
      Ptr<WifiPhy> phy = wdev->GetPhy();
      if (phy) {
        phy->TraceConnectWithoutContext(
            "PhyTxDrop", MakeCallback(&CafbcrSimulation::OnPhyTxDrop, this));
        phy->TraceConnectWithoutContext(
            "PhyRxDrop", MakeCallback(&CafbcrSimulation::OnPhyRxDrop, this));
      }

      // MAC drops（用 context 字符串透传 nodeIdx）
      Ptr<WifiMac> mac = wdev->GetMac();
      if (mac) {
        mac->TraceConnect(
            "MacTxDrop", std::to_string(i),
            MakeCallback(&CafbcrSimulation::OnMacTxDropCtx, this));
        // DroppedMpdu 含重传耗尽 (WIFI_MAC_DROP_REACHED_RETRY_LIMIT)
        mac->TraceConnect(
            "DroppedMpdu", std::to_string(i),
            MakeCallback(&CafbcrSimulation::OnDroppedMpduCtx, this));
      }
    }
  }
}

// ==================== Drop trace callbacks ====================
void CafbcrSimulation::OnMacTxDropCtx(std::string context, Ptr<const Packet> /*p*/)
{
  uint32_t nodeIdx = 0;
  try { nodeIdx = std::stoul(context); } catch (...) { nodeIdx = UINT32_MAX; }
  m_macTxDrop++;
  m_macDropByNodeIdx[nodeIdx]++;
}

void CafbcrSimulation::OnDroppedMpduCtx(std::string context,
                                       WifiMacDropReason reason,
                                       Ptr<const WifiMpdu> /*mpdu*/)
{
  uint32_t nodeIdx = 0;
  try { nodeIdx = std::stoul(context); } catch (...) { nodeIdx = UINT32_MAX; }
  m_macMpduDrop++;
  m_macMpduDropByReason[(uint32_t)reason]++;
  m_macMpduDropByNodeIdx[nodeIdx]++;
}

// ==================== Drop trace callbacks ====================
void CafbcrSimulation::OnIpv4Drop(const Ipv4Header& header,
                                 Ptr<const Packet> /*p*/,
                                 Ipv4L3Protocol::DropReason reason,
                                 Ptr<Ipv4> ipv4,
                                 uint32_t /*iface*/)
{
  m_dropTotal++;
  m_dropByReason[(uint32_t)reason]++;

  // 节点索引：用 m_ipv4ToNodeIdx 映射避免 ClusterNode aggregate 错位
  auto it = m_ipv4ToNodeIdx.find(ipv4);
  if (it != m_ipv4ToNodeIdx.end()) {
    m_dropByNodeIdx[it->second]++;
  }

  // 是否是 UDP 测试流被丢的
  Ipv4Address src = header.GetSource();
  Ipv4Address dst = header.GetDestination();
  for (const auto& f : m_testFlows) {
    if (src == f.srcAddr && dst == f.dstAddr) {
      m_dropTestFlow++;
      break;
    }
  }
}

void CafbcrSimulation::OnPhyTxDrop(Ptr<const Packet> /*p*/) { m_phyTxDrop++; }

void CafbcrSimulation::OnPhyRxDrop(Ptr<const Packet> /*p*/,
                                  WifiPhyRxfailureReason /*reason*/) {
  m_phyRxDrop++;
}

// ==================== InstallApplications ====================
// 职责：安装应用层（ComputeTask应用 + UDP测试流 + FlowMonitor）
void CafbcrSimulation::InstallApplications()
{
  std::cout << "[4] 安装应用层" << std::endl;
  std::ofstream taskInputs;
  if (!csvOnly) taskInputs.open(std::filesystem::path(taskOutputDir) / "task-inputs.jsonl");
  taskInputs << std::setprecision(17);
  NS_ABORT_MSG_IF(!csvOnly && !taskInputs, "Cannot open task demand configuration output");
  std::ofstream assignments;
  if (!csvOnly) assignments.open(std::filesystem::path(taskOutputDir) / "server-assignments.jsonl");
  NS_ABORT_MSG_IF(!csvOnly && !assignments, "Cannot open configured server assignment output");

  // 第一步：安装FlowMonitor（必须在应用安装前）
  monitor = flowmon.InstallAll();

  // Step 2: install the compute-task applications (all protocols; can be disabled).
  if (enableComputeTask) {
    uint32_t nodeIndex = 0;
    const auto& nodeConfigs = config->GetNodeConfigs();
    const auto fixedAssignments = comparison::FixedAssignments(nodeConfigs);

    // 收集高算力节点（COMMAND + RELAY）作为卸载目标
    // 低算力节点(TASK/SWARM)将任务卸载到高算力节点，触发CA-BCR路由
    std::vector<uint32_t> highComputeIndices;
    for (uint32_t i = 0; i < allNodes.GetN(); ++i) {
      if (nodeConfigs[i].nodeType == COMMAND || nodeConfigs[i].nodeType == RELAY) {
        highComputeIndices.push_back(i);
      }
    }
    uint32_t offloadRoundRobin = 0;

    for (auto it = allNodes.Begin(); it != allNodes.End(); ++it, ++nodeIndex) {
      Ptr<Node> node = *it;
      const UavTypeConfig& nodeConfig = nodeConfigs[nodeIndex];

      // 安装客户端应用（生成任务）
      if (nodeConfig.generateTasks) {
        Ptr<cafbcr::ComputeTaskClientApp> clientApp =
            CreateObject<cafbcr::ComputeTaskClientApp>();

        // Compute-aware scheduling: every server listens on port 9999, and a
        // client selects a local server with high compute capacity and low
        // load from its neighbor table for each task. The configured remote
        // address remains a fallback when no neighbor qualifies.
        constexpr uint16_t kComputeTaskPort = 9999;
        uint32_t targetIndex = nodeIndex;
        if (!highComputeIndices.empty() &&
            (nodeConfig.nodeType == TASK || nodeConfig.nodeType == SWARM)) {
          targetIndex = highComputeIndices[offloadRoundRobin % highComputeIndices.size()];
          offloadRoundRobin++;
        }
        Ipv4Address targetAddress = interfaces.GetAddress(targetIndex);
        if (taskSelection == "fixed") {
          const auto target = fixedAssignments[nodeIndex];
          targetAddress = target ? interfaces.GetAddress(*target) : Ipv4Address::GetAny();
          clientApp->SetServerSelectionPolicy(ComputeTaskClientApp::ServerSelectionPolicy::FIXED_ASSIGNMENT);
        }

        if (!csvOnly) assignments << "{\"schema\":" << TaskObservationWriter::Quote(comparison::ASSIGNMENT_SCHEMA)
          << ",\"source_node\":" << node->GetId() << ",\"source_index\":" << nodeIndex
          << ",\"selection_mode\":" << TaskObservationWriter::Quote(taskSelection)
          << ",\"target_use\":\"" << (taskSelection == "fixed" ? "fixed_assignment" : "native_fallback") << '"'
          << ",\"target_index\":" << (taskSelection == "fixed"
              ? (fixedAssignments[nodeIndex] ? std::to_string(*fixedAssignments[nodeIndex]) : "null")
              : std::to_string(targetIndex))
          << ",\"target_address\":" << TaskObservationWriter::Quote([&] { std::ostringstream ip; ip << targetAddress; return ip.str(); }())
          << ",\"information_source\":\"offline_server_membership_and_type\"}\n";

        clientApp->SetRemoteAddress(targetAddress);
        clientApp->SetUseTaskHeader(taskWire == "header-v1");
        clientApp->SetRemotePort(kComputeTaskPort);
        clientApp->SetTaskInterval(Seconds(nodeConfig.taskGenerationInterval));
        clientApp->SetTaskWorkload(nodeConfig.minTaskWorkload, nodeConfig.maxTaskWorkload);
        clientApp->SetTaskDataSize(nodeConfig.minTaskDataSize, nodeConfig.maxTaskDataSize);
        clientApp->SetTaskDeadline(Seconds(nodeConfig.minTaskDeadline),
                                  Seconds(nodeConfig.maxTaskDeadline));
        Time generationStart = taskStartMode == "bounded-1s"
            ? demand::BoundedTaskStart(Seconds(taskGenStart), nodeIndex, allNodes.GetN())
            : demand::TaskStart(Seconds(taskGenStart), nodeIndex);
        clientApp->AssignStreams(demand::TaskStream(nodeIndex));
        clientApp->SetObservationWindow(generationStart,
                                        Seconds(taskGenStop), Seconds(simulationTime));
        if (cfrAdapter) cfrAdapter->BindClient(nodeIndex, clientApp);
        if (mognnAdapter) mognnAdapter->BindClient(node, clientApp);
        taskObserver->Attach(clientApp);
        if (taskWireObserver) taskWireObserver->Attach(clientApp);
        if (moDiagnosticObserver) moDiagnosticObserver->AttachTask(clientApp);

        if (!csvOnly) taskInputs << "{\"demand_schema\":" << TaskObservationWriter::Quote(demand::SCHEMA)
          << ",\"node_index\":" << nodeIndex << ",\"source_node\":" << node->GetId()
          << ",\"demand_stream\":" << demand::TaskStream(nodeIndex)
          << ",\"selection_stream\":" << demand::TaskStream(nodeIndex) + 1
          << ",\"start_ns\":" << generationStart.GetNanoSeconds()
          << ",\"stop_ns\":" << Seconds(taskGenStop).GetNanoSeconds()
          << ",\"interval_ns\":" << Seconds(nodeConfig.taskGenerationInterval).GetNanoSeconds()
          << ",\"workload_min_gflop\":" << nodeConfig.minTaskWorkload
          << ",\"workload_max_gflop\":" << nodeConfig.maxTaskWorkload
          << ",\"payload_min_bytes\":" << nodeConfig.minTaskDataSize
          << ",\"payload_max_bytes\":" << nodeConfig.maxTaskDataSize
          << ",\"deadline_min_ns\":" << Seconds(nodeConfig.minTaskDeadline).GetNanoSeconds()
          << ",\"deadline_max_ns\":" << Seconds(nodeConfig.maxTaskDeadline).GetNanoSeconds() << "}\n";

        node->AddApplication(clientApp);
        clientApp->SetStartTime(generationStart);
        clientApp->SetStopTime(Seconds(simulationTime) + TimeStep(1));
      }

      // Install the server application. Every processTasks node listens on
      // port 9999 so any high-capacity neighbor can receive a task selected
      // dynamically by the client.
      if (nodeConfig.processTasks && !cfrAdapter) {
        Ptr<cafbcr::ComputeTaskServerApp> serverApp =
            CreateObject<cafbcr::ComputeTaskServerApp>();

        constexpr uint16_t kComputeTaskPort = 9999;
        serverApp->SetLocalPort(kComputeTaskPort);
        serverApp->SetComputeCapacity(nodeConfig.computeCapacity);
        serverApp->SetQueueCapacity(nodeConfig.taskQueueCapacity);
        serverApp->SetSynchronizeQueueLoad(syncQueueLoad);
        serverApp->SetObservationStop(Seconds(simulationTime));
        serverApp->SetUseTaskHeader(taskWire == "header-v1");
        taskObserver->Attach(serverApp);
        if (taskWireObserver) taskWireObserver->Attach(serverApp);
        if (moDiagnosticObserver) moDiagnosticObserver->AttachTask(serverApp);
        if (queueObserver) queueObserver->Attach(serverApp);

        node->AddApplication(serverApp);
        serverApp->SetStartTime(Seconds(0.0));
        serverApp->SetStopTime(Seconds(simulationTime) + TimeStep(1));
      }
    }
  }
  if (!csvOnly) {
    taskInputs.close();
    NS_ABORT_MSG_IF(!taskInputs, "Cannot write task demand configuration");
  }

  // 第三步：安装UDP测试流（基准测试）
  Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable>();
  uv->SetStream(demand::UDP_STREAM);
  auto pairs = demand::UdpPairs(allNodes.GetN(), numTestFlows, uv);
  std::ofstream udpInputs;
  if (!csvOnly) udpInputs.open(std::filesystem::path(taskOutputDir) / "udp-flows.jsonl");
  NS_ABORT_MSG_IF(!csvOnly && !udpInputs, "Cannot open UDP flow input output");
  const bool fixedUdp = udpDemand == "fixed-attempts";
  std::ofstream udpPlan;
  if (fixedUdp && !csvOnly) {
    udpPlan.open(std::filesystem::path(taskOutputDir) / "udp-demand.csv");
    NS_ABORT_MSG_IF(!udpPlan, "Cannot open fixed UDP demand output");
    udpPlan << "schema,flow_index,attempt_index,source_node,destination_node,source_ip,destination_ip,"
               "port,generation_ns,packet_size_bytes,useful_payload_bytes\n";
  }

  for (uint32_t i = 0; i < pairs.size(); ++i) {
    auto [src, dst] = pairs[i];
    uint16_t port = demand::UdpPort(i);
    Ptr<Node> srcNode = allNodes.Get(src);
    Ptr<Node> dstNode = allNodes.Get(dst);

    // 记录TestFlow信息
    TestFlow flow;
    flow.srcNode = src;
    flow.dstNode = dst;
    flow.port = port;
    flow.srcAddr = interfaces.GetAddress(src);
    flow.dstAddr = interfaces.GetAddress(dst);
    m_testFlows.push_back(flow);

    // Preserve the old nominal budget, start, interval and size. In the new mode,
    // stop independently of socket success; retain the old mode for paired checks.
    uint32_t maxPackets = static_cast<uint32_t>(std::max(20.0, std::floor(simulationTime - 50.0)));
    Time start = Seconds(30) + NanoSeconds(demand::NODE_PHASE_NS * i);
    Time latestStop = Seconds(std::max(0.0, simulationTime - 2.0));
    auto plan = udp_demand::FixedPlan(start, latestStop, Seconds(0.5), maxPackets);
    if (udpStart >= 0) {
      start = udp_demand::WindowStart(Seconds(udpStart), Seconds(.5), i, numTestFlows);
      latestStop = Seconds(udpStop);
      plan = udp_demand::WindowPlan(start, latestStop, Seconds(.5));
      maxPackets = static_cast<uint32_t>(plan.attempts);
    }
    Time stop = fixedUdp ? plan.stop : latestStop;
    uint32_t successfulCap = fixedUdp ? 0 : maxPackets;
    bool active = start < stop;
    if (!csvOnly) {
    udpInputs << "{\"demand_schema\":" << TaskObservationWriter::Quote(demand::SCHEMA)
      << ",\"flow_index\":" << i << ",\"source_node\":" << src << ",\"destination_node\":" << dst
      << ",\"port\":" << port << ",\"stream\":" << demand::UDP_STREAM
      << ",\"start_ns\":" << start.GetNanoSeconds() << ",\"stop_ns\":" << stop.GetNanoSeconds()
      << ",\"server_start_ns\":5000000000,\"server_stop_ns\":" << Seconds(simulationTime).GetNanoSeconds()
      << ",\"interval_ns\":500000000,\"packet_size_bytes\":512,\"max_successful_sends\":";
    if (fixedUdp) {
      udpInputs << "null,\"udp_client_max_packets\":0,\"udp_demand_schema\":"
        << TaskObservationWriter::Quote(udp_demand::SCHEMA)
        << ",\"planned_attempts\":" << plan.attempts << ",\"requested_attempts\":" << maxPackets
        << ",\"latest_stop_ns\":" << latestStop.GetNanoSeconds();
      for (uint64_t attempt = 0; attempt < plan.attempts; ++attempt) {
        udpPlan << udp_demand::SCHEMA << ',' << i << ',' << attempt << ',' << src << ',' << dst << ','
                << flow.srcAddr << ',' << flow.dstAddr << ',' << port << ','
                << start.GetNanoSeconds() + static_cast<int64_t>(attempt) * 500000000LL << ",512,500\n";
      }
    } else {
      udpInputs << maxPackets;
    }
    udpInputs << ",\"active\":" << (active ? "true" : "false") << "}\n";
    }
    udpObserver->AddFlow(i, src, dst, flow.srcAddr, flow.dstAddr, port,
                         start, stop, Seconds(0.5), 512, successfulCap, plan.attempts);
    if (!active) continue;

    // 安装UdpServer（接收端）
    UdpServerHelper server(port);
    ApplicationContainer serverApp = server.Install(dstNode);
    // 提前启动Server，给充足时间准备
    serverApp.Start(Seconds(5.0));
    serverApp.Stop(Seconds(simulationTime));

    // 安装UdpClient（发送端）
    UdpClientHelper client(flow.dstAddr, port);
    // 为了让路由表充分建立，延后UDP流启动时间
    // 对于大规模网络(200+节点)，需要足够的收敛时间
    client.SetAttribute("MaxPackets", UintegerValue(successfulCap));
    client.SetAttribute("Interval", TimeValue(Seconds(0.5)));
    client.SetAttribute("PacketSize", UintegerValue(512));
    ApplicationContainer clientApp = client.Install(srcNode);
    // 延后启动到30s，确保路由表已建立
    clientApp.Start(start);
    clientApp.Stop(stop);
    udpObserver->Attach(i, DynamicCast<UdpClient>(clientApp.Get(0)),
                        DynamicCast<UdpServer>(serverApp.Get(0)));
    if (runDiagnostics) runDiagnostics->AttachUdp(DynamicCast<UdpClient>(clientApp.Get(0)));
    if (moDiagnosticObserver) moDiagnosticObserver->AttachUdp(DynamicCast<UdpClient>(clientApp.Get(0)));
    if (networkObserver) networkObserver->AttachUdp(i, src, dst, flow.srcAddr, flow.dstAddr, port,
                              DynamicCast<UdpClient>(clientApp.Get(0)),
                              DynamicCast<UdpServer>(serverApp.Get(0)));
  }
  if (!csvOnly) {
  udpInputs.close();
  NS_ABORT_MSG_IF(!udpInputs, "Cannot write UDP flow inputs");
  if (fixedUdp) {
    udpPlan.close();
    NS_ABORT_MSG_IF(!udpPlan, "Cannot write fixed UDP demand");
  }
  }
}

// ==================== ConfigureStatisticsCollection ====================
// 职责：配置定期统计收集任务，在仿真运行时定期执行
void CafbcrSimulation::ConfigureStatisticsCollection()
{
  // 从仿真开始后10秒开始，每隔statisticsCollectionInterval秒收集一次统计
  for (double t = 10.0; t < simulationTime; t += statisticsCollectionInterval) {
    Simulator::Schedule(Seconds(t), &CafbcrSimulation::DoCollectStatistics, this);

    if (m_metrics) {
      // 链路寿命统计：必须周期性采样。单次末尾调用时所有链路 lastSeenTime=now，
      // 永远不会触发 (now - lastSeenTime) > 3s 的断开判定，m_linkLifetimes 会始终为空。
      Simulator::Schedule(Seconds(t), &PerformanceMetrics::CollectLinkMetrics, m_metrics);

      // 对于分簇协议，还需要定期收集分簇性能指标
      if (m_metrics->IsClusteringAlgorithm(protocol)) {
        Simulator::Schedule(Seconds(t), &PerformanceMetrics::CollectClusteringMetrics, m_metrics);
      }
    }
  }
}

// ==================== DoCollectStatistics ====================
// 职责：实际执行统计收集（在仿真运行时定期被调用）
void CafbcrSimulation::DoCollectStatistics()
{
  if (csvOnly && m_metrics && m_metrics->IsClusteringAlgorithm(protocol))
    csvResults.SampleClustering(allNodes);
  totalTasksGenerated = 0;
  totalTasksCompleted = 0;
  uint32_t nodeIndex = 0;
  const auto& nodeConfigs = config->GetNodeConfigs();

  for (auto it = allNodes.Begin(); it != allNodes.End(); ++it, ++nodeIndex) {
    Ptr<Node> node = *it;
    const UavTypeConfig& nodeConfig = nodeConfigs[nodeIndex];

    // 遍历节点的所有应用来查找ComputeTaskClientApp和ComputeTaskServerApp
    if (nodeConfig.generateTasks) {
      for (uint32_t appIndex = 0; appIndex < node->GetNApplications(); appIndex++) {
        Ptr<cafbcr::ComputeTaskClientApp> clientApp =
            DynamicCast<cafbcr::ComputeTaskClientApp>(node->GetApplication(appIndex));
        if (clientApp) {
          NodeStats& stats = nodeStats[node->GetId()];
          stats.tasksSent = clientApp->GetTasksGenerated();
          stats.tasksCompleted = clientApp->GetTasksCompleted();
          stats.deadlineMissRate = clientApp->GetDeadlineMissRate();

          totalTasksGenerated += stats.tasksSent;
          totalTasksCompleted += stats.tasksCompleted;
          break;
        }
      }
    }

    if (nodeConfig.processTasks) {
      for (uint32_t appIndex = 0; appIndex < node->GetNApplications(); appIndex++) {
        Ptr<cafbcr::ComputeTaskServerApp> serverApp =
            DynamicCast<cafbcr::ComputeTaskServerApp>(node->GetApplication(appIndex));
        if (serverApp) {
          NodeStats& stats = nodeStats[node->GetId()];
          stats.tasksReceived = serverApp->GetTasksReceived();
          stats.tasksProcessed = serverApp->GetTasksProcessed();
          stats.tasksDropped = serverApp->GetTasksDropped();
          stats.avgProcessingDelay = serverApp->GetAverageProcessingDelay();
          break;
        }
      }
    }
  }

  // 定期打印当前统计信息到日志（可选，用于仿真过程中的实时监测）
  if (verbose) {
    std::cout << "[" << Simulator::Now().GetSeconds() << "s] 统计信息: "
              << "总任务=" << totalTasksGenerated
              << " 已完成=" << totalTasksCompleted << std::endl;
  }
}

// ==================== Run ====================
void CafbcrSimulation::Run()
{
  const auto setupStarted = csvRunDiagnostics ? RunDiagnostics::Now() : RunDiagnostics::Mark{};
  const auto writeRenewalTraceMetadata = [this](bool complete) {
    if (caRenewalTraceDir.empty()) return;
    std::ofstream out(std::filesystem::path(caRenewalTraceDir) / "trace-run.json");
    NS_ABORT_MSG_IF(!out, "Cannot open renewal sidecar metadata");
    out << "{\"schema\":\"cafbcr-renewal-sidecar-run-v1\",\"complete\":" << (complete ? "true" : "false")
        << ",\"protocol\":" << TaskObservationWriter::Quote(protocol)
        << ",\"build_label\":" << TaskObservationWriter::Quote(csvBuildId)
        << ",\"build_identity_status\":\"caller_supplied_label\""
        << ",\"seed\":" << seed << ",\"rng_run\":" << RngSeedManager::GetRun()
        << ",\"capture_start_ns\":0,\"capture_end_ns\":" << Seconds(simulationTime).GetNanoSeconds()
        << ",\"csv_run_diagnostics\":" << (csvRunDiagnostics ? "true" : "false")
        << ",\"results_csv\":" << TaskObservationWriter::Quote(resultsCsv)
        << ",\"join_schema\":" << TaskObservationWriter::Quote(JoinObservationWriter::SCHEMA)
        << ",\"renewal_schema\":" << TaskObservationWriter::Quote(JoinObservationWriter::RENEWAL_SCHEMA)
        << ",\"scenario\":" << (scenarioContract ? scenarioContract->Preview() : "null")
        << ",\"arguments\":" << commandArguments << "}\n";
    out.close();
    NS_ABORT_MSG_IF(!out, "Cannot flush renewal sidecar metadata");
  };
  // 初始化随机种子
  SeedManager::SetSeed(seed);
  if (!csvOnly && taskOutputDir.empty()) {
    auto stamp = std::chrono::system_clock::now().time_since_epoch().count();
    taskOutputDir = (std::filesystem::path(outputDir) / TaskObservationWriter::SCHEMA /
      (protocol + "-seed" + std::to_string(seed) + "-run" + std::to_string(RngSeedManager::GetRun()) +
       "-" + std::to_string(stamp))).string();
  }
  std::ostringstream metadata;
  metadata << std::setprecision(17) << "{\"schema\":" << TaskObservationWriter::Quote(TaskObservationWriter::SCHEMA)
    << ",\"routing_variant\":" << TaskObservationWriter::Quote(
         protocol == "cafbcr" ? caVariant :
         protocol == "qrcf" ? qrcfPreset : "native")
    << ",\"protocol\":" << TaskObservationWriter::Quote(protocol)
    << ",\"cfr_eng_profile\":" << (protocol == "cfr-eng"
      ? "{\"identity\":\"cfr-eng-local-v1\",\"pddqn\":false,\"compute\":" + TaskObservationWriter::Quote(cfrEngCompute) +
        ",\"deadline_policy\":\"evaluation_only\",\"terminal_assignment\":\"common_fixed\",\"task_wire\":\"cfr-task-header\",\"flops_per_cycle\":16,\"result_ratio\":0.05}"
      : "null")
    << ",\"entrypoint_identity\":\"cafbcr-csv-results-entry-v1\""
    << ",\"routing_library_identity\":\"cafbcr-initial-topk-v1\""
    << ",\"abc_cost_observation\":" << (observeAbcCost ? "true" : "false")
    << ",\"initial_ch_selector\":" << (protocol == "cafbcr" ? TaskObservationWriter::Quote(caInitialSelector) : "null")
    << ",\"initial_selection_observation\":" << (observeInitialSelection ? "true" : "false")
    << ",\"phy_header_boundary_guard\":" << (phyHeaderBoundaryGuard ? "true" : "false")
    << ",\"packet_loss_observation\":{\"schema\":" << TaskObservationWriter::Quote(PacketLossObservationWriter::SCHEMA)
    << ",\"enabled\":" << (observePacketLoss ? "true" : "false")
    << ",\"ca_next_hop_requested\":" << (observeCaNextHop ? "true" : "false")
    << ",\"ca_next_hop_effective\":" << (observePacketLoss && observeCaNextHop && protocol == "cafbcr" ? "true" : "false")
    << ",\"phy_receive_failure_cause\":\"unavailable\",\"protocol_feedback\":false}"
    << ",\"baseline_profile\":" << BaselineProfileJson()
    << ",\"proxy_renewal_observation_schema\":" << (protocol == "cafbcr" ?
        TaskObservationWriter::Quote(JoinObservationWriter::RENEWAL_SCHEMA) : "null")
    << ",\"channel\":" << channelConfig.Json()
    << ",\"comparison_schema\":" << TaskObservationWriter::Quote(comparison::SCHEMA)
    << ",\"routing_configuration\":" << TaskObservationWriter::Quote(
         protocol == "cafbcr" ? (caLoopPolicy == "off" ? "cafbcr-hardfix" : "cafbcr-hardfix-" + caLoopPolicy) :
         protocol == "qrcf" ? "qrcf-" + qrcfPreset : protocol + "-native")
    << ",\"ca_loop_policy\":{\"schema\":"
    << TaskObservationWriter::Quote(caLoopPolicy == "alternate-revisit" ? RevisitRecovery::SCHEMA : ForwardingHistory::SCHEMA)
    << ",\"requested\":"
    << TaskObservationWriter::Quote(caLoopPolicy)
    << ",\"enabled\":" << (protocol == "cafbcr" && caLoopPolicy != "off" ? "true" : "false")
    << ",\"scope\":" << TaskObservationWriter::Quote(caLoopPolicy == "alternate-revisit" ?
        "CA_only_local_alternatives_not_face_routing" : "CA_only_early_termination_not_geographic_recovery")
    << ",\"default\":\"off\"}"
    << ",\"ca_mechanism_controls\":";
  if (protocol == "cafbcr") {
    metadata << "{\"schema\":\"cafbcr-asym-controls-v1\",\"disable_asym\":"
      << (disableAsym ? "true" : "false")
      << ",\"proxy_join_requested\":" << (caProxyJoin ? "true" : "false")
      << ",\"proxy_renewal_requested\":" << (caProxyRenewal ? "true" : "false")
      << ",\"range_gating_requested\":" << (caRangeGating ? "true" : "false")
      << ",\"proxy_join_effective\":" << (!disableAsym && caProxyJoin ? "true" : "false")
      << ",\"proxy_renewal_effective\":" << (!disableAsym && caProxyJoin && caProxyRenewal ? "true" : "false")
      << ",\"renewal_policy_requested\":" << TaskObservationWriter::Quote(caProxyRenewalPolicy)
      << ",\"renewal_policy_effective\":" << TaskObservationWriter::Quote(
          !disableAsym && caProxyJoin && caProxyRenewal ? caProxyRenewalPolicy : "disabled")
      << ",\"renewal_window_ms\":" << caProxyRenewalWindowMs
      << ",\"renewal_slots\":32,\"renewal_feedback_header_bytes\":14"
      << ",\"renewal_feedback_batch_window_ms\":" << caProxyAckBatchMs
      << ",\"renewal_feedback_max_records\":8"
      << ",\"renewal_feedback_batching_effective\":"
      << (!disableAsym && caProxyJoin && caProxyRenewal && caProxyRenewalPolicy=="ack-batch" ? "true" : "false")
      ;
    if (caProxyRenewalPolicy=="direct-feedback")
      metadata << ",\"direct_maintenance_feedback\":{\"profile\":\"ch-live-direct-cm-v1\",\"effective\":"
        << (!disableAsym && caProxyJoin && caProxyRenewal ? "true" : "false")
        << ",\"extension_base_bytes\":10,\"record_bytes\":10,\"max_records\":8"
        << ",\"window\":\"min_CH_and_source_half_lease_from_CM_source_time\""
        << ",\"fallback\":\"immediate_HELLO_at_coverage_expiry\"}";
    if (caDirectCmPolicy != "legacy")
      metadata << ",\"direct_cm_policy\":" << TaskObservationWriter::Quote(caDirectCmPolicy);
    if (caControlUnicastPolicy != "legacy")
      metadata << ",\"control_unicast_policy\":" << TaskObservationWriter::Quote(caControlUnicastPolicy);
    if (caProxyRelayPolicy != "legacy")
      metadata << ",\"proxy_relay_policy\":" << TaskObservationWriter::Quote(caProxyRelayPolicy)
        << ",\"relay_coordination_effective\":true";
    metadata << ",\"range_gating_effective\":" << (!disableAsym && caRangeGating ? "true" : "false")
      << ",\"proxy_disabled_join\":\"ordinary_join_req_resp\""
      << ",\"proxy_eligibility\":\"fresh_neighbor_and_configured_range_independent_of_data_gate\""
      << ",\"range_scope\":\"data_plane_only\""
      << ",\"grant_policy\":\"pending_admission_until_cm_or_expiry_intended_idle_source_v1\"}";
  } else {
    metadata << "null";
  }
  if (caLinkProbes) metadata<<",\"link_measurement\":{\"schema\":\"cafbcr-bidirectional-gate-v1\",\"probe_enabled\":true,\"candidate_gate\":"
    <<TaskObservationWriter::Quote(caLinkGate)<<",\"interval_s\":"<<caLinkProbeInterval<<",\"timeout_s\":"<<caLinkProbeTimeout
    <<",\"validity_s_from_send\":"<<caLinkValidity<<",\"header_bytes\":20,\"ipv4_bytes_per_message\":49,\"ttl\":1,\"observer_enabled\":"<<(observeLinkProbes?"true":"false")
    <<",\"score_policy\":\"unchanged\",\"legacy_local_control_and_proxy_rule\":\"unchanged\"}";
  metadata
    << ",\"task_selection_mode\":" << TaskObservationWriter::Quote(taskSelection)
    << ",\"selection_information\":" << TaskObservationWriter::Quote(taskSelection == "fixed"
         ? "offline_server_membership_and_type_no_online_state"
         : protocol == "cafbcr" ? "received_compute_advertisements_with_fallback"
         : "native_neighbor_fields_if_available_with_fallback")
    << ",\"comparison_scope\":" << TaskObservationWriter::Quote(protocol == "cfr-eng" ? "engineering_joint_compute_and_routing_common_terminal" : taskSelection == "fixed"
         ? "routing_with_common_fixed_assignment" : "native_system_information_differs")
    << ",\"server_assignment_schema\":" << TaskObservationWriter::Quote(comparison::ASSIGNMENT_SCHEMA)
    << ",\"routing_information_assumptions\":{\"common_local_information_proven\":false,\"cost_alignment\":\"pending\"}"
    << ",\"hello_method\":" << TaskObservationWriter::Quote(helloMethod)
    << ",\"fitness_mode\":" << TaskObservationWriter::Quote(fanetMode)
    << ",\"ca_fitness_weights\":";
  if (protocol == "cafbcr") {
    double wLHTP, wEnergy, wDegree, wDistance, wCompute;
    config->GetFitnessWeights(wLHTP, wEnergy, wDegree, wDistance, wCompute);
    metadata << "{\"schema\":\"cafbcr-fitness-weights-v1\",\"requested_wC\":" << sweepWc
      << ",\"order\":[\"LHTP\",\"Energy\",\"Degree\",\"Distance\",\"Compute\"]"
      << ",\"configured\":[" << wLHTP << ',' << wEnergy << ',' << wDegree << ','
      << wDistance << ',' << wCompute << ']'
      << ",\"scope\":\"initial_selector_and_runtime_node_fitness\""
      << ",\"other_weight_policy\":\"equal_share_of_one_minus_compute\"}";
  } else {
    metadata << "null";
  }
  metadata << ",\"seed\":" << seed << ",\"run\":" << RngSeedManager::GetRun()
    << ",\"nodes\":" << config->GetNodeCount()
    << ",\"compute_tasks_enabled\":" << (enableComputeTask ? "true" : "false")
    << ",\"generation_start_ns\":" << Seconds(taskGenStart).GetNanoSeconds()
    << ",\"generation_stop_ns\":" << Seconds(taskGenStop).GetNanoSeconds()
    << ",\"observation_stop_ns\":" << Seconds(simulationTime).GetNanoSeconds()
    << ",\"maximum_deadline_ns\":" << Seconds(maximumTaskDeadline).GetNanoSeconds()
    << ",\"deadline_coverage\":" << (taskDeadlineCoverage ? "true" : "false")
    << ",\"require_deadline_coverage\":" << (requireTaskDeadlineCoverage ? "true" : "false")
    << ",\"cleanup_ticks\":2,\"tick_ns\":" << TimeStep(1).GetNanoSeconds()
    << ",\"demand_schema\":" << TaskObservationWriter::Quote(demand::SCHEMA)
    << ",\"task_start_mode\":" << TaskObservationWriter::Quote(taskStartMode)
    << ",\"task_start_spread_ns\":" << (taskStartMode == "bounded-1s" ? "1000000000" : "null")
    << ",\"node_phase_ns\":" << (taskStartMode == "legacy-index" ? std::to_string(demand::NODE_PHASE_NS) : "null")
    << ",\"task_stream_base\":" << demand::TASK_STREAM_BASE << ",\"task_stream_stride\":2"
    << ",\"task_attribute_model\":\"uniform_workload_truncated_uniform_bytes_uniform_deadline\""
    << ",\"task_wire_mode\":" << TaskObservationWriter::Quote(protocol == "cfr-eng" ? "cfr-task-header-local-v1" : taskWire)
    << ",\"task_wire_schema\":" << (protocol == "cfr-eng" ? TaskObservationWriter::Quote("cfr-task-header-local-v1") : taskWire == "header-v1" ? TaskObservationWriter::Quote(ComputeTaskHeader::SCHEMA) : "null")
    << ",\"task_header_bytes\":" << (protocol == "cfr-eng" ? "null" : std::to_string(taskWire == "header-v1" ? ComputeTaskHeader::SIZE : 0))
    << ",\"task_response_payload_bytes\":8,\"task_response_semantics\":\"placeholder_completion_notification\""
    << ",\"task_routing_tag\":" << TaskObservationWriter::Quote(protocol == "cfr-eng" ? "cfr_path_tag_no_cafbcr_tag" : "retained_cache_header_mode_apps_decode_bytes_only")
    << ",\"udp_stream\":" << demand::UDP_STREAM << ",\"udp_flow_count\":" << numTestFlows
    << ",\"udp_pair_sampler\":\"integer_partial_fisher_yates\""
    << ",\"udp_packet_accounting\":" << TaskObservationWriter::Quote(udpDemand == "fixed-attempts"
         ? "fixed_attempts_v1" : "legacy_success_count_pending_revision")
    << ",\"udp_demand_mode\":" << TaskObservationWriter::Quote(udpDemand)
    << ",\"udp_window_mode\":" << TaskObservationWriter::Quote(udpStart >= 0 ? "explicit-window" : "legacy-time")
    << ",\"udp_start_s\":" << udpStart << ",\"udp_stop_s\":" << udpStop
    << ",\"csv_configuration\":" << (csvConfiguration ? "true" : "false")
    << ",\"udp_observation_schema\":" << TaskObservationWriter::Quote(UdpObservationWriter::SCHEMA)
    << ",\"network_observation_schema\":" << TaskObservationWriter::Quote(NetworkObservationWriter::SCHEMA)
    << ",\"phy_completion_observation\":{\"enabled\":" << (observePhyCompletion ? "true" : "false")
    << ",\"schema\":" << TaskObservationWriter::Quote(PhyCompletionObserver::SCHEMA) << "}"
    << ",\"udp_demand_schema\":" << (udpDemand == "fixed-attempts"
         ? TaskObservationWriter::Quote(udp_demand::SCHEMA) : "null")
    << ",\"queue_load_schema\":" << TaskObservationWriter::Quote(QueueLoadWriter::SCHEMA)
    << ",\"queue_load_sync\":" << (protocol != "cfr-eng" && syncQueueLoad ? "true" : "false")
    << ",\"queue_load_definition\":\"waiting_tasks_divided_by_waiting_capacity\""
    << ",\"neighbor_compute_schema\":" << TaskObservationWriter::Quote(NeighborComputeWriter::SCHEMA)
    << ",\"neighbor_compute_tracking\":" << (protocol == "cafbcr" ? "true" : "false")
    << ",\"join_event_schema\":" << TaskObservationWriter::Quote(JoinObservationWriter::SCHEMA)
    << ",\"join_observation\":" << (protocol == "cafbcr" && observeJoin ? "true" : "false")
    << ",\"compute_freshness_policy\":\"ca_hello_receive_age_lt_twice_receiver_base_interval\""
    << ",\"visible_state_age_status\":\"receive_age_at_selection_or_null_with_status\""
    << ",\"build_identity_status\":\"not_bound_by_runner\",\"formal_eligible\":false"
    << ",\"arguments\":" << commandArguments << '}';
  taskObserver = std::make_unique<TaskObservationWriter>(taskOutputDir, Seconds(simulationTime), metadata.str(), !csvOnly);
  if (observePacketLoss)
    packetLossObserver = std::make_unique<PacketLossObservationWriter>(taskOutputDir, Seconds(simulationTime),
                                                                    observeCaNextHop && protocol == "cafbcr");
  if (!csvOnly) {
    queueObserver = std::make_unique<QueueLoadWriter>(taskOutputDir, Seconds(simulationTime));
    neighborObserver = std::make_unique<NeighborComputeWriter>(taskOutputDir, Seconds(simulationTime));
    if (protocol != "cfr-eng") taskWireObserver = std::make_unique<TaskWireWriter>(taskOutputDir, Seconds(simulationTime), taskWire == "header-v1");
  }
  udpObserver = std::make_unique<UdpObservationWriter>(taskOutputDir, Seconds(30), Seconds(simulationTime),
                                                     udpDemand == "fixed-attempts", !csvOnly);
  if (csvFailureSummary) {
    failureSummary = std::make_unique<FailureSummary>(Seconds(simulationTime));
    failureSummary->AttachUdp(*udpObserver);
  }
  const uint16_t controlPort = protocol == "aodv" ? 654 :
                               protocol == "olsr" ? 698 :
                               protocol == "qrcf" ? 17930 :
                               protocol == "icra" ? 888 : protocol == "cfr-eng" ? 26910 : UsesMo() ? 26920 : 889;
  if (controlMeter == "l3") controlCostMeter = std::make_unique<ControlCostMeter>(controlPort);
  if (csvRunDiagnostics) runDiagnostics = std::make_unique<RunDiagnostics>(Seconds(simulationTime),
      protocol=="cafbcr", protocol=="cafbcr" ? caLoopPolicy : "N/A",
      protocol=="cafbcr" ? caProxyRenewalPolicy : "N/A", enableComputeTask, setupStarted);
  if (!caCmSummary.empty()) cmSummary = std::make_unique<CmSummary>(Seconds(simulationTime),
        Seconds(taskGenStart), Seconds(taskGenStop), caDirectCmPolicy=="feedback-covered");
  if (!csvOnly) networkObserver = std::make_unique<NetworkObservationWriter>(taskOutputDir, Seconds(30),
                                  Seconds(simulationTime), controlPort, protocol == "cafbcr", observePhyCompletion);
  if (protocol == "cafbcr" && !csvOnly)
    loopObserver = std::make_unique<LoopObservationWriter>(taskOutputDir, Seconds(simulationTime),
                                                       caLoopPolicy == "terminate-revisit");
  if (protocol == "cafbcr" && !csvOnly)
    recoveryObserver = std::make_unique<RecoveryObservationWriter>(taskOutputDir, Seconds(simulationTime),
                                                               caLoopPolicy == "alternate-revisit");
  if (protocol == "cafbcr" && (observeJoin || !caRenewalTraceDir.empty())) {
    const auto directory = caRenewalTraceDir.empty() ? taskOutputDir : caRenewalTraceDir;
    if (!caRenewalTraceDir.empty()) {
      // Atomically reserve the final directory again after validation.
      std::filesystem::create_directories(std::filesystem::path(directory).parent_path());
      NS_ABORT_MSG_IF(!std::filesystem::create_directory(directory), "Renewal sidecar directory already exists");
      writeRenewalTraceMetadata(false);
    }
    joinObserver = std::make_unique<JoinObservationWriter>(directory, Seconds(simulationTime));
  }
  if (protocol=="cafbcr" && caLinkProbes && observeLinkProbes)
    linkObserver=std::make_unique<LinkObservationWriter>(taskOutputDir,Seconds(simulationTime));
  std::cout << (csvOnly ? "Result CSV: " + resultsCsv : "Task observation output: " + taskOutputDir) << std::endl;

  // 注：曾尝试切换到 ns3::HeapScheduler 想加快 Destroy 时的事件释放，
  // 但在本仿真规模下触发了 default-simulator-impl.cc:136 的事件时序断言
  // (next.key.m_ts >= m_currentTs)，疑似该版本 HeapScheduler 实现有缺陷。
  // 保持默认 MapScheduler 即可。

  // 按照正确的顺序执行
  CreateNodes();           // 创建节点 + 移动模型
  CreateDevices();         // 创建网络设备
  InstallEnergyModel();    // 装能量源（在 internet 前）
  InstallInternetStack();  // 安装协议栈和路由协议（其中会同步 ClusterNode 能量）
  for (uint32_t i = 0; i < allNodes.GetN(); ++i) {
    if (networkObserver) {
      networkObserver->AttachIp(i, allNodes.Get(i));
      networkObserver->AttachWifi(i, allNodes.Get(i));
    }
    if (packetLossObserver) packetLossObserver->Attach(i, allNodes.Get(i));
    if (controlCostMeter) controlCostMeter->Attach(allNodes.Get(i));
    if (failureSummary) failureSummary->AttachNode(allNodes.Get(i));
    if (runDiagnostics) runDiagnostics->AttachNode(allNodes.Get(i), controlPort);
    if (cmSummary) cmSummary->Attach(allNodes.Get(i));
  }

  // 初始化性能指标收集（必须在应用安装前初始化）
  m_metrics = CreateObject<ns3::PerformanceMetrics>();
  m_metrics->SetNodes(allNodes);

  // 安装应用层（计算任务 + UDP测试流 + FlowMonitor）
  InstallApplications();

  // 设置FlowMonitor到PerformanceMetrics
  m_metrics->SetFlowMonitor(monitor, DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier()));

  // 配置统计收集任务（在Simulator::Run()之前配置）
  ConfigureStatisticsCollection();

  std::cout << "[5] 运行仿真..." << std::endl;

  // Process all events at T_obs, close task applications one tick later, then stop.
  // Applications and observer reject timestamps after T_obs.
  Simulator::Stop(Seconds(simulationTime) + TimeStep(2));
  if (runDiagnostics) runDiagnostics->BeginRun();
  Simulator::Run();
  if (runDiagnostics) runDiagnostics->EndRun();
  taskObserver->Finalize();
  if (moDiagnosticObserver) moDiagnosticObserver->Finalize(csvBuildId, seed, RngSeedManager::GetRun(),
      scenarioContract ? scenarioContract->Preview() : "{}");
  if (mognnAdapter) {
    const auto summary = mognnAdapter->SummaryJson(*taskObserver, csvBuildId, seed,
                                                RngSeedManager::GetRun(), Seconds(simulationTime));
    std::cout << "[MOGNN tasks] " << summary << std::endl;
    if (!moSummary.empty()) {
      const std::filesystem::path path(moSummary);
      if (!path.parent_path().empty()) std::filesystem::create_directories(path.parent_path());
      std::ofstream out(path); out << summary << '\n'; out.close();
      NS_ABORT_MSG_IF(!out, "Cannot save MOGNN task partition");
    }
  }
  if (queueObserver) queueObserver->Finalize();
  if (neighborObserver) neighborObserver->Finalize();
  if (joinObserver) joinObserver->Finalize();
  if (linkObserver) linkObserver->Finalize();

  // 仿真结束后收集最终统计（而不是在仿真中途）
  if (csvOnly) udpObserver->Finalize();
  if (failureSummary) failureSummary->Finalize(*taskObserver, *udpObserver);
  if (runDiagnostics) runDiagnostics->Finalize(*taskObserver, *udpObserver);
  FinalizeStatistics();
  if (cmSummary) cmSummary->Save(caCmSummary, csvBuildId, seed, RngSeedManager::GetRun(),
                                scenarioContract->Preview(), runDiagnostics->ControlCmIpTx());
  if (!csvOnly) udpObserver->Finalize();
  if (taskWireObserver) taskWireObserver->Finalize();
  if (networkObserver) networkObserver->Finalize();
  if (loopObserver) loopObserver->Finalize();
  if (recoveryObserver) recoveryObserver->Finalize();
  if (!csvOnly) wireless::FinishChannel(channelConfig, shadowModel, taskOutputDir);
  if (!csvOnly) SaveBaselineRuntime();
  if (packetLossObserver) packetLossObserver->Finalize();
  if (!csvOnly) SavePhyHeaderBoundarySummary();
  SaveAbcCostObservation();
  SaveInitialSelectionObservation();
  // FinalizeStatistics has saved the CSV and JoinObservationWriter has flushed
  // both streams; failures before this point retain complete=false.
  writeRenewalTraceMetadata(true);
  if (caQHelloDiagnostics) {
    QLearningHelloManager::Statistics total;
    uint64_t controllers = 0;
    for (uint32_t i = 0; i < allNodes.GetN(); ++i) {
      auto routing = allNodes.Get(i)->GetObject<cafbcr::RoutingProtocol>();
      if (!routing) continue;
      auto q = routing->GetQLearningHelloManager();
      if (!q) continue;
      ++controllers;
      const auto& s = q->GetStatistics();
      total.decisions += s.decisions; total.feedbackWindows += s.feedbackWindows;
      total.updates += s.updates; total.learnedDecisions += s.learnedDecisions;
      total.receivedHellos += s.receivedHellos; total.routingObservations += s.routingObservations;
      for (unsigned a = 0; a < QLearningHelloManager::NUM_ACTIONS; ++a) total.actions[a] += s.actions[a];
    }
    std::cout << "[CA-QHello] {\"schema\":\"cafbcr-qhello-local-v1\",\"controllers\":" << controllers
      << ",\"decisions\":" << total.decisions << ",\"feedbackWindows\":" << total.feedbackWindows
      << ",\"updates\":" << total.updates << ",\"pendingActions\":" << total.decisions-total.feedbackWindows
      << ",\"learnedDecisions\":" << total.learnedDecisions << ",\"receivedHellos\":" << total.receivedHellos
      << ",\"routingObservations\":" << total.routingObservations << ",\"actions\":[";
    for (unsigned a = 0; a < QLearningHelloManager::NUM_ACTIONS; ++a) std::cout << (a ? "," : "") << total.actions[a];
    std::cout << "]}" << std::endl;
  }

  // Simulator::Destroy() 在大规模仿真下耗时极长（200 节点 / 300s 实测约 30 分钟），
  // 原因是 ns-3 的 Ptr<> 引用计数会在销毁时级联回收整个对象图（节点 + 设备 +
  // 数据包 + 邻居表 + 簇），属于 ns-3 设计的固有开销，无法绕过。
  // 默认走 _Exit(0) 快速退出（结果已经写入 CSV，OS 回收内存）；
  // 仅在 --cleanShutdown=true（如做 valgrind 内存检查）时走完整销毁。
  if (cleanShutdown) {
    Simulator::Destroy();
  } else {
    std::cout.flush();
    std::_Exit(0);
  }
}

void CafbcrSimulation::SaveAbcCostObservation() const
{
  if (!observeAbcCost) return;
  using Record = cafbcrElection::CostRecord;
  static const std::vector<std::pair<const char*, uint64_t Record::*>> fields = {
    {"initialization_calls", &Record::initializationCalls},
    {"initialization_candidate_visits", &Record::initializationCandidateVisits},
    {"initialization_state_visits", &Record::initializationStateVisits},
    {"backup_scan_visits", &Record::backupScanVisits},
    {"selection_rounds", &Record::selectionRounds},
    {"state_save_visits", &Record::stateSaveVisits},
    {"state_restore_visits", &Record::stateRestoreVisits},
    {"init_bee_visits", &Record::initBeeVisits},
    {"init_bee_fitness_calls", &Record::initBeeFitnessCalls},
    {"employed_visits", &Record::employedVisits},
    {"employed_neighbor_visits", &Record::employedNeighborVisits},
    {"employed_no_neighbor_fallbacks", &Record::employedNoNeighborFallbacks},
    {"employed_fitness_calls", &Record::employedFitnessCalls},
    {"onlooker_visits", &Record::onlookerVisits},
    {"onlooker_trials", &Record::onlookerTrials},
    {"onlooker_selected_trials", &Record::onlookerSelectedTrials},
    {"onlooker_fitness_calls", &Record::onlookerFitnessCalls},
    {"probability_calls", &Record::probabilityCalls},
    {"probability_visits", &Record::probabilityVisits},
    {"best_scan_calls", &Record::bestScanCalls},
    {"best_scan_visits", &Record::bestScanVisits},
    {"scout_scan_visits", &Record::scoutScanVisits},
    {"scout_reinitializations", &Record::scoutReinitializations},
    {"cycle_iterations", &Record::cycleIterations},
    {"commit_duplicate_visits", &Record::commitDuplicateVisits},
    {"commit_candidate_visits", &Record::commitCandidateVisits},
    {"committed_heads", &Record::committedHeads},
    {"topk_candidate_visits", &Record::topkCandidateVisits},
    {"topk_comparisons", &Record::topkComparisons}
  };
  std::ofstream csv(std::filesystem::path(taskOutputDir) / "abc-cost.csv");
  std::ofstream summary(std::filesystem::path(taskOutputDir) / "abc-cost-summary.json");
  NS_ABORT_MSG_IF(!csv || !summary, "Cannot save ABC cost observation");
  csv << "node,invocation,selector,outcome,n,k,initialized_before,sim_start_ns,sim_end_ns,host_wall_ns,setup_host_ns,search_host_ns,commit_host_ns,fitness_calls";
  for (const auto& field : fields) csv << ',' << field.first;
  csv << '\n';
  uint64_t requests = 0, completed = 0, skipped = 0, empty = 0, hostNs = 0, fitness = 0;
  uint32_t enabledNodes = 0;
  Record totals;
  // All ordinary output finalizers have finished. Object lookup and file I/O
  // here cannot feed information back into an election or routing decision.
  for (uint32_t i = 0; i < allNodes.GetN(); ++i) {
    auto election = allNodes.Get(i)->GetObject<cafbcrElection>();
    NS_ABORT_MSG_IF(!election || !election->IsCostObservationEnabled(), "ABC observation not enabled on a CA node");
    ++enabledNodes;
    for (const auto& row : election->GetCostRecords()) {
      NS_ABORT_MSG_IF(row.simulationStartNs != row.simulationEndNs, "Cost observation must not advance simulation time");
      NS_ABORT_MSG_IF(row.hostWallNs != row.setupHostNs + row.searchHostNs + row.commitHostNs, "Host phases do not sum");
      ++requests;
      completed += std::string(row.outcome) == "completed";
      skipped += std::string(row.outcome) == "skipped_existing_backup";
      empty += std::string(row.outcome) == "no_nodes";
      hostNs += row.hostWallNs;
      fitness += row.FitnessCalls();
      csv << allNodes.Get(i)->GetId() << ',' << row.invocation << ',' << row.selector << ',' << row.outcome
          << ',' << row.nodes << ',' << row.requestedHeads << ',' << row.initializedBefore
          << ',' << row.simulationStartNs << ',' << row.simulationEndNs
          << ',' << row.hostWallNs << ',' << row.setupHostNs << ',' << row.searchHostNs
          << ',' << row.commitHostNs << ',' << row.FitnessCalls();
      for (const auto& field : fields) {
        csv << ',' << row.*(field.second);
        totals.*(field.second) += row.*(field.second);
      }
      csv << '\n';
    }
  }
  NS_ABORT_MSG_IF(requests != completed + skipped + empty, "Unknown ABC outcome");
  summary << "{\"schema\":\"cafbcr-initial-election-cost-summary-v2\",\"formal_eligible\":false"
          << ",\"selector\":" << TaskObservationWriter::Quote(caInitialSelector)
          << ",\"enabled\":true,\"measurement_valid\":true,\"scope\":\"StartElection_only\""
          << ",\"nodes_enabled\":" << enabledNodes << ",\"requests\":" << requests
          << ",\"completed\":" << completed << ",\"skipped_existing_backup\":" << skipped
          << ",\"no_nodes\":" << empty << ",\"host_wall_ns\":" << hostNs
          << ",\"fitness_calls\":" << fitness
          << ",\"processing_delay_injected\":false,\"simulated_processing_delay_ns\":null"
          << ",\"cpu_energy_j\":null,\"shared_fitness_internal_neighbor_visits\":null"
          << ",\"global_information_communication_cost\":\"not_modeled\""
          << ",\"routing_fitness_before_StartElection_counted\":false,\"observer_instrumentation_in_host_time\":true"
          << ",\"record_storage_and_final_file_io_in_host_time\":false,\"counters\":{";
  for (size_t i = 0; i < fields.size(); ++i) {
    if (i) summary << ',';
    summary << TaskObservationWriter::Quote(fields[i].first) << ':' << totals.*(fields[i].second);
  }
  summary << "}}\n";
}

void CafbcrSimulation::SaveInitialSelectionObservation() const
{
  if (!observeInitialSelection) return;
  std::ofstream out(std::filesystem::path(taskOutputDir) / "initial-selection.json");
  NS_ABORT_MSG_IF(!out, "Cannot save initial selection observation");
  out << std::setprecision(17);
  auto vector = [&out](const Vector& v) { out << '[' << v.x << ',' << v.y << ',' << v.z << ']'; };
  auto boolean = [](bool value) { return value ? "true" : "false"; };
  out << "{\"schema\":\"cafbcr-initial-selection-v1\",\"formal_eligible\":false"
      << ",\"enabled\":true,\"selector\":" << TaskObservationWriter::Quote(caInitialSelector)
      << ",\"global_candidate_set\":true,\"extra_fitness_calls_for_observation\":0"
      << ",\"fitness_source\":\"first_existing_InitBee_pass\",\"records\":[";
  bool firstRecord = true;
  for (uint32_t i = 0; i < allNodes.GetN(); ++i) {
    auto election = allNodes.Get(i)->GetObject<cafbcrElection>();
    NS_ABORT_MSG_IF(!election || !election->IsInitialSelectionObserved(), "Initial observation not enabled on a CA node");
    for (const auto& record : election->GetInitialSelectionRecords()) {
      if (!firstRecord) out << ',';
      firstRecord = false;
      out << "{\"owner_node\":" << allNodes.Get(i)->GetId()
          << ",\"time_ns\":" << record.simulationNs << ",\"n\":" << record.nodes
          << ",\"k\":" << record.requestedHeads << ",\"weights\":[";
      for (size_t w = 0; w < record.weights.size(); ++w) {
        if (w) out << ',';
        out << record.weights[w];
      }
      out << "],\"candidates\":[";
      bool firstCandidate = true;
      for (const auto& c : record.candidates) {
        if (!firstCandidate) out << ',';
        firstCandidate = false;
        out << "{\"node\":" << c.nodeId << ",\"role\":" << unsigned(c.role)
            << ",\"eligible\":" << boolean(c.eligible) << ",\"selected\":" << boolean(c.selected)
            << ",\"f0\":";
        if (c.scored) out << c.fitness0; else out << "null";
        out << ",\"cached_fitness\":" << c.cachedFitness << ",\"fitness_mode\":"
            << TaskObservationWriter::Quote(ClusterNode::FitnessModeName(c.fitnessMode))
            << ",\"network_scale\":" << c.networkScale << ",\"cached_degree\":" << c.degree
            << ",\"energy_j\":" << c.energy << ",\"origin_energy_j\":" << c.originEnergy
            << ",\"compute_capacity\":" << c.computeCapacity << ",\"load\":" << c.load
            << ",\"max_range_m\":" << c.maxRange << ",\"lhtp_use_max_range\":" << boolean(c.lhtpUseMaxRange)
            << ",\"position\":";
        vector(c.position);
        out << ",\"velocity\":";
        vector(c.velocity);
        out << ",\"neighbors\":[";
        bool firstNeighbor = true;
        for (const auto& neighbor : c.neighbors) {
          if (!firstNeighbor) out << ',';
          firstNeighbor = false;
          const auto& s = neighbor.state;
          out << "{\"key\":" << neighbor.key << ",\"node\":" << s.NodeId
              << ",\"ip\":\"" << s.ipAddress << "\",\"cluster\":" << s.clusterId
              << ",\"role\":" << unsigned(s.Role) << ",\"update_ns\":" << s.UpdateTime.GetNanoSeconds()
              << ",\"lifetime_ns\":" << s.NeighborLifeLength.GetNanoSeconds()
              << ",\"energy_j\":" << s.ResEnergy << ",\"cached_fitness\":" << s.FitnessValue
              << ",\"compute_capacity\":" << s.computeCapacity << ",\"load\":" << s.currentLoad
              << ",\"compute_status\":" << TaskObservationWriter::Quote(neighbor.compute.StatusName())
              << ",\"compute_usable\":" << boolean(neighbor.compute.IsUsable())
              << ",\"compute_received_ns\":";
          if (neighbor.compute.receivedAt) out << neighbor.compute.receivedAt->GetNanoSeconds(); else out << "null";
          out << ",\"compute_expires_ns\":";
          if (neighbor.compute.expiresAt) out << neighbor.compute.expiresAt->GetNanoSeconds(); else out << "null";
          out << ",\"position\":";
          vector(s.Position);
          out << ",\"velocity\":";
          vector(s.Velocity);
          out << '}';
        }
        out << "]}";
      }
      out << "]}";
    }
  }
  out << "]}\n";
}

void CafbcrSimulation::SavePhyHeaderBoundarySummary() const
{
  std::ofstream out(std::filesystem::path(taskOutputDir) / "phy-header-boundary-summary.json");
  NS_ABORT_MSG_IF(!out, "Cannot save PHY header-boundary summary");
  uint64_t totalDeferrals = 0;
  uint32_t guardedPhys = 0;
  std::ostringstream perNode;
  for (uint32_t i = 0; i < devices.GetN(); ++i) {
    auto device = DynamicCast<WifiNetDevice>(devices.Get(i));
    NS_ABORT_MSG_IF(!device, "Header-boundary summary requires Wi-Fi devices");
    auto phy = DynamicCast<CafbcrHeaderBoundaryYansWifiPhy>(device->GetPhy());
    NS_ABORT_MSG_IF(phyHeaderBoundaryGuard != static_cast<bool>(phy), "PHY adaptation/configuration mismatch");
    const uint64_t count = phy ? phy->GetHeaderBoundaryDeferrals() : 0;
    totalDeferrals += count;
    guardedPhys += phy ? 1 : 0;
    if (i) perNode << ',';
    perNode << "{\"node\":" << allNodes.Get(i)->GetId() << ",\"deferrals\":" << count << '}';
  }
  out << "{\"schema\":\"cafbcr-entry-header-boundary-v1\",\"formal_eligible\":false"
      << ",\"enabled\":" << (phyHeaderBoundaryGuard ? "true" : "false")
      << ",\"modulation\":\"ERP_OFDM\",\"policy\":\"due_header_then_same_timestamp_arrival\""
      << ",\"observation_stop_ns\":" << Seconds(simulationTime).GetNanoSeconds()
      << ",\"nodes\":" << devices.GetN() << ",\"guarded_phys\":" << guardedPhys
      << ",\"deferrals\":" << totalDeferrals << ",\"measurement_valid\":true"
      << ",\"per_node\":[" << perNode.str() << "]}\n";
}

// ==================== FinalizeStatistics ====================
// 职责：仿真结束前汇总FlowMonitor数据和任务应用数据，生成性能报告
void CafbcrSimulation::FinalizeStatistics()
{

  // 从FlowMonitor提取性能数据
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());

  // 刷新丢包状态，确保在途包被正确统计
  monitor->CheckForLostPackets();
  std::map<FlowId, FlowMonitor::FlowStats> allStats = monitor->GetFlowStats();

  double totalPDR = 0.0;
  double totalDelay = 0.0;
  uint64_t totalHops = 0;
  uint64_t totalRxBytes = 0;
  uint64_t totalTxPackets = 0;
  uint32_t validFlows = 0;
  double minStartTime = std::numeric_limits<double>::max();
  double maxStopTime = 0.0;

  // Export the existing FlowMonitor matches without changing legacy calculations.
  std::ofstream udpFlowmon;
  if (!csvOnly) udpFlowmon.open(std::filesystem::path(taskOutputDir) / "udp-flowmonitor.csv");
  NS_ABORT_MSG_IF(!csvOnly && !udpFlowmon, "Cannot open raw UDP FlowMonitor output");
  udpFlowmon << "flow_index,flow_id,protocol,source_ip,destination_ip,source_port,destination_port,"
                "tx_packets,rx_packets,rx_bytes,delay_sum_ns,first_tx_ns,last_rx_ns,times_forwarded,lost_packets\n";
  uint32_t flowIndex = 0;

  // 遍历所有测试流，计算PDR、延迟、跳数、吞吐量
  for (const auto& testFlow : m_testFlows) {
    for (const auto& stat : allStats) {
      Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(stat.first);

      if (t.sourceAddress == testFlow.srcAddr && t.destinationAddress == testFlow.dstAddr &&
          (t.destinationPort == testFlow.port || t.sourcePort == testFlow.port)) {

        if (!csvOnly) udpFlowmon << flowIndex << ',' << stat.first << ',' << unsigned(t.protocol) << ','
                   << t.sourceAddress << ',' << t.destinationAddress << ',' << t.sourcePort << ','
                   << t.destinationPort << ',' << stat.second.txPackets << ',' << stat.second.rxPackets << ','
                   << stat.second.rxBytes << ',' << stat.second.delaySum.GetNanoSeconds() << ','
                   << stat.second.timeFirstTxPacket.GetNanoSeconds() << ','
                   << stat.second.timeLastRxPacket.GetNanoSeconds() << ',' << stat.second.timesForwarded << ','
                   << stat.second.lostPackets << '\n';

        uint64_t txPackets = stat.second.txPackets;
        uint64_t rxPackets = stat.second.rxPackets;
        uint64_t rxBytes = stat.second.rxBytes;

        if (txPackets > 0) {
          validFlows++;
          double pdr = (double)rxPackets / txPackets * 100.0;
          totalPDR += pdr;

          double delayMs = stat.second.delaySum.GetMilliSeconds();
          double avgDelay = (rxPackets > 0) ? delayMs / rxPackets : 0.0;
          totalDelay += avgDelay;

          uint64_t hops = (rxPackets > 0) ? stat.second.timesForwarded / rxPackets : 0;
          totalHops += hops;

          totalRxBytes += rxBytes;
          totalTxPackets += txPackets;

          // 追踪实际传输时间窗口
          minStartTime = std::min(minStartTime, stat.second.timeFirstTxPacket.GetSeconds());
          maxStopTime = std::max(maxStopTime, stat.second.timeLastRxPacket.GetSeconds());
        }
      }
    }
    ++flowIndex;
  }

  if (!csvOnly) {
  udpFlowmon.close();
  NS_ABORT_MSG_IF(!udpFlowmon, "Cannot write raw UDP FlowMonitor output");
  std::ofstream taskFlowmon(std::filesystem::path(taskOutputDir) / "task-flowmonitor.csv");
  NS_ABORT_MSG_IF(!taskFlowmon, "Cannot open task FlowMonitor output");
  taskFlowmon << "flow_id,kind,source_ip,destination_ip,source_port,destination_port,tx_packets,rx_packets,tx_bytes,rx_bytes\n";
  for (const auto& [id, stats] : allStats) {
    const auto tuple = classifier->FindFlow(id);
    const uint16_t taskPort = protocol == "cfr-eng" ? 26911 : 9999;
    if (tuple.protocol != 17 || (tuple.sourcePort != taskPort && tuple.destinationPort != taskPort)) continue;
    taskFlowmon << id << ',' << (protocol == "cfr-eng" ? "cfr-data-or-ack" : tuple.destinationPort == 9999 ? "request" : "completion") << ','
      << tuple.sourceAddress << ',' << tuple.destinationAddress << ',' << tuple.sourcePort << ','
      << tuple.destinationPort << ',' << stats.txPackets << ',' << stats.rxPackets << ','
      << stats.txBytes << ',' << stats.rxBytes << '\n';
  }
  taskFlowmon.close();
  NS_ABORT_MSG_IF(!taskFlowmon, "Cannot write task FlowMonitor output");
  }

  // 计算平均值
  double avgPDR = (validFlows > 0) ? totalPDR / validFlows : 0.0;
  double avgDelay = (validFlows > 0) ? totalDelay / validFlows : 0.0;
  double avgHops = (validFlows > 0) ? totalHops / validFlows : 0.0;

  // 根据实际传输时间窗口计算吞吐量（不是用仿真总时长）
  double duration = (minStartTime <= maxStopTime) ? (maxStopTime - minStartTime) : 0.0;
  double throughput = (duration > 0.0) ? (totalRxBytes * 8.0) / (duration * 1000.0) : 0.0; // kbps

  // 从路由协议获取控制包统计
  uint64_t totalControlPackets = 0;
  double controlThroughput = 0.0;
  uint64_t helloPacketCount = 0;
  // 非分簇协议靠各自路由协议的 GetAverageLinkLifetime() 统计链路时长
  // (分簇协议走 ClusterNode::Neighbors 那条路径，由 PerformanceMetrics::CollectLinkMetrics 算)
  double sumAvgLinkLifetime = 0.0;

  uint32_t totalNodes = allNodes.GetN();
  for (uint32_t i = 0; i < totalNodes; i++) {
    Ptr<Node> node = allNodes.Get(i);

    if (protocol == "cafbcr") {
      Ptr<cafbcr::RoutingProtocol> routing = node->GetObject<cafbcr::RoutingProtocol>();
      if (routing) {
        totalControlPackets += routing->GetControlPacketCount();
        controlThroughput += routing->GetControlThroughput();
        helloPacketCount += routing->GetHelloCount();
      }
    } else if (UsesMo()) {
      auto routing = node->GetObject<mo::RoutingProtocol>();
      NS_ABORT_MSG_IF(!routing, "Missing MOGNN statistics source");
      totalControlPackets += routing->GetControlPacketCount();
      helloPacketCount += routing->GetHelloCount();
      controlThroughput += routing->GetControlThroughput();
      sumAvgLinkLifetime += routing->GetAverageLinkLifetime();
    } else if (protocol == "cfr-eng") {
      auto routing = node->GetObject<cfreng::RoutingProtocol>();
      NS_ABORT_MSG_IF(!routing, "Missing CFR-ENG statistics source");
      totalControlPackets += routing->GetControlPacketCount();
      helloPacketCount += routing->GetHelloCount();
      controlThroughput += routing->GetControlThroughput();
      sumAvgLinkLifetime += routing->GetAverageLinkLifetime();
    } else if (protocol == "qrcf") {
      auto routing = node->GetObject<qrcf::RoutingProtocol>();
      NS_ABORT_MSG_IF(!routing, "Missing QRCF statistics source");
      totalControlPackets += routing->GetControlPacketCount();
      helloPacketCount += routing->GetHelloCount();
      controlThroughput += routing->GetControlThroughput();
      sumAvgLinkLifetime += routing->GetDiagnostics().AverageCompletedLinkSeconds();
    } else if (protocol == "aodv") {
      Ptr<aodv::RoutingProtocol> routing = node->GetObject<aodv::RoutingProtocol>();
      if (routing) {
        totalControlPackets += routing->GetTotalControlPackets();
        helloPacketCount += routing->GetHELLOCount();
        controlThroughput += routing->GetControlThroughout();  // 注意拼写
        sumAvgLinkLifetime += routing->GetAverageLinkLifetime();
      }
    } else if (protocol == "olsr") {
      Ptr<olsr::RoutingProtocol> routing = node->GetObject<olsr::RoutingProtocol>();
      if (routing) {
        totalControlPackets += routing->GetTotalControlPackets();
        helloPacketCount += routing->GetHelloCount();
        controlThroughput += routing->GetControlThroughput();
        sumAvgLinkLifetime += routing->GetAverageLinkLifetime();
      }
    } else if (protocol == "icra") {
      Ptr<icra::RoutingProtocol> routing = node->GetObject<icra::RoutingProtocol>();
      if (routing) {
        totalControlPackets += routing->GetControlPacketCount();
        controlThroughput += routing->GetControlThroughput();
        helloPacketCount += routing->GetHelloCount();
      }
    }
  }

  double controlOverheadRatio = (totalTxPackets > 0) ? (double)totalControlPackets / totalTxPackets : 0.0;
  if (controlCostMeter) controlThroughput = static_cast<double>(controlCostMeter->Bytes()) / 1000.0;
  double controlThroughputPerTime = (simulationTime > 0.0) ? controlThroughput / simulationTime : 0.0;
  if (controlCostMeter) {
    std::cout << "Control meter (l3): packets=" << controlCostMeter->Packets()
              << " bytes=" << controlCostMeter->Bytes()
              << " kB/s=" << controlThroughputPerTime
              << " kbit/s=" << controlThroughputPerTime * 8.0 << std::endl;
  }

  // 收集能耗（每节点异构 initial - remaining）
  double totalEnergyConsumed = 0.0;
  uint16_t deadNodes = 0;
  for (uint32_t i = 0; i < allNodes.GetN(); ++i) {
    Ptr<Node> n = allNodes.Get(i);
    Ptr<EnergySourceContainer> sc = n->GetObject<EnergySourceContainer>();
    if (!sc || sc->GetN() == 0) continue;
    Ptr<BasicEnergySource> src = DynamicCast<BasicEnergySource>(sc->Get(0));
    if (!src) continue;
    double initial   = src->GetInitialEnergy();
    double remaining = src->GetRemainingEnergy();
    totalEnergyConsumed += (initial - remaining);
    // 低于初始5% 或 100mJ 认为已耗尽
    if (remaining <= 0.1 || (initial > 0 && remaining <= initial * 0.05)) deadNodes++;
  }

  // ========== 计算任务端到端时延汇总（先算好，下面 commonMetrics 要带上）==========
  // ComputeTaskClientApp.responseTime = 网络往返 + 服务端排队等待 + 服务端处理时延
  double sumResponseTime = 0.0;
  double sumProcessingDelay = 0.0;
  double sumQueueWaitTime = 0.0;
  uint32_t completedTasks = 0;
  uint32_t processedTasks = 0;
  uint32_t generatedTasks = 0;
  uint32_t missedDeadline = 0;

  for (uint32_t i = 0; i < allNodes.GetN(); ++i) {
    Ptr<Node> n = allNodes.Get(i);
    for (uint32_t a = 0; a < n->GetNApplications(); ++a) {
      auto client = DynamicCast<cafbcr::ComputeTaskClientApp>(n->GetApplication(a));
      if (client) {
        uint32_t gen = client->GetTasksGenerated();
        uint32_t done = client->GetTasksCompleted();
        generatedTasks += gen;
        completedTasks += done;
        missedDeadline += client->GetTasksMissedDeadline();
        if (done) sumResponseTime += client->GetAverageResponseTime() * done;
      }
      auto server = DynamicCast<cafbcr::ComputeTaskServerApp>(n->GetApplication(a));
      if (server) {
        uint32_t proc = server->GetTasksProcessed();
        processedTasks += proc;
        if (proc) {
          sumProcessingDelay += server->GetAverageProcessingDelay() * proc;
          sumQueueWaitTime += server->GetAverageQueueWaitTime() * proc;
        }
      }
    }
  }

  if (cfrAdapter) {
    const auto& summary = taskObserver->GetSummary();
    processedTasks = summary.finished;
    if (summary.processingSamples)
      sumProcessingDelay = summary.processingNs / 1e9 / summary.processingSamples * processedTasks;
    if (summary.queueSamples)
      sumQueueWaitTime = summary.queueNs / 1e9 / summary.queueSamples * processedTasks;
  }

  double avgResponseTime = (completedTasks > 0)
                             ? (sumResponseTime / completedTasks) * 1000.0 : 0.0; // ms
  double avgProcessingDelay = (processedTasks > 0)
                             ? (sumProcessingDelay / processedTasks) * 1000.0 : 0.0; // ms
  double avgQueueWaitTime = (processedTasks > 0)
                             ? (sumQueueWaitTime / processedTasks) * 1000.0 : 0.0; // ms
  double taskCompletionRate = (generatedTasks > 0)
                             ? 100.0 * completedTasks / generatedTasks : 0.0;
  // missedDeadline covers both timeouts and late-but-completed tasks, so the
  // denominator must be generatedTasks: the rate then answers "what fraction
  // of generated tasks missed their deadline" and stays within [0, 100]%.
  double deadlineMissRate = (generatedTasks > 0)
                             ? 100.0 * missedDeadline / generatedTasks : 0.0;

  // 构建CommonMetrics
  ns3::PerformanceMetrics::CommonMetrics commonMetrics;
  commonMetrics.pdr = avgPDR;
  commonMetrics.avgDelay = avgDelay;
  commonMetrics.avgHops = avgHops;
  commonMetrics.validFlows = validFlows;
  commonMetrics.throughput = throughput;
  commonMetrics.controlOverheadRatio = controlOverheadRatio;
  commonMetrics.controlPacketNum = totalControlPackets;
  commonMetrics.ControlPacketTHP = controlThroughputPerTime;
  commonMetrics.helloPacketCount = helloPacketCount;
  commonMetrics.totalEnergyConsumed = totalEnergyConsumed;
  commonMetrics.avgEnergyPerNode = (allNodes.GetN() > 0)
                                     ? totalEnergyConsumed / allNodes.GetN() : 0.0;
  commonMetrics.deadNodes = deadNodes;
  // AvergeLinkLength 显式赋值（结构体没有 in-class 初始化）。
  //   - 分簇协议: 置 0，下游走 CalculateAverageLinkLifetime() 兜底（基于 ClusterNode::Neighbors）
  //   - 非分簇协议: 由各协议自己的 GetAverageLinkLifetime() 累加后取均值
  if (protocol == "olsr" || protocol == "aodv" || protocol == "qrcf" || protocol == "cfr-eng" || UsesMo()) {
    commonMetrics.AvergeLinkLength = (totalNodes > 0)
                                       ? sumAvgLinkLifetime / totalNodes : 0.0;
  } else {
    commonMetrics.AvergeLinkLength = 0.0;
  }
  // 计算任务字段
  commonMetrics.tasksGenerated     = generatedTasks;
  commonMetrics.tasksCompleted     = completedTasks;
  commonMetrics.taskCompletionRate = taskCompletionRate;
  commonMetrics.avgResponseTime    = avgResponseTime;
  commonMetrics.avgProcessingDelay = avgProcessingDelay;
  commonMetrics.avgQueueWaitTime   = avgQueueWaitTime;
  commonMetrics.deadlineMissRate   = deadlineMissRate;

  // 设置CommonMetrics并生成报告
  if (m_metrics) {
    m_metrics->SetCommonMetrics(commonMetrics);
    m_metrics->PrintComprehensiveReport(std::cout, protocol,
                                        allNodes.GetN(), simulationTime);

    // 对于分簇协议，最终收集分簇性能指标（PrintComprehensiveReport会自动输出）
    if (m_metrics->IsClusteringAlgorithm(protocol)) {
      m_metrics->CollectClusteringMetrics();  // 最终收集
      m_metrics->CollectLinkMetrics();        // 链路指标
      if (csvOnly) csvResults.SampleClustering(allNodes);
    }

    // CSV label: CA-FBCR variants use the ca-* labels, baselines use their own names.
    const auto csvAlgo = CsvAlgorithm();

    if (csvOnly) {
      CsvResults::InputConfig inputs{taskGenStart, taskGenStop, taskStartMode, udpStart, udpStop,
                                    numTestFlows, controlMeter};
      csvResults.Append(resultsCsv, csvAlgo, protocol, allNodes.GetN(), minSpeed, maxSpeed, seed,
          simulationTime, RngSeedManager::GetRun(), channelConfig, taskSelection, csvBuildId,
          taskDeadlineCoverage, commonMetrics, m_metrics, taskObserver->GetSummary(),
          udpObserver->GetCounts(), udpDemand == "fixed-attempts", csvConfiguration ? &inputs : nullptr,
          failureSummary.get(), runDiagnostics.get(), scenarioContract ? &*scenarioContract : nullptr,
          csvUdpMetrics);
      std::cout << "性能指标已追加至: " << resultsCsv << std::endl;
      return;
    }

    // 文件名带 size 和 speed 区段，避免不同 (size, speed) 组合共写一份
    // wC 扫描时额外带 wc 后缀，与主 bench 隔离
    std::ostringstream fnOss;
    if (sweepWc >= 0.0) {
      fnOss << "ca-fbcr-wcSweep_" << allNodes.GetN() << "nodes_speed"
            << (int)minSpeed << "-" << (int)maxSpeed
            << "_wc" << std::fixed << std::setprecision(2) << sweepWc << ".csv";
    } else if (asymLevel >= 0) {
      // [asymmetry-stress] 每档功率差单独一份，避免不同 level 行键冲突
      fnOss << "ca-fbcr-asymSweep_" << allNodes.GetN() << "nodes_speed"
            << (int)minSpeed << "-" << (int)maxSpeed
            << "_L" << asymLevel << ".csv";
    } else {
      fnOss << "ca-fbcr-benchmark_" << allNodes.GetN() << "nodes_speed"
            << (int)minSpeed << "-" << (int)maxSpeed << ".csv";
    }
    std::string csvFilename = fnOss.str();
    csvFilename.insert(csvFilename.size() - 4, "_" + fanetMode);
    if (protocol == "cafbcr") csvFilename.insert(csvFilename.size() - 4, "_hardfix");
    if (protocol == "cafbcr" && !disableAsym && caProxyJoin && caProxyRenewal)
      csvFilename.insert(csvFilename.size() - 4, "_proxyrenewal");
    if (protocol == "cafbcr" && !disableAsym && caProxyJoin && caProxyRenewal && caProxyRenewalPolicy=="ack-batch")
      csvFilename.insert(csvFilename.size() - 4, "_ackbatch");
    if (protocol == "cafbcr" && !disableAsym && caProxyJoin && caProxyRenewal && caProxyRenewalPolicy=="ack-stagger")
      csvFilename.insert(csvFilename.size() - 4, "_ackstagger");
    if (protocol == "cafbcr" && !disableAsym && caProxyJoin && caProxyRenewal && caProxyRenewalPolicy=="direct-feedback")
      csvFilename.insert(csvFilename.size() - 4, "_directfeedback");
    if (caDirectCmPolicy=="feedback-covered")
      csvFilename.insert(csvFilename.size() - 4, "_feedbackcovered");
    if (caControlUnicastPolicy=="one-hop")
      csvFilename.insert(csvFilename.size() - 4, "_controlonehop");
    if (caProxyRelayPolicy!="legacy")
      csvFilename.insert(csvFilename.size() - 4, "_relayackstagger");
    if (channelConfig.model == "shadow") csvFilename.insert(csvFilename.size() - 4, "_shadow");
    if (channelConfig.model == "lognormal") csvFilename.insert(csvFilename.size() - 4, "_lognormal");
    // Keep the shared CSV schema, but never append new task semantics to old campaigns.
    csvFilename = (std::filesystem::path(taskOutputDir) / ("legacy-" + csvFilename)).string();

    m_metrics->SaveToCSV(csvFilename, csvAlgo, seed,
                         allNodes.GetN(), minSpeed, maxSpeed);
    std::cout << "性能指标已保存至: " << csvFilename << " (algo=" << csvAlgo << ")" << std::endl;
  }

  std::cout << "\n========== 计算任务端到端性能 ==========\n"
            << std::fixed << std::setprecision(2)
            << "生成/完成/完成率     : " << generatedTasks
            << " / " << completedTasks
            << " / " << taskCompletionRate << "%\n"
            << "端到端响应时延       : " << avgResponseTime << " ms"
            << "  (网络往返 + 排队 + 处理)\n"
            << "服务端已完成任务处理时延: " << avgProcessingDelay << " ms"
            << "  (workload / computeCapacity)\n"
            << "服务端已完成任务排队时延: " << avgQueueWaitTime << " ms"
            << " (服务端完成样本=" << processedTasks << ")\n"
            << "截止时间违约率       : " << deadlineMissRate << "%\n"
            << "对比 UDP 基准流时延  : " << avgDelay << " ms (仅网络层)\n"
            << "=========================================" << std::endl;

  // 更新 main Report 使用的聚合值
  totalTasksGenerated = generatedTasks;
  totalTasksCompleted = completedTasks;

  // ========== Drop trace 摘要（仅当 --traceDrops=true）==========
  if (traceDrops) {
    static const char* kReasonName[] = {
        "?", "TTL_EXPIRED", "NO_ROUTE", "BAD_CHECKSUM",
        "INTERFACE_DOWN", "ROUTE_ERROR", "FRAGMENT_TIMEOUT"};

    std::cout << "\n========== Drop Trace ==========\n"
              << "IPv4 总 drop: " << m_dropTotal << "\n"
              << "其中测试流 drop: " << m_dropTestFlow << "\n"
              << "按原因分布:" << std::endl;
    for (const auto& [reason, cnt] : m_dropByReason) {
      const char* name = (reason >= 1 && reason <= 6) ? kReasonName[reason] : "UNKNOWN";
      std::cout << "  [" << reason << "] " << name << ": " << cnt << std::endl;
    }

    // 打印 drop 最多的 Top10 节点（看是不是集中在簇头/特定位置）
    std::vector<std::pair<uint32_t, uint64_t>> sortedNodes(m_dropByNodeIdx.begin(),
                                                            m_dropByNodeIdx.end());
    std::sort(sortedNodes.begin(), sortedNodes.end(),
              [](const auto& a, const auto& b) { return a.second > b.second; });
    std::cout << "Top 10 drop 节点 (nodeId: count):" << std::endl;
    for (size_t i = 0; i < std::min<size_t>(10, sortedNodes.size()); ++i) {
      // 标注节点类型
      const auto& nodeConfigs = config->GetNodeConfigs();
      uint32_t id = sortedNodes[i].first;
      const char* typeName = "?";
      if (id < nodeConfigs.size()) {
        switch (nodeConfigs[id].nodeType) {
          case COMMAND: typeName = "COMMAND"; break;
          case RELAY:   typeName = "RELAY";   break;
          case TASK:    typeName = "TASK";    break;
          case SWARM:   typeName = "SWARM";   break;
        }
      }
      std::cout << "  Node " << id << " (" << typeName << "): " << sortedNodes[i].second
                << std::endl;
    }

    // ===== FlowMonitor 总账（核对丢包去向）=====
    // totalTxPackets is the actual number of test-flow transmissions accumulated above.
    std::cout << "\n=== 测试流 FlowMonitor 账本 ===" << std::endl;
    std::cout << "总发送 (txPackets):       " << totalTxPackets << std::endl;
    std::cout << "总接收 (rxBytes/512≈包):  " << (totalRxBytes / 512) << std::endl;
    std::cout << "TTL 环路 drop (IPv4):     " << m_dropTestFlow << std::endl;

    // ===== MAC 层 drop（队列满 + 重传耗尽）=====
    std::cout << "\nMAC TxDrop (队列满) 总计: " << m_macTxDrop << std::endl;
    std::cout << "MAC DroppedMpdu 总计:    " << m_macMpduDrop << std::endl;
    if (m_macMpduDrop > 0) {
      // WifiMacDropReason 取值（ns-3.40 wifi-mac.h）：
      //   0 = WIFI_MAC_DROP_FAILED_ENQUEUE
      //   1 = WIFI_MAC_DROP_EXPIRED_LIFETIME
      //   2 = WIFI_MAC_DROP_REACHED_RETRY_LIMIT
      //   3 = WIFI_MAC_DROP_QOS_OLD_PACKET
      static const char* kMpduReason[] = {
          "FAILED_ENQUEUE", "EXPIRED_LIFETIME", "REACHED_RETRY_LIMIT", "QOS_OLD_PACKET"};
      std::cout << "DroppedMpdu 按原因分布:" << std::endl;
      for (const auto& [r, c] : m_macMpduDropByReason) {
        const char* name = (r < 4) ? kMpduReason[r] : "UNKNOWN";
        std::cout << "  [" << r << "] " << name << ": " << c << std::endl;
      }

      std::vector<std::pair<uint32_t, uint64_t>> sortedMpdu(
          m_macMpduDropByNodeIdx.begin(), m_macMpduDropByNodeIdx.end());
      std::sort(sortedMpdu.begin(), sortedMpdu.end(),
                [](const auto& a, const auto& b) { return a.second > b.second; });
      std::cout << "Top 10 DroppedMpdu 节点:" << std::endl;
      for (size_t i = 0; i < std::min<size_t>(10, sortedMpdu.size()); ++i) {
        const auto& nodeConfigs = config->GetNodeConfigs();
        uint32_t id = sortedMpdu[i].first;
        const char* typeName = "?";
        if (id < nodeConfigs.size()) {
          switch (nodeConfigs[id].nodeType) {
            case COMMAND: typeName = "COMMAND"; break;
            case RELAY:   typeName = "RELAY";   break;
            case TASK:    typeName = "TASK";    break;
            case SWARM:   typeName = "SWARM";   break;
          }
        }
        std::cout << "  Node " << id << " (" << typeName << "): "
                  << sortedMpdu[i].second << std::endl;
      }
    }

    // ===== Per-node MAC TxDrop distribution =====
    if (m_macTxDrop > 0) {
      std::cout << "\nMAC TxDrop 节点分布 (Top 10):" << std::endl;
    std::vector<std::pair<uint32_t, uint64_t>> sortedMac(m_macDropByNodeIdx.begin(),
                                                         m_macDropByNodeIdx.end());
    std::sort(sortedMac.begin(), sortedMac.end(),
              [](const auto& a, const auto& b) { return a.second > b.second; });
    std::cout << "Top 10 MAC drop 节点 (nodeId: count):" << std::endl;
    for (size_t i = 0; i < std::min<size_t>(10, sortedMac.size()); ++i) {
      const auto& nodeConfigs = config->GetNodeConfigs();
      uint32_t id = sortedMac[i].first;
      const char* typeName = "?";
      if (id < nodeConfigs.size()) {
        switch (nodeConfigs[id].nodeType) {
          case COMMAND: typeName = "COMMAND"; break;
          case RELAY:   typeName = "RELAY";   break;
          case TASK:    typeName = "TASK";    break;
          case SWARM:   typeName = "SWARM";   break;
        }
      }
      std::cout << "  Node " << id << " (" << typeName << "): " << sortedMac[i].second
                << std::endl;
      }
    }

    std::cout << "\nPHY TxDrop: " << m_phyTxDrop << "\n"
              << "PHY RxDrop: " << m_phyRxDrop << "\n"
              << "================================" << std::endl;
  }
}

std::string CafbcrSimulation::CsvAlgorithm() const
{
  std::string label = protocol;
  if (UsesMo()) label += taskSelection == "fixed" ? "-fixed-destination" : "-native-offload";
  if (protocol == "cfr-eng" && cfrEngCompute != "joint") label += "-" + cfrEngCompute;
  if (protocol == "cafbcr")
    label = helloMethod == "fuzzy" ? "ca-fbcr" : helloMethod == "qlearning" ? "ca-qbcr" : "ca-bcr";
  if (disableAsym) label += "-noasym";
  else if (protocol == "cafbcr") {
    if (!caProxyJoin) label += "-noproxy";
    if (!caRangeGating) label += "-norange";
    if (caLinkProbes) label += "-linkprobe-" + caLinkGate;
  }
  return label;
}

scenario::Contract CafbcrSimulation::MakeScenarioContract() const
{
  scenario::Contract result;
  const bool ca = protocol == "cafbcr";
  result.Set("Algorithm", CsvAlgorithm());
  result.Set("Nodes", config->GetNodeCount());
  result.Set("MinSpeed(m/s)", minSpeed); result.Set("MaxSpeed(m/s)", maxSpeed);
  result.Set("AsymLevel", asymLevel); result.Set("AsymRangeProfile", asymRangeProfile);
  result.Set("SimTime(s)", simulationTime);
  result.Set("ChannelModel", channelConfig.model);
  if (channelConfig.model == "lognormal") result.Set("PathLossExponent", channelConfig.pathLossExponent);
  else result.Set("PathLossExponent", "N/A");
  result.Set("ShadowSigma(dB)", channelConfig.model == "legacy" ? 0 : channelConfig.sigmaDb);
  result.Set("ChannelProfile", channelConfig.Json());
  result.Set("TaskSelection", taskSelection);
  result.Set("TaskGenStart(s)", taskGenStart); result.Set("TaskGenStop(s)", taskGenStop);
  result.Set("TaskStartMode", taskStartMode);
  result.Set("DeadlineWindowComplete", taskDeadlineCoverage);
  result.Set("UDPWindowMode", udpStart >= 0 ? "explicit-window" : "legacy-time");
  if (udpStart >= 0) { result.Set("UDPStart(s)", udpStart); result.Set("UDPStop(s)", udpStop); }
  else { result.Set("UDPStart(s)", "N/A"); result.Set("UDPStop(s)", "N/A"); }
  result.Set("UDPFlows", numTestFlows); result.Set("UDPInterval(s)", .5); result.Set("UDPPacketBytes", 512);
  result.Set("UDPDemand", udpDemand); result.Set("ControlMeter", controlMeter);
  // Effective shared ARP settings. These are applied with Config::SetDefault
  // before the stack is installed, so they are a run-wide condition for every
  // protocol started through this entry. Recording the effective values, not
  // the presence of a CLI flag, keeps omitted and explicit defaults identical.
  result.Set("arpDeadReprobe", arpDeadReprobe);
  result.Set("arpMaxRetries", arpMaxRetries);
  result.Set("arpWaitReplyMs", arpWaitReplyMs);
  if (csvUdpMetrics) result.Set("UDPMetricProfile", CsvResults::UDP_METRIC_PROFILE);
  result.Set("FanetMode", ClusterNode::FitnessModeName(fitnessMode));
  result.Set("ComputeEnabled", enableComputeTask);
  result.Set("TaskWire", protocol == "cfr-eng" ? "cfr-task-header-local-v1" : taskWire);
  result.Set("QueueSyncRequested", syncQueueLoad);
  result.Set("QueueSyncModel", protocol == "cfr-eng" ? "cfr-own-reservation-load" : "ca-server-waiting-queue");
  result.Set("HelloMethod", ca ? helloMethod : "N/A");
  if (ca && helloMethod == "qlearning") {
    result.Set("CAQHelloProfile", QLearningHelloManager::ConfigurationJson());
    result.Set("CAHelloBaseInterval(s)", config->GetHelloInterval());
    result.Set("CANeighborTtl(s)", 2 * config->GetHelloInterval());
  }
  result.Set("CAInitialSelector", ca ? caInitialSelector : "N/A");
  result.Set("CAProxyJoin", ca && !disableAsym && caProxyJoin);
  result.Set("CAProxyRenewal", ca && !disableAsym && caProxyJoin && caProxyRenewal);
  result.Set("CARangeGating", ca && !disableAsym && caRangeGating);
  result.Set("CALoopPolicy", ca ? caLoopPolicy : "N/A");
  result.Set("CAProxyRenewalPolicy", ca ? caProxyRenewalPolicy : "N/A");
  // Absence means the historical legacy behavior; keep all old ScenarioIds.
  if (caDirectCmPolicy != "legacy") result.Set("CADirectCmPolicy", caDirectCmPolicy);
  if (caControlUnicastPolicy != "legacy") result.Set("CAControlUnicastPolicy", caControlUnicastPolicy);
  if (caProxyRelayPolicy != "legacy") result.Set("CAProxyRelayPolicy", caProxyRelayPolicy);
  result.Set("CAProxyRenewalWindowMs", ca ? caProxyRenewalWindowMs : 0);
  result.Set("CAProxyAckBatchMs", ca ? caProxyAckBatchMs : 0);
  result.Set("CALinkGate", ca ? caLinkGate : "N/A");
  result.Set("CALinkProbes", ca && caLinkProbes);
  result.Set("CALinkProbeInterval", ca ? caLinkProbeInterval : 0);
  result.Set("CALinkProbeTimeout", ca ? caLinkProbeTimeout : 0);
  result.Set("CALinkValidity", ca ? caLinkValidity : 0);
  result.Set("ProtocolProfile", BaselineProfileJson());
  result.Set("CfrEngCompute", protocol == "cfr-eng" ? cfrEngCompute : "N/A");
  result.Set("StatsInterval(s)", statisticsCollectionInterval);
  result.Set("PhyHeaderBoundaryGuard", phyHeaderBoundaryGuard);
  result.Set("MobilityModel", "GaussMarkov-TimeStep0.5-Alpha0.85-class-scaled-mean");
  result.Set("MobilityStreamPolicy", "existing-implicit-streams-not-full-pairing-proof");
  const std::array<UavTypeConfig,4> defaults{UavTypeConfig::CreateCommandNode(), UavTypeConfig::CreateRelayNode(),
                                           UavTypeConfig::CreateTaskNode(), UavTypeConfig::CreateSwarmNode()};
  const std::array<uint32_t,4> counts{numCommandNodes,numRelayNodes,numTaskNodes,numSwarmNodes};
  for (unsigned type = 0; type < defaults.size(); ++type) {
    const auto& node = defaults[type];
    const auto radio = scenario::Resolve(type, asymLevel, asymRangeProfile, node.maxCommunicationRange);
    const auto prefix = node.GetNodeTypeString() + ".";
    result.Set(prefix + "Count", counts[type]); result.Set(prefix + "TxDbm", radio.txDbm);
    result.Set(prefix + "RxDbm", radio.rxDbm); result.Set(prefix + "NominalRange(m)", radio.range);
    result.Set(prefix + "DataMode", radio.dataMode);
    result.Set(prefix + "ComputeCapacity", node.computeCapacity); result.Set(prefix + "QueueCapacity", node.taskQueueCapacity);
    result.Set(prefix + "Battery(J)", node.batteryCapacity); result.Set(prefix + "IdlePower(W)", node.idlePowerConsumption);
    result.Set(prefix + "GenerateTasks", node.generateTasks); result.Set(prefix + "ProcessTasks", node.processTasks);
    result.Set(prefix + "TaskInterval(s)", node.taskGenerationInterval);
    result.Set(prefix + "MinWorkload", node.minTaskWorkload); result.Set(prefix + "MaxWorkload", node.maxTaskWorkload);
    result.Set(prefix + "MinPayload", node.minTaskDataSize); result.Set(prefix + "MaxPayload", node.maxTaskDataSize);
    result.Set(prefix + "MinDeadline(s)", node.minTaskDeadline); result.Set(prefix + "MaxDeadline(s)", node.maxTaskDeadline);
  }
  double l,e,d,r,c; config->GetFitnessWeights(l,e,d,r,c);
  result.Set("WeightLhtp", l); result.Set("WeightEnergy", e); result.Set("WeightDegree", d);
  result.Set("WeightDistance", r); result.Set("WeightCompute", c);
  return result;
}

std::string CafbcrSimulation::BaselineProfileJson() const
{
  if (UsesMo() && mognnAdapter) return mognnAdapter->ProfileJson();
  std::ostringstream out;
  out << std::setprecision(17);
  if (protocol == "olsr") {
    out << "{\"schema\":\"cafbcr-baseline-lineup-v1\",\"hello_interval_s\":" << olsrHelloInterval
        << ",\"other_intervals\":\"protocol_defaults_checked_in_baseline_runtime\"}";
  } else if (protocol == "qrcf") {
    out << "{\"schema\":\"cafbcr-baseline-lineup-v1\",\"preset\":" << TaskObservationWriter::Quote(qrcfPreset)
        << ",\"range_m\":" << qrcfSettings.range
        << ",\"range_scope\":\"nominal_protocol_filter_not_phy_cutoff\""
        << ",\"hello_lower_s\":" << qrcfSettings.helloLower << ",\"hello_upper_s\":" << qrcfSettings.helloUpper
        << ",\"outcome_updates\":" << (qrcfSettings.outcomeUpdates ? "true" : "false")
        << ",\"bootstrap\":" << (qrcfSettings.bootstrap ? "true" : "false")
        << ",\"hello_jitter\":" << (qrcfSettings.helloJitter ? "true" : "false")
        << ",\"stream_base\":1000,\"destination_position_service\":\"simulation_current_position_lookup\""
        << ",\"location_exchange_cost\":\"not_implemented\",\"remote_q_table_lookup\":false}";
  } else {
    out << "null";
  }
  return out.str();
}

void CafbcrSimulation::SaveBaselineRuntime() const
{
  if (protocol != "olsr" && protocol != "qrcf") return;
  std::ofstream out(std::filesystem::path(taskOutputDir) / "baseline-runtime.json");
  NS_ABORT_MSG_IF(!out, "Cannot open baseline runtime audit");
  out << std::setprecision(17) << "{\"schema\":\"cafbcr-baseline-lineup-v1\",\"formal_eligible\":false"
      << ",\"protocol\":" << TaskObservationWriter::Quote(protocol) << ",\"profile\":" << BaselineProfileJson()
      << ",\"checked_nodes\":" << allNodes.GetN();
  if (protocol == "olsr") {
    for (const auto& item : std::vector<std::pair<std::string, std::string>>{
        {"HelloInterval", "hello_interval_s"}, {"TcInterval", "tc_interval_s"},
        {"MidInterval", "mid_interval_s"}, {"HnaInterval", "hna_interval_s"}}) {
      TimeValue first;
      for (uint32_t i = 0; i < allNodes.GetN(); ++i) {
        auto routing = allNodes.Get(i)->GetObject<olsr::RoutingProtocol>();
        NS_ABORT_MSG_IF(!routing, "OLSR is not installed on every node");
        TimeValue actual;
        routing->GetAttribute(item.first, actual);
        if (!i) first = actual;
        NS_ABORT_MSG_IF(actual.Get() != first.Get(), "OLSR interval differs between nodes");
        NS_ABORT_MSG_IF(item.first == "HelloInterval" && actual.Get() != Seconds(olsrHelloInterval),
                        "Installed OLSR HELLO differs from requested interval");
      }
      out << ",\"" << item.second << "\":" << first.Get().GetSeconds();
    }
  } else if (protocol == "qrcf") {
    std::ofstream diagnostics(std::filesystem::path(taskOutputDir) / "qrcf-diagnostics.csv");
    NS_ABORT_MSG_IF(!diagnostics, "Cannot open QRCF native diagnostics");
    diagnostics << "node,";
    qrcf::Diagnostics::WriteColumns(diagnostics);
    diagnostics << '\n' << std::setprecision(17);
    for (uint32_t i = 0; i < allNodes.GetN(); ++i) {
      auto routing = allNodes.Get(i)->GetObject<qrcf::RoutingProtocol>();
      NS_ABORT_MSG_IF(!routing || !routing->DataRoutingReady(), "QRCF destination lookup is missing");
      const auto& actual = routing->GetSettings();
      NS_ABORT_MSG_IF(actual.PresetName() != qrcfPreset || actual.range != qrcfSettings.range ||
                      actual.outcomeUpdates != qrcfSettings.outcomeUpdates || actual.bootstrap != qrcfSettings.bootstrap ||
                      actual.helloJitter != qrcfSettings.helloJitter, "Installed QRCF profile differs from requested profile");
      diagnostics << allNodes.Get(i)->GetId() << ',';
      routing->GetDiagnostics().WriteValues(diagnostics);
      diagnostics << '\n';
    }
    NS_ABORT_MSG_IF(!diagnostics, "QRCF diagnostics write failed");
    out << ",\"preset\":" << TaskObservationWriter::Quote(qrcfPreset)
        << ",\"destination_lookup_ready\":true";
  }
  out << "}\n";
  NS_ABORT_MSG_IF(!out, "Baseline runtime audit write failed");
}

// ==================== Report ====================
void CafbcrSimulation::Report(std::ostream& os)
{
  // double completionRate = (totalTasksGenerated > 0) ? (100.0 * totalTasksCompleted / totalTasksGenerated) : 0.0;
  // os << "========== 计算任务统计 ========== \n生成: " << totalTasksGenerated
  //    << " | 完成: " << totalTasksCompleted << " | 完成率: " << std::fixed
  //    << std::setprecision(2) << completionRate << "%" << std::endl;
}

// ==================== Main ====================
int main(int argc, char* argv[])
{
  CafbcrSimulation sim;
  if (!sim.Configure(argc, argv)) {
    std::cerr << "Configuration failed." << std::endl;
    return 1;
  }
  std::cout << "[FANET] fitnessMode=" << sim.GetFitnessModeName()
            << " nodesCreated=" << NodeList::GetNNodes() << std::endl;
  if (sim.IsDryRun()) return 0;
  sim.Run();
  sim.Report(std::cout);
  return 0;
}
