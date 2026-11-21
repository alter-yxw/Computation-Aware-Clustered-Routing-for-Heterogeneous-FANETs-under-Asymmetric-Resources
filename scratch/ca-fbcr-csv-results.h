/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef CA_FBCR_CSV_RESULTS_H
#define CA_FBCR_CSV_RESULTS_H

#include "ca-fbcr-task-observer.h"
#include "ca-fbcr-udp-observer.h"
#include "ca-fbcr-shadow-channel.h"
#include "ca-fbcr-failure-summary.h"
#include "ca-fbcr-run-diagnostics.h"
#include "ca-fbcr-scenario-contract.h"
#include "ns3/performance-metrics.h"
#include "ns3/fanet-clustermanager.h"

namespace ns3::cafbcr {

// One end-of-run row. No event files, external postprocessing or shared FBCR edits.
class CsvResults
{
public:
  struct InputConfig {
    double taskStart, taskStop;
    std::string taskStartMode;
    double udpStart, udpStop;
    uint32_t udpFlows;
    std::string controlMeter;
  };

  static std::string Header()
  {
    return "Algorithm,Nodes,Speed(m/s),Seed,PDR(%),AvgDelay(ms),AvgHops,ValidFlows,"
      "Throughput(kbps),OverHeadRatio,ControlThroughput,HelloRatio,AvgLinkLen(s),TotalEnergy(J),AvgEnergy(J),"
      "DeadNodes,IsClusteringAlgo,TotalClusterNum,AvgClusterNum,AvgClusteringRatio(%),AvgClusterSize,"
      "AvgClusterLifetime(s),AvgClusterSwitchCount(/node),AvgRoleChangeCount(/node),"
      "TasksGenerated,TasksCompleted,TaskCompletionRate(%),AvgResponseTime(ms),AvgProcessingDelay(ms),"
      "AvgQueueWaitTime(ms),DeadlineMissRate(%),DeadlineSatisfactionRate(%),TasksOnTime,TasksLate,"
      "TasksUnconfirmedDue,TasksPending,TasksReceived,TasksProcessed,UDPAttempts,UDPReceived,UDPAttemptPDR(%),"
      "SimTime(s),RngRun,ChannelModel,PathLossExponent,ShadowSigma(dB),TaskSelection,BuildId,DeadlineWindowComplete";
  }

  static std::string ConfigurationHeader()
  {
    return Header() + ",TaskGenStart(s),TaskGenStop(s),TaskStartMode,UDPWindowMode,"
      "UDPStart(s),UDPStop(s),UDPFlows,UDPInterval(s),UDPPacketBytes,ControlMeter";
  }

  static constexpr const char* UDP_METRIC_PROFILE = "cafbcr-udp-unique-delay-v1";

  static std::string UdpMetricsHeader()
  {
    return "UDPMetricProfile,UDPDelaySamples,UDPDelaySum(ns),UDPMeanDelay(ms)";
  }

  // Reuse the passive ledger's first eligible delivery of each local attempt.
  // Both Tx and Rx are in [30s, simulation end); late deliveries after UDP stop
  // remain eligible. No flow weighting, duplicate samples or zero-loss imputation.
  static std::string UdpMetricsValues(const UdpObservationWriter::Counts& udp)
  {
    NS_ABORT_MSG_IF(udp.errors || (!udp.unique && udp.delayNs), "Invalid UDP delay ledger");
    std::ostringstream out;
    out << UDP_METRIC_PROFILE << ',' << udp.unique << ',' << udp.delayNs << ','
        << Ratio(udp.delayNs / 1e6, udp.unique);
    return out.str();
  }

  void SampleClustering(const NodeContainer& nodes)
  {
    const auto clusters = ClusterManager::GetInstance()->GetAllClusters();
    uint64_t members = 0, clustered = 0;
    for (const auto& [id, cluster] : clusters) members += cluster->GetAllMembers().size();
    for (uint32_t i = 0; i < nodes.GetN(); ++i) {
      auto cluster = nodes.Get(i)->GetObject<ClusterNode>();
      if (cluster && cluster->GetClusterId() != 0xffff) ++clustered;
    }
    ++m_samples;
    m_clusters += clusters.size();
    m_ratio += nodes.GetN() ? 100.0 * clustered / nodes.GetN() : 0;
    m_size += clusters.empty() ? 0 : static_cast<double>(members) / clusters.size();
  }

  void Append(const std::string& filename, const std::string& algorithm,
              const std::string& protocol,
              uint32_t nodes, double minSpeed, double maxSpeed, uint32_t seed,
              double duration, uint64_t run, const wireless::ChannelConfig& channel,
              const std::string& selection, const std::string& buildId, bool completeWindow,
              const PerformanceMetrics::CommonMetrics& net, Ptr<PerformanceMetrics> metrics,
              const TaskObservationWriter::Summary& task, const UdpObservationWriter::Counts& udp,
              bool fixedUdp, const InputConfig* inputs = nullptr,
              const FailureSummary* failures = nullptr,
              const RunDiagnostics* diagnostics = nullptr,
              const scenario::Contract* scenarioConfig = nullptr,
              bool udpMetrics = false)
  {
    NS_ABORT_MSG_IF(udp.errors, "UDP counters are invalid; no result row written");
    const auto path = std::filesystem::path(filename);
    if (!path.parent_path().empty()) std::filesystem::create_directories(path.parent_path());
    const bool headerNeeded = !std::filesystem::exists(path) || std::filesystem::file_size(path) == 0;
    const auto expectedHeader = (inputs ? ConfigurationHeader() : Header()) +
      (udpMetrics ? "," + UdpMetricsHeader() : "") +
      (failures ? "," + FailureSummary::CsvHeader() : "") +
      (diagnostics ? "," + RunDiagnostics::CsvHeader() : "") +
      (scenarioConfig ? "," + scenario::Contract::CsvHeader() : "");
    if (!headerNeeded) {
      std::ifstream existing(path);
      std::string header;
      std::getline(existing, header);
      NS_ABORT_MSG_IF(header != expectedHeader, "CSV schema differs; use a separate result filename");
    }
    std::ostringstream row;
    row << std::setprecision(12) << algorithm << ',' << nodes << ',' << minSpeed << '~' << maxSpeed
      << ',' << seed << ',' << Number(net.pdr, net.validFlows) << ','
      << Number(net.avgDelay, udp.unique) << ',' << Number(net.avgHops, udp.unique) << ','
      << net.validFlows << ',' << net.throughput << ',' << net.controlOverheadRatio << ','
      << net.ControlPacketTHP << ',' << Ratio(net.helloPacketCount, net.controlPacketNum) << ','
      << (net.AvergeLinkLength > 0 ? net.AvergeLinkLength : metrics->CalculateAverageLinkLifetime()) << ','
      << net.totalEnergyConsumed << ',' << net.avgEnergyPerNode << ',' << net.deadNodes << ',';
    // Classify the installed protocol, as in the sampling path. Display labels
    // may include independent ablations such as -noproxy or -norange.
    if (metrics->IsClusteringAlgorithm(protocol)) {
      uint32_t total;
      double lifetime, size;
      ClusterManager::GetInstance()->GetClusteringStatistics(total, lifetime, size);
      row << "Yes," << total << ',' << Ratio(m_clusters, m_samples) << ','
        << Ratio(m_ratio, m_samples) << ',' << Ratio(m_size, m_samples) << ',' << lifetime << ','
        << metrics->GetClusterSwitchRate() << ',' << metrics->GetRoleChangeRate();
    } else row << "No,N/A,N/A,N/A,N/A,N/A,N/A,N/A";
    const auto& t = task.tasks;
    const uint64_t acknowledged = t.onTime + t.late;
    const uint64_t denominator = fixedUdp ? udp.windowPlanned : udp.windowAttempts;
    row << ',' << t.generated << ',' << acknowledged << ',' << Ratio(100.0 * acknowledged, t.generated)
      << ',' << Ratio(1000.0 * t.responseSeconds, acknowledged)
      << ',' << Ratio(task.processingNs / 1e6, task.processingSamples)
      << ',' << Ratio(task.queueNs / 1e6, task.queueSamples)
      << ',' << Ratio(100.0 * (t.late + t.unconfirmedDue), t.generated)
      << ',' << Ratio(100.0 * t.onTime, t.generated) << ',' << t.onTime << ',' << t.late
      << ',' << t.unconfirmedDue << ',' << t.unconfirmedFuture << ',' << task.received << ',' << task.finished
      << ',' << denominator << ',' << udp.unique << ',' << Ratio(100.0 * udp.unique, denominator)
      << ',' << duration << ',' << run << ',' << channel.model << ',';
    if (channel.model == "lognormal") row << channel.pathLossExponent; else row << "N/A";
    row << ',' << (channel.model == "legacy" ? 0 : channel.sigmaDb) << ',' << selection << ',' << buildId
      << ',' << completeWindow;
    if (inputs) {
      row << ',' << inputs->taskStart << ',' << inputs->taskStop << ',' << inputs->taskStartMode
          << ',' << (inputs->udpStart >= 0 ? "explicit-window" : "legacy-time") << ',';
      if (inputs->udpStart >= 0) row << inputs->udpStart << ',' << inputs->udpStop;
      else row << "N/A,N/A";
      row << ',' << inputs->udpFlows << ",0.5,512," << inputs->controlMeter;
    }
    if (udpMetrics) row << ',' << UdpMetricsValues(udp);
    if (failures) row << ',' << failures->CsvValues();
    if (diagnostics) row << ',' << diagnostics->CsvValues();
    if (scenarioConfig) row << ',' << scenarioConfig->CsvValues();
    row << '\n';
    std::ofstream out(path, std::ios::app);
    NS_ABORT_MSG_IF(!out, "Cannot append result CSV: " << path);
    if (headerNeeded) out << expectedHeader << '\n';
    out << row.str();
    out.flush();
    NS_ABORT_MSG_IF(!out, "Result CSV write failed: " << path);
  }

private:
  static std::string Number(double value, uint64_t samples)
  {
    if (!samples) return "N/A";
    std::ostringstream out;
    out << std::setprecision(12) << value;
    return out.str();
  }
  static std::string Ratio(double value, uint64_t denominator)
  { return denominator ? Number(value / denominator, denominator) : "N/A"; }
  uint64_t m_samples = 0;
  double m_clusters = 0, m_ratio = 0, m_size = 0;
};
} // namespace ns3::cafbcr
#endif
