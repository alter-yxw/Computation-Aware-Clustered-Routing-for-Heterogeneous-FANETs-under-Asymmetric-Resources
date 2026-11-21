/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef CA_FBCR_MOGNN_ADAPTER_H
#define CA_FBCR_MOGNN_ADAPTER_H

#include "ca-fbcr-task-observer.h"
#include "ns3/mo-routing.h"
#include "ns3/cafbcr-scenario-config.h"
#include "ns3/ipv4-interface-container.h"
#include "ns3/node-container.h"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>

namespace ns3::cafbcr {

// Private CA scenario bridge. Imported MO policy/routing and shared Wi-Fi,
// energy, CPU executor, task generation, wire format and ACK rules stay intact.
class MognnAdapter
{
public:
  struct Settings {
    std::string model, relayMode = "legacy";
    double slotSeconds = .01, linkBps = 6e6, helloSeconds = 1, holdSeconds = 4;
    double relayBatchWindowMs = 20;
    bool relaySuppressDuplicates = false;
    double serverHelloScale = 1.0;
  };

  static bool Supports(const std::string& protocol)
  {
    return protocol == "mognn" || protocol == "mognn-baseline" || protocol == "mognn-local";
  }

  void Configure(Settings settings, const std::vector<UavTypeConfig>& configs,
                 const std::string& mode, const std::string& selection, double horizonSeconds,
                 bool computeEnabled)
  {
    if (!Supports(mode) || configs.empty() || configs.size() > 4096 ||
        (settings.relayMode != "legacy" && settings.relayMode != "batch-v1") ||
        (settings.relayMode == "batch-v1" && configs.size() > mo::RelayBatchHeader::MAX_NODES) ||
        (settings.relayMode == "batch-v1" &&
         (!std::isfinite(settings.relayBatchWindowMs) ||
          settings.relayBatchWindowMs < 1 || settings.relayBatchWindowMs > 1000)) ||
        (settings.relayMode != "batch-v1" &&
         (settings.relaySuppressDuplicates || settings.relayBatchWindowMs != 20)) ||
        !std::isfinite(settings.serverHelloScale) ||
        settings.serverHelloScale < 0.1 || settings.serverHelloScale > 4.0 ||
        (selection != "fixed" && selection != "native") || (mode == "mognn-local" && selection != "native") ||
        !std::isfinite(settings.slotSeconds) || settings.slotSeconds <= 0 ||
        !std::isfinite(settings.linkBps) || settings.linkBps <= 0 ||
        !std::isfinite(settings.helloSeconds) || settings.helloSeconds < .05 ||
        !std::isfinite(settings.holdSeconds) || settings.holdSeconds < .1 || settings.holdSeconds <= settings.helloSeconds ||
        !std::isfinite(horizonSeconds) || horizonSeconds <= 0 ||
        !std::isfinite(horizonSeconds / settings.slotSeconds) ||
        !std::isfinite(settings.linkBps / (8 * REFERENCE_BYTES) * settings.slotSeconds) ||
        settings.holdSeconds >= Time::Max().GetSeconds())
      throw std::invalid_argument("Invalid MOGNN settings, horizon, node count or task selection");
    m_settings = std::move(settings); m_mode = mode; m_selection = selection;
    m_configs = configs; m_horizon = horizonSeconds / m_settings.slotSeconds; m_computeEnabled = computeEnabled;
    m_policy = std::make_shared<mo::Policy>();
    if (mode == "mognn") {
      if (m_settings.model.empty()) m_settings.model = "contrib/mognn/models/BAT800.txt";
      m_settings.model = std::filesystem::absolute(m_settings.model).string();
      m_policy->Load(m_settings.model); // strict dimensions/K/finite data; never fall back
      std::ifstream in(m_settings.model, std::ios::binary);
      if (!in) throw std::invalid_argument("Cannot read MOGNN model identity");
      uint64_t value = 14695981039346656037ULL;
      char c;
      while (in.get(c)) { value = (value ^ static_cast<unsigned char>(c)) * 1099511628211ULL; ++m_modelBytes; }
      if (!in.eof()) throw std::invalid_argument("Cannot finish MOGNN model identity");
      std::ostringstream hash; hash << std::hex << std::setw(16) << std::setfill('0') << value;
      m_modelFnv64 = hash.str();
    } else if (!m_settings.model.empty()) {
      throw std::invalid_argument("Only learned MOGNN loads moModel; baseline/local must not claim weights");
    }
    // A public configured unit conversion, not another node's observed queue.
    // GFLOP/s divided by GFLOP/reference-packet is packets/s; the same ratio
    // results if both numerator and denominator are first converted to cycles.
    double workRate = 0, packetRate = 0;
    for (const auto& c : configs) if (computeEnabled && c.generateTasks) {
      workRate += (c.minTaskWorkload + c.maxTaskWorkload) / 2 / c.taskGenerationInterval;
      packetRate += (c.minTaskDataSize + c.maxTaskDataSize) / 2.0 / REFERENCE_BYTES / c.taskGenerationInterval;
    }
    m_referenceGflop = packetRate > 0 ? workRate / packetRate : 1;
    if (!std::isfinite(m_referenceGflop) || m_referenceGflop <= 0) throw std::invalid_argument("Invalid MOGNN CPU unit");
  }

  const Settings& GetSettings() const { return m_settings; }

  std::string ProfileJson() const
  {
    std::ostringstream out;
    out << std::setprecision(17) << "{\"schema\":\"mognn-cafbcr-adapter-v1\",\"mode\":" << TaskObservationWriter::Quote(m_mode)
        << ",\"task_choice\":" << TaskObservationWriter::Quote(m_selection == "fixed" ? "fixed-common-destination-control" : "native-local-or-server")
        << ",\"upstream_commit\":\"524bc4ef6af2a2a8c238b91470d10d2aeda14915\""
        << ",\"model_fnv1a64\":" << (m_modelFnv64.empty() ? "null" : TaskObservationWriter::Quote(m_modelFnv64))
        << ",\"model_bytes\":" << m_modelBytes << ",\"model_identity_scope\":\"actual-file-fnv64; external-handoff-also-freezes-sha256\""
        << ",\"reference_bytes\":" << REFERENCE_BYTES << ",\"reference_gflop_per_packet\":" << m_referenceGflop
        << ",\"slot_seconds\":" << m_settings.slotSeconds << ",\"link_bps_estimate\":" << m_settings.linkBps
        << ",\"horizon_slots\":" << m_horizon << ",\"hello_seconds\":" << m_settings.helloSeconds
        << ",\"hold_seconds\":" << m_settings.holdSeconds << ",\"cache_seconds\":0.25,\"result_payload_bytes\":8"
        << ",\"relay_mode\":" << TaskObservationWriter::Quote(m_settings.relayMode)
        << ",\"relay_batch_window_ms\":" << (m_settings.relayMode == "batch-v1" ? m_settings.relayBatchWindowMs : 0)
        << (m_settings.relayMode == "batch-v1" && m_settings.relaySuppressDuplicates
                ? ",\"relay_duplicate_suppression\":true" : "")
        << ",\"relay_batch_max_payload_bytes\":" << (m_settings.relayMode == "batch-v1" ? mo::RelayBatchHeader::MAX_BYTES : 0)
        << ",\"relay_max_nodes\":" << (m_settings.relayMode == "batch-v1" ? mo::RelayBatchHeader::MAX_NODES : 4096)
        << ",\"relay_original_stamp_preserved\":true"
        << ",\"executor\":\"ca-native-header-v1-fifo-source-ack\",\"online_information\":\"own-configuration-and-radio-received-descriptors\""
        << ",\"cpu_energy_added\":false,\"simulated_inference_delay_added\":false,\"flood_adaptation\":true"
        << ",\"paper_numerical_reproduction\":false"
        << (m_settings.serverHelloScale != 1.0
                ? ",\"server_hello_scale\":" + [&] {
                      std::ostringstream s;
                      s << std::setprecision(17) << m_settings.serverHelloScale;
                      return s.str();
                  }()
                : "")
        << "}";
    return out.str();
  }

  void Install(const NodeContainer& nodes, const Ipv4InterfaceContainer& interfaces)
  {
    if (nodes.GetN() != m_configs.size() || !m_policy) throw std::logic_error("MOGNN installation differs");
    for (uint32_t i = 0; i < nodes.GetN(); ++i) {
      auto route = nodes.Get(i)->GetObject<mo::RoutingProtocol>();
      if (!route) throw std::logic_error("MOGNN routing is not installed");
      const auto& c = m_configs[i];
      mo::Vertex vertex;
      vertex.rate = c.processTasks ? c.computeCapacity / m_referenceGflop * m_settings.slotSeconds : 0;
      vertex.arrival = m_computeEnabled && c.generateTasks ?
          (c.minTaskDataSize + c.maxTaskDataSize) / 2.0 / REFERENCE_BYTES / c.taskGenerationInterval * m_settings.slotSeconds : 0;
      vertex.server = c.processTasks && (c.nodeType == COMMAND || c.nodeType == RELAY);
      route->SetAttribute("RelayBatchWindow", TimeValue(MilliSeconds(m_settings.relayBatchWindowMs)));
      route->SetAttribute("SuppressDuplicateRelays", BooleanValue(m_settings.relaySuppressDuplicates));
      route->SetAttribute("ServerHelloScale", DoubleValue(m_settings.serverHelloScale));
      route->Configure(i, nodes.GetN(), vertex, m_policy, m_mode, m_settings.slotSeconds, m_horizon,
                       m_settings.linkBps / (8 * REFERENCE_BYTES) * m_settings.slotSeconds, m_settings.relayMode);
      m_addresses.emplace(nodes.Get(i)->GetId(), interfaces.GetAddress(i));
    }
  }

  void BindClient(Ptr<Node> node, Ptr<ComputeTaskClientApp> client)
  {
    m_clients.emplace(node->GetId(), client);
    if (m_selection == "native") {
      // Avoid the CA neighbor selector entirely. MO selects from its received
      // graph; the own address is only the initial draft before that choice.
      client->SetServerSelectionPolicy(ComputeTaskClientApp::ServerSelectionPolicy::FIXED_ASSIGNMENT);
      client->SetRemoteAddress(m_addresses.at(node->GetId()));
      auto route = node->GetObject<mo::RoutingProtocol>();
      client->SetTaskDestinationSelector([route](const ComputeTaskRecord& task) {
        return route->SelectDestination(task.taskId, task.dataSize / REFERENCE_BYTES, 8.0 / REFERENCE_BYTES);
      }, m_mode + "-native-offload");
    }
  }

  std::string SummaryJson(const TaskObservationWriter& observation, const std::string& build,
                          uint32_t seed, uint64_t run, Time cutoff) const
  {
    struct Counts { uint64_t generated = 0, received = 0, finished = 0, on = 0, late = 0, due = 0, pending = 0; };
    Counts counts[2];
    observation.VisitFinalTasks([&](uint32_t source, uint32_t taskId, bool received, bool finished, bool ack, const std::string&) {
      const auto& task = m_clients.at(source)->GetTaskRecords().at(taskId);
      auto& c = counts[task.selectedServer == m_addresses.at(source) ? 0 : 1];
      ++c.generated; c.received += received; c.finished += finished;
      if (ack) {
        if (*task.acknowledgedAt <= task.generatedAt + task.deadline) ++c.on; else ++c.late;
      } else if (task.generatedAt + task.deadline <= cutoff) ++c.due;
      else ++c.pending;
    });
    std::ostringstream out;
    out << "{\"schema\":\"mognn-cafbcr-task-partition-v1\",\"build_id\":" << TaskObservationWriter::Quote(build)
        << ",\"seed\":" << seed << ",\"rng_run\":" << run << ",\"profile\":" << ProfileJson();
    for (int i = 0; i < 2; ++i) {
      const auto& c = counts[i];
      out << ",\"" << (i ? "remote" : "local") << "\":{\"generated\":" << c.generated
          << ",\"received\":" << c.received << ",\"processed\":" << c.finished
          << ",\"confirmed\":" << c.on + c.late << ",\"on_time\":" << c.on << ",\"late\":" << c.late
          << ",\"unconfirmed_due\":" << c.due << ",\"pending\":" << c.pending << '}';
    }
    out << '}'; return out.str();
  }

private:
  static constexpr double REFERENCE_BYTES = 900;
  Settings m_settings;
  std::string m_mode, m_selection, m_modelFnv64;
  std::vector<UavTypeConfig> m_configs;
  std::shared_ptr<mo::Policy> m_policy;
  std::map<uint32_t, Ipv4Address> m_addresses;
  std::map<uint32_t, Ptr<ComputeTaskClientApp>> m_clients;
  double m_referenceGflop = 1, m_horizon = 0;
  bool m_computeEnabled = true;
  uint64_t m_modelBytes = 0;
};

} // namespace ns3::cafbcr
#endif
