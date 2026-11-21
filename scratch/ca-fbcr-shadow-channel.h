/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef CA_FBCR_SHADOW_CHANNEL_H
#define CA_FBCR_SHADOW_CHANNEL_H

// CA experiment entrypoints only: no FANET/FBCR library or benchmarktest hooks.
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/propagation-delay-model.h"
#include "ca-fbcr-common-demand.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace ns3::cafbcr::wireless {

// The CA entrypoint reserves 2M for routing, 3M for tasks and 4M for UDP.
// Keep the shadow block above those explicit namespaces as well as avoiding
// the automatic allocator. v1 incorrectly reused UDP stream 4,000,000.
inline constexpr int64_t SHADOW_STREAM_BASE = 8000000;
inline constexpr double CHANNEL_FREQUENCY_HZ = 2.4e9;
inline constexpr double REFERENCE_DISTANCE_M = 1.0;

inline double ReferenceLossDb()
{
  // 2.4 GHz, unit antenna gains, unit system loss; never use the ns-3
  // LogDistance default reference loss, which is for 5.15 GHz.
  return 20*std::log10(4*std::acos(-1.0)*REFERENCE_DISTANCE_M*CHANNEL_FREQUENCY_HZ/299792458.0);
}

struct ChannelConfig
{
  std::string model = "legacy";
  double sigmaDb = 4.0; // Diagnostic value, not a fitted UAV parameter.
  double correlationDistance = 20.0;
  double updateSeconds = 0.1;
  double maxRange = -1.0; // Auto: legacy=600 m, shadow=no hard cutoff.
  double pathLossExponent = 2.0; // Physical free-space reference, not a UAV fit.
  int64_t streamBase = SHADOW_STREAM_BASE;
  uint32_t tracePairs = 0;
  bool pairingAudit = false; // Passive all-node/all-pair records, separate from physical configuration.
  // Explicit opt-in reference sampling for sigma=0. The default zero control
  // still has no pair RNGs/update events. This option is NOT a passive audit.
  bool pairingProcess = false;

  void AddOptions(CommandLine& cmd)
  {
    cmd.AddValue("channelModel", "CA channel [legacy|shadow|lognormal]; lognormal=close-in log-distance + correlated log-normal shadow", model);
    cmd.AddValue("channelMaxRange", "Hard PHY cutoff in m; -1=auto (legacy 600, shadow none), 0=none", maxRange);
    cmd.AddValue("shadowSigmaDb", "Log-normal shadow standard deviation in dB; 0 is the no-shadow control", sigmaDb);
    cmd.AddValue("shadowCorrDistance", "Shadow correlation distance in m for the sum of endpoint displacements", correlationDistance);
    cmd.AddValue("shadowUpdate", "Fixed shadow update interval in seconds (independent of packet times)", updateSeconds);
    cmd.AddValue("shadowStreamBase", "First explicit ns-3 stream for unordered node pairs; reserve a disjoint block", streamBase);
    cmd.AddValue("shadowTracePairs", "Write passive epoch traces for the first 0..16 unordered pairs", tracePairs);
    cmd.AddValue("pathLossExponent", "Close-in mean loss exponent for channelModel=lognormal; reference is 1 m at 2.4 GHz", pathLossExponent);
    cmd.AddValue("channelPairingAudit", "Passive all-node positions and all-pair shadow/innovation records at fixed epochs; default off", pairingAudit);
    cmd.AddValue("shadowPairingProcess", "Opt-in fixed-epoch reference process even for sigma=0; zero physical shadow, extra sampling/events explicitly recorded", pairingProcess);
  }
  double EffectiveRange() const { return maxRange == -1 ? (model == "legacy" ? 600 : 0) : maxRange; }
  std::string Validate() const
  {
    if (model != "legacy" && model != "shadow" && model != "lognormal") return "channelModel must be legacy, shadow or lognormal";
    if (!std::isfinite(pathLossExponent) || pathLossExponent < 1 || pathLossExponent > 6)
      return "pathLossExponent must be finite in [1,6]";
    if (model != "lognormal" && pathLossExponent != 2)
      return "pathLossExponent requires channelModel=lognormal";
    if (!std::isfinite(maxRange) || (maxRange < 0 && maxRange != -1)) return "channelMaxRange must be -1 or nonnegative";
    if (!std::isfinite(sigmaDb) || sigmaDb < 0 || sigmaDb > 20) return "shadowSigmaDb must be finite in [0,20] dB";
    if (!std::isfinite(correlationDistance) || correlationDistance <= 0) return "shadowCorrDistance must be finite and positive";
    if (!std::isfinite(updateSeconds) || updateSeconds < .001 || updateSeconds > 10)
      return "shadowUpdate must be finite in [0.001,10] seconds";
    if (streamBase < SHADOW_STREAM_BASE || streamBase > std::numeric_limits<int64_t>::max() - (1LL << 32))
      return "shadowStreamBase must be >=8000000 with room for the pair stream block";
    if (streamBase <= demand::UDP_STREAM ||
        streamBase <= demand::TaskStream(demand::MAX_NODES-1)+1)
      return "shadow stream block must be above the explicit task and UDP demand streams";
    if (tracePairs > 16) return "shadowTracePairs must be in [0,16]";
    if (pairingProcess && model == "legacy") return "shadowPairingProcess requires channelModel=shadow or lognormal";
    if (pairingAudit && (model == "legacy" || (sigmaDb == 0 && !pairingProcess)))
      return "channelPairingAudit requires a stochastic channel with positive shadowSigmaDb";
    if (model == "legacy" && (EffectiveRange() != 600 || sigmaDb != 4 ||
        correlationDistance != 20 || updateSeconds != .1 || streamBase != SHADOW_STREAM_BASE || tracePairs != 0))
      return "legacy preserves the original channel; shadow controls require channelModel=shadow or lognormal";
    return "";
  }
  std::string Json() const
  {
    std::ostringstream out; out << std::setprecision(17);
    out << "{\"schema\":\"cafbcr-lognormal-channel-v1\",\"model\":\"" << model
      << "\",\"mean_path_loss\":\"" << (model == "lognormal" ? "LogDistance" : "Friis")
      << "\",\"frequency_hz\":" << CHANNEL_FREQUENCY_HZ
      << ",\"path_loss_exponent\":";
    if (model == "lognormal") out << pathLossExponent; else out << "null";
    out << ",\"reference_distance_m\":";
    if (model == "lognormal") out << REFERENCE_DISTANCE_M; else out << "null";
    out << ",\"reference_loss_db\":";
    if (model == "lognormal") out << ReferenceLossDb(); else out << "null";
    out << ",\"below_reference_rule\":";
    if (model == "lognormal") out << "\"clamp_loss_to_reference_for_d_le_1m\""; else out << "null";
    out << ",\"delay\":\"ConstantSpeed\",\"hard_cutoff_m\":";
    if (EffectiveRange() == 0) out << "null"; else out << EffectiveRange();
    out << ",\"shadow_enabled\":" << (model != "legacy" && sigmaDb > 0 ? "true" : "false")
      << ",\"sigma_db\":" << (model != "legacy" ? sigmaDb : 0)
      << ",\"correlation_distance_m\":";
    if (model != "legacy") out << correlationDistance; else out << "null";
    out << ",\"update_seconds\":";
    if (model != "legacy") out << updateSeconds; else out << "null";
    out << ",\"pair_stream_base\":";
    if (model != "legacy") out << streamBase; else out << "null";
    out << ",\"reserved_udp_stream\":" << demand::UDP_STREAM
      << ",\"reserved_task_stream_begin\":" << demand::TASK_STREAM_BASE
      << ",\"reserved_task_stream_end\":" << demand::TaskStream(demand::MAX_NODES-1)+1
      << ",\"pair_stream_rule\":\"base+j*(j-1)/2+i_for_local_indices_i_lt_j\""
      << ",\"displacement\":\"sum_of_endpoint_3d_displacements_per_fixed_epoch\""
      << ",\"reciprocal_shadow\":true,\"cross_link_shadow_correlation\":false"
      << ",\"update_trigger\":\"fixed_epochs_not_packets\",\"trace_pair_limit\":" << tracePairs
      << ",\"parameter_status\":\"diagnostic_not_environment_calibrated\"";
    if (pairingProcess) out << ",\"pairing_process\":{\"schema\":\"cafbcr-zero-shadow-reference-v1\",\"enabled\":true,"
      << "\"zero_sigma_reference\":" << (sigmaDb == 0 ? "true" : "false")
      << ",\"physical_offset_rule\":\"sigma_times_reference_process;zero_sigma_exactly_zero\","
      << "\"sampling\":\"fixed_epochs_all_unordered_pairs\",\"passive_only\":false}";
    out << '}';
    return out.str();
  }
};

// Sample-and-hold Gaussian shadow process on fixed epochs. Each unordered
// pair owns an explicit RNG stream; all pairs update irrespective of traffic.
// X_0=sigma*Z; X_k=rho*X_(k-1)+sigma*sqrt(1-rho^2)*Z_k,
// rho=exp(-(distance moved by endpoint i + distance moved by endpoint j)/dc).
// This experimental link process is not a 3GPP scenario or static obstacle map:
// returning to a previously visited location need not restore the old value.
class CorrelatedShadowing : public PropagationLossModel
{
public:
  static TypeId GetTypeId()
  {
    static TypeId tid = TypeId("ns3::CafbcrCorrelatedShadowing")
      .SetParent<PropagationLossModel>().SetGroupName("Propagation")
      .AddConstructor<CorrelatedShadowing>();
    return tid;
  }
  void Configure(const NodeContainer& nodes, const ChannelConfig& cfg,
                 const std::filesystem::path& output = {})
  {
    if (m_configured) throw std::logic_error("shadow model cannot be reconfigured");
    const auto error = cfg.Validate();
    if (!error.empty() || cfg.model == "legacy") throw std::invalid_argument(error.empty() ? "stochastic channel model required" : error);
    m_configured = true; m_cfg = cfg;
    if (nodes.GetN() > 65536) throw std::invalid_argument("shadow pair stream block supports at most 65536 nodes");
    for (uint32_t n = 0; n < nodes.GetN(); ++n) {
      auto mobility = nodes.Get(n)->GetObject<MobilityModel>();
      if (!mobility || m_indices.count(PeekPointer(mobility))) throw std::invalid_argument("distinct node mobility models required");
      m_indices[PeekPointer(mobility)] = n;
      m_nodeIds.push_back(nodes.Get(n)->GetId());
      m_mobility.push_back(mobility); m_positions.push_back(mobility->GetPosition());
    }
    if (cfg.pairingAudit) {
      if (output.empty()) throw std::invalid_argument("pairing audit requires an output directory");
      for (const auto* name : {"channel-pairing-positions.csv", "channel-pairing-states.csv", "channel-pairing-summary.json"})
        if (std::filesystem::exists(output/name)) throw std::invalid_argument("preserve existing pairing audit artifacts");
      m_auditOutput = output;
      m_auditPositions.open(output/"channel-pairing-positions.csv");
      m_auditStates.open(output/"channel-pairing-states.csv");
      if (!m_auditPositions || !m_auditStates) throw std::runtime_error("cannot open pairing audit");
      m_auditPositions << std::setprecision(17) << "time_ns,local_index,node_id,x_m,y_m,z_m\n";
      m_auditStates << std::setprecision(17) << "time_ns,pair_index,displacement_m,rho,normal_innovation,shadow_db\n";
      AuditPositions(m_positions); // Positions have already been read by Configure.
    }
    if (cfg.tracePairs && !output.empty()) {
      m_trace.open(output / "shadow-epochs.csv");
      if (!m_trace) throw std::runtime_error("cannot open shadow trace");
      m_trace << std::setprecision(17) << "time_ns,pair_index,node_i,node_j,displacement_m,rho,shadow_db\n";
    }
    if (cfg.sigmaDb == 0 && !cfg.pairingProcess) return; // Preserve the original zero control.
    for (uint32_t j = 1; j < nodes.GetN(); ++j) {
      for (uint32_t i = 0; i < j; ++i) {
        Pair p; p.i = i; p.j = j;
        // Set Stream during construction. CreateObject()+SetStream() would
        // already consume one implicit stream and shift unrelated randomness.
        p.normal = CreateObjectWithAttributes<NormalRandomVariable>(
          "Stream", IntegerValue(cfg.streamBase + static_cast<int64_t>(m_pairs.size())));
        const double normal = p.normal->GetValue();
        p.value = cfg.sigmaDb * normal; ++m_draws;
        m_pairs.push_back(p); Record(m_pairs.size()-1, 0, 1, &normal);
      }
    }
    m_update = Simulator::Schedule(Seconds(cfg.updateSeconds), &CorrelatedShadowing::Update, this);
  }
  double Value(uint32_t i, uint32_t j) const
  {
    if (!m_configured || i >= m_mobility.size() || j >= m_mobility.size()) throw std::out_of_range("unknown shadow endpoint");
    if (i == j || m_cfg.sigmaDb == 0) return 0;
    if (i > j) std::swap(i,j);
    return m_pairs.at(static_cast<uint64_t>(j)*(j-1)/2+i).value;
  }
  uint64_t Signature() const { return m_signature; }
  uint64_t Epochs() const { return m_epochs; }
  void Finish(const std::filesystem::path& output)
  {
    Simulator::Cancel(m_update);
    if (m_trace.is_open()) { m_trace.flush(); if (!m_trace) throw std::runtime_error("shadow trace write failed"); }
    std::ofstream out(output / "channel-summary.json");
    out << std::setprecision(17) << "{\"configuration\":" << m_cfg.Json()
      << ",\"nodes\":" << m_mobility.size() << ",\"initialized_pairs\":" << m_pairs.size()
      << ",\"completed_update_epochs\":" << m_epochs << ",\"state_records\":" << m_records
      << ",\"state_signature_fnv64\":\"" << std::hex << m_signature << std::dec
      << "\",\"normal_draws\":" << m_draws << ",\"packet_queries\":" << m_queries
      << ",\"shadow_db_mean_across_pair_epochs\":";
    if (m_records) out << static_cast<double>(m_sum/m_records); else out << "null";
    out << ",\"shadow_db_sd_across_pair_epochs\":";
    if (m_records) out << std::sqrt(std::max(0.0L,m_sumSq/m_records-(m_sum/m_records)*(m_sum/m_records)));
    else out << "null";
    if (m_cfg.pairingProcess) {
      const bool zero = m_cfg.sigmaDb == 0;
      out << ",\"zero_sigma_reference_overhead_vs_default\":{\"position_queries\":"
        << (zero ? m_mobility.size()*m_epochs : 0) << ",\"normal_draws\":" << (zero ? m_draws : 0)
        << ",\"scheduled_updates\":" << (zero ? m_epochs+1 : 0) << ",\"completed_updates\":" << (zero ? m_epochs : 0) << '}';
    }
    out << ",\"formal_eligible\":false}\n";
    if (!out) throw std::runtime_error("shadow summary write failed");
    if (m_cfg.pairingAudit) {
      if (output != m_auditOutput) throw std::invalid_argument("pairing audit output directory changed");
      m_auditPositions.flush(); m_auditStates.flush();
      if (!m_auditPositions || !m_auditStates) throw std::runtime_error("pairing audit write failed");
      const bool valid = m_auditPositionRecords == m_mobility.size()*(m_epochs+1) &&
        m_auditStateRecords == m_records && m_auditNormalDraws == m_draws;
      std::ofstream audit(output/"channel-pairing-summary.json");
      audit << "{\"schema\":\"cafbcr-channel-pairing-audit-v1\",\"formal_eligible\":false,"
        << "\"enabled\":true,\"measurement_valid\":" << (valid ? "true" : "false")
        << ",\"nodes\":" << m_mobility.size() << ",\"pairs\":" << m_pairs.size()
        << ",\"completed_update_epochs\":" << m_epochs
        << ",\"position_records\":" << m_auditPositionRecords
        << ",\"state_records\":" << m_auditStateRecords << ",\"normal_draws\":" << m_auditNormalDraws
        << ",\"capture_end_ns\":" << Simulator::Now().GetNanoSeconds()
        << ",\"scope\":\"initial_and_completed_fixed_epochs;not_arbitrary_packet_times\","
        << "\"new_position_queries\":0,\"new_random_draws\":0,\"new_scheduled_events\":0";
      if (m_cfg.pairingProcess) audit << ",\"audit_overhead_reference\":\"configured_reference_process\","
        << "\"zero_sigma_reference_process\":" << (m_cfg.sigmaDb == 0 ? "true" : "false");
      audit << "}\n";
      if (!audit) throw std::runtime_error("pairing audit summary write failed");
    }
  }
protected:
  int64_t DoAssignStreams(int64_t) override
  { throw std::logic_error("pair streams are fixed by Configure; do not reassign channel streams"); }
  void DoDispose() override
  {
    Simulator::Cancel(m_update); m_pairs.clear(); m_mobility.clear(); m_indices.clear();
    PropagationLossModel::DoDispose();
  }
private:
  struct Pair { uint32_t i = 0, j = 0; double value = 0; Ptr<NormalRandomVariable> normal; };
  double DoCalcRxPower(double dbm, Ptr<MobilityModel> a, Ptr<MobilityModel> b) const override
  {
    ++m_queries;
    return dbm + Value(m_indices.at(PeekPointer(a)), m_indices.at(PeekPointer(b)));
  }
  void AuditPositions(const std::vector<Vector>& positions)
  {
    for (size_t n = 0; n < positions.size(); ++n) {
      const auto& p = positions[n];
      m_auditPositions << Simulator::Now().GetNanoSeconds() << ',' << n << ',' << m_nodeIds[n]
        << ',' << p.x << ',' << p.y << ',' << p.z << '\n';
      ++m_auditPositionRecords;
    }
  }
  void Record(size_t index, double distance, double rho, const double* normal)
  {
    const auto& p = m_pairs[index]; ++m_records;
    m_sum += p.value; m_sumSq += p.value*p.value;
    // Diagnostic signature only; the external manifest supplies SHA-256.
    uint64_t bits; std::memcpy(&bits, &p.value, sizeof(bits));
    for (unsigned k = 0; k < 8; ++k) { m_signature ^= (bits >> (8*k)) & 255; m_signature *= 1099511628211ULL; }
    if (m_trace.is_open() && index < m_cfg.tracePairs)
      m_trace << Simulator::Now().GetNanoSeconds() << ',' << index << ',' << p.i << ',' << p.j
        << ',' << distance << ',' << rho << ',' << p.value << '\n';
    if (m_cfg.pairingAudit) {
      m_auditStates << Simulator::Now().GetNanoSeconds() << ',' << index << ',' << distance << ',' << rho << ',';
      if (normal) { m_auditStates << *normal; ++m_auditNormalDraws; }
      else m_auditStates << "null";
      m_auditStates << ',' << p.value << '\n'; ++m_auditStateRecords;
    }
  }
  void Update()
  {
    std::vector<Vector> next; std::vector<double> distances;
    for (size_t n = 0; n < m_mobility.size(); ++n) {
      next.push_back(m_mobility[n]->GetPosition());
      distances.push_back(CalculateDistance(next.back(), m_positions[n]));
    }
    if (m_cfg.pairingAudit) AuditPositions(next); // Reuse the original update's positions.
    ++m_epochs;
    for (size_t k = 0; k < m_pairs.size(); ++k) {
      auto& p = m_pairs[k]; double d = distances[p.i] + distances[p.j];
      double rho = std::exp(-d/m_cfg.correlationDistance);
      double normal = 0;
      if (d > 0) {
        // expm1 avoids cancellation in 1-exp(-2d/dc) for tiny displacements.
        normal = p.normal->GetValue();
        p.value = rho*p.value + m_cfg.sigmaDb*std::sqrt(-std::expm1(-2*d/m_cfg.correlationDistance))*normal;
        ++m_draws;
      }
      Record(k,d,rho,d > 0 ? &normal : nullptr);
    }
    m_positions = std::move(next);
    m_update = Simulator::Schedule(Seconds(m_cfg.updateSeconds), &CorrelatedShadowing::Update, this);
  }
  ChannelConfig m_cfg;
  bool m_configured = false;
  std::vector<Ptr<MobilityModel>> m_mobility;
  std::vector<uint32_t> m_nodeIds;
  std::vector<Vector> m_positions;
  std::map<const MobilityModel*,uint32_t> m_indices;
  std::vector<Pair> m_pairs;
  EventId m_update;
  std::ofstream m_trace;
  std::ofstream m_auditPositions, m_auditStates;
  std::filesystem::path m_auditOutput;
  uint64_t m_auditPositionRecords = 0, m_auditStateRecords = 0, m_auditNormalDraws = 0;
  uint64_t m_epochs = 0, m_records = 0, m_draws = 0, m_signature = 14695981039346656037ULL;
  long double m_sum = 0, m_sumSq = 0;
  mutable uint64_t m_queries = 0;
};

inline Ptr<PropagationLossModel> CreateMeanLoss(const ChannelConfig& cfg)
{
  const auto error = cfg.Validate(); if (!error.empty()) throw std::invalid_argument(error);
  if (cfg.model == "lognormal") {
    auto closeIn = CreateObject<LogDistancePropagationLossModel>();
    closeIn->SetPathLossExponent(cfg.pathLossExponent);
    closeIn->SetReference(REFERENCE_DISTANCE_M, ReferenceLossDb());
    return closeIn;
  }
  return CreateObjectWithAttributes<FriisPropagationLossModel>("Frequency", DoubleValue(2.4e9));
}

inline Ptr<YansWifiChannel> CreateChannel(const NodeContainer& nodes, const ChannelConfig& cfg,
                                        Ptr<CorrelatedShadowing>& shadow,
                                        const std::filesystem::path& output = {})
{
  const auto error = cfg.Validate(); if (!error.empty()) throw std::invalid_argument(error);
  if (cfg.model == "legacy") {
    // Preserve the original helper construction order and attributes exactly.
    YansWifiChannelHelper original;
    original.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    original.AddPropagationLoss("ns3::FriisPropagationLossModel", "Frequency", DoubleValue(2.4e9));
    original.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange", DoubleValue(600.0));
    return original.Create();
  }
  auto channel = CreateObject<YansWifiChannel>();
  auto mean = CreateMeanLoss(cfg);
  shadow = CreateObject<CorrelatedShadowing>(); shadow->Configure(nodes,cfg,output);
  mean->SetNext(shadow);
  if (cfg.EffectiveRange() > 0)
    shadow->SetNext(CreateObjectWithAttributes<RangePropagationLossModel>("MaxRange", DoubleValue(cfg.EffectiveRange())));
  channel->SetPropagationLossModel(mean);
  channel->SetPropagationDelayModel(CreateObject<ConstantSpeedPropagationDelayModel>());
  return channel;
}

inline void FinishChannel(const ChannelConfig& cfg, Ptr<CorrelatedShadowing> shadow,
                          const std::filesystem::path& output)
{
  if (shadow) { shadow->Finish(output); return; }
  std::ofstream out(output / "channel-summary.json");
  out << "{\"configuration\":" << cfg.Json() << ",\"initialized_pairs\":0,\"normal_draws\":0,\"formal_eligible\":false}\n";
  if (!out) throw std::runtime_error("channel summary write failed");
}
} // namespace ns3::cafbcr::wireless
#endif
