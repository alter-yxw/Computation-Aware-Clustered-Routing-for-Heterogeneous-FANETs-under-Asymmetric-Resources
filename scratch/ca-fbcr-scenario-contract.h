/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef CA_FBCR_SCENARIO_CONTRACT_H
#define CA_FBCR_SCENARIO_CONTRACT_H

#include <array>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>

namespace ns3::cafbcr::scenario {

// CA-entry configuration only. Nominal ranges are protocol inputs, not PHY cutoffs.
struct Radio {
  double txDbm, rxDbm, range;
  std::string dataMode;
};

inline std::string Validate(double minimum, double maximum, int level, const std::string& ranges)
{
  if (!std::isfinite(minimum) || !std::isfinite(maximum) || minimum < 0 || maximum < minimum)
    return "Require finite 0 <= MinSpeed <= MaxSpeed (SWARM mean-velocity distribution)";
  if (level < -1 || level > 4) return "asymLevel must be -1 or 0..4; L4 matches the current default";
  if (ranges != "linked" && ranges != "fixed-default")
    return "asymRangeProfile must be linked or fixed-default";
  return {};
}

inline Radio Resolve(unsigned type, int level, const std::string& ranges, double defaultRange)
{
  if (type >= 4 || !Validate(0, 0, level, ranges).empty())
    throw std::invalid_argument("Invalid CA radio profile");
  static const std::array<double,4> tx{22,20,16,12}, rx{-85,-82,-80,-80};
  static const std::array<const char*,4> modes{
    "ErpOfdmRate54Mbps","ErpOfdmRate24Mbps","ErpOfdmRate6Mbps","ErpOfdmRate6Mbps"};
  Radio result{tx[type],rx[type],defaultRange,modes[type]};
  if (level >= 0) {
    static const std::array<double,5> taskTx{20,19,18,17,16}, swarmTx{20,18,16,14,12};
    static const std::array<double,5> taskRange{480,460,440,420,400}, swarmRange{480,420,360,305,250};
    if (type == 2) result.txDbm = taskTx[level];
    if (type == 3) result.txDbm = swarmTx[level];
    if (ranges == "linked") {
      if (type == 2) result.range = taskRange[level];
      if (type == 3) result.range = swarmRange[level];
    }
  }
  return result;
}

class Contract {
public:
  static constexpr const char* SCHEMA = "cafbcr-scenario-v1";
  template<class T> void Set(const std::string& name, const T& value)
  {
    std::ostringstream out; out << std::setprecision(17) << value;
    m_fields[name] = out.str();
  }
  static std::string Quote(const std::string& value)
  {
    std::ostringstream out; out << '"';
    for (unsigned char c : value) {
      if (c == '"' || c == '\\') out << '\\' << c;
      else if (c < 32) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << unsigned(c) << std::dec;
      else out << c;
    }
    out << '"'; return out.str();
  }
  std::string Json() const
  {
    std::ostringstream out; out << '{'; bool first = true;
    for (const auto& [key,value] : m_fields) {
      if (!first) out << ',';
      first = false; out << Quote(key) << ':' << Quote(value);
    }
    out << '}'; return out.str();
  }
  std::string Id() const
  {
    // Deterministic configuration identity, not a cryptographic source/build hash.
    // The CA entry records the effective arpDeadReprobe, arpMaxRetries and
    // arpWaitReplyMs values in this field set. Omitted flags and explicit
    // defaults therefore share one id; any non-default value changes it.
    // Ids written before these fields existed are historical v1 ids and are
    // not equivalent to the new default id; do not resume them as new runs.
    uint64_t hash = 14695981039346656037ULL;
    for (unsigned char c : Json()) { hash ^= c; hash *= 1099511628211ULL; }
    std::ostringstream out; out << SCHEMA << '-' << std::hex << std::setw(16) << std::setfill('0') << hash;
    return out.str();
  }
  static std::string CsvHeader()
  {
    return "ScenarioSchema,ScenarioId,MinSpeed(m/s),MaxSpeed(m/s),AsymLevel,AsymRangeProfile,ScenarioConfig";
  }
  std::string CsvValues() const
  {
    auto json = Json(); std::string escaped;
    for (char c : json) { escaped += c; if (c == '"') escaped += c; }
    return std::string(SCHEMA) + ',' + Id() + ',' + m_fields.at("MinSpeed(m/s)") + ',' +
      m_fields.at("MaxSpeed(m/s)") + ',' + m_fields.at("AsymLevel") + ',' +
      m_fields.at("AsymRangeProfile") + ",\"" + escaped + '"';
  }
  std::string Preview() const
  { return "{\"schema\":" + Quote(SCHEMA) + ",\"id\":" + Quote(Id()) + ",\"config\":" + Json() + '}'; }
private:
  std::map<std::string,std::string> m_fields;
};
} // namespace ns3::cafbcr::scenario
#endif
