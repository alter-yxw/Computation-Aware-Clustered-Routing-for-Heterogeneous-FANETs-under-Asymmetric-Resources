/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef CA_FBCR_COMMON_DEMAND_H
#define CA_FBCR_COMMON_DEMAND_H

#include "ns3/nstime.h"
#include "ns3/random-variable-stream.h"
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace ns3::cafbcr::demand {

inline constexpr const char* SCHEMA = "cafbcr-common-demand-v1";
// CA protocol streams start at 2,000,000; these ranges do not overlap for MAX_NODES.
inline constexpr int64_t TASK_STREAM_BASE = 3000000;
inline constexpr int64_t UDP_STREAM = 4000000;
inline constexpr int64_t NODE_PHASE_NS = 100000000;
// The scenario uses one /16 IPv4 subnet (network and broadcast are reserved).
inline constexpr uint32_t MAX_NODES = 65534;
inline constexpr uint32_t MAX_UDP_FLOWS = 56535; // ports 9000..65535 excluding 9999

inline int64_t TaskStream(uint32_t nodeIndex)
{
  return TASK_STREAM_BASE + 2LL * nodeIndex;
}

inline Time TaskStart(Time generationStart, uint32_t nodeIndex)
{
  return generationStart + NanoSeconds(NODE_PHASE_NS * nodeIndex);
}

// Spread all task sources across [start, start + 1 s), regardless of network
// size. Integer arithmetic and no RNG draws keep the schedule common to every
// protocol. The legacy TaskStart and UDP flow staggering remain separate.
inline Time BoundedTaskStart(Time generationStart, uint32_t nodeIndex, uint32_t nodes)
{
  if (nodes == 0 || nodes > MAX_NODES || nodeIndex >= nodes)
    throw std::invalid_argument("Invalid node index/count for bounded task starts");
  return generationStart + NanoSeconds((1000000000LL * nodeIndex) / nodes);
}

inline std::string Validate(uint32_t nodes, uint32_t flows)
{
  if (nodes == 0 || nodes > MAX_NODES) return "Node count must be in [1, 65534] for this scenario";
  if (flows > uint64_t(nodes) * (nodes - 1)) return "UDP flow count exceeds N*(N-1) distinct directed pairs";
  if (flows > MAX_UDP_FLOWS) return "UDP flow count exceeds the available ports (9999 is reserved for compute)";
  return {};
}

inline uint16_t UdpPort(uint32_t flowIndex)
{
  if (flowIndex >= MAX_UDP_FLOWS) throw std::invalid_argument("UDP flow port index out of range");
  return 9000 + flowIndex + (flowIndex >= 999 ? 1 : 0);
}

// Partial Fisher-Yates on a virtual array of N*(N-1) ranks. Exactly one integer
// draw per selected pair, O(flows) memory, no retry loop even for a full small-N set.
inline std::vector<std::pair<uint16_t, uint16_t>> UdpPairs(
    uint32_t nodes, uint32_t flows, Ptr<UniformRandomVariable> random)
{
  auto error = Validate(nodes, flows);
  if (!error.empty()) throw std::invalid_argument(error);
  const uint32_t count = uint64_t(nodes) * (nodes - 1);
  std::map<uint32_t, uint32_t> swapped;
  auto value = [&swapped](uint32_t rank) {
    auto it = swapped.find(rank);
    return it == swapped.end() ? rank : it->second;
  };
  std::vector<std::pair<uint16_t, uint16_t>> pairs;
  pairs.reserve(flows);
  for (uint32_t i = 0; i < flows; ++i) {
    uint32_t selected = random->GetInteger(i, count - 1);
    uint32_t rank = value(selected);
    swapped[selected] = value(i);
    swapped.erase(i);
    uint32_t source = rank / (nodes - 1);
    uint32_t destination = rank % (nodes - 1);
    if (destination >= source) ++destination;
    pairs.emplace_back(source, destination);
  }
  return pairs;
}

} // namespace ns3::cafbcr::demand
#endif
