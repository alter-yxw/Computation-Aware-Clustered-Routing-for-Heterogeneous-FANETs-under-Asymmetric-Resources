/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef CA_FBCR_COMPARISON_CONFIG_H
#define CA_FBCR_COMPARISON_CONFIG_H

#include "ca-fbcr-task-observer.h"
#include "ns3/cafbcr-scenario-config.h"
#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/uinteger.h"
#include <cmath>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace ns3::cafbcr::comparison {
inline constexpr const char* SCHEMA = "cafbcr-comparison-v1";
inline constexpr const char* ASSIGNMENT_SCHEMA = "cafbcr-server-assignment-v1";

// Use configured server membership/type only. No route, neighbor, position, queue or RNG.
// Prefer a non-self COMMAND/RELAY server; otherwise any non-self server, then self.
inline std::vector<std::optional<uint32_t>> FixedAssignments(const std::vector<UavTypeConfig>& nodes)
{
  std::vector<uint32_t> high, all;
  for (uint32_t i = 0; i < nodes.size(); ++i) {
    if (!nodes[i].processTasks || !std::isfinite(nodes[i].computeCapacity) || nodes[i].computeCapacity <= 0)
      continue;
    all.push_back(i);
    if (nodes[i].nodeType == COMMAND || nodes[i].nodeType == RELAY) high.push_back(i);
  }
  std::vector<std::optional<uint32_t>> result(nodes.size());
  uint32_t round = 0;
  for (uint32_t i = 0; i < nodes.size(); ++i) {
    if (!nodes[i].generateTasks) continue;
    std::vector<uint32_t> candidates;
    for (auto server : high) if (server != i) candidates.push_back(server);
    if (candidates.empty()) for (auto server : all) if (server != i) candidates.push_back(server);
    if (!candidates.empty()) result[i] = candidates[round % candidates.size()];
    else if (nodes[i].processTasks && std::isfinite(nodes[i].computeCapacity) && nodes[i].computeCapacity > 0)
      result[i] = i;
    ++round;
  }
  return result;
}
} // namespace ns3::cafbcr::comparison
#endif
