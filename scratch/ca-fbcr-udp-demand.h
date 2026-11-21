/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef CA_FBCR_UDP_DEMAND_H
#define CA_FBCR_UDP_DEMAND_H

#include "ns3/nstime.h"
#include <algorithm>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace ns3::cafbcr::udp_demand {

inline constexpr const char* SCHEMA = "cafbcr-fixed-udp-demand-v1";

struct Plan {
  Time stop;
  uint64_t attempts;
};

// UdpClient MaxPackets=0 disables its success cap. A pre-installed stop event
// cancels the periodic Send at the exclusive end, independently of Send result.
// Keep the existing start, interval, packet size and nominal packet budget.
inline Plan FixedPlan(Time start, Time latestStop, Time interval, uint32_t budget)
{
  const int64_t first = start.GetNanoSeconds();
  const int64_t last = latestStop.GetNanoSeconds();
  const int64_t step = interval.GetNanoSeconds();
  if (first < 0 || last < 0 || step <= 0) throw std::invalid_argument("Invalid UDP demand times");
  if (last <= first) return {latestStop, 0};
  const uint64_t available = 1 + static_cast<uint64_t>((last - first - 1) / step);
  const uint64_t attempts = std::min<uint64_t>(budget, available);
  // Multiplication cannot overflow: here attempts*step < last-first.
  Time stop = attempts < available ? NanoSeconds(first + static_cast<int64_t>(attempts) * step) : latestStop;
  return {stop, attempts};
}

// Interleave flows within one send interval, independently of observation time.
// For a 120 s window and 0.5 s interval, every flow has exactly 240 attempts.
inline Time WindowStart(Time begin, Time interval, uint32_t flow, uint32_t flows)
{
  const int64_t step = interval.GetNanoSeconds();
  if (begin.GetNanoSeconds() < 0 || step <= 0 || flows == 0 || flow >= flows)
    throw std::invalid_argument("Invalid explicit UDP flow start");
  // Quotient/remainder avoids multiplying a potentially large interval by flow.
  const int64_t phase = (step / flows) * flow + (step % flows) * uint64_t(flow) / flows;
  return begin + NanoSeconds(phase);
}

inline Plan WindowPlan(Time start, Time end, Time interval)
{
  if (end <= start || interval.GetNanoSeconds() <= 0)
    throw std::invalid_argument("Invalid explicit UDP window");
  const uint64_t count = 1 + static_cast<uint64_t>(
      (end.GetNanoSeconds() - start.GetNanoSeconds() - 1) / interval.GetNanoSeconds());
  if (count > std::numeric_limits<uint32_t>::max())
    throw std::invalid_argument("Explicit UDP window exceeds the supported attempt count");
  return FixedPlan(start, end, interval, static_cast<uint32_t>(count));
}

inline uint64_t InWindow(Time start, Time interval, uint64_t attempts, Time begin, Time end)
{
  const int64_t first = start.GetNanoSeconds();
  const int64_t step = interval.GetNanoSeconds();
  if (step <= 0) throw std::invalid_argument("Invalid UDP demand interval");
  if (end <= begin) return 0;
  auto before = [&](int64_t edge) -> uint64_t {
    if (edge <= first) return 0;
    return std::min(attempts, 1 + static_cast<uint64_t>((edge - first - 1) / step));
  };
  return before(end.GetNanoSeconds()) - before(begin.GetNanoSeconds());
}

} // namespace ns3::cafbcr::udp_demand
#endif
