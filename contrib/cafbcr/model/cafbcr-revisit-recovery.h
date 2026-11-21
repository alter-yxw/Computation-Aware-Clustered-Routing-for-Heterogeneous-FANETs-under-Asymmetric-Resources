/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef CAFBCR_REVISIT_RECOVERY_H
#define CAFBCR_REVISIT_RECOVERY_H

#include "cafbcr-forwarding-history.h"

namespace ns3::cafbcr {

struct RevisitRecoveryEvent
{
  Time time;
  const char* action;
  uint64_t observationUid;
  Ipv4Address source, destination;
  uint8_t protocol, ttl;
  uint16_t identification;
  uint32_t packetBytes;
  std::optional<Time> firstForwarded;
  std::optional<uint8_t> firstTtl;
  std::optional<Ipv4Address> selected;
  std::vector<Ipv4Address> tried;
  uint32_t entries, payloadBytes, hopBytes;
};

// Bounded, node-local alternative forwarding on a lower-TTL revisit. This
// is neither face routing nor a complete path search. No wire state is added.
class RevisitRecovery : public Object
{
public:
  static TypeId GetTypeId();
  static constexpr const char* SCHEMA = "cafbcr-alternate-revisit-v1";
  using Identity = ForwardingHistory::Identity;
  struct Candidate {
    Identity identity;
    uint64_t observationUid = 0;
    uint8_t ttl = 0;
    uint32_t packetBytes = 0;
    bool track = false, revisit = false;
    std::optional<Time> firstForwarded;
    std::optional<uint8_t> firstTtl;
    std::vector<Ipv4Address> tried;
  };
  struct Stats {
    uint64_t checks = 0, nativeForwards = 0, alternateForwards = 0, revisits = 0,
      exhausted = 0, budgetStops = 0, invalidPosition = 0, expired = 0, evicted = 0,
      copiedPayloadBytes = 0, identityComparisons = 0, fragmentsUntracked = 0,
      oversizedUntracked = 0, ttlDelegated = 0, duplicateHopOverflow = 0;
    uint32_t entries = 0, payloadBytes = 0, hopBytes = 0,
      peakEntries = 0, peakPayloadBytes = 0, peakHopBytes = 0;
  };
  Candidate Inspect(Ptr<const Packet> packet, const Ipv4Header& header);
  void Remember(Candidate&& candidate, Ipv4Address nextHop);
  void Stop(const Candidate& candidate, const char* reason);
  Stats GetStats() const { return m_stats; }
  uint32_t GetEntryLimit() const { return m_entryLimit; }
  uint32_t GetPayloadLimit() const { return m_payloadLimit; }
  uint32_t GetNextHopLimit() const { return m_nextHopLimit; }
  Time GetLifetime() const { return m_lifetime; }
private:
  friend class CafbcrRecoveryStateCase;
  struct Visit { Identity identity; Time time; uint8_t ttl; std::vector<Ipv4Address> tried; };
  void Purge();
  void UpdateSize();
  void EraseOldest();
  void Emit(const char* action, const Candidate& candidate,
            std::optional<Ipv4Address> selected = std::nullopt);
  void DoDispose() override;
  std::deque<Visit> m_visits;
  uint32_t m_entryLimit = 256, m_payloadLimit = 256 * 1024, m_nextHopLimit = 8;
  Time m_lifetime = Seconds(1);
  Stats m_stats;
  TracedCallback<const RevisitRecoveryEvent&> m_decision;
};
} // namespace ns3::cafbcr
#endif
