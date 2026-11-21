/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef CAFBCR_FORWARDING_HISTORY_H
#define CAFBCR_FORWARDING_HISTORY_H

#include "ns3/ipv4-header.h"
#include "ns3/object.h"
#include "ns3/nstime.h"
#include "ns3/packet.h"
#include "ns3/traced-callback.h"
#include <deque>
#include <optional>
#include <vector>

namespace ns3::cafbcr {

struct ForwardingHistoryEvent
{
  Time time;
  const char* action;
  uint64_t observationUid; // Trace correlation only; never part of identity.
  Ipv4Address source, destination;
  uint8_t protocol, ttl;
  uint16_t identification, fragmentOffset;
  bool moreFragments;
  uint32_t packetBytes;
  std::optional<Time> firstForwarded;
  std::optional<uint8_t> firstTtl;
  uint32_t entries, trackedPayloadBytes;
  uint64_t expired, evicted;
};

// Experimental local early termination. It is not a geographic recovery
// algorithm and does not guarantee delivery. Identity is the wire IPv4 tuple
// plus exact IPv4 payload bytes; TTL and ns-3 UID are excluded from equality.
class ForwardingHistory : public Object
{
public:
  static TypeId GetTypeId();
  static constexpr const char* SCHEMA = "cafbcr-revisit-termination-v1";
  struct Stats {
    uint64_t checks = 0, remembered = 0, revisitDrops = 0, sameTtlForwards = 0,
      higherTtlForwards = 0, expired = 0, evicted = 0, copiedPayloadBytes = 0,
      identityComparisons = 0, fragmentsUntracked = 0, oversizedUntracked = 0,
      ttlDelegated = 0;
    uint32_t entries = 0, payloadBytes = 0, peakEntries = 0, peakPayloadBytes = 0;
  };
  struct Identity {
    Ipv4Address source, destination;
    uint8_t protocol = 0;
    uint16_t identification = 0;
    std::vector<uint8_t> payload;
    bool operator==(const Identity& other) const;
  };
  struct Candidate {
    Identity identity;
    uint64_t observationUid = 0;
    uint8_t ttl = 0;
    uint16_t fragmentOffset = 0;
    bool moreFragments = false, track = false, revisit = false;
    uint32_t packetBytes = 0;
  };

  Candidate Inspect(Ptr<const Packet> packet, const Ipv4Header& header);
  // Called only when native routing has selected a unicast callback. TTL<=1
  // stays with IPv4 and is not remembered as a successful forwarding attempt.
  void Remember(Candidate&& candidate);
  Stats GetStats() const { return m_stats; }
  uint32_t GetEntryLimit() const { return m_entryLimit; }
  uint32_t GetPayloadLimit() const { return m_payloadLimit; }
  Time GetLifetime() const { return m_lifetime; }

private:
  friend class CafbcrForwardingHistoryTestCase;
  struct Visit { Identity identity; Time time; uint8_t ttl; };
  void Purge();
  void UpdateSize();
  void Emit(const char* action, const Candidate& candidate,
            std::optional<Time> first = std::nullopt,
            std::optional<uint8_t> ttl = std::nullopt);
  void DoDispose() override;
  std::deque<Visit> m_visits;
  uint32_t m_entryLimit = 256, m_payloadLimit = 256 * 1024;
  Time m_lifetime = Seconds(1);
  Stats m_stats;
  TracedCallback<const ForwardingHistoryEvent&> m_decision;
};
} // namespace ns3::cafbcr
#endif
