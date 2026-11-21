/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "cafbcr-forwarding-history.h"
#include "ns3/simulator.h"
#include <algorithm>

namespace ns3::cafbcr {
NS_OBJECT_ENSURE_REGISTERED(ForwardingHistory);

TypeId ForwardingHistory::GetTypeId()
{
  static TypeId tid = TypeId("ns3::cafbcr::ForwardingHistory")
    .SetParent<Object>().SetGroupName("cafbcr").AddConstructor<ForwardingHistory>()
    .AddTraceSource("Decision", "Local forwarding history actions; UID is observation only",
      MakeTraceSourceAccessor(&ForwardingHistory::m_decision),
      "ns3::cafbcr::ForwardingHistoryEvent::TracedCallback");
  return tid;
}

bool ForwardingHistory::Identity::operator==(const Identity& other) const
{
  return source == other.source && destination == other.destination && protocol == other.protocol &&
    identification == other.identification && payload == other.payload;
}

void ForwardingHistory::UpdateSize()
{
  m_stats.entries = m_visits.size();
  m_stats.peakEntries = std::max(m_stats.peakEntries, m_stats.entries);
  m_stats.peakPayloadBytes = std::max(m_stats.peakPayloadBytes, m_stats.payloadBytes);
}

void ForwardingHistory::Purge()
{
  while (!m_visits.empty() && Simulator::Now() - m_visits.front().time >= m_lifetime) {
    m_stats.payloadBytes -= m_visits.front().identity.payload.size();
    m_visits.pop_front(); ++m_stats.expired;
  }
  UpdateSize();
}

void ForwardingHistory::Emit(const char* action, const Candidate& candidate,
                            std::optional<Time> first, std::optional<uint8_t> ttl)
{
  if (m_decision.IsEmpty()) return;
  m_decision(ForwardingHistoryEvent{Simulator::Now(), action, candidate.observationUid,
    candidate.identity.source, candidate.identity.destination, candidate.identity.protocol,
    candidate.ttl, candidate.identity.identification, candidate.fragmentOffset,
    candidate.moreFragments, candidate.packetBytes, first, ttl,
    m_stats.entries, m_stats.payloadBytes, m_stats.expired, m_stats.evicted});
}

ForwardingHistory::Candidate ForwardingHistory::Inspect(Ptr<const Packet> packet, const Ipv4Header& header)
{
  ++m_stats.checks;
  Purge();
  Candidate candidate;
  candidate.identity.source = header.GetSource();
  candidate.identity.destination = header.GetDestination();
  candidate.identity.protocol = header.GetProtocol();
  candidate.identity.identification = header.GetIdentification();
  candidate.observationUid = packet->GetUid(); // Event metadata, not a lookup key.
  candidate.packetBytes = packet->GetSize();
  candidate.ttl = header.GetTtl();
  candidate.fragmentOffset = header.GetFragmentOffset();
  candidate.moreFragments = !header.IsLastFragment();
  if (candidate.ttl <= 1) {
    ++m_stats.ttlDelegated; Emit("ttl_delegated", candidate); return candidate;
  }
  if (candidate.fragmentOffset || candidate.moreFragments) {
    ++m_stats.fragmentsUntracked; Emit("fragment_untracked", candidate); return candidate;
  }
  if (!m_entryLimit || candidate.packetBytes > m_payloadLimit) {
    ++m_stats.oversizedUntracked; Emit("oversized_untracked", candidate); return candidate;
  }
  candidate.track = true;
  candidate.identity.payload.resize(candidate.packetBytes);
  packet->CopyData(candidate.identity.payload.data(), candidate.packetBytes);
  m_stats.copiedPayloadBytes += candidate.packetBytes;
  for (const auto& visit : m_visits) {
    ++m_stats.identityComparisons;
    if (candidate.identity == visit.identity) {
      if (candidate.ttl < visit.ttl) {
        candidate.revisit = true;
        ++m_stats.revisitDrops;
        Emit("revisit_drop", candidate, visit.time, visit.ttl);
      }
      break;
    }
  }
  return candidate;
}

void ForwardingHistory::Remember(Candidate&& candidate)
{
  if (!candidate.track || candidate.revisit) return;
  for (auto it = m_visits.begin(); it != m_visits.end(); ++it) {
    ++m_stats.identityComparisons;
    if (candidate.identity == it->identity) {
      if (candidate.ttl == it->ttl) {
        ++m_stats.sameTtlForwards;
        Emit("same_ttl_forward", candidate, it->time, it->ttl);
        return; // Duplicates do not extend the first observation's lifetime.
      }
      // A higher TTL is compatible with identity reuse, not proof of a loop.
      ++m_stats.higherTtlForwards;
      m_stats.payloadBytes -= it->identity.payload.size(); m_visits.erase(it);
      break;
    }
  }
  while (!m_visits.empty() && (m_visits.size() >= m_entryLimit ||
         m_stats.payloadBytes + candidate.packetBytes > m_payloadLimit)) {
    m_stats.payloadBytes -= m_visits.front().identity.payload.size();
    m_visits.pop_front(); ++m_stats.evicted;
  }
  m_stats.payloadBytes += candidate.packetBytes;
  ++m_stats.remembered;
  Identity stored{candidate.identity.source, candidate.identity.destination, candidate.identity.protocol,
                  candidate.identity.identification, std::move(candidate.identity.payload)};
  m_visits.push_back(Visit{std::move(stored), Simulator::Now(), candidate.ttl});
  UpdateSize();
  Emit("remember", candidate, Simulator::Now(), candidate.ttl);
}

void ForwardingHistory::DoDispose()
{
  m_visits.clear(); m_stats.payloadBytes = 0; UpdateSize();
  Object::DoDispose();
}
} // namespace ns3::cafbcr
