/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "cafbcr-revisit-recovery.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"
#include <algorithm>
#include <string>

namespace ns3::cafbcr {
NS_OBJECT_ENSURE_REGISTERED(RevisitRecovery);

TypeId RevisitRecovery::GetTypeId()
{
  static TypeId tid = TypeId("ns3::cafbcr::RevisitRecovery")
    .SetParent<Object>().SetGroupName("cafbcr").AddConstructor<RevisitRecovery>()
    .AddAttribute("NextHopLimit", "Maximum stored distinct next hops per local wire identity",
      UintegerValue(8), MakeUintegerAccessor(&RevisitRecovery::m_nextHopLimit),
      MakeUintegerChecker<uint32_t>(1, 64))
    .AddTraceSource("Decision", "Local alternative forwarding; UID is trace metadata only",
      MakeTraceSourceAccessor(&RevisitRecovery::m_decision),
      "ns3::cafbcr::RevisitRecoveryEvent::TracedCallback");
  return tid;
}

void RevisitRecovery::UpdateSize()
{
  m_stats.entries = m_visits.size();
  m_stats.peakEntries = std::max(m_stats.peakEntries, m_stats.entries);
  m_stats.peakPayloadBytes = std::max(m_stats.peakPayloadBytes, m_stats.payloadBytes);
  m_stats.peakHopBytes = std::max(m_stats.peakHopBytes, m_stats.hopBytes);
}

void RevisitRecovery::EraseOldest()
{
  m_stats.payloadBytes -= m_visits.front().identity.payload.size();
  m_stats.hopBytes -= 4 * m_visits.front().tried.size();
  m_visits.pop_front();
}

void RevisitRecovery::Purge()
{
  while (!m_visits.empty() && Simulator::Now() - m_visits.front().time >= m_lifetime) {
    EraseOldest(); ++m_stats.expired;
  }
  UpdateSize();
}

void RevisitRecovery::Emit(const char* action, const Candidate& c,
                           std::optional<Ipv4Address> selected)
{
  if (m_decision.IsEmpty()) return;
  m_decision(RevisitRecoveryEvent{Simulator::Now(), action, c.observationUid,
    c.identity.source, c.identity.destination, c.identity.protocol, c.ttl,
    c.identity.identification, c.packetBytes, c.firstForwarded, c.firstTtl,
    selected, c.tried, m_stats.entries, m_stats.payloadBytes, m_stats.hopBytes});
}

RevisitRecovery::Candidate RevisitRecovery::Inspect(Ptr<const Packet> packet, const Ipv4Header& header)
{
  ++m_stats.checks; Purge();
  Candidate c;
  c.identity.source = header.GetSource(); c.identity.destination = header.GetDestination();
  c.identity.protocol = header.GetProtocol(); c.identity.identification = header.GetIdentification();
  c.observationUid = packet->GetUid(); c.ttl = header.GetTtl(); c.packetBytes = packet->GetSize();
  if (c.ttl <= 1) { ++m_stats.ttlDelegated; Emit("ttl_delegated", c); return c; }
  if (header.GetFragmentOffset() || !header.IsLastFragment()) {
    ++m_stats.fragmentsUntracked; Emit("fragment_untracked", c); return c;
  }
  if (!m_entryLimit || c.packetBytes > m_payloadLimit) {
    ++m_stats.oversizedUntracked; Emit("oversized_untracked", c); return c;
  }
  c.track = true; c.identity.payload.resize(c.packetBytes);
  packet->CopyData(c.identity.payload.data(), c.packetBytes);
  m_stats.copiedPayloadBytes += c.packetBytes;
  for (const auto& visit : m_visits) {
    ++m_stats.identityComparisons;
    if (!(c.identity == visit.identity)) continue;
    // Equal TTL may be a duplicate. Higher TTL may be identity reuse.
    if (c.ttl <= visit.ttl) {
      c.firstForwarded = visit.time; c.firstTtl = visit.ttl; c.tried = visit.tried;
      c.revisit = c.ttl < visit.ttl;
      if (c.revisit) { ++m_stats.revisits; Emit("revisit", c); }
    }
    break;
  }
  return c;
}

void RevisitRecovery::Remember(Candidate&& c, Ipv4Address nextHop)
{
  if (!c.track) return;
  if (c.revisit) ++m_stats.alternateForwards;
  else ++m_stats.nativeForwards;
  for (auto it = m_visits.begin(); it != m_visits.end(); ++it) {
    ++m_stats.identityComparisons;
    if (!(c.identity == it->identity)) continue;
    if (c.ttl > it->ttl) {
      m_stats.payloadBytes -= it->identity.payload.size();
      m_stats.hopBytes -= 4 * it->tried.size(); m_visits.erase(it); break;
    }
    if (std::find(it->tried.begin(), it->tried.end(), nextHop) == it->tried.end()) {
      if (it->tried.size() < m_nextHopLimit) { it->tried.push_back(nextHop); m_stats.hopBytes += 4; }
      else { ++m_stats.duplicateHopOverflow; Emit("duplicate_hop_overflow_untracked", c, nextHop); }
    }
    c.firstForwarded = it->time; c.firstTtl = it->ttl;
    // Event tried[] is the list BEFORE this callback; selected is separate.
    UpdateSize(); Emit(c.revisit ? "alternate_forward" : "native_forward", c, nextHop);
    return; // Neither revisits nor equal-TTL duplicates extend the lifetime.
  }
  while (!m_visits.empty() && (m_visits.size() >= m_entryLimit ||
         m_stats.payloadBytes + c.packetBytes > m_payloadLimit)) { EraseOldest(); ++m_stats.evicted; }
  c.firstForwarded = Simulator::Now(); c.firstTtl = c.ttl;
  m_stats.payloadBytes += c.packetBytes; m_stats.hopBytes += 4;
  Identity stored{c.identity.source, c.identity.destination, c.identity.protocol,
                  c.identity.identification, std::move(c.identity.payload)};
  m_visits.push_back(Visit{std::move(stored), Simulator::Now(), c.ttl, {nextHop}});
  UpdateSize(); Emit("native_forward", c, nextHop);
}

void RevisitRecovery::Stop(const Candidate& c, const char* reason)
{
  if (std::string(reason) == "budget_exhausted") ++m_stats.budgetStops;
  else if (std::string(reason) == "invalid_position") ++m_stats.invalidPosition;
  else ++m_stats.exhausted;
  Emit(reason, c);
}

void RevisitRecovery::DoDispose()
{
  m_visits.clear(); m_stats.payloadBytes = 0; m_stats.hopBytes = 0; UpdateSize();
  Object::DoDispose();
}
} // namespace ns3::cafbcr
