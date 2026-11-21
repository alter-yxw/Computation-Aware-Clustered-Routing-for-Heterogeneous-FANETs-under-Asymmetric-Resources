/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef CA_FBCR_FAILURE_SUMMARY_H
#define CA_FBCR_FAILURE_SUMMARY_H

#include "ca-fbcr-task-observer.h"
#include "ca-fbcr-udp-observer.h"
#include "ns3/arp-cache.h"
#include "ns3/arp-l3-protocol.h"
#include "ns3/ipv4-interface.h"
#include "ns3/ipv4-l3-protocol.h"
#include "ns3/node.h"
#include "ns3/wifi-mac.h"
#include "ns3/wifi-mpdu.h"
#include "ns3/wifi-net-device.h"
#include <array>
#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <sstream>
#include <vector>

namespace ns3::cafbcr {

// CA experiment only. Observe native drop callbacks and existing application
// outcomes. No packet construction/copy/mutation, RNG, scheduling, mobility
// query, forwarding decision, or event-file output. Packet UID is used only
// to join a registered UDP application attempt to its native drop observations.
class FailureSummary
{
public:
  static constexpr const char* SCHEMA = "cafbcr-failure-summary-v1";
  enum Traffic { REQUEST, REPLY, UDP, TRAFFIC_COUNT };
  enum Reason { NO_ROUTE, TTL, MAC_RETRY, ARP, APP_SEND, OTHER, REASON_COUNT };
  struct Counts {
    uint64_t missing = 0, unobserved = 0, multiple = 0, dropEvents = 0;
    std::array<uint64_t, REASON_COUNT> evidence{};
  };

  explicit FailureSummary(Time end) : m_end(end) {}
  ~FailureSummary() { Disconnect(); if (m_udpObserver) m_udpObserver->SetAttemptObserver({}); }

  void AttachUdp(UdpObservationWriter& udp)
  {
    NS_ABORT_MSG_IF(m_udpObserver, "UDP attempt observer already attached");
    m_udpObserver = &udp;
    udp.SetAttemptObserver([this](uint64_t uid, bool inWindow) {
      if (inWindow && !m_udp.emplace(uid, 0).second) ++m_errors;
    });
  }

  void AttachNode(Ptr<Node> node)
  {
    Binding b;
    b.ip = node->GetObject<Ipv4L3Protocol>(); b.arp = node->GetObject<ArpL3Protocol>();
    NS_ABORT_MSG_IF(!b.ip || !b.arp, "Failure summary requires IPv4/ARP");
    Check(b.ip->TraceConnectWithoutContext("Drop", MakeCallback(&FailureSummary::IpDrop, this)));
    Check(b.arp->TraceConnectWithoutContext("Drop", MakeCallback(&FailureSummary::ArpDrop, this)));
    for (uint32_t i = 0; i < b.ip->GetNInterfaces(); ++i) {
      if (auto cache = b.ip->GetInterface(i)->GetArpCache()) {
        Check(cache->TraceConnectWithoutContext("Drop", MakeCallback(&FailureSummary::ArpDrop, this)));
        b.caches.push_back(cache);
      }
    }
    for (uint32_t i = 0; i < node->GetNDevices(); ++i) {
      if (auto wifi = DynamicCast<WifiNetDevice>(node->GetDevice(i))) {
        auto mac = wifi->GetMac();
        Check(mac->TraceConnectWithoutContext("DroppedMpdu", MakeCallback(&FailureSummary::MpduDrop, this)));
        Check(mac->TraceConnectWithoutContext("MacTxDrop", MakeCallback(&FailureSummary::MacDrop, this)));
        b.macs.push_back(mac);
      }
    }
    m_bindings.push_back(std::move(b));
  }

  void Finalize(const TaskObservationWriter& tasks, const UdpObservationWriter& udp)
  {
    NS_ABORT_MSG_IF(m_finalized || Simulator::Now() < m_end, "Finalize failure summary once after cutoff");
    uint64_t generated = 0, received = 0, finished = 0, acknowledged = 0;
    tasks.VisitFinalTasks([&](uint32_t source, uint32_t sequence, bool rx, bool done, bool ack,
                             const std::string& failure) {
      ++generated; received += rx; finished += done; acknowledged += ack;
      IncludeTask({source, sequence}, rx, done, ack, failure);
    });
    for (const auto& [key, evidence] : m_tasks) if (!m_seenTasks.count(key)) ++m_errors;
    uint64_t attempts = 0, delivered = 0;
    udp.VisitFinalAttempts([&](uint64_t uid, bool eligible) {
      ++attempts; delivered += eligible;
      auto it = m_udp.find(uid);
      if (it == m_udp.end()) ++m_errors;
      if (!eligible) Missing(m_counts[UDP], it == m_udp.end() ? 0 : it->second);
    });
    const auto& t = tasks.GetSummary(); const auto u = udp.GetCounts();
    m_udpSubmitFailed = u.failed;
    if (generated != t.tasks.generated || received != t.received || finished != t.finished ||
        acknowledged != t.tasks.onTime + t.tasks.late || attempts != u.windowAttempts ||
        delivered != u.unique || attempts != m_udp.size() ||
        m_counts[REQUEST].missing != generated - received ||
        m_counts[REPLY].missing != finished - acknowledged ||
        m_counts[UDP].missing != attempts - delivered) ++m_errors;
    m_finalized = true;
    Disconnect();
    if (m_udpObserver) { m_udpObserver->SetAttemptObserver({}); m_udpObserver = nullptr; }
    NS_ABORT_MSG_IF(m_errors, "Failure summary does not reconcile with application ledgers");
  }

  const Counts& GetCounts(Traffic kind) const { return m_counts.at(kind); }
  uint64_t GetUnmatchedDropEvents() const { return m_unmatched; }

  static std::string CsvHeader()
  {
    std::ostringstream s;
    s << "FailureSummarySchema,FailureSummaryErrors,UnmatchedDropEvents";
    for (const char* prefix : {"TaskReq", "TaskReply", "UDP"}) {
      s << ',' << prefix << "Missing";
      for (const char* reason : {"NoRoute", "Ttl", "MacRetry", "Arp", "AppSend", "Other"}) {
        if (std::string(prefix) == "UDP" && std::string(reason) == "AppSend") continue;
        s << ',' << prefix << reason << "Evidence";
      }
      s << ',' << prefix << "Unobserved," << prefix << "MultiEvidence," << prefix << "DropEvents";
    }
    s << ",UDPSubmitFailed";
    return s.str();
  }

  std::string CsvValues() const
  {
    NS_ABORT_MSG_IF(!m_finalized, "Write failure summary only after finalization");
    std::ostringstream s; s << SCHEMA << ',' << m_errors << ',' << m_unmatched;
    for (unsigned kind = 0; kind < TRAFFIC_COUNT; ++kind) {
      const auto& c = m_counts[kind]; s << ',' << c.missing;
      for (unsigned reason = 0; reason < REASON_COUNT; ++reason)
        if (kind != UDP || reason != APP_SEND) s << ',' << c.evidence[reason];
      s << ',' << c.unobserved << ',' << c.multiple << ',' << c.dropEvents;
    }
    s << ',' << m_udpSubmitFailed;
    return s.str();
  }

private:
  friend class CafbcrFailureSummaryTestCase;
  using TaskKey = std::pair<uint32_t, uint32_t>;
  struct Binding {
    Ptr<Ipv4L3Protocol> ip; Ptr<ArpL3Protocol> arp;
    std::vector<Ptr<ArpCache>> caches; std::vector<Ptr<WifiMac>> macs;
  };
  static void Check(bool success) { NS_ABORT_MSG_IF(!success, "Missing native drop trace"); }
  static void Missing(Counts& counts, uint8_t mask)
  {
    ++counts.missing;
    unsigned reasons = 0;
    for (unsigned i = 0; i < REASON_COUNT; ++i)
      if (mask & (1u << i)) { ++counts.evidence[i]; ++reasons; }
    counts.unobserved += reasons == 0; counts.multiple += reasons > 1;
  }
  void IncludeTask(TaskKey key, bool received, bool finished, bool acknowledged,
                   const std::string& failure)
  {
    if (!m_seenTasks.insert(key).second) { ++m_errors; return; }
    auto it = m_tasks.find(key);
    auto masks = it == m_tasks.end() ? std::array<uint8_t, 2>{} : it->second;
    if (!received) {
      if (failure == "no_server" || failure == "send_failed") masks[REQUEST] |= 1u << APP_SEND;
      Missing(m_counts[REQUEST], masks[REQUEST]);
    }
    if (finished && !acknowledged) {
      if (failure == "response_send_failed") masks[REPLY] |= 1u << APP_SEND;
      Missing(m_counts[REPLY], masks[REPLY]);
    }
  }
  static bool IsUdp(Ptr<const Packet> p, bool llc)
  {
    std::array<uint8_t, 40> b{};
    const auto size = p->CopyData(b.data(), std::min<uint32_t>(b.size(), p->GetSize()));
    const unsigned offset = llc ? 8 : 0;
    if (llc && (size < 8 || b[0] != 0xaa || b[1] != 0xaa || b[2] != 3 ||
                b[6] != 8 || b[7] != 0)) return false;
    return size >= offset + 20 && (b[offset] >> 4) == 4 &&
           (b[offset] & 15) >= 5 && b[offset + 9] == 17;
  }
  void Drop(Ptr<const Packet> p, Reason reason, bool udp)
  {
    if (Simulator::Now() > m_end) return;
    CafbcrComputeTaskTag task;
    if (udp && p->PeekPacketTag(task) && std::isfinite(task.GetWorkload()) && task.GetWorkload() >= 0) {
      const Traffic kind = task.GetWorkload() == 0 ? REPLY : REQUEST;
      m_tasks[{task.GetSourceNodeId(), task.GetTaskId()}][kind] |= 1u << reason;
      ++m_counts[kind].dropEvents;
    } else if (udp && m_udp.count(p->GetUid())) {
      m_udp.at(p->GetUid()) |= 1u << reason; ++m_counts[UDP].dropEvents;
    } else ++m_unmatched;
  }
  void IpDrop(const Ipv4Header& h, Ptr<const Packet> p, Ipv4L3Protocol::DropReason reason,
              Ptr<Ipv4>, uint32_t)
  {
    Drop(p, reason == Ipv4L3Protocol::DROP_NO_ROUTE ? NO_ROUTE :
            reason == Ipv4L3Protocol::DROP_TTL_EXPIRED ? TTL : OTHER, h.GetProtocol() == 17);
  }
  void ArpDrop(Ptr<const Packet> p) { Drop(p, ARP, IsUdp(p, false)); }
  void MacDrop(Ptr<const Packet> p) { Drop(p, OTHER, IsUdp(p, true)); }
  void MpduDrop(WifiMacDropReason reason, Ptr<const WifiMpdu> mpdu)
  {
    if (!mpdu->GetHeader().IsData()) return;
    Drop(mpdu->GetPacket(), reason == WIFI_MAC_DROP_REACHED_RETRY_LIMIT ? MAC_RETRY : OTHER,
         IsUdp(mpdu->GetPacket(), true));
  }
  void Disconnect()
  {
    for (auto& b : m_bindings) {
      b.ip->TraceDisconnectWithoutContext("Drop", MakeCallback(&FailureSummary::IpDrop, this));
      b.arp->TraceDisconnectWithoutContext("Drop", MakeCallback(&FailureSummary::ArpDrop, this));
      for (auto cache : b.caches)
        cache->TraceDisconnectWithoutContext("Drop", MakeCallback(&FailureSummary::ArpDrop, this));
      for (auto mac : b.macs) {
        mac->TraceDisconnectWithoutContext("DroppedMpdu", MakeCallback(&FailureSummary::MpduDrop, this));
        mac->TraceDisconnectWithoutContext("MacTxDrop", MakeCallback(&FailureSummary::MacDrop, this));
      }
    }
    m_bindings.clear();
  }

  Time m_end;
  bool m_finalized = false;
  uint64_t m_errors = 0, m_unmatched = 0, m_udpSubmitFailed = 0;
  std::array<Counts, TRAFFIC_COUNT> m_counts{};
  std::map<TaskKey, std::array<uint8_t, 2>> m_tasks;
  std::set<TaskKey> m_seenTasks;
  std::map<uint64_t, uint8_t> m_udp;
  std::vector<Binding> m_bindings;
  UdpObservationWriter* m_udpObserver = nullptr;
};
} // namespace ns3::cafbcr
#endif
