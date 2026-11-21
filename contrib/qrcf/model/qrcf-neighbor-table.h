/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef QRCF_NEIGHBOR_TABLE_H
#define QRCF_NEIGHBOR_TABLE_H
#include "qrcf-packet.h"
#include "ns3/ipv4-address.h"
#include "ns3/nstime.h"
#include <map>

namespace ns3::qrcf
{
class NeighborTable
{
  public:
    struct Entry
    {
        HelloHeader hello;
        Time received;
        Time expires;
        std::optional<HelloHeader> previousHello;
        std::optional<Time> previousReceived;
        std::set<uint64_t> receivedIds;
    };
    // Deduplicate all IDs retained in this neighbor entry, not just the latest.
    // Caller determines validity policy at exact expiry via includeBoundary.
    bool Receive(Ipv4Address sender, const HelloHeader& hello, Time now, Time upperHello);
    std::size_t Purge(Time now, bool includeBoundary);
    const std::map<Ipv4Address, Entry>& Entries() const { return m_entries; }
    std::set<uint32_t> NodeIds() const;
    void Clear() { m_entries.clear(); }
  private:
    std::map<Ipv4Address, Entry> m_entries;
};
} // namespace ns3::qrcf
#endif
