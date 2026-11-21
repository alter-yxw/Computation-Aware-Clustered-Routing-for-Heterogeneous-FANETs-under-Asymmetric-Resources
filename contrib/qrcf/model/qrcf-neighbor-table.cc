/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "qrcf-neighbor-table.h"
#include <stdexcept>

namespace ns3::qrcf
{
bool NeighborTable::Receive(Ipv4Address sender, const HelloHeader& hello, Time now, Time upper)
{
    if (!hello.IsValid() || upper <= Time(0) || now < Time(0) || sender == Ipv4Address::GetAny() ||
        sender.IsBroadcast() || sender.IsMulticast())
    { throw std::invalid_argument("Invalid QRCF neighbor observation"); }
    const auto it = m_entries.find(sender);
    if (it != m_entries.end())
    {
        auto& entry = it->second;
        if (!entry.receivedIds.insert(hello.messageId).second) { return false; }
        entry.previousHello = entry.hello;
        entry.previousReceived = entry.received;
        entry.hello = hello; entry.received = now; entry.expires = now+upper;
    }
    else { m_entries.emplace(sender,Entry{hello,now,now+upper,{},{},{hello.messageId}}); }
    return true;
}
std::size_t NeighborTable::Purge(Time now, bool includeBoundary)
{
    std::size_t removed = 0;
    for (auto i = m_entries.begin(); i != m_entries.end();)
    {
        if (includeBoundary ? i->second.expires <= now : i->second.expires < now)
        { i = m_entries.erase(i); ++removed; }
        else { ++i; }
    }
    return removed;
}
std::set<uint32_t> NeighborTable::NodeIds() const
{
    std::set<uint32_t> ids;
    for (const auto& [address, entry] : m_entries) { ids.insert(entry.hello.nodeId); }
    return ids;
}
} // namespace ns3::qrcf
