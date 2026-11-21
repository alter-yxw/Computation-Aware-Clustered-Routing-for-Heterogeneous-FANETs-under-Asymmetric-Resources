/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "mo-relay-batch.h"
#include <cstring>
#include <stdexcept>

namespace ns3::mo {
TypeId StateHeader::GetTypeId()
{
    static TypeId id = TypeId("ns3::mo::StateHeader").SetParent<Header>().AddConstructor<StateHeader>();
    return id;
}
void StateHeader::Serialize(Buffer::Iterator b) const
{
    b.WriteU8(1); b.WriteU8(kind); b.WriteU8(ttl); b.WriteU8(vertex.server);
    b.WriteHtonU32(slot); b.WriteHtonU32(sequence); b.WriteHtonU64(stamp); b.WriteHtonU32(address.Get());
    for (double v : {vertex.rate, vertex.arrival})
    { uint64_t bits; std::memcpy(&bits, &v, 8); b.WriteHtonU64(bits); }
    b.WriteHtonU32(neighbors.size());
    for (auto n : neighbors) b.WriteHtonU32(n);
}
uint32_t StateHeader::Deserialize(Buffer::Iterator b)
{
    if (b.GetRemainingSize() < 44 || b.ReadU8() != 1) return 0;
    kind = b.ReadU8(); ttl = b.ReadU8(); vertex.server = b.ReadU8() != 0;
    slot = b.ReadNtohU32(); sequence = b.ReadNtohU32(); stamp = b.ReadNtohU64(); address = Ipv4Address(b.ReadNtohU32());
    for (double* v : {&vertex.rate, &vertex.arrival})
    { uint64_t bits = b.ReadNtohU64(); std::memcpy(v, &bits, 8); }
    auto size = b.ReadNtohU32();
    if (size > 4096 || b.GetRemainingSize() < 4 * size) return 0;
    neighbors.resize(size);
    for (auto& n : neighbors) n = b.ReadNtohU32();
    return GetSerializedSize();
}
void StateHeader::Print(std::ostream& out) const { out << "MO state " << slot << ':' << sequence; }

TypeId RelayBatchHeader::GetTypeId()
{
    static TypeId id = TypeId("ns3::mo::RelayBatchHeader").SetParent<Header>().AddConstructor<RelayBatchHeader>();
    return id;
}
uint32_t RelayBatchHeader::GetSerializedSize() const
{
    uint32_t n = PREFIX_BYTES;
    for (const auto& r : records) n += 2 + r.GetSerializedSize();
    return n;
}
void RelayBatchHeader::Serialize(Buffer::Iterator b) const
{
    if (records.empty() || GetSerializedSize() > MAX_BYTES) throw std::runtime_error("Invalid MO batch size");
    b.WriteHtonU32(MAGIC); b.WriteU8(1); b.WriteU8(0); b.WriteHtonU16(records.size());
    for (const auto& r : records) {
        const auto n = r.GetSerializedSize(); b.WriteHtonU16(n); r.Serialize(b); b.Next(n);
    }
}
uint32_t RelayBatchHeader::Deserialize(Buffer::Iterator b)
{
    records.clear();
    const auto total = b.GetRemainingSize();
    if (total < PREFIX_BYTES || total > MAX_BYTES || b.ReadNtohU32() != MAGIC ||
        b.ReadU8() != 1 || b.ReadU8() != 0) return 0;
    const auto count = b.ReadNtohU16();
    if (!count || count > (MAX_BYTES - PREFIX_BYTES) / 46) return 0;
    std::vector<StateHeader> decoded;
    for (uint32_t i = 0; i < count; ++i) {
        if (b.GetRemainingSize() < 2) return 0;
        auto n = b.ReadNtohU16();
        if (n < 44 || n > b.GetRemainingSize()) return 0;
        StateHeader r;
        if (r.Deserialize(b) != n || r.kind != 2) return 0;
        b.Next(n); decoded.push_back(std::move(r));
    }
    if (b.GetRemainingSize()) return 0; // no partial acceptance or trailing records
    records = std::move(decoded); return total;
}
void RelayBatchHeader::Print(std::ostream& out) const { out << "MO relay batch records=" << records.size(); }

std::optional<RelayBatchQueue::Pending> RelayBatchQueue::Put(const StateHeader& state, Time now)
{
    if (state.kind != 2 || state.GetSerializedSize() + 2 + RelayBatchHeader::PREFIX_BYTES > RelayBatchHeader::MAX_BYTES)
        throw std::runtime_error("MO relay record exceeds batch contract");
    auto it = m_pending.find(state.slot);
    std::optional<Pending> prior;
    if (it != m_pending.end()) {
        if (int32_t(state.sequence - it->second.state.sequence) <= 0)
            throw std::runtime_error("MO relay queue requires an accepted newer version");
        prior = it->second;
    }
    m_pending[state.slot] = {state, now};
    return prior;
}
std::optional<RelayBatchQueue::Pending> RelayBatchQueue::Cancel(uint32_t slot, uint32_t sequence)
{
    auto it = m_pending.find(slot);
    if (it == m_pending.end() || it->second.state.sequence != sequence)
        return std::nullopt;
    auto prior = it->second;
    m_pending.erase(it);
    return prior;
}
RelayBatchQueue::Flush RelayBatchQueue::Drain(Time now, Time hold)
{
    Flush out; RelayBatchHeader batch;
    for (const auto& [slot, p] : m_pending) {
        const bool expired = p.state.stamp > uint64_t(now.GetNanoSeconds()) || now - NanoSeconds(p.state.stamp) >= hold;
        out.outcomes.push_back({p, expired});
        if (expired) continue;
        if (batch.GetSerializedSize() + 2 + p.state.GetSerializedSize() > RelayBatchHeader::MAX_BYTES) {
            out.packets.push_back(std::move(batch)); batch = RelayBatchHeader();
        }
        batch.records.push_back(p.state);
    }
    if (!batch.records.empty()) out.packets.push_back(std::move(batch));
    m_pending.clear(); return out;
}
std::vector<RelayBatchQueue::Pending> RelayBatchQueue::Clear()
{
    std::vector<Pending> out;
    for (const auto& [slot, p] : m_pending) out.push_back(p);
    m_pending.clear(); return out;
}
}
