/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef MO_RELAY_BATCH_H
#define MO_RELAY_BATCH_H
#include "mo-policy.h"
#include "ns3/header.h"
#include "ns3/ipv4-address.h"
#include "ns3/nstime.h"
#include <map>
#include <optional>

namespace ns3::mo {

// The original version-1 descriptor, byte-for-byte unchanged.
class StateHeader : public Header
{
public:
    uint8_t kind = 1, ttl = 64;
    uint32_t slot = 0, sequence = 0;
    uint64_t stamp = 0;
    Ipv4Address address;
    Vertex vertex;
    std::vector<uint32_t> neighbors;
    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const override { return GetTypeId(); }
    uint32_t GetSerializedSize() const override { return 44 + 4 * neighbors.size(); }
    void Serialize(Buffer::Iterator) const override;
    uint32_t Deserialize(Buffer::Iterator) override;
    void Print(std::ostream&) const override;
};

// UDP payload: magic:u32, version:u8, reserved:u8, count:u16,
// then repeated length:u16 and an unchanged kind-2 StateHeader. Network order.
class RelayBatchHeader : public Header
{
public:
    static constexpr uint32_t MAGIC = 0x4d4f4231; // MOB1
    static constexpr uint32_t MAX_BYTES = 1200, PREFIX_BYTES = 8, MAX_NODES = 287;
    std::vector<StateHeader> records;
    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const override { return GetTypeId(); }
    uint32_t GetSerializedSize() const override;
    void Serialize(Buffer::Iterator) const override;
    uint32_t Deserialize(Buffer::Iterator) override;
    void Print(std::ostream&) const override;
};

// Local pending records only. No timers/RNG or simulator-global information.
// RoutingProtocol fixes a flush deadline on the first insertion; replacements
// never postpone it. Each pending version retains its original wire stamp.
class RelayBatchQueue
{
public:
    struct Pending { StateHeader state; Time enqueued; };
    struct Outcome { Pending record; bool expired; };
    struct Flush { std::vector<RelayBatchHeader> packets; std::vector<Outcome> outcomes; };
    std::optional<Pending> Put(const StateHeader&, Time);
    std::optional<Pending> Cancel(uint32_t slot, uint32_t sequence);
    Flush Drain(Time now, Time hold);
    std::vector<Pending> Clear();
    uint32_t Size() const { return m_pending.size(); }
private:
    std::map<uint32_t, Pending> m_pending;
};
}
#endif
