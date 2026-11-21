/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "cfr-packet.h"
#include <cstring>

namespace ns3::cfr
{
NS_OBJECT_ENSURE_REGISTERED(HelloHeader);
NS_OBJECT_ENSURE_REGISTERED(PathTag);
NS_OBJECT_ENSURE_REGISTERED(TaskHeader);
static void WriteDouble(Buffer::Iterator& i, double v)
{
    uint64_t bits;
    std::memcpy(&bits, &v, 8);
    i.WriteHtonU64(bits);
}
static double ReadDouble(Buffer::Iterator& i)
{
    auto bits = i.ReadNtohU64();
    double v;
    std::memcpy(&v, &bits, 8);
    return v;
}
TypeId HelloHeader::GetTypeId()
{
    static TypeId tid = TypeId("ns3::cfr::HelloHeader").SetParent<Header>().AddConstructor<HelloHeader>();
    return tid;
}
TypeId HelloHeader::GetInstanceTypeId() const { return GetTypeId(); }
uint32_t HelloHeader::GetSerializedSize() const { return 4 + 50 * records.size(); }
void HelloHeader::Serialize(Buffer::Iterator i) const
{
    i.WriteHtonU16(0xcf01);
    i.WriteHtonU16(records.size());
    for (const auto& r : records)
    {
        i.WriteHtonU16(r.slot); i.WriteHtonU32(r.address.Get());
        WriteDouble(i, r.position.x); WriteDouble(i, r.position.y); WriteDouble(i, r.position.z);
        WriteDouble(i, r.ghz); i.WriteHtonU32(r.sequence); i.WriteHtonU64(r.stampNs);
    }
}
uint32_t HelloHeader::Deserialize(Buffer::Iterator i)
{
    if (i.GetRemainingSize() < 4 || i.ReadNtohU16() != 0xcf01) return 0;
    auto count = i.ReadNtohU16();
    if (count == 0 || i.GetRemainingSize() < 50u * count) return 0;
    records.clear();
    for (uint32_t j = 0; j < count; ++j)
    {
        NeighborRecord r;
        r.slot = i.ReadNtohU16(); r.address = Ipv4Address(i.ReadNtohU32());
        r.position.x = ReadDouble(i); r.position.y = ReadDouble(i); r.position.z = ReadDouble(i);
        r.ghz = ReadDouble(i); r.sequence = i.ReadNtohU32(); r.stampNs = i.ReadNtohU64();
        records.push_back(r);
    }
    return GetSerializedSize();
}
void HelloHeader::Print(std::ostream& os) const { os << "CFR Hello records=" << records.size(); }
TypeId PathTag::GetTypeId()
{
    static TypeId tid = TypeId("ns3::cfr::PathTag").SetParent<Tag>().AddConstructor<PathTag>();
    return tid;
}
TypeId PathTag::GetInstanceTypeId() const { return GetTypeId(); }
uint32_t PathTag::GetSerializedSize() const { return 2 + 2 * path.size(); }
void PathTag::Serialize(TagBuffer i) const
{
    i.WriteU16(path.size());
    for (auto id : path) i.WriteU16(id);
}
void PathTag::Deserialize(TagBuffer i)
{
    auto count = i.ReadU16(); path.clear();
    for (uint32_t j = 0; j < count; ++j) path.push_back(i.ReadU16());
}
void PathTag::Print(std::ostream& os) const { os << "CFR path length=" << path.size(); }
TypeId TaskHeader::GetTypeId()
{
    static TypeId tid = TypeId("ns3::cfr::TaskHeader").SetParent<Header>().AddConstructor<TaskHeader>();
    return tid;
}
TypeId TaskHeader::GetInstanceTypeId() const { return GetTypeId(); }
uint32_t TaskHeader::GetSerializedSize() const { return 139 + 2 * path.size(); }
void TaskHeader::Serialize(Buffer::Iterator i) const
{
    i.WriteHtonU16(0xcf02); i.WriteU8(kind);
    i.WriteHtonU32(source.Get()); i.WriteHtonU32(destination.Get());
    for (auto v : {taskId, originalBytes, wireBytes, chunk, chunks}) i.WriteHtonU32(v);
    i.WriteHtonU16(hop);
    for (auto v : {createdNs, deadlineNs, sentNs}) i.WriteHtonU64(v);
    for (auto v : {originalCycles, remainingCycles, computeSeconds, queueSeconds, networkSeconds,
                   computeJoules, communicationJoules, hopCompute, hopEnergy, txWatts}) WriteDouble(i, v);
    i.WriteHtonU16(path.size());
    for (auto id : path) i.WriteHtonU16(id);
}
uint32_t TaskHeader::Deserialize(Buffer::Iterator i)
{
    if (i.GetRemainingSize() < 139 || i.ReadNtohU16() != 0xcf02) return 0;
    kind = i.ReadU8(); source = Ipv4Address(i.ReadNtohU32()); destination = Ipv4Address(i.ReadNtohU32());
    taskId = i.ReadNtohU32(); originalBytes = i.ReadNtohU32(); wireBytes = i.ReadNtohU32();
    chunk = i.ReadNtohU32(); chunks = i.ReadNtohU32(); hop = i.ReadNtohU16();
    createdNs = i.ReadNtohU64(); deadlineNs = i.ReadNtohU64(); sentNs = i.ReadNtohU64();
    originalCycles = ReadDouble(i); remainingCycles = ReadDouble(i);
    computeSeconds = ReadDouble(i); queueSeconds = ReadDouble(i); networkSeconds = ReadDouble(i);
    computeJoules = ReadDouble(i); communicationJoules = ReadDouble(i);
    hopCompute = ReadDouble(i); hopEnergy = ReadDouble(i); txWatts = ReadDouble(i);
    auto count = i.ReadNtohU16();
    if (i.GetRemainingSize() < 2u * count) return 0;
    path.clear();
    for (uint32_t j = 0; j < count; ++j) path.push_back(i.ReadNtohU16());
    return GetSerializedSize();
}
void TaskHeader::Print(std::ostream& os) const
{
    os << "CFR task=" << source << ':' << taskId << " remainingCycles=" << remainingCycles;
}
}
