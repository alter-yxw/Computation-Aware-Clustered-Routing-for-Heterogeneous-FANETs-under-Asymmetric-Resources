/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef CFR_PACKET_H
#define CFR_PACKET_H
#include "ns3/header.h"
#include "ns3/ipv4-address.h"
#include "ns3/tag.h"
#include "ns3/vector.h"
#include <vector>

namespace ns3::cfr
{
struct NeighborRecord
{
    uint16_t slot = 0;
    Ipv4Address address;
    Vector position;
    double ghz = 0;
    uint32_t sequence = 0;
    uint64_t stampNs = 0;
};
class HelloHeader : public Header
{
  public:
    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const override;
    uint32_t GetSerializedSize() const override;
    void Serialize(Buffer::Iterator i) const override;
    uint32_t Deserialize(Buffer::Iterator i) override;
    void Print(std::ostream& os) const override;
    std::vector<NeighborRecord> records; // first record is the sender
};

class PathTag : public Tag
{
  public:
    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const override;
    uint32_t GetSerializedSize() const override;
    void Serialize(TagBuffer i) const override;
    void Deserialize(TagBuffer i) override;
    void Print(std::ostream& os) const override;
    std::vector<uint16_t> path;
};

class TaskHeader : public Header
{
  public:
    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const override;
    uint32_t GetSerializedSize() const override;
    void Serialize(Buffer::Iterator i) const override;
    uint32_t Deserialize(Buffer::Iterator i) override;
    void Print(std::ostream& os) const override;
    uint8_t kind = 0; // 0=task fragment, 1=completion ACK
    Ipv4Address source, destination;
    uint32_t taskId = 0, originalBytes = 0, wireBytes = 0, chunk = 0, chunks = 1;
    uint16_t hop = 0;
    uint64_t createdNs = 0, deadlineNs = 0, sentNs = 0;
    double originalCycles = 0, remainingCycles = 0;
    double computeSeconds = 0, queueSeconds = 0, networkSeconds = 0;
    double computeJoules = 0, communicationJoules = 0;
    double hopCompute = 0, hopEnergy = 0, txWatts = 0;
    std::vector<uint16_t> path;
};
}
#endif
