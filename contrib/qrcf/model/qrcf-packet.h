/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef QRCF_PACKET_H
#define QRCF_PACKET_H
#include "qrcf-model.h"
#include "ns3/header.h"
#include "ns3/ipv4-address.h"

namespace ns3::qrcf
{
// Literal Equation (2). Transport source address identifies the neighbor.
// No free gamma/Q/neighbor-set metadata is silently appended to this header.
class HelloHeader : public Header
{
  public:
    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const override;
    uint32_t GetSerializedSize() const override;
    void Serialize(Buffer::Iterator) const override;
    uint32_t Deserialize(Buffer::Iterator) override;
    void Print(std::ostream&) const override;
    bool IsValid() const;

    uint64_t messageId = 0; // unique node/sequence composition is a caller concern
    uint32_t nodeId = 0;
    Vector position;
    Motion motion{0,0,0};
    double energy = 0;
    double helloInterval = 0;
};

// Explicitly approved wire extensions. Their bytes are part of control cost.
class MessageHeader : public Header
{
  public:
    // Version 1 keeps the original HELLO/FEEDBACK bytes and adds one kind.
    enum class Kind : uint8_t { HELLO = 1, FEEDBACK = 2, VALUE = 3 };
    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const override;
    uint32_t GetSerializedSize() const override { return 4; }
    void Serialize(Buffer::Iterator) const override;
    uint32_t Deserialize(Buffer::Iterator) override;
    void Print(std::ostream&) const override;
    Kind kind = Kind::HELLO;
};
// Fixed gamma extension. The former zero entry count is reserved on the wire
// to preserve the approved no-Q Hello format; Q entries are not supported.
class LearningHeader : public Header
{
  public:
    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const override;
    uint32_t GetSerializedSize() const override;
    void Serialize(Buffer::Iterator) const override;
    uint32_t Deserialize(Buffer::Iterator) override;
    void Print(std::ostream&) const override;
    bool IsValid() const;
    double gamma = 0;
};
class FeedbackHeader : public Header
{
  public:
    enum class Reason : uint8_t { LOCAL_MINIMUM = 1 };
    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const override;
    uint32_t GetSerializedSize() const override { return 13; }
    void Serialize(Buffer::Iterator) const override;
    uint32_t Deserialize(Buffer::Iterator) override;
    void Print(std::ostream&) const override;
    uint64_t dataId = 0;
    Ipv4Address destination;
    Reason reason = Reason::LOCAL_MINIMUM;
};
// One real, single-hop receipt for one pending data action. This carries only
// the next state's value for the same destination, never a remote Q table.
class ValueHeader : public Header
{
  public:
    enum class Status : uint8_t { TRANSIT = 0, TERMINAL = 1 };
    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const override;
    uint32_t GetSerializedSize() const override { return 29; }
    void Serialize(Buffer::Iterator) const override;
    uint32_t Deserialize(Buffer::Iterator) override;
    void Print(std::ostream&) const override;
    bool IsValid() const;
    bool IsTerminal() const { return status == Status::TERMINAL; }

    uint64_t dataId = 0;
    Ipv4Address destination;
    double gamma = 0;
    double maximumQ = 0;
    Status status = Status::TRANSIT;
};
} // namespace ns3::qrcf
#endif
