/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "qrcf-packet.h"
#include <cmath>
#include <cstring>

namespace ns3::qrcf
{
NS_OBJECT_ENSURE_REGISTERED(HelloHeader);
NS_OBJECT_ENSURE_REGISTERED(MessageHeader);
NS_OBJECT_ENSURE_REGISTERED(LearningHeader);
NS_OBJECT_ENSURE_REGISTERED(FeedbackHeader);
NS_OBJECT_ENSURE_REGISTERED(ValueHeader);
namespace
{
void PutDouble(Buffer::Iterator& i, double value)
{
    static_assert(sizeof(double) == sizeof(uint64_t));
    uint64_t bits;
    std::memcpy(&bits,&value,sizeof(bits));
    i.WriteHtonU64(bits);
}
double GetDouble(Buffer::Iterator& i)
{
    const uint64_t bits = i.ReadNtohU64();
    double value;
    std::memcpy(&value,&bits,sizeof(value));
    return value;
}
}
TypeId HelloHeader::GetTypeId()
{
    static TypeId tid = TypeId("ns3::qrcf::HelloHeader").SetParent<Header>()
        .SetGroupName("Qrcf").AddConstructor<HelloHeader>();
    return tid;
}
TypeId HelloHeader::GetInstanceTypeId() const { return GetTypeId(); }
uint32_t HelloHeader::GetSerializedSize() const { return 8+4+8*8; }
void HelloHeader::Serialize(Buffer::Iterator i) const
{
    i.WriteHtonU64(messageId);
    i.WriteHtonU32(nodeId);
    for (double x : {position.x,position.y,position.z,motion.speed,motion.azimuth,
                     motion.polar,energy,helloInterval}) { PutDouble(i,x); }
}
uint32_t HelloHeader::Deserialize(Buffer::Iterator i)
{
    if (i.GetRemainingSize() < GetSerializedSize()) { return 0; }
    messageId = i.ReadNtohU64(); nodeId = i.ReadNtohU32();
    position.x = GetDouble(i); position.y = GetDouble(i); position.z = GetDouble(i);
    motion.speed = GetDouble(i); motion.azimuth = GetDouble(i); motion.polar = GetDouble(i);
    energy = GetDouble(i); helloInterval = GetDouble(i);
    return IsValid() ? GetSerializedSize() : 0;
}
bool HelloHeader::IsValid() const
{
    for (double x : {position.x,position.y,position.z,motion.speed,motion.azimuth,
                     motion.polar,energy,helloInterval})
    { if (!std::isfinite(x)) { return false; } }
    return motion.speed >= 0 && energy >= 0 && helloInterval > 0;
}
void HelloHeader::Print(std::ostream& os) const
{
    os << "QRCF Hello id=" << messageId << " node=" << nodeId << " HT=" << helloInterval;
}
TypeId MessageHeader::GetTypeId()
{
    static TypeId tid = TypeId("ns3::qrcf::MessageHeader").SetParent<Header>()
        .SetGroupName("Qrcf").AddConstructor<MessageHeader>();
    return tid;
}
TypeId MessageHeader::GetInstanceTypeId() const { return GetTypeId(); }
void MessageHeader::Serialize(Buffer::Iterator i) const
{ i.WriteHtonU16(0x5152); i.WriteU8(1); i.WriteU8(static_cast<uint8_t>(kind)); }
uint32_t MessageHeader::Deserialize(Buffer::Iterator i)
{
    if (i.GetRemainingSize() < 4 || i.ReadNtohU16() != 0x5152 || i.ReadU8() != 1) { return 0; }
    kind = static_cast<Kind>(i.ReadU8());
    return kind == Kind::HELLO || kind == Kind::FEEDBACK || kind == Kind::VALUE ? 4 : 0;
}
void MessageHeader::Print(std::ostream& os) const { os << "QRCF type=" << unsigned(kind); }
TypeId LearningHeader::GetTypeId()
{
    static TypeId tid = TypeId("ns3::qrcf::LearningHeader").SetParent<Header>()
        .SetGroupName("Qrcf").AddConstructor<LearningHeader>();
    return tid;
}
TypeId LearningHeader::GetInstanceTypeId() const { return GetTypeId(); }
uint32_t LearningHeader::GetSerializedSize() const { return 10; }
bool LearningHeader::IsValid() const
{
    return std::isfinite(gamma) && gamma >= 0 && gamma <= 1;
}
void LearningHeader::Serialize(Buffer::Iterator i) const
{
    NS_ABORT_MSG_IF(!IsValid(),"Invalid QRCF learning advertisement");
    PutDouble(i,gamma); i.WriteHtonU16(0);
}
uint32_t LearningHeader::Deserialize(Buffer::Iterator i)
{
    if (i.GetRemainingSize() < GetSerializedSize()) { return 0; }
    gamma = GetDouble(i);
    if (i.ReadNtohU16() != 0) { return 0; }
    return IsValid() ? GetSerializedSize() : 0;
}
void LearningHeader::Print(std::ostream& os) const
{ os << "QRCF gamma=" << gamma; }
TypeId FeedbackHeader::GetTypeId()
{
    static TypeId tid = TypeId("ns3::qrcf::FeedbackHeader").SetParent<Header>()
        .SetGroupName("Qrcf").AddConstructor<FeedbackHeader>();
    return tid;
}
TypeId FeedbackHeader::GetInstanceTypeId() const { return GetTypeId(); }
void FeedbackHeader::Serialize(Buffer::Iterator i) const
{ i.WriteHtonU64(dataId); i.WriteHtonU32(destination.Get()); i.WriteU8(static_cast<uint8_t>(reason)); }
uint32_t FeedbackHeader::Deserialize(Buffer::Iterator i)
{
    if (i.GetRemainingSize() < 13) { return 0; }
    dataId = i.ReadNtohU64(); destination = Ipv4Address(i.ReadNtohU32()); reason = static_cast<Reason>(i.ReadU8());
    return reason == Reason::LOCAL_MINIMUM && destination != Ipv4Address::GetAny() &&
        !destination.IsMulticast() && !destination.IsBroadcast() ? 13 : 0;
}
void FeedbackHeader::Print(std::ostream& os) const
{ os << "QRCF feedback data=" << dataId << " destination=" << destination; }
TypeId ValueHeader::GetTypeId()
{
    static TypeId tid = TypeId("ns3::qrcf::ValueHeader").SetParent<Header>()
        .SetGroupName("Qrcf").AddConstructor<ValueHeader>();
    return tid;
}
TypeId ValueHeader::GetInstanceTypeId() const { return GetTypeId(); }
bool ValueHeader::IsValid() const
{
    return !destination.IsAny() && !destination.IsBroadcast() && !destination.IsMulticast() &&
        !destination.IsLocalhost() && std::isfinite(gamma) && gamma >= 0 && gamma <= 1 &&
        std::isfinite(maximumQ) && (status == Status::TRANSIT || status == Status::TERMINAL);
}
void ValueHeader::Serialize(Buffer::Iterator i) const
{
    NS_ABORT_MSG_IF(!IsValid(),"Invalid QRCF single-hop value receipt");
    i.WriteHtonU64(dataId); i.WriteHtonU32(destination.Get());
    PutDouble(i,gamma); PutDouble(i,maximumQ); i.WriteU8(static_cast<uint8_t>(status));
}
uint32_t ValueHeader::Deserialize(Buffer::Iterator i)
{
    if (i.GetRemainingSize() < GetSerializedSize()) { return 0; }
    dataId = i.ReadNtohU64(); destination = Ipv4Address(i.ReadNtohU32());
    gamma = GetDouble(i); maximumQ = GetDouble(i); status = static_cast<Status>(i.ReadU8());
    return IsValid() ? GetSerializedSize() : 0;
}
void ValueHeader::Print(std::ostream& os) const
{
    os << "QRCF value data=" << dataId << " destination=" << destination
       << " gamma=" << gamma << " maximumQ=" << maximumQ << " terminal=" << IsTerminal();
}
} // namespace ns3::qrcf
