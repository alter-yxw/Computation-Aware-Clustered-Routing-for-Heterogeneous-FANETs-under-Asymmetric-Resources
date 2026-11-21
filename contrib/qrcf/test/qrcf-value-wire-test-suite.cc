/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "ns3/qrcf-packet.h"
#include "ns3/packet.h"
#include "ns3/test.h"
#include <iomanip>
#include <limits>
#include <sstream>
#include <vector>

using namespace ns3;
using namespace ns3::qrcf;

namespace
{
std::vector<uint8_t> Bytes(Ptr<const Packet> packet)
{
    std::vector<uint8_t> bytes(packet->GetSize());
    packet->CopyData(bytes.data(),bytes.size());
    return bytes;
}
std::string Hex(Ptr<const Packet> packet)
{
    std::ostringstream stream;
    stream << std::hex << std::setfill('0');
    for (const auto byte:Bytes(packet)) { stream << std::setw(2) << unsigned(byte); }
    return stream.str();
}

class LegacyWireCompatibilityTest : public TestCase
{
  public:
    LegacyWireCompatibilityTest():TestCase("value receipts preserve version-1 Hello and feedback wire bytes") {}
    void DoRun() override
    {
        HelloHeader hello;
        hello.messageId=0x0102030405060708ULL; hello.nodeId=0x0a0b0c0d;
        hello.position={0,0,0}; hello.motion={0,0,0}; hello.energy=1; hello.helloInterval=0.5;
        LearningHeader learning; learning.gamma=0.25;
        MessageHeader envelope;
        auto packet=Create<Packet>(); packet->AddHeader(learning); packet->AddHeader(hello);
        packet->AddHeader(envelope);
        NS_TEST_ASSERT_MSG_EQ(packet->GetSize(),90,"Hello remains 90 protocol bytes and 118 IPv4/UDP bytes");
        const std::string expectedHello=
            "51520101" "0102030405060708" "0a0b0c0d"
            "0000000000000000" "0000000000000000" "0000000000000000"
            "0000000000000000" "0000000000000000" "0000000000000000"
            "3ff0000000000000" "3fe0000000000000" "3fd0000000000000" "0000";
        NS_TEST_ASSERT_MSG_EQ(Hex(packet),expectedHello,"Existing Hello version, kind, field order and reserved zero count are unchanged");
        MessageHeader decodedEnvelope; HelloHeader decodedHello; LearningHeader decodedLearning;
        NS_TEST_ASSERT_MSG_EQ(packet->RemoveHeader(decodedEnvelope),4,"Decode old Hello envelope");
        NS_TEST_ASSERT_MSG_EQ((decodedEnvelope.kind==MessageHeader::Kind::HELLO),true,"Old Hello kind remains one");
        NS_TEST_ASSERT_MSG_EQ(packet->RemoveHeader(decodedHello),76,"Decode unchanged literal Hello");
        NS_TEST_ASSERT_MSG_EQ(packet->RemoveHeader(decodedLearning),10,"Decode unchanged gamma extension");
        NS_TEST_ASSERT_MSG_EQ(packet->GetSize(),0,"Consume exactly the legacy frame");

        FeedbackHeader feedback;
        feedback.dataId=0x0102030405060708ULL; feedback.destination=Ipv4Address("10.1.2.3");
        envelope.kind=MessageHeader::Kind::FEEDBACK;
        auto failure=Create<Packet>(); failure->AddHeader(feedback); failure->AddHeader(envelope);
        NS_TEST_ASSERT_MSG_EQ(failure->GetSize(),17,"Feedback remains 17 protocol bytes and 45 IPv4/UDP bytes");
        NS_TEST_ASSERT_MSG_EQ(Hex(failure),"5152010201020304050607080a01020301",
            "Existing feedback version, kind and payload are byte-compatible");
        NS_TEST_ASSERT_MSG_EQ(failure->RemoveHeader(decodedEnvelope),4,"Decode old feedback envelope");
        FeedbackHeader decodedFeedback;
        NS_TEST_ASSERT_MSG_EQ(failure->RemoveHeader(decodedFeedback),13,"Decode unchanged feedback payload");
        NS_TEST_ASSERT_MSG_EQ(decodedFeedback.destination,feedback.destination,"Feedback destination roundtrip");
    }
};

class ValueWireTest : public TestCase
{
  public:
    ValueWireTest():TestCase("single-hop value receipts encode one destination and a finite signed value") {}
    void DoRun() override
    {
        ValueHeader value;
        value.dataId=0x0102030405060708ULL; value.destination=Ipv4Address("10.1.2.3");
        value.gamma=0.75; value.maximumQ=-2.5;
        MessageHeader envelope; envelope.kind=MessageHeader::Kind::VALUE;
        auto packet=Create<Packet>(); packet->AddHeader(value); packet->AddHeader(envelope);
        NS_TEST_ASSERT_MSG_EQ(value.GetSerializedSize(),29,"Receipt payload contains only one action's next-state value");
        NS_TEST_ASSERT_MSG_EQ(packet->GetSize()+28,61,"Receipt plus envelope and UDP/IPv4 costs exactly 61 L3 bytes");
        const std::string expected=
            "51520103" "0102030405060708" "0a010203" "3fe8000000000000" "c004000000000000" "00";
        NS_TEST_ASSERT_MSG_EQ(Hex(packet),expected,"Network-order token, destination, gamma, signed Q and transit status");
        MessageHeader decodedEnvelope; ValueHeader decoded;
        NS_TEST_ASSERT_MSG_EQ(packet->RemoveHeader(decodedEnvelope),4,"Every mode can recognize the new envelope kind");
        NS_TEST_ASSERT_MSG_EQ((decodedEnvelope.kind==MessageHeader::Kind::VALUE),true,"Value kind is three within version one");
        NS_TEST_ASSERT_MSG_EQ(packet->RemoveHeader(decoded),29,"Decode complete receipt");
        NS_TEST_ASSERT_MSG_EQ(packet->GetSize(),0,"One receipt contains no Q-table list or extra data");
        NS_TEST_ASSERT_MSG_EQ(decoded.dataId,value.dataId,"Preserve the 64-bit per-hop action identifier");
        NS_TEST_ASSERT_MSG_EQ(decoded.destination,value.destination,"Preserve destination context");
        NS_TEST_ASSERT_MSG_EQ_TOL(decoded.gamma,0.75,1e-12,"Gamma roundtrip");
        NS_TEST_ASSERT_MSG_EQ_TOL(decoded.maximumQ,-2.5,1e-12,"Negative next-state values remain valid");
        NS_TEST_ASSERT_MSG_EQ(decoded.IsTerminal(),false,"Transit status roundtrip");

        value.status=ValueHeader::Status::TERMINAL; value.gamma=1; value.maximumQ=0;
        auto terminal=Create<Packet>(); terminal->AddHeader(value);
        NS_TEST_ASSERT_MSG_EQ(terminal->RemoveHeader(decoded),29,"Terminal receipt has the same bounded wire size");
        NS_TEST_ASSERT_MSG_EQ(decoded.IsTerminal(),true,"Terminal status roundtrip");
        value.gamma=0;
        NS_TEST_ASSERT_MSG_EQ(value.IsValid(),true,"Gamma accepts both interval boundaries");
    }
};

class InvalidValueWireTest : public TestCase
{
  public:
    InvalidValueWireTest():TestCase("truncated and malformed value receipts are rejected before learning") {}
    void DoRun() override
    {
        ValueHeader valid;
        valid.destination=Ipv4Address("10.1.2.3"); valid.gamma=0.5; valid.maximumQ=-1;
        auto packet=Create<Packet>(); packet->AddHeader(valid);
        const auto encoded=Bytes(packet);
        for (uint32_t length=0;length<29;++length)
        {
            auto shortPacket=Create<Packet>(encoded.data(),length); ValueHeader decoded;
            NS_TEST_ASSERT_MSG_EQ(shortPacket->RemoveHeader(decoded),0,"Reject receipt truncated to " << length << " bytes");
        }
        const auto rejects=[](const std::vector<uint8_t>& bytes) {
            auto malformed=Create<Packet>(bytes.data(),bytes.size()); ValueHeader decoded;
            return malformed->RemoveHeader(decoded)==0;
        };
        auto bad=encoded; bad[28]=2;
        NS_TEST_ASSERT_MSG_EQ(rejects(bad),true,"Only transit=0 and terminal=1 are accepted statuses");
        bad=encoded; bad[12]=0x7f; bad[13]=0xf8;
        NS_TEST_ASSERT_MSG_EQ(rejects(bad),true,"Reject a wire NaN gamma");
        bad=encoded; bad[12]=0x3f; bad[13]=0xf8;
        NS_TEST_ASSERT_MSG_EQ(rejects(bad),true,"Reject a wire gamma above one");
        bad=encoded; bad[20]=0x7f; bad[21]=0xf0;
        for (uint32_t i=22;i<28;++i) { bad[i]=0; }
        NS_TEST_ASSERT_MSG_EQ(rejects(bad),true,"Reject an infinite wire maximum Q");
        for (const auto destination:{Ipv4Address::GetAny(),Ipv4Address::GetBroadcast(),
                                     Ipv4Address("224.1.2.3"),Ipv4Address::GetLoopback()})
        {
            bad=encoded;
            const auto address=destination.Get();
            for (uint32_t i=0;i<4;++i) { bad[8+i]=static_cast<uint8_t>(address>>(24-8*i)); }
            NS_TEST_ASSERT_MSG_EQ(rejects(bad),true,"Reject a non-unicast or local-only receipt destination");
        }
        for (double gamma:{-0.01,1.01,std::numeric_limits<double>::quiet_NaN(),
                            std::numeric_limits<double>::infinity()})
        {
            auto invalid=valid; invalid.gamma=gamma;
            NS_TEST_ASSERT_MSG_EQ(invalid.IsValid(),false,"Sender validation rejects an invalid gamma");
        }
        for (double q:{std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity(),
                        -std::numeric_limits<double>::infinity()})
        {
            auto invalid=valid; invalid.maximumQ=q;
            NS_TEST_ASSERT_MSG_EQ(invalid.IsValid(),false,"Sender validation rejects nonfinite Q values");
        }

        MessageHeader envelope; envelope.kind=MessageHeader::Kind::VALUE;
        auto frame=Create<Packet>(); frame->AddHeader(envelope); auto prefix=Bytes(frame);
        for (uint32_t length=0;length<4;++length)
        {
            auto truncated=Create<Packet>(prefix.data(),length); MessageHeader decoded;
            NS_TEST_ASSERT_MSG_EQ(truncated->RemoveHeader(decoded),0,"Reject truncated message envelope");
        }
        prefix[3]=4;
        auto unknown=Create<Packet>(prefix.data(),prefix.size()); MessageHeader decoded;
        NS_TEST_ASSERT_MSG_EQ(unknown->RemoveHeader(decoded),0,"Unknown message kinds remain rejected");
    }
};

class ValueWireSuite : public TestSuite
{
  public:
    ValueWireSuite():TestSuite("qrcf-value-wire",UNIT)
    {
        AddTestCase(new LegacyWireCompatibilityTest,TestCase::QUICK);
        AddTestCase(new ValueWireTest,TestCase::QUICK);
        AddTestCase(new InvalidValueWireTest,TestCase::QUICK);
    }
} g_valueWireSuite;
} // namespace
