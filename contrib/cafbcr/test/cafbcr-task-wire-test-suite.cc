/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "ns3/cafbcr-compute-task-app.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/error-model.h"
#include "ns3/string.h"
#include "ns3/test.h"
#include "../../../scratch/ca-fbcr-task-wire-observer.h"
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace ns3::cafbcr {
class TaskTagTestModel : public ErrorModel
{
public:
  bool corruptCache = false;
private:
  bool DoCorrupt(Ptr<Packet> packet) override
  {
    packet->RemoveAllPacketTags();
    if (corruptCache) {
      CafbcrComputeTaskTag cache;
      cache.SetSourceNodeId(42); cache.SetTaskId(987); cache.SetWorkload(99);
      packet->AddPacketTag(cache);
    }
    return false;
  }
  void DoReset() override {}
};

class TaskHeaderCodecCase : public TestCase
{
public:
  TaskHeaderCodecCase() : TestCase("task header has a fixed network-order byte layout without packet tags") {}
private:
  void DoRun() override
  {
    CafbcrComputeTaskTag task;
    task.SetSourceNodeId(0x1234); task.SetTaskId(0x10203040); task.SetWorkload(2.5); task.SetDataSize(64);
    ComputeTaskHeader header;
    const auto deadline = NanoSeconds(0x0102030405060708LL);
    header.SetTask(task, deadline, ComputeTaskHeader::REQUEST);
    auto packet = Create<Packet>(64);
    packet->AddHeader(header);
    std::vector<uint8_t> bytes(packet->GetSize());
    packet->CopyData(bytes.data(), bytes.size());
    std::ostringstream hex;
    for (uint32_t i = 0; i < 32; ++i) hex << std::hex << std::setw(2) << std::setfill('0') << unsigned(bytes[i]);
    NS_TEST_EXPECT_MSG_EQ(hex.str(), "cafc010100201234102030400000004040040000000000000102030405060708", "independent golden bytes");
    auto restored = Create<Packet>(bytes.data(), bytes.size());
    ComputeTaskHeader decoded;
    NS_TEST_EXPECT_MSG_EQ(restored->RemoveHeader(decoded), 32, "raw bytes carry the complete header");
    NS_TEST_EXPECT_MSG_EQ(restored->GetSize(), 64, "metadata is charged separately from payload");
    NS_TEST_EXPECT_MSG_EQ(decoded.GetDeadline(), deadline, "integer nanoseconds survive serialization");
    NS_TEST_EXPECT_MSG_EQ(decoded.ToTag().GetWorkload(), 2.5, "workload decoded from bytes");
    NS_TEST_EXPECT_MSG_EQ(decoded.ToTag().GetSourceNodeId(), 0x1234, "source decoded from bytes");
    for (uint32_t offset : {0U, 2U, 3U, 5U}) {
      auto changed = bytes;
      changed[offset] ^= 0x7f;
      auto invalid = Create<Packet>(changed.data(), changed.size());
      ComputeTaskHeader bad;
      NS_TEST_EXPECT_MSG_EQ(invalid->PeekHeader(bad), 0, "invalid magic version kind or length rejected");
    }
    auto shortPacket = Create<Packet>(bytes.data(), 31);
    ComputeTaskHeader shortHeader;
    NS_TEST_EXPECT_MSG_EQ(shortPacket->PeekHeader(shortHeader), 0, "short buffer is bounded");
  }
};

class TaskWireNetworkCase : public TestCase
{
public:
  enum Kind { BASIC, LEGACY, STRIP, CACHE_DIFFERENT, FRAGMENTED, TOO_LARGE,
              SHORT, BAD_VERSION, WRONG_KIND, PAYLOAD_MISMATCH };
  TaskWireNetworkCase(const char* name, const char* artifact, Kind kind)
    : TestCase(name), m_artifact(artifact), m_kind(kind) {}
private:
  void Frame(Ptr<const Packet> packet) { m_frames.push_back(packet->GetSize()); }
  static void Send(Ptr<Socket> socket, Ptr<Packet> packet) { socket->Send(packet); }
  void DoRun() override
  {
    const char* output = std::getenv("CAFBCR_WIRE_TEST_OUTPUT");
    auto folder = std::filesystem::path(output ? output : "/tmp/cafbcr-task-wire-tests") / m_artifact;
    NS_TEST_ASSERT_MSG_EQ(std::filesystem::exists(folder), false, "retain earlier artifacts");
    std::filesystem::create_directories(folder);
    const bool header = m_kind != LEGACY;
    const bool malformed = m_kind >= SHORT;
    const uint32_t payload = m_kind == FRAGMENTED ? 5000 : (m_kind == TOO_LARGE ? 70000 : 64);
    TaskWireWriter writer(folder.string(), Seconds(1), header);
    NodeContainer nodes;
    nodes.Create(2);
    InternetStackHelper internet;
    internet.Install(nodes);
    PointToPointHelper link;
    link.SetDeviceAttribute("DataRate", StringValue("1Mbps"));
    link.SetChannelAttribute("Delay", TimeValue(MilliSeconds(1)));
    if (m_kind == FRAGMENTED) link.SetDeviceAttribute("Mtu", UintegerValue(400));
    auto devices = link.Install(nodes);
    Ipv4AddressHelper address;
    address.SetBase("10.9.0.0", "255.255.255.0");
    auto ips = address.Assign(devices);
    for (uint32_t i = 0; i < 2; ++i) {
      devices.Get(i)->TraceConnectWithoutContext("PhyRxEnd", MakeCallback(&TaskWireNetworkCase::Frame, this));
      if (m_kind == STRIP || m_kind == CACHE_DIFFERENT || m_kind == FRAGMENTED) {
        auto model = CreateObject<TaskTagTestModel>();
        model->corruptCache = m_kind == CACHE_DIFFERENT;
        DynamicCast<PointToPointNetDevice>(devices.Get(i))->SetReceiveErrorModel(model);
      }
    }
    auto server = CreateObject<ComputeTaskServerApp>();
    server->SetLocalPort(9999); server->SetComputeCapacity(1); server->SetQueueCapacity(10);
    server->SetUseTaskHeader(header); server->SetObservationStop(Seconds(1));
    nodes.Get(0)->AddApplication(server);
    server->SetStartTime(Seconds(0)); server->SetStopTime(Seconds(1) + TimeStep(1));
    writer.Attach(server);
    Ptr<ComputeTaskClientApp> client;
    Ptr<Socket> manual;
    if (!malformed) {
      client = CreateObject<ComputeTaskClientApp>();
      client->SetRemoteAddress(ips.GetAddress(0)); client->SetRemotePort(9999);
      client->SetServerSelectionPolicy(ComputeTaskClientApp::ServerSelectionPolicy::FIXED_ASSIGNMENT);
      client->SetUseTaskHeader(header); client->SetTaskWorkload(.01, .01);
      client->SetTaskDataSize(payload, payload); client->SetTaskDeadline(Seconds(.2), Seconds(.2));
      client->SetTaskInterval(Seconds(5)); client->AssignStreams(3000002);
      client->SetObservationWindow(Seconds(.1), Seconds(.11), Seconds(1));
      nodes.Get(1)->AddApplication(client);
      client->SetStartTime(Seconds(0)); client->SetStopTime(Seconds(1) + TimeStep(1));
      writer.Attach(client);
    } else {
      CafbcrComputeTaskTag task;
      task.SetSourceNodeId(1); task.SetTaskId(7); task.SetWorkload(.01); task.SetDataSize(64); task.SetDeadline(.2);
      ComputeTaskHeader wire;
      wire.SetTask(task, Seconds(.2), m_kind == WRONG_KIND ? ComputeTaskHeader::COMPLETION : ComputeTaskHeader::REQUEST);
      auto packet = Create<Packet>(m_kind == WRONG_KIND ? 8 : (m_kind == PAYLOAD_MISMATCH ? 63 : 64));
      packet->AddHeader(wire);
      std::vector<uint8_t> bytes(packet->GetSize()); packet->CopyData(bytes.data(), bytes.size());
      if (m_kind == BAD_VERSION) bytes[2] = 2;
      if (m_kind == SHORT) bytes.resize(31);
      auto raw = Create<Packet>(bytes.data(), bytes.size());
      raw->AddPacketTag(task); // valid cache must not rescue an invalid wire header
      manual = Socket::CreateSocket(nodes.Get(1), UdpSocketFactory::GetTypeId());
      manual->Connect(InetSocketAddress(ips.GetAddress(0), 9999));
      Simulator::Schedule(Seconds(.1), &Send, manual, raw);
    }
    Simulator::Stop(Seconds(1) + TimeStep(2));
    Simulator::Run();
    writer.Finalize();
    uint64_t expected = malformed || m_kind == TOO_LARGE ? 0 : 1;
    NS_TEST_EXPECT_MSG_EQ(server->GetTasksReceived(), expected, "only valid on-wire requests enter the queue");
    NS_TEST_EXPECT_MSG_EQ(server->GetTasksFinished(), expected, "execution retains task identity and workload");
    if (client) {
      NS_TEST_EXPECT_MSG_EQ(client->GetTaskSummary().generated, 1, "failed submissions remain generated demands");
      NS_TEST_EXPECT_MSG_EQ(client->GetTaskSummary().onTime, expected, "completion decodes from the wire");
      if (expected && m_kind != FRAGMENTED) {
        int64_t expectedNs = header ? 13568000 : 13056000;
        auto response = *client->GetTaskRecords().at(0).acknowledgedAt - Seconds(.1);
        NS_TEST_EXPECT_MSG_EQ(response.GetNanoSeconds(), expectedNs, "independent frame serialization and propagation delay");
      }
    }
    if (!malformed) {
      uint64_t frameBytes = 0;
      for (auto bytes : m_frames) frameBytes += bytes;
      uint64_t expectedFrames = m_kind == TOO_LARGE ? 0 : (m_kind == FRAGMENTED ? 15 : 2);
      uint64_t expectedBytes = m_kind == TOO_LARGE ? 0 :
        (m_kind == FRAGMENTED ? 5418 : (header ? 196 : 132));
      NS_TEST_EXPECT_MSG_EQ(m_frames.size(), expectedFrames, "actual received data frames including IP fragmentation");
      NS_TEST_EXPECT_MSG_EQ(frameBytes, expectedBytes, "payload metadata UDP IPv4 and PPP bytes are actually transmitted");
    }
    std::ofstream evidence(folder / "fixture.json");
    evidence << "{\"header\":" << (header ? "true" : "false") << ",\"payload\":" << payload
      << ",\"malformed\":" << (malformed ? "true" : "false") << ",\"expected_finished\":" << expected
      << ",\"case\":\"" << m_artifact << "\",\"received_frame_bytes\":[";
    for (size_t i = 0; i < m_frames.size(); ++i) { if (i) evidence << ','; evidence << m_frames[i]; }
    evidence << "]}\n"; evidence.close();
    if (manual) manual->Close();
    Simulator::Destroy();
  }
  std::string m_artifact;
  Kind m_kind;
  std::vector<uint32_t> m_frames;
};

class TaskWireSuite : public TestSuite
{
public:
  TaskWireSuite() : TestSuite("cafbcr-task-wire", UNIT)
  {
    using C = TaskWireNetworkCase;
    AddTestCase(new TaskHeaderCodecCase(), TestCase::QUICK);
    AddTestCase(new C("real header contributes request and response link bytes", "header", C::BASIC), TestCase::QUICK);
    AddTestCase(new C("legacy caller keeps its original bytes and timing", "legacy", C::LEGACY), TestCase::QUICK);
    AddTestCase(new C("task completes after all simulation tags are removed", "stripped", C::STRIP), TestCase::QUICK);
    AddTestCase(new C("wire values override inconsistent cached task tags", "cache-different", C::CACHE_DIFFERENT), TestCase::QUICK);
    AddTestCase(new C("fragmented task reassembles its real header without tags", "fragmented", C::FRAGMENTED), TestCase::QUICK);
    AddTestCase(new C("oversized UDP failure retains generated demand and attempted bytes", "oversized", C::TOO_LARGE), TestCase::QUICK);
    AddTestCase(new C("short header cannot fall back to a valid cached tag", "short", C::SHORT), TestCase::QUICK);
    AddTestCase(new C("unsupported wire version cannot fall back to a cached tag", "bad-version", C::BAD_VERSION), TestCase::QUICK);
    AddTestCase(new C("completion message cannot be processed as a request", "wrong-kind", C::WRONG_KIND), TestCase::QUICK);
    AddTestCase(new C("declared payload must equal the received application bytes", "payload-mismatch", C::PAYLOAD_MISMATCH), TestCase::QUICK);
  }
} g_taskWireSuite;
} // namespace ns3::cafbcr
