/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "cafbcr-packet.h"
#include "ns3/address-utils.h"
#include "ns3/packet.h"
#include "ns3/log.h"

namespace ns3 {
namespace cafbcr {

NS_LOG_COMPONENT_DEFINE("cafbcrPacket");

//-----------------------------------------------------------------------------
// TypeHeader
//-----------------------------------------------------------------------------
TypeHeader::TypeHeader(MessageType t)
  : m_type(t),
    m_valid(true)
{
}

TypeHeader::~TypeHeader()
{
}

TypeId TypeHeader::GetTypeId()
{
  static TypeId tid = TypeId("ns3::cafbcr::TypeHeader")
    .SetParent<Header>()
    .SetGroupName("cafbcr")
    .AddConstructor<TypeHeader>();
  return tid;
}

TypeId TypeHeader::GetInstanceTypeId() const
{
  return GetTypeId();
}

uint32_t TypeHeader::GetSerializedSize() const
{
  return 1; // 1 byte for message type
}

void TypeHeader::Serialize(Buffer::Iterator start) const
{
  start.WriteU8(static_cast<uint8_t>(m_type));
}

uint32_t TypeHeader::Deserialize(Buffer::Iterator start)
{
  uint8_t type = start.ReadU8();
  m_valid = true;
  
  switch(type) {
    case CAFBCR_HELLO:
    case CAFBCR_CH_CALL:
    case CAFBCR_JOIN_REQ:
    case CAFBCR_JOIN_RESP:
    case CAFBCR_CH_LIVE:
    case CAFBCR_CM_LIVE:
    case CAFBCR_PROXY_JOIN_REQ:
    case CAFBCR_RENEWAL_ACK:
    case CAFBCR_LINK_REQUEST:
    case CAFBCR_LINK_REPLY:
      m_type = static_cast<MessageType>(type);
      break;
    default:
      m_valid = false;
  }
  
  return GetSerializedSize();
}

void TypeHeader::Print(std::ostream &os) const
{
  os << "cafbcr MessageType: " << static_cast<uint32_t>(m_type);
}

//-----------------------------------------------------------------------------
// HelloHeader
//-----------------------------------------------------------------------------
HelloHeader::HelloHeader()
  : m_nodeId(0),
    m_clusterId(0xFFFF),
    m_ipAddress(Ipv4Address()),
    m_position(Vector(0, 0, 0)),
    m_velocity(Vector(0, 0, 0)),
    m_residualEnergy(0.0),
    m_fitness(0.0),
    m_role(Idle),
    m_timestamp(0.0),
    m_computeCapacity(1.0),
    m_currentLoad(0.0f),
    m_intentJoinChId(0xFFFF)
{
}

HelloHeader::HelloHeader(uint16_t nodeId, uint16_t clusterId, Ipv4Address ip,
                         Vector pos, Vector vel, double energy, double fitness,
                         NodeRole role, double timestamp)
  : m_nodeId(nodeId),
    m_clusterId(clusterId),
    m_ipAddress(ip),
    m_position(pos),
    m_velocity(vel),
    m_residualEnergy(energy),
    m_fitness(fitness),
    m_role(role),
    m_timestamp(timestamp),
    m_computeCapacity(1.0),
    m_currentLoad(0.0f),
    m_intentJoinChId(0xFFFF)
{
}

HelloHeader::~HelloHeader()
{
}

TypeId HelloHeader::GetTypeId()
{
  static TypeId tid = TypeId("ns3::cafbcr::HelloHeader")
    .SetParent<Header>()
    .SetGroupName("cafbcr")
    .AddConstructor<HelloHeader>();
  return tid;
}

TypeId HelloHeader::GetInstanceTypeId() const
{
  return GetTypeId();
}

uint32_t HelloHeader::GetSerializedSize() const
{
  return 2 + 2 + 4 + 8*3 + 8*3 + 8 + 8 + 1 + 8 + 8 + 4 + 2; // 95 bytes (+ intentJoinChId 2B)
}

void HelloHeader::Serialize(Buffer::Iterator start) const
{
  start.WriteHtonU16(m_nodeId);
  start.WriteHtonU16(m_clusterId);
  WriteTo(start, m_ipAddress);
  start.WriteHtonU64(static_cast<uint64_t>(static_cast<int64_t>(m_position.x * 1000000)));
  start.WriteHtonU64(static_cast<uint64_t>(static_cast<int64_t>(m_position.y * 1000000)));
  start.WriteHtonU64(static_cast<uint64_t>(static_cast<int64_t>(m_position.z * 1000000)));
  start.WriteHtonU64(static_cast<uint64_t>(static_cast<int64_t>(m_velocity.x * 1000000)));
  start.WriteHtonU64(static_cast<uint64_t>(static_cast<int64_t>(m_velocity.y * 1000000)));
  start.WriteHtonU64(static_cast<uint64_t>(static_cast<int64_t>(m_velocity.z * 1000000)));
  start.WriteHtonU64(static_cast<uint64_t>(static_cast<int64_t>(m_residualEnergy * 1000000)));
  start.WriteHtonU64(static_cast<uint64_t>(static_cast<int64_t>(m_fitness * 1000000)));
  start.WriteU8(static_cast<uint8_t>(m_role));
  start.WriteHtonU64(static_cast<uint64_t>(static_cast<int64_t>(m_timestamp * 1000000)));
  // ---- 新增计算能力字段序列化 ----
  start.WriteHtonU64(static_cast<uint64_t>(static_cast<int64_t>(m_computeCapacity * 1000000)));
  start.WriteHtonU32(static_cast<uint32_t>(m_currentLoad * 1000000));
  // ---- 新增 intent 字段 ----
  start.WriteHtonU16(m_intentJoinChId);
}

uint32_t HelloHeader::Deserialize(Buffer::Iterator start)
{
  m_nodeId = start.ReadNtohU16();
  m_clusterId = start.ReadNtohU16();
  ReadFrom(start, m_ipAddress);
  m_position.x = static_cast<double>(static_cast<int64_t>(start.ReadNtohU64())) / 1000000.0;
  m_position.y = static_cast<double>(static_cast<int64_t>(start.ReadNtohU64())) / 1000000.0;
  m_position.z = static_cast<double>(static_cast<int64_t>(start.ReadNtohU64())) / 1000000.0;
  m_velocity.x = static_cast<double>(static_cast<int64_t>(start.ReadNtohU64())) / 1000000.0;
  m_velocity.y = static_cast<double>(static_cast<int64_t>(start.ReadNtohU64())) / 1000000.0;
  m_velocity.z = static_cast<double>(static_cast<int64_t>(start.ReadNtohU64())) / 1000000.0;
  m_residualEnergy = static_cast<double>(static_cast<int64_t>(start.ReadNtohU64())) / 1000000.0;
  m_fitness = static_cast<double>(static_cast<int64_t>(start.ReadNtohU64())) / 1000000.0;
  m_role = static_cast<NodeRole>(start.ReadU8());
  m_timestamp = static_cast<double>(static_cast<int64_t>(start.ReadNtohU64())) / 1000000.0;
  // ---- 新增计算能力字段反序列化 ----
  m_computeCapacity = static_cast<double>(static_cast<int64_t>(start.ReadNtohU64())) / 1000000.0;
  m_currentLoad = static_cast<float>(start.ReadNtohU32()) / 1000000.0f;
  // ---- 新增 intent 字段 ----
  m_intentJoinChId = start.ReadNtohU16();
  return GetSerializedSize();
}

void HelloHeader::Print(std::ostream &os) const
{
  os << "Hello from Node " << m_nodeId << " Cluster " << m_clusterId 
     << " Fitness " << m_fitness;
}

//-----------------------------------------------------------------------------
// ClusterHeadAnnounceHeader
//-----------------------------------------------------------------------------
ClusterHeadAnnounceHeader::ClusterHeadAnnounceHeader()
  : m_nodeId(0),
    m_clusterId(0xFFFF),
    m_ipAddress(Ipv4Address()),
    m_position(Vector(0, 0, 0)),
    m_velocity(Vector(0, 0, 0)),
    m_residualEnergy(0.0),
    m_fitnessValue(0.0),
    m_timestamp(0.0)
{
}

ClusterHeadAnnounceHeader::ClusterHeadAnnounceHeader(uint16_t nodeId, uint16_t clusterId,
                                                     Ipv4Address ip, Vector pos, Vector vel,
                                                     double energy, double fitness, double timestamp)
  : m_nodeId(nodeId),
    m_clusterId(clusterId),
    m_ipAddress(ip),
    m_position(pos),
    m_velocity(vel),
    m_residualEnergy(energy),
    m_fitnessValue(fitness),
    m_timestamp(timestamp)
{
}

ClusterHeadAnnounceHeader::~ClusterHeadAnnounceHeader()
{
}

TypeId ClusterHeadAnnounceHeader::GetTypeId()
{
  static TypeId tid = TypeId("ns3::cafbcr::ClusterHeadAnnounceHeader")
    .SetParent<Header>()
    .SetGroupName("cafbcr")
    .AddConstructor<ClusterHeadAnnounceHeader>();
  return tid;
}

TypeId ClusterHeadAnnounceHeader::GetInstanceTypeId() const
{
  return GetTypeId();
}

uint32_t ClusterHeadAnnounceHeader::GetSerializedSize() const
{
  return 2 + 2 + 4 + 8*3 + 8*3 + 8 + 8 + 8; // 67 bytes
}

void ClusterHeadAnnounceHeader::Serialize(Buffer::Iterator start) const
{
  start.WriteHtonU16(m_nodeId);
  start.WriteHtonU16(m_clusterId);
  WriteTo(start, m_ipAddress);
  start.WriteHtonU64(static_cast<uint64_t>(static_cast<int64_t>(m_position.x * 1000000)));
  start.WriteHtonU64(static_cast<uint64_t>(static_cast<int64_t>(m_position.y * 1000000)));
  start.WriteHtonU64(static_cast<uint64_t>(static_cast<int64_t>(m_position.z * 1000000)));
  start.WriteHtonU64(static_cast<uint64_t>(static_cast<int64_t>(m_velocity.x * 1000000)));
  start.WriteHtonU64(static_cast<uint64_t>(static_cast<int64_t>(m_velocity.y * 1000000)));
  start.WriteHtonU64(static_cast<uint64_t>(static_cast<int64_t>(m_velocity.z * 1000000)));
  start.WriteHtonU64(static_cast<uint64_t>(static_cast<int64_t>(m_residualEnergy * 1000000)));
  start.WriteHtonU64(static_cast<uint64_t>(static_cast<int64_t>(m_fitnessValue * 1000000)));
  start.WriteHtonU64(static_cast<uint64_t>(static_cast<int64_t>(m_timestamp * 1000000)));
}

uint32_t ClusterHeadAnnounceHeader::Deserialize(Buffer::Iterator start)
{
  m_nodeId = start.ReadNtohU16();
  m_clusterId = start.ReadNtohU16();
  ReadFrom(start, m_ipAddress);
  m_position.x = static_cast<double>(static_cast<int64_t>(start.ReadNtohU64())) / 1000000.0;
  m_position.y = static_cast<double>(static_cast<int64_t>(start.ReadNtohU64())) / 1000000.0;
  m_position.z = static_cast<double>(static_cast<int64_t>(start.ReadNtohU64())) / 1000000.0;
  m_velocity.x = static_cast<double>(static_cast<int64_t>(start.ReadNtohU64())) / 1000000.0;
  m_velocity.y = static_cast<double>(static_cast<int64_t>(start.ReadNtohU64())) / 1000000.0;
  m_velocity.z = static_cast<double>(static_cast<int64_t>(start.ReadNtohU64())) / 1000000.0;
  m_residualEnergy = static_cast<double>(static_cast<int64_t>(start.ReadNtohU64())) / 1000000.0;
  m_fitnessValue = static_cast<double>(static_cast<int64_t>(start.ReadNtohU64())) / 1000000.0;
  m_timestamp = static_cast<double>(static_cast<int64_t>(start.ReadNtohU64())) / 1000000.0;
  return GetSerializedSize();
}

void ClusterHeadAnnounceHeader::Print(std::ostream &os) const
{
  os << "CH_CALL from Node " << m_nodeId << " Cluster " << m_clusterId
     << " Fitness " << m_fitnessValue;
}

//-----------------------------------------------------------------------------
// JoinClusterRequestHeader
//-----------------------------------------------------------------------------
JoinClusterRequestHeader::JoinClusterRequestHeader()
  : m_nodeId(0),
    m_ipAddress(Ipv4Address()),
    m_position(Vector(0, 0, 0)),
    m_velocity(Vector(0, 0, 0)),
    m_residualEnergy(0.0),
    m_fitness(0.0),
    m_role(Idle),
    m_timestamp(0.0)
{
}

JoinClusterRequestHeader::JoinClusterRequestHeader(uint16_t nodeId, Ipv4Address ip,
                                                   Vector pos, Vector vel, double energy,
                                                   double fitness, NodeRole role, double timestamp)
  : m_nodeId(nodeId),
    m_ipAddress(ip),
    m_position(pos),
    m_velocity(vel),
    m_residualEnergy(energy),
    m_fitness(fitness),
    m_role(role),
    m_timestamp(timestamp)
{
}

JoinClusterRequestHeader::~JoinClusterRequestHeader()
{
}

TypeId JoinClusterRequestHeader::GetTypeId()
{
  static TypeId tid = TypeId("ns3::cafbcr::JoinClusterRequestHeader")
    .SetParent<Header>()
    .SetGroupName("cafbcr")
    .AddConstructor<JoinClusterRequestHeader>();
  return tid;
}

TypeId JoinClusterRequestHeader::GetInstanceTypeId() const
{
  return GetTypeId();
}

uint32_t JoinClusterRequestHeader::GetSerializedSize() const
{
  return 2 + 4 + 8*3 + 8*3 + 8 + 8 + 1 + 8; // 66 bytes
}

void JoinClusterRequestHeader::Serialize(Buffer::Iterator start) const
{
  start.WriteHtonU16(m_nodeId);
  WriteTo(start, m_ipAddress);
  start.WriteHtonU64(static_cast<uint64_t>(static_cast<int64_t>(m_position.x * 1000000)));
  start.WriteHtonU64(static_cast<uint64_t>(static_cast<int64_t>(m_position.y * 1000000)));
  start.WriteHtonU64(static_cast<uint64_t>(static_cast<int64_t>(m_position.z * 1000000)));
  start.WriteHtonU64(static_cast<uint64_t>(static_cast<int64_t>(m_velocity.x * 1000000)));
  start.WriteHtonU64(static_cast<uint64_t>(static_cast<int64_t>(m_velocity.y * 1000000)));
  start.WriteHtonU64(static_cast<uint64_t>(static_cast<int64_t>(m_velocity.z * 1000000)));
  start.WriteHtonU64(static_cast<uint64_t>(static_cast<int64_t>(m_residualEnergy * 1000000)));
  start.WriteHtonU64(static_cast<uint64_t>(static_cast<int64_t>(m_fitness * 1000000)));
  start.WriteU8(static_cast<uint8_t>(m_role));
  start.WriteHtonU64(static_cast<uint64_t>(static_cast<int64_t>(m_timestamp * 1000000)));
}

uint32_t JoinClusterRequestHeader::Deserialize(Buffer::Iterator start)
{
  m_nodeId = start.ReadNtohU16();
  ReadFrom(start, m_ipAddress);
  m_position.x = static_cast<double>(static_cast<int64_t>(start.ReadNtohU64())) / 1000000.0;
  m_position.y = static_cast<double>(static_cast<int64_t>(start.ReadNtohU64())) / 1000000.0;
  m_position.z = static_cast<double>(static_cast<int64_t>(start.ReadNtohU64())) / 1000000.0;
  m_velocity.x = static_cast<double>(static_cast<int64_t>(start.ReadNtohU64())) / 1000000.0;
  m_velocity.y = static_cast<double>(static_cast<int64_t>(start.ReadNtohU64())) / 1000000.0;
  m_velocity.z = static_cast<double>(static_cast<int64_t>(start.ReadNtohU64())) / 1000000.0;
  m_residualEnergy = static_cast<double>(static_cast<int64_t>(start.ReadNtohU64())) / 1000000.0;
  m_fitness = static_cast<double>(static_cast<int64_t>(start.ReadNtohU64())) / 1000000.0;
  m_role = static_cast<NodeRole>(start.ReadU8());
  m_timestamp = static_cast<double>(static_cast<int64_t>(start.ReadNtohU64())) / 1000000.0;
  return GetSerializedSize();
}

void JoinClusterRequestHeader::Print(std::ostream &os) const
{
  os << "JOIN_REQ from Node " << m_nodeId << " Fitness " << m_fitness;
}

//-----------------------------------------------------------------------------
// ClusterHeadResponseHeader
//-----------------------------------------------------------------------------
ClusterHeadResponseHeader::ClusterHeadResponseHeader()
  : m_chNodeId(0),
    m_clusterId(0xFFFF),
    m_chIpAddress(Ipv4Address()),
    m_timestamp(0.0)
{
}

ClusterHeadResponseHeader::ClusterHeadResponseHeader(uint16_t chNodeId, uint16_t clusterId,
                                                     Ipv4Address chIp, double timestamp)
  : m_chNodeId(chNodeId),
    m_clusterId(clusterId),
    m_chIpAddress(chIp),
    m_timestamp(timestamp)
{
}

ClusterHeadResponseHeader::~ClusterHeadResponseHeader()
{
}

TypeId ClusterHeadResponseHeader::GetTypeId()
{
  static TypeId tid = TypeId("ns3::cafbcr::ClusterHeadResponseHeader")
    .SetParent<Header>()
    .SetGroupName("cafbcr")
    .AddConstructor<ClusterHeadResponseHeader>();
  return tid;
}

TypeId ClusterHeadResponseHeader::GetInstanceTypeId() const
{
  return GetTypeId();
}

uint32_t ClusterHeadResponseHeader::GetSerializedSize() const
{
  return 2 + 2 + 4 + 8; // 16 bytes
}

void ClusterHeadResponseHeader::Serialize(Buffer::Iterator start) const
{
  start.WriteHtonU16(m_chNodeId);
  start.WriteHtonU16(m_clusterId);
  WriteTo(start, m_chIpAddress);
  start.WriteHtonU64(static_cast<uint64_t>(static_cast<int64_t>(m_timestamp * 1000000)));
}

uint32_t ClusterHeadResponseHeader::Deserialize(Buffer::Iterator start)
{
  m_chNodeId = start.ReadNtohU16();
  m_clusterId = start.ReadNtohU16();
  ReadFrom(start, m_chIpAddress);
  m_timestamp = static_cast<double>(static_cast<int64_t>(start.ReadNtohU64())) / 1000000.0;
  return GetSerializedSize();
}

void ClusterHeadResponseHeader::Print(std::ostream &os) const
{
  os << "JOIN_RESP from CH " << m_chNodeId << " Cluster " << m_clusterId;
}

//-----------------------------------------------------------------------------
// ClusterHeadBeaconHeader
//-----------------------------------------------------------------------------
ClusterHeadBeaconHeader::ClusterHeadBeaconHeader()
  : m_chNodeId(0),
    m_clusterId(0xFFFF),
    m_chIpAddress(Ipv4Address()),
    m_position(Vector(0, 0, 0)),
    m_velocity(Vector(0, 0, 0)),
    m_fitness(0.0),
    m_timestamp(0.0)
{
  for (uint8_t i = 0; i < kGrantedMembersCap; ++i) m_grantedMembers[i] = 0xFFFF;
}

ClusterHeadBeaconHeader::ClusterHeadBeaconHeader(uint16_t chNodeId, uint16_t clusterId,
                                                 Ipv4Address chIp, Vector pos, Vector vel,
                                                 double fitness, double timestamp)
  : m_chNodeId(chNodeId),
    m_clusterId(clusterId),
    m_chIpAddress(chIp),
    m_position(pos),
    m_velocity(vel),
    m_fitness(fitness),
    m_timestamp(timestamp)
{
  for (uint8_t i = 0; i < kGrantedMembersCap; ++i) m_grantedMembers[i] = 0xFFFF;
}

ClusterHeadBeaconHeader::~ClusterHeadBeaconHeader()
{
}

TypeId ClusterHeadBeaconHeader::GetTypeId()
{
  static TypeId tid = TypeId("ns3::cafbcr::ClusterHeadBeaconHeader")
    .SetParent<Header>()
    .SetGroupName("cafbcr")
    .AddConstructor<ClusterHeadBeaconHeader>();
  return tid;
}

TypeId ClusterHeadBeaconHeader::GetInstanceTypeId() const
{
  return GetTypeId();
}

uint32_t ClusterHeadBeaconHeader::GetSerializedSize() const
{
  return 2 + 2 + 4 + 8*3 + 8*3 + 8 + 8 + 2 * kGrantedMembersCap; // 72 + 16 = 88 bytes
}

void ClusterHeadBeaconHeader::Serialize(Buffer::Iterator start) const
{
  start.WriteHtonU16(m_chNodeId);
  start.WriteHtonU16(m_clusterId);
  WriteTo(start, m_chIpAddress);
  start.WriteHtonU64(static_cast<uint64_t>(static_cast<int64_t>(m_position.x * 1000000)));
  start.WriteHtonU64(static_cast<uint64_t>(static_cast<int64_t>(m_position.y * 1000000)));
  start.WriteHtonU64(static_cast<uint64_t>(static_cast<int64_t>(m_position.z * 1000000)));
  start.WriteHtonU64(static_cast<uint64_t>(static_cast<int64_t>(m_velocity.x * 1000000)));
  start.WriteHtonU64(static_cast<uint64_t>(static_cast<int64_t>(m_velocity.y * 1000000)));
  start.WriteHtonU64(static_cast<uint64_t>(static_cast<int64_t>(m_velocity.z * 1000000)));
  start.WriteHtonU64(static_cast<uint64_t>(static_cast<int64_t>(m_fitness * 1000000)));
  start.WriteHtonU64(static_cast<uint64_t>(static_cast<int64_t>(m_timestamp * 1000000)));
  for (uint8_t i = 0; i < kGrantedMembersCap; ++i) start.WriteHtonU16(m_grantedMembers[i]);
}

uint32_t ClusterHeadBeaconHeader::Deserialize(Buffer::Iterator start)
{
  m_chNodeId = start.ReadNtohU16();
  m_clusterId = start.ReadNtohU16();
  ReadFrom(start, m_chIpAddress);
  m_position.x = static_cast<double>(static_cast<int64_t>(start.ReadNtohU64())) / 1000000.0;
  m_position.y = static_cast<double>(static_cast<int64_t>(start.ReadNtohU64())) / 1000000.0;
  m_position.z = static_cast<double>(static_cast<int64_t>(start.ReadNtohU64())) / 1000000.0;
  m_velocity.x = static_cast<double>(static_cast<int64_t>(start.ReadNtohU64())) / 1000000.0;
  m_velocity.y = static_cast<double>(static_cast<int64_t>(start.ReadNtohU64())) / 1000000.0;
  m_velocity.z = static_cast<double>(static_cast<int64_t>(start.ReadNtohU64())) / 1000000.0;
  m_fitness = static_cast<double>(static_cast<int64_t>(start.ReadNtohU64())) / 1000000.0;
  m_timestamp = static_cast<double>(static_cast<int64_t>(start.ReadNtohU64())) / 1000000.0;
  for (uint8_t i = 0; i < kGrantedMembersCap; ++i) m_grantedMembers[i] = start.ReadNtohU16();
  return GetSerializedSize();
}

void ClusterHeadBeaconHeader::Print(std::ostream &os) const
{
  os << "CH_LIVE from CH " << m_chNodeId << " Cluster " << m_clusterId;
}

TypeId DirectMaintenanceHeader::GetTypeId()
{
  static TypeId tid = TypeId("ns3::cafbcr::DirectMaintenanceHeader").SetParent<Header>()
    .SetGroupName("cafbcr").AddConstructor<DirectMaintenanceHeader>();
  return tid;
}

bool DirectMaintenanceHeader::AddRecord(uint16_t origin, int64_t sourceUs)
{
  if (m_records.size() == kMaxRecords) return false;
  m_records.push_back({origin, sourceUs});
  m_valid = m_windowUs > 0;
  return true;
}

void DirectMaintenanceHeader::Serialize(Buffer::Iterator start) const
{
  start.WriteU8(0xd1); // extension version, distinct from grants and type-eight ACKs
  start.WriteU8(m_records.size());
  start.WriteHtonU64(static_cast<uint64_t>(m_windowUs));
  for (const auto& record : m_records) {
    start.WriteHtonU16(record.origin);
    start.WriteHtonU64(static_cast<uint64_t>(record.sourceUs));
  }
}

uint32_t DirectMaintenanceHeader::Deserialize(Buffer::Iterator start)
{
  m_valid = false; m_records.clear(); m_windowUs = 0;
  const auto size = start.GetRemainingSize();
  if (size < 10 || start.ReadU8() != 0xd1) return 0;
  const auto count = start.ReadU8();
  if (!count || count > kMaxRecords || size != 10u + 10u * count) return 0;
  m_windowUs = static_cast<int64_t>(start.ReadNtohU64());
  if (m_windowUs <= 0) return 0;
  for (uint8_t i = 0; i < count; ++i)
    m_records.push_back({start.ReadNtohU16(), static_cast<int64_t>(start.ReadNtohU64())});
  m_valid = true;
  return GetSerializedSize();
}

void DirectMaintenanceHeader::Print(std::ostream& os) const
{
  os << "Direct CM coverage window " << m_windowUs << " us, records " << m_records.size();
}

//-----------------------------------------------------------------------------
// ClusterMemberBeaconHeader
//-----------------------------------------------------------------------------
ClusterMemberBeaconHeader::ClusterMemberBeaconHeader()
  : m_nodeId(0),
    m_clusterId(0xFFFF),
    m_ipAddress(Ipv4Address()),
    m_position(Vector(0, 0, 0)),
    m_velocity(Vector(0, 0, 0)),
    m_fitness(0.0),
    m_timestamp(0.0)
{
}

ClusterMemberBeaconHeader::ClusterMemberBeaconHeader(uint16_t nodeId, uint16_t clusterId,
                                                     Ipv4Address ip, Vector pos, Vector vel,
                                                     double fitness, double timestamp)
  : m_nodeId(nodeId),
    m_clusterId(clusterId),
    m_ipAddress(ip),
    m_position(pos),
    m_velocity(vel),
    m_fitness(fitness),
    m_timestamp(timestamp)
{
}

ClusterMemberBeaconHeader::~ClusterMemberBeaconHeader()
{
}

TypeId ClusterMemberBeaconHeader::GetTypeId()
{
  static TypeId tid = TypeId("ns3::cafbcr::ClusterMemberBeaconHeader")
    .SetParent<Header>()
    .SetGroupName("cafbcr")
    .AddConstructor<ClusterMemberBeaconHeader>();
  return tid;
}

TypeId ClusterMemberBeaconHeader::GetInstanceTypeId() const
{
  return GetTypeId();
}

uint32_t ClusterMemberBeaconHeader::GetSerializedSize() const
{
  return 2 + 2 + 4 + 8*3 + 8*3 + 8 + 8; // 59 bytes
}

void ClusterMemberBeaconHeader::Serialize(Buffer::Iterator start) const
{
  start.WriteHtonU16(m_nodeId);
  start.WriteHtonU16(m_clusterId);
  WriteTo(start, m_ipAddress);
  start.WriteHtonU64(static_cast<uint64_t>(static_cast<int64_t>(m_position.x * 1000000)));
  start.WriteHtonU64(static_cast<uint64_t>(static_cast<int64_t>(m_position.y * 1000000)));
  start.WriteHtonU64(static_cast<uint64_t>(static_cast<int64_t>(m_position.z * 1000000)));
  start.WriteHtonU64(static_cast<uint64_t>(static_cast<int64_t>(m_velocity.x * 1000000)));
  start.WriteHtonU64(static_cast<uint64_t>(static_cast<int64_t>(m_velocity.y * 1000000)));
  start.WriteHtonU64(static_cast<uint64_t>(static_cast<int64_t>(m_velocity.z * 1000000)));
  start.WriteHtonU64(static_cast<uint64_t>(static_cast<int64_t>(m_fitness * 1000000)));
  start.WriteHtonU64(static_cast<uint64_t>(static_cast<int64_t>(m_timestamp * 1000000)));
}

uint32_t ClusterMemberBeaconHeader::Deserialize(Buffer::Iterator start)
{
  m_nodeId = start.ReadNtohU16();
  m_clusterId = start.ReadNtohU16();
  ReadFrom(start, m_ipAddress);
  m_position.x = static_cast<double>(static_cast<int64_t>(start.ReadNtohU64())) / 1000000.0;
  m_position.y = static_cast<double>(static_cast<int64_t>(start.ReadNtohU64())) / 1000000.0;
  m_position.z = static_cast<double>(static_cast<int64_t>(start.ReadNtohU64())) / 1000000.0;
  m_velocity.x = static_cast<double>(static_cast<int64_t>(start.ReadNtohU64())) / 1000000.0;
  m_velocity.y = static_cast<double>(static_cast<int64_t>(start.ReadNtohU64())) / 1000000.0;
  m_velocity.z = static_cast<double>(static_cast<int64_t>(start.ReadNtohU64())) / 1000000.0;
  m_fitness = static_cast<double>(static_cast<int64_t>(start.ReadNtohU64())) / 1000000.0;
  m_receivedTimeUs = static_cast<int64_t>(start.ReadNtohU64());
  m_timestamp = static_cast<double>(*m_receivedTimeUs) / 1000000.0;
  return GetSerializedSize();
}

void ClusterMemberBeaconHeader::Print(std::ostream &os) const
{
  os << "CM_LIVE from Node " << m_nodeId << " Cluster " << m_clusterId;
}

TypeId LinkProbeHeader::GetTypeId()
{
  static TypeId tid=TypeId("ns3::cafbcr::LinkProbeHeader").SetParent<Header>()
    .SetGroupName("cafbcr").AddConstructor<LinkProbeHeader>();return tid;
}
void LinkProbeHeader::Serialize(Buffer::Iterator start) const
{
  start.WriteHtonU16(m_origin);start.WriteHtonU16(m_peer);
  start.WriteHtonU64(m_nonce);start.WriteHtonU64(m_sentNs);
}
uint32_t LinkProbeHeader::Deserialize(Buffer::Iterator start)
{
  if (start.GetRemainingSize()<20) return 0;
  m_origin=start.ReadNtohU16();m_peer=start.ReadNtohU16();
  m_nonce=start.ReadNtohU64();m_sentNs=start.ReadNtohU64();return 20;
}
void LinkProbeHeader::Print(std::ostream& os) const
{ os << "LINK origin=" << m_origin << " peer=" << m_peer << " nonce=" << m_nonce << " sent_ns=" << m_sentNs; }

TypeId RenewalAckHeader::GetTypeId()
{
  static TypeId tid = TypeId("ns3::cafbcr::RenewalAckHeader").SetParent<Header>()
    .SetGroupName("cafbcr").AddConstructor<RenewalAckHeader>();
  return tid;
}
void RenewalAckHeader::Serialize(Buffer::Iterator start) const
{
  start.WriteHtonU16(m_origin); start.WriteHtonU16(m_head); start.WriteHtonU16(m_cluster);
  start.WriteHtonU64(static_cast<uint64_t>(m_sourceUs));
}
uint32_t RenewalAckHeader::Deserialize(Buffer::Iterator start)
{
  m_origin = start.ReadNtohU16(); m_head = start.ReadNtohU16(); m_cluster = start.ReadNtohU16();
  m_sourceUs = static_cast<int64_t>(start.ReadNtohU64());
  return GetSerializedSize();
}
void RenewalAckHeader::Print(std::ostream& os) const
{
  os << "RENEWAL_ACK origin=" << m_origin << " head=" << m_head
     << " cluster=" << m_cluster << " source_us=" << m_sourceUs;
}

} // namespace cafbcr
} // namespace ns3
