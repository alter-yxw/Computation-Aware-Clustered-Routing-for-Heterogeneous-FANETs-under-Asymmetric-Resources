/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef CAFBCR_PACKET_H
#define CAFBCR_PACKET_H

#include "ns3/header.h"
#include "ns3/ipv4-address.h"
#include "ns3/nstime.h"
#include "ns3/vector.h"
#include "ns3/fanet.h"
#include <optional>
#include <vector>

namespace ns3 {
namespace cafbcr {

/**
 * \brief CA-FBCR message types.
 */
enum MessageType {
  CAFBCR_HELLO = 1,           ///< Hello beacon.
  CAFBCR_CH_CALL = 2,         ///< Cluster-head announcement.
  CAFBCR_JOIN_REQ = 3,        ///< Cluster-join request (direct unicast to a reachable CH).
  CAFBCR_JOIN_RESP = 4,       ///< Cluster-join response.
  CAFBCR_CH_LIVE = 5,         ///< Cluster-head beacon.
  CAFBCR_CM_LIVE = 6,         ///< Cluster-member beacon.
  CAFBCR_PROXY_JOIN_REQ = 7,  ///< Proxy join: a relay sends it to a CH on behalf of a distant SWARM node.
  CAFBCR_RENEWAL_ACK = 8,    ///< CH confirmation of a received source observation
  CAFBCR_LINK_REQUEST = 9,
  CAFBCR_LINK_REPLY = 10
};

/**
 * \brief Message type header.
 */
class TypeHeader : public Header
{
public:
  TypeHeader(MessageType t = CAFBCR_HELLO);
  virtual ~TypeHeader();

  static TypeId GetTypeId();
  TypeId GetInstanceTypeId() const override;
  uint32_t GetSerializedSize() const override;
  void Serialize(Buffer::Iterator start) const override;
  uint32_t Deserialize(Buffer::Iterator start) override;
  void Print(std::ostream &os) const override;

  MessageType Get() const { return m_type; }
  void Set(MessageType t) { m_type = t; }
  bool IsValid() const { return m_valid; }

private:
  MessageType m_type;
  bool m_valid;
};

/**
 * \brief Hello header.
 *
 * Carries node position, velocity, energy, fitness and compute state.
 */
class HelloHeader : public Header
{
public:
  HelloHeader();
  HelloHeader(uint16_t nodeId, uint16_t clusterId, Ipv4Address ip,
              Vector pos, Vector vel, double energy, double fitness,
              NodeRole role, double timestamp);
  virtual ~HelloHeader();

  static TypeId GetTypeId();
  TypeId GetInstanceTypeId() const override;
  uint32_t GetSerializedSize() const override;
  void Serialize(Buffer::Iterator start) const override;
  uint32_t Deserialize(Buffer::Iterator start) override;
  void Print(std::ostream &os) const override;

  // Getters
  uint16_t GetNodeId() const { return m_nodeId; }
  uint16_t GetClusterId() const { return m_clusterId; }
  Ipv4Address GetIpAddress() const { return m_ipAddress; }
  Vector GetPosition() const { return m_position; }
  Vector GetVelocity() const { return m_velocity; }
  double GetResidualEnergy() const { return m_residualEnergy; }
  double GetFitness() const { return m_fitness; }
  NodeRole GetRole() const { return m_role; }
  double GetTimeStamp() const { return m_timestamp; }

  // Setters
  void SetNodeId(uint16_t id) { m_nodeId = id; }
  void SetClusterId(uint16_t cid) { m_clusterId = cid; }
  void SetIpAddress(Ipv4Address ip) { m_ipAddress = ip; }
  void SetPosition(Vector pos) { m_position = pos; }
  void SetVelocity(Vector vel) { m_velocity = vel; }
  void SetResidualEnergy(double energy) { m_residualEnergy = energy; }
  void SetFitness(double fit) { m_fitness = fit; }
  void SetRole(NodeRole role) { m_role = role; }
  void SetTimeStamp(double ts) { m_timestamp = ts; }

  // ---- Compute-state accessors ----
  double GetComputeCapacity() const { return m_computeCapacity; }
  void SetComputeCapacity(double cap) { m_computeCapacity = cap; }
  float GetCurrentLoad() const { return m_currentLoad; }
  void SetCurrentLoad(float load) { m_currentLoad = load; }

  // ---- Asymmetric-link join piggyback ----
  // An Idle node advertises the CH it wants to join; a relay within broadcast
  // range that can reach that CH then sends PROXY_JOIN_REQ. 0xFFFF means no intent.
  uint16_t GetIntentJoinChId() const { return m_intentJoinChId; }
  void SetIntentJoinChId(uint16_t id) { m_intentJoinChId = id; }

private:
  uint16_t m_nodeId;
  uint16_t m_clusterId;
  Ipv4Address m_ipAddress;
  Vector m_position;
  Vector m_velocity;
  double m_residualEnergy;
  double m_fitness;
  NodeRole m_role;
  double m_timestamp;
  // ---- Compute state ----
  double m_computeCapacity;  // 8 bytes
  float m_currentLoad;       // 4 bytes
  // ---- Asymmetric join piggyback ----
  uint16_t m_intentJoinChId; // 2 bytes
};

/**
 * \brief Cluster-head announcement header.
 *
 * A cluster-head candidate broadcasts its state.
 */
class ClusterHeadAnnounceHeader : public Header
{
public:
  ClusterHeadAnnounceHeader();
  ClusterHeadAnnounceHeader(uint16_t nodeId, uint16_t clusterId, Ipv4Address ip,
                            Vector pos, Vector vel, double energy, double fitness, double timestamp);
  virtual ~ClusterHeadAnnounceHeader();

  static TypeId GetTypeId();
  TypeId GetInstanceTypeId() const override;
  uint32_t GetSerializedSize() const override;
  void Serialize(Buffer::Iterator start) const override;
  uint32_t Deserialize(Buffer::Iterator start) override;
  void Print(std::ostream &os) const override;

  // Getters
  uint16_t GetNodeId() const { return m_nodeId; }
  uint16_t GetClusterId() const { return m_clusterId; }
  Ipv4Address GetIpAddress() const { return m_ipAddress; }
  Vector GetPosition() const { return m_position; }
  Vector GetVelocity() const { return m_velocity; }
  double GetResidualEnergy() const { return m_residualEnergy; }
  double GetFitnessValue() const { return m_fitnessValue; }
  double GetTimeStamp() const { return m_timestamp; }

private:
  uint16_t m_nodeId;
  uint16_t m_clusterId;
  Ipv4Address m_ipAddress;
  Vector m_position;
  Vector m_velocity;
  double m_residualEnergy;
  double m_fitnessValue;
  double m_timestamp;
};

/**
 * \brief Cluster-join request header.
 *
 * A regular node asks a cluster head to join its cluster.
 */
class JoinClusterRequestHeader : public Header
{
public:
  JoinClusterRequestHeader();
  JoinClusterRequestHeader(uint16_t nodeId, Ipv4Address ip, Vector pos,
                           Vector vel, double energy, double fitness,
                           NodeRole role, double timestamp);
  virtual ~JoinClusterRequestHeader();

  static TypeId GetTypeId();
  TypeId GetInstanceTypeId() const override;
  uint32_t GetSerializedSize() const override;
  void Serialize(Buffer::Iterator start) const override;
  uint32_t Deserialize(Buffer::Iterator start) override;
  void Print(std::ostream &os) const override;

  // Getters
  uint16_t GetNodeId() const { return m_nodeId; }
  Ipv4Address GetIpAddress() const { return m_ipAddress; }
  Vector GetPosition() const { return m_position; }
  Vector GetVelocity() const { return m_velocity; }
  double GetResidualEnergy() const { return m_residualEnergy; }
  double GetFitness() const { return m_fitness; }
  NodeRole GetRole() const { return m_role; }
  double GetTimeStamp() const { return m_timestamp; }

private:
  uint16_t m_nodeId;
  Ipv4Address m_ipAddress;
  Vector m_position;
  Vector m_velocity;
  double m_residualEnergy;
  double m_fitness;
  NodeRole m_role;
  double m_timestamp;
};

/**
 * \brief Cluster-join response header.
 *
 * A cluster head confirms membership to the requesting node.
 */
class ClusterHeadResponseHeader : public Header
{
public:
  ClusterHeadResponseHeader();
  ClusterHeadResponseHeader(uint16_t chNodeId, uint16_t clusterId,
                            Ipv4Address chIp, double timestamp);
  virtual ~ClusterHeadResponseHeader();

  static TypeId GetTypeId();
  TypeId GetInstanceTypeId() const override;
  uint32_t GetSerializedSize() const override;
  void Serialize(Buffer::Iterator start) const override;
  uint32_t Deserialize(Buffer::Iterator start) override;
  void Print(std::ostream &os) const override;

  // Getters
  uint16_t GetChNodeId() const { return m_chNodeId; }
  uint16_t GetClusterId() const { return m_clusterId; }
  Ipv4Address GetChIpAddress() const { return m_chIpAddress; }
  double GetTimeStamp() const { return m_timestamp; }

private:
  uint16_t m_chNodeId;
  uint16_t m_clusterId;
  Ipv4Address m_chIpAddress;
  double m_timestamp;
};

/**
 * \brief Cluster-head beacon header.
 *
 * Sent periodically by a cluster head to maintain the cluster structure.
 */
class ClusterHeadBeaconHeader : public Header
{
public:
  ClusterHeadBeaconHeader();
  ClusterHeadBeaconHeader(uint16_t chNodeId, uint16_t clusterId, Ipv4Address chIp,
                          Vector pos, Vector vel, double fitness, double timestamp);
  virtual ~ClusterHeadBeaconHeader();

  static TypeId GetTypeId();
  TypeId GetInstanceTypeId() const override;
  uint32_t GetSerializedSize() const override;
  void Serialize(Buffer::Iterator start) const override;
  uint32_t Deserialize(Buffer::Iterator start) override;
  void Print(std::ostream &os) const override;

  // Getters
  uint16_t GetChNodeId() const { return m_chNodeId; }
  uint16_t GetClusterId() const { return m_clusterId; }
  Ipv4Address GetChIpAddress() const { return m_chIpAddress; }
  Vector GetPosition() const { return m_position; }
  Vector GetVelocity() const { return m_velocity; }
  double GetFitness() const { return m_fitness; }
  double GetTimeStamp() const { return m_timestamp; }

  // ---- Asymmetric join piggyback: recently granted member IDs ----
  // The CH piggybacks this list in its broadcast; a distant SWARM node
  // confirms membership on seeing its own ID. 0xFFFF marks an empty slot;
  // at most eight entries.
  static constexpr uint8_t kGrantedMembersCap = 8;
  void SetGrantedMember(uint8_t idx, uint16_t nodeId) {
    if (idx < kGrantedMembersCap) m_grantedMembers[idx] = nodeId;
  }
  uint16_t GetGrantedMember(uint8_t idx) const {
    return (idx < kGrantedMembersCap) ? m_grantedMembers[idx] : 0xFFFF;
  }

private:
  uint16_t m_chNodeId;
  uint16_t m_clusterId;
  Ipv4Address m_chIpAddress;
  Vector m_position;
  Vector m_velocity;
  double m_fitness;
  double m_timestamp;
  uint16_t m_grantedMembers[8];  // 16 bytes
};

// Optional CH_LIVE tail, emitted only by the direct-feedback candidate. The
// unmodified CH header supplies head/cluster identity; each bounded record is
// evidence of a real direct CM, using its exact source-side wire timestamp.
class DirectMaintenanceHeader : public Header
{
public:
  struct Record { uint16_t origin; int64_t sourceUs; };
  static constexpr uint8_t kMaxRecords = 8;
  explicit DirectMaintenanceHeader(int64_t windowUs = 0) : m_windowUs(windowUs) {}
  static TypeId GetTypeId();
  TypeId GetInstanceTypeId() const override { return GetTypeId(); }
  uint32_t GetSerializedSize() const override { return 10 + 10 * m_records.size(); }
  void Serialize(Buffer::Iterator start) const override;
  uint32_t Deserialize(Buffer::Iterator start) override;
  void Print(std::ostream& os) const override;
  bool AddRecord(uint16_t origin, int64_t sourceUs);
  const std::vector<Record>& GetRecords() const { return m_records; }
  int64_t GetWindowUs() const { return m_windowUs; }
  bool IsValid() const { return m_valid; }
private:
  std::vector<Record> m_records;
  int64_t m_windowUs = 0;
  bool m_valid = false;
};

/**
 * \brief Cluster-member beacon header.
 *
 * A cluster member reports its state to the cluster head.
 */
class ClusterMemberBeaconHeader : public Header
{
public:
  ClusterMemberBeaconHeader();
  ClusterMemberBeaconHeader(uint16_t nodeId, uint16_t clusterId, Ipv4Address ip,
                            Vector pos, Vector vel, double fitness, double timestamp);
  virtual ~ClusterMemberBeaconHeader();

  static TypeId GetTypeId();
  TypeId GetInstanceTypeId() const override;
  uint32_t GetSerializedSize() const override;
  void Serialize(Buffer::Iterator start) const override;
  uint32_t Deserialize(Buffer::Iterator start) override;
  void Print(std::ostream &os) const override;

  // Getters
  uint16_t GetNodeId() const { return m_nodeId; }
  uint16_t GetClusterId() const { return m_clusterId; }
  Ipv4Address GetIpAddress() const { return m_ipAddress; }
  Vector GetPosition() const { return m_position; }
  Vector GetVelocity() const { return m_velocity; }
  double GetFitness() const { return m_fitness; }
  double GetTimeStamp() const { return m_timestamp; }
  // The exact signed integer received on the wire, without another floating
  // point conversion. Absent on a newly constructed, not-yet-decoded header.
  std::optional<int64_t> GetReceivedTimeStampUs() const { return m_receivedTimeUs; }

private:
  uint16_t m_nodeId;
  uint16_t m_clusterId;
  Ipv4Address m_ipAddress;
  Vector m_position;
  Vector m_velocity;
  double m_fitness;
  double m_timestamp;
  std::optional<int64_t> m_receivedTimeUs;
};

// Real one-hop request/reply fields. The nonce belongs to the source node.
class LinkProbeHeader : public Header
{
public:
  LinkProbeHeader(uint16_t origin=0xffff, uint16_t peer=0xffff, uint64_t nonce=0, uint64_t sentNs=0)
    : m_origin(origin),m_peer(peer),m_nonce(nonce),m_sentNs(sentNs) {}
  static TypeId GetTypeId();
  TypeId GetInstanceTypeId() const override { return GetTypeId(); }
  uint32_t GetSerializedSize() const override { return 20; }
  void Serialize(Buffer::Iterator start) const override;
  uint32_t Deserialize(Buffer::Iterator start) override;
  void Print(std::ostream& os) const override;
  uint16_t GetOrigin() const { return m_origin; }
  uint16_t GetPeer() const { return m_peer; }
  uint64_t GetNonce() const { return m_nonce; }
  uint64_t GetSentNs() const { return m_sentNs; }
private:
  uint16_t m_origin,m_peer;
  uint64_t m_nonce,m_sentNs;
};

// Feedback identifies a source observation, not a relay or simulator packet.
// All fields are real control bytes; no global member lookup is needed by a relay.
class RenewalAckHeader : public Header
{
public:
  RenewalAckHeader(uint16_t origin = 0xffff, uint16_t head = 0xffff,
                   uint16_t cluster = 0xffff, int64_t sourceUs = 0)
    : m_origin(origin), m_head(head), m_cluster(cluster), m_sourceUs(sourceUs) {}
  static TypeId GetTypeId();
  TypeId GetInstanceTypeId() const override { return GetTypeId(); }
  uint32_t GetSerializedSize() const override { return 14; }
  void Serialize(Buffer::Iterator start) const override;
  uint32_t Deserialize(Buffer::Iterator start) override;
  void Print(std::ostream& os) const override;
  uint16_t GetOrigin() const { return m_origin; }
  uint16_t GetHead() const { return m_head; }
  uint16_t GetCluster() const { return m_cluster; }
  int64_t GetSourceUs() const { return m_sourceUs; }
private:
  uint16_t m_origin, m_head, m_cluster;
  int64_t m_sourceUs;
};

} // namespace cafbcr
} // namespace ns3

#endif /* CAFBCR_PACKET_H */
