/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
/* Packet header definitions for the clustering routing family. */

// #include <bit>
// #include <cstring>
#include "icra-packet.h"
#include "ns3/address-utils.h"
#include "ns3/packet.h"
#include "ns3/log.h"
#include <algorithm>
#include <cmath>
#include <cstring>

NS_LOG_COMPONENT_DEFINE ("IcraPacket");

static uint64_t DoubleToUint64(double value){
  uint64_t result;
  std::memcpy(&result, &value, sizeof(double)); // 将 double 转换成 uint64_t
  return result;
}

static double Uint64ToDouble(uint64_t value){
  double result;
  std::memcpy(&result, &value, sizeof(double)); // 将 uint64_t 转换回 double
  return result;
}

static uint32_t FloatToUint32(float value){
  uint32_t result;
  std::memcpy(&result, &value, sizeof(float)); // 将 float 转换成 uint32_t
  return result;
}

static double Uint32ToFloat(uint32_t value){
  double result;
  std::memcpy(&result, &value, sizeof(float)); // 将 uint32_t 转换回 float
  return result;
}

namespace ns3{
namespace icra{

/*Type Section*/

NS_OBJECT_ENSURE_REGISTERED (TypeHeader);


TypeHeader::TypeHeader( MessageType t)
  : m_type (t),
    m_valid (true)
{}

TypeId TypeHeader::GetTypeId(){
  static TypeId tid = TypeId("ns3::icra::TypeHeader")
  .SetParent<Header> ()
  .AddConstructor<TypeHeader> ();
  return tid;
}

TypeId TypeHeader::GetInstanceTypeId() const
{ 
  return GetTypeId();
}

uint32_t TypeHeader::GetSerializedSize() const 
{ 
  return 1; // Base Header contributes one byte.
}

void TypeHeader::Serialize (Buffer::Iterator i) const
{
  i.WriteU8((uint8_t) m_type);
}

uint32_t TypeHeader::Deserialize (Buffer::Iterator start){
  if (start.GetRemainingSize() < 1) { m_valid = false; return 0; }
  Buffer::Iterator i = start;
  uint8_t type = i.ReadU8();
  m_valid = true;
  switch (type)
  {
    case ICRA_HELLO:
    case ICRA_CH_CALL:
    case ICRA_JOIN_REQ:
    case ICRA_JOIN_RESP:
    case ICRA_CH_LIVE:
    case ICRA_CM_LIVE:
    case ICRA_UTILITY:
    case ICRA_GS_REPORT:
    case ICRA_GS_POLICY:
    case ICRA_PAPER_CONTROL:
    {
      m_type = (MessageType) type;
      break;
    }
    default:
      m_valid = false;
  }
  uint32_t dist = i.GetDistanceFrom (start);
  NS_ASSERT (dist == GetSerializedSize());
  return dist;
}

void TypeHeader::Print (std::ostream & os) const
{
  switch (m_type)
  {
    case ICRA_HELLO:
        { os << "HELLO";  break;}
    case ICRA_CH_CALL:
        { os << "CH_ANNOUNCE"; break;}
    case ICRA_JOIN_REQ:
        { os << "JOIN_REQ"; break;}
    case ICRA_JOIN_RESP:
        { os << "JOIN_ACK"; break;}
    case ICRA_CH_LIVE:
        { os << "CH_BEACON"; break;}
    case ICRA_CM_LIVE:
        { os << "CM_BEACON"; break;}
    case ICRA_UTILITY:
        { os << "UTILITY"; break;}
    case ICRA_GS_REPORT:
        { os << "GS_REPORT"; break;}
    case ICRA_GS_POLICY:
        { os << "GS_POLICY"; break;}
    case ICRA_PAPER_CONTROL:
        { os << "PAPER_CONTROL"; break;}
    default:
        os << "UNDEFINED_TYPE";
  }
}

bool TypeHeader::operator== (TypeHeader const &o) const
{
  return (m_type == o.m_type && m_valid ==o.m_valid);
}

std::ostream & operator<< (std::ostream & os, TypeHeader const & h)
{
  h.Print(os);
  return os;
}

/*HELLO  Section*/
NS_OBJECT_ENSURE_REGISTERED(HelloHeader);

HelloHeader::HelloHeader ( NodeRole role, uint16_t NodeId, uint16_t clusterId, Vector position,  Ipv4Address addr,
                           Vector velocity, double InitialEnergy, double ResidualEnergy,  double fitness)
  : m_role            (role),
    m_nodeid          (NodeId),
    m_clusterId       (clusterId),
    m_PosX            (static_cast<uint16_t>(position.x)),
    m_PosY            (static_cast<uint16_t>(position.y)),
    m_PosZ            (static_cast<uint16_t>(position.z)),
    m_address         (addr),
    m_VelX            ((float) (velocity.x)),
    m_VelY            ((float) (velocity.y)),
    m_VelZ            ((float) (velocity.z)),
    m_InitialEnergy   (InitialEnergy),
    m_ResidualEnergy  (ResidualEnergy),
    m_FitnessValue    (fitness)
{
}

HelloHeader::~HelloHeader()
{
}

TypeId HelloHeader::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::icra::HelloHeader")
    .SetParent<Header> ()
    .AddConstructor<HelloHeader> ();
  return tid;
}

TypeId HelloHeader::GetInstanceTypeId() const
{
  return GetTypeId();
}

uint32_t HelloHeader::GetSerializedSize () const
{ 
  return 1 + 2*5 + 4*4 + 8*3;
}

void HelloHeader::Serialize (Buffer::Iterator i ) const
{
  NS_LOG_DEBUG ("Serialize HELLO NodeId: "<< m_nodeid <<", NodeRole: " << m_role << ", ClusterId: " << m_clusterId 
                <<", Position:("<< m_PosX <<","<< m_PosY <<","<<m_PosZ<<"), IpAddress: "<< m_address 
                <<", Velocity:("<< m_VelX <<","<<m_VelY<<", "<< m_VelZ <<"), InitialEnergy: " << m_InitialEnergy
                <<", ResidualEnergy:"<< m_ResidualEnergy <<"FitnessValue: "<<m_FitnessValue<<".");
  i.WriteU8      ((uint8_t) m_role);
  i.WriteHtonU16 (m_nodeid);
  i.WriteHtonU16 (m_clusterId); 
  i.WriteHtonU16 (m_PosX);
  i.WriteHtonU16 (m_PosY);
  i.WriteHtonU16 (m_PosZ);    
  WriteTo        (i, m_address);
  i.WriteHtonU32 (FloatToUint32(m_VelX));
  i.WriteHtonU32 (FloatToUint32(m_VelY));
  i.WriteHtonU32 (FloatToUint32(m_VelZ));
  i.WriteHtonU64 (DoubleToUint64(m_InitialEnergy));
  i.WriteHtonU64 (DoubleToUint64(m_ResidualEnergy));
  i.WriteHtonU64 (DoubleToUint64(m_FitnessValue));
//   i.WriteHtonU64 (DoubleToUint64(m_timestamp));
}

uint32_t HelloHeader::Deserialize (Buffer::Iterator start)
{
  Buffer::Iterator iter = start;
  uint8_t role = iter.ReadU8();
  m_nodeid         = iter.ReadNtohU16();
  m_clusterId      = iter.ReadNtohU16();
  m_PosX           = iter.ReadNtohU16();
  m_PosY           = iter.ReadNtohU16();
  m_PosZ           = iter.ReadNtohU16();
  ReadFrom(iter, m_address);
  m_VelX           = Uint32ToFloat(iter.ReadNtohU32());
  m_VelY           = Uint32ToFloat(iter.ReadNtohU32());
  m_VelZ           = Uint32ToFloat(iter.ReadNtohU32());
  m_InitialEnergy  = Uint64ToDouble(iter.ReadNtohU64());
  m_ResidualEnergy = Uint64ToDouble(iter.ReadNtohU64());
  m_FitnessValue   = Uint64ToDouble(iter.ReadNtohU64());
  switch(role){
    case Idle:
    case Member:
    case Head:
    case BackupHead:
    case Isolate:{
      m_role = (NodeRole) role;
      break;
    }
  }

  NS_LOG_DEBUG("Deserialize HELLO NodeId: "<< m_nodeid <<", NodeRole: " << m_role << ", ClusterId: " << m_clusterId 
                <<", Position:("<< m_PosX <<","<< m_PosY <<","<<m_PosZ<<"), IpAddress: "<< m_address 
                <<", Velocity:("<< m_VelX <<","<<m_VelY<<", "<< m_VelZ <<"), InitialEnergy: " << m_InitialEnergy
                <<", ResidualEnergy:"<< m_ResidualEnergy <<"FitnessValue: "<<m_FitnessValue<<".");

  uint32_t dist = iter.GetDistanceFrom (start);
  NS_ASSERT (dist == GetSerializedSize());
  return dist;
}

void HelloHeader::Print(std::ostream &os) const
{
  os << " NodeRole: "       << m_role
     << " NodeId: "         << m_nodeid
     << " ClusterId: "      << m_clusterId
     << " Position: ("      << m_PosX << ", " << m_PosY << ", " << m_PosZ <<")"
     << " IpAddress "       << m_address  
     << " Velocity: ("      << m_VelX << ", " << m_VelY << ", " << m_VelZ <<")"
     << " InitialEnergy: "  << m_InitialEnergy
     << " ResidualEnergy: " << m_ResidualEnergy
     << " FitnessValue: "   << m_FitnessValue;
}

void HelloHeader::SetPosition(Vector position)
{
  m_PosX = static_cast<uint16_t>(position.x);
  m_PosY = static_cast<uint16_t>(position.y);
  m_PosZ = static_cast<uint16_t>(position.z);
}

Vector HelloHeader::GetPosition() const 
{
  Vector Pos;
  Pos.x = m_PosX;
  Pos.y = m_PosY;
  Pos.z = m_PosZ;
  return Pos;
}

void HelloHeader::SetVelocity(Vector velocity)
{
  m_VelX = (float) velocity.x;
  m_VelY = (float) velocity.y;
  m_VelZ = (float) velocity.z;
}

Vector HelloHeader::GetVelocity() const 
{ 
  Vector Vel;
  Vel.x = m_VelX;
  Vel.y = m_VelY;
  Vel.z = m_VelZ;
  return Vel;
}

std::ostream & operator<< (std::ostream & os, HelloHeader const &h)
{
  h.Print(os);
  return os;
}

bool HelloHeader::operator== (HelloHeader const & o) const
{
  return ( m_role           == o.m_role
      &&   m_nodeid         == o.m_nodeid      
      &&   m_clusterId      == o.m_clusterId
      &&   m_PosX           == o.m_PosX 
      &&   m_PosY           == o.m_PosY
      &&   m_PosZ           == o.m_PosZ
      &&   m_address        == o.m_address
      &&   m_VelX           == o.m_VelX
      &&   m_VelY           == o.m_VelY
      &&   m_VelZ           == o.m_VelZ
      &&   m_InitialEnergy  == o.m_InitialEnergy
      &&   m_ResidualEnergy == o.m_ResidualEnergy
      &&   m_FitnessValue   == o.m_FitnessValue);
}

/* Cluster Header Announce Header Definitaion*/
ClusterHeadAnnounceHeader::ClusterHeadAnnounceHeader( uint16_t NodeId, double ResidualEnergy,Ipv4Address addr,
                                                      Vector pos, Vector vel, double FitnessValue, double Time)
  : m_nodeid (NodeId),
    m_ResidualEnergy(ResidualEnergy),
    m_address(addr),
    m_CHpositionX (static_cast<uint16_t>(pos.x)),
    m_CHpositionY (static_cast<uint16_t>(pos.y)),
    m_CHpositionZ (static_cast<uint16_t>(pos.z)),
    m_CHVelocityX ((float) (vel.x)),
    m_CHVelocityY ((float) (vel.y)),
    m_CHVelocityZ ((float) (vel.z)),
    m_FitnessValue(FitnessValue),
    m_timestamp (Time)
{
}

NS_OBJECT_ENSURE_REGISTERED (ClusterHeadAnnounceHeader);

TypeId ClusterHeadAnnounceHeader::GetTypeId()
{
   static TypeId tid = TypeId("ns3::icra::ClusterHeadAnnounceHeader")
   .SetParent<Header> ()
   .AddConstructor<ClusterHeadAnnounceHeader> ();
   return tid;
}

TypeId ClusterHeadAnnounceHeader::GetInstanceTypeId() const
{
   return GetTypeId();
}

uint32_t ClusterHeadAnnounceHeader::GetSerializedSize() const
{
   return 48;
}

void ClusterHeadAnnounceHeader::Serialize(Buffer::Iterator i) const
{   
    NS_LOG_DEBUG("Serialize CH_CALL NodeId: "<< m_nodeid <<", ResidualEnergy: "<< m_ResidualEnergy
        <<", IpAddress: "<<m_address <<", Position: ("<< m_CHpositionX <<", "<< m_CHpositionY <<", "<< m_CHpositionZ
        <<"), Velocity: ("<< m_CHVelocityX <<", "<< m_CHVelocityY <<", "<< m_CHVelocityZ
        <<", FitnessValue: "<< m_FitnessValue<<", Time: "<< m_timestamp);

    i.WriteHtonU16(m_nodeid);
    i.WriteHtonU64(DoubleToUint64(m_ResidualEnergy));
    WriteTo(i, m_address);
    i.WriteHtonU16(m_CHpositionX);
    i.WriteHtonU16(m_CHpositionY);
    i.WriteHtonU16(m_CHpositionZ);
    i.WriteHtonU32(FloatToUint32(m_CHVelocityX));
    i.WriteHtonU32(FloatToUint32(m_CHVelocityY));
    i.WriteHtonU32(FloatToUint32(m_CHVelocityZ));
    i.WriteHtonU64(DoubleToUint64(m_FitnessValue));
    i.WriteHtonU64(DoubleToUint64(m_timestamp));
}

uint32_t ClusterHeadAnnounceHeader::Deserialize(Buffer::Iterator start) 
{
    Buffer::Iterator iter = start;
    m_nodeid         = iter.ReadNtohU16();
    m_ResidualEnergy = Uint64ToDouble(iter.ReadNtohU64());
    ReadFrom(iter, m_address);
    m_CHpositionX    = iter.ReadNtohU16();
    m_CHpositionY    = iter.ReadNtohU16();
    m_CHpositionZ    = iter.ReadNtohU16();
    m_CHVelocityX    = Uint32ToFloat(iter.ReadNtohU32());
    m_CHVelocityY    = Uint32ToFloat(iter.ReadNtohU32());
    m_CHVelocityZ    = Uint32ToFloat(iter.ReadNtohU32());
    m_FitnessValue   = Uint64ToDouble(iter.ReadNtohU64());
    m_timestamp      = Uint64ToDouble(iter.ReadNtohU64());

    NS_LOG_DEBUG("Deserialize CH_CALL NodeId: "<< m_nodeid <<", ResidualEnergy: "<< m_ResidualEnergy 
                    <<", IpAddress:"<< m_address <<", Position: ("<< m_CHpositionX <<", "<< m_CHpositionY <<", "<< m_CHpositionZ
                    <<"), Velocity: ("<< m_CHVelocityX <<", "<< m_CHVelocityY <<", "<< m_CHVelocityZ
                    <<", FitnessValue: "<< m_FitnessValue<<", Time: "<< m_timestamp);    
    
    uint32_t dist = iter.GetDistanceFrom(start);
    NS_ASSERT (dist == GetSerializedSize());
    return dist;
}

void ClusterHeadAnnounceHeader::SetPosition(Vector position)
{
    m_CHpositionX = static_cast<uint16_t>(position.x);
    m_CHpositionY = static_cast<uint16_t>(position.y);
    m_CHpositionZ = static_cast<uint16_t>(position.z);
}

Vector ClusterHeadAnnounceHeader::GetPosition() const
{
    Vector Pos;
    Pos.x = m_CHpositionX;
    Pos.y = m_CHpositionY;
    Pos.z = m_CHpositionZ;
    return Pos;
}

void ClusterHeadAnnounceHeader::SetVelocity(Vector velocity)
{
    m_CHVelocityX = (float) (velocity.x);
    m_CHVelocityY = (float) (velocity.y);
    m_CHVelocityZ = (float) (velocity.z);
}

Vector ClusterHeadAnnounceHeader::GetVelocity() const
{
    Vector Vel;
    Vel.x = m_CHVelocityX;
    Vel.y = m_CHVelocityY;
    Vel.z = m_CHVelocityZ;
    return Vel;
}

void ClusterHeadAnnounceHeader::Print(std::ostream & os) const
{
    os  << " NodeId: "         << m_nodeid
        << " ResidualEnergy: " << m_ResidualEnergy
        << " IpAddress: "      << m_address
        << " Position: ("      << m_CHpositionX << ", " << m_CHpositionY << ", " << m_CHpositionZ << ")"
        << " Velocity: ("      << m_CHVelocityX << ", " << m_CHVelocityY << ", " << m_CHVelocityZ << ")"
        << " FitnessValue: "   << m_FitnessValue
        << " Time: "           << m_timestamp;
}

bool ClusterHeadAnnounceHeader:: operator== (ClusterHeadAnnounceHeader const & o) const
{
    return (   m_nodeid          == o.m_nodeid 
            && m_ResidualEnergy  == o.m_ResidualEnergy
            && m_address         == o.m_address
            && m_CHpositionX     == o.m_CHpositionX
            && m_CHpositionY     == o.m_CHpositionY
            && m_CHpositionZ     == o.m_CHpositionZ
            && m_CHVelocityX     == o.m_CHVelocityX
            && m_CHVelocityY     == o.m_CHVelocityY
            && m_CHVelocityZ     == o.m_CHVelocityZ
            && m_FitnessValue    == o.m_FitnessValue
            && m_timestamp       == o.m_timestamp);
}

std::ostream & operator<< (std::ostream & os, ClusterHeadAnnounceHeader const & h)
{
    h.Print(os);
    return os;
}


/* Cluster Member Request Header Definitaion*/
NS_OBJECT_ENSURE_REGISTERED (JoinClusterRequestHeader);

JoinClusterRequestHeader::JoinClusterRequestHeader( uint16_t NodeId, NodeRole role, Ipv4Address Addr,
                                                    double ResidualEnergy, double Time, Vector pos, Vector vel)
    :   m_nodeid            (NodeId),
        m_Role              (role),
        m_address           (Addr),
        m_ResidualEnergy    (ResidualEnergy),
        m_timestamp         (Time),
        m_CMpositionX       (static_cast<uint16_t>(pos.x)),
        m_CMpositionY       (static_cast<uint16_t>(pos.y)),
        m_CMpositionZ       (static_cast<uint16_t>(pos.z)),
        m_CMVelocityX       ((float) (vel.x)),
        m_CMVelocityY       ((float) (vel.y)),
        m_CMVelocityZ       ((float) (vel.z))
{}



TypeId JoinClusterRequestHeader::GetTypeId ()
{
    static TypeId tid = TypeId("ns3::icra::JoinClusterRequestHeader")
        .SetParent<Header> ()
        .AddConstructor<JoinClusterRequestHeader> ();
    return tid;
}
TypeId JoinClusterRequestHeader::GetInstanceTypeId() const
{
    return GetTypeId();
}
uint32_t JoinClusterRequestHeader::GetSerializedSize() const
{ 
    return 41; 
}

void JoinClusterRequestHeader::Serialize(Buffer::Iterator i) const
{
    NS_LOG_DEBUG("Serialize JOIN_REQ NodeId: "<< m_nodeid <<", RoleType: "<<m_Role<<", IpAddress: "<< m_address
                <<", ResidualEnergy: "<< m_ResidualEnergy <<", Time: "<< m_timestamp
                <<", Position: ("<< m_CMpositionX <<", "<< m_CMpositionY <<", "<< m_CMpositionZ 
                <<"), Velocity: (" << m_CMVelocityX <<", "<< m_CMVelocityY <<", "<< m_CMVelocityZ <<").");
    i.WriteHtonU16(m_nodeid);
    i.WriteU8((uint8_t) m_Role);
    WriteTo(i, m_address);
    i.WriteHtonU64(DoubleToUint64(m_ResidualEnergy));
    i.WriteHtonU64(DoubleToUint64(m_timestamp));
    i.WriteHtonU16(m_CMpositionX);
    i.WriteHtonU16(m_CMpositionY);
    i.WriteHtonU16(m_CMpositionZ);
    i.WriteHtonU32(FloatToUint32(m_CMVelocityX));
    i.WriteHtonU32(FloatToUint32(m_CMVelocityY));
    i.WriteHtonU32(FloatToUint32(m_CMVelocityZ));
}

uint32_t JoinClusterRequestHeader::Deserialize(Buffer::Iterator start)
{
    Buffer::Iterator i = start;
    m_nodeid            = i.ReadNtohU16();
    uint8_t type = i.ReadU8();
    switch (type)
    {
    case Idle:
    case BackupHead:
    case Head:
    case Member:
    case Isolate:
        {
            m_Role = (NodeRole) type;
            break;
        }   
    default:
        m_Role = Idle;
    }
    ReadFrom(i, m_address);
    m_ResidualEnergy    = Uint64ToDouble(i.ReadNtohU64());
    m_timestamp         = Uint64ToDouble(i.ReadNtohU64());
    m_CMpositionX       = (i.ReadNtohU16());
    m_CMpositionY       = (i.ReadNtohU16());
    m_CMpositionZ       = (i.ReadNtohU16());
    m_CMVelocityX       = Uint32ToFloat(i.ReadNtohU32());
    m_CMVelocityY       = Uint32ToFloat(i.ReadNtohU32());
    m_CMVelocityZ       = Uint32ToFloat(i.ReadNtohU32());

    NS_LOG_DEBUG("Deserialize JOIN_REQ NodeId: "<< m_nodeid <<"IpAddress: "<< m_address
                <<", ResidualEnergy: "<< m_ResidualEnergy <<", Time: "<< m_timestamp
                <<", Position: ("<< m_CMpositionX <<", "<< m_CMpositionY <<", "<< m_CMpositionZ 
                <<"), Velocity: ("<< m_CMVelocityX <<", "<< m_CMVelocityY <<", "<< m_CMVelocityZ <<").");
    
    uint32_t dist = i.GetDistanceFrom(start);
    NS_ASSERT(dist == GetSerializedSize());

    return dist;
}

void JoinClusterRequestHeader::Print(std::ostream & os) const
{
    os << " NodeId: " << m_nodeid << "IpAddress: " << m_address 
        <<" ResidualEnetgy: " << m_ResidualEnergy << " Time: " << m_timestamp
        <<" Position:("<< m_CMpositionX << ", "<< m_CMpositionY << ", "<< m_CMpositionZ
        <<"), Velocity: ("<< m_CMVelocityX << ", "<< m_CMVelocityY << ", "<< m_CMVelocityZ<<")";
}

void JoinClusterRequestHeader::SetPosition(Vector pos)
{
    m_CMpositionX = static_cast<uint16_t>(pos.x);
    m_CMpositionY = static_cast<uint16_t>(pos.y);
    m_CMpositionZ = static_cast<uint16_t>(pos.z);
}

Vector JoinClusterRequestHeader::GetPosition() const
{
    Vector Pos;
    Pos.x = m_CMpositionX;
    Pos.y = m_CMpositionY;
    Pos.z = m_CMpositionZ;
    return Pos;
}

void JoinClusterRequestHeader::SetVelocity(Vector vel)
{
    m_CMVelocityX = (float)(vel.x);
    m_CMVelocityY = (float)(vel.y);
    m_CMVelocityZ = (float)(vel.z);
}

Vector JoinClusterRequestHeader::GetVelocity() const
{
    Vector Vel;
    Vel.x = m_CMVelocityX;
    Vel.y = m_CMVelocityY;
    Vel.z = m_CMVelocityZ;
    return Vel;
}

std::ostream & operator<< ( std::ostream & os, JoinClusterRequestHeader const & h)
{
    h.Print(os);
    return os;
}

bool JoinClusterRequestHeader::operator== (JoinClusterRequestHeader const & o) const
{
    return (m_nodeid == o.m_nodeid && m_ResidualEnergy == o.m_ResidualEnergy && m_timestamp == o.m_timestamp
            && m_CMpositionX == o.m_CMpositionX && m_CMpositionY == o.m_CMpositionY && m_CMpositionZ == o.m_CMpositionZ
            && m_CMVelocityX == o.m_CMVelocityX && m_CMVelocityY == o.m_CMVelocityY && m_CMVelocityZ == o.m_CMVelocityZ);
}

NS_OBJECT_ENSURE_REGISTERED(ClusterHeadResponseHeader);

ClusterHeadResponseHeader::ClusterHeadResponseHeader(uint16_t ChNodeId, uint16_t ClusterId, Ipv4Address Addr, double timestamp)
    :   m_nodeid            (ChNodeId),
        m_clusterId         (ClusterId),
        m_address           (Addr),
        m_timestamp         (timestamp)
{}



TypeId ClusterHeadResponseHeader::GetTypeId()
{
    static TypeId tid = TypeId("ns3::icra::ClusterHeadResponseHeader")
        .SetParent<Header> ()
        .AddConstructor<ClusterHeadResponseHeader> ();
    return tid;
}

TypeId ClusterHeadResponseHeader::GetInstanceTypeId() const
{
    return GetTypeId();
}

uint32_t ClusterHeadResponseHeader::GetSerializedSize() const
{
    return 16;
}

void ClusterHeadResponseHeader::Serialize(Buffer::Iterator i) const
{
    NS_LOG_DEBUG( "Serialize JOIN_RESP ChNodeId: "<< m_nodeid << ", ClusterId:" <<m_clusterId
                        <<", IpAddress: "<< m_address <<", Time: "<< m_timestamp);
    
    i.WriteHtonU16(m_nodeid);
    i.WriteHtonU16(m_clusterId);
    WriteTo(i, m_address);
    i.WriteHtonU64(DoubleToUint64(m_timestamp));


}

uint32_t ClusterHeadResponseHeader::Deserialize(Buffer::Iterator start)
{
    Buffer::Iterator i = start;
    m_nodeid            = i.ReadNtohU16();
    m_clusterId         = i.ReadNtohU16();
    ReadFrom(i, m_address);
    m_timestamp         = Uint64ToDouble(i.ReadNtohU64());


    NS_LOG_DEBUG("Deserialize JOIN_RESP ChNodeId: "<< m_nodeid <<", IpAddress: "<< m_address
                    << ", ClusterId:"<<m_clusterId <<", Time: "<< m_timestamp);
    
    uint32_t dist = i.GetDistanceFrom(start);
    NS_ASSERT(dist == GetSerializedSize());

    return dist;
}

void ClusterHeadResponseHeader::Print (std::ostream & os) const
{
    os  << " ChNodeId: " << m_nodeid <<" ClusterId: "<<m_clusterId<< " IpAddress: " <<m_address << " Time: " << m_timestamp;
}

std::ostream & operator<<(std::ostream & os, ClusterHeadResponseHeader const & h)
{
    h.Print(os);
    return os;
}

bool ClusterHeadResponseHeader::operator== ( ClusterHeadResponseHeader const & o) const
{
    return ( m_nodeid==o.m_nodeid && m_clusterId==o.m_clusterId && m_address==o.m_address  && m_timestamp==o.m_timestamp);
}


NS_OBJECT_ENSURE_REGISTERED(ClusterHeadBeaconHeader);

ClusterHeadBeaconHeader::ClusterHeadBeaconHeader( uint16_t ChNodeId, uint16_t ClusterId, Ipv4Address Addr,
                                                  Vector pos, Vector vel, uint8_t MemberDegree,
                                                  double ResidualEnergy, double fit, double LatestTime)
    :   m_ChNodeId          (ChNodeId),
        m_ClusterId         (ClusterId),
        m_address           (Addr),
        m_posX              (static_cast<uint16_t>(pos.x)),
        m_posY              (static_cast<uint16_t>(pos.y)),
        m_posZ              (static_cast<uint16_t>(pos.z)),
        m_velX              ((float) (vel.x)),
        m_velY              ((float) (vel.y)),
        m_velZ              ((float) (vel.z)),
        m_MemberDegree      (MemberDegree),
        m_ResidualEnergy    (ResidualEnergy),
        m_FitnessValue      (fit),
        m_LatestTime        (LatestTime)
{
}

TypeId ClusterHeadBeaconHeader::GetTypeId()
{
    static TypeId tid = TypeId("ns3::icra::ClusterHeadBeaconHeader")
        .SetParent<Header> ()
        .AddConstructor<ClusterHeadBeaconHeader> ();
    return tid;
}

TypeId ClusterHeadBeaconHeader::GetInstanceTypeId() const
{
    return GetTypeId();
}

uint32_t ClusterHeadBeaconHeader::GetSerializedSize() const
{
    return 51;
}

void ClusterHeadBeaconHeader::Serialize(Buffer::Iterator i) const
{
    NS_LOG_DEBUG("Serialize CH_BEACON ChNodeId: "<< m_ChNodeId <<", ClusterId: "<< m_ClusterId <<", Ip:"<<m_address
                    <<", Position:(" << m_posX <<","<<m_posY<<","<<m_posZ<<"), Velocity:("<<m_velX<<","<<m_velY<<","<<m_velZ
                    <<"), MemberDegree: "<< m_MemberDegree << ", ResidualEnergy: " << m_ResidualEnergy
                    << "FitValue: "<<m_FitnessValue<<", LatestTime: "<< m_LatestTime);
    i.WriteHtonU16(m_ChNodeId);
    i.WriteHtonU16(m_ClusterId);
    WriteTo(i,m_address),
    i.WriteHtonU16(m_posX),
    i.WriteHtonU16(m_posY),
    i.WriteHtonU16(m_posZ),
    i.WriteHtonU32(FloatToUint32(m_velX)),
    i.WriteHtonU32(FloatToUint32(m_velY)),
    i.WriteHtonU32(FloatToUint32(m_velZ)),
    i.WriteU8(m_MemberDegree);
    i.WriteHtonU64(DoubleToUint64(m_ResidualEnergy));
    i.WriteHtonU64(DoubleToUint64(m_FitnessValue));
    i.WriteHtonU64(DoubleToUint64(m_LatestTime));
}

uint32_t ClusterHeadBeaconHeader::Deserialize(Buffer::Iterator start)
{
    Buffer::Iterator i = start;
    m_ChNodeId          = i.ReadNtohU16();
    m_ClusterId         = i.ReadNtohU16();
    ReadFrom(i, m_address),
    m_posX              = i.ReadNtohU16(),
    m_posY              = i.ReadNtohU16(),
    m_posZ              = i.ReadNtohU16(),
    m_velX              = FloatToUint32(i.ReadNtohU32()),
    m_velY              = FloatToUint32(i.ReadNtohU32()),
    m_velZ              = FloatToUint32(i.ReadNtohU32()),
    m_MemberDegree      = i.ReadU8();
    m_ResidualEnergy    = Uint64ToDouble(i.ReadNtohU64());
    m_FitnessValue      = Uint64ToDouble(i.ReadNtohU64());
    m_LatestTime        = Uint64ToDouble(i.ReadNtohU64());

    NS_LOG_DEBUG("Deserialize CH_BEACON ChNodeId: "<< m_ChNodeId <<", ClusterId: "<< m_ClusterId <<", Ip:"<<m_address
                <<", Position:(" << m_posX <<","<<m_posY<<","<<m_posZ<<"), Velocity:("<<m_velX<<","<<m_velY<<","<<m_velZ
                <<"), MemberDegree: "<< m_MemberDegree << ", ResidualEnergy: "<<m_ResidualEnergy
                <<", FitValue: "<<m_FitnessValue<<", LatestTime: "<< m_LatestTime);
    
    uint32_t dist = i.GetDistanceFrom(start);
    NS_ASSERT(dist == GetSerializedSize());
    return dist;
}

void ClusterHeadBeaconHeader::Print(std::ostream & os) const
{
    os << " ChNodeId: " << m_ChNodeId << " ClusterId: " << m_ClusterId << " MemberDegree: " << m_MemberDegree
       << " ResidualEnergy: "<<m_ResidualEnergy << " FitnessValue:"<< m_FitnessValue << "LatestTime: " << m_LatestTime;
}

void ClusterHeadBeaconHeader::SetPosition(Vector pos)
{
    m_posX = static_cast<uint16_t>(pos.x);
    m_posY = static_cast<uint16_t>(pos.x);
    m_posZ = static_cast<uint16_t>(pos.x);
}

void ClusterHeadBeaconHeader::SetVelocity(Vector vel)
{
    m_velX = (float)(vel.x);
    m_velY = (float)(vel.y);
    m_velZ = (float)(vel.z);
}

std::ostream & operator<<(std::ostream & os, ClusterHeadBeaconHeader const & h)
{
    h.Print(os);
    return os;
}

bool ClusterHeadBeaconHeader::operator== (ClusterHeadBeaconHeader const & o) const
{
    return (   m_ChNodeId == o.m_ChNodeId && m_ClusterId == o.m_ClusterId && m_address == o.m_address
           &&  m_posX == o.m_posX && m_posY == o.m_posY && m_posZ == o.m_posZ
           &&  m_velX == o.m_velX && m_velY == o.m_velY && m_velZ == o.m_velZ
           &&  m_MemberDegree == o.m_MemberDegree && m_ResidualEnergy == o.m_ResidualEnergy
           &&  m_FitnessValue == o.m_FitnessValue &&m_LatestTime == o.m_LatestTime);
}



ClusterMemberBeaconHeader::ClusterMemberBeaconHeader(uint16_t NodeId, uint16_t ClusterId, Ipv4Address addr, Vector pos, Vector vel, double fit, double LatestTime)
    :   m_NodeId            (NodeId),
        m_ClusterId         (ClusterId),
        m_address           (addr),
        m_posX              (static_cast<uint16_t> (pos.x)),
        m_posY              (static_cast<uint16_t> (pos.y)),
        m_posZ              (static_cast<uint16_t> (pos.z)),
        m_velX              ((float) (vel.x)),
        m_velY              ((float) (vel.y)),
        m_velZ              ((float) (vel.z)),
        m_FitnessValue      (fit),
        m_LatestTime        (LatestTime)
{
}

TypeId ClusterMemberBeaconHeader::GetTypeId()
{
    static TypeId tid = TypeId("ns3::icra::ClusterMemberBeaconHeader")
        .SetParent<Header> ()
        .AddConstructor<ClusterMemberBeaconHeader> ();
    return tid;
}

TypeId ClusterMemberBeaconHeader::GetInstanceTypeId() const
{
    return GetTypeId();
}

uint32_t ClusterMemberBeaconHeader::GetSerializedSize() const
{
    return 42;
}

void ClusterMemberBeaconHeader::Serialize(Buffer::Iterator i) const
{
    NS_LOG_DEBUG("Serialize CM_BEACON NodeId: "<< m_NodeId <<", ClusterId: "<< m_ClusterId
                 <<"Position: " << m_posX   << "," << m_posY  << "," << m_posZ
                 <<"), Velocity" << m_posX   << "," << m_velY  << "," << m_velZ
                 <<") ,FitnessVlaue"<<m_FitnessValue << ", LatestTime: " << m_LatestTime);
    i.WriteHtonU16(m_NodeId);
    i.WriteHtonU16(m_ClusterId);
    WriteTo(i,m_address),
    i.WriteHtonU16(m_posX),
    i.WriteHtonU16(m_posY),
    i.WriteHtonU16(m_posZ),
    i.WriteHtonU32(FloatToUint32(m_velX)),
    i.WriteHtonU32(FloatToUint32(m_velY)),
    i.WriteHtonU32(FloatToUint32(m_velZ)),
    i.WriteHtonU64(DoubleToUint64(m_FitnessValue)),
    i.WriteHtonU64(DoubleToUint64(m_LatestTime));
}

uint32_t ClusterMemberBeaconHeader::Deserialize(Buffer::Iterator start)
{
    NS_LOG_DEBUG("Deserialize CM_BEACON NodeId: "<< m_NodeId <<", ClusterId: "<< m_ClusterId << "Ip: " << m_address
                 << "Position: " << m_posX   << "," << m_posY  << "," << m_posZ
                 <<"), Velocity" << m_posX   << "," << m_velY  << "," << m_velZ << ") LatestTime: "<< m_LatestTime);
    Buffer::Iterator i = start;
    m_NodeId        =  i.ReadNtohU16();
    m_ClusterId     =  i.ReadNtohU16();
    ReadFrom(i,m_address);
    m_posX          =  i.ReadNtohU16();
    m_posY          =  i.ReadNtohU16();
    m_posZ          =  i.ReadNtohU16();
    m_velX          =  Uint32ToFloat (i.ReadNtohU32());
    m_velY          =  Uint32ToFloat (i.ReadNtohU32());
    m_velZ          =  Uint32ToFloat (i.ReadNtohU32());
    m_FitnessValue  =  Uint64ToDouble(i.ReadNtohU64());
    m_LatestTime    =  Uint64ToDouble(i.ReadNtohU64());
    
    uint32_t dist = i.GetDistanceFrom(start);

    NS_ASSERT(dist == GetSerializedSize());
    return dist;
}

void ClusterMemberBeaconHeader::Print(std::ostream & os) const
{
    os << " NodeId: "  << m_NodeId << " ClusterId: " << m_ClusterId <<"Ip: "<< m_address
       << "Position: " << m_posX   << "," << m_posY  << "," << m_posZ
       <<"), Velocity" << m_posX   << "," << m_velY  << "," << m_velZ
       << ") FitnessValue"<<m_FitnessValue << ", LatestTime: " << m_LatestTime;
}

void ClusterMemberBeaconHeader::SetPosition(Vector pos)
{
    m_posX = static_cast<uint16_t>(pos.x);
    m_posY = static_cast<uint16_t>(pos.y);
    m_posZ = static_cast<uint16_t>(pos.z);
}

void ClusterMemberBeaconHeader::SetVelocity(Vector vel)
{
    m_velX = (float)(vel.x);
    m_velY = (float)(vel.y);
    m_velZ = (float)(vel.z);
}

std::ostream & operator<<(std::ostream & os, ClusterMemberBeaconHeader const & h)
{
    h.Print(os);
    return os;
}

bool ClusterMemberBeaconHeader::operator== (ClusterMemberBeaconHeader const & o) const
{
    return (m_NodeId==o.m_NodeId && m_ClusterId==o.m_ClusterId && m_address == o.m_address
         && m_posX==o.m_posX && m_posY==o.m_posY && m_posZ==o.m_posZ 
         && m_velX==o.m_velX && m_velY==o.m_velY && m_velZ==m_velZ
         && m_FitnessValue == o.m_FitnessValue && m_LatestTime==o.m_LatestTime);
}


/* ICRA Utility Header - 新增实现 */
NS_OBJECT_ENSURE_REGISTERED(UtilityHeader);

UtilityHeader::UtilityHeader(uint16_t NodeId, double utilityValue, Ipv4Address addr,
                             Vector pos, Vector vel)
  : m_nodeId(NodeId),
    m_utilityValue(utilityValue),
    m_address(addr),
    m_posX(static_cast<uint16_t>(pos.x)),
    m_posY(static_cast<uint16_t>(pos.y)),
    m_posZ(static_cast<uint16_t>(pos.z)),
    m_velX((float)(vel.x)),
    m_velY((float)(vel.y)),
    m_velZ((float)(vel.z))
{
}

TypeId UtilityHeader::GetTypeId()
{
  static TypeId tid = TypeId("ns3::icra::UtilityHeader")
    .SetParent<Header>()
    .AddConstructor<UtilityHeader>();
  return tid;
}

TypeId UtilityHeader::GetInstanceTypeId() const
{
  return GetTypeId();
}

uint32_t UtilityHeader::GetSerializedSize() const
{
  return 2 + 8 + 4 + 2*3 + 4*3; // NodeId(2) + Utility(8) + IP(4) + Pos(6) + Vel(12) = 34
}

void UtilityHeader::Serialize(Buffer::Iterator i) const
{
  NS_LOG_DEBUG("Serialize UTILITY NodeId: " << m_nodeId 
               << ", Utility: " << m_utilityValue
               << ", IP: " << m_address);
  
  i.WriteHtonU16(m_nodeId);
  i.WriteHtonU64(DoubleToUint64(m_utilityValue));
  WriteTo(i, m_address);
  i.WriteHtonU16(m_posX);
  i.WriteHtonU16(m_posY);
  i.WriteHtonU16(m_posZ);
  i.WriteHtonU32(FloatToUint32(m_velX));
  i.WriteHtonU32(FloatToUint32(m_velY));
  i.WriteHtonU32(FloatToUint32(m_velZ));
}

uint32_t UtilityHeader::Deserialize(Buffer::Iterator start)
{
  Buffer::Iterator i = start;
  m_nodeId = i.ReadNtohU16();
  m_utilityValue = Uint64ToDouble(i.ReadNtohU64());
  ReadFrom(i, m_address);
  m_posX = i.ReadNtohU16();
  m_posY = i.ReadNtohU16();
  m_posZ = i.ReadNtohU16();
  m_velX = Uint32ToFloat(i.ReadNtohU32());
  m_velY = Uint32ToFloat(i.ReadNtohU32());
  m_velZ = Uint32ToFloat(i.ReadNtohU32());
  
  NS_LOG_DEBUG("Deserialize UTILITY NodeId: " << m_nodeId 
               << ", Utility: " << m_utilityValue
               << ", IP: " << m_address);
  
  uint32_t dist = i.GetDistanceFrom(start);
  NS_ASSERT(dist == GetSerializedSize());
  return dist;
}

void UtilityHeader::Print(std::ostream &os) const
{
  os << " NodeId: " << m_nodeId
     << " Utility: " << m_utilityValue
     << " IP: " << m_address
     << " Pos: (" << m_posX << "," << m_posY << "," << m_posZ << ")"
     << " Vel: (" << m_velX << "," << m_velY << "," << m_velZ << ")";
}

void UtilityHeader::SetPosition(Vector pos)
{
  m_posX = static_cast<uint16_t>(pos.x);
  m_posY = static_cast<uint16_t>(pos.y);
  m_posZ = static_cast<uint16_t>(pos.z);
}

Vector UtilityHeader::GetPosition() const
{
  return Vector(m_posX, m_posY, m_posZ);
}

void UtilityHeader::SetVelocity(Vector vel)
{
  m_velX = (float)(vel.x);
  m_velY = (float)(vel.y);
  m_velZ = (float)(vel.z);
}

Vector UtilityHeader::GetVelocity() const
{
  return Vector(m_velX, m_velY, m_velZ);
}

bool UtilityHeader::operator==(UtilityHeader const &o) const
{
  return (m_nodeId == o.m_nodeId 
       && m_utilityValue == o.m_utilityValue
       && m_address == o.m_address
       && m_posX == o.m_posX && m_posY == o.m_posY && m_posZ == o.m_posZ
       && m_velX == o.m_velX && m_velY == o.m_velY && m_velZ == o.m_velZ);
}

std::ostream & operator<<(std::ostream & os, UtilityHeader const & h)
{
  h.Print(os);
  return os;
}





namespace {
void WriteDouble(Buffer::Iterator& i, double value)
{
  i.WriteHtonU64(DoubleToUint64(value));
}

double ReadDouble(Buffer::Iterator& i)
{
  return Uint64ToDouble(i.ReadNtohU64());
}

void WriteVector(Buffer::Iterator& i, Vector value)
{
  WriteDouble(i, value.x);
  WriteDouble(i, value.y);
  WriteDouble(i, value.z);
}

Vector ReadVector(Buffer::Iterator& i)
{
  const double x = ReadDouble(i);
  const double y = ReadDouble(i);
  const double z = ReadDouble(i);
  return Vector(x, y, z);
}

bool FiniteVector(Vector value)
{
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

bool ValidRole(NodeRole role)
{
  return role >= Idle && role <= Isolate;
}

bool UnitValue(double value)
{
  return std::isfinite(value) && value >= 0 && value <= 1;
}

bool ValidEnergy(double origin, double residual)
{
  return std::isfinite(origin) && origin > 0 && std::isfinite(residual) &&
      residual >= 0 && residual <= origin + 1e-9 * std::max(1.0, origin);
}
} // namespace

NS_OBJECT_ENSURE_REGISTERED(GsReportHeader);

TypeId GsReportHeader::GetTypeId()
{
  static TypeId tid = TypeId("ns3::icra::GsReportHeader")
      .SetParent<Header>().AddConstructor<GsReportHeader>();
  return tid;
}

TypeId GsReportHeader::GetInstanceTypeId() const { return GetTypeId(); }
uint32_t GsReportHeader::GetSerializedSize() const { return 125; }

void GsReportHeader::Serialize(Buffer::Iterator i) const
{
  i.WriteHtonU16(nodeId);
  for (double factor : factors) { WriteDouble(i, factor); }
  WriteDouble(i, energyChange);
  i.WriteHtonU32(roleChanges);
  i.WriteHtonU32(epoch);
  i.WriteHtonU64(timestampNs);
  WriteDouble(i, originEnergy);
  WriteDouble(i, residualEnergy);
  WriteVector(i, position);
  WriteVector(i, velocity);
  i.WriteHtonU16(degree);
  i.WriteU8(static_cast<uint8_t>(role));
}

uint32_t GsReportHeader::Deserialize(Buffer::Iterator i)
{
  if (i.GetRemainingSize() < GetSerializedSize()) { m_wellFormed = false; return 0; }
  m_wellFormed = true;
  nodeId = i.ReadNtohU16();
  for (double& factor : factors) { factor = ReadDouble(i); }
  energyChange = ReadDouble(i);
  roleChanges = i.ReadNtohU32();
  epoch = i.ReadNtohU32();
  timestampNs = i.ReadNtohU64();
  originEnergy = ReadDouble(i);
  residualEnergy = ReadDouble(i);
  position = ReadVector(i);
  velocity = ReadVector(i);
  degree = i.ReadNtohU16();
  role = static_cast<NodeRole>(i.ReadU8());
  return GetSerializedSize();
}

double GsReportHeader::GetSpeed() const
{
  return std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z);
}

bool GsReportHeader::IsValid() const
{
  return m_wellFormed && nodeId != 0xffff &&
      std::all_of(factors.begin(), factors.end(), UnitValue) &&
      UnitValue(energyChange) && ValidEnergy(originEnergy, residualEnergy) &&
      FiniteVector(position) && FiniteVector(velocity) && ValidRole(role);
}

void GsReportHeader::Print(std::ostream& os) const
{
  os << "GS_REPORT node=" << nodeId << " epoch=" << epoch << " timestamp_ns=" << timestampNs;
}

NS_OBJECT_ENSURE_REGISTERED(GsPolicyHeader);

TypeId GsPolicyHeader::GetTypeId()
{
  static TypeId tid = TypeId("ns3::icra::GsPolicyHeader")
      .SetParent<Header>().AddConstructor<GsPolicyHeader>();
  return tid;
}

TypeId GsPolicyHeader::GetInstanceTypeId() const { return GetTypeId(); }
uint32_t GsPolicyHeader::GetSerializedSize() const { return 44; }

void GsPolicyHeader::Serialize(Buffer::Iterator i) const
{
  for (double weight : weights) { WriteDouble(i, weight); }
  i.WriteHtonU32(epoch);
  i.WriteHtonU64(timestampNs);
}

uint32_t GsPolicyHeader::Deserialize(Buffer::Iterator i)
{
  if (i.GetRemainingSize() < GetSerializedSize()) { m_wellFormed = false; return 0; }
  m_wellFormed = true;
  for (double& weight : weights) { weight = ReadDouble(i); }
  epoch = i.ReadNtohU32();
  timestampNs = i.ReadNtohU64();
  return GetSerializedSize();
}

bool GsPolicyHeader::IsValid() const
{
  double sum = 0;
  for (double weight : weights) {
    if (!UnitValue(weight) || std::abs(weight * 20.0 - std::round(weight * 20.0)) > 1e-6) {
      return false;
    }
    sum += weight;
  }
  return m_wellFormed && std::abs(sum - 1.0) <= 1e-6;
}

void GsPolicyHeader::Print(std::ostream& os) const
{
  os << "GS_POLICY epoch=" << epoch << " weights=" << weights[0] << ':' << weights[1]
     << ':' << weights[2] << ':' << weights[3] << " timestamp_ns=" << timestampNs;
}

NS_OBJECT_ENSURE_REGISTERED(PaperControlHeader);

TypeId PaperControlHeader::GetTypeId()
{
  static TypeId tid = TypeId("ns3::icra::PaperControlHeader")
      .SetParent<Header>().AddConstructor<PaperControlHeader>();
  return tid;
}

TypeId PaperControlHeader::GetInstanceTypeId() const { return GetTypeId(); }
uint32_t PaperControlHeader::GetSerializedSize() const { return 99; }

void PaperControlHeader::Serialize(Buffer::Iterator i) const
{
  i.WriteU8(static_cast<uint8_t>(kind));
  i.WriteHtonU16(nodeId);
  i.WriteHtonU16(headId);
  i.WriteHtonU16(clusterId);
  i.WriteHtonU32(epoch);
  i.WriteHtonU32(stateEpoch);
  i.WriteHtonU16(transaction);
  i.WriteU8(static_cast<uint8_t>(role));
  i.WriteU8(forwarder ? 1 : 0);
  WriteVector(i, position);
  WriteVector(i, velocity);
  WriteDouble(i, originEnergy);
  WriteDouble(i, residualEnergy);
  WriteDouble(i, utility);
  WriteDouble(i, timestamp);
}

uint32_t PaperControlHeader::Deserialize(Buffer::Iterator i)
{
  if (i.GetRemainingSize() < GetSerializedSize()) { m_wellFormed = false; return 0; }
  kind = static_cast<Kind>(i.ReadU8());
  nodeId = i.ReadNtohU16();
  headId = i.ReadNtohU16();
  clusterId = i.ReadNtohU16();
  epoch = i.ReadNtohU32();
  stateEpoch = i.ReadNtohU32();
  transaction = i.ReadNtohU16();
  role = static_cast<NodeRole>(i.ReadU8());
  const uint8_t flag = i.ReadU8();
  forwarder = flag == 1;
  m_wellFormed = flag <= 1;
  position = ReadVector(i);
  velocity = ReadVector(i);
  originEnergy = ReadDouble(i);
  residualEnergy = ReadDouble(i);
  utility = ReadDouble(i);
  timestamp = ReadDouble(i);
  return GetSerializedSize();
}

bool PaperControlHeader::IsValid() const
{
  return m_wellFormed && kind >= HELLO && kind <= UTILITY && nodeId != 0xffff &&
      stateEpoch <= epoch && ValidRole(role) && FiniteVector(position) && FiniteVector(velocity) &&
      ValidEnergy(originEnergy, residualEnergy) &&
      std::isfinite(utility) && std::isfinite(timestamp) && timestamp >= 0;
}

void PaperControlHeader::Print(std::ostream& os) const
{
  os << "PAPER_CONTROL kind=" << static_cast<unsigned>(kind) << " node=" << nodeId
     << " head=" << headId << " cluster=" << clusterId << " epoch=" << epoch
     << " stateEpoch=" << stateEpoch << " transaction=" << transaction;
}

} // namespace icra
} // namesapce ns3
