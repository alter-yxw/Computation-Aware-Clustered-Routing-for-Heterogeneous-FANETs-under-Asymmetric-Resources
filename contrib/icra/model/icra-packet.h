/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef ICRA_PACKET_H
#define ICRA_PACKET_H

// #include <iostream>
// #include <cstring>
#include "ns3/header.h"
#include "ns3/fanet-node.h"
#include "ns3/enum.h"
#include "ns3/ipv4-address.h"
#include <map>
#include <array>
#include "ns3/nstime.h"
#include "ns3/vector.h"

namespace ns3{
namespace icra{

enum MessageType {
    ICRA_HELLO        = 1,  //!< BioCluster: Hello head at initial state, to develop the neighbor degree and fitness of being a ClusterHead  
    ICRA_CH_CALL      = 2,  //!< BioCluster: One Node Declare to be a CH, and call for join.
    ICRA_JOIN_REQ     = 3,  //!< BioCluster: One Node receive the CALLs, and join one CH after compare the fitness
    ICRA_JOIN_RESP    = 4,  //!< BioCluster: CH reply acknowledge, infrom CM had succeed  to Join the cluster
    ICRA_CH_LIVE      = 5,  //!< BioCluster: CH peroidly infrom the CM ailve
    ICRA_CM_LIVE      = 6,  //!< BioCluster: CM peroidly inform CH alive
    ICRA_UTILITY      = 7,  //!< ICRA: exchange the utility value
    ICRA_GS_REPORT    = 8,  //!< Report on the ideal out-of-band GS link.
    ICRA_GS_POLICY    = 9,  //!< Policy on the ideal out-of-band GS link.
    ICRA_PAPER_CONTROL = 10 //!< Epoch-aware paper clustering and IFN handshake.


    // ICRA_RREQ         = 7,  //!< BioCluster: Routing Request
    // ICRA_RREP         = 8,  //!< BioCluster: Routing Reply
    // ICRA_RERR         = 9   //!< BioCluster: Routing Error
};



class TypeHeader: public Header
{
    /// c-tor
public:
    // TypeHeader ();
    TypeHeader (MessageType t = ICRA_HELLO);
    ~TypeHeader(){}; 

    ///\name Header serialization/deserialization
    static TypeId   GetTypeId();
    TypeId          GetInstanceTypeId() const;
    uint32_t        GetSerializedSize() const;
    void            Serialize(Buffer::Iterator start) const;
    uint32_t        Deserialize (Buffer::Iterator Start);
    void            Print (std::ostream &os) const;

    //Return Type
    MessageType Get () const { return m_type; }
    //check if type if vaild
    bool IsValid() const     { return m_valid;}
    bool operator== (TypeHeader const & o) const;

private:
    MessageType m_type;
    bool m_valid;
};

std::ostream & operator<< (std::ostream & os, TypeHeader const & h);

/*Hello Packet*/

class HelloHeader :public Header
{
public:
    HelloHeader ( NodeRole role = Idle,
                  uint16_t NodeId = -1,
                  uint16_t clusterId = -1, 
                  Vector position = Vector(0, 0, 0),
                  Ipv4Address addr = "",
                  Vector velocity = Vector(0.0, 0.0, 0.0),
                  double InitialEnergy = 100,
                  double ResidualEnergy = 100,
                  double fitness = 0);

    ~HelloHeader();
    static TypeId GetTypeId();
    TypeId      GetInstanceTypeId() const;
    uint32_t    GetSerializedSize () const;
    void        Serialize (Buffer::Iterator start) const;
    uint32_t    Deserialize(Buffer::Iterator start);
    void        Print(std::ostream &os) const;


    void        SetNodeId (uint16_t id)           { m_nodeid=id;}
    uint16_t    GetNodeId (void) const            { return m_nodeid;}

    void        SetClusterId(uint16_t id)         { m_clusterId = id;}
    uint16_t    GetClusterId() const              { return m_clusterId;}
    
    void        SetIpAddress(Ipv4Address addr)    { m_address = addr;}
    Ipv4Address GetIpAddress() const              { return m_address;}

    void        SetPosition(Vector pos); 
    Vector      GetPosition() const;
    
    void        SetRole(NodeRole role)            { m_role = role;}
    NodeRole    GetRole() const                   { return m_role;}
    
    void        SetVelocity(Vector vel);
    Vector      GetVelocity() const;

    void        SetInitialEnergy (double energy)  { m_InitialEnergy = energy;}
    double      GetInitialEnergy () const         { return m_InitialEnergy;}
    void        SetResidualEnergy (double energy) { m_ResidualEnergy = energy;}
    double      GetResidualEnergy () const        { return m_ResidualEnergy;}

    void        SetFitness(double value)          { m_FitnessValue = value;}
    double      GetFitness() const                { return m_FitnessValue;}

    // void        SetTimeStamp (double timestamp)  { m_timestamp = timestamp;}
    // double      GetTimeStamp (void) const        { return m_timestamp;}

    bool operator== (HelloHeader const &o) const;

private:
    NodeRole        m_role;             ///< Node Role (Idle,Member,Head,BackupHead,Isoloate)
    uint16_t        m_nodeid;           ///< Device node id
    uint16_t        m_clusterId;        ///< cluster Id
    // uint16_t        m_NeighborDegree;   ///< neighbor degree    
    uint16_t        m_PosX;             ///< Position x
    uint16_t        m_PosY;             ///< Position y
    uint16_t        m_PosZ;             ///< Position z
    Ipv4Address     m_address;          ///< Device IP address 32-bit
    float           m_VelX;             ///< Velocity x
    float           m_VelY;             ///< Velocity y
    float           m_VelZ;             ///< Velocity z
    double          m_InitialEnergy;    ///< Initial Energy
    double          m_ResidualEnergy;   ///< Residual Energy    
    double          m_FitnessValue;     ///< FitnessValue
    // double          m_timestamp;        ///< Packet send Timestamp
};

std::ostream & operator<< (std::ostream & os, HelloHeader const &);

/*Cluster Head Announce Header Section*/
class ClusterHeadAnnounceHeader : public Header
{
public:
    ClusterHeadAnnounceHeader(uint16_t NodeId = 0, double ResidualEnergy = 100, Ipv4Address addr = "", 
                              Vector pos = Vector(0,0,0), Vector vel = Vector(0,0,0),  double FitnessValue =0, double Time = 0);
    ~ClusterHeadAnnounceHeader(){};
    ///\name Header Serialize / Deserialize
    static TypeId   GetTypeId         ();
    TypeId          GetInstanceTypeId () const;
    uint32_t        GetSerializedSize () const;
    void            Serialize   (Buffer::Iterator start) const;
    uint32_t        Deserialize (Buffer::Iterator start);
    void            Print       (std::ostream &os) const;

    ///\name Field
    void        SetNodeId(uint16_t id)                { m_nodeid = id;}
    uint16_t    GetNodeId () const                    { return m_nodeid;}

    void        SetResidualEnergy(double ResEnergy)   { m_ResidualEnergy = ResEnergy;}
    double      GetResidualEnergy() const             { return m_ResidualEnergy;}

    void        SetIpAddress(Ipv4Address addr)        { m_address = addr;}
    Ipv4Address GetIpAddress() const                  { return m_address;}

    Vector      GetPosition() const;
    void        SetPosition(Vector pos);

    void        SetVelocity(Vector vel);
    Vector      GetVelocity() const;

    void        SetFitnessValue(double FitnessValue)  { m_FitnessValue = FitnessValue;}
    double      GetFitnessValue() const               { return m_FitnessValue;}

    void        SetTimeStamp(double timestamp)        { m_timestamp = timestamp;}
    double      GetTimeStamp(void) const              { return m_timestamp;}

    bool operator== (ClusterHeadAnnounceHeader const &o) const;

private:
    uint16_t        m_nodeid;
    double          m_ResidualEnergy;
    Ipv4Address     m_address;
    // uint16_t     m_NeighborDegree;
    uint16_t        m_CHpositionX;
    uint16_t        m_CHpositionY;
    uint16_t        m_CHpositionZ;
    float           m_CHVelocityX;
    float           m_CHVelocityY;
    float           m_CHVelocityZ;
    double          m_FitnessValue;
    double          m_timestamp;
    

};
std::ostream & operator<< (std::ostream & os, ClusterHeadAnnounceHeader const &);

/* Node Request to be the Cluster Member*/
class JoinClusterRequestHeader : public Header
{
public:
    JoinClusterRequestHeader(uint16_t NodeId = 0, NodeRole role = Idle, Ipv4Address Addr = "", double ResidualEnergy = 0,
                             double Time = 0, Vector Position = Vector(0,0,0), Vector Velocity = Vector(0,0,0));
    ~JoinClusterRequestHeader(){};
    ///\name JoinClusterRequest serialize / Deserialize
    static TypeId   GetTypeId();
    TypeId          GetInstanceTypeId () const;
    uint32_t        GetSerializedSize () const;
    void            Serialize (Buffer::Iterator start) const;
    uint32_t        Deserialize (Buffer::Iterator start);
    void            Print (std::ostream &os) const;    

    ///\name Field
    void            SetNodeId(uint16_t id )             { m_nodeid = id;}
    uint16_t        GetNodeId () const                  { return m_nodeid;}

    void            RecordRole(NodeRole role)           { m_Role = role;}
    NodeRole        GetRole() const                     { return m_Role;}

    void            SetIpAddress(Ipv4Address addr)      { m_address = addr;}
    Ipv4Address     GetIpAddress() const                { return m_address;}

    void            SetResidualEnergy(double ResEnergy) { m_ResidualEnergy = ResEnergy;}
    double          GetResidualEnergy() const           { return m_ResidualEnergy;}

    void            SetTimeStamp(Time timestamp)        { m_timestamp = timestamp.GetTimeStep();}
    double          GetTimeStamp(void) const            { return m_timestamp;}

    void            SetPosition (Vector pos);
    Vector          GetPosition () const;

    void            SetVelocity (Vector vel);
    Vector          GetVelocity () const;

    bool operator== (JoinClusterRequestHeader const &o) const;

private:
    uint16_t    m_nodeid;
    NodeRole    m_Role;
    Ipv4Address m_address;
    double      m_ResidualEnergy;
    double      m_timestamp;
    uint16_t    m_CMpositionX;
    uint16_t    m_CMpositionY;
    uint16_t    m_CMpositionZ;
    float       m_CMVelocityX;
    float       m_CMVelocityY;
    float       m_CMVelocityZ;

};
std::ostream & operator<< (std::ostream & os, JoinClusterRequestHeader const &);

/* Cluster Head Reply to the Cluster Member, and keep telling that CH is Online*/
class ClusterHeadResponseHeader : public Header
{
public:
    ClusterHeadResponseHeader(uint16_t ChNodeId = 0, uint16_t clusterId = 0, Ipv4Address IpAddr = "", double timestamp = 0);
    ~ClusterHeadResponseHeader(){};
    ///\name ClusterHeadResponse Serialize / Deserialize
    static TypeId   GetTypeId();
    TypeId          GetInstanceTypeId() const;
    uint32_t        GetSerializedSize() const;
    void            Serialize(Buffer::Iterator start) const;
    uint32_t        Deserialize(Buffer::Iterator start);
    void            Print (std::ostream &os) const;  
    bool            operator== (ClusterHeadResponseHeader const &o) const;

    ///\name Field
    void         SetChNodeId(uint16_t ChNodeId )         { m_nodeid = ChNodeId;}
    uint16_t     GetChNodeId () const                    { return m_nodeid;}

    void         SetClusterId(uint16_t ClusterId)        { m_clusterId = ClusterId;}
    uint16_t     GetClusterId() const                    { return m_clusterId;}

    void         SetIpv4Address(Ipv4Address ip)          { m_address = ip;}
    Ipv4Address  GetIpv4Address() const                  { return m_address;}

    void         SetTimeStamp(double timestamp)            { m_timestamp = timestamp;}
    double       GetTimeStamp(void) const                { return m_timestamp;}
 

private:
    uint16_t     m_nodeid;
    uint16_t     m_clusterId;
    Ipv4Address  m_address;
    double       m_timestamp;

};
std::ostream & operator<< (std::ostream & os, ClusterHeadResponseHeader const &);

/* Cluster Head Alive Beacon Head , indicate that the Cluster Head is Still normally working*/
class ClusterHeadBeaconHeader : public Header
{
public:
    ClusterHeadBeaconHeader( uint16_t ChNodeId = 0, uint16_t ClusterId = 0, Ipv4Address ip = "",
                             Vector pos = Vector(0,0,0), Vector vel = Vector(0,0,0), uint8_t MemberDegree = 0,
                             double residualEnergy = 0.0,double fitvalue =0.0, double LatestTime = 0);
    ~ClusterHeadBeaconHeader(){};

    static TypeId   GetTypeId();
    TypeId          GetInstanceTypeId() const;
    uint32_t        GetSerializedSize() const;
    void            Serialize(Buffer::Iterator start) const;
    uint32_t        Deserialize(Buffer::Iterator start);
    void            Print(std::ostream &os) const;

    ///\name Field
    void            SetChNodeId(uint16_t ChNodeId)          { m_ChNodeId = ChNodeId;}
    uint16_t        GetChNodeId() const                     { return m_ChNodeId;}

    void            SetClusterId(uint16_t ClusterId)        { m_ClusterId = ClusterId;}
    uint16_t        GetClusterId() const                    { return m_ClusterId;}
    void            SetAddress(Ipv4Address addr)            { m_address = addr;}
    Ipv4Address     GetAddress() const                      { return m_address;}

    void            SetPosition(Vector pos);
    Vector          GetPosition() const                     { return Vector(m_posX, m_posY, m_posZ);}
    void            SetVelocity(Vector Vel);
    Vector          GetVelocity() const                     { return Vector(m_velX, m_velY, m_velZ);}
    
    void            SetMemberDegree(uint8_t MemberDegree)   { m_MemberDegree = MemberDegree;}
    uint8_t         GetMemberDegree() const                 { return m_MemberDegree;}

    void            UpdateFitness(double fit)               { m_FitnessValue = fit;}
    double          GetFitnessValue() const                 { return m_FitnessValue;}
    
    void            UpdateEnergy(double residualEnergy)     { m_ResidualEnergy = residualEnergy;}
    double          GetResidualEnergy() const               { return m_ResidualEnergy;}

    void            SetLatestTime(double LatestTime)        { m_LatestTime = LatestTime;}
    double          GetLatestTime() const                   { return m_LatestTime;}

    bool           operator== (ClusterHeadBeaconHeader const &o) const;

private:
    uint16_t        m_ChNodeId;
    uint16_t        m_ClusterId;
    Ipv4Address     m_address;
    uint16_t        m_posX;             
    uint16_t        m_posY;
    uint16_t        m_posZ;
    float           m_velX;
    float           m_velY;
    float           m_velZ;
    uint8_t         m_MemberDegree;
    double          m_ResidualEnergy;
    double          m_FitnessValue;
    double          m_LatestTime;

};
std::ostream & operator<< (std::ostream & os, ClusterHeadBeaconHeader const &);



/*Cluster Member Beacon Header, inform the clusterHead the the Member is still in the satisfied communication range*/
class ClusterMemberBeaconHeader : public Header
{
public:
    ClusterMemberBeaconHeader( uint16_t NodeId = 0, uint16_t ClusterId = 0, Ipv4Address addr = "", Vector pos = Vector(0,0,0),
                               Vector vel = Vector(0,0,0), double fitvalue=0.0, double LatestTime = 0.0);
    ~ClusterMemberBeaconHeader(){};

    static TypeId   GetTypeId();
    TypeId          GetInstanceTypeId() const;
    uint32_t        GetSerializedSize() const;
    void            Serialize(Buffer::Iterator start) const;
    uint32_t        Deserialize(Buffer::Iterator start);
    void            Print(std::ostream &os) const;

    ///\name Field
    void            SetNodeId(uint16_t NodeId)              { m_NodeId = NodeId;}
    uint16_t        GetNodeId() const                       { return m_NodeId;}
    void            SetClusterId(uint16_t ClusterId)        { m_ClusterId = ClusterId;}
    uint16_t        GetClusterId() const                    { return m_ClusterId;}

    void            SetAddress(Ipv4Address addr)            { m_address = addr;}
    Ipv4Address     GetAddress() const                      { return m_address;}
    
    void            SetPosition(Vector pos);         
    Vector          GetPosition() const                     { return Vector(m_posX, m_posY, m_posZ);}
    void            SetVelocity(Vector Vel);
    Vector          GetVelocity() const                     { return Vector(m_velX, m_velY, m_velZ);}

    void            UpdateFitness(double fit)               { m_FitnessValue = fit;}
    double          GetFitnessValue() const                 { return m_FitnessValue;}
    void            SetLatestTime(double LatestTime)        { m_LatestTime = LatestTime;}
    double          GetLatestTime() const                   { return m_LatestTime;}
    
    bool            operator== (ClusterMemberBeaconHeader const &o) const;
private:
    uint16_t        m_NodeId;
    uint16_t        m_ClusterId;
    Ipv4Address     m_address;
    uint16_t        m_posX;             
    uint16_t        m_posY;
    uint16_t        m_posZ;
    float           m_velX;
    float           m_velY;
    float           m_velZ;
    double          m_FitnessValue;
    double          m_LatestTime;

};
std::ostream & operator<< (std::ostream & os, ClusterMemberBeaconHeader const &);


/*ICRA Utility Message Header*/
class UtilityHeader : public Header
{
public:
    UtilityHeader(uint16_t NodeId = 0, double utilityValue = 0.0, Ipv4Address addr = "", 
                  Vector pos = Vector(0,0,0), Vector vel = Vector(0,0,0));
    ~UtilityHeader(){};
    
    static TypeId   GetTypeId();
    TypeId          GetInstanceTypeId() const;
    uint32_t        GetSerializedSize() const;
    void            Serialize(Buffer::Iterator start) const;
    uint32_t        Deserialize(Buffer::Iterator start);
    void            Print(std::ostream &os) const;

    void        SetNodeId(uint16_t id)              { m_nodeId = id; }
    uint16_t    GetNodeId() const                   { return m_nodeId; }
    
    void        SetUtilityValue(double utility)     { m_utilityValue = utility; }
    double      GetUtilityValue() const             { return m_utilityValue; }
    
    void        SetIpAddress(Ipv4Address addr)      { m_address = addr; }
    Ipv4Address GetIpAddress() const                { return m_address; }
    
    void        SetPosition(Vector pos);
    Vector      GetPosition() const;
    
    void        SetVelocity(Vector vel);
    Vector      GetVelocity() const;
    
    bool operator== (UtilityHeader const &o) const;

private:
    uint16_t    m_nodeId;
    double      m_utilityValue;
    Ipv4Address m_address;
    uint16_t    m_posX;
    uint16_t    m_posY;
    uint16_t    m_posZ;
    float       m_velX;
    float       m_velY;
    float       m_velZ;
};

std::ostream & operator<< (std::ostream & os, UtilityHeader const &);


// Serialized messages on an ideal, delayed out-of-band GS link. Their bytes
// are accounted separately from A2A IP transmissions.
class GsReportHeader : public Header
{
public:
    GsReportHeader() = default;
    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const override;
    uint32_t GetSerializedSize() const override;
    void Serialize(Buffer::Iterator start) const override;
    uint32_t Deserialize(Buffer::Iterator start) override;
    void Print(std::ostream& os) const override;
    bool IsValid() const;
    uint16_t GetNodeId() const { return nodeId; }
    const std::array<double, 4>& GetFactors() const { return factors; }
    double GetEnergyChange() const { return energyChange; }
    uint32_t GetRoleChanges() const { return roleChanges; }
    uint32_t GetEpoch() const { return epoch; }
    uint64_t GetTimestampNs() const { return timestampNs; }
    double GetOriginEnergy() const { return originEnergy; }
    double GetResidualEnergy() const { return residualEnergy; }
    Vector GetPosition() const { return position; }
    Vector GetVelocity() const { return velocity; }
    double GetSpeed() const;
    uint16_t GetDegree() const { return degree; }
    NodeRole GetRole() const { return role; }

    uint16_t nodeId = 0xffff;
    std::array<double, 4> factors{};
    double energyChange = 0;
    uint32_t roleChanges = 0;
    uint32_t epoch = 0;
    uint64_t timestampNs = 0;
    double originEnergy = 100;
    double residualEnergy = 100;
    Vector position;
    Vector velocity;
    uint16_t degree = 0;
    NodeRole role = Idle;
private:
    bool m_wellFormed = true;
};

class GsPolicyHeader : public Header
{
public:
    GsPolicyHeader() = default;
    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const override;
    uint32_t GetSerializedSize() const override;
    void Serialize(Buffer::Iterator start) const override;
    uint32_t Deserialize(Buffer::Iterator start) override;
    void Print(std::ostream& os) const override;
    bool IsValid() const;
    const std::array<double, 4>& GetWeights() const { return weights; }
    uint32_t GetEpoch() const { return epoch; }
    uint64_t GetTimestampNs() const { return timestampNs; }

    std::array<double, 4> weights{{0.25, 0.25, 0.25, 0.25}};
    uint32_t epoch = 0;
    uint64_t timestampNs = 0;
private:
    bool m_wellFormed = true;
};

// Full-precision local state and explicit epochs for paper mode. Legacy headers
// above retain their original layout for the runtime legacy preset.
class PaperControlHeader : public Header
{
public:
    enum Kind : uint8_t {
        HELLO = 1, CALL = 2, JOIN = 3, ACCEPT = 4,
        IFN_REQUEST = 5, IFN_ACK = 6, LEAVE = 7, UTILITY = 8
    };
    PaperControlHeader() = default;
    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const override;
    uint32_t GetSerializedSize() const override;
    void Serialize(Buffer::Iterator start) const override;
    uint32_t Deserialize(Buffer::Iterator start) override;
    void Print(std::ostream& os) const override;
    bool IsValid() const;

    Kind kind = HELLO;
    uint16_t nodeId = 0xffff;
    uint16_t headId = 0xffff;
    uint16_t clusterId = 0xffff;
    uint32_t epoch = 0;
    // Epoch in which the advertised role/head state was established. A CH that
    // has not yet re-declared after a strategy update keeps its previous value,
    // so peers can tell a current declaration from a still-serving old one.
    uint32_t stateEpoch = 0;
    // JOIN/ACCEPT transaction identifier. ACCEPT echoes the JOIN value so a
    // delayed response can never close a different join attempt.
    uint16_t transaction = 0;
    NodeRole role = Idle;
    bool forwarder = false;
    Vector position;
    Vector velocity;
    double originEnergy = 100;
    double residualEnergy = 100;
    double utility = 0;
    double timestamp = 0;
private:
    bool m_wellFormed = true;
};

} // namespace icra
} // namespace ns3 

#endif /*icra_PACKET_H*/
