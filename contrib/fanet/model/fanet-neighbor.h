/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef FANET_NEIGHBOR_H
#define FANET_NEIGHBOR_H

#include "ns3/vector.h"
#include "ns3/simulator.h"
#include "ns3/timer.h"
#include "ns3/node.h"
#include "ns3/node-list.h"
#include "ns3/object.h"
#include "fanet.h"
#include "ns3/wifi-mac-header.h"
#include <map>
#include <optional>

namespace ns3{

// class PerformanceMetrics;  // Forward declaration.


class Neighbors : public Object
{
public:
    Neighbors();
    Neighbors(uint16_t myId);
    virtual ~Neighbors(){};
    static TypeId GetTypeId();
    // Opt-in advertisement freshness. Existing protocols remain UNTRACKED.
    // Kept in an aggregated object so NeighborInfo and Neighbors retain their layout.
    struct ComputeObservation
    {
        enum Status { UNTRACKED, UNKNOWN, INVALID, EXPIRED, FRESH };
        Status status = UNTRACKED;
        std::optional<Time> receivedAt;
        std::optional<Time> expiresAt;
        bool IsUsable() const { return status == UNTRACKED || status == FRESH; }
        const char* StatusName() const;
    };
    void EnableComputeFreshness();
    // Call only after storing an advertisement that actually carries both fields.
    void RecordComputeAdvertisement(uint16_t nodeId, Time lifetime);
    // Passive: does not purge, refresh a timestamp, or read another node's state.
    ComputeObservation GetComputeObservation(uint16_t nodeId) const;
    bool HasUsableComputeState(uint16_t nodeId) const;
    /*  Define The Structure of Neighbors Informantion */
    struct NeighborInfo
    {
        uint16_t        NodeId;
        Ipv4Address     ipAddress;
        uint16_t        clusterId;
        // Mac48Address    HardwareAddress;
        Vector          Position;
        Vector          Velocity;
        double          ResEnergy;
        double          FitnessValue;
        NodeRole        Role;
        Time            UpdateTime;
        Time            NeighborLifeLength;
        // ---- Compute state ----
        double          computeCapacity;  // Neighbor compute capacity in GFLOPS.
        float           currentLoad;      // Neighbor load in [0.0, 1.0].

        // NeighborInfo() =  default;
        NeighborInfo( uint16_t     id,
                      Ipv4Address  ip,
                      uint16_t     clusterId,
                      /*Mac48Address mac,*/
                      Vector       pos,
                      Vector       vel,
                      double       energy,
                      double       fitness,
                      NodeRole     role,
                      Time         LatestTime,
                      Time         lifelenth,
                      double       compCap = 1.0,
                      float        curLoad = 0.0f)
            : NodeId(id),
              ipAddress(ip),
              clusterId(clusterId),
              /*HardwareAddress(mac),*/
              Position(pos),
              Velocity(vel),
              ResEnergy(energy),
              FitnessValue(fitness),
              Role(role),
              UpdateTime(LatestTime),
              NeighborLifeLength(lifelenth),
              computeCapacity(compCap),
              currentLoad(curLoad) {}
    };
    // void ChangeMyClusterId(uint16_t Id) {m_ClusterId = Id;}
    // void ChangeMyRole(NodeRole role)    {m_role = role;}
    uint16_t     GetMyNodeId()          {return m_NodeId;}
    /*  Neignbors Informantions Management*/
    bool         IsNeighbor         ( uint16_t NeighborId);
    void         AddNeighbor(NeighborInfo newNeighbor);
    void         DeleteNeighbor(uint16_t NeighborId);

    /* Vector       GetVelocity(uint16_t NodeId) {return GetAll().at(NodeId).Velocity;}*/
    void         Purge(); /* remove the expired neighbors */
    // void         ScheduleTimer();
    void         Clear(); /* Clear all neithbor info */
    Callback<void, WifiMacHeader const&> GetTxErrorCallBack()     {return m_txErrorCallback;}

    /*---Next Hop Find Function----*/
    /// \param linkAware if true, apply link-lifetime, distance-quality and
    ///        intra-cluster weights to the candidate fitness.
    uint16_t                GreedyNeighbor(Vector dstPos, Vector myPos, uint16_t prevHopId = 0xFFFF,
                                           Vector myVel = Vector(0, 0, 0), bool linkAware = false,
                                           double maxRange = 500.0, uint16_t myClusterId = 0xFFFF,
                                           bool predictPosition = false);
    uint16_t                PerimeterNextHop(Vector dstPos, Vector myPos, Vector & lasthopPos);
    /// Heterogeneous FANET perimeter forwarding: pre-filter the candidate
    /// pool by myMaxRange, then apply the right-hand rule. A non-positive
    /// myMaxRange disables filtering and is equivalent to PerimeterNextHop.
    uint16_t                PerimeterNextHopWithinRange(Vector dstPos, Vector myPos,
                                                        Vector & lasthopPos, double myMaxRange);
    // std::vector<uint16_t>   GetNeighborInRange (double angle, Vector dstPos, Vector myPos);
    bool                    IsLocalMaximum(Vector dstPos, Vector myPos);
    uint16_t                GetNearestClusterHead(Vector dstPos);    // Nearest neighboring cluster head.
    // double                  CalculateProgress(uint16_t neighborId, Vector dstPos, Vector myPos);

    // uint16_t     BestNeighbor(uint16_t DstId,uint16_t myId, NodeRole myRole, uint16_t dstChId, uint16_t myChId, Vector dstPos, Vector myPos);
    uint16_t     NearestNeighbor(Vector dstPos, Vector myPos);
    uint16_t     GetNeighborDegree()            { return GetAll().size();}    
    bool         IsEmpty() const                { return m_neighborTable.empty(); }
    const std::map<uint16_t,NeighborInfo>& GetAll();    
    // Passive diagnostics: unlike GetAll(), this never purges or reschedules.
    const std::map<uint16_t,NeighborInfo>& PeekAll() const { return m_neighborTable; }
    
private:
    std::map<uint16_t, NeighborInfo>            m_neighborTable;
    Callback<void, WifiMacHeader const &>       m_txErrorCallback;
    // Callback<void, uint16_t>                    m_handleLinkFailure;
    Timer                                       m_timer;
    uint16_t                                    m_NodeId;
    // uint16_t                                    m_ClusterId;
    // NodeRole                                    m_role;
    // Ptr<ClusterNode>                            m_clusternode;
    void   ProcessTxError(WifiMacHeader const &);
    
    double      GetAngle(Vector v1, Vector v2);
    uint16_t    RightHandRule(std::vector<uint16_t> candidates, Vector dstPos, Vector myPos);
};


} /* namsp ns3*/




#endif /* FANET_NEIGHBOR_H */
