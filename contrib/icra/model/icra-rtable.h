/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef ICRA_RTABLE_H
#define ICRA_RTABLE_H

#include <stdint.h>
#include <cassert>
#include <map>
#include <sys/types.h>
#include "ns3/ipv4.h"
#include "ns3/ipv4-route.h"
#include "ns3/timer.h"
#include "ns3/vector.h"
#include "ns3/net-device.h"
#include "ns3/output-stream-wrapper.h"
#include "icra-packet.h"

namespace ns3{
namespace icra{


enum RouteType{
    ROUTE_DIRECT,        // Direct-neighbor route.
    ROUTE_INTRA_CLUSTER, // Intra-cluster route.
    ROUTE_INTER_CLUSTER, // Inter-cluster route.
    ROUTE_GREEDY,        // GPSR greedy route.
    ROUTE_PERIMETER      // GPSR perimeter route.
};


class RoutingTableEntry
{
public:
    RoutingTableEntry();
    RoutingTableEntry(
        Ipv4Address dst, uint16_t dstNodeId, uint16_t dstClusterId, Vector DestPos,
        Ipv4Address NextHop, uint16_t nexthopNodeId, NodeRole NextHopRole, Vector NextHopPos,
        RouteType type, uint16_t HopCount );

    /* Get ****  */
    Ipv4Address GetDestAddress()   const   { return m_Destination;}
    uint16_t    GetDestNodeId()    const   { return m_DstNodeId;}
    uint16_t    GetDestClusterId() const   { return m_DstClusterId;}
    Vector      GetDestPosition()  const   { return Vector(m_DstPosX,m_DstPosY,m_DstPosZ);}

    Ipv4Address GetNextHopAddress()  const { return m_NextHop;}
    uint16_t    GetNextHopNodeId()   const { return m_NextHopNodeId;}
    NodeRole    GetNextHopRole()     const { return m_NextHopRole;}
    Vector      GetNextHopPosition() const { return Vector(m_NextHopPosX,m_NextHopPosY,m_NextHopPosZ);}

    bool        IsValid() const             { return m_isValid;}


    RouteType   GetRouteType()    const      { return m_RouteType;}
    uint16_t    GetHopCounts()    const      { return m_HopCount;}
    double      GetRouteFitness() const      { return m_PathFitness;}
    Time        GetLifeTime()     const      { return m_LifeTime;}
    // Time        GetUpdateTime()   const      { return m_LastUpdateTime;}

    /* Set *****  */
    void   SetDestAddress(Ipv4Address dst)      { m_Destination = dst;}
    void   SetDestNodeId(uint16_t NodeId)       { m_DstNodeId = NodeId;}
    void   SetDestClusterId(uint16_t clusterId) { m_DstClusterId = clusterId;}
    void   SetDestPosition(Vector pos);
    void   SetNextHopAddress(Ipv4Address dst)   { m_NextHop = dst;}
    void   SetNextHopNodeId(uint16_t nodeId)    { m_NextHopNodeId = nodeId;}
    void   SetNextHopPosition(Vector pos);
    void   SetNextHopRole(NodeRole role)        { m_NextHopRole = role;}

    void   SetExpireTime(Time t)                { m_LifeTime = t;}
    void   SetValid(bool j)                     { m_isValid  = j;}
    void   SetHopCount(uint16_t count)          { m_HopCount = count;}
    void   SetRouteType(RouteType t)            { m_RouteType = t;}
    void   SetRouteFitness(double v)            { m_PathFitness = v;}

    void   UpdateLifetime(Time t);
    bool   IsExpired() const;

private:

    Ipv4Address  m_Destination;   //
    uint16_t     m_DstNodeId;
    uint16_t     m_DstClusterId;
    uint16_t     m_DstPosX, m_DstPosY, m_DstPosZ;

    // bool         m_NextHopIsCH;
    Ipv4Address  m_NextHop;        //
    uint16_t     m_NextHopNodeId;
    NodeRole     m_NextHopRole;    //
    uint16_t     m_NextHopPosX, m_NextHopPosY, m_NextHopPosZ;

    bool         m_isValid;        //

    uint16_t     m_HopCount;       //
    RouteType    m_RouteType;      //
    double       m_PathFitness;
    Time         m_LifeTime;       //
    // Time         m_LastUpdateTime; //

};



class RoutingTable
{
public:
    RoutingTable();

    bool AddRoute(RoutingTableEntry& entry);
    bool DeleteRoute(Ipv4Address dst);
    bool LookupRoute(Ipv4Address dst, RoutingTableEntry& entry);
    bool UpdateRoute(Ipv4Address dst, RoutingTableEntry& entry);

    // GPSR helpers.
    bool GetNearestNeighborToDst(Vector dstPos, Ipv4Address& nextHop);
    std::vector<RoutingTableEntry> GetNeighborsCloserToDst(Vector dstPos, Vector myPos);

    // Route-table maintenance.
    void Purge();  // Remove expired routes.
    void Clear();  // Clear the route table.
    uint32_t GetSize() const { return m_Routes.size();}

    // Print the route table.
    void Print(Ptr<OutputStreamWrapper> stream) const;

private:

    std::map<Ipv4Address, RoutingTableEntry>    m_Routes;         // Direct-neighbor routes.
    Time m_entryLifeTime;  // Default route lifetime.
    std::map<uint16_t, Ipv4Address>             m_clusterHeadTable;  // clusterId -> CH IP.
};


} //namespace icra
} //namespace ns3

#endif /* ICRA_RTABLE_H */