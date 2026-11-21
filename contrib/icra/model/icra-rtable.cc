/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
/*
 Routing Table Entry
*/
#include "icra-rtable.h"
#include <algorithm>
#include <iomanip>
#include "ns3/simulator.h"
#include "ns3/log.h"

namespace ns3{

NS_LOG_COMPONENT_DEFINE("IcraRoutingTable");

namespace icra{


RoutingTableEntry::RoutingTableEntry()
    :       m_Destination(Ipv4Address()),
            m_DstNodeId(0xFFFF), m_DstClusterId(0xFFFF),
            m_DstPosX(0), m_DstPosY(0), m_DstPosZ(0),
            m_NextHop(Ipv4Address()),
            m_NextHopNodeId(0xFFFF), m_NextHopRole(Idle),
            m_NextHopPosX(0), m_NextHopPosY(0), m_NextHopPosZ(0),
            m_isValid(false),
            m_HopCount(0), m_RouteType(ROUTE_DIRECT), m_PathFitness(0.0),
            m_LifeTime(Seconds(0))
{    
}

RoutingTableEntry::RoutingTableEntry(
                Ipv4Address dst, uint16_t dstNodeId, uint16_t dstClusterId, Vector DestPos,
                Ipv4Address NextHop, uint16_t nexthopNodeId, NodeRole NextHopRole, Vector NextHopPos,
                RouteType type, uint16_t HopCount)
    :   m_Destination(dst),
        m_DstNodeId(dstNodeId), m_DstClusterId(dstClusterId),
        m_NextHop(NextHop),
        m_NextHopNodeId(nexthopNodeId), m_NextHopRole(NextHopRole),
        m_isValid(true),
        m_HopCount(HopCount), m_RouteType(type), m_PathFitness(0.0),
        m_LifeTime(Seconds(30))
{
    SetDestPosition(DestPos);
    SetNextHopPosition(NextHopPos);
};

void RoutingTableEntry::SetDestPosition(Vector pos)
{
    m_DstPosX = static_cast<uint16_t>(pos.x);
    m_DstPosY = static_cast<uint16_t>(pos.y);
    m_DstPosZ = static_cast<uint16_t>(pos.z);
}

void RoutingTableEntry::SetNextHopPosition(Vector pos)
{
    m_NextHopPosX = static_cast<uint16_t>(pos.x);
    m_NextHopPosY = static_cast<uint16_t>(pos.y);
    m_NextHopPosZ = static_cast<uint16_t>(pos.z);
}

void RoutingTableEntry::UpdateLifetime(Time newtime)
{
    m_LifeTime = Simulator::Now() + newtime;
}

bool RoutingTableEntry::IsExpired() const
{
    return m_LifeTime < Simulator::Now();
}





RoutingTable::RoutingTable()
    :   m_entryLifeTime(Seconds(10.0))
{
}

bool RoutingTable::AddRoute(RoutingTableEntry & entry)
{
    NS_LOG_FUNCTION(this<<entry.GetDestAddress());
    if(m_Routes.find(entry.GetDestAddress()) != m_Routes.end())
    {
        return UpdateRoute(entry.GetDestAddress(), entry);
    }

    entry.UpdateLifetime(m_entryLifeTime);
    m_Routes[entry.GetDestAddress()] = entry;
    return true;
}

bool RoutingTable::DeleteRoute(Ipv4Address dst)
{
    NS_LOG_FUNCTION(this << dst);

    auto it = m_Routes.find(dst);
    if( it != m_Routes.end() ) {
        m_Routes.erase(it);
        return true;
    }
    return false;
}

bool RoutingTable::LookupRoute(Ipv4Address dst, RoutingTableEntry & entry)
{
    NS_LOG_FUNCTION(this);
    Purge();  
    auto it = m_Routes.find(dst);
    if( it != m_Routes.end() && it ->second.IsValid())
    {
        entry = it->second;
        return true;
    }
    return false;  
}

bool RoutingTable::UpdateRoute(Ipv4Address dst, RoutingTableEntry & entry)
{
    NS_LOG_FUNCTION(this << dst);
    auto it = m_Routes.find(dst);
    if (it != m_Routes.end())
    {
        entry.UpdateLifetime(m_entryLifeTime);
        it->second = entry;
        return true;
    }
    return false;
}

bool RoutingTable::GetNearestNeighborToDst(Vector dstPos, Ipv4Address& nextHop)
{
  double minDistance = std::numeric_limits<double>::max();
  bool found = false;
  
  for (auto& entry : m_Routes)
  {
    if (entry.second.GetRouteType() == ROUTE_DIRECT && entry.second.IsValid())
    {
      Vector neighborPos = entry.second.GetDestPosition();
      double distance = CalculateDistance(neighborPos, dstPos);
           
      if (distance < minDistance)
      {
        minDistance = distance;
        nextHop = entry.first;
        found = true;
      }
    }
  }
    
  return found;
}

std::vector<RoutingTableEntry> RoutingTable::GetNeighborsCloserToDst(Vector dstPos, Vector myPos)
{
  std::vector<RoutingTableEntry> candidates;
  double myDistance = CalculateDistance(myPos, dstPos);
  for (auto& entry : m_Routes)
  {
    if (entry.second.GetRouteType() == ROUTE_DIRECT && entry.second.IsValid())
    {
      Vector neighborPos = entry.second.GetDestPosition();
      double neighborDistance = CalculateDistance(neighborPos, dstPos);     
      if (neighborDistance < myDistance)
      {
        candidates.push_back(entry.second);
      }
    }
  }    
  return candidates;
}

void RoutingTable::Purge()
{
  NS_LOG_FUNCTION(this);
  auto it = m_Routes.begin();
  while (it != m_Routes.end())
  {
    if(it->second.IsExpired())
    {
      NS_LOG_DEBUG("Removing expired route to " << it->first);
      it = m_Routes.erase(it);
    }
    else{
      ++it;
    }
  }
}


void RoutingTable::Clear()
{
  NS_LOG_FUNCTION(this);
  m_Routes.clear();
}

void RoutingTable::Print(Ptr<OutputStreamWrapper> stream) const
{
  *stream->GetStream() << "Destination\t\tNextHop\t\tHopCount\tType\tValid\n";
  for (auto& entry : m_Routes)
  {
    *stream->GetStream() << entry.first << "\t"
                         << entry.second.GetNextHopAddress() << "\t"
                         << entry.second.GetHopCounts() << "\t\t"
                         << entry.second.GetRouteType() << "\t"
                         << entry.second.IsValid() << "\n";
  }
}

} // namespace icra
} // namespace ns3

