/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef CLUSTERMANAGER_H
#define CLUSTERMANAGER_H

#include "fanet-node.h"
#include "ns3/object.h"
#include "ns3/nstime.h"
#include "ns3/core-module.h"
#include "ns3/vector.h"
#include <map>

namespace ns3{


struct GatewayInfo {
  Ipv4Address gatewayAddr;         // Gateway node address.
  uint16_t    gatewayNodeId;       // Gateway node ID.
  Ipv4Address linkedClusterHead;   // Head address of the linked cluster.
  uint16_t    linkedClusterId;     // Linked cluster ID.
  uint16_t    linkedClusterHeadId; // Linked cluster-head ID.
  double      linkQuality;         // Link quality.
  double      distance;            // Distance.
  Time        lastUpdate;          // Last update time.
  bool        isActive;            // Whether the entry is valid.

  GatewayInfo()
    : gatewayAddr("0.0.0.0"),
      gatewayNodeId(0xFFFF),
      linkedClusterHead("0.0.0.0"),
      linkedClusterId(0xFFFF),
      linkedClusterHeadId(0xFFFF),
      linkQuality(0.0),
      distance(0.0),
      lastUpdate(Seconds(0)),
      isActive(true) {}
};



class Cluster :public Object
{
public:
  Cluster();
  virtual ~Cluster(){};
  static TypeId GetTypeId();
  void    SetClusterId (uint16_t clusterId) { m_ClusterId = clusterId;}
  void    RecordInitializeTime (Time time)  { m_InitializeTime = time.GetSeconds();}
  double  GetInitializeTime()               { return m_InitializeTime;}
  void    UpdateLatestTime (Time time)      { m_LastestTime = time.GetSeconds();}
  double  GetLifeTime () const              {return (m_LastestTime - m_InitializeTime);}
  void    UpdateMemberTime(uint16_t NodeId, Time now);
  Time    GetMemberUpdateTime(uint16_t NodeId) const;

  void AddClusterMember(Ptr <ClusterNode> node, Time now)
  {
    m_ClusterMembers[node->GetNodeId()] = node;
    m_MembersUpdateTime[node->GetNodeId()] = now;
    node->SetClusterId(m_ClusterId);
  }
  void RemoveMember(Ptr <ClusterNode> node)
  {
    node->SetClusterId(0xFFFF);
    m_ClusterMembers.erase(node->GetNodeId());
    m_MembersUpdateTime.erase(node->GetNodeId());
  }
  // Opt-in bookkeeping only: retiring an old registration must not clear a
  // node's newer association. Keep RemoveMember's legacy behavior unchanged.
  void RemoveMemberRecord(uint16_t nodeId)
  {
    m_ClusterMembers.erase(nodeId);
    m_MembersUpdateTime.erase(nodeId);
  }
  bool ExistMember(uint16_t NodeId) const
  {
    return m_ClusterMembers.find(NodeId) != m_ClusterMembers.end();
  }

  std::map<uint16_t, Ptr<ClusterNode>> &GetAllMembers() { return m_ClusterMembers;}
  std::map<uint16_t, Time> GetAllMembersStamp()         { return m_MembersUpdateTime;}

  void SetClusterHead(Ptr <ClusterNode> chNode)
  {
    AddClusterMember(chNode,Simulator::Now());
    chNode->SetRole(Head);
    m_ClusterHeadId = chNode->GetNodeId();
  }
  void DropClusterHead(Ptr <ClusterNode> chNode)
  {
    chNode->SetRole(Member);
    m_ClusterHeadId = 0xFFFF;
  }

  uint16_t GetClusterId() const {return m_ClusterId;}
  uint16_t GetClusterHeadId() const {return m_ClusterHeadId;}
  uint16_t GetClusterScale() {return m_ClusterMembers.size();}

  /*Gateway managerment*/
  void   AddGateway(const GatewayInfo& gwInfo);
  void   UpdateGateway(Ipv4Address gwAddr, uint16_t linkedClusterId, double linkQuality, double distance);
  void   RemoveGateway(Ipv4Address gwAddr, uint16_t linkedClusterId);
  void   CleanExpiredGateways(Time timeout);
  std::vector<GatewayInfo> GetGatewaysToCluster(uint16_t targetClusterId) const;
  const std::map<Ipv4Address, std::vector<GatewayInfo>>& GetAllGateways() const { return m_gatewayTable; }
  bool   HasGatewayToCluster(uint16_t targetClusterId) const;
private:
  uint16_t                                          m_ClusterId;
  uint16_t                                          m_ClusterHeadId;
  std::map<uint16_t, Ptr<ClusterNode> >             m_ClusterMembers;
  std::map<uint16_t, Time>                          m_MembersUpdateTime; // [s]
  double                                            m_InitializeTime;
  double                                            m_LastestTime;

  std::map<Ipv4Address, std::vector<GatewayInfo>>   m_gatewayTable;

}; // class Cluster:public Object



struct ClusterNodeContainer
{
public:
  void CreateClusterNode (Ptr< ClusterNode> Node)
  {
    nodes.push_back(Node);
  }

private:
std::vector<Ptr <ClusterNode>> nodes;
};


// Cluster statistics.
struct ClusterStatistics {
  uint16_t  clusterId;
  Time      creationTime;    // Cluster creation time.
  Time      dissolutionTime; // Cluster dissolution time.
  double    lifetime;        // Cluster lifetime in seconds.
  uint32_t  maxSize;         // Maximum cluster size.
  uint32_t  dissolutionSize; // Cluster size at dissolution.
  uint16_t  clusterHeadId;   // Cluster-head ID.
  ClusterStatistics()
    : clusterId(0xFFFF),
      creationTime(Seconds(0)),
      dissolutionTime(Seconds(0)),
      lifetime(0.0),
      maxSize(0),
      dissolutionSize(0),
      clusterHeadId(0xFFFF) {}
};



class ClusterManager : public Object
{
public:
  static TypeId GetTypeId();
  ClusterManager();
  virtual ~ClusterManager()   {}
  static Ptr<ClusterManager>          GetInstance();
  /**
   * \brief Assign ns-3 RNG streams to the cluster manager.
   * \param stream First stream number.
   * \return Number of streams consumed.
   */
  int64_t                             AssignStreams(int64_t stream);
  void                                RemoveCluster(uint16_t clusterId);
  void                                CreateCluster(uint16_t clusterId, Ptr<ClusterNode> ChNode);
  Ptr<Cluster>                        GetCluster(uint16_t clusterId);
  std::map<uint16_t, Ptr<Cluster>>&   GetAllClusters();
  /*Cluster Member Management*/
  void            JoinCluster(uint16_t clusterId, Ptr<ClusterNode> node);
  void            LeaveCluster(uint16_t nodeId, uint16_t clusterId);
  uint16_t        GetClusterId(uint16_t nodeId);
  uint16_t        GetClusterHeadId(uint16_t clusterId);
  bool            ExistMember(uint16_t NodeId);
  bool            ExistHead(uint16_t NodeId);
  Ptr<Cluster>    FindNodeCluster(uint16_t nodeId);

  void CheckClusteringStatus(uint16_t NodeId);

  /*Cluster Id Manager*/
  void        ClusterIdIncrease()        { GlobalClusterId++; }
  void        ClusterIdInitial()         { GlobalClusterId = 0; }
  uint16_t    ArrangeClusterId()         { return GlobalClusterId; }
  bool        IsInCluster(uint16_t NoddeId, uint16_t ClusterId);
  /*clustering status information manager*/
  const std::vector<ClusterStatistics>& GetClusterHistory() const { return m_clusterHistory; }
  void GetClusteringStatistics(uint32_t& totalClusters, double& avgLifetime, double& avgSize) const;
  void ResetStatistics();
  void GetActiveClusterStats(uint32_t& count, double& avgSize, double &AvgSizePro) const;
  void SendJoinRequest(uint16_t NodeId, Ipv4Address dst);
  void SendClusterHeadCall(uint16_t NodeId);

private:
  static Ptr<ClusterManager>          m_instance;
  Ptr<UniformRandomVariable>          m_rand;                 ///< Reproducible RNG source (replaces unseeded libc rand).
  std::map<uint16_t, Ptr<Cluster>>    m_clusters;
  uint16_t                            GlobalClusterId;
  /*For Clustering Status information collecting*/
  std::vector<ClusterStatistics>      m_clusterHistory;  // Cluster history.
  std::map<uint16_t, uint32_t>        m_clusterSizeTracker; // Current size per cluster.

  void RecordClusterCreation(uint16_t clusterId, uint16_t chNodeId);
  void RecordClusterDissolution(uint16_t clusterId);
  void UpdateClusterSize(uint16_t clusterId);

};




}

#endif /* FANET_CLUSTERMANAGER_H */
