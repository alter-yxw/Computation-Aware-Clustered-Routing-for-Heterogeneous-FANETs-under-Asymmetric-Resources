/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */

#ifndef FANET_NODE_H
#define FANET_NODE_H

#include "ns3/object.h"
#include "ns3/node.h"
#include "ns3/vector.h"
#include "ns3/traced-callback.h"
// #include "fanet.h"
#include "fanet-neighbor.h"
#include <map>
#include <string>

namespace ns3{

// class Neighbors;

class ClusterNode : public Object
{
public:
  // Only fitness differs between modes; neighbor/routing improvements are shared.
  enum FitnessMode
  {
    HOMOGENEOUS,
    HETEROGENEOUS_COMPUTE
  };
  void SetFitnessMode(FitnessMode mode) { m_fitnessMode = mode; }
  FitnessMode GetFitnessMode() const { return m_fitnessMode; }
  static const char* FitnessModeName(FitnessMode mode);
  static bool ParseFitnessMode(const std::string& name, FitnessMode& mode);

  static const uint16_t INVALID_CLUSTER_ID = 0xFFFF;
  static const uint16_t INVALID_NODE_ID = 0xFFFF;
  static TypeId GetTypeId();

  ClusterNode ();
  virtual ~ClusterNode();

  /*  Set Node Id from ns3::Node */
  void      SetNodeId(uint16_t NodeId);
  uint16_t  GetNodeId() const;
  void      BindNode(Ptr <Node> node) {m_node = node;}
  Ptr<Node> GetNode() const { return m_node;}
  /*Set Network Scale*/
  void      SetNetworkScale(uint16_t scale) {m_NodesQuantities = scale;}
  uint16_t  GetNetworkScale() const { return m_NodesQuantities;}
  /*Belongs to which cluster*/
  void      SetClusterId(uint16_t clusterid) {
    const auto previous = m_ClusterId;
    m_ClusterId = clusterid;
    if (previous != clusterid) m_clusterState(m_role, previous, m_role, clusterid);
  }
  uint16_t  GetClusterId() const { return m_ClusterId;}
  uint16_t  GetClusterHeadId();
  /*  Set and Get Node Energy*/
  void      SetOriginEnergy(double Energy);
  double    GetOriginEnergy() const;
  void      updateResidualEnergy( double Energy );
  double    GetResidualEnergy () const;
  /*  Set and Get Node Velocity*/
  void      SetVelocity (Vector vel);
  Vector    GetVelocity () const;
  /* Position*/
  void      SetPosition (Vector pos);
  Vector    GetPosition () const;

  uint16_t  GetNeighborDegree() const          { return m_NeighborDegree;}
  void      UpdateNeighborDegree()             { m_NeighborDegree = m_Neighbors->GetAll().size();}
  void      SetNeighborDegree(uint16_t degree) { m_NeighborDegree = degree;}
  Ptr<Neighbors> GetNeighborTable()            { return m_Neighbors;}

  /*Set the type of the node (CH，CM)*/
  void      SetRole(NodeRole role) {
    const auto previous = m_role;
    m_role = role;
    if (previous != role) m_clusterState(previous, m_ClusterId, role, m_ClusterId);
  }
  NodeRole  GetRoleType()           { return m_role;}
  bool      IsClusterHead()   const { return m_role==Head;  }
  bool      IsClusterMember() const { return m_role==Member;}

  // Homogeneous uses four factors and ignores WeightCompute, including normalization.
  double    ComputeFitness(float WeightLHTP,float WeightEnergyLevel,float WeightDegree,float WeightDistance, float WeightCompute = 0.0f);
  void      UpdateFitness(double NewValue)  { m_FitnessValue = NewValue;}
  double    GetFitnessValue() const {return m_FitnessValue;}
  void      UpdateLastContactTime(double time) {m_LastUpdateTime= time;}
  double    GetLastUpdateTime() const { return m_LastUpdateTime;}
  void      PrintNodeInfo();

  // ---- Compute state ----
  void      SetComputeCapacity(double gflops);
  double    GetComputeCapacity() const;
  void      SetTaskQueueCapacity(uint32_t cap);
  uint32_t  GetTaskQueueCapacity() const;
  // Explicit snapshot for an application-owned waiting queue. Invalid snapshots
  // return false without changing state. Legacy incremental APIs stay unchanged.
  bool      SetTaskQueueSnapshot(uint32_t waiting, uint32_t capacity);
  uint32_t  GetCurrentTaskQueueLength() const { return m_currentQueueLen; }
  void      EnqueueTask();
  void      DequeueTask(double processingTime);
  double    GetCurrentLoad() const;
  double    GetAvailableCompute() const;
  double    ComputeServiceScore() const;

  // Actual reachable communication range of this node in meters, limited by
  // its TX power and RX sensitivity. It may differ from the peer's range,
  // which models asymmetric links in heterogeneous FANETs.
  void      SetMaxRange(double meters)  { m_maxRange = meters; }
  double    GetMaxRange() const         { return m_maxRange; }

  // When true, the LHTP distance threshold in ComputeFitness uses
  // m_maxRange instead of the hard-coded 250 m. Defaults to false to
  // preserve the legacy fitness behaviour.
  void      SetLhtpUseMaxRange(bool use) { m_lhtpUseMaxRange = use; }
  bool      GetLhtpUseMaxRange() const   { return m_lhtpUseMaxRange; }

private:
  TracedCallback<uint8_t, uint16_t, uint8_t, uint16_t> m_clusterState;
  uint16_t       m_NodeId;           // Node ID.
  uint16_t       m_ClusterId;
  uint16_t       m_NodesQuantities;
  uint16_t       m_NeighborDegree;
  double         m_OriginEnergy;
  double         m_ResidualEnergy;
  double         m_FitnessValue;
  NodeRole       m_role;

  Vector          m_velocity;
  Vector          m_position;
  Ptr<Neighbors>  m_Neighbors;      // Neighbor information.
  double          m_LastUpdateTime;
  Ptr<Node>       m_node;

  // ---- Compute state ----
  double         m_computeCapacity;      // Compute capacity in GFLOPS.
  uint32_t       m_taskQueueCapacity;    // Maximum task-queue capacity.
  uint32_t       m_currentQueueLen;      // Current number of queued tasks.
  double         m_currentLoad;          // Current load in [0.0, 1.0].
  double         m_avgProcessingDelay;   // Moving-average service delay in seconds.

  double         m_maxRange;             // Reachable communication range in meters.
  bool           m_lhtpUseMaxRange;      // LHTP uses m_maxRange instead of the hard-coded 250 m.
  FitnessMode    m_fitnessMode;
};   /*class ClusterNode : public Object*/

} //  namespace ns3
#endif /* FANET_NODE_H */
