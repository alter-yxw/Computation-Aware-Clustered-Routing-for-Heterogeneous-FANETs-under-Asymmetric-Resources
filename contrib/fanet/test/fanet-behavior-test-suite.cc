/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "ns3/fanet-clustermanager.h"
#include "ns3/fanet-neighbor.h"
#include "ns3/fanet-node.h"
#include "ns3/simulator.h"
#include "ns3/test.h"
#include "ns3/enum.h"

#include <string>

using namespace ns3;

namespace
{

class SimulationScope
{
  public:
    ~SimulationScope() { Simulator::Destroy(); }
};

Neighbors::NeighborInfo
Neighbor(uint16_t id, Vector position, NodeRole role = Member, float load = 0.0f)
{
    return Neighbors::NeighborInfo(id,
                                  Ipv4Address("10.0.0.1"),
                                  1,
                                  position,
                                  Vector(),
                                  100.0,
                                  1.0,
                                  role,
                                  Simulator::Now(),
                                  Seconds(1000),
                                  1.0,
                                  load);
}

class FanetSemanticsTestCase : public TestCase
{
  public:
    enum Decision
    {
        FITNESS,
        FITNESS_HOMOGENEOUS,
        GREEDY,
        PERIMETER,
        ANGLE,
        PURGE,
        CLUSTER_STATUS,
        QUEUE_SNAPSHOT
    };

    FanetSemanticsTestCase(const std::string& name, Decision decision)
        : TestCase(name),
          m_decision(decision)
    {
    }

  private:
    void DoRun() override
    {
        SimulationScope scope;
        switch (m_decision)
        {
        case FITNESS:
            CheckFitness(ClusterNode::HETEROGENEOUS_COMPUTE);
            break;
        case FITNESS_HOMOGENEOUS:
            CheckFitness(ClusterNode::HOMOGENEOUS);
            break;
        case GREEDY:
            CheckGreedy();
            break;
        case PERIMETER:
            CheckPerimeter();
            break;
        case ANGLE:
            CheckAngle();
            break;
        case PURGE:
            CheckPurge();
            break;
        case CLUSTER_STATUS:
            CheckClusterStatus();
            break;
        case QUEUE_SNAPSHOT:
            CheckQueueSnapshot();
            break;
        }
    }

    void CheckFitness(ClusterNode::FitnessMode mode)
    {
        auto node = CreateObjectWithAttributes<ClusterNode>("FitnessMode", EnumValue(mode));
        EnumValue attribute;
        node->GetAttribute("FitnessMode", attribute);
        NS_TEST_ASSERT_MSG_EQ(attribute.Get(), mode, "FitnessMode attribute selects evaluation semantics");
        node->SetNodeId(0);
        node->SetNetworkScale(100);
        node->SetPosition(Vector());
        node->SetVelocity(Vector());
        node->GetNeighborTable()->AddNeighbor(Neighbor(1, Vector(100, 0, 0)));
        node->GetNeighborTable()->AddNeighbor(Neighbor(2, Vector(400, 0, 0)));
        node->UpdateNeighborDegree();

        const bool homogeneous = mode == ClusterNode::HOMOGENEOUS;
        const double degree = homogeneous ? 2.0 / 99.0 : 1.0 / 10.0;
        NS_TEST_ASSERT_MSG_EQ_TOL(node->ComputeFitness(0, 0, 1, 0), degree, 1e-12,
                                  "Homogeneous includes the 400 m neighbor; heterogeneous uses local density");
        const double distance = (1.0 / 101.0 + 1.0 / 401.0) / 2.0;
        const double fallback = (homogeneous ? 0.25 : 0.2) * (1.0 + 1.0 + degree + distance);
        NS_TEST_ASSERT_MSG_EQ_TOL(node->ComputeFitness(0, 0, 0, 0), fallback, 1e-12,
                                  "Zero weights use the selected four/five-factor fallback");
        NS_TEST_ASSERT_MSG_EQ_TOL(node->ComputeFitness(0, 0, 0, 0, 1),
                                  (homogeneous ? fallback : 1.0), 1e-12,
                                  "Homogeneous ignores compute weights even when all four weights are zero");
        NS_TEST_ASSERT_MSG_EQ_TOL(node->ComputeFitness(0, 0, 2, 0, 2),
                                  (homogeneous ? degree : 0.5 * (degree + 1.0)), 1e-12,
                                  "Only active factors participate in weight normalization");
        node->SetComputeCapacity(0.5);
        node->EnqueueTask();
        NS_TEST_ASSERT_MSG_EQ_TOL(node->ComputeFitness(0, 0, 1, 0), degree, 1e-12,
                                  "Degree is independent of local computation state");
        node->SetNetworkScale(1);
        NS_TEST_ASSERT_MSG_EQ_TOL(node->ComputeFitness(0, 0, 1, 0), (homogeneous ? 0.0 : degree), 1e-12,
                                  "Homogeneous guards the singleton normalization denominator");
        node->SetNetworkScale(100);
        node->updateResidualEnergy(0);
        NS_TEST_ASSERT_MSG_EQ(node->ComputeFitness(0, 0, 1, 0), 0.0,
                              "Depleted nodes must have zero fitness");
    }

    void CheckGreedy()
    {
        auto neighbors = CreateObject<Neighbors>();
        neighbors->AddNeighbor(Neighbor(1, Vector(1000, 0, 0), Member, 1.0f));
        neighbors->AddNeighbor(Neighbor(2, Vector(800, 0, 0)));
        NS_TEST_ASSERT_MSG_EQ(neighbors->GreedyNeighbor(Vector(2000, 0, 0), Vector()),
                              2,
                              "Greedy forwarding accounts for link quality and load");
    }

    void CheckPerimeter()
    {
        auto neighbors = CreateObject<Neighbors>();
        neighbors->AddNeighbor(Neighbor(1, Vector(0, 100, 0), Head));
        neighbors->AddNeighbor(Neighbor(2, Vector(100, 0, 0)));
        Vector previous;
        NS_TEST_ASSERT_MSG_EQ(neighbors->PerimeterNextHop(Vector(1000, 0, 0), Vector(), previous),
                              2,
                              "Perimeter selection uses all neighbors");
        neighbors->Clear();
        neighbors->AddNeighbor(Neighbor(2, Vector(100, 0, 0)));
        NS_TEST_ASSERT_MSG_EQ(neighbors->PerimeterNextHop(Vector(1000, 0, 0), Vector(), previous),
                              2, "Perimeter selection includes members");
    }

    void CheckAngle()
    {
        auto neighbors = CreateObject<Neighbors>();
        neighbors->AddNeighbor(Neighbor(1, Vector(100, 100, -1000), Head));
        neighbors->AddNeighbor(Neighbor(2, Vector(0, 100, 1000), Head));
        Vector previous;
        NS_TEST_ASSERT_MSG_EQ(neighbors->PerimeterNextHop(Vector(100, 0, 100), Vector(), previous),
                              1,
                              "Perimeter angles use the XY projection");
    }

    void CheckPurge()
    {
        auto neighbors = CreateObject<Neighbors>();
        auto entry = Neighbor(1, Vector());
        // Expire before the first periodic cleanup; inspect without an explicit Purge.
        entry.NeighborLifeLength = Seconds(1);
        neighbors->AddNeighbor(entry);
        Simulator::Schedule(Seconds(3), [this, neighbors]() {
            NS_TEST_ASSERT_MSG_EQ(neighbors->IsEmpty(), true,
                                  "Periodic cleanup removes expired entries");
        });
        Simulator::Schedule(Seconds(11), [this, neighbors]() {
            NS_TEST_ASSERT_MSG_EQ(neighbors->IsEmpty(), true,
                                  "Expired neighbors stay removed");
        });
        Simulator::Run();
    }

    void CheckClusterStatus()
    {
        auto node = CreateObject<Node>();
        auto clusterNode = CreateObject<ClusterNode>();
        clusterNode->SetNodeId(node->GetId());
        clusterNode->SetClusterId(42);
        node->AggregateObject(clusterNode);
        auto manager = CreateObject<ClusterManager>();
        Simulator::Schedule(Seconds(1), [this, manager, clusterNode]() {
            clusterNode->GetNeighborTable()->AddNeighbor(Neighbor(1, Vector(), Head));
            manager->CheckClusteringStatus(clusterNode->GetNodeId());
            NS_TEST_ASSERT_MSG_EQ(clusterNode->GetRoleType(), Idle,
                                  "Unclustered nodes remain eligible to rejoin");
            NS_TEST_ASSERT_MSG_EQ(clusterNode->GetClusterId(), 0xFFFF,
                                  "Unclustered nodes clear their old cluster identifier");
            Simulator::Stop();
        });
        Simulator::Run();
    }

    void CheckQueueSnapshot()
    {
        auto node = CreateObject<ClusterNode>();
        node->SetComputeCapacity(20);
        // Exercise the pre-existing incremental API used by SBCR, including its
        // capacity guard, before and after opt-in snapshot use.
        node->SetTaskQueueCapacity(2);
        node->EnqueueTask(); node->EnqueueTask(); node->EnqueueTask();
        NS_TEST_ASSERT_MSG_EQ(node->GetCurrentTaskQueueLength(), 2, "legacy queue saturates");
        NS_TEST_ASSERT_MSG_EQ(node->GetCurrentLoad(), 1.0, "legacy load is retained");
        node->DequeueTask(1.0);
        NS_TEST_ASSERT_MSG_EQ(node->GetCurrentLoad(), 0.5, "legacy decrement is retained");
        NS_TEST_ASSERT_MSG_EQ(node->SetTaskQueueSnapshot(3, 4), true, "valid authoritative snapshot");
        NS_TEST_ASSERT_MSG_EQ(node->GetCurrentTaskQueueLength(), 3, "exact waiting count");
        NS_TEST_ASSERT_MSG_EQ(node->GetCurrentLoad(), 0.75, "exact queue fraction");
        NS_TEST_ASSERT_MSG_EQ(node->GetAvailableCompute(), 5.0, "existing score sees the snapshot");
        NS_TEST_ASSERT_MSG_EQ(node->SetTaskQueueSnapshot(0, 0), false, "undefined ratio rejected");
        NS_TEST_ASSERT_MSG_EQ(node->SetTaskQueueSnapshot(5, 4), false, "overfull snapshot rejected");
        NS_TEST_ASSERT_MSG_EQ(node->GetCurrentTaskQueueLength(), 3, "invalid requests do not mutate count");
        NS_TEST_ASSERT_MSG_EQ(node->GetTaskQueueCapacity(), 4, "invalid requests do not mutate capacity");
        NS_TEST_ASSERT_MSG_EQ(node->GetCurrentLoad(), 0.75, "invalid requests do not mutate load");
        node->DequeueTask(0);
        NS_TEST_ASSERT_MSG_EQ(node->GetCurrentLoad(), 0.5, "legacy API remains consistent after snapshot");
        NS_TEST_ASSERT_MSG_EQ(node->SetTaskQueueSnapshot(0, 2), true, "reset to empty actual queue");
        NS_TEST_ASSERT_MSG_EQ(node->GetCurrentLoad(), 0.0, "idle queue load");
    }

    Decision m_decision;
};

class FanetSemanticsTestSuite : public TestSuite
{
  public:
    FanetSemanticsTestSuite()
        : TestSuite("fanet-behavior", UNIT)
    {
        AddTestCase(new FanetSemanticsTestCase("heterogeneous compute fitness", FanetSemanticsTestCase::FITNESS), TestCase::QUICK);
        AddTestCase(new FanetSemanticsTestCase("homogeneous fitness", FanetSemanticsTestCase::FITNESS_HOMOGENEOUS), TestCase::QUICK);
        AddTestCase(new FanetSemanticsTestCase("shared greedy forwarding", FanetSemanticsTestCase::GREEDY), TestCase::QUICK);
        AddTestCase(new FanetSemanticsTestCase("shared perimeter candidates", FanetSemanticsTestCase::PERIMETER), TestCase::QUICK);
        AddTestCase(new FanetSemanticsTestCase("shared angle calculation", FanetSemanticsTestCase::ANGLE), TestCase::QUICK);
        AddTestCase(new FanetSemanticsTestCase("shared periodic cleanup", FanetSemanticsTestCase::PURGE), TestCase::QUICK);
        AddTestCase(new FanetSemanticsTestCase("shared unclustered roles", FanetSemanticsTestCase::CLUSTER_STATUS), TestCase::QUICK);
        AddTestCase(new FanetSemanticsTestCase("explicit queue snapshots and legacy counters", FanetSemanticsTestCase::QUEUE_SNAPSHOT), TestCase::QUICK);
    }
};

FanetSemanticsTestSuite g_fanetSemanticsTestSuite;

} // namespace
