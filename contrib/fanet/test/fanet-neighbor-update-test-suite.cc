/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "ns3/fanet-neighbor.h"
#include "ns3/simulator.h"
#include "ns3/test.h"

#include <string>

using namespace ns3;

namespace
{
struct SimulationScope
{
    ~SimulationScope() { Simulator::Destroy(); }
};

Neighbors::NeighborInfo Entry(uint16_t id, Time lifetime)
{
    return Neighbors::NeighborInfo(id, Ipv4Address(0x0a000000 + id), 42,
        Vector(10, 20, 30), Vector(1, 2, 3), 90, .6, Member,
        Simulator::Now(), lifetime, 20, .25f);
}

class NeighborUpdateCase : public TestCase
{
  public:
    explicit NeighborUpdateCase(const std::string& name) : TestCase(name), m_case(name) {}

  private:
    void DoRun() override
    {
        SimulationScope scope;
        auto neighbors = CreateObject<Neighbors>();
        if (m_case == "timestamp-contract") { CheckTimestamp(neighbors); return; }
        if (m_case == "live-compute-clock") { CheckComputeClock(neighbors); return; }
        if (m_case == "periodic-cleanup") { CheckTimer(neighbors); return; }

        const bool survivor = m_case == "exact-with-survivor";
        const bool tracked = m_case == "exact-tracked";
        const bool other = m_case == "other-expires";
        const bool newId = m_case == "new-id";
        if (tracked) neighbors->EnableComputeFreshness();
        if (!newId) {
            neighbors->AddNeighbor(Entry(7, other ? Seconds(3) : Seconds(1)));
            if (tracked) neighbors->RecordComputeAdvertisement(7, Seconds(10));
        }
        if (survivor || other || newId)
            neighbors->AddNeighbor(Entry(8, survivor ? Seconds(10) : Seconds(1)));

        const Time renewal = m_case == "before-expiry" ? Seconds(1) - TimeStep(1)
                           : m_case == "after-expiry" ? Seconds(1) + TimeStep(1) : Seconds(1);
        bool completed = false;
        Simulator::Schedule(renewal, [&, this] {
            // A prior GetAll/IsNeighbor would remove the expired entry and mask this defect.
            NS_TEST_ASSERT_MSG_EQ(neighbors->PeekAll().count(7), (newId ? 0u : 1u),
                                  "Inspect stored state without cleaning it first");
            auto incoming = Entry(7, Seconds(3));
            incoming.ipAddress = Ipv4Address("10.9.8.7");
            incoming.clusterId = 51; incoming.Role = Head;
            incoming.Position = Vector(40, 50, 60); incoming.Velocity = Vector(4, 5, 6);
            incoming.ResEnergy = 70; incoming.FitnessValue = .8;
            incoming.computeCapacity = 80; incoming.currentLoad = .75f;
            neighbors->AddNeighbor(incoming);
            const auto& table = neighbors->PeekAll();
            NS_TEST_ASSERT_MSG_EQ(table.count(7), 1u, "Renewed neighbor must remain in the table");
            NS_TEST_ASSERT_MSG_EQ(table.size(), (survivor ? 2u : 1u), "Only expired entries are removed");
            const auto& current = table.at(7);
            NS_TEST_ASSERT_MSG_EQ(current.NodeId, 7, "Node ID is retained");
            NS_TEST_ASSERT_MSG_EQ(current.ipAddress, incoming.ipAddress, "Address is updated");
            NS_TEST_ASSERT_MSG_EQ(current.clusterId, 51, "Cluster ID is updated");
            NS_TEST_ASSERT_MSG_EQ(current.Role, Head, "Role is updated");
            NS_TEST_ASSERT_MSG_EQ(current.Position, incoming.Position, "Position is updated");
            NS_TEST_ASSERT_MSG_EQ(current.Velocity, incoming.Velocity, "Velocity is updated");
            NS_TEST_ASSERT_MSG_EQ(current.ResEnergy, 70, "Energy is updated");
            NS_TEST_ASSERT_MSG_EQ(current.FitnessValue, .8, "Fitness is updated");
            NS_TEST_ASSERT_MSG_EQ(current.UpdateTime, renewal, "Reception time is retained");
            NS_TEST_ASSERT_MSG_EQ(current.NeighborLifeLength, Seconds(3), "Caller still supplies TTL");
            NS_TEST_ASSERT_MSG_EQ(current.computeCapacity, 80, "Compute capacity is updated");
            NS_TEST_ASSERT_MSG_EQ(current.currentLoad, .75f, "Compute load is updated");
            if (survivor) {
                NS_TEST_ASSERT_MSG_EQ(table.at(8).UpdateTime, Seconds(0), "Other live entry is untouched");
                NS_TEST_ASSERT_MSG_EQ(table.at(8).computeCapacity, 20, "Other payload is untouched");
            }
            if (tracked) {
                NS_TEST_ASSERT_MSG_EQ(neighbors->GetComputeObservation(7).status,
                    Neighbors::ComputeObservation::UNKNOWN, "Expired incarnation loses compute history");
                neighbors->RecordComputeAdvertisement(7, Seconds(2));
                const auto observation = neighbors->GetComputeObservation(7);
                NS_TEST_ASSERT_MSG_EQ(observation.status, Neighbors::ComputeObservation::FRESH,
                                      "New advertisement establishes fresh state");
                NS_TEST_ASSERT_MSG_EQ(observation.receivedAt.value(), renewal, "New receive clock");
            } else {
                NS_TEST_ASSERT_MSG_EQ(neighbors->GetComputeObservation(7).status,
                    Neighbors::ComputeObservation::UNTRACKED, "Legacy callers remain untracked");
            }
            completed = true;
        });
        Simulator::Stop(Seconds(1.1));
        Simulator::Run();
        NS_TEST_ASSERT_MSG_EQ(completed, true, "Update callback completed");
    }

    void CheckTimestamp(Ptr<Neighbors> neighbors)
    {
        Simulator::Schedule(Seconds(1), [this, neighbors] {
            auto zero = Entry(7, Seconds(3)); zero.UpdateTime = Seconds(0);
            auto nonzero = Entry(8, Seconds(3)); nonzero.UpdateTime = MilliSeconds(200);
            neighbors->AddNeighbor(zero); neighbors->AddNeighbor(nonzero);
            NS_TEST_ASSERT_MSG_EQ(neighbors->PeekAll().at(7).UpdateTime, Seconds(0),
                                  "Existing zero timestamp sentinel is preserved");
            NS_TEST_ASSERT_MSG_EQ(neighbors->PeekAll().at(8).UpdateTime, Seconds(1),
                                  "Nonzero input timestamp is still replaced by Now");
        });
        Simulator::Stop(Seconds(1.1)); Simulator::Run();
    }

    void CheckComputeClock(Ptr<Neighbors> neighbors)
    {
        neighbors->EnableComputeFreshness();
        neighbors->AddNeighbor(Entry(7, Seconds(3)));
        neighbors->RecordComputeAdvertisement(7, Seconds(1));
        Simulator::Schedule(MilliSeconds(500), [this, neighbors] {
            neighbors->AddNeighbor(Entry(7, Seconds(5)));
            const auto observation = neighbors->GetComputeObservation(7);
            NS_TEST_ASSERT_MSG_EQ(observation.receivedAt.value(), Seconds(0), "Live update keeps compute age");
            NS_TEST_ASSERT_MSG_EQ(observation.expiresAt.value(), Seconds(1), "Live update cannot renew compute TTL");
        });
        Simulator::Schedule(Seconds(1), [this, neighbors] {
            NS_TEST_ASSERT_MSG_EQ(neighbors->GetComputeObservation(7).status,
                Neighbors::ComputeObservation::EXPIRED, "Compute state expires despite renewed liveness");
            NS_TEST_ASSERT_MSG_EQ(neighbors->PeekAll().count(7), 1u, "Neighbor itself is still live");
        });
        Simulator::Stop(Seconds(1.1)); Simulator::Run();
    }

    void CheckTimer(Ptr<Neighbors> neighbors)
    {
        neighbors->AddNeighbor(Entry(7, Seconds(1)));
        Simulator::Schedule(Seconds(1), [neighbors] { neighbors->AddNeighbor(Entry(7, Seconds(3))); });
        Simulator::Schedule(Seconds(2) + TimeStep(1), [this, neighbors] {
            NS_TEST_ASSERT_MSG_EQ(neighbors->PeekAll().count(7), 1u, "Renewed TTL survives existing timer");
        });
        Simulator::Schedule(Seconds(4) - TimeStep(1), [this, neighbors] {
            NS_TEST_ASSERT_MSG_EQ(neighbors->PeekAll().count(7), 1u, "No early expiry");
        });
        Simulator::Schedule(Seconds(4) + TimeStep(1), [this, neighbors] {
            NS_TEST_ASSERT_MSG_EQ(neighbors->PeekAll().size(), 0u, "Two-second timer still purges at expiry");
        });
        Simulator::Schedule(Seconds(4.5), [neighbors] { neighbors->AddNeighbor(Entry(8, Seconds(1))); });
        Simulator::Schedule(Seconds(6.5) + TimeStep(1), [this, neighbors] {
            NS_TEST_ASSERT_MSG_EQ(neighbors->PeekAll().size(), 0u, "Insertion after empty table restarts cleanup");
        });
        Simulator::Stop(Seconds(6.6)); Simulator::Run();
    }

    std::string m_case;
};

class NeighborUpdateSuite : public TestSuite
{
  public:
    NeighborUpdateSuite() : TestSuite("fanet-neighbor-update", UNIT)
    {
        for (const char* name : {"before-expiry", "exact-expiry", "after-expiry",
             "exact-with-survivor", "exact-tracked", "other-expires", "new-id",
             "timestamp-contract", "live-compute-clock", "periodic-cleanup"})
            AddTestCase(new NeighborUpdateCase(name), TestCase::QUICK);
    }
} g_neighborUpdateSuite;
} // namespace
