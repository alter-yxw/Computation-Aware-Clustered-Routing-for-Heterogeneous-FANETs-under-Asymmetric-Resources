/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef ICRA_HELPER_H
#define ICRA_HELPER_H

#include "ns3/object-factory.h"
#include "ns3/node.h"
#include "ns3/node-container.h"
#include "ns3/ipv4-routing-helper.h"
#include "ns3/nstime.h"
#include <string>
namespace ns3
{
namespace icra { class QLearningWeightOptimizer; }

class icraHelper : public Ipv4RoutingHelper{

public:
    icraHelper();
    virtual ~icraHelper();

    icraHelper * Copy(void) const;

    virtual Ptr<Ipv4RoutingProtocol> Create (Ptr<Node> node) const;

    void Set(std::string name, const AttributeValue &value);

    int64_t AssignStream(NodeContainer c, int64_t stream);

    // Paper mechanism defaults; legacy preserves the previous fixed-weight variant.
    void SetPreset(const std::string& preset);
    const std::string& GetPreset() const { return m_preset; }
    void SetMaxRange(double meters);
    void SetGroundStationDelay(Time delay);
    void SetExcludedNodeId(uint16_t nodeId);

    // Q-learning configuration.
    void SetQLearningParameters(double alpha, double lambda, double epsilon);
    void SetQLearningModelFile(const std::string& filename);
private:
    ObjectFactory   m_agentFactory;
    std::string     m_modelFile;
    std::string     m_preset = "paper";
    double          m_alpha = 0.8;
    double          m_lambda = 0.5;
    double          m_epsilon = 0.1;
    double          m_range = 500.0;
    Time            m_episodeInterval = Seconds(2.0);
    double          m_lhtpThreshold = 0.1;
    uint32_t        m_roleChangeThreshold = 2;
    Time            m_groundStationDelay = MilliSeconds(1);
    uint16_t        m_excludedNodeId = 0xffff;
    mutable Ptr<icra::QLearningWeightOptimizer> m_configuredOptimizer;

};

} // namespace ns3

#endif /* ICRA_HELPER_H */
