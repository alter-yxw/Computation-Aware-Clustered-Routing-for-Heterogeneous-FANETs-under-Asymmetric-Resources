/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "qrcf-discovery.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ns3::qrcf
{
Discovery::Discovery(Settings settings)
    : m_settings(settings), m_interval(Model::InitialHello(settings.helloLower,settings.helloUpper))
{ m_settings.Validate(); }
HelloHeader Discovery::MakeHello(uint32_t nodeId, uint64_t messageId, Vector position,
                                 const Motion& motion, double energy) const
{
    HelloHeader hello;
    hello.messageId = messageId; hello.nodeId = nodeId; hello.position = position;
    hello.motion = motion; hello.energy = energy; hello.helloInterval = m_interval;
    if (!hello.IsValid()) { throw std::invalid_argument("Invalid QRCF own Hello state"); }
    return hello;
}
bool Discovery::AfterHello(Vector position, const Motion& motion, Time now,
                           const std::vector<Scalar>& connections,
                           const std::set<uint32_t>& currentNeighbors)
{
    if (now < Time(0) || (m_previous && now <= m_previous->time))
    { throw std::invalid_argument("QRCF Hello sampling must advance time"); }
    // Approved policy: retain out-of-range cache records for gamma/history,
    // but omit them from the current CT sample domain.
    Model::Velocity(motion); // validate before storing a sample
    m_gamma = m_previousNeighbors ? Model::Discount(*m_previousNeighbors,currentNeighbors).value_or(0) : 0;
    m_previousNeighbors = currentNeighbors;
    double next = m_interval;
    bool defined = true;
    if (m_previous)
    {
        const auto predicted = Model::Predict(m_previous->position,m_previous->motion,
                                              (now-m_previous->time).GetSeconds());
        m_predictionError = CalculateDistance(position,predicted);
        if (!connections.empty())
        {
            double sum = 0, maximum = 0;
            for (const auto& ct : connections)
            {
                if (!ct || !std::isfinite(*ct) || *ct < 0 || *ct > m_settings.connectionCap)
                { defined = false; break; }
                sum += *ct; maximum = std::max(maximum,*ct);
            }
            if (defined && maximum > 0)
            {
                const auto adapted = Model::AdaptHello(m_interval,m_settings.helloLower,m_settings.helloUpper,
                    *m_predictionError,m_settings.errorMin,m_settings.errorMax,
                    sum/connections.size(),maximum);
                defined = adapted.has_value();
                if (adapted) { next = *adapted; }
            }
        }
    }
    m_previous = Sample{position,motion,now};
    if (!defined) { return false; }
    m_interval = next;
    return true;
}
} // namespace ns3::qrcf
