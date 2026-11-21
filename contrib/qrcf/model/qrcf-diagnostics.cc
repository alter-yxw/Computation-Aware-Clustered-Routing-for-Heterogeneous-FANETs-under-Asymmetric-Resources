/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "qrcf-diagnostics.h"
#include <stdexcept>

namespace ns3::qrcf
{
void Diagnostics::ObserveCandidates(std::size_t neighbors, std::size_t filtered)
{
    if (filtered > neighbors) { throw std::invalid_argument("QRCF FS is not a subset of NTable"); }
    ++m_candidateSamples; m_neighborSum += neighbors; m_filteredSum += filtered;
}
void Diagnostics::ObserveHelloInterval(Time interval)
{
    if (interval <= Time(0)) { throw std::invalid_argument("Invalid QRCF observed HT"); }
    ++m_intervalSamples; m_intervalSum += interval.GetSeconds();
}
void Diagnostics::ObserveNeighbors(const std::set<uint32_t>& neighbors, Time now)
{
    for (auto it = m_linkStarts.begin(); it != m_linkStarts.end();)
    {
        if (!neighbors.count(it->first))
        {
            if (now < it->second) { throw std::invalid_argument("QRCF link time moved backwards"); }
            ++m_completedLinks;
            m_linkSeconds += (now-it->second).GetSeconds();
            it = m_linkStarts.erase(it);
        }
        else { ++it; }
    }
    for (uint32_t peer : neighbors) { m_linkStarts.emplace(peer,now); }
}
double Diagnostics::AverageNeighborCount() const
{ return m_candidateSamples ? double(m_neighborSum)/m_candidateSamples : 0; }
double Diagnostics::AverageFilteredCount() const
{ return m_candidateSamples ? double(m_filteredSum)/m_candidateSamples : 0; }
double Diagnostics::AverageHelloSeconds() const
{ return m_intervalSamples ? m_intervalSum/m_intervalSamples : 0; }
double Diagnostics::AverageCompletedLinkSeconds() const
{ return m_completedLinks ? m_linkSeconds/m_completedLinks : 0; }
void Diagnostics::WriteColumns(std::ostream& os)
{
    os << "HelloTx,HelloRx,DuplicateHello,InvalidControl,ControlTx,ControlL3Packets,ControlL3Bytes,FeedbackTx,FeedbackRx,"
          "QUpdates,Explorations,Exploitations,RouteRequests,NoRoute,LocalMinima,LinkFailures,"
          "UndefinedFormulas,OutOfRangeCtExclusions,CandidateSamples,MeanNeighbors,MeanFiltered,HelloIntervalSamples,"
          "MeanHelloSeconds,CompletedDirectedLinks,OpenDirectedLinks,MeanCompletedLinkSeconds";
}
void Diagnostics::WriteValues(std::ostream& os) const
{
    os << helloTx << ',' << helloRx << ',' << duplicateHello << ',' << invalidControl << ','
       << controlTx << ',' << controlL3Packets << ',' << controlBytes << ',' << feedbackTx << ',' << feedbackRx << ','
       << qUpdates << ',' << explorations << ',' << exploitations << ',' << routeRequests << ','
       << noRoute << ',' << localMinima << ',' << linkFailures << ',' << undefinedFormulas << ','
       << outOfRangeCtExclusions << ',' << m_candidateSamples << ',' << AverageNeighborCount() << ',' << AverageFilteredCount() << ','
       << m_intervalSamples << ',' << AverageHelloSeconds() << ',' << m_completedLinks << ','
       << OpenLinks() << ',' << AverageCompletedLinkSeconds();
}
} // namespace ns3::qrcf
