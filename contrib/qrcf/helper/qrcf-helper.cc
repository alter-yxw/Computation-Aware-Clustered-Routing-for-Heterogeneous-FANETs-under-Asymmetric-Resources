/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "qrcf-helper.h"
#include "ns3/qrcf-routing.h"

namespace ns3
{
QrcfHelper* QrcfHelper::Copy() const { return new QrcfHelper(*this); }
Ptr<Ipv4RoutingProtocol> QrcfHelper::Create(Ptr<Node> node) const
{
    auto routing = CreateObject<qrcf::RoutingProtocol>();
    routing->Configure(node,m_settings);
    node->AggregateObject(routing);
    return routing;
}
int64_t QrcfHelper::AssignStreams(NodeContainer nodes,int64_t stream) const
{
    int64_t used = 0;
    for (auto it = nodes.Begin(); it != nodes.End(); ++it)
    {
        auto routing = (*it)->GetObject<qrcf::RoutingProtocol>();
        NS_ABORT_MSG_IF(!routing,"QrcfHelper must be installed before AssignStreams");
        used += routing->AssignStreams(stream+used);
    }
    return used;
}
}
