/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "cfr-eng-helper.h"
#include "ns3/cfr-eng-routing.h"
#include "ns3/node.h"
namespace ns3
{
CfrEngHelper::CfrEngHelper() { m_factory.SetTypeId(cfreng::RoutingProtocol::GetTypeId()); }
CfrEngHelper* CfrEngHelper::Copy() const { return new CfrEngHelper(*this); }
Ptr<Ipv4RoutingProtocol> CfrEngHelper::Create(Ptr<Node> node) const
{
    auto routing = m_factory.Create<cfreng::RoutingProtocol>();
    routing->SetNode(node); node->AggregateObject(routing); return routing;
}
void CfrEngHelper::Set(const std::string& name, const AttributeValue& value) { m_factory.Set(name, value); }
}
