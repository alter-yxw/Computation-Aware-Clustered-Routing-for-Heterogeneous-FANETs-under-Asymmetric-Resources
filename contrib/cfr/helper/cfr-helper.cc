/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "cfr-helper.h"
#include "ns3/cfr-routing.h"
#include "ns3/node.h"
namespace ns3
{
CfrHelper::CfrHelper() { m_factory.SetTypeId(cfr::RoutingProtocol::GetTypeId()); }
CfrHelper* CfrHelper::Copy() const { return new CfrHelper(*this); }
Ptr<Ipv4RoutingProtocol> CfrHelper::Create(Ptr<Node> node) const
{
    auto routing = m_factory.Create<cfr::RoutingProtocol>();
    routing->SetNode(node); node->AggregateObject(routing); return routing;
}
void CfrHelper::Set(const std::string& name, const AttributeValue& value) { m_factory.Set(name, value); }
}
