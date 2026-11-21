/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "mo-helper.h"
#include "ns3/mo-routing.h"
#include "ns3/node.h"
namespace ns3
{
MoHelper::MoHelper() { m_factory.SetTypeId(mo::RoutingProtocol::GetTypeId()); }
MoHelper* MoHelper::Copy() const { return new MoHelper(*this); }
Ptr<Ipv4RoutingProtocol> MoHelper::Create(Ptr<Node> node) const
{
    auto routing = m_factory.Create<mo::RoutingProtocol>(); routing->SetNode(node); node->AggregateObject(routing); return routing;
}
void MoHelper::Set(const std::string& name, const AttributeValue& value) { m_factory.Set(name, value); }
}
