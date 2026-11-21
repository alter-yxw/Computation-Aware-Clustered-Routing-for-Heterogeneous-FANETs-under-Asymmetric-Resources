/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef MO_HELPER_H
#define MO_HELPER_H
#include "ns3/ipv4-routing-helper.h"
#include "ns3/object-factory.h"
namespace ns3
{
class MoHelper : public Ipv4RoutingHelper
{
  public:
    MoHelper();
    MoHelper* Copy() const override;
    Ptr<Ipv4RoutingProtocol> Create(Ptr<Node> node) const override;
    void Set(const std::string& name, const AttributeValue& value);
  private:
    ObjectFactory m_factory;
};
}
#endif
