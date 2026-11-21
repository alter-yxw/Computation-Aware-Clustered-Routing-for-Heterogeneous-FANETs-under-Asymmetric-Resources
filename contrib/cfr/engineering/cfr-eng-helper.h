/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef CFR_ENG_HELPER_H
#define CFR_ENG_HELPER_H
#include "ns3/ipv4-routing-helper.h"
#include "ns3/object-factory.h"
namespace ns3
{
// Scenario configures node identity, CPU, destination location service and model
// after address assignment, without consulting any global topology in decisions.
class CfrEngHelper : public Ipv4RoutingHelper
{
  public:
    CfrEngHelper();
    CfrEngHelper* Copy() const override;
    Ptr<Ipv4RoutingProtocol> Create(Ptr<Node> node) const override;
    void Set(const std::string& name, const AttributeValue& value);
  private:
    ObjectFactory m_factory;
};
}
#endif
