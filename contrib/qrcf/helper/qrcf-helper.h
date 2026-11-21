/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef QRCF_HELPER_H
#define QRCF_HELPER_H
#include "ns3/ipv4-routing-helper.h"
#include "ns3/node-container.h"
#include "ns3/qrcf-settings.h"

namespace ns3
{
class QrcfHelper : public Ipv4RoutingHelper
{
  public:
    QrcfHelper* Copy() const override;
    Ptr<Ipv4RoutingProtocol> Create(Ptr<Node>) const override;
    void SetSettings(qrcf::Settings settings) { settings.Validate(); m_settings = settings; }
    int64_t AssignStreams(NodeContainer nodes,int64_t stream) const;
  private:
    qrcf::Settings m_settings;
};
}
#endif
