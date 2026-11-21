/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
// Port of zhongyuanzhao/multihop-offload, MIT; see ../upstream/LICENSE.
#ifndef MO_POLICY_H
#define MO_POLICY_H
#include <cstdint>
#include <string>
#include <vector>

namespace ns3::mo
{
struct Vertex
{
    double rate = 0;    // CPU packets per policy slot; zero denotes a relay.
    double arrival = 0; // Input packets per policy slot at the source virtual edge.
    bool server = false;
};
struct Link
{
    uint32_t a, b;
    double rate;
};
struct Solution
{
    std::vector<double> prediction, weights, cpuWeights;
    std::vector<std::vector<double>> distance;
    std::vector<std::vector<int>> next, hops;
};
class Policy
{
  public:
    // The released checkpoint is ChebConv K=1, i.e. a per-edge dense network.
    // Refuse other orders rather than silently approximate their graph filter.
    void Load(const std::string& path);
    std::vector<double> Predict(const std::vector<std::vector<double>>& features) const;
    Solution Solve(const std::vector<Vertex>& nodes, const std::vector<Link>& links,
                   double horizonSlots, bool learned) const;
    // Upstream offloading_v3.py: separate upload/download/CPU slot lower bounds.
    static uint32_t Select(const Solution& solution, const std::vector<Vertex>& nodes,
                           uint32_t source, double upload, double download, bool localOnly = false);
    static double Cost(const Solution& solution, uint32_t source, uint32_t destination,
                       double upload, double download);
    static std::vector<double> Delay(const std::vector<Link>& links,
                                     const std::vector<double>& arrivals, double horizonSlots);
  private:
    struct Layer { uint32_t input, output; std::vector<double> kernel, bias; };
    std::vector<Layer> m_layers;
};
}
#endif
