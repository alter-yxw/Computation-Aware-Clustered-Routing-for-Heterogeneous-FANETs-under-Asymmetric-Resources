/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
// A small stream interface used for comparison with the original Python pipeline.
#include "../model/mo-policy.h"
#include <iomanip>
#include <iostream>
#include <stdexcept>
int main(int argc, char** argv)
{
    try
    {
        if (argc != 2) throw std::runtime_error("policy-driver CHECKPOINT < case.txt");
        ns3::mo::Policy policy; policy.Load(argv[1]);
        uint32_t n, e, learned; double horizon, up, down;
        if (!(std::cin >> n >> e >> learned >> horizon >> up >> down)) throw std::runtime_error("Invalid case");
        std::vector<ns3::mo::Vertex> nodes(n);
        std::vector<ns3::mo::Link> links(e);
        for (auto& v : nodes) std::cin >> v.rate >> v.arrival >> v.server;
        for (auto& v : links) std::cin >> v.a >> v.b >> v.rate;
        if (!std::cin) throw std::runtime_error("Truncated case");
        auto s = policy.Solve(nodes, links, horizon, learned);
        std::cout << std::setprecision(17);
        for (const auto* values : {&s.prediction, &s.weights, &s.cpuWeights})
        { for (auto x : *values) std::cout << x << ' '; std::cout << '\n'; }
        for (const auto& row : s.distance) { for (auto x : row) std::cout << x << ' '; std::cout << '\n'; }
        for (uint32_t i = 0; i < n; ++i) std::cout << policy.Select(s, nodes, i, up, down) << ' ';
        std::cout << '\n';
    }
    catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
