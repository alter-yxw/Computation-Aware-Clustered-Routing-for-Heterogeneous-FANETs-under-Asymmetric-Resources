/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "mo-policy.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>

namespace ns3::mo
{
namespace
{
constexpr double INF = std::numeric_limits<double>::infinity();
double QueueDelay(double rate, double arrival, double horizon, double divisor)
{
    if (rate <= 0) return INF;
    // Equality is singular in the released code. Treat it as unstable as in Alg.1.
    return rate > arrival ? 1 / (rate - arrival) : horizon * arrival / (divisor * rate);
}
}
void Policy::Load(const std::string& path)
{
    std::ifstream in(path);
    std::string magic;
    uint32_t layers = 0, order = 0;
    if (!(in >> magic >> layers >> order) || magic != "MO_UPSTREAM_V1" || layers != 5 || order != 1)
        throw std::runtime_error("Invalid MO checkpoint (expected released 5-layer K=1 model): " + path);
    std::vector<Layer> loaded;
    const uint32_t dims[] = {4, 32, 32, 32, 32, 1};
    for (uint32_t l = 0; l < layers; ++l)
    {
        Layer layer;
        if (!(in >> layer.input >> layer.output) || layer.input != dims[l] || layer.output != dims[l + 1])
            throw std::runtime_error("Invalid MO layer dimensions");
        layer.kernel.resize(layer.input * layer.output); layer.bias.resize(layer.output);
        for (auto* values : {&layer.kernel, &layer.bias})
            for (double& v : *values)
                if (!(in >> v) || !std::isfinite(v)) throw std::runtime_error("Invalid MO weights");
        loaded.push_back(std::move(layer));
    }
    if (in >> magic) throw std::runtime_error("Trailing MO checkpoint content");
    m_layers = std::move(loaded);
}
std::vector<double> Policy::Predict(const std::vector<std::vector<double>>& features) const
{
    if (m_layers.empty()) throw std::runtime_error("MO learned policy needs a loaded checkpoint");
    std::vector<double> result;
    for (auto x : features)
    {
        if (x.size() != 4) throw std::runtime_error("MO input requires [virtual, rate, arrival, server]");
        for (double v : x) if (!std::isfinite(v) || v < 0) throw std::runtime_error("Invalid MO features");
        for (size_t l = 0; l < m_layers.size(); ++l)
        {
            const auto& layer = m_layers[l];
            std::vector<double> y = layer.bias;
            for (uint32_t i = 0; i < layer.input; ++i)
                for (uint32_t j = 0; j < layer.output; ++j) y[j] += x[i] * layer.kernel[i * layer.output + j];
            // TensorFlow's LeakyRelu alpha attribute is a float even for double
            // inputs. Preserve that conversion to match the released forward.
            for (auto& v : y) if (v < 0) v *= l + 1 == m_layers.size() ? 0 : double(0.2f);
            x = std::move(y);
        }
        if (!std::isfinite(x[0])) throw std::runtime_error("Non-finite MO prediction");
        result.push_back(x[0]);
    }
    return result;
}
std::vector<double> Policy::Delay(const std::vector<Link>& links,
                                 const std::vector<double>& arrivals, double horizon)
{
    if (links.size() != arrivals.size() || !std::isfinite(horizon) || horizon <= 0)
        throw std::runtime_error("Invalid MO queue inputs");
    // Conflict graph = line graph of physical connectivity (paper experiment).
    std::vector<std::vector<uint32_t>> conflict(links.size());
    for (uint32_t i = 0; i < links.size(); ++i)
    {
        if (!std::isfinite(links[i].rate) || links[i].rate <= 0 ||
            !std::isfinite(arrivals[i]) || arrivals[i] < 0) throw std::runtime_error("Invalid MO link rate");
        for (uint32_t j = 0; j < i; ++j)
            if (links[i].a == links[j].a || links[i].a == links[j].b ||
                links[i].b == links[j].a || links[i].b == links[j].b)
            { conflict[i].push_back(j); conflict[j].push_back(i); }
    }
    std::vector<double> mu(links.size()), busy(links.size());
    for (uint32_t i = 0; i < links.size(); ++i) mu[i] = links[i].rate / (1 + conflict[i].size());
    for (int k = 0; k < 10; ++k)
    {
        for (uint32_t i = 0; i < links.size(); ++i) busy[i] = std::min(1.0, arrivals[i] / mu[i]);
        for (uint32_t i = 0; i < links.size(); ++i)
        {
            double contenders = 1;
            for (auto j : conflict[i]) contenders += busy[j];
            mu[i] = links[i].rate / contenders;
        }
    }
    for (uint32_t i = 0; i < links.size(); ++i) mu[i] = QueueDelay(mu[i], arrivals[i], horizon, 101);
    return mu;
}
Solution Policy::Solve(const std::vector<Vertex>& nodes, const std::vector<Link>& links,
                       double horizon, bool learned) const
{
    Solution s;
    const auto n = nodes.size();
    std::vector<std::vector<double>> features;
    for (const auto& e : links)
    {
        if (e.a >= n || e.b >= n || e.a == e.b || !std::isfinite(e.rate) || e.rate <= 0)
            throw std::runtime_error("Invalid MO connectivity edge");
        features.push_back({0, e.rate, 0, 0});
    }
    for (const auto& v : nodes)
    {
        if (!std::isfinite(v.rate) || v.rate < 0 || !std::isfinite(v.arrival) || v.arrival < 0)
            throw std::runtime_error("Invalid MO vertex");
        if (v.rate > 0) features.push_back({1, v.rate, v.arrival, double(v.server)});
    }
    s.cpuWeights.resize(n, INF);
    if (learned)
    {
        s.prediction = Predict(features);
        std::vector<double> physical(s.prediction.begin(), s.prediction.begin() + links.size());
        s.weights = Delay(links, physical, horizon);
        size_t p = links.size();
        for (size_t i = 0; i < n; ++i) if (nodes[i].rate > 0)
            s.cpuWeights[i] = QueueDelay(nodes[i].rate, s.prediction[p++], horizon, 100);
    }
    else
    {
        for (const auto& e : links) s.weights.push_back(1 / e.rate);
        for (size_t i = 0; i < n; ++i) if (nodes[i].rate > 0) s.cpuWeights[i] = 1 / nodes[i].rate;
    }
    s.distance.assign(n, std::vector<double>(n, INF));
    s.next.assign(n, std::vector<int>(n, -1));
    s.hops.assign(n, std::vector<int>(n, 1000000));
    for (size_t i = 0; i < n; ++i) { s.distance[i][i] = 0; s.hops[i][i] = 0; }
    for (size_t i = 0; i < links.size(); ++i)
    {
        const auto& e = links[i];
        s.distance[e.a][e.b] = s.distance[e.b][e.a] = s.weights[i];
        s.next[e.a][e.b] = e.b; s.next[e.b][e.a] = e.a;
        s.hops[e.a][e.b] = s.hops[e.b][e.a] = 1;
    }
    for (size_t k = 0; k < n; ++k)
        for (size_t i = 0; i < n; ++i)
            for (size_t j = 0; j < n; ++j)
            {
                if (s.distance[i][k] + s.distance[k][j] < s.distance[i][j])
                { s.distance[i][j] = s.distance[i][k] + s.distance[k][j]; s.next[i][j] = s.next[i][k]; }
                s.hops[i][j] = std::min(s.hops[i][j], s.hops[i][k] + s.hops[k][j]);
            }
    return s;
}
double Policy::Cost(const Solution& s, uint32_t source, uint32_t dest, double up, double down)
{
    if (source == dest) return up * s.cpuWeights.at(source);
    return std::max(up * s.distance.at(source).at(dest), double(s.hops.at(source).at(dest))) +
           std::max(down * s.distance.at(dest).at(source), double(s.hops.at(dest).at(source))) +
           std::max(up * s.cpuWeights.at(dest), 1.0);
}
uint32_t Policy::Select(const Solution& s, const std::vector<Vertex>& nodes,
                        uint32_t source, double upload, double download, bool localOnly)
{
    if (source >= nodes.size() || upload <= 0 || download < 0 ||
        !std::isfinite(upload) || !std::isfinite(download)) throw std::runtime_error("Invalid MO task");
    if (localOnly) return source;
    double best = INF;
    uint32_t destination = source;
    // Upstream lists servers before local; deterministic tie break by node slot.
    for (uint32_t j = 0; j < nodes.size(); ++j) if (nodes[j].server && j != source)
    {
        const auto cost = Cost(s, source, j, upload, download);
        if (cost < best) { best = cost; destination = j; }
    }
    if (Cost(s, source, source, upload, download) < best) destination = source;
    return destination;
}
}
