/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef QRCF_MODEL_H
#define QRCF_MODEL_H

#include "ns3/vector.h"
#include <array>
#include <cstdint>
#include <optional>
#include <set>

namespace ns3::qrcf
{

// A missing value denotes an unspecified/undefined paper domain. The protocol
// caller must supply an explicit policy; the formula layer never invents one.
using Scalar = std::optional<double>;
struct Motion
{
    double speed;
    double azimuth; // theta, radians
    double polar;   // phi, radians FROM +z, not elevation above the xy plane
};
using Components = std::array<double, 4>; // speed, CT, energy, lambda

class Model
{
  public:
    static Vector Velocity(const Motion& motion);                       // (5)-(7)
    static double DirectionDifference(const Motion& a, const Motion& b); // literal MD
    // alpha1/alpha2 are explicit because ahead/behind and tied motion are
    // underspecified in section 5.1.2. Zero denominator returns nullopt.
    static Scalar ConnectionTime(double range, double distance, const Motion& a,
                                 const Motion& b, int alpha1, int alpha2); // (3)-(7)
    static Vector Predict(Vector position, const Motion& motion, double seconds); // (9)
    static double InitialHello(double lower, double upper);             // (8), t=0
    static Scalar AdaptHello(double current, double lower, double upper,
                             double predictionError, double errorMin, double errorMax,
                             double meanConnection, double maxConnection); // (8)
    // Align cylinder z with self->destination (the sole supported geometry).
    // Orthogonal projection is equivalent to a rigid coordinate rotation for
    // the full-azimuth cylinder and avoids singular axis/atan cases entirely.
    // This is a geometric correction, NOT the published Eq (14)-(15).
    static std::optional<bool> AlignedFilter(Vector self, Vector neighbor,
                                            Vector destination, double range);
    static double RelativeSpeed(const Motion& a, const Motion& b);        // (19)-(22)
    static Scalar Normalize(double value, double minimum, double maximum);// (23)-(25),(31)
    static Scalar MovementAngle(Vector self, Vector neighbor, Vector destination); // (26)-(30)
    static double Reward(const Components& normalized, const Components& weights); // (32), ordinary
    static double LearningRate(double normalizedConnection);             // (33)
    static Scalar Discount(const std::set<uint32_t>& previous,
                           const std::set<uint32_t>& current);            // (34)
    static double UpdateQ(double oldQ, double reward, double alpha,
                          double gamma, double maximumNextQ);            // (1)
};
} // namespace ns3::qrcf
#endif
