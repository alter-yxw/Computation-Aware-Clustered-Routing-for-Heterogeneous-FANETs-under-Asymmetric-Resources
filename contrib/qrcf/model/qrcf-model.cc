/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "qrcf-model.h"
#include <algorithm>
#include <cmath>
#include <iterator>
#include <numeric>
#include <stdexcept>

namespace ns3::qrcf
{
namespace
{
void Require(bool condition, const char* message)
{
    if (!condition) { throw std::invalid_argument(message); }
}
void Unit(double x)
{
    Require(std::isfinite(x) && x >= 0 && x <= 1, "QRCF normalized value outside [0,1]");
}
void Bounds(double lower, double upper)
{
    Require(std::isfinite(lower) && std::isfinite(upper) && lower > 0 && upper >= lower,
            "QRCF needs explicit positive Hello bounds");
}
double Dot(Vector a, Vector b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
Scalar Angle(Vector a, Vector b)
{
    const double denominator = a.GetLength()*b.GetLength();
    if (!(denominator > 0) || !std::isfinite(denominator)) { return {}; }
    const double cosine = Dot(a,b)/denominator;
    // Clamp ONLY floating point roundoff in a mathematically bounded cosine.
    return std::acos(std::clamp(cosine, -1.0, 1.0));
}
}
Vector Model::Velocity(const Motion& m)
{
    Require(std::isfinite(m.speed) && m.speed >= 0 && std::isfinite(m.azimuth) &&
            std::isfinite(m.polar), "Invalid QRCF spherical motion");
    return {m.speed*std::sin(m.polar)*std::cos(m.azimuth),
            m.speed*std::sin(m.polar)*std::sin(m.azimuth), m.speed*std::cos(m.polar)};
}
double Model::DirectionDifference(const Motion& a, const Motion& b)
{
    Velocity(a); Velocity(b);
    // The paper mixes speed and angular units here. Do not normalize or wrap.
    return std::hypot(a.speed-b.speed, a.azimuth-b.azimuth, a.polar-b.polar);
}
Scalar Model::ConnectionTime(double range, double distance, const Motion& a,
                             const Motion& b, int alpha1, int alpha2)
{
    Require(std::isfinite(range) && range > 0 && std::isfinite(distance) && distance >= 0,
            "Invalid QRCF range/distance");
    Require((alpha1 == -1 || alpha1 == 1) && (alpha2 == -1 || alpha2 == 1),
            "QRCF CT coefficients must be +1 or -1");
    const Vector av = Velocity(a), bv = Velocity(b);
    const double denominator = (av - Vector{bv.x*alpha2,bv.y*alpha2,bv.z*alpha2}).GetLength();
    if (denominator == 0) { return {}; }
    return (range-alpha1*distance)/denominator;
}
Vector Model::Predict(Vector position, const Motion& motion, double seconds)
{
    Require(std::isfinite(seconds) && seconds >= 0, "Invalid prediction interval");
    const Vector v = Velocity(motion);
    return position + Vector{v.x*seconds,v.y*seconds,v.z*seconds};
}
double Model::InitialHello(double lower, double upper)
{
    Bounds(lower,upper); return (lower+upper)/2;
}
Scalar Model::AdaptHello(double current, double lower, double upper, double error,
                         double errorMin, double errorMax, double meanCT, double maxCT)
{
    Bounds(lower,upper);
    Require(std::isfinite(current) && current >= lower && current <= upper &&
            std::isfinite(error) && error >= 0 && std::isfinite(errorMin) &&
            std::isfinite(errorMax) && errorMin >= 0 && errorMax > errorMin,
            "Invalid QRCF Hello adaptation input");
    if (error > errorMin && error < errorMax) { return current; }
    if (!std::isfinite(meanCT) || !std::isfinite(maxCT) || meanCT < 0 || maxCT <= 0 || meanCT > maxCT)
    { return {}; }
    const double factor = std::exp(-meanCT/maxCT);
    return error >= errorMax ? std::max(current*factor,lower)
                             : std::min(current*(1+factor),upper);
}
double Model::RelativeSpeed(const Motion& a, const Motion& b)
{
    return (Velocity(a)-Velocity(b)).GetLength();
}
std::optional<bool> Model::AlignedFilter(Vector self, Vector neighbor, Vector destination, double range)
{
    Require(std::isfinite(range) && range > 0, "Invalid cylinder range");
    const Vector axis = destination-self;
    const double length = axis.GetLength();
    if (!(length > 0) || !std::isfinite(length)) { return {}; }
    const Vector unit{axis.x/length,axis.y/length,axis.z/length};
    const Vector offset = neighbor-self;
    const double height = Dot(offset,unit);
    const Vector radial = offset-Vector{unit.x*height,unit.y*height,unit.z*height};
    return height >= 0 && height <= range && radial.GetLength() <= range;
}
Scalar Model::Normalize(double value, double minimum, double maximum)
{
    if (!std::isfinite(value) || !std::isfinite(minimum) || !std::isfinite(maximum) || maximum == minimum)
    { return {}; }
    Require(maximum > minimum && value >= minimum && value <= maximum,
            "QRCF normalization requires value within min/max");
    return (value-minimum)/(maximum-minimum);
}
Scalar Model::MovementAngle(Vector self, Vector neighbor, Vector destination)
{
    return Angle(self-destination, self-neighbor); // position vectors, NOT velocity
}
double Model::Reward(const Components& n, const Components& w)
{
    for (double x : n) { Unit(x); }
    for (double x : w) { Unit(x); }
    Require(std::abs(std::accumulate(w.begin(),w.end(),0.0)-1) <= 1e-12,
            "QRCF reward weights must sum to one");
    return w[0]*(1-n[0]) + w[1]*n[1] + w[2]*n[2] + w[3]*(1-n[3]);
}
double Model::LearningRate(double ct) { Unit(ct); return std::exp(-ct); }
Scalar Model::Discount(const std::set<uint32_t>& a, const std::set<uint32_t>& b)
{
    std::vector<uint32_t> intersection;
    std::set_intersection(a.begin(),a.end(),b.begin(),b.end(),std::back_inserter(intersection));
    const std::size_t unionSize = a.size()+b.size()-intersection.size();
    if (unionSize == 0) { return {}; }
    return double(intersection.size())/unionSize;
}
double Model::UpdateQ(double q, double reward, double alpha, double gamma, double next)
{
    Unit(alpha); Unit(gamma);
    Require(alpha > 0 && std::isfinite(q) && std::isfinite(reward) && std::isfinite(next),
            "Invalid QRCF Q-learning update");
    return (1-alpha)*q + alpha*(reward+gamma*next);
}
} // namespace ns3::qrcf
