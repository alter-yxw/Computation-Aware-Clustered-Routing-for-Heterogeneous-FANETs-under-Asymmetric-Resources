/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#include "qrcf-settings.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ns3::qrcf
{
void Settings::ApplyPreset(const std::string& preset)
{
    if (preset != "legacy-noq" && preset != "staggered-noq" &&
        preset != "outcome-immediate" && preset != "outcome-bellman")
    { throw std::invalid_argument("Unknown QRCF preset: " + preset); }
    outcomeUpdates = preset == "outcome-immediate" || preset == "outcome-bellman";
    bootstrap = preset == "outcome-bellman";
    helloJitter = preset != "legacy-noq";
}
std::string Settings::PresetName() const
{
    if (!outcomeUpdates && !bootstrap) { return helloJitter ? "staggered-noq" : "legacy-noq"; }
    if (outcomeUpdates && helloJitter) { return bootstrap ? "outcome-bellman" : "outcome-immediate"; }
    throw std::invalid_argument("QRCF mechanism flags do not describe a supported preset");
}
void Settings::Validate() const
{
    PresetName();
    Model::InitialHello(helloLower,helloUpper);
    Model::Reward({0,0,0,0},weights);
    if (!std::isfinite(range) || range <= 0 || !std::isfinite(errorMin) || errorMin < 0 ||
        !std::isfinite(errorMax) || errorMax <= errorMin || !std::isfinite(directionThreshold) ||
        directionThreshold < 0 || directionThreshold > 1 || !std::isfinite(rewardMax) ||
        rewardMax < 1 || !std::isfinite(rewardMin) || rewardMin >= 0 ||
        !std::isfinite(connectionCap) || connectionCap <= 0)
    { throw std::invalid_argument("Invalid QRCF reproduction settings"); }
}
Scalar Settings::Normalize(std::size_t component, double value, double minimum, double maximum) const
{
    if (component >= weights.size()) { throw std::invalid_argument("Invalid QRCF component index"); }
    if (std::isfinite(value) && value == minimum && minimum == maximum) { return 0.5; }
    return Model::Normalize(value,minimum,maximum);
}
Scalar Settings::CapConnection(Scalar literal, bool knownZeroDenominator) const
{
    if (!literal) { return knownZeroDenominator ? Scalar{connectionCap} : Scalar{}; }
    if (std::isnan(*literal) || *literal < 0) { return {}; }
    return std::min(*literal,connectionCap);
}
std::optional<bool> Settings::Filter(Vector self, Vector neighbor, Vector destination) const
{
    return Model::AlignedFilter(self,neighbor,destination,range);
}
Motion Settings::FromVelocity(Vector velocity)
{
    const double speed = velocity.GetLength();
    if (!std::isfinite(speed)) { throw std::invalid_argument("Nonfinite QRCF velocity"); }
    if (speed == 0) { return {0,0,0}; } // explicitly approved stationary convention
    double theta = std::atan2(velocity.y,velocity.x);
    if (theta < 0) { theta += 2*std::acos(-1.0); }
    return {speed,theta,std::acos(std::clamp(velocity.z/speed,-1.0,1.0))};
}
Scalar Settings::Connection(Vector self, const Motion& selfMotion, Vector peer,
                            const Motion& peerMotion, Scalar previousDistance) const
{
    const double distance = CalculateDistance(self,peer);
    if (distance > range) { return {}; }
    const Vector selfVelocity = Model::Velocity(selfMotion);
    const Vector peerVelocity = Model::Velocity(peerMotion);
    const auto dot = [](Vector a, Vector b) { return a.x*b.x+a.y*b.y+a.z*b.z; };
    int alpha1 = 1, alpha2 = 1;
    if (Model::DirectionDifference(selfMotion,peerMotion) <= directionThreshold)
    {
        // Projection of faster-minus-slower onto the faster node's heading.
        // A faster node behind approaches, a faster node ahead separates.
        if (selfMotion.speed > peerMotion.speed)
        { alpha1 = dot(self-peer,selfVelocity) < 0 ? -1 : 1; }
        else if (peerMotion.speed > selfMotion.speed)
        { alpha1 = dot(peer-self,peerVelocity) < 0 ? -1 : 1; }
        // Equal speed/side-by-side explicitly approved to use +1.
    }
    else
    {
        alpha2 = -1;
        const bool approaching = previousDistance ? distance <= *previousDistance :
            dot(self-peer,selfVelocity-peerVelocity) <= 0;
        alpha1 = approaching ? -1 : 1;
    }
    const auto literal = Model::ConnectionTime(range,distance,selfMotion,peerMotion,alpha1,alpha2);
    return CapConnection(literal,!literal.has_value()); // only zero denominator in this valid domain
}
} // namespace ns3::qrcf
