/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef QRCF_SETTINGS_H
#define QRCF_SETTINGS_H
#include "qrcf-model.h"
#include <string>

namespace ns3::qrcf
{
// Values explicitly approved by the user on 2026-10-18. These are engineering
// reproduction settings; the paper does NOT give these numerical constants.
struct Settings
{
    double range = 250; // paper value; benchmark must override with shared MaxRange
    double helloLower = 0.25;
    double helloUpper = 1;
    double errorMin = 1;
    double errorMax = 5;
    double directionThreshold = 0.5;
    Components weights{0.25,0.25,0.25,0.25};
    double rewardMax = 2;
    double rewardMin = -1;
    double connectionCap = 60;
    // Versioned mechanism profiles; numerical paper/engineering constants above
    // are deliberately unchanged by ApplyPreset.
    bool outcomeUpdates = true;
    bool bootstrap = true;
    bool helloJitter = true;

    void ApplyPreset(const std::string& preset);
    std::string PresetName() const;

    void Validate() const;
    // Approved tie/infinite-CT policies for the paper's formulas.
    Scalar Normalize(std::size_t component, double value, double minimum, double maximum) const;
    Scalar CapConnection(Scalar literal, bool knownZeroDenominator) const;
    static Motion FromVelocity(Vector velocity);
    Scalar Connection(Vector self, const Motion& selfMotion, Vector peer,
                      const Motion& peerMotion, Scalar previousDistance) const;
    std::optional<bool> Filter(Vector self, Vector neighbor, Vector destination) const;
};
} // namespace ns3::qrcf
#endif
