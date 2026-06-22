#pragma once

#include "planning/IPlanner.hpp"

class SimplePlanner : public IPlanner
{
public:
    struct Config
    {
        double desired_distance = 4.0;    // m — standoff distance
        double vfov_half_rad    = 0.5236; // camera V-FOV half-angle (rad) — 30° default
        double min_z            = 2.0;    // minimum safe altitude (m)
        double target_track_z   = 1.40;   // m — camera aim point height on the person
    };

    explicit SimplePlanner(const Config& cfg);

    Reference update(
        const State&          drone,
        const TargetEstimate& target,
        const IESDFMap&       esdf) override;

private:
    Config cfg_;
};
