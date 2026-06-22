#pragma once

#include "planning/IPlanner.hpp"

class SimplePlanner : public IPlanner
{
public:
    struct Config
    {
        double desired_distance = 4.0;   // m — standoff distance
        double target_track_z   = 1.40;  // m — camera aim point height on the person
    };

    explicit SimplePlanner(const Config& cfg);

    Reference update(
        const State&          drone,
        const TargetEstimate& target,
        const IESDFMap&       esdf) override;

private:
    Config cfg_;
};
