#pragma once

#include "planning/IPlanner.hpp"

class SimplePlanner : public IPlanner
{
public:
    explicit SimplePlanner(double desired_distance);

    Reference update(
        const State&          drone,
        const TargetEstimate& target,
        const IESDFMap&       esdf) override;

private:
    double desired_distance_;
};
