#pragma once

#include "planning/IPlanner.hpp"

class SimplePlanner : public IPlanner
{
public:
    explicit SimplePlanner(double desired_distance)
        : desired_distance_(desired_distance) {}

    Reference update(
        const State& /*drone*/,
        const TargetEstimate& target) override
    {
        const auto& t = target.horizon[0];

        Reference ref;
        ref.x = t.x - desired_distance_;
        ref.y = t.y;
        ref.z = t.z;
        return ref;
    }

private:
    double desired_distance_;
};
