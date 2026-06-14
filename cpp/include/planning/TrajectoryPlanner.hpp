#pragma once

#include "models/State.hpp"
#include "models/TargetState.hpp"

struct TrajectoryPlanner
{
    explicit TrajectoryPlanner(double desired_distance)
        : desired_distance_(desired_distance) {}

    State computeDesired(const State& /*drone*/,
                         const TargetState& target) const
    {
        State ref{};
        ref.x = target.x - desired_distance_;
        ref.y = target.y;
        ref.z = target.z;
        return ref;
    }

private:
    double desired_distance_;
};
