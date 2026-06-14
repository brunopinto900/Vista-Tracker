#pragma once

#include "models/State.hpp"
#include "models/TargetState.hpp"

struct TrajectoryPlanner
{
    State computeDesired(const State& /*drone*/,
                         const TargetState& target)
    {
        State ref{};
        ref.x = target.x - 4.0;
        ref.y = target.y;
        ref.z = target.z;
        return ref;
    }
};
