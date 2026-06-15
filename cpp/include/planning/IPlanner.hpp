#pragma once

#include "models/State.hpp"
#include "models/TargetEstimate.hpp"
#include "models/Reference.hpp"

class IPlanner
{
public:
    virtual ~IPlanner() = default;

    virtual Reference update(
        const State& drone,
        const TargetEstimate& target) = 0;
};
