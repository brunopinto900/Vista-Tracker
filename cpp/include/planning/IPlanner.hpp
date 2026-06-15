#pragma once

#include "models/State.hpp"
#include "models/TargetEstimate.hpp"
#include "models/Reference.hpp"
#include "mapping/IESDFMap.hpp"

class IPlanner
{
public:
    virtual ~IPlanner() = default;

    virtual Reference update(
        const State&          drone,
        const TargetEstimate& target,
        const IESDFMap&       esdf) = 0;
};
