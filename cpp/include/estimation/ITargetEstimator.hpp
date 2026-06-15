#pragma once

#include "models/Detection.hpp"
#include "models/TargetEstimate.hpp"

class ITargetEstimator
{
public:
    virtual ~ITargetEstimator() = default;

    virtual TargetEstimate update(const Detection& detection, double dt) = 0;
};
