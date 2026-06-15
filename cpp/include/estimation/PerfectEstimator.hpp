#pragma once

#include "estimation/ITargetEstimator.hpp"

class PerfectEstimator : public ITargetEstimator
{
public:
    TargetEstimate update(const Detection& det, double dt) override;

private:
    struct { double x, y, z; } prev_{};
    bool prev_valid_ = false;
};
