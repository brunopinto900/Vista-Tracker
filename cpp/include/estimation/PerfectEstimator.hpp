#pragma once

#include "estimation/ITargetEstimator.hpp"

class PerfectEstimator : public ITargetEstimator
{
public:
    explicit PerfectEstimator(int horizon);

    TargetEstimate update(const Detection& det, double dt) override;

private:
    int  horizon_;

    struct { double x, y, z; } prev_{};
    bool prev_valid_ = false;
};
