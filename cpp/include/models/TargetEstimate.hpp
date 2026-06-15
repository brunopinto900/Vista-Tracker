#pragma once

#include <vector>

struct PredictedTargetState
{
    double x  = 0.0;
    double y  = 0.0;
    double z  = 0.0;

    double vx = 0.0;
    double vy = 0.0;
    double vz = 0.0;
};

struct TargetEstimate
{
    std::vector<PredictedTargetState> horizon;

    double timestamp = 0.0;
};
