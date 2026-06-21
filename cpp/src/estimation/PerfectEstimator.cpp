#include "estimation/PerfectEstimator.hpp"

PerfectEstimator::PerfectEstimator(int horizon)
    : horizon_(horizon) {}

TargetEstimate PerfectEstimator::update(const Detection& det, double dt)
{
    PredictedTargetState s;
    s.x = det.u;
    s.y = det.v;
    s.z = det.z;

    if (det.has_velocity)
    {
        s.vx = det.vx;
        s.vy = det.vy;
        s.vz = det.vz;
    }
    else if (dt > 0.0 && prev_valid_)
    {
        s.vx = (det.u - prev_.x) / dt;
        s.vy = (det.v - prev_.y) / dt;
        s.vz = (det.z - prev_.z) / dt;
    }

    prev_       = { s.x, s.y, s.z };
    prev_valid_ = det.valid;

    TargetEstimate est;
    est.horizon.reserve(horizon_);
    est.timestamp = det.timestamp;

    for (int k = 0; k < horizon_; ++k)
    {
        PredictedTargetState step;
        step.x  = s.x  + s.vx * k * dt;
        step.y  = s.y  + s.vy * k * dt;
        step.z  = s.z  + s.vz * k * dt;
        step.vx = s.vx;
        step.vy = s.vy;
        step.vz = s.vz;
        est.horizon.push_back(step);
    }

    return est;
}
