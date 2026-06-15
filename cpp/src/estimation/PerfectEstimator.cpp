#include "estimation/PerfectEstimator.hpp"

TargetEstimate PerfectEstimator::update(const Detection& det, double dt)
{
    PredictedTargetState s;
    s.x = det.u;
    s.y = det.v;
    s.z = det.z;

    if (dt > 0.0 && prev_valid_)
    {
        s.vx = (det.u - prev_.x) / dt;
        s.vy = (det.v - prev_.y) / dt;
        s.vz = (det.z - prev_.z) / dt;
    }

    prev_       = { s.x, s.y, s.z };
    prev_valid_ = det.valid;

    TargetEstimate est;
    est.horizon   = { s };
    est.timestamp = det.timestamp;
    return est;
}
