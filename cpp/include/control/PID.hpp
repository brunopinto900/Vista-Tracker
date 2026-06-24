#pragma once

#include <algorithm>

// Velocity-error PID (kp + ki).  Anti-windup clamps ki contribution to
// ±max_contribution to prevent accumulation during large transients.
class PID
{
public:
    PID(double kp, double ki, double max_contribution = 4.0)
        : kp_(kp), ki_(ki), max_contribution_(max_contribution), integral_(0.0) {}

    double update(double error, double dt)
    {
        integral_ += error * dt;
        if (ki_ > 1e-9) {
            const double lim = max_contribution_ / ki_;
            integral_ = std::clamp(integral_, -lim, lim);
        }
        return kp_ * error + ki_ * integral_;
    }

    void reset() { integral_ = 0.0; }

private:
    double kp_, ki_, max_contribution_;
    double integral_;
};
