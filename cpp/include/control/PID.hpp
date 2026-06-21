#pragma once

#include <algorithm>

// Velocity-error PID (kp + ki).  Anti-windup clamps ki contribution to
// ±kMaxContrib to prevent accumulation during large transients.
class PID
{
public:
    static constexpr double kMaxContrib = 4.0;  // max m/s² from integral

    PID(double kp, double ki)
        : kp_(kp), ki_(ki), integral_(0.0) {}

    double update(double error, double dt)
    {
        integral_ += error * dt;
        if (ki_ > 1e-9) {
            const double lim = kMaxContrib / ki_;
            integral_ = std::clamp(integral_, -lim, lim);
        }
        return kp_ * error + ki_ * integral_;
    }

    void reset() { integral_ = 0.0; }

private:
    double kp_, ki_;
    double integral_;
};
