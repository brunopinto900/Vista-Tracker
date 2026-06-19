#pragma once

// Position-error integrating PID (kp + ki only).
// Velocity damping is handled separately in PIDController via kv * drone_vel,
// which avoids numerical-derivative noise from kd * d(error)/dt.
class PID
{
public:
    PID(double kp, double ki)
        : kp_(kp), ki_(ki), integral_(0.0) {}

    double update(double error, double dt)
    {
        integral_ += error * dt;
        return kp_ * error + ki_ * integral_;
    }

    void reset() { integral_ = 0.0; }

private:
    double kp_, ki_;
    double integral_;
};
