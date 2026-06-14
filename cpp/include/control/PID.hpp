#pragma once

class PID
{
public:
    PID(double kp, double ki, double kd)
        : kp_(kp), ki_(ki), kd_(kd),
          integral_(0.0), prev_error_(0.0) {}

    double update(double error, double dt)
    {
        integral_  += error * dt;
        double derivative = (error - prev_error_) / dt;
        prev_error_ = error;

        return kp_ * error
             + ki_ * integral_
             + kd_ * derivative;
    }

    void reset()
    {
        integral_   = 0.0;
        prev_error_ = 0.0;
    }

private:
    double kp_, ki_, kd_;
    double integral_;
    double prev_error_;
};
