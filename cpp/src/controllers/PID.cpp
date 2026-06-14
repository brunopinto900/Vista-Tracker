#include "controllers/PID.hpp"

PID::PID(
    double kp,
    double ki,
    double kd)
    :
    kp_(kp),
    ki_(ki),
    kd_(kd),
    integral_(0.0),
    previous_error_(0.0)
{
}

double PID::update(
    double error,
    double dt)
{
    integral_ += error * dt;

    double derivative =
        (error - previous_error_) / dt;

    previous_error_ = error;

    return
        kp_ * error +
        ki_ * integral_ +
        kd_ * derivative;
}