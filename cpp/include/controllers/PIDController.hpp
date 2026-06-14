#pragma once

#include "IController.hpp"
#include "PID.hpp"

class PIDController : public IController
{
public:

    PIDController(
        double desired_distance,
        double kp,
        double ki,
        double kd);

    ControlCommand update(
        const State& drone,
        const TargetState& target,
        double dt) override;

private:

    double desired_distance_;

    PID pid_x_;
    PID pid_y_;
    PID pid_z_;
};