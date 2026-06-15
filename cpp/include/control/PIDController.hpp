#pragma once

#include "control/IController.hpp"
#include "control/PID.hpp"

class PIDController : public IController
{
public:
    PIDController(double kp, double ki, double kd);

    ControlCommand update(
        const State&     drone,
        const Reference& reference,
        double           dt) override;

private:
    PID pid_x_;
    PID pid_y_;
    PID pid_z_;
};
