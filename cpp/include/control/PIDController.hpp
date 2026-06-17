#pragma once

#include "control/IController.hpp"
#include "control/PID.hpp"

// Cascade controller: position error → desired acceleration →
//                     attitude setpoints → body-rate commands.
// Serves as a placeholder until the MPC replaces it.
class PIDController : public IController
{
public:
    // attitude_kp: inner-loop proportional gain (rad/s per rad of attitude error)
    PIDController(double kp, double ki, double kd, double attitude_kp = 5.0);

    ControlCommand update(
        const State&     drone,
        const Reference& reference,
        double           dt) override;

private:
    PID    pid_x_;
    PID    pid_y_;
    PID    pid_z_;
    double attitude_kp_;
};
