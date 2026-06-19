#pragma once

#include "control/IController.hpp"
#include "control/PID.hpp"

// PX4-style position-velocity cascade controller.
//
// Outer loop (position P):   vel_sp = kp_pos * pos_err + ref_vel  (feedforward)
// Inner loop (velocity PID): accel  = kp_vel * vel_err + ki_vel * ∫vel_err
// Attitude conversion:       accel  → roll/pitch setpoints → body-rate commands
class PIDController : public IController
{
public:
    PIDController(double kp_pos, double kp_vel, double ki_vel,
                  double attitude_kp = 5.0,
                  double yaw_kp      = 0.3);

    ControlCommand update(
        const State&     drone,
        const Reference& reference,
        double           dt) override;

private:
    double kp_pos_;
    PID    pid_vx_, pid_vy_, pid_vz_;  // velocity-error integrators (kp_vel, ki_vel)
    double attitude_kp_;
    double yaw_kp_;
};
