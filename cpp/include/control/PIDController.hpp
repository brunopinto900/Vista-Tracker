#pragma once

#include "control/IController.hpp"
#include "control/PID.hpp"

// PX4-style position-velocity cascade controller.
//
// Outer loop (position PI):  vel_sp = kp_pos * pos_err + ki_pos * ∫pos_err + ref_vel
// Inner loop (velocity PID): accel  = kp_vel * vel_err + ki_vel * ∫vel_err
// Attitude conversion:       accel  → roll/pitch setpoints → body-rate commands
class PIDController : public IController
{
public:
    PIDController(double kp_pos, double ki_pos, double kp_vel, double ki_vel,
                  double attitude_kp           = 6.680,
                  double yaw_kp                = 0.287,
                  double max_tilt_rad          = 0.5,
                  double max_thrust            = 2.0,
                  double max_ipos_contribution = 1.0,
                  double max_ivel_contribution = 4.0);

    ControlCommand update(
        const State&     drone,
        const Reference& reference,
        double           dt) override;

private:
    double kp_pos_, ki_pos_;
    double ip_x_ = 0.0, ip_y_ = 0.0, ip_z_ = 0.0;  // position-error integrals
    PID    pid_vx_, pid_vy_, pid_vz_;  // velocity-error integrators (kp_vel, ki_vel)
    double attitude_kp_;
    double yaw_kp_;
    double max_tilt_rad_;
    double max_thrust_;
    double max_ipos_contribution_;
    double max_ivel_contribution_;
};
