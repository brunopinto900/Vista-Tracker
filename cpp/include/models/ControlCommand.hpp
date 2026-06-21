#pragma once

// Output of IController — maps directly to PX4's body-rate setpoint interface.
// roll_rate / pitch_rate / yaw_rate in rad/s (body frame).
// thrust normalised to [0, 1] where 1.0 = full collective.
struct ControlCommand
{
    double roll_rate  = 0.0;
    double pitch_rate = 0.0;
    double yaw_rate   = 0.0;
    double thrust     = 1.0;  // default: hover
    double vx_sp = 0.0;       // velocity setpoint from outer position loop
    double vy_sp = 0.0;
    double vz_sp = 0.0;
};