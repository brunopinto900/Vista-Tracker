#include "control/PIDController.hpp"

PIDController::PIDController(double kp, double ki, double kd)
    : pid_x_(kp, ki, kd),
      pid_y_(kp, ki, kd),
      pid_z_(kp, ki, kd) {}

ControlCommand PIDController::update(
    const State&     drone,
    const Reference& reference,
    double           dt)
{
    ControlCommand cmd;
    cmd.vx = pid_x_.update(reference.x - drone.x, dt);
    cmd.vy = pid_y_.update(reference.y - drone.y, dt);
    cmd.vz = pid_z_.update(reference.z - drone.z, dt);
    return cmd;
}
