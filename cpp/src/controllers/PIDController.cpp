#include "controllers/PIDController.hpp"

#include <cmath>

PIDController::PIDController(
    double desired_distance,
    double kp,
    double ki,
    double kd)
    :
    desired_distance_(desired_distance),
    pid_x_(kp, ki, kd),
    pid_y_(kp, ki, kd),
    pid_z_(kp, ki, kd)
{
}

ControlCommand PIDController::update(
    const State& drone,
    const TargetState& target,
    double dt)
{
    ControlCommand cmd;

    double heading =
        std::atan2(
            target.vy,
            target.vx);

    double desired_x =
        target.x -
        desired_distance_ *
        std::cos(heading);

    double desired_y =
        target.y -
        desired_distance_ *
        std::sin(heading);

    double desired_z =
        target.z;

    double ex =
        desired_x - drone.x;

    double ey =
        desired_y - drone.y;

    double ez =
        desired_z - drone.z;

    cmd.vx_cmd =
        pid_x_.update(ex, dt);

    cmd.vy_cmd =
        pid_y_.update(ey, dt);

    cmd.vz_cmd =
        pid_z_.update(ez, dt);

    return cmd;
}