#include "control/PIDController.hpp"

#include <cmath>
#include <algorithm>

static constexpr double kG        = 9.81;
static constexpr double kMaxAngle = 0.5;    // rad (~28°) — tilt limit
static constexpr double kMaxThrust = 2.0;   // normalised

PIDController::PIDController(double kp, double ki, double kd,
                             double attitude_kp, double yaw_kp)
    : pid_x_(kp, ki, kd)
    , pid_y_(kp, ki, kd)
    , pid_z_(kp, ki, kd)
    , attitude_kp_(attitude_kp)
    , yaw_kp_(yaw_kp)
{}

ControlCommand PIDController::update(
    const State&     drone,
    const Reference& reference,
    double           dt)
{
    // ── Outer loop: position error → desired world-frame acceleration ─────────
    const double ax_des = pid_x_.update(reference.x - drone.x, dt);
    const double ay_des = pid_y_.update(reference.y - drone.y, dt);
    const double az_des = pid_z_.update(reference.z - drone.z, dt);

    // ── Attitude setpoints from desired horizontal acceleration ───────────────
    // Rotate world-frame desired acceleration into body horizontal plane so
    // roll/pitch commands are correct at any yaw angle.
    const double cy = std::cos(drone.yaw);
    const double sy = std::sin(drone.yaw);
    const double ax_body =  ax_des * cy + ay_des * sy;
    const double ay_body = -ax_des * sy + ay_des * cy;

    const double pitch_des = std::clamp(std::atan2(ax_body, kG), -kMaxAngle, kMaxAngle);
    const double roll_des  = std::clamp(-std::atan2(ay_body, kG), -kMaxAngle, kMaxAngle);
    const double thrust    = std::clamp((kG + az_des) / kG, 0.0, kMaxThrust);

    // ── Inner loop: attitude error → body rates ───────────────────────────────
    const double roll_rate  = attitude_kp_ * (roll_des  - drone.roll);
    const double pitch_rate = attitude_kp_ * (pitch_des - drone.pitch);

    // Yaw: wrap error to [-π, π] then apply separate yaw_kp (slower plant)
    const double yaw_err = std::remainder(reference.yaw - drone.yaw, 2.0 * M_PI);
    const double yaw_rate = yaw_kp_ * yaw_err;

    return {roll_rate, pitch_rate, yaw_rate, thrust};
}
