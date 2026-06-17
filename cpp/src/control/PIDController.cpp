#include "control/PIDController.hpp"

#include <cmath>
#include <algorithm>

static constexpr double kG        = 9.81;
static constexpr double kMaxAngle = 0.5;    // rad (~28°) — tilt limit
static constexpr double kMaxThrust = 2.0;   // normalised

PIDController::PIDController(double kp, double ki, double kd, double attitude_kp)
    : pid_x_(kp, ki, kd)
    , pid_y_(kp, ki, kd)
    , pid_z_(kp, ki, kd)
    , attitude_kp_(attitude_kp)
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
    // Small-angle: pitch ≈ ax/g, roll ≈ -ay/g (ENU, x-forward, y-left).
    const double pitch_des = std::clamp(std::atan2(ax_des, kG), -kMaxAngle, kMaxAngle);
    const double roll_des  = std::clamp(-std::atan2(ay_des, kG), -kMaxAngle, kMaxAngle);
    const double thrust    = std::clamp((kG + az_des) / kG, 0.0, kMaxThrust);

    // ── Inner loop: attitude error → body rates ───────────────────────────────
    const double roll_rate  = attitude_kp_ * (roll_des  - drone.roll);
    const double pitch_rate = attitude_kp_ * (pitch_des - drone.pitch);
    const double yaw_rate   = 0.0;  // yaw held fixed; MPC will control this

    return {roll_rate, pitch_rate, yaw_rate, thrust};
}
