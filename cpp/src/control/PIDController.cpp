#include "control/PIDController.hpp"

#include <cmath>
#include <algorithm>
#include <cstdio>

static constexpr double kG = 9.81;

PIDController::PIDController(double kp_pos, double ki_pos, double kp_vel, double ki_vel,
                             double attitude_kp, double yaw_kp,
                             double max_tilt_rad, double max_thrust,
                             double max_ipos_contribution, double max_ivel_contribution)
    : kp_pos_(kp_pos)
    , ki_pos_(ki_pos)
    , pid_vx_(kp_vel, ki_vel, max_ivel_contribution)
    , pid_vy_(kp_vel, ki_vel, max_ivel_contribution)
    , pid_vz_(kp_vel, ki_vel, max_ivel_contribution)
    , attitude_kp_(attitude_kp)
    , yaw_kp_(yaw_kp)
    , max_tilt_rad_(max_tilt_rad)
    , max_thrust_(max_thrust)
    , max_ipos_contribution_(max_ipos_contribution)
    , max_ivel_contribution_(max_ivel_contribution)
{}

ControlCommand PIDController::update(
    const State&     drone,
    const Reference& reference,
    double           dt)
{
    // ── Outer loop: position PI → velocity setpoint (with feedforward) ────────
    const double ex = reference.x - drone.x;
    const double ey = reference.y - drone.y;
    const double ez = reference.z - drone.z;
    ip_x_ += ex * dt;
    ip_y_ += ey * dt;
    ip_z_ += ez * dt;
    // Anti-windup: clamp integral so its velocity contribution stays bounded
    if (ki_pos_ > 1e-9) {
        const double lim = max_ipos_contribution_ / ki_pos_;
        ip_x_ = std::clamp(ip_x_, -lim, lim);
        ip_y_ = std::clamp(ip_y_, -lim, lim);
        ip_z_ = std::clamp(ip_z_, -lim, lim);
    }
    const double vx_sp = kp_pos_ * ex + ki_pos_ * ip_x_ + reference.vx;
    const double vy_sp = kp_pos_ * ey + ki_pos_ * ip_y_ + reference.vy;
    const double vz_sp = kp_pos_ * ez + ki_pos_ * ip_z_ + reference.vz;

    // ── Inner loop x/y: velocity PID → desired horizontal acceleration ──────────
    const double ax_des = pid_vx_.update(vx_sp - drone.vx, dt);
    const double ay_des = pid_vy_.update(vy_sp - drone.vy, dt);

    // Rotate world-frame desired acceleration into body horizontal plane so
    // roll/pitch commands are correct at any yaw angle.
    const double cy = std::cos(drone.yaw);
    const double sy = std::sin(drone.yaw);
    const double ax_body =  ax_des * cy + ay_des * sy;
    const double ay_body = -ax_des * sy + ay_des * cy;

    // Camera aim gets first allocation (up to max_tilt_rad_).
    // Propulsion gets a symmetric budget of (max_tilt_rad_ - cam_pitch) in each direction:
    // braking cannot steal more tilt from the camera than forward flight contributes,
    // so the person stays in frame during deceleration as well as approach.
    // Altitude reference is handled geometrically by the planner (ref.z).
    const double cam_pitch   = std::clamp(reference.camera_pitch, 0.0, max_tilt_rad_);
    const double prop_pitch  = std::atan2(ax_body, kG);
    const double prop_budget = max_tilt_rad_ - cam_pitch;
    const double pitch_des   = cam_pitch + std::clamp(prop_pitch, -prop_budget, prop_budget);

    // ── Inner loop z: velocity PID ────────────────────────────────────────────
    const double az_des = pid_vz_.update(vz_sp - drone.vz, dt);

    const double roll_des  = std::clamp(-std::atan2(ay_body, kG), -max_tilt_rad_, max_tilt_rad_);
    const double thrust    = std::clamp((kG + az_des) / kG, 0.0, max_thrust_);

    // ── Inner loop: attitude error → body rates ───────────────────────────────
    const double roll_rate  = attitude_kp_ * (roll_des  - drone.roll);
    const double pitch_rate = attitude_kp_ * (pitch_des - drone.pitch);

    // Yaw: wrap error to [-π, π] then apply separate yaw_kp (slower plant)
    const double yaw_err = std::remainder(reference.yaw - drone.yaw, 2.0 * M_PI);
    const double yaw_rate = yaw_kp_ * yaw_err;

    return {roll_rate, pitch_rate, yaw_rate, thrust, vx_sp, vy_sp, vz_sp};
}
