#include "control/PIDController.hpp"

#include <cmath>
#include <algorithm>
#include <cstdio>

static constexpr double kG              = 9.81;
static constexpr double kMaxAngle       = 0.5;   // rad (~28°) — tilt limit
static constexpr double kMaxThrust      = 2.0;   // normalised
static constexpr double kMaxIposCont    = 1.0;   // max m/s contribution from position integral
static constexpr double kMaxIvelCont    = 4.0;   // max m/s² contribution from velocity integral
static constexpr double kCamPitchZGain  = 3.0;   // (m/s)/rad — climb rate added per radian of pitch deficit

PIDController::PIDController(double kp_pos, double ki_pos, double kp_vel, double ki_vel,
                             double attitude_kp, double yaw_kp)
    : kp_pos_(kp_pos)
    , ki_pos_(ki_pos)
    , pid_vx_(kp_vel, ki_vel)
    , pid_vy_(kp_vel, ki_vel)
    , pid_vz_(kp_vel, ki_vel)
    , attitude_kp_(attitude_kp)
    , yaw_kp_(yaw_kp)
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
        const double lim = kMaxIposCont / ki_pos_;
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

    // Camera aim takes full priority: it gets its allocation first (up to kMaxAngle).
    // Propulsion pitch is then clamped to whatever budget remains so the total
    // stays in [-kMaxAngle, kMaxAngle] without silently stealing from the camera aim.
    const double cam_pitch  = std::clamp(reference.camera_pitch, -kMaxAngle, kMaxAngle);
    const double prop_pitch = std::atan2(ax_body, kG);
    const double pitch_des  = cam_pitch + std::clamp(prop_pitch,
                                                      -kMaxAngle - cam_pitch,
                                                       kMaxAngle - cam_pitch);

    // When prop_pitch fights cam_pitch (e.g. braking nose-up vs camera nose-down),
    // climb to steepen the natural camera angle and absorb the conflict.
    const double pitch_deficit  = std::min(0.0, cam_pitch + prop_pitch);
    const double vz_sp_adjusted = vz_sp - kCamPitchZGain * pitch_deficit;

    // ── Inner loop z: velocity PID with camera-conflict feedforward ───────────
    const double az_des = pid_vz_.update(vz_sp_adjusted - drone.vz, dt);

    const double vx_body = drone.vx * cy + drone.vy * sy;
    //std::printf("[PID] cam_pitch=%.4f  prop_pitch=%.4f  pitch_des=%.4f\n",
    //            cam_pitch, prop_pitch, pitch_des);
    //std::printf("[PID] vx_body=%.4f (%s)  pitch_deficit=%.4f  vz_ff=%.4f\n",
    //            vx_body, vx_body >= 0.0 ? "forward" : "backward",
    //            pitch_deficit, -kCamPitchZGain * pitch_deficit);
    const double roll_des  = std::clamp(-std::atan2(ay_body, kG), -kMaxAngle, kMaxAngle);
    const double thrust    = std::clamp((kG + az_des) / kG, 0.0, kMaxThrust);

    // ── Inner loop: attitude error → body rates ───────────────────────────────
    const double roll_rate  = attitude_kp_ * (roll_des  - drone.roll);
    const double pitch_rate = attitude_kp_ * (pitch_des - drone.pitch);

    // Yaw: wrap error to [-π, π] then apply separate yaw_kp (slower plant)
    const double yaw_err = std::remainder(reference.yaw - drone.yaw, 2.0 * M_PI);
    const double yaw_rate = yaw_kp_ * yaw_err;

    return {roll_rate, pitch_rate, yaw_rate, thrust, vx_sp, vy_sp, vz_sp};
}
