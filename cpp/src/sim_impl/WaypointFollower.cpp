#include "sim_impl/WaypointFollower.hpp"

#include <cmath>
#include <algorithm>

static constexpr double kReachThreshold = 0.1;  // metres
static constexpr double kMinSpeedForTurn = 0.05; // avoid div/0

WaypointFollower::WaypointFollower(const TargetTrajectory& traj,
                                   const TargetState&      init)
    : traj_(traj)
    , state_(init)
    , heading_(0.0)
{}

TargetState WaypointFollower::step(double dt)
{
    if (done_ || traj_.waypoints.empty())
        return state_;

    const Waypoint& wp = traj_.waypoints[idx_];

    // ── Hold at waypoint ─────────────────────────────────────────────────────
    if (holding_)
    {
        hold_elapsed_ += dt;
        state_.vx = state_.vy = state_.vz = 0.0;

        if (hold_elapsed_ >= wp.hold)
        {
            holding_      = false;
            hold_elapsed_ = 0.0;
            advance();
        }
        return state_;
    }

    // ── Direction to waypoint ────────────────────────────────────────────────
    double dx   = wp.x - state_.x;
    double dy   = wp.y - state_.y;
    double dz   = wp.z - state_.z;
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (dist < kReachThreshold)
    {
        if (wp.hold > 0.0)
            holding_ = true;
        else
            advance();
        return state_;
    }

    // ── Trapezoidal longitudinal profile ─────────────────────────────────────
    double cur_speed = std::sqrt(state_.vx * state_.vx +
                                 state_.vy * state_.vy +
                                 state_.vz * state_.vz);

    double d_brake  = (cur_speed * cur_speed) /
                      (2.0 * traj_.max_accel + 1e-9);
    double v_target = (d_brake >= dist) ? 0.0
                    : std::min(wp.speed, traj_.max_speed);

    double dv     = v_target - cur_speed;
    double dv_max = traj_.max_accel * dt;
    dv            = std::clamp(dv, -dv_max, dv_max);

    double new_speed = std::max(0.0, cur_speed + dv);

    // ── Smooth heading (lateral acceleration constraint) ──────────────────────
    // max yaw rate = max_lateral_accel / speed
    // → large lateral_accel + low speed → person can spin in place
    // → small lateral_accel + high speed → car needs wide arc
    double desired_heading = std::atan2(dy, dx);

    double heading_err = desired_heading - heading_;
    // Wrap to [-π, π]
    while (heading_err >  M_PI) heading_err -= 2.0 * M_PI;
    while (heading_err < -M_PI) heading_err += 2.0 * M_PI;

    double eff_speed    = std::max(new_speed, kMinSpeedForTurn);
    double max_yaw_rate = traj_.max_lateral_accel / eff_speed;
    double dheading_max = max_yaw_rate * dt;

    heading_ += std::clamp(heading_err, -dheading_max, dheading_max);

    // ── Integrate position ────────────────────────────────────────────────────
    // xy follows the (smoothed) heading; z interpolates directly
    double uz = dz / dist;

    state_.x  += std::cos(heading_) * new_speed * dt;
    state_.y  += std::sin(heading_) * new_speed * dt;
    state_.z  += uz * new_speed * dt;

    state_.vx  = std::cos(heading_) * new_speed;
    state_.vy  = std::sin(heading_) * new_speed;
    state_.vz  = uz * new_speed;

    return state_;
}

void WaypointFollower::advance()
{
    ++idx_;
    if (idx_ >= static_cast<int>(traj_.waypoints.size()))
    {
        if (traj_.loop)
            idx_ = 0;
        else
            done_ = true;
    }
}
