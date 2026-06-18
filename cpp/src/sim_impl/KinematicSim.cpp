#include "sim_impl/KinematicSim.hpp"

#include <cmath>

static constexpr double kG = 9.81;

KinematicSim::KinematicSim(const State&            drone,
                             const TargetTrajectory& traj,
                             const World&            world,
                             double                  wn,
                             double                  zeta,
                             double                  wn_yaw,
                             double                  zeta_yaw)
    : drone_(drone)
    , wn_(wn)
    , zeta_(zeta)
    , wn_yaw_(wn_yaw)
    , zeta_yaw_(zeta_yaw)
    , world_(world)
    , follower_(traj, [&] {
        TargetState t;
        t.x = traj.waypoints.empty() ? 0.0 : traj.waypoints[0].x;
        t.y = traj.waypoints.empty() ? 0.0 : traj.waypoints[0].y;
        t.z = traj.waypoints.empty() ? 0.0 : traj.waypoints[0].z;
        return t;
    }())
{
    target_ = follower_.step(0.0);
}

void KinematicSim::update(const ControlCommand& cmd, double dt)
{
    // ── Second-order body-rate dynamics ───────────────────────────────────────
    // Roll/pitch: differential thrust — wn=25 rad/s, wn·dt=1.25 > stability
    // boundary, so we sub-step at dt_inner = dt/50 (wn·dt_inner = 0.025).
    // Yaw: reaction torque — wn=4 rad/s, wn·dt=0.2, within stability boundary
    // but sub-stepped at the same rate for consistency.
    static constexpr int kInnerSteps = 50;
    const double dt_inner = dt / kInnerSteps;

    const double wn2_rp  = wn_ * wn_;
    const double damp_rp = 2.0 * zeta_ * wn_;

    const double wn2_yaw  = wn_yaw_ * wn_yaw_;
    const double damp_yaw = 2.0 * zeta_yaw_ * wn_yaw_;

    auto step2_rp = [&](double w_cmd, double& w, double& w_dot) {
        for (int i = 0; i < kInnerSteps; ++i) {
            const double w_ddot = wn2_rp * (w_cmd - w) - damp_rp * w_dot;
            w_dot += w_ddot * dt_inner;
            w     += w_dot  * dt_inner;
        }
    };

    auto step2_yaw = [&](double w_cmd, double& w, double& w_dot) {
        for (int i = 0; i < kInnerSteps; ++i) {
            const double w_ddot = wn2_yaw * (w_cmd - w) - damp_yaw * w_dot;
            w_dot += w_ddot * dt_inner;
            w     += w_dot  * dt_inner;
        }
    };

    step2_rp(cmd.roll_rate,  drone_.wx,       wx_dot_);
    step2_rp(cmd.pitch_rate, drone_.wy,       wy_dot_);
    step2_yaw(cmd.yaw_rate,  drone_.wz,       wz_dot_);
    step2_rp(cmd.thrust,     thrust_actual_,  thrust_dot_);

    // ── Attitude integration (ZYX Euler) ──────────────────────────────────────
    drone_.roll  += drone_.wx * dt;
    drone_.pitch += drone_.wy * dt;
    drone_.yaw   += drone_.wz * dt;

    // ── World-frame acceleration ───────────────────────────────────────────────
    // Body z-axis in world frame via R = Rz(yaw)·Ry(pitch)·Rx(roll):
    //   R·[0,0,1] = [R02, R12, R22]
    const double cr = std::cos(drone_.roll),  sr = std::sin(drone_.roll);
    const double cp = std::cos(drone_.pitch), sp = std::sin(drone_.pitch);
    const double cy = std::cos(drone_.yaw),   sy = std::sin(drone_.yaw);

    const double T = thrust_actual_ * kG;

    const double ax = (cy*sp*cr + sy*sr) * T;
    const double ay = (sy*sp*cr - cy*sr) * T;
    const double az =  cp*cr             * T - kG;

    // ── Velocity and position integration ────────────────────────────────────
    drone_.vx += ax * dt;
    drone_.vy += ay * dt;
    drone_.vz += az * dt;

    drone_.x += drone_.vx * dt;
    drone_.y += drone_.vy * dt;
    drone_.z += drone_.vz * dt;

    target_ = follower_.step(dt);
}

State       KinematicSim::getDroneState()  const { return drone_;  }
TargetState KinematicSim::getTargetTruth() const { return target_; }
SensorData  KinematicSim::getSensors()     const { return {};      }
