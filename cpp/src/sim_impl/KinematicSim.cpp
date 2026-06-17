#include "sim_impl/KinematicSim.hpp"

#include <cmath>

static constexpr double kG = 9.81;

KinematicSim::KinematicSim(const State&            drone,
                             const TargetTrajectory& traj,
                             const World&            world,
                             double                  wn,
                             double                  zeta)
    : drone_(drone)
    , wn_(wn)
    , zeta_(zeta)
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
    // Models the PX4 inner rate-controller response as a second-order system:
    //   ẅ + 2·ζ·wn·ẇ + wn²·w = wn²·w_cmd
    //
    // Equivalent transfer function: W(s)/W_cmd(s) = wn² / (s²+2ζwn·s+wn²)
    //
    // Typical small quad: wn=25 rad/s, zeta=0.7 → 4.3% overshoot, ts≈330 ms.
    const double wn2        = wn_ * wn_;
    const double two_zeta_wn = 2.0 * zeta_ * wn_;

    auto step2 = [&](double w_cmd, double& w, double& w_dot) {
        double w_ddot = wn2 * (w_cmd - w) - two_zeta_wn * w_dot;
        w_dot += w_ddot * dt;
        w     += w_dot  * dt;
    };

    step2(cmd.roll_rate,  drone_.wx, wx_dot_);
    step2(cmd.pitch_rate, drone_.wy, wy_dot_);
    step2(cmd.yaw_rate,   drone_.wz, wz_dot_);
    step2(cmd.thrust,     thrust_actual_, thrust_dot_);

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
