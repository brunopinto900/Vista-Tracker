#include "sim_impl/KinematicSim.hpp"

#include <cmath>

static constexpr double kG = 9.81;  // m/s²

KinematicSim::KinematicSim(const State&            drone,
                             const TargetTrajectory& traj,
                             const World&            world,
                             double                  tau)
    : drone_(drone)
    , tau_(tau)
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
    // ── First-order lag on body rates and thrust ──────────────────────────────
    // Models the PX4 inner-loop response: commanded rates are not achieved
    // instantaneously — they ramp with time constant tau_.
    const double alpha = dt / tau_;
    drone_.wx    += (cmd.roll_rate  - drone_.wx)    * alpha;
    drone_.wy    += (cmd.pitch_rate - drone_.wy)    * alpha;
    drone_.wz    += (cmd.yaw_rate   - drone_.wz)    * alpha;
    thrust_actual_ += (cmd.thrust  - thrust_actual_) * alpha;

    // ── Attitude integration (ZYX Euler) ──────────────────────────────────────
    drone_.roll  += drone_.wx * dt;
    drone_.pitch += drone_.wy * dt;
    drone_.yaw   += drone_.wz * dt;

    // ── World-frame acceleration ───────────────────────────────────────────────
    // The body z-axis (thrust direction) in world frame via ZYX rotation:
    //   R * [0, 0, 1]  =  [R02, R12, R22]
    // where R = Rz(yaw) * Ry(pitch) * Rx(roll).
    const double cr = std::cos(drone_.roll),  sr = std::sin(drone_.roll);
    const double cp = std::cos(drone_.pitch), sp = std::sin(drone_.pitch);
    const double cy = std::cos(drone_.yaw),   sy = std::sin(drone_.yaw);

    const double T = thrust_actual_ * kG;  // thrust magnitude (m/s²)

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
