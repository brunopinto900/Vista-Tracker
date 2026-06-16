#include "sim_impl/KinematicSim.hpp"

KinematicSim::KinematicSim(const State&            drone,
                            const TargetTrajectory& traj,
                            const World&            world)
    : drone_(drone)
    , world_(world)
    , follower_(traj, [&] {
        TargetState t;
        t.x = traj.waypoints[0].x;
        t.y = traj.waypoints[0].y;
        t.z = traj.waypoints[0].z;
        return t;
    }())
{
    target_ = follower_.step(0.0);
}

void KinematicSim::update(const ControlCommand& cmd, double dt)
{
    drone_.x += cmd.vx * dt;
    drone_.y += cmd.vy * dt;
    drone_.z += cmd.vz * dt;

    drone_.vx = cmd.vx;
    drone_.vy = cmd.vy;
    drone_.vz = cmd.vz;

    target_ = follower_.step(dt);
}

State       KinematicSim::getDroneState()  const { return drone_;  }
TargetState KinematicSim::getTargetTruth() const { return target_; }
SensorData  KinematicSim::getSensors()     const { return {};      }
