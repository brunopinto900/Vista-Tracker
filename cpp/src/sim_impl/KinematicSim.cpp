#include "sim_impl/KinematicSim.hpp"

KinematicSim::KinematicSim(const State& d, const TargetState& t, const World& w)
    : drone_(d), target_(t), world_(w) {}

void KinematicSim::update(const ControlCommand& cmd, double dt)
{
    drone_.x += cmd.vx * dt;
    drone_.y += cmd.vy * dt;
    drone_.z += cmd.vz * dt;

    drone_.vx = cmd.vx;
    drone_.vy = cmd.vy;
    drone_.vz = cmd.vz;

    target_.x += target_.vx * dt;
    target_.y += target_.vy * dt;
    target_.z += target_.vz * dt;
}

State       KinematicSim::getDroneState()  const { return drone_;  }
TargetState KinematicSim::getTargetTruth() const { return target_; }
SensorData  KinematicSim::getSensors()     const { return {};      }
