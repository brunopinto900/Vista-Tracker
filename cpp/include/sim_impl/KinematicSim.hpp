#pragma once

#include "sim/ISimulator.hpp"
#include "world/World.hpp"

class KinematicSim : public ISimulator
{
public:
    KinematicSim(const State& d, const TargetState& t, const World& w)
        : drone_(d), target_(t), world_(w) {}

    void update(const ControlCommand& cmd, double dt) override
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

    State       getDroneState()  const override { return drone_;  }
    TargetState getTargetTruth() const override { return target_; }
    SensorData  getSensors()     const override { return {};       }

private:
    State       drone_;
    TargetState target_;
    World       world_;
};
