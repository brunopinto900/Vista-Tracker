#pragma once

#include "sim/ISimulator.hpp"
#include "sim_impl/WaypointFollower.hpp"
#include "world/World.hpp"

class KinematicSim : public ISimulator
{
public:
    KinematicSim(const State&            drone,
                 const TargetTrajectory& traj,
                 const World&            world);

    void update(const ControlCommand& cmd, double dt) override;

    State       getDroneState()  const override;
    TargetState getTargetTruth() const override;
    SensorData  getSensors()     const override;

private:
    State            drone_;
    TargetState      target_;
    World            world_;
    WaypointFollower follower_;
};
