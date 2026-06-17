#pragma once

#include "sim/ISimulator.hpp"
#include "sim_impl/WaypointFollower.hpp"
#include "world/World.hpp"

class KinematicSim : public ISimulator
{
public:
    // tau: first-order lag time constant (s) for body-rate + thrust channels,
    //      modelling the PX4 inner-loop response delay.
    KinematicSim(const State&            drone,
                 const TargetTrajectory& traj,
                 const World&            world,
                 double                  tau = 0.1);

    void update(const ControlCommand& cmd, double dt) override;

    State       getDroneState()  const override;
    TargetState getTargetTruth() const override;
    SensorData  getSensors()     const override;

private:
    State            drone_;
    double           thrust_actual_ = 1.0;  // lagged thrust, initialised at hover
    double           tau_;
    TargetState      target_;
    World            world_;
    WaypointFollower follower_;
};
