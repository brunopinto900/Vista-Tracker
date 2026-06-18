#pragma once

#include "sim/ISimulator.hpp"
#include "sim_impl/WaypointFollower.hpp"
#include "world/World.hpp"

class KinematicSim : public ISimulator
{
public:
    KinematicSim(const State&            drone,
                 const TargetTrajectory& traj,
                 const World&            world,
                 double                  wn       = 25.0,  // roll/pitch bandwidth (rad/s)
                 double                  zeta     = 0.7,
                 double                  wn_yaw   = 4.0,   // yaw bandwidth — reaction torque limited
                 double                  zeta_yaw = 0.7);

    void update(const ControlCommand& cmd, double dt) override;

    State       getDroneState()  const override;
    TargetState getTargetTruth() const override;
    SensorData  getSensors()     const override;

private:
    State            drone_;
    double           thrust_actual_ = 1.0;  // initialised at hover
    double           thrust_dot_    = 0.0;
    double           wx_dot_        = 0.0;  // body-rate derivatives for 2nd-order ODE
    double           wy_dot_        = 0.0;
    double           wz_dot_        = 0.0;
    double           wn_;
    double           zeta_;
    double           wn_yaw_;
    double           zeta_yaw_;
    TargetState      target_;
    World            world_;
    WaypointFollower follower_;
};
