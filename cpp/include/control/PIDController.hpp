#pragma once

#include "control/IController.hpp"

class PIDController : public IController
{
private:
    double kp = 1.0;

public:
    ControlCommand update(
        const State& drone,
        const State& desired,
        double dt) override
    {
        ControlCommand cmd;

        cmd.vx = kp * (desired.x - drone.x);
        cmd.vy = kp * (desired.y - drone.y);
        cmd.vz = kp * (desired.z - drone.z);

        return cmd;
    }
};
