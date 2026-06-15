#pragma once

#include "control/IController.hpp"
#include "control/PID.hpp"

class PIDController : public IController
{
public:
    PIDController(double kp, double ki, double kd)
        : pid_x_(kp, ki, kd),
          pid_y_(kp, ki, kd),
          pid_z_(kp, ki, kd) {}

    ControlCommand update(
        const State& drone,
        const Reference& reference,
        double dt) override
    {
        ControlCommand cmd;
        cmd.vx = pid_x_.update(reference.x - drone.x, dt);
        cmd.vy = pid_y_.update(reference.y - drone.y, dt);
        cmd.vz = pid_z_.update(reference.z - drone.z, dt);
        return cmd;
    }

private:
    PID pid_x_;
    PID pid_y_;
    PID pid_z_;
};
