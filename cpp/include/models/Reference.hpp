#pragma once

#include "models/State.hpp"
#include <vector>

struct Reference
{
    double x   = 0.0;
    double y   = 0.0;
    double z   = 0.0;
    double vx  = 0.0;  // velocity feedforward (target velocity)
    double vy  = 0.0;
    double vz  = 0.0;
    double yaw          = 0.0;  // desired heading (rad) — camera-facing the target in azimuth
    double camera_pitch = 0.0;  // desired camera pitch (rad, positive = looking down) toward track_z

    std::vector<State> trajectory;  // optional MPC horizon

    bool   deadlock_active = false;  // ideal standoff blocked; using angle-shifted proxy
    double deadlock_angle  = 0.0;    // chosen viewpoint bearing angle [rad]
};
