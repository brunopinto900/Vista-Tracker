#pragma once

#include "models/State.hpp"
#include <vector>

struct Reference
{
    double x   = 0.0;
    double y   = 0.0;
    double z   = 0.0;
    double yaw = 0.0;  // desired heading (rad) — camera-facing the target

    std::vector<State> trajectory;  // optional MPC horizon
};
