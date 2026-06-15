#pragma once

#include "models/State.hpp"
#include <vector>

struct Reference
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    std::vector<State> trajectory;  // optional MPC horizon
};
