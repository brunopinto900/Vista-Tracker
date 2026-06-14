#pragma once

#include "models/State.hpp"
#include "world/Obstacle.hpp"
#include <cmath>

class CameraModel
{
public:
    double range = 10.0;

    bool isVisible(const State& drone, const Obstacle& obs) const
    {
        double dx = obs.x - drone.x;
        double dy = obs.y - drone.y;

        return (dx * dx + dy * dy) <= range * range;
    }
};
