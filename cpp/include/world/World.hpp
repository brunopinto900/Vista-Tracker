#pragma once

#include <vector>
#include "world/Grid.hpp"
#include "world/Obstacle.hpp"

class World
{
public:
    Grid                  grid;
    std::vector<Obstacle> obstacles;

    bool collision(double x, double y) const
    {
        for (const auto& o : obstacles)
            if (o.contains(x, y))
                return true;
        return false;
    }
};
