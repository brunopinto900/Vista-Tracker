#pragma once

#include "models/TargetState.hpp"
#include "world/World.hpp"

struct Config
{
    struct {
        double x, y, z;
    } drone_init;

    TargetState target_init;

    struct {
        double dt;
        double T;
    } sim;

    World world;
};