#pragma once

#include "models/TargetState.hpp"
#include "world/World.hpp"

struct ControllerConfig
{
    double kp               = 1.0;
    double ki               = 0.0;
    double kd               = 0.0;
    double desired_distance = 4.0;
};

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

    ControllerConfig controller;

    World world;
};