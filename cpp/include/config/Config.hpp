#pragma once

#include "models/TargetTrajectory.hpp"
#include "world/World.hpp"
#include <string>

struct EstimatorConfig
{
    int         horizon      = 1;
    std::string motion_model = "CV";  // "CV" | "CTRV" | "CA"
};

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

    struct {
        double dt;
        double T;
    } sim;

    EstimatorConfig  estimator;
    ControllerConfig controller;
    TargetTrajectory trajectory;

    World world;
};
