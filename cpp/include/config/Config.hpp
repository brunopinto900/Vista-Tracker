#pragma once

#include "models/TargetTrajectory.hpp"
#include "world/World.hpp"
#include <string>

struct EstimatorConfig
{
    int         horizon      = 1;
    std::string motion_model = "CV";  // "CV" | "CTRV" | "CA"
};

struct DroneConfig
{
    double tau = 0.1;   // body-rate actuator lag (s) — models PX4 inner-loop response
};

struct ControllerConfig
{
    double kp               = 1.0;
    double ki               = 0.0;
    double kd               = 0.0;
    double desired_distance = 4.0;
    double attitude_kp      = 5.0;  // inner loop: (rad/s) per rad of attitude error
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

    DroneConfig      drone;
    EstimatorConfig  estimator;
    ControllerConfig controller;
    TargetTrajectory trajectory;

    World world;
};
