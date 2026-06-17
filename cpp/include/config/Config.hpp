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
    // Second-order body-rate response: models PX4 rate controller dynamics.
    // Typical small quad: wn ≈ 25 rad/s, zeta ≈ 0.7 (4.3 % overshoot).
    double wn   = 25.0;  // natural frequency (rad/s)
    double zeta = 0.7;   // damping ratio
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
