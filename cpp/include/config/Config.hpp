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
    // Roll/pitch: differential thrust — high authority
    double wn   = 25.0;  // natural frequency (rad/s)
    double zeta = 0.7;   // damping ratio
    // Yaw: reaction torque imbalance — much lower authority (~6× slower)
    double wn_yaw   = 4.0;
    double zeta_yaw = 0.7;
};

struct ControllerConfig
{
    double kp               = 1.0;
    double ki               = 0.0;
    double kd               = 0.0;
    double desired_distance = 4.0;
    double attitude_kp      = 5.0;  // roll/pitch inner loop (rad/s per rad)
    double yaw_kp           = 0.3;  // yaw inner loop (rad/s per rad) — slower plant
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
