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

struct PlannerConfig
{
    // Waypoint sequencer
    double standoff_dist    = 4.0;   // desired distance from target (m)
    double wp_reach_thresh  = 0.5;   // advance waypoint when within this radius (m)
    double replan_goal_dist = 1.5;   // replan when goal shifts more than this (m)
    // RRT
    double step_size        = 0.8;   // extension step (m)
    double goal_bias        = 0.10;  // fraction of samples directed at goal
    double safety_margin    = 0.3;   // min ESDF clearance (m)
    double edge_check_res   = 0.1;   // collision-check resolution along edge (m)
    int    max_iter         = 4000;  // max RRT iterations per call
    double goal_tol         = 0.5;   // goal-reached radius (m)
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

    PlannerConfig planner;
    World         world;
};
