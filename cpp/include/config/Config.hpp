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
    double kp_pos           = 1.5;  // outer position loop P gain (pos error → vel setpoint)
    double ki_pos           = 0.0;  // outer position loop I gain (eliminates pos steady-state error)
    double kp_vel           = 5.0;  // inner velocity loop P gain (vel error → acceleration)
    double ki_vel           = 0.2;  // inner velocity loop I gain (eliminates vel steady-state error)
    double desired_distance = 4.0;
    double attitude_kp      = 5.0;  // roll/pitch inner loop (rad/s per rad)
    double yaw_kp           = 0.3;  // yaw inner loop (rad/s per rad) — slower plant
};

struct TrackingCameraConfig
{
    double fov_deg  = 60.0;  // horizontal full FOV (degrees, ±30°)
    double vfov_deg = 30.0;  // vertical half-angle (degrees, ±30°)
};

struct TargetConfig
{
    double height  = 1.80;  // m — total person height
    double width   = 0.50;  // m — shoulder width (physical extent)
    double track_z = 1.40;  // m — camera aim point (upper back / head level)
};

struct PlannerConfig
{
    // Waypoint sequencer
    double standoff_dist    = 4.0;   // computed at runtime from FOV bounding-box geometry
    double wp_reach_thresh  = 0.5;   // advance waypoint when within this radius (m)
    double replan_goal_dist = 1.5;   // replan when goal shifts more than this (m)
    // Visibility-aware altitude
    double theta_des_deg    = 20.0;  // desired viewing angle of target below horizon (deg)
    double theta_safe_deg   =  5.0;  // FOV safety margin (deg)
    double min_z            = 2.0;   // altitude floor (m) — safety clearance above person
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

    TrackingCameraConfig tracking_camera;
    TargetConfig         target;
    PlannerConfig        planner;
    World         world;
};
