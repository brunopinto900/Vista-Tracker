#include "config/ConfigLoader.hpp"

#include <yaml-cpp/yaml.h>
#include <filesystem>

namespace fs = std::filesystem;

// ── public ───────────────────────────────────────────────────────────────────

Config ConfigLoader::load(const std::string& path)
{
    YAML::Node node = YAML::LoadFile(path);

    Config cfg;

    // If scenario declares a base, load it first then apply overrides on top
    if (node["base"])
    {
        fs::path base = fs::path(path).parent_path()
                        / node["base"].as<std::string>();
        YAML::Node base_node = YAML::LoadFile(base.string());
        applyNode(cfg, base_node);
    }

    applyNode(cfg, node);
    return cfg;
}

// ── private ──────────────────────────────────────────────────────────────────

void ConfigLoader::applyNode(Config& cfg, const YAML::Node& n)
{
    if (auto s = n["drone_init"])
    {
        if (s["x"]) cfg.drone_init.x = s["x"].as<double>();
        if (s["y"]) cfg.drone_init.y = s["y"].as<double>();
        if (s["z"]) cfg.drone_init.z = s["z"].as<double>();
    }

    if (auto s = n["drone"])
    {
        if (s["wn"])       cfg.drone.wn       = s["wn"].as<double>();
        if (s["zeta"])     cfg.drone.zeta     = s["zeta"].as<double>();
        if (s["wn_yaw"])   cfg.drone.wn_yaw   = s["wn_yaw"].as<double>();
        if (s["zeta_yaw"]) cfg.drone.zeta_yaw = s["zeta_yaw"].as<double>();
    }

    if (auto s = n["sim"])
    {
        if (s["dt"]) cfg.sim.dt = s["dt"].as<double>();
        if (s["T"])  cfg.sim.T  = s["T"].as<double>();
    }

    if (auto s = n["estimator"])
    {
        if (s["horizon"])      cfg.estimator.horizon      = s["horizon"].as<int>();
        if (s["motion_model"]) cfg.estimator.motion_model = s["motion_model"].as<std::string>();
    }

    if (auto s = n["controller"])
    {
        if (s["kp_pos"])           cfg.controller.kp_pos           = s["kp_pos"].as<double>();
        if (s["ki_pos"])           cfg.controller.ki_pos           = s["ki_pos"].as<double>();
        if (s["kp_vel"])           cfg.controller.kp_vel           = s["kp_vel"].as<double>();
        if (s["ki_vel"])           cfg.controller.ki_vel           = s["ki_vel"].as<double>();
        if (s["desired_distance"]) cfg.controller.desired_distance = s["desired_distance"].as<double>();
        if (s["attitude_kp"])      cfg.controller.attitude_kp      = s["attitude_kp"].as<double>();
        if (s["yaw_kp"])           cfg.controller.yaw_kp           = s["yaw_kp"].as<double>();
    }

    if (auto s = n["tracking_camera"])
    {
        if (s["fov"])  cfg.tracking_camera.fov_deg  = s["fov"].as<double>();
        if (s["vfov"]) cfg.tracking_camera.vfov_deg = s["vfov"].as<double>();
    }

    if (auto s = n["target"])
    {
        if (s["height"])  cfg.target.height  = s["height"].as<double>();
        if (s["width"])   cfg.target.width    = s["width"].as<double>();
        if (s["track_z"]) cfg.target.track_z  = s["track_z"].as<double>();
    }

    if (auto s = n["planner"])
    {
        if (s["standoff_dist"])    cfg.planner.standoff_dist    = s["standoff_dist"].as<double>();
        if (s["wp_reach_thresh"])  cfg.planner.wp_reach_thresh  = s["wp_reach_thresh"].as<double>();
        if (s["replan_goal_dist"]) cfg.planner.replan_goal_dist = s["replan_goal_dist"].as<double>();
        if (auto r = s["rrt"])
        {
            if (r["step_size"])      cfg.planner.step_size      = r["step_size"].as<double>();
            if (r["goal_bias"])      cfg.planner.goal_bias      = r["goal_bias"].as<double>();
            if (r["safety_margin"])  cfg.planner.safety_margin  = r["safety_margin"].as<double>();
            if (r["edge_check_res"]) cfg.planner.edge_check_res = r["edge_check_res"].as<double>();
            if (r["max_iter"])       cfg.planner.max_iter       = r["max_iter"].as<int>();
            if (r["goal_tol"])       cfg.planner.goal_tol       = r["goal_tol"].as<double>();
        }
    }

    // Trajectory is replaced entirely when present (partial waypoint merge makes no sense)
    if (auto s = n["target_trajectory"])
    {
        if (s["type"])              cfg.trajectory.type              = s["type"].as<std::string>();
        if (s["max_accel"])         cfg.trajectory.max_accel         = s["max_accel"].as<double>();
        if (s["max_speed"])         cfg.trajectory.max_speed         = s["max_speed"].as<double>();
        if (s["max_lateral_accel"]) cfg.trajectory.max_lateral_accel = s["max_lateral_accel"].as<double>();
        if (s["loop"])              cfg.trajectory.loop              = s["loop"].as<bool>();

        if (s["waypoints"])
        {
            cfg.trajectory.waypoints.clear();
            for (auto wp : s["waypoints"])
            {
                Waypoint w;
                w.x     = wp["pos"][0].as<double>();
                w.y     = wp["pos"][1].as<double>();
                w.z     = wp["pos"][2].as<double>();
                w.speed = wp["speed"].as<double>();
                w.hold  = wp["hold"] ? wp["hold"].as<double>() : 0.0;
                cfg.trajectory.waypoints.push_back(w);
            }
        }
    }

    if (auto s = n["world"])
    {
        if (auto g = s["grid"])
        {
            if (g["x_min"]) cfg.world.grid.x_min = g["x_min"].as<double>();
            if (g["x_max"]) cfg.world.grid.x_max = g["x_max"].as<double>();
            if (g["y_min"]) cfg.world.grid.y_min = g["y_min"].as<double>();
            if (g["y_max"]) cfg.world.grid.y_max = g["y_max"].as<double>();
        }

        if (s["obstacles"])
        {
            cfg.world.obstacles.clear();
            for (auto o : s["obstacles"])
                cfg.world.obstacles.push_back({
                    o["x"].as<double>(),
                    o["y"].as<double>(),
                    o["size"].as<double>()
                });
        }
    }
}
