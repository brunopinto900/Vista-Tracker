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
        if (s["kp"])               cfg.controller.kp               = s["kp"].as<double>();
        if (s["ki"])               cfg.controller.ki               = s["ki"].as<double>();
        if (s["kd"])               cfg.controller.kd               = s["kd"].as<double>();
        if (s["desired_distance"]) cfg.controller.desired_distance = s["desired_distance"].as<double>();
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
