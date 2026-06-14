#include "config/ConfigLoader.hpp"

#include <yaml-cpp/yaml.h>

Config ConfigLoader::load(
    const std::string& path)
{
    YAML::Node root = YAML::LoadFile(path);

    Config cfg;

    cfg.drone_init.x = root["drone_init"]["x"].as<double>();
    cfg.drone_init.y = root["drone_init"]["y"].as<double>();
    cfg.drone_init.z = root["drone_init"]["z"].as<double>();

    cfg.target_init.x  = root["target_init"]["x"].as<double>();
    cfg.target_init.y  = root["target_init"]["y"].as<double>();
    cfg.target_init.z  = root["target_init"]["z"].as<double>();
    cfg.target_init.vx = root["target_init"]["vx"].as<double>();
    cfg.target_init.vy = root["target_init"]["vy"].as<double>();
    cfg.target_init.vz = root["target_init"]["vz"].as<double>();

    cfg.sim.dt = root["sim"]["dt"].as<double>();
    cfg.sim.T  = root["sim"]["T"].as<double>();

    for (auto o : root["world"]["obstacles"])
    {
        cfg.world.obstacles.push_back({
            o["x"].as<double>(),
            o["y"].as<double>(),
            o["size"].as<double>()
        });
    }

    return cfg;
}