#include "config/ConfigLoader.hpp"

#include <yaml-cpp/yaml.h>

Config ConfigLoader::load(
    const std::string& filename)
{
    YAML::Node yaml =
        YAML::LoadFile(filename);

    Config cfg;

    cfg.simulation.dt =
        yaml["simulation"]["dt"].as<double>();

    cfg.simulation.sim_time =
        yaml["simulation"]["sim_time"].as<double>();

    cfg.drone.x =
        yaml["drone"]["x"].as<double>();

    cfg.drone.y =
        yaml["drone"]["y"].as<double>();

    cfg.drone.z =
        yaml["drone"]["z"].as<double>();

    cfg.target.x =
        yaml["target"]["x"].as<double>();

    cfg.target.y =
        yaml["target"]["y"].as<double>();

    cfg.target.z =
        yaml["target"]["z"].as<double>();

    cfg.target.vx =
        yaml["target"]["vx"].as<double>();

    cfg.target.vy =
        yaml["target"]["vy"].as<double>();

    cfg.target.vz =
        yaml["target"]["vz"].as<double>();

    cfg.controller.desired_distance =
        yaml["controller"]["desired_distance"]
            .as<double>();

    cfg.controller.pid.kp =
        yaml["controller"]["pid"]["kp"]
            .as<double>();

    cfg.controller.pid.ki =
        yaml["controller"]["pid"]["ki"]
            .as<double>();

    cfg.controller.pid.kd =
        yaml["controller"]["pid"]["kd"]
            .as<double>();

    return cfg;
}