#include <fstream>
#include <iostream>
#include <string>
#include <filesystem>

#include "config/ConfigLoader.hpp"
#include "sim_impl/KinematicSim.hpp"
#include "perception/GroundTruthPerception.hpp"
#include "estimation/PerfectEstimator.hpp"
#include "mapping/FakeESDFMap.hpp"
#include "planning/SimplePlanner.hpp"
#include "control/PIDController.hpp"

namespace fs = std::filesystem;

static const std::string kDefaultConfig   = "../../config/config.yaml";
static const std::string kScenariosDir    = "../../config/scenarios";

static void listScenarios()
{
    std::cout << "Available scenarios:\n";
    for (const auto& entry : fs::directory_iterator(kScenariosDir))
        if (entry.path().extension() == ".yaml")
            std::cout << "  " << entry.path().stem().string() << "\n";
}

static std::string resolveConfig(const std::string& arg)
{
    // Full or relative path that exists → use as-is
    if (fs::exists(arg))
        return arg;

    // Bare name → look in scenarios dir
    std::string candidate = kScenariosDir + "/" + arg + ".yaml";
    if (fs::exists(candidate))
        return candidate;

    std::cerr << "error: scenario '" << arg << "' not found.\n"
              << "       tried: " << candidate << "\n";
    std::exit(1);
}

int main(int argc, char* argv[])
{
    if (argc > 1 && std::string(argv[1]) == "--list")
    {
        listScenarios();
        return 0;
    }

    const std::string config_path = (argc > 1)
        ? resolveConfig(argv[1])
        : kDefaultConfig;

    Config cfg = ConfigLoader::load(config_path);

    std::cout << "[config] scenario        " << config_path << "\n"
              << "[config] drone_init      x=" << cfg.drone_init.x
              << " y=" << cfg.drone_init.y
              << " z=" << cfg.drone_init.z << "\n"
              << "[config] trajectory      type=" << cfg.trajectory.type
              << " waypoints=" << cfg.trajectory.waypoints.size()
              << " max_speed=" << cfg.trajectory.max_speed
              << " max_accel=" << cfg.trajectory.max_accel
              << " max_lateral_accel=" << cfg.trajectory.max_lateral_accel
              << " loop=" << (cfg.trajectory.loop ? "true" : "false") << "\n"
              << "[config] sim             dt=" << cfg.sim.dt
              << " T=" << cfg.sim.T << "\n"
              << "[config] estimator       horizon=" << cfg.estimator.horizon
              << " motion_model=" << cfg.estimator.motion_model << "\n"
              << "[config] controller      kp=" << cfg.controller.kp
              << " ki=" << cfg.controller.ki
              << " kd=" << cfg.controller.kd
              << " desired_distance=" << cfg.controller.desired_distance << "\n"
              << "[config] world.grid      x=[" << cfg.world.grid.x_min
              << ", " << cfg.world.grid.x_max << "]"
              << " y=[" << cfg.world.grid.y_min
              << ", " << cfg.world.grid.y_max << "]\n"
              << "[config] world.obstacles " << cfg.world.obstacles.size() << " loaded\n";

    State drone{};
    drone.x = cfg.drone_init.x;
    drone.y = cfg.drone_init.y;
    drone.z = cfg.drone_init.z;

    KinematicSim          sim(drone, cfg.trajectory, cfg.world);
    GroundTruthPerception perception(sim);
    PerfectEstimator      estimator(cfg.estimator.horizon);
    FakeESDFMap           esdf(cfg.world);
    SimplePlanner         planner(cfg.controller.desired_distance);
    PIDController         controller(cfg.controller.kp,
                                     cfg.controller.ki,
                                     cfg.controller.kd);

    std::ofstream file("../../data/log.csv");

    file << "t,"
         << "target_x,target_y,target_z,"
         << "drone_x,drone_y,drone_z,"
         << "drone_vx,drone_vy,drone_vz,"
         << "target_vx,target_vy,target_vz,"
         << "vx_cmd,vy_cmd,vz_cmd\n";

    for (double t = 0.0; t < cfg.sim.T; t += cfg.sim.dt)
    {
        State       d  = sim.getDroneState();
        TargetState tr = sim.getTargetTruth();

        Detection      det = perception.update();
        TargetEstimate est = estimator.update(det, cfg.sim.dt);
        Reference      ref = planner.update(d, est, esdf);
        ControlCommand cmd = controller.update(d, ref, cfg.sim.dt);

        sim.update(cmd, cfg.sim.dt);

        file << t         << ","
             << tr.x      << "," << tr.y  << "," << tr.z  << ","
             << d.x       << "," << d.y   << "," << d.z   << ","
             << d.vx      << "," << d.vy  << "," << d.vz  << ","
             << tr.vx     << "," << tr.vy << "," << tr.vz << ","
             << cmd.vx    << "," << cmd.vy << "," << cmd.vz
             << "\n";
    }

    return 0;
}
