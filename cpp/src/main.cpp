#include <fstream>
#include <iostream>
#include <string>
#include <filesystem>

#include "config/ConfigLoader.hpp"
#include "sim_impl/KinematicSim.hpp"
#include "perception/GroundTruthPerception.hpp"
#include "estimation/PerfectEstimator.hpp"
#include "mapping/FakeESDFMap.hpp"
#include "planning/RRTPIDPlanner.hpp"
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
              << "[config] planner         standoff=" << cfg.planner.standoff_dist
              << " wp_thresh=" << cfg.planner.wp_reach_thresh
              << " replan_dist=" << cfg.planner.replan_goal_dist << "\n"
              << "[config] planner.rrt     step=" << cfg.planner.step_size
              << " bias=" << cfg.planner.goal_bias
              << " margin=" << cfg.planner.safety_margin
              << " max_iter=" << cfg.planner.max_iter << "\n"
              << "[config] world.grid      x=[" << cfg.world.grid.x_min
              << ", " << cfg.world.grid.x_max << "]"
              << " y=[" << cfg.world.grid.y_min
              << ", " << cfg.world.grid.y_max << "]\n"
              << "[config] world.obstacles " << cfg.world.obstacles.size() << " loaded\n";

    State drone{};
    drone.x = cfg.drone_init.x;
    drone.y = cfg.drone_init.y;
    drone.z = cfg.drone_init.z;

    KinematicSim          sim(drone, cfg.trajectory, cfg.world,
                              cfg.drone.wn, cfg.drone.zeta,
                              cfg.drone.wn_yaw, cfg.drone.zeta_yaw);
    GroundTruthPerception perception(sim);
    PerfectEstimator      estimator(cfg.estimator.horizon);
    FakeESDFMap           esdf(cfg.world);

    RRTPIDPlanner::Config planner_cfg;
    planner_cfg.standoff_dist    = cfg.planner.standoff_dist;
    planner_cfg.wp_reach_thresh  = cfg.planner.wp_reach_thresh;
    planner_cfg.replan_goal_dist = cfg.planner.replan_goal_dist;
    planner_cfg.z_ref            = cfg.drone_init.z;
    planner_cfg.rrt.step_size      = cfg.planner.step_size;
    planner_cfg.rrt.goal_bias      = cfg.planner.goal_bias;
    planner_cfg.rrt.safety_margin  = cfg.planner.safety_margin;
    planner_cfg.rrt.edge_check_res = cfg.planner.edge_check_res;
    planner_cfg.rrt.max_iter       = cfg.planner.max_iter;
    planner_cfg.rrt.goal_tol       = cfg.planner.goal_tol;
    planner_cfg.rrt.x_min          = cfg.world.grid.x_min;
    planner_cfg.rrt.x_max          = cfg.world.grid.x_max;
    planner_cfg.rrt.y_min          = cfg.world.grid.y_min;
    planner_cfg.rrt.y_max          = cfg.world.grid.y_max;

    RRTPIDPlanner         planner(planner_cfg);
    PIDController         controller(cfg.controller.kp,
                                     cfg.controller.ki,
                                     cfg.controller.kd,
                                     cfg.controller.attitude_kp,
                                     cfg.controller.yaw_kp);

    std::ofstream file("../../data/log.csv");

    file << "t,"
         << "target_x,target_y,target_z,"
         << "drone_x,drone_y,drone_z,"
         << "drone_vx,drone_vy,drone_vz,"
         << "drone_roll,drone_pitch,drone_yaw,"
         << "drone_wx,drone_wy,drone_wz,"
         << "target_vx,target_vy,target_vz,"
         << "roll_rate,pitch_rate,yaw_rate,thrust\n";

    for (double t = 0.0; t < cfg.sim.T; t += cfg.sim.dt)
    {
        State       d  = sim.getDroneState();
        TargetState tr = sim.getTargetTruth();

        Detection      det = perception.update();
        TargetEstimate est = estimator.update(det, cfg.sim.dt);
        Reference      ref = planner.update(d, est, esdf);
        ControlCommand cmd = controller.update(d, ref, cfg.sim.dt);

        sim.update(cmd, cfg.sim.dt);

        file << t              << ","
             << tr.x           << "," << tr.y          << "," << tr.z          << ","
             << d.x            << "," << d.y           << "," << d.z           << ","
             << d.vx           << "," << d.vy          << "," << d.vz          << ","
             << d.roll         << "," << d.pitch        << "," << d.yaw         << ","
             << d.wx           << "," << d.wy          << "," << d.wz          << ","
             << tr.vx          << "," << tr.vy          << "," << tr.vz         << ","
             << cmd.roll_rate  << "," << cmd.pitch_rate << "," << cmd.yaw_rate  << ","
             << cmd.thrust
             << "\n";
    }

    return 0;
}
