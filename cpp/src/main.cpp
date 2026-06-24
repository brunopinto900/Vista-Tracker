#include <fstream>
#include <iostream>
#include <string>
#include <filesystem>
#include <cmath>

#include "config/ConfigLoader.hpp"
#include "sim_impl/KinematicSim.hpp"
#include "perception/GroundTruthPerception.hpp"
#include "estimation/PerfectEstimator.hpp"
#include "mapping/FakeESDFMap.hpp"
#include "planning/RRTPIDPlanner.hpp"
#include "control/PIDController.hpp"

namespace fs = std::filesystem;

// Minimum horizontal standoff so the full person bounding box [0, h_top] fits
// within the usable VFOV half-angle phi at the altitude floor min_z.
//
// At min_z, both the head (h_top) and feet (0) must be within phi of the
// boresight aimed at h_aim.  Each constraint reduces to a quadratic in r;
// the binding one (larger root) is returned.
//
//   Head: tan(phi)*r² - (h_top-h_aim)*r + tan(phi)*(min_z-h_aim)*(min_z-h_top) = 0
//   Feet: tan(phi)*r² - h_aim*r         + tan(phi)*min_z*(min_z-h_aim)          = 0
static double computeStandoffMin(double min_z, double h_aim, double h_top, double phi_rad)
{
    const double tanp = std::tan(phi_rad);

    auto largeRoot = [&](double A, double B, double C) {
        const double disc = B * B - 4.0 * A * C;
        return (-B + std::sqrt(std::max(disc, 0.0))) / (2.0 * A);
    };

    const double r_head = largeRoot(tanp,
                                     -(h_top - h_aim),
                                     tanp * (min_z - h_aim) * (min_z - h_top));
    const double r_feet = largeRoot(tanp,
                                     -h_aim,
                                     tanp * min_z * (min_z - h_aim));
    return std::max(r_head, r_feet);
}

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
              << "[config] controller      kp_pos=" << cfg.controller.kp_pos
              << " ki_pos=" << cfg.controller.ki_pos
              << " kp_vel=" << cfg.controller.kp_vel
              << " ki_vel=" << cfg.controller.ki_vel
              << " max_tilt=" << cfg.controller.max_tilt_rad << " rad\n"
              << "[config] tracking_camera  hfov=" << cfg.tracking_camera.fov_deg << "°\n"
              << "[config] target          height=" << cfg.target.height
              << " width=" << cfg.target.width
              << " track_z=" << cfg.target.track_z << "\n"
              << "[config] planner         wp_thresh=" << cfg.planner.wp_reach_thresh
              << " replan_dist=" << cfg.planner.replan_goal_dist
              << " min_z=" << cfg.planner.min_z << "\n"
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
    planner_cfg.wp_reach_thresh  = cfg.planner.wp_reach_thresh;
    planner_cfg.replan_goal_dist = cfg.planner.replan_goal_dist;
    // Geometric VFOV half-angle for a 16:9 sensor, derived from HFOV.
    // Matches TRACKING_HALF_VFOV = atan(tan(HFOV/2) * 9/16) used in the visualisers.
    {
        const double hfov_half = cfg.tracking_camera.fov_deg * M_PI / 180.0 / 2.0;
        planner_cfg.vfov_half_rad = std::atan(std::tan(hfov_half) * 9.0 / 16.0);
    }
    planner_cfg.theta_des_rad  = cfg.planner.theta_des_deg  * M_PI / 180.0;
    planner_cfg.theta_safe_rad = cfg.planner.theta_safe_deg * M_PI / 180.0;
    planner_cfg.min_z          = cfg.planner.min_z;
    planner_cfg.target_track_z = cfg.target.track_z;
    planner_cfg.target_height  = cfg.target.height;
    // Log the floor-altitude standoff as a reference value.
    {
        const double phi_rad      = planner_cfg.vfov_half_rad - planner_cfg.theta_safe_rad;
        const double standoff_min = computeStandoffMin(
            planner_cfg.min_z, planner_cfg.target_track_z,
            cfg.target.height, phi_rad);
        std::cout << "[planner] standoff_at_min_z = " << standoff_min
                  << " m  (bounding-box FOV: min_z=" << planner_cfg.min_z
                  << " h_aim=" << planner_cfg.target_track_z
                  << " h_top=" << cfg.target.height
                  << " phi=" << phi_rad * 180.0 / M_PI << "°) — computed per-step from drone.z\n";
    }
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
    PIDController         controller(cfg.controller.kp_pos,
                                     cfg.controller.ki_pos,
                                     cfg.controller.kp_vel,
                                     cfg.controller.ki_vel,
                                     cfg.controller.attitude_kp,
                                     cfg.controller.yaw_kp,
                                     cfg.controller.max_tilt_rad,
                                     cfg.controller.max_thrust,
                                     cfg.controller.max_ipos_contribution,
                                     cfg.controller.max_ivel_contribution);

    std::ofstream file("../../data/log.csv");

    file << "t,"
         << "target_x,target_y,target_z,"
         << "drone_x,drone_y,drone_z,"
         << "drone_vx,drone_vy,drone_vz,"
         << "drone_roll,drone_pitch,drone_yaw,"
         << "drone_wx,drone_wy,drone_wz,"
         << "target_vx,target_vy,target_vz,"
         << "roll_rate,pitch_rate,yaw_rate,thrust,"
         << "ref_x,ref_y,ref_z,ref_yaw,ref_camera_pitch,"
         << "vel_ref_x,vel_ref_y,vel_ref_z,"
         << "deadlock_active,deadlock_angle\n";

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
             << cmd.thrust     << ","
             << ref.x          << "," << ref.y          << "," << ref.z         << ","
             << ref.yaw        << "," << ref.camera_pitch << ","
             << cmd.vx_sp      << "," << cmd.vy_sp      << "," << cmd.vz_sp     << ","
             << (ref.deadlock_active ? 1 : 0) << ","
             << ref.deadlock_angle
             << "\n";
    }

    return 0;
}
