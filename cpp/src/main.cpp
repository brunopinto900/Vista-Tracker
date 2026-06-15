#include <fstream>
#include <iostream>

#include "config/ConfigLoader.hpp"
#include "sim_impl/KinematicSim.hpp"
#include "perception/GroundTruthPerception.hpp"
#include "estimation/PerfectEstimator.hpp"
#include "mapping/FakeESDFMap.hpp"
#include "planning/SimplePlanner.hpp"
#include "control/PIDController.hpp"

int main()
{
    Config cfg = ConfigLoader::load("../../config/config.yaml");

    State drone{};
    drone.x = cfg.drone_init.x;
    drone.y = cfg.drone_init.y;
    drone.z = cfg.drone_init.z;

    std::cout << "[config] drone_init      x=" << cfg.drone_init.x
              << " y=" << cfg.drone_init.y
              << " z=" << cfg.drone_init.z << "\n"
              << "[config] target_init     x=" << cfg.target_init.x
              << " y=" << cfg.target_init.y
              << " z=" << cfg.target_init.z
              << " vx=" << cfg.target_init.vx
              << " vy=" << cfg.target_init.vy
              << " vz=" << cfg.target_init.vz << "\n"
              << "[config] sim             dt=" << cfg.sim.dt
              << " T=" << cfg.sim.T << "\n"
              << "[config] estimator       horizon=" << cfg.estimator.horizon << "\n"
              << "[config] controller      kp=" << cfg.controller.kp
              << " ki=" << cfg.controller.ki
              << " kd=" << cfg.controller.kd
              << " desired_distance=" << cfg.controller.desired_distance << "\n"
              << "[config] world.grid      x=[" << cfg.world.grid.x_min << ", " << cfg.world.grid.x_max << "]"
              << " y=[" << cfg.world.grid.y_min << ", " << cfg.world.grid.y_max << "]\n"
              << "[config] world.obstacles " << cfg.world.obstacles.size() << " loaded\n";

    KinematicSim          sim(drone, cfg.target_init, cfg.world);
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
