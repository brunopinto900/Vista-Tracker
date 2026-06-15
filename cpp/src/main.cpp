#include <fstream>

#include "config/ConfigLoader.hpp"
#include "sim_impl/KinematicSim.hpp"
#include "perception/GroundTruthPerception.hpp"
#include "estimation/PerfectEstimator.hpp"
#include "planning/SimplePlanner.hpp"
#include "control/PIDController.hpp"

int main()
{
    Config cfg = ConfigLoader::load("../../config/config.yaml");

    State drone{};
    drone.x = cfg.drone_init.x;
    drone.y = cfg.drone_init.y;
    drone.z = cfg.drone_init.z;

    KinematicSim          sim(drone, cfg.target_init, cfg.world);
    GroundTruthPerception perception(sim);
    PerfectEstimator      estimator;
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
        Reference      ref = planner.update(d, est);
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
