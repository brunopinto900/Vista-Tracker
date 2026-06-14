#include <fstream>

#include "config/ConfigLoader.hpp"
#include "sim_impl/KinematicSim.hpp"
#include "control/PIDController.hpp"
#include "planning/TrajectoryPlanner.hpp"

int main()
{
    Config cfg = ConfigLoader::load(
        "../../config/config.yaml");

    State drone{};
    drone.x = cfg.drone_init.x;
    drone.y = cfg.drone_init.y;
    drone.z = cfg.drone_init.z;

    TargetState target = cfg.target_init;

    KinematicSim    sim(drone, target, cfg.world);
    PIDController   controller(cfg.controller.kp,
                               cfg.controller.ki,
                               cfg.controller.kd);
    TrajectoryPlanner planner(cfg.controller.desired_distance);

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
        TargetState tr = sim.getTargetState();

        State ref = planner.computeDesired(d, tr);

        ControlCommand cmd = controller.update(d, ref, cfg.sim.dt);

        sim.applyControl(cmd, cfg.sim.dt);
        sim.stepTarget(cfg.sim.dt);

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