#include <fstream>

#include "config/ConfigLoader.hpp"

#include "controllers/PIDController.hpp"
#include "simulation/Simulator.hpp"

int main()
{
    Config cfg =
        ConfigLoader::load(
            "../../config/config.yaml");

    double dt =
        cfg.simulation.dt;

    double sim_time =
        cfg.simulation.sim_time;

    State drone =
        cfg.drone;

    TargetState target =
        cfg.target;

    PIDController controller(
        cfg.controller.desired_distance,
        cfg.controller.pid.kp,
        cfg.controller.pid.ki,
        cfg.controller.pid.kd);

    Simulator simulator;

    std::ofstream file(
        "../../data/trajectory.csv");

    file << "t,"
     << "target_x,target_y,target_z,"
     << "drone_x,drone_y,drone_z,"
     << "drone_vx,drone_vy,drone_vz,"
     << "target_vx,target_vy,target_vz,"
     << "vx_cmd,vy_cmd,vz_cmd\n";


    for(double t = 0.0;
        t <= sim_time;
        t += dt)
    {
        target.x += target.vx * dt;
        target.y += target.vy * dt;
        target.z += target.vz * dt;

        auto cmd =
            controller.update(
                drone,
                target,
                dt);

        simulator.step(
            drone,
            cmd,
            dt);

        file << t << ","
     << target.x << "," << target.y << "," << target.z << ","
     << drone.x << "," << drone.y << "," << drone.z << ","
     << drone.x_dot << "," << drone.y_dot << "," << drone.z_dot << ","
     << target.vx << "," << target.vy << "," << target.vz << ","
     << cmd.vx_cmd << "," << cmd.vy_cmd << "," << cmd.vz_cmd
     << "\n";
    }

    return 0;
}