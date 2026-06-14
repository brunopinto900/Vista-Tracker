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

    file
        << "time,"
        << "target_x,target_y,"
        << "drone_x,drone_y\n";

    for(double t = 0.0;
        t <= sim_time;
        t += dt)
    {
        target.x += target.vx * dt;
        target.y += target.vy * dt;
        target.z += target.vz * dt;

        auto command =
            controller.update(
                drone,
                target,
                dt);

        simulator.step(
            drone,
            command,
            dt);

        file
            << t << ","
            << target.x << ","
            << target.y << ","
            << drone.x << ","
            << drone.y
            << "\n";
    }

    return 0;
}