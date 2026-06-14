#include "simulation/Simulator.hpp"

void Simulator::step(
    State& drone,
    const ControlCommand& command,
    double dt)
{
    drone.x += command.vx_cmd * dt;
    drone.y += command.vy_cmd * dt;
    drone.z += command.vz_cmd * dt;

    drone.x_dot = command.vx_cmd;
    drone.y_dot = command.vy_cmd;
    drone.z_dot = command.vz_cmd;
}