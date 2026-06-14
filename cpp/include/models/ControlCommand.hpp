#pragma once

struct ControlCommand
{
    double vx_cmd{};
    double vy_cmd{};
    double vz_cmd{};

    double yaw_rate_cmd{};
};