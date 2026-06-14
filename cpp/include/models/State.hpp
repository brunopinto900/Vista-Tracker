#pragma once

struct State
{
    // Position
    double x{};
    double y{};
    double z{};

    // Attitude
    double roll{};
    double pitch{};
    double yaw{};

    // Linear velocity
    double x_dot{};
    double y_dot{};
    double z_dot{};

    // Angular velocity
    double roll_rate{};
    double pitch_rate{};
    double yaw_rate{};
};