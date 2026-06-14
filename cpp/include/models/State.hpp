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
    double vx{};
    double vy{};
    double vz{};

    // Angular velocity
    double wx{};
    double wy{};
    double wz{};
};