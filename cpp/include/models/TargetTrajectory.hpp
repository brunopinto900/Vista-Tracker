#pragma once

#include "models/Waypoint.hpp"
#include <string>
#include <vector>

struct TargetTrajectory
{
    // ── Target type ──────────────────────────────────────────────────────────
    // Drives kinematic constraints here and EKF motion model selection later.
    // Supported: "person" | "bicycle" | "car"
    std::string type = "person";

    // ── Kinematic constraints ─────────────────────────────────────────────────
    double max_accel         = 1.0;   // m/s²  — longitudinal
    double max_speed         = 2.0;   // m/s   — hard cap on waypoint speeds
    double max_lateral_accel = 4.0;   // m/s²  — controls turning radius
                                      //   person ≈ 4.0 (pivot freely)
                                      //   bicycle ≈ 3.0
                                      //   car     ≈ 3.0 (large radius at speed)

    // ── Waypoints ────────────────────────────────────────────────────────────
    std::vector<Waypoint> waypoints;
    bool loop = false;
};
