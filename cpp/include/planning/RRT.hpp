#pragma once

#include "mapping/IESDFMap.hpp"
#include <array>
#include <random>
#include <vector>

struct RRTConfig
{
    double step_size      = 0.8;   // metres per extension
    double goal_bias      = 0.10;  // fraction of samples that are the goal
    double safety_margin  = 0.3;   // min ESDF clearance (m)
    double edge_check_res = 0.1;   // metres between collision checks along an edge
    int    max_iter       = 4000;
    double goal_tol       = 0.5;   // metres to declare goal reached
    // random-sample bounds (set to world extents)
    double x_min = -15.0, x_max = 65.0;
    double y_min = -25.0, y_max = 25.0;
};

// 2-D RRT: plans in the (x, y) plane.
// FakeESDFMap ignores z, so planning at fixed altitude is correct.
class RRT
{
public:
    RRT(const RRTConfig& cfg, unsigned seed = 42);

    // Returns waypoints from start to goal (inclusive), empty on failure.
    std::vector<std::array<double, 2>> plan(
        std::array<double, 2> start,
        std::array<double, 2> goal,
        const IESDFMap&       esdf);

private:
    struct Node { double x, y; int parent; };

    bool edgeClear(const Node& a, const Node& b, const IESDFMap& esdf) const;

    RRTConfig cfg_;
    std::mt19937 rng_;
    std::uniform_real_distribution<double> dist_x_, dist_y_, dist_u_;
};
