#pragma once

#include "planning/IPlanner.hpp"
#include "planning/RRT.hpp"
#include <array>
#include <vector>

// RRT + waypoint sequencer baseline.
//
// The sequencer lives here, not in the controller — mirroring how an MPC
// controller owns its horizon internally.  IController sees only the single
// Reference point it has always consumed; nothing downstream changes.
//
// On each update():
//   1. computeGoal()  → 4 m standoff point on drone-side of target
//   2. replan if goal drifted or path exhausted  → RRT(drone, goal, esdf)
//   3. advance wp_idx_ if drone reached current waypoint
//   4. return Reference{path[wp_idx_], yaw = atan2(target - drone)}
//
// reference.trajectory is populated with remaining waypoints so that a future
// MPC controller can consume the horizon without changing this planner.
class RRTPIDPlanner : public IPlanner
{
public:
    struct Config
    {
        double standoff_dist    = 4.0;  // metres from target
        double wp_reach_thresh  = 0.5;  // advance waypoint within this radius (m)
        double replan_goal_dist = 1.5;  // replan when goal shifts more than this (m)
        double z_ref            = 4.0;  // fixed flight altitude (m)
        RRTConfig rrt{};
    };

    explicit RRTPIDPlanner(const Config& cfg, unsigned rrt_seed = 42);

    Reference update(
        const State&          drone,
        const TargetEstimate& target,
        const IESDFMap&       esdf) override;

private:
    std::array<double, 2> computeGoal(
        const State& drone, const TargetEstimate& target) const;

    Config cfg_;
    RRT    rrt_;

    std::vector<std::array<double, 2>> path_;
    std::size_t                        wp_idx_    = 0;
    std::array<double, 2>              last_goal_ = {1e18, 1e18};
};
