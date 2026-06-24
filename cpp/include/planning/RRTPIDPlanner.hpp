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
        double wp_reach_thresh  = 0.5;  // advance waypoint within this radius (m)
        double vfov_half_rad    = 0.3142;  // geometric V-FOV half-angle (rad) — 18° for 60° HFOV / 16:9
        double theta_des_rad    = 0.2269;  // desired viewing angle below horizon (rad) — 13°
        double theta_safe_rad   = 0.0524;  // FOV safety margin (rad) — 3°
        double min_z              = 0.0;     // altitude floor (m)
        double target_track_z     = 0.90;   // camera aim point height on target (m)
        double target_height      = 1.80;   // full person height (m) — used for standoff geometry
        double target_ground_strip = 0.30;  // ground visible below feet (m); adds third standoff constraint
        double path_resample_step = 0.15;   // arc-length step for Catmull-Rom resampling (m); 0 = off
        RRTConfig rrt{};
    };

    explicit RRTPIDPlanner(const Config& cfg, unsigned rrt_seed = 42);

    Reference update(
        const State&          drone,
        const TargetEstimate& target,
        const IESDFMap&       esdf) override;

private:
    std::array<double, 2> computeGoal(
        const State& drone, const TargetEstimate& target, double standoff) const;

    Config cfg_;
    RRT    rrt_;

    std::vector<std::array<double, 2>> path_;
    std::size_t                        wp_idx_          = 0;
    std::array<double, 2>              last_goal_       = {0.0, 0.0};
    bool                               last_goal_valid_ = false;
    bool                               rrt_failed_      = false;
};
