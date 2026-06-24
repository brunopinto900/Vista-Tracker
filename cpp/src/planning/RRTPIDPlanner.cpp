#include "planning/RRTPIDPlanner.hpp"
#include "models/State.hpp"

#include <algorithm>
#include <cmath>

RRTPIDPlanner::RRTPIDPlanner(const Config& cfg, unsigned rrt_seed)
    : cfg_(cfg)
    , rrt_(cfg.rrt, rrt_seed)
{}

std::array<double, 2> RRTPIDPlanner::computeGoal(
    const State& drone, const TargetEstimate& target) const
{
    const auto& t = target.horizon[0];
    double dx = drone.x - t.x;
    double dy = drone.y - t.y;
    double d  = std::sqrt(dx * dx + dy * dy);
    if (d < 1e-6)
        return {t.x + cfg_.standoff_dist, t.y};
    double s = cfg_.standoff_dist / d;
    return {t.x + dx * s, t.y + dy * s};
}

// When the ideal standoff goal is inside an obstacle's safety margin, scan
// alternate bearing angles (±5°, ±10°, …, ±180°) around the target until a
// feasible position is found.  This lets the drone reposition to the side of
// an obstacle rather than freezing whenever the target passes close to one.
static std::array<double, 2> findFeasibleGoal(
    const std::array<double, 2>& ideal,
    const PredictedTargetState&  t,
    double                       standoff,
    double                       margin,
    const IESDFMap&              esdf)
{
    if (esdf.query(ideal[0], ideal[1], 0.0) >= static_cast<float>(margin))
        return ideal;

    double base_angle = std::atan2(ideal[1] - t.y, ideal[0] - t.x);

    // Try increasing angular offsets on both sides
    for (int step = 1; step <= 36; ++step)
    {
        for (int sign : {1, -1})
        {
            double angle = base_angle + sign * step * (M_PI / 36.0);  // 5° increments
            double gx = t.x + standoff * std::cos(angle);
            double gy = t.y + standoff * std::sin(angle);
            if (esdf.query(gx, gy, 0.0) >= static_cast<float>(margin))
                return {gx, gy};
        }
    }

    return ideal;  // give up — let RRT handle the infeasible goal
}

Reference RRTPIDPlanner::update(
    const State&          drone,
    const TargetEstimate& target,
    const IESDFMap&       esdf)
{
    const auto& t = target.horizon[0];

    auto ideal_goal = computeGoal(drone, target);

    // When the ideal standoff is inside an obstacle, find the nearest feasible
    // standoff by scanning alternate bearing angles.  This prevents planning
    // toward an infeasible goal and avoids permanent hold-in-place.
    auto goal = findFeasibleGoal(
        ideal_goal, t, cfg_.standoff_dist, cfg_.rrt.safety_margin, esdf);

    const bool ideal_feasible = (esdf.query(ideal_goal[0], ideal_goal[1], 0.0) >=
                                  static_cast<float>(cfg_.rrt.safety_margin));

    double goal_drift = last_goal_valid_
        ? std::sqrt(std::pow(goal[0] - last_goal_[0], 2) +
                    std::pow(goal[1] - last_goal_[1], 2))
        : 1e9;  // force first replan

    // Replan when:
    //   - no path exists (first call)
    //   - last RRT call failed (retry each cycle until success)
    //   - goal has drifted far enough from the last non-trivial plan anchor
    // Path consumption alone does NOT trigger replan — at standoff the path is
    // consumed instantly, which would pin last_goal_ and prevent drift from
    // accumulating for the next obstacle-routing replan.
    if (path_.empty() || goal_drift > cfg_.replan_goal_dist || rrt_failed_)
    {
        // If the drone is inside the safety-margin zone (e.g. it tracked to a
        // tight standoff right against an obstacle face), RRT cannot grow from
        // that start — every edge begins in the infeasible region and is
        // rejected.  Find the nearest feasible point and use it as the RRT
        // start, then prepend the drone's actual position so the sequencer
        // bridges the gap naturally.
        std::array<double, 2> rrt_start{drone.x, drone.y};
        if (esdf.query(drone.x, drone.y, 0.0) < static_cast<float>(cfg_.rrt.safety_margin))
        {
            bool found = false;
            for (double r = cfg_.rrt.edge_check_res; r <= 3.0 && !found; r += cfg_.rrt.edge_check_res)
            {
                for (int ang = 0; ang < 72 && !found; ++ang)
                {
                    double theta = ang * (M_PI / 36.0);
                    double tx = drone.x + r * std::cos(theta);
                    double ty = drone.y + r * std::sin(theta);
                    if (esdf.query(tx, ty, 0.0) >= static_cast<float>(cfg_.rrt.safety_margin))
                    {
                        rrt_start = {tx, ty};
                        found = true;
                    }
                }
            }
        }

        auto new_path = rrt_.plan(rrt_start, goal, esdf);
        // Prepend drone's actual position so the path begins where the drone is
        if (!new_path.empty() && (rrt_start[0] != drone.x || rrt_start[1] != drone.y))
            new_path.insert(new_path.begin(), {drone.x, drone.y});
        if (!new_path.empty())
        {
            path_       = std::move(new_path);
            wp_idx_     = 0;
            rrt_failed_ = false;
            // Anchor last_goal_ only for non-trivial paths (> 2 nodes = [start, goal]).
            // Trivial paths mean the drone is already at standoff; keeping the anchor
            // fixed lets drift accumulate so a real obstacle-routing replan fires when
            // the target has moved far enough.
            if (path_.size() > 2)
            {
                last_goal_       = goal;
                last_goal_valid_ = true;
            }
        }
        else
        {
            // RRT failed — keep existing path so the drone continues on the last
            // valid obstacle-free route.  If there is no prior path, the reference
            // block below holds position.
            rrt_failed_ = true;
        }
    }

    // Sequencer: skip waypoints the drone has already reached
    while (wp_idx_ < path_.size())
    {
        double dx = path_[wp_idx_][0] - drone.x;
        double dy = path_[wp_idx_][1] - drone.y;
        if (std::sqrt(dx * dx + dy * dy) > cfg_.wp_reach_thresh)
            break;
        ++wp_idx_;
    }

    // Visibility-Aware Target Following (NED formulation, adapted to ENU).
    //
    // θ_margin = FOV budget remaining after body tilt and safety reserve:
    //   θ_margin = θ_FOV/2 − |θ_body| − θ_safe
    //
    // θ_ref is the actual viewing angle commanded (clamped to margin so the
    // target stays in-frame even when body pitches for forward propulsion).
    //
    // h_des = r·tan(θ_ref) gives the altitude the drone must be above track_z
    // to see the target at the desired angle with the current standoff range r.
    // When body pitch grows (propulsion load), θ_margin shrinks → h_des shrinks
    // → drone descends → less pitch required → stable feedback.
    const double horiz_dist   = std::hypot(t.x - drone.x, t.y - drone.y);
    const double theta_body   = std::abs(drone.pitch);
    const double theta_margin = cfg_.vfov_half_rad - theta_body - cfg_.theta_safe_rad;
    const double theta_ref    = std::clamp(cfg_.theta_des_rad, 0.0,
                                            std::max(0.0, theta_margin));
    const double h_des        = horiz_dist * std::tan(theta_ref);

    Reference ref;
    ref.z            = std::max(cfg_.target_track_z + h_des, cfg_.min_z);
    ref.yaw          = std::atan2(t.y - drone.y, t.x - drone.x);
    ref.camera_pitch = theta_ref;
    ref.deadlock_active = !ideal_feasible;
    ref.deadlock_angle  = !ideal_feasible
        ? std::atan2(goal[1] - t.y, goal[0] - t.x)
        : 0.0;

    if (!path_.empty() && wp_idx_ < path_.size())
    {
        // Following a valid RRT path
        ref.x  = path_[wp_idx_][0];
        ref.y  = path_[wp_idx_][1];
        ref.vx = t.vx;
        ref.vy = t.vy;
    }
    else if (!rrt_failed_ && ideal_feasible)
    {
        // Path consumed and ideal standoff is in free space — direct tracking.
        ref.x  = ideal_goal[0];
        ref.y  = ideal_goal[1];
        ref.vx = t.vx;
        ref.vy = t.vy;
    }
    else if (!rrt_failed_)
    {
        // Path consumed but ideal standoff is blocked (target near obstacle).
        // Track the nearest feasible standoff instead.
        ref.x  = goal[0];
        ref.y  = goal[1];
        ref.vx = 0.0;  // no feedforward when goal is an angle-shifted proxy
        ref.vy = 0.0;
    }
    else
    {
        // RRT failed — hold position and zero feedforward so the drone does not
        // fly through obstacles toward the direct standoff goal.
        ref.x  = drone.x;
        ref.y  = drone.y;
        ref.vx = 0.0;
        ref.vy = 0.0;
    }

    // Populate trajectory for future MPC consumer
    ref.trajectory.clear();
    for (std::size_t i = wp_idx_; i < path_.size(); ++i)
    {
        State s{};
        s.x = path_[i][0];
        s.y = path_[i][1];
        s.z = ref.z;
        ref.trajectory.push_back(s);
    }

    return ref;
}
