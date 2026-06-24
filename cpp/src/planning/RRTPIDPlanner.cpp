#include "planning/RRTPIDPlanner.hpp"
#include "models/State.hpp"

#include <algorithm>
#include <cmath>

// Catmull-Rom spline path smoothing with uniform arc-length resampling.
//
// The raw RRT output is piecewise-linear with 0.8 m steps; waypoint transitions
// produce step changes in ref.x/y → velocity error spikes → pitch/altitude jumps.
// This replaces the sparse waypoints with a dense, C1-continuous spline so the
// reference moves smoothly and pitch/altitude references inherit that smoothness.
//
// Algorithm:
//   1. Add phantom endpoints so every segment has four control points.
//   2. Evaluate the spline at 20 sub-steps per segment → dense polyline.
//   3. Resample the dense polyline at uniform arc-length intervals (step).
static std::vector<std::array<double,2>> smoothAndResample(
    const std::vector<std::array<double,2>>& pts, double step)
{
    const std::size_t n = pts.size();
    if (n < 3 || step <= 0.0)
        return pts;

    // Phantom endpoints: mirror first/last segment outward
    const std::array<double,2> pre  = {2*pts[0][0]-pts[1][0],   2*pts[0][1]-pts[1][1]};
    const std::array<double,2> post = {2*pts[n-1][0]-pts[n-2][0], 2*pts[n-1][1]-pts[n-2][1]};

    std::vector<std::array<double,2>> ctrl;
    ctrl.reserve(n + 2);
    ctrl.push_back(pre);
    ctrl.insert(ctrl.end(), pts.begin(), pts.end());
    ctrl.push_back(post);

    // Catmull-Rom evaluation: segment seg (ctrl[seg..seg+3]) at parameter t∈[0,1]
    auto evalCR = [&](std::size_t seg, double t) -> std::array<double,2> {
        const auto& p0 = ctrl[seg];
        const auto& p1 = ctrl[seg+1];
        const auto& p2 = ctrl[seg+2];
        const auto& p3 = ctrl[seg+3];
        const double t2 = t*t, t3 = t2*t;
        return {
            0.5*(2*p1[0] + (-p0[0]+p2[0])*t + ( 2*p0[0]-5*p1[0]+4*p2[0]-p3[0])*t2 + (-p0[0]+3*p1[0]-3*p2[0]+p3[0])*t3),
            0.5*(2*p1[1] + (-p0[1]+p2[1])*t + ( 2*p0[1]-5*p1[1]+4*p2[1]-p3[1])*t2 + (-p0[1]+3*p1[1]-3*p2[1]+p3[1])*t3)
        };
    };

    // Build dense polyline: 20 sub-samples per spline segment
    const int SUB = 20;
    std::vector<std::array<double,2>> dense;
    dense.reserve((n - 1) * SUB + 1);
    dense.push_back(pts.front());
    for (std::size_t seg = 0; seg + 1 < n; ++seg)
        for (int k = 1; k <= SUB; ++k)
            dense.push_back(evalCR(seg, static_cast<double>(k) / SUB));

    // Resample dense polyline at uniform arc-length intervals
    std::vector<std::array<double,2>> result;
    result.push_back(dense.front());
    double leftover = 0.0;

    for (std::size_t i = 1; i < dense.size(); ++i)
    {
        const double dx  = dense[i][0] - dense[i-1][0];
        const double dy  = dense[i][1] - dense[i-1][1];
        const double len = std::sqrt(dx*dx + dy*dy);
        if (len < 1e-9)
            continue;
        double d = step - leftover;  // distance to next emit point from segment start
        while (d <= len)
        {
            const double frac = d / len;
            result.push_back({dense[i-1][0] + frac*dx, dense[i-1][1] + frac*dy});
            d += step;
        }
        leftover = len - (d - step);
    }

    result.push_back(dense.back());
    return result;
}

// Minimum horizontal standoff so:
//   • full person bounding box [0, h_top] fits within VFOV half-angle phi, AND
//   • a ground strip of depth `ground_strip` below feet is also visible.
//
// Each visibility constraint (head, feet, ground) produces a quadratic in r;
// the largest root is binding.  Derivation for a point at height h:
//   tan(phi)·r² − (h_aim−h)·r + tan(phi)·(alt−h_aim)·(alt−h) = 0
static double computeStandoffMin(double alt, double h_aim, double h_top,
                                 double phi_rad, double ground_strip)
{
    const double tanp = std::tan(phi_rad);
    auto largeRoot = [&](double A, double B, double C) {
        const double disc = B * B - 4.0 * A * C;
        return (-B + std::sqrt(std::max(disc, 0.0))) / (2.0 * A);
    };
    const double r_head  = largeRoot(tanp, -(h_top - h_aim),
                                     tanp * (alt - h_aim) * (alt - h_top));
    const double r_feet  = largeRoot(tanp, -h_aim,
                                     tanp * alt * (alt - h_aim));
    // h_ground = -ground_strip (below feet) → (h_aim − h_ground) = h_aim + ground_strip
    const double r_ground = largeRoot(tanp, -(h_aim + ground_strip),
                                      tanp * (alt - h_aim) * (alt + ground_strip));
    return std::max({r_head, r_feet, r_ground});
}

RRTPIDPlanner::RRTPIDPlanner(const Config& cfg, unsigned rrt_seed)
    : cfg_(cfg)
    , rrt_(cfg.rrt, rrt_seed)
{}

std::array<double, 2> RRTPIDPlanner::computeGoal(
    const State& drone, const TargetEstimate& target, double standoff) const
{
    const auto& t = target.horizon[0];
    double dx = drone.x - t.x;
    double dy = drone.y - t.y;
    double d  = std::sqrt(dx * dx + dy * dy);
    if (d < 1e-6)
        return {t.x + standoff, t.y};
    double s = standoff / d;
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

    // Standoff geometry: recompute each step from actual drone altitude.
    // Higher altitude → steeper view angle → person fits in frame at shorter range.
    // Clamp to min_z so the formula never receives an altitude below the floor.
    const double phi_rad      = cfg_.vfov_half_rad - cfg_.theta_safe_rad;
    const double standoff_dist = computeStandoffMin(
        std::max(drone.z, cfg_.min_z),
        cfg_.target_track_z, cfg_.target_height, phi_rad,
        cfg_.target_ground_strip);

    auto ideal_goal = computeGoal(drone, target, standoff_dist);

    // When the ideal standoff is inside an obstacle, find the nearest feasible
    // standoff by scanning alternate bearing angles.  This prevents planning
    // toward an infeasible goal and avoids permanent hold-in-place.
    auto goal = findFeasibleGoal(
        ideal_goal, t, standoff_dist, cfg_.rrt.safety_margin, esdf);

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
            // Evaluate triviality on the raw RRT output before smoothing.
            // A trivial path ([start, goal] = 2 nodes) means the drone is already
            // at standoff; keeping last_goal_ fixed lets drift accumulate so a
            // real obstacle-routing replan fires when the target has moved far enough.
            const bool non_trivial = (new_path.size() > 2);
            if (non_trivial && cfg_.path_resample_step > 0.0)
                new_path = smoothAndResample(new_path, cfg_.path_resample_step);
            path_       = std::move(new_path);
            wp_idx_     = 0;
            rrt_failed_ = false;
            if (non_trivial)
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
    ref.z   = std::max(cfg_.target_track_z + h_des, cfg_.min_z);
    ref.yaw = std::atan2(t.y - drone.y, t.x - drone.x);

    // When the altitude floor clips h_des, the geometric viewing angle is larger
    // than theta_ref.  Command the actual angle so the body pitches to keep the
    // boresight on track_z rather than under-aiming.
    if (ref.z > cfg_.target_track_z + h_des && horiz_dist > 1e-6)
        ref.camera_pitch = std::clamp(
            std::atan2(ref.z - cfg_.target_track_z, horiz_dist),
            0.0, std::max(0.0, theta_margin));
    else
        ref.camera_pitch = theta_ref;
    ref.deadlock_active = !ideal_feasible;
    ref.deadlock_angle  = !ideal_feasible
        ? std::atan2(goal[1] - t.y, goal[0] - t.x)
        : 0.0;

    ref.camera_pitch = 0;
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
