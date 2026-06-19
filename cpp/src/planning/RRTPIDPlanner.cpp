#include "planning/RRTPIDPlanner.hpp"
#include "models/State.hpp"

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
        return {t.x + cfg_.standoff_dist, t.y};  // arbitrary offset when on top of target
    double s = cfg_.standoff_dist / d;
    return {t.x + dx * s, t.y + dy * s};
}

Reference RRTPIDPlanner::update(
    const State&          drone,
    const TargetEstimate& target,
    const IESDFMap&       esdf)
{
    const auto& t = target.horizon[0];

    auto goal = computeGoal(drone, target);

    // Replan when goal drifted, path is exhausted, or first call
    double goal_drift = std::sqrt(
        std::pow(goal[0] - last_goal_[0], 2) +
        std::pow(goal[1] - last_goal_[1], 2));

    if (path_.empty() || wp_idx_ >= path_.size() || goal_drift > cfg_.replan_goal_dist)
    {
        path_      = rrt_.plan({drone.x, drone.y}, goal, esdf);
        wp_idx_    = 0;
        last_goal_ = goal;
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

    Reference ref;
    if (!path_.empty() && wp_idx_ < path_.size())
    {
        ref.x = path_[wp_idx_][0];
        ref.y = path_[wp_idx_][1];
    }
    else
    {
        // RRT failed or path fully consumed — fall back to direct goal
        ref.x = goal[0];
        ref.y = goal[1];
    }
    ref.z   = cfg_.z_ref;
    ref.yaw = std::atan2(t.y - drone.y, t.x - drone.x);  // always face target

    // Populate trajectory for future MPC consumer
    ref.trajectory.clear();
    for (std::size_t i = wp_idx_; i < path_.size(); ++i)
    {
        State s{};
        s.x = path_[i][0];
        s.y = path_[i][1];
        s.z = cfg_.z_ref;
        ref.trajectory.push_back(s);
    }

    return ref;
}
