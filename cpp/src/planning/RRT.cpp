#include "planning/RRT.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

RRT::RRT(const RRTConfig& cfg, unsigned seed)
    : cfg_(cfg)
    , rng_(seed)
    , dist_x_(cfg.x_min, cfg.x_max)
    , dist_y_(cfg.y_min, cfg.y_max)
    , dist_u_(0.0, 1.0)
{}

static double dist2(double ax, double ay, double bx, double by)
{
    double dx = bx - ax, dy = by - ay;
    return std::sqrt(dx * dx + dy * dy);
}

bool RRT::edgeClear(const Node& a, const Node& b, const IESDFMap& esdf) const
{
    double dx = b.x - a.x, dy = b.y - a.y;
    double d  = std::sqrt(dx * dx + dy * dy);
    int steps = std::max(1, static_cast<int>(std::ceil(d / cfg_.edge_check_res)));
    for (int i = 0; i <= steps; ++i)
    {
        double t = static_cast<double>(i) / steps;
        if (esdf.query(a.x + t * dx, a.y + t * dy, 0.0) < cfg_.safety_margin)
            return false;
    }
    return true;
}

std::vector<std::array<double, 2>> RRT::plan(
    std::array<double, 2> start,
    std::array<double, 2> goal,
    const IESDFMap&       esdf)
{
    std::vector<Node> nodes;
    nodes.reserve(static_cast<size_t>(cfg_.max_iter));
    nodes.push_back({start[0], start[1], -1});

    for (int iter = 0; iter < cfg_.max_iter; ++iter)
    {
        // Sample: goal-biased
        double sx, sy;
        if (dist_u_(rng_) < cfg_.goal_bias)
            { sx = goal[0]; sy = goal[1]; }
        else
            { sx = dist_x_(rng_); sy = dist_y_(rng_); }

        // Nearest node
        int    nearest = 0;
        double best    = std::numeric_limits<double>::max();
        for (int i = 0; i < static_cast<int>(nodes.size()); ++i)
        {
            double d = dist2(nodes[i].x, nodes[i].y, sx, sy);
            if (d < best) { best = d; nearest = i; }
        }

        // Steer
        const Node& nr = nodes[nearest];
        double dx = sx - nr.x, dy = sy - nr.y;
        double d  = std::sqrt(dx * dx + dy * dy);
        if (d < 1e-9) continue;
        double s = std::min(cfg_.step_size, d) / d;
        Node nw{nr.x + dx * s, nr.y + dy * s, nearest};

        // Collision check
        if (esdf.query(nw.x, nw.y, 0.0) < cfg_.safety_margin) continue;
        if (!edgeClear(nr, nw, esdf)) continue;

        nodes.push_back(nw);
        int idx = static_cast<int>(nodes.size()) - 1;

        // Goal reached?
        if (dist2(nw.x, nw.y, goal[0], goal[1]) < cfg_.goal_tol)
        {
            std::vector<std::array<double, 2>> path;
            for (int i = idx; i >= 0; i = nodes[i].parent)
                path.push_back({nodes[i].x, nodes[i].y});
            std::reverse(path.begin(), path.end());
            path.push_back(goal);
            return path;
        }
    }

    return {};  // no path found within budget
}
