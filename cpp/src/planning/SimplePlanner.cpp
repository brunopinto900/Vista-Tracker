#include "planning/SimplePlanner.hpp"

#include <cmath>

SimplePlanner::SimplePlanner(double desired_distance)
    : desired_distance_(desired_distance) {}

Reference SimplePlanner::update(
    const State&          drone,
    const TargetEstimate& target,
    const IESDFMap&       /*esdf*/)
{
    const auto& t = target.horizon[0];

    Reference ref;
    ref.x   = t.x - desired_distance_;
    ref.y   = t.y;
    ref.z   = t.z;
    ref.yaw = std::atan2(t.y - drone.y, t.x - drone.x);  // face the target
    return ref;
}
