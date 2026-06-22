#include "planning/SimplePlanner.hpp"

#include <cmath>

SimplePlanner::SimplePlanner(const Config& cfg) : cfg_(cfg) {}

Reference SimplePlanner::update(
    const State&          drone,
    const TargetEstimate& target,
    const IESDFMap&       /*esdf*/)
{
    const auto& t = target.horizon[0];

    const double horiz_dist  = std::hypot(t.x - drone.x, t.y - drone.y);
    const double camera_pitch = (horiz_dist < 1e-6)
        ? M_PI_2
        : std::atan2(drone.z - cfg_.target_track_z, horiz_dist);

    const double z_vfov = cfg_.target_track_z
                        + horiz_dist * std::tan(0.9 * cfg_.vfov_half_rad);
    Reference ref;
    ref.x            = t.x - cfg_.desired_distance;
    ref.y            = t.y;
    ref.z            = std::max(z_vfov, cfg_.min_z);
    ref.yaw          = std::atan2(t.y - drone.y, t.x - drone.x);
    ref.camera_pitch = camera_pitch;
    return ref;
}
