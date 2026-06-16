#pragma once

#include "models/TargetTrajectory.hpp"
#include "models/TargetState.hpp"

// Drives a target through a waypoint list with:
//   - trapezoidal longitudinal velocity profile (max_accel, max_speed)
//   - smooth heading via lateral acceleration limit (max_lateral_accel)
//     → turning radius = v² / max_lateral_accel
//     → person: turns freely;  car: large radius at speed
class WaypointFollower
{
public:
    WaypointFollower(const TargetTrajectory& traj, const TargetState& init);

    TargetState step(double dt);

    bool done() const { return done_; }

private:
    void advance();

    TargetTrajectory traj_;
    TargetState      state_;
    double           heading_ = 0.0;  // radians, current travel direction

    int    idx_          = 0;
    bool   holding_      = false;
    double hold_elapsed_ = 0.0;
    bool   done_         = false;
};
