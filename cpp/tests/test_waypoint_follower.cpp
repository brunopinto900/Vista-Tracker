#include <gtest/gtest.h>
#include <cmath>
#include "sim_impl/WaypointFollower.hpp"

static TargetTrajectory make_traj(double max_accel        = 2.0,
                                   double max_speed        = 3.0,
                                   double max_lateral_accel = 10.0)
{
    TargetTrajectory t;
    t.max_accel         = max_accel;
    t.max_speed         = max_speed;
    t.max_lateral_accel = max_lateral_accel;
    t.loop              = false;
    return t;
}

static double speed(const TargetState& s)
{
    return std::sqrt(s.vx * s.vx + s.vy * s.vy + s.vz * s.vz);
}

// After enough steps the follower should reach the single waypoint and stop.
TEST(WaypointFollower, ReachesSingleWaypoint)
{
    auto traj = make_traj();
    traj.waypoints.push_back({10.0, 0.0, 0.0, 1.0, 0.0});

    WaypointFollower f(traj, {});
    for (int i = 0; i < 2000 && !f.done(); ++i)
        f.step(0.05);

    EXPECT_TRUE(f.done());
}

// During a hold the follower must output zero velocity for at least the hold
// duration, then continue to the next waypoint.
TEST(WaypointFollower, HoldKeepsVelocityZeroForSpecifiedDuration)
{
    const double hold_s = 1.0;
    const double dt     = 0.05;

    auto traj = make_traj();
    traj.waypoints.push_back({0.0,  0.0, 0.0, 1.0, hold_s}); // start == pos → instant reach
    traj.waypoints.push_back({10.0, 0.0, 0.0, 1.0, 0.0});

    WaypointFollower f(traj, {});

    double zero_speed_time = 0.0;
    for (int i = 0; i < 2000 && !f.done(); ++i)
    {
        TargetState s = f.step(dt);
        if (speed(s) < 1e-9)
            zero_speed_time += dt;
    }

    EXPECT_GE(zero_speed_time, hold_s - dt);  // one-step tolerance
}

// The braking ramp (sqrt(2*a*dist)) guarantees the target decelerates
// continuously as it approaches the waypoint.  In discrete time the speed
// lags the ramp by at most one max_accel*dt step, so arrival speed is
// bounded by roughly sqrt(2*a*threshold) + max_accel*dt.
// For max_accel=2.0, dt=0.05, threshold=0.1: bound ≈ 0.63 + 0.1 + lag ≈ 1.0.
TEST(WaypointFollower, BrakesAndStopsAtFinalWaypoint)
{
    const double dt        = 0.05;
    const double max_accel = 2.0;
    const double max_speed = 3.0;

    auto traj = make_traj(max_accel, max_speed);
    traj.waypoints.push_back({20.0, 0.0, 0.0, max_speed, 0.0});

    WaypointFollower f(traj, {});

    double max_spd = 0.0;
    TargetState last{};
    for (int i = 0; i < 2000 && !f.done(); ++i)
    {
        last    = f.step(dt);
        max_spd = std::max(max_spd, speed(last));
    }

    EXPECT_GT(max_spd,       max_speed * 0.9);  // accelerated to cruise speed
    EXPECT_LT(speed(last),   max_speed * 0.35); // braking reduced speed to < 35% of max
    EXPECT_TRUE(f.done());
}

// With loop=false the follower must report done() after the last waypoint.
// With loop=true the follower must never report done() within a finite run.
TEST(WaypointFollower, LoopFlagControlsTermination)
{
    auto no_loop = make_traj();
    no_loop.waypoints.push_back({2.0, 0.0, 0.0, 2.0, 0.0});
    no_loop.loop = false;

    WaypointFollower f_no_loop(no_loop, {});
    for (int i = 0; i < 2000; ++i) f_no_loop.step(0.05);
    EXPECT_TRUE(f_no_loop.done());

    auto looped = make_traj();
    looped.waypoints.push_back({2.0,  0.0, 0.0, 2.0, 0.0});
    looped.waypoints.push_back({0.0,  0.0, 0.0, 2.0, 0.0});
    looped.loop = true;

    WaypointFollower f_loop(looped, {});
    for (int i = 0; i < 2000; ++i) f_loop.step(0.05);
    EXPECT_FALSE(f_loop.done());
}

// With a tight lateral-accel budget the heading must change gradually —
// the measured yaw rate must not exceed max_lateral_accel / speed.
TEST(WaypointFollower, YawRateBoundedByLateralAccel)
{
    const double max_la = 0.5;   // tight — forces a wide arc
    const double dt     = 0.05;

    auto traj = make_traj(/*max_accel=*/2.0, /*max_speed=*/2.0, max_la);
    traj.waypoints.push_back({5.0,  0.0, 0.0, 2.0, 0.0}); // run straight
    traj.waypoints.push_back({5.0, 10.0, 0.0, 2.0, 0.0}); // sharp 90° turn

    WaypointFollower f(traj, {});

    TargetState prev = f.step(dt);
    for (int i = 1; i < 500; ++i)
    {
        TargetState cur = f.step(dt);
        double spd = speed(cur);
        if (spd < 0.1) { prev = cur; continue; }

        double dh = std::atan2(cur.vy, cur.vx) - std::atan2(prev.vy, prev.vx);
        while (dh >  M_PI) dh -= 2.0 * M_PI;
        while (dh < -M_PI) dh += 2.0 * M_PI;

        double yaw_rate = std::abs(dh) / dt;
        EXPECT_LE(yaw_rate, max_la / spd + 1e-6)
            << "step " << i << ": yaw_rate=" << yaw_rate
            << " limit=" << max_la / spd;

        prev = cur;
    }
}
