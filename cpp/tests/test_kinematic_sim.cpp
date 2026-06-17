#include <gtest/gtest.h>
#include <cmath>
#include "sim_impl/KinematicSim.hpp"

// Helper: sim with a trivial one-waypoint trajectory (target just sits at origin)
static KinematicSim make_sim(double tau = 0.1)
{
    State init{};
    TargetTrajectory traj;
    traj.max_accel = 1.0;
    traj.max_speed = 2.0;
    traj.waypoints.push_back({0.0, 0.0, 0.0, 1.0, 0.0});

    return KinematicSim(init, traj, World{}, tau);
}

// At zero angles and T=1 (exact hover) the drone should not accelerate
// vertically — thrust exactly cancels gravity.
TEST(KinematicSim, HoverEquilibriumNoVerticalDrift)
{
    const double dt  = 0.01;
    const double tau = 0.01;  // fast lag so it settles quickly

    auto sim = make_sim(tau);
    ControlCommand hover{0.0, 0.0, 0.0, 1.0};

    // Let the lag settle (5*tau = 0.05 s → 5 steps)
    for (int i = 0; i < 5; ++i)
        sim.update(hover, dt);

    State s = sim.getDroneState();
    EXPECT_NEAR(s.vz, 0.0, 0.05);   // not climbing or falling
    EXPECT_NEAR(s.vx, 0.0, 0.01);
    EXPECT_NEAR(s.vy, 0.0, 0.01);
}

// Positive pitch_rate → positive pitch angle → positive x acceleration.
TEST(KinematicSim, PitchRateGivesForwardAcceleration)
{
    const double dt  = 0.01;
    const double tau = 0.01;

    auto sim = make_sim(tau);

    // Pitch forward while maintaining hover thrust
    ControlCommand cmd{0.0, 1.0, 0.0, 1.0};
    for (int i = 0; i < 100; ++i)
        sim.update(cmd, dt);

    State s = sim.getDroneState();
    EXPECT_GT(s.pitch, 0.0);   // pitched forward
    EXPECT_GT(s.vx,    0.0);   // moving forward
}

// Thrust > 1.0 produces net upward acceleration from hover.
TEST(KinematicSim, ThrustAboveOneClimbs)
{
    const double dt  = 0.01;
    const double tau = 0.01;

    auto sim = make_sim(tau);

    // Settle at hover
    for (int i = 0; i < 10; ++i) sim.update({0,0,0,1.0}, dt);

    // Increase thrust
    for (int i = 0; i < 50; ++i) sim.update({0,0,0,1.5}, dt);

    EXPECT_GT(sim.getDroneState().vz, 0.1);
}

// First-order lag: after exactly tau seconds, the actual body rate must
// equal (1 - 1/e) ≈ 63.2% of the commanded rate.
TEST(KinematicSim, LagSettlesTo63PercentAfterOneTau)
{
    const double tau = 0.2;
    const double dt  = 0.001;
    const int    N   = static_cast<int>(tau / dt);

    auto sim = make_sim(tau);
    ControlCommand cmd{1.0, 0.0, 0.0, 1.0};  // roll_rate = 1 rad/s

    for (int i = 0; i < N; ++i)
        sim.update(cmd, dt);

    const double expected = 1.0 * (1.0 - std::exp(-1.0));  // ≈ 0.632
    EXPECT_NEAR(sim.getDroneState().wx, expected, 0.01);
}

// After five time constants the output must be within 1% of the setpoint.
TEST(KinematicSim, LagFullySettlesAfterFiveTau)
{
    const double tau = 0.1;
    const double dt  = 0.001;
    const int    N   = static_cast<int>(5.0 * tau / dt);

    auto sim = make_sim(tau);
    ControlCommand cmd{1.0, 0.0, 0.0, 1.0};

    for (int i = 0; i < N; ++i)
        sim.update(cmd, dt);

    EXPECT_NEAR(sim.getDroneState().wx, 1.0, 0.01);
}
