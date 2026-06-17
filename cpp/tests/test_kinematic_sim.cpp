#include <gtest/gtest.h>
#include <cmath>
#include "sim_impl/KinematicSim.hpp"

static KinematicSim make_sim(double wn = 25.0, double zeta = 0.7)
{
    State init{};
    TargetTrajectory traj;
    traj.max_accel = 1.0;
    traj.max_speed = 2.0;
    traj.waypoints.push_back({0.0, 0.0, 0.0, 1.0, 0.0});

    return KinematicSim(init, traj, World{}, wn, zeta);
}

// dt=0.001 keeps wn*dt = 0.025 — well inside the Euler stability region.
// Settling time for wn=25, zeta=0.7: ts ≈ 5.8/(zeta*wn) ≈ 330 ms → 330 steps.
static constexpr double kDt      = 0.001;
static constexpr int    kSettle  = 400;   // steps to let rate dynamics settle

// At zero angles and T=1 (exact hover) the drone should not accelerate
// vertically — thrust exactly cancels gravity.
TEST(KinematicSim, HoverEquilibriumNoVerticalDrift)
{
    auto sim = make_sim();
    ControlCommand hover{0.0, 0.0, 0.0, 1.0};

    for (int i = 0; i < kSettle; ++i)
        sim.update(hover, kDt);

    State s = sim.getDroneState();
    EXPECT_NEAR(s.vz, 0.0, 0.05);
    EXPECT_NEAR(s.vx, 0.0, 0.01);
    EXPECT_NEAR(s.vy, 0.0, 0.01);
}

// Positive pitch_rate command → pitch angle builds up → positive x acceleration.
TEST(KinematicSim, PitchRateGivesForwardAcceleration)
{
    auto sim = make_sim();

    // Settle at hover first, then pitch forward
    for (int i = 0; i < kSettle; ++i) sim.update({0,0,0,1.0}, kDt);
    for (int i = 0; i < 200;     ++i) sim.update({0,1,0,1.0}, kDt);

    State s = sim.getDroneState();
    EXPECT_GT(s.pitch, 0.0);
    EXPECT_GT(s.vx,    0.0);
}

// Thrust > 1.0 produces net upward acceleration from hover.
TEST(KinematicSim, ThrustAboveOneClimbs)
{
    auto sim = make_sim();

    for (int i = 0; i < kSettle; ++i) sim.update({0,0,0,1.0}, kDt);
    for (int i = 0; i < 200;     ++i) sim.update({0,0,0,1.5}, kDt);

    EXPECT_GT(sim.getDroneState().vz, 0.1);
}

// Second-order step response must settle to within 1% of the command.
// Settling time for zeta=0.7: ts ≈ 5.8 / (zeta*wn).
TEST(KinematicSim, StepResponseSettles)
{
    const double wn   = 25.0;
    const double zeta = 0.7;
    const double dt   = 0.001;
    // Run for 3× the theoretical settling time to be safe.
    const int    N    = static_cast<int>(3.0 * 5.8 / (zeta * wn) / dt);

    auto sim = make_sim(wn, zeta);
    ControlCommand cmd{1.0, 0.0, 0.0, 1.0};  // step: roll_rate = 1 rad/s

    for (int i = 0; i < N; ++i)
        sim.update(cmd, dt);

    EXPECT_NEAR(sim.getDroneState().wx, 1.0, 0.01);
}

// For an underdamped system (zeta < 1) the step response must overshoot.
// Peak overshoot: OS = exp(-π·zeta / sqrt(1-zeta²)) × 100%.
// For zeta=0.7: OS ≈ 4.3%, so peak wx > 1.0.
TEST(KinematicSim, UnderdampedResponseOvershoots)
{
    const double wn   = 25.0;
    const double zeta = 0.7;
    const double dt   = 0.001;
    // Run past the peak time: tp = π / (wn * sqrt(1-zeta²))
    const double tp = M_PI / (wn * std::sqrt(1.0 - zeta * zeta));
    const int    N  = static_cast<int>(2.0 * tp / dt);

    auto sim = make_sim(wn, zeta);
    ControlCommand cmd{1.0, 0.0, 0.0, 1.0};

    double peak = 0.0;
    for (int i = 0; i < N; ++i)
    {
        sim.update(cmd, dt);
        peak = std::max(peak, sim.getDroneState().wx);
    }

    EXPECT_GT(peak, 1.0);  // did overshoot
}

// Critically damped (zeta=1) response must NOT overshoot.
TEST(KinematicSim, CriticallyDampedNoOvershoot)
{
    const double wn   = 25.0;
    const double zeta = 1.0;
    const double dt   = 0.001;
    const int    N    = static_cast<int>(1.0 / dt);  // 1 s

    auto sim = make_sim(wn, zeta);
    ControlCommand cmd{1.0, 0.0, 0.0, 1.0};

    double peak = 0.0;
    for (int i = 0; i < N; ++i)
    {
        sim.update(cmd, dt);
        peak = std::max(peak, sim.getDroneState().wx);
    }

    EXPECT_LE(peak, 1.001);  // no overshoot (tiny tolerance for numerics)
}
