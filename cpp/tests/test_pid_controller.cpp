#include <gtest/gtest.h>
#include <cmath>
#include "control/PIDController.hpp"
#include "models/State.hpp"
#include "models/Reference.hpp"

static constexpr double kG  = 9.81;
static constexpr double kDt = 0.05;

// kp=1, ki=0, kd=0 → stateless; att_kp=1, yaw_kp=1 for easy hand-calculation.
static PIDController make_ctrl(double kp = 1.0, double att_kp = 1.0, double yaw_kp = 1.0)
{
    return PIDController(kp, 0.0, 0.0, att_kp, yaw_kp);
}

// At zero position error, thrust must equal gravity-normalised hover (1.0)
// and all rate commands must be zero.
TEST(PIDController, HoverAtZeroError)
{
    auto ctrl = make_ctrl();
    State     s{};  s.z = 2.0;
    Reference r{};  r.x = s.x; r.y = s.y; r.z = s.z;

    auto cmd = ctrl.update(s, r, kDt);

    EXPECT_NEAR(cmd.thrust,     1.0, 1e-6);
    EXPECT_NEAR(cmd.roll_rate,  0.0, 1e-6);
    EXPECT_NEAR(cmd.pitch_rate, 0.0, 1e-6);
    EXPECT_NEAR(cmd.yaw_rate,   0.0, 1e-6);
}

// With yaw=0 (nose along world +x), a +x position error must produce a
// positive pitch command and no roll.
TEST(PIDController, YawZeroForwardErrorProducesPitch)
{
    auto ctrl = make_ctrl();
    State     s{};  s.yaw = 0.0;
    Reference r{};  r.x = 1.0;  // 1 m ahead in world +x

    auto cmd = ctrl.update(s, r, kDt);

    EXPECT_GT(cmd.pitch_rate, 0.0);
    EXPECT_NEAR(cmd.roll_rate, 0.0, 1e-6);
}

// With yaw=π/2 (nose along world +y), the same +x world error must produce
// roll, not pitch.  This directly tests the body-frame rotation that decouples
// the axes: without it, pitch and roll would be swapped.
TEST(PIDController, YawNinetyForwardErrorProducesRoll)
{
    auto ctrl = make_ctrl();
    State     s{};  s.yaw = M_PI / 2.0;
    Reference r{};  r.x = 1.0;  // +x world error; drone faces +y

    auto cmd = ctrl.update(s, r, kDt);

    EXPECT_GT(cmd.roll_rate,  0.0);
    EXPECT_NEAR(cmd.pitch_rate, 0.0, 1e-6);
}

// Yaw error that straddles the ±π discontinuity must wrap to the shortest arc.
// drone yaw ≈ +π,  ref yaw ≈ −π  →  true error ≈ +0.1 rad (not ±2π − 0.1).
TEST(PIDController, YawErrorWrapsAtPiBoundary)
{
    auto ctrl = make_ctrl(/*kp*/1.0, /*att_kp*/1.0, /*yaw_kp*/1.0);
    State     s{};  s.yaw   =  M_PI - 0.05;
    Reference r{};  r.yaw   = -(M_PI - 0.05);

    auto cmd = ctrl.update(s, r, kDt);

    // shortest-path error = 0.1 rad, yaw_kp=1 → yaw_rate ≈ 0.1
    EXPECT_NEAR(cmd.yaw_rate, 0.1, 0.01);
}
