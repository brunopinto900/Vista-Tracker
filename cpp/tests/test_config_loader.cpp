#include <gtest/gtest.h>
#include "config/ConfigLoader.hpp"

static const std::string kFix = TEST_FIXTURES_DIR;

// All fields in a standalone file (no base:) are applied to the Config.
TEST(ConfigLoader, StandaloneLoadsAllFields)
{
    Config cfg = ConfigLoader::load(kFix + "/standalone.yaml");

    EXPECT_DOUBLE_EQ(cfg.sim.dt,                    0.05);
    EXPECT_DOUBLE_EQ(cfg.sim.T,                    10.0);
    EXPECT_EQ       (cfg.estimator.horizon,         10);
    EXPECT_EQ       (cfg.estimator.motion_model,    "CA");
    EXPECT_DOUBLE_EQ(cfg.controller.kp_pos, 1.5);
    EXPECT_DOUBLE_EQ(cfg.controller.kp_vel, 1.0);
    EXPECT_DOUBLE_EQ(cfg.drone_init.x,              1.0);
    EXPECT_DOUBLE_EQ(cfg.drone_init.y,              2.0);
    EXPECT_DOUBLE_EQ(cfg.drone_init.z,              3.0);
    EXPECT_EQ       (cfg.world.obstacles.size(),    0u);
}

// Fields not mentioned in the scenario file are inherited from base.
// Fields that are mentioned replace the base value.
TEST(ConfigLoader, BaseInheritanceThenOverride)
{
    Config cfg = ConfigLoader::load(kFix + "/override_kp.yaml");

    EXPECT_DOUBLE_EQ(cfg.sim.dt,          0.05);  // from base
    EXPECT_DOUBLE_EQ(cfg.controller.ki_vel,  0.0);  // from base
    EXPECT_DOUBLE_EQ(cfg.controller.kp_pos, 2.5);  // overridden by scenario
    EXPECT_EQ       (cfg.estimator.horizon, 5);   // from base
}

// A scenario that declares world: with obstacles: loads exactly those obstacles.
TEST(ConfigLoader, ScenarioObstaclesLoadedCorrectly)
{
    Config cfg = ConfigLoader::load(kFix + "/world_with_obstacle.yaml");

    ASSERT_EQ(cfg.world.obstacles.size(), 1u);
    EXPECT_DOUBLE_EQ(cfg.world.obstacles[0].x,    4.0);
    EXPECT_DOUBLE_EQ(cfg.world.obstacles[0].y,    0.0);
    EXPECT_DOUBLE_EQ(cfg.world.obstacles[0].size, 1.5);
}

// Grid bounds from a scenario are applied on top of the base (which has none).
TEST(ConfigLoader, GridBoundsLoadedFromScenario)
{
    Config cfg = ConfigLoader::load(kFix + "/world_with_obstacle.yaml");

    EXPECT_DOUBLE_EQ(cfg.world.grid.x_min, -5.0);
    EXPECT_DOUBLE_EQ(cfg.world.grid.x_max, 10.0);
    EXPECT_DOUBLE_EQ(cfg.world.grid.y_min, -5.0);
    EXPECT_DOUBLE_EQ(cfg.world.grid.y_max,  5.0);
}
