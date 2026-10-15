// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>

#include <stdexcept>

#include "drone_sim/battery_model.hpp"

TEST(BatteryModel, HoverDrainsSlowerThanMoving) {
  drone_sim::BatteryConfig config;
  config.percent = 100.0;
  config.hover_drain_pct_per_s = 1.0;
  config.move_drain_pct_per_s = 1.0;
  config.low_percent = 25.0;
  config.critical_percent = 10.0;

  drone_sim::BatteryModel hover(config);
  drone_sim::BatteryModel moving(config);
  hover.update(0.0, 1.0);
  moving.update(2.0, 1.0);

  EXPECT_NEAR(hover.percent(), 99.0, 1e-9);
  EXPECT_NEAR(moving.percent(), 97.0, 1e-9);
  EXPECT_LT(moving.percent(), hover.percent());
  EXPECT_NEAR(moving.last_drain_pct_per_s(), 3.0, 1e-9);
}

TEST(BatteryModel, ClampsAtEmptyAndFull) {
  drone_sim::BatteryConfig config;
  config.percent = 1.0;
  config.hover_drain_pct_per_s = 10.0;
  config.move_drain_pct_per_s = 0.0;
  drone_sim::BatteryModel model(config);
  model.update(0.0, 5.0);
  EXPECT_DOUBLE_EQ(model.percent(), 0.0);
  EXPECT_TRUE(model.critical());

  drone_sim::BatteryConfig overfull;
  overfull.percent = 140.0;
  overfull.hover_drain_pct_per_s = 0.0;
  drone_sim::BatteryModel clamped(overfull);
  EXPECT_DOUBLE_EQ(clamped.percent(), 100.0);
}

TEST(BatteryModel, RejectsBadInputs) {
  drone_sim::BatteryConfig config;
  config.hover_drain_pct_per_s = -1.0;
  // Parentheses keep this an expression. `Type(name);` declares a variable.
  EXPECT_THROW((drone_sim::BatteryModel(config)), std::invalid_argument);

  drone_sim::BatteryConfig inverted;
  inverted.low_percent = 10.0;
  inverted.critical_percent = 20.0;
  EXPECT_THROW((drone_sim::BatteryModel(inverted)), std::invalid_argument);

  drone_sim::BatteryModel model;
  EXPECT_THROW(model.update(-1.0, 0.1), std::invalid_argument);
  EXPECT_THROW(model.update(0.0, -0.1), std::invalid_argument);
}

TEST(BatteryModel, LowAndCriticalFlagsFollowThresholds) {
  drone_sim::BatteryConfig config;
  config.percent = 30.0;
  config.hover_drain_pct_per_s = 10.0;
  config.move_drain_pct_per_s = 0.0;
  config.low_percent = 25.0;
  config.critical_percent = 10.0;
  drone_sim::BatteryModel model(config);
  EXPECT_FALSE(model.low());
  model.update(0.0, 1.0);
  EXPECT_NEAR(model.percent(), 20.0, 1e-9);
  EXPECT_TRUE(model.low());
  EXPECT_FALSE(model.critical());
  model.update(0.0, 1.0);
  EXPECT_TRUE(model.critical());
}
