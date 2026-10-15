// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "drone_sim/kinematics.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;

drone_sim::KinematicsConfig fast_config() {
  drone_sim::KinematicsConfig config;
  config.max_speed_m_s = 4.0;
  config.max_yaw_rate_rad_s = 2.0;
  config.position_gain = 3.0;
  config.velocity_tau_s = 0.05;
  return config;
}

}  // namespace

TEST(Kinematics, RejectsInvalidConfigAndDt) {
  drone_sim::KinematicsConfig config;
  config.velocity_tau_s = 0.0;
  // Parentheses keep this an expression. `Type(name);` declares a variable.
  EXPECT_THROW((drone_sim::KinematicModel(config)), std::invalid_argument);

  drone_sim::KinematicModel model;
  EXPECT_THROW(model.step(-0.01), std::invalid_argument);
  EXPECT_THROW(model.set_setpoint(drone_sim::Setpoint{0.0, 0.0, 0.0, NAN, 0.0}),
               std::invalid_argument);
}

TEST(Kinematics, MovesTowardSetpointAndStaysOnAxis) {
  drone_sim::KinematicModel model(fast_config());
  model.set_setpoint(drone_sim::Setpoint{5.0, 0.0, 0.0, 0.0, 0.0});
  model.step(0.05);
  EXPECT_GT(model.pose().x, 0.0);
  EXPECT_NEAR(model.pose().y, 0.0, 1e-9);
  EXPECT_NEAR(model.pose().z, 0.0, 1e-9);
}

TEST(Kinematics, ReachesSetpointWithinSpeedCap) {
  auto config = fast_config();
  config.max_speed_m_s = 2.0;
  drone_sim::KinematicModel model(config);
  model.set_setpoint(drone_sim::Setpoint{3.0, 4.0, 0.0, 0.0, 0.0});

  for (int i = 0; i < 1000; ++i) {
    model.step(0.01);
    const auto& pose = model.pose();
    const double speed = std::sqrt(pose.vx * pose.vx + pose.vy * pose.vy + pose.vz * pose.vz);
    EXPECT_LE(speed, config.max_speed_m_s + 1e-9);
  }

  const auto& pose = model.pose();
  EXPECT_NEAR(pose.x, 3.0, 0.05);
  EXPECT_NEAR(pose.y, 4.0, 0.05);
  EXPECT_NEAR(pose.z, 0.0, 0.05);
}

TEST(Kinematics, SetpointSpeedOverridesConfigCap) {
  auto config = fast_config();
  config.max_speed_m_s = 5.0;
  drone_sim::KinematicModel model(config);
  model.set_setpoint(drone_sim::Setpoint{20.0, 0.0, 0.0, 0.0, 1.0});

  double peak = 0.0;
  for (int i = 0; i < 200; ++i) {
    model.step(0.02);
    peak = std::max(peak, std::abs(model.pose().vx));
  }
  EXPECT_LE(peak, 1.0 + 1e-9);
  EXPECT_GT(peak, 0.5);
}

TEST(Kinematics, YawTakesTheShortWayAcrossTheBranchCut) {
  EXPECT_NEAR(drone_sim::wrap_pi(3.0 * kPi), -kPi, 1e-9);
  EXPECT_NEAR(drone_sim::wrap_pi(0.0), 0.0, 1e-12);

  drone_sim::KinematicModel model(fast_config());
  drone_sim::Pose initial;
  initial.yaw = 3.0;
  model.set_pose(initial);
  model.set_setpoint(drone_sim::Setpoint{0.0, 0.0, 0.0, -3.0, 0.0});

  model.step(0.05);
  // +3 rad to -3 rad is a short positive step across +pi, not a long negative sweep.
  EXPECT_GT(model.pose().yaw, 3.0 - 1e-9);

  for (int i = 0; i < 400; ++i) {
    model.step(0.01);
  }
  EXPECT_NEAR(model.pose().yaw, -3.0, 0.02);
}

TEST(Kinematics, BodyFrameOfANorthFacingVehicle) {
  const double yaw = kPi / 2.0;
  const auto forward = drone_sim::world_velocity_to_body(0.0, 2.0, 0.5, yaw);
  EXPECT_NEAR(forward.x, 2.0, 1e-9);
  EXPECT_NEAR(forward.y, 0.0, 1e-9);
  EXPECT_NEAR(forward.z, 0.5, 1e-9);

  const auto right = drone_sim::world_velocity_to_body(1.5, 0.0, 0.0, yaw);
  EXPECT_NEAR(right.x, 0.0, 1e-9);
  EXPECT_NEAR(right.y, -1.5, 1e-9);
}

TEST(Kinematics, HoldsPositionWhenNoSetpointWasSent) {
  drone_sim::KinematicModel model(fast_config());
  model.step(0.1);
  EXPECT_NEAR(model.pose().x, 0.0, 1e-9);
  EXPECT_NEAR(model.pose().vx, 0.0, 1e-9);
}
