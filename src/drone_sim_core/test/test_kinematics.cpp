// Copyright 2026 Drone Waypoint Demo

#include <algorithm>
#include <cmath>
#include <limits>

#include "gtest/gtest.h"

#include "drone_sim_core/kinematics.hpp"

namespace
{

using drone_sim_core::FlightMode;
using drone_sim_core::KinematicDrone;
using drone_sim_core::Setpoint;
using drone_sim_core::SimParams;
using drone_sim_core::Vec3;
using drone_sim_core::outside_geofence;

Setpoint point(double x, double y, double z, double yaw = 0.0)
{
  Setpoint setpoint;
  setpoint.position = {x, y, z};
  setpoint.yaw = yaw;
  return setpoint;
}

void step(KinematicDrone & drone, double seconds, double dt = 0.05)
{
  const int ticks = static_cast<int>(std::ceil(seconds / dt));
  for (int i = 0; i < ticks; ++i) {
    drone.update(dt);
  }
}

}  // namespace

TEST(Geofence, BoundaryIsInsideAndJustBeyondIsOutside)
{
  SimParams params;
  params.geofence_radius = 10.0;
  params.geofence_max_alt = 30.0;
  params.geofence_min_alt = -0.5;
  EXPECT_FALSE(outside_geofence(Vec3{10.0, 0.0, 30.0}, params));
  EXPECT_TRUE(outside_geofence(Vec3{10.01, 0.0, 0.0}, params));
  EXPECT_TRUE(outside_geofence(Vec3{0.0, 0.0, 30.01}, params));
  EXPECT_TRUE(outside_geofence(Vec3{0.0, 0.0, -0.51}, params));
}

TEST(KinematicDrone, RejectsInconsistentParameters)
{
  SimParams params;
  params.battery_rtl_percent = 10.0;
  params.battery_critical_percent = 10.0;
  EXPECT_THROW(KinematicDrone drone(params), std::invalid_argument);

  params = SimParams();
  params.geofence_radius = 0.0;
  EXPECT_THROW(KinematicDrone drone(params), std::invalid_argument);

  params = SimParams();
  params.home.z = 100.0;
  params.geofence_max_alt = 30.0;
  EXPECT_THROW(KinematicDrone drone(params), std::invalid_argument);
}

TEST(KinematicDrone, StaysPutUntilASetpointIsAccepted)
{
  KinematicDrone drone(SimParams{});
  step(drone, 2.0);
  const auto state = drone.state();
  EXPECT_EQ(state.mode, FlightMode::Disarmed);
  EXPECT_DOUBLE_EQ(state.position.x, 0.0);
  EXPECT_DOUBLE_EQ(state.position.y, 0.0);
  EXPECT_DOUBLE_EQ(state.position.z, 0.0);
  EXPECT_DOUBLE_EQ(state.battery_percent, 100.0);
}

TEST(KinematicDrone, TracksSetpointWithinTolerance)
{
  SimParams params;
  params.max_horizontal_speed = 5.0;
  params.max_vertical_speed = 5.0;
  params.position_p_gain = 2.0;
  KinematicDrone drone(params);
  ASSERT_TRUE(drone.set_setpoint(point(10.0, -4.0, 6.0)));
  step(drone, 20.0);
  const auto state = drone.state();
  EXPECT_EQ(state.mode, FlightMode::Flying);
  EXPECT_NEAR(state.position.x, 10.0, 0.15);
  EXPECT_NEAR(state.position.y, -4.0, 0.15);
  EXPECT_NEAR(state.position.z, 6.0, 0.15);
  EXPECT_NEAR(state.velocity.x, 0.0, 0.05);
  EXPECT_NEAR(state.velocity.y, 0.0, 0.05);
  EXPECT_NEAR(state.velocity.z, 0.0, 0.05);
}

TEST(KinematicDrone, CapsHorizontalSpeed)
{
  SimParams params;
  params.max_horizontal_speed = 3.0;
  params.position_p_gain = 10.0;
  KinematicDrone drone(params);
  ASSERT_TRUE(drone.set_setpoint(point(100.0, 0.0, 0.0)));
  drone.update(0.05);
  const auto state = drone.state();
  EXPECT_NEAR(std::hypot(state.velocity.x, state.velocity.y), 3.0, 1e-9);
}

TEST(KinematicDrone, DrainsBatteryInFlightAndRechargesOnTheGround)
{
  SimParams params;
  params.battery_flight_percent_per_sec = 2.0;
  params.battery_idle_percent_per_sec = 0.0;
  params.recharge_percent_per_sec = 1.0;
  KinematicDrone drone(params);
  ASSERT_TRUE(drone.set_setpoint(point(8.0, 0.0, 4.0)));
  step(drone, 1.0);
  EXPECT_LT(drone.state().battery_percent, 100.0);

  SimParams parked;
  parked.battery_initial = 40.0;
  parked.recharge_percent_per_sec = 5.0;
  parked.battery_idle_percent_per_sec = 0.0;
  KinematicDrone sitting(parked);
  step(sitting, 1.0);
  EXPECT_NEAR(sitting.state().battery_percent, 45.0, 1e-6);
}

TEST(KinematicDrone, LowBatteryTriggersReturnToHome)
{
  SimParams params;
  params.battery_initial = 30.0;
  params.battery_rtl_percent = 28.0;
  params.battery_critical_percent = 10.0;
  params.battery_flight_percent_per_sec = 5.0;
  params.battery_idle_percent_per_sec = 0.0;
  params.max_horizontal_speed = 4.0;
  KinematicDrone drone(params);
  ASSERT_TRUE(drone.set_setpoint(point(12.0, 0.0, 5.0)));
  double farthest = 0.0;
  bool saw_rtl = false;
  for (int i = 0; i < 600; ++i) {
    drone.update(0.05);
    farthest = std::max(farthest, drone.state().position.x);
    if (drone.state().mode == FlightMode::ReturnToHome ||
      drone.state().mode == FlightMode::Failsafe)
    {
      saw_rtl = true;
    }
    if (drone.state().mode == FlightMode::Landed) {
      break;
    }
  }
  EXPECT_GT(farthest, 0.5);
  EXPECT_TRUE(saw_rtl);
  EXPECT_EQ(drone.state().mode, FlightMode::Landed);
  EXPECT_TRUE(drone.state().failsafe_active);
  EXPECT_NEAR(drone.state().position.x, 0.0, 0.3);
  EXPECT_NEAR(drone.state().position.z, 0.0, 1e-6);
}

TEST(KinematicDrone, CriticalBatteryUsesFailsafeDescent)
{
  SimParams params;
  // Climb in place until the RTL threshold, then keep descending long enough
  // to cross the lower critical threshold before touchdown.
  params.battery_initial = 60.0;
  params.battery_rtl_percent = 50.0;
  params.battery_critical_percent = 42.0;
  params.battery_flight_percent_per_sec = 10.0;
  params.battery_idle_percent_per_sec = 0.0;
  params.max_horizontal_speed = 4.0;
  params.max_vertical_speed = 1.0;
  params.geofence_max_alt = 40.0;
  KinematicDrone drone(params);
  ASSERT_TRUE(drone.set_setpoint(point(0.0, 0.0, 15.0)));
  bool saw_failsafe = false;
  for (int i = 0; i < 800; ++i) {
    drone.update(0.05);
    if (drone.state().mode == FlightMode::Failsafe) {
      saw_failsafe = true;
    }
    if (drone.state().mode == FlightMode::Landed) {
      break;
    }
  }
  EXPECT_TRUE(saw_failsafe);
  EXPECT_EQ(drone.state().mode, FlightMode::Landed);
  EXPECT_TRUE(drone.state().failsafe_active);
  EXPECT_NEAR(drone.state().position.x, 0.0, 0.4);
  EXPECT_NEAR(drone.state().position.y, 0.0, 0.4);
}

TEST(KinematicDrone, GeofenceBreachReturnsHome)
{
  SimParams params;
  params.geofence_radius = 6.0;
  params.max_horizontal_speed = 8.0;
  params.max_vertical_speed = 4.0;
  params.position_p_gain = 2.0;
  params.battery_flight_percent_per_sec = 0.0;
  params.battery_idle_percent_per_sec = 0.0;
  KinematicDrone drone(params);
  ASSERT_TRUE(drone.set_setpoint(point(30.0, 0.0, 3.0)));
  bool saw_breach = false;
  for (int i = 0; i < 800; ++i) {
    drone.update(0.05);
    if (drone.state().geofence_breached) {
      saw_breach = true;
    }
    if (saw_breach && drone.state().mode == FlightMode::Landed) {
      break;
    }
  }
  EXPECT_TRUE(saw_breach);
  EXPECT_EQ(drone.state().mode, FlightMode::Landed);
  EXPECT_TRUE(drone.state().failsafe_active);
  EXPECT_NEAR(drone.state().position.x, 0.0, 0.3);
  EXPECT_NEAR(drone.state().position.z, 0.0, 1e-6);
  EXPECT_FALSE(drone.state().geofence_breached);
}

TEST(KinematicDrone, AltitudeBreachReturnsHome)
{
  SimParams params;
  params.geofence_max_alt = 5.0;
  params.max_vertical_speed = 3.0;
  params.battery_flight_percent_per_sec = 0.0;
  params.battery_idle_percent_per_sec = 0.0;
  KinematicDrone drone(params);
  ASSERT_TRUE(drone.set_setpoint(point(0.0, 0.0, 20.0)));
  double highest = 0.0;
  bool saw_breach = false;
  for (int i = 0; i < 800; ++i) {
    drone.update(0.05);
    highest = std::max(highest, drone.state().position.z);
    if (drone.state().geofence_breached) {
      saw_breach = true;
    }
    if (saw_breach && drone.state().mode == FlightMode::Landed) {
      break;
    }
  }
  EXPECT_TRUE(saw_breach);
  EXPECT_GT(highest, 5.0);
  EXPECT_LT(highest, 5.0 + params.max_vertical_speed * 0.05 + 0.05);
  EXPECT_EQ(drone.state().mode, FlightMode::Landed);
  EXPECT_TRUE(drone.state().failsafe_active);
}

TEST(KinematicDrone, PilotRtlLandsWithoutFailsafeLatch)
{
  SimParams params;
  params.battery_flight_percent_per_sec = 0.0;
  params.battery_idle_percent_per_sec = 0.0;
  params.max_horizontal_speed = 6.0;
  params.max_vertical_speed = 4.0;
  KinematicDrone drone(params);
  ASSERT_TRUE(drone.set_setpoint(point(8.0, -3.0, 5.0)));
  step(drone, 12.0);
  ASSERT_EQ(drone.state().mode, FlightMode::Flying);
  drone.command_rtl();
  for (int i = 0; i < 800; ++i) {
    drone.update(0.05);
    if (drone.state().mode == FlightMode::Landed) {
      break;
    }
  }
  EXPECT_EQ(drone.state().mode, FlightMode::Landed);
  EXPECT_FALSE(drone.state().failsafe_active);
  EXPECT_NEAR(drone.state().position.x, 0.0, 0.3);
  EXPECT_NEAR(drone.state().position.y, 0.0, 0.3);
  EXPECT_NEAR(drone.state().position.z, 0.0, 1e-6);
}

TEST(KinematicDrone, SetpointsAreIgnoredDuringReturn)
{
  SimParams params;
  params.battery_flight_percent_per_sec = 0.0;
  params.battery_idle_percent_per_sec = 0.0;
  params.max_horizontal_speed = 6.0;
  params.max_vertical_speed = 4.0;
  KinematicDrone drone(params);
  ASSERT_TRUE(drone.set_setpoint(point(6.0, 0.0, 4.0)));
  step(drone, 8.0);
  drone.command_rtl();
  EXPECT_FALSE(drone.set_setpoint(point(80.0, 40.0, 10.0)));
  for (int i = 0; i < 800; ++i) {
    drone.update(0.05);
    if (drone.state().mode == FlightMode::Landed) {
      break;
    }
  }
  EXPECT_EQ(drone.state().mode, FlightMode::Landed);
  EXPECT_NEAR(drone.state().position.x, 0.0, 0.3);
  EXPECT_LT(drone.state().position.x, 10.0);
}

TEST(KinematicDrone, RefusesToArmBelowCriticalBattery)
{
  SimParams params;
  params.battery_initial = 5.0;
  params.battery_critical_percent = 10.0;
  params.battery_rtl_percent = 20.0;
  KinematicDrone drone(params);
  EXPECT_FALSE(drone.set_setpoint(point(10.0, 0.0, 5.0)));
  step(drone, 2.0);
  EXPECT_EQ(drone.state().mode, FlightMode::Disarmed);
  EXPECT_DOUBLE_EQ(drone.state().position.x, 0.0);
  EXPECT_TRUE(drone.state().failsafe_active);
}

TEST(KinematicDrone, IgnoresNonFiniteSetpointAndBadDt)
{
  KinematicDrone drone(SimParams{});
  Setpoint bad = point(1.0, 0.0, 1.0);
  bad.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(drone.set_setpoint(bad));
  ASSERT_TRUE(drone.set_setpoint(point(5.0, 0.0, 2.0)));
  const auto armed = drone.state();
  drone.update(0.0);
  drone.update(-0.2);
  drone.update(std::numeric_limits<double>::quiet_NaN());
  EXPECT_DOUBLE_EQ(drone.state().position.x, armed.position.x);
  EXPECT_EQ(drone.state().mode, FlightMode::Flying);
}

TEST(KinematicDrone, YawTracksSetpoint)
{
  SimParams params;
  params.max_yaw_rate = 2.0;
  params.yaw_p_gain = 4.0;
  params.battery_flight_percent_per_sec = 0.0;
  params.battery_idle_percent_per_sec = 0.0;
  KinematicDrone drone(params);
  ASSERT_TRUE(drone.set_setpoint(point(0.0, 0.0, 0.0, 1.5707963267948966)));
  step(drone, 5.0);
  EXPECT_NEAR(drone.state().yaw, 1.5707963267948966, 0.05);
}
