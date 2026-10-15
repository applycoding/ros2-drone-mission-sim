// Copyright 2026 Drone Waypoint Demo

#include "drone_sim_core/kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace drone_sim_core
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

double wrap_pi(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double horizontal_distance(const Vec3 & a, const Vec3 & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

bool finite_setpoint(const Setpoint & setpoint)
{
  return std::isfinite(setpoint.position.x) &&
         std::isfinite(setpoint.position.y) &&
         std::isfinite(setpoint.position.z) &&
         std::isfinite(setpoint.yaw);
}

void validate_params(const SimParams & params)
{
  if (!(params.max_horizontal_speed > 0.0) || !(params.max_vertical_speed > 0.0)) {
    throw std::invalid_argument("speed limits must be positive");
  }
  if (!(params.max_yaw_rate > 0.0) || !(params.position_p_gain > 0.0) ||
    !(params.yaw_p_gain > 0.0))
  {
    throw std::invalid_argument("gains and yaw rate must be positive");
  }
  if (!(params.geofence_radius > 0.0) ||
    !(params.geofence_max_alt > params.geofence_min_alt))
  {
    throw std::invalid_argument("geofence limits are inconsistent");
  }
  if (!(params.rtl_acceptance > 0.0) || params.land_altitude < 0.0) {
    throw std::invalid_argument("landing geometry is inconsistent");
  }
  if (!(params.battery_rtl_percent > params.battery_critical_percent) ||
    params.battery_critical_percent < 0.0 || params.battery_rtl_percent > 100.0)
  {
    throw std::invalid_argument("battery thresholds are inconsistent");
  }
  if (params.battery_idle_percent_per_sec < 0.0 ||
    params.battery_flight_percent_per_sec < 0.0 ||
    params.recharge_percent_per_sec < 0.0)
  {
    throw std::invalid_argument("battery rates must be non-negative");
  }
  if (outside_geofence(params.home, params)) {
    throw std::invalid_argument("home is outside the geofence");
  }
}

bool is_airborne(FlightMode mode)
{
  return mode == FlightMode::Flying ||
         mode == FlightMode::ReturnToHome ||
         mode == FlightMode::Failsafe;
}

}  // namespace

bool outside_geofence(const Vec3 & position, const SimParams & params)
{
  if (horizontal_distance(position, params.home) > params.geofence_radius) {
    return true;
  }
  if (position.z > params.geofence_max_alt || position.z < params.geofence_min_alt) {
    return true;
  }
  return false;
}

KinematicDrone::KinematicDrone(SimParams params)
: params_(params)
{
  validate_params(params_);
  position_ = params_.home;
  battery_ = std::clamp(params_.battery_initial, 0.0, 100.0);
  active_setpoint_.position = params_.home;
}

bool KinematicDrone::arm()
{
  if (battery_ <= params_.battery_critical_percent || outside_geofence(position_, params_)) {
    failsafe_latched_ = true;
    return false;
  }
  failsafe_latched_ = false;
  emergency_land_ = false;
  if (mode_ == FlightMode::Disarmed || mode_ == FlightMode::Landed) {
    mode_ = FlightMode::Flying;
  }
  return true;
}

bool KinematicDrone::set_setpoint(const Setpoint & setpoint)
{
  if (!setpoint.valid || !finite_setpoint(setpoint)) {
    return false;
  }
  if (setpoint.return_to_home) {
    if (mode_ != FlightMode::Flying && mode_ != FlightMode::ReturnToHome) {
      return mode_ == FlightMode::Landed || mode_ == FlightMode::Disarmed;
    }
    command_rtl();
    return true;
  }
  if (mode_ == FlightMode::ReturnToHome || mode_ == FlightMode::Failsafe) {
    return false;
  }
  if ((mode_ == FlightMode::Disarmed || mode_ == FlightMode::Landed) && !arm()) {
    return false;
  }
  active_setpoint_ = setpoint;
  return mode_ == FlightMode::Flying;
}

void KinematicDrone::command_rtl()
{
  if (mode_ == FlightMode::Flying || mode_ == FlightMode::ReturnToHome) {
    mode_ = FlightMode::ReturnToHome;
  }
}

void KinematicDrone::update(double dt)
{
  if (!std::isfinite(dt) || dt <= 0.0) {
    return;
  }
  // A stalled timer should not teleport the kinematic model.
  dt = std::min(dt, 0.25);
  const bool airborne = is_airborne(mode_);
  if (airborne) {
    integrate(dt);
  } else {
    velocity_ = {};
    if (mode_ == FlightMode::Landed) {
      position_.z = params_.home.z;
    }
  }
  update_battery(dt, airborne);
  apply_automatic_mode_changes();
  maybe_land();
}

void KinematicDrone::integrate(double dt)
{
  Vec3 target = active_setpoint_.position;
  double target_yaw = active_setpoint_.yaw;
  double horizontal_cap = params_.max_horizontal_speed;

  if (emergency_land_) {
    target = position_;
    target.z = params_.home.z;
    target_yaw = yaw_;
  } else if (mode_ == FlightMode::Failsafe) {
    target = params_.home;
    horizontal_cap *= 0.4;
    target_yaw = yaw_toward(params_.home);
  } else if (mode_ == FlightMode::ReturnToHome) {
    const double horiz = horizontal_distance(position_, params_.home);
    target.x = params_.home.x;
    target.y = params_.home.y;
    target.z = horiz <= params_.rtl_acceptance ? params_.home.z : transit_altitude();
    target_yaw = yaw_toward(params_.home);
  }

  if (mode_ != FlightMode::Flying) {
    target.z = std::clamp(target.z, params_.home.z, params_.geofence_max_alt);
  }

  const double error_x = target.x - position_.x;
  const double error_y = target.y - position_.y;
  const double error_z = target.z - position_.z;
  const double horiz = std::hypot(error_x, error_y);
  const double horizontal_speed = std::min(horizontal_cap, horiz * params_.position_p_gain);
  if (horiz > 1e-6) {
    velocity_.x = horizontal_speed * error_x / horiz;
    velocity_.y = horizontal_speed * error_y / horiz;
  } else {
    velocity_.x = 0.0;
    velocity_.y = 0.0;
  }
  velocity_.z = std::clamp(
    error_z * params_.position_p_gain,
    -params_.max_vertical_speed,
    params_.max_vertical_speed);

  position_.x += velocity_.x * dt;
  position_.y += velocity_.y * dt;
  position_.z += velocity_.z * dt;
  if (position_.z < params_.home.z) {
    position_.z = params_.home.z;
    velocity_.z = 0.0;
  }

  const double yaw_error = wrap_pi(target_yaw - yaw_);
  const double yaw_rate = std::clamp(
    yaw_error * params_.yaw_p_gain,
    -params_.max_yaw_rate,
    params_.max_yaw_rate);
  yaw_ = wrap_pi(yaw_ + yaw_rate * dt);
}

void KinematicDrone::update_battery(double dt, bool airborne)
{
  if (airborne) {
    const double horizontal = std::hypot(velocity_.x, velocity_.y);
    const double horizontal_factor =
      horizontal / std::max(params_.max_horizontal_speed, 1e-6);
    const double vertical_factor =
      std::abs(velocity_.z) / std::max(params_.max_vertical_speed, 1e-6);
    const double effort = std::clamp(
      std::max(horizontal_factor, 0.25) + 0.25 * vertical_factor,
      0.0,
      1.5);
    const double drain = params_.battery_idle_percent_per_sec +
      params_.battery_flight_percent_per_sec * effort;
    battery_ -= drain * dt;
  } else if (params_.recharge_percent_per_sec > 0.0) {
    battery_ += params_.recharge_percent_per_sec * dt;
  }
  battery_ = std::clamp(battery_, 0.0, 100.0);
}

void KinematicDrone::apply_automatic_mode_changes()
{
  if (!is_airborne(mode_)) {
    return;
  }
  if (battery_ <= 0.0) {
    emergency_land_ = true;
    mode_ = FlightMode::Failsafe;
    failsafe_latched_ = true;
    return;
  }
  if (battery_ <= params_.battery_critical_percent) {
    mode_ = FlightMode::Failsafe;
    failsafe_latched_ = true;
    return;
  }
  if (mode_ == FlightMode::Flying &&
    (battery_ <= params_.battery_rtl_percent || outside_geofence(position_, params_)))
  {
    mode_ = FlightMode::ReturnToHome;
    failsafe_latched_ = true;
  }
}

void KinematicDrone::maybe_land()
{
  if (mode_ != FlightMode::ReturnToHome && mode_ != FlightMode::Failsafe) {
    return;
  }
  const double horiz = horizontal_distance(position_, params_.home);
  const bool low = (position_.z - params_.home.z) <= params_.land_altitude;
  const bool over_home = horiz <= params_.rtl_acceptance;
  if ((low && over_home) || (emergency_land_ && low)) {
    if (over_home) {
      position_.x = params_.home.x;
      position_.y = params_.home.y;
    }
    position_.z = params_.home.z;
    velocity_ = {};
    mode_ = FlightMode::Landed;
    emergency_land_ = false;
  }
}

double KinematicDrone::transit_altitude() const
{
  const double clearance = std::min(2.0, params_.geofence_max_alt - params_.home.z);
  const double floor = params_.home.z + std::max(0.0, clearance);
  return std::clamp(std::max(position_.z, floor), params_.home.z, params_.geofence_max_alt);
}

double KinematicDrone::yaw_toward(const Vec3 & target) const
{
  const double dx = target.x - position_.x;
  const double dy = target.y - position_.y;
  if (std::hypot(dx, dy) < 1e-3) {
    return yaw_;
  }
  return std::atan2(dy, dx);
}

SimState KinematicDrone::state() const
{
  SimState out;
  out.position = position_;
  out.velocity = velocity_;
  out.yaw = yaw_;
  out.battery_percent = battery_;
  out.mode = mode_;
  out.geofence_breached = outside_geofence(position_, params_);
  out.failsafe_active = failsafe_latched_;
  const double dx = position_.x - params_.home.x;
  const double dy = position_.y - params_.home.y;
  const double dz = position_.z - params_.home.z;
  out.distance_home = std::sqrt(dx * dx + dy * dy + dz * dz);
  return out;
}

const SimParams & KinematicDrone::params() const
{
  return params_;
}

}  // namespace drone_sim_core
