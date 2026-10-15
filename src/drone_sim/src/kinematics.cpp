// SPDX-License-Identifier: MIT
#include "drone_sim/kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace drone_sim {
namespace {

constexpr double kPi = 3.14159265358979323846;

bool finite_setpoint(const Setpoint& setpoint) {
  return std::isfinite(setpoint.x) && std::isfinite(setpoint.y) &&
         std::isfinite(setpoint.z) && std::isfinite(setpoint.yaw) &&
         std::isfinite(setpoint.max_speed_m_s);
}

}  // namespace

double wrap_pi(double angle) {
  if (!std::isfinite(angle)) {
    throw std::invalid_argument("angle must be finite");
  }
  angle = std::fmod(angle + kPi, 2.0 * kPi);
  if (angle < 0.0) {
    angle += 2.0 * kPi;
  }
  return angle - kPi;
}

BodyVelocity world_velocity_to_body(double vx, double vy, double vz, double yaw) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  BodyVelocity body;
  body.x = c * vx + s * vy;
  body.y = -s * vx + c * vy;
  body.z = vz;
  return body;
}

KinematicModel::KinematicModel(KinematicsConfig config) : config_(config) {
  if (config_.max_speed_m_s < 0.0 || config_.max_yaw_rate_rad_s < 0.0 ||
      config_.position_gain < 0.0 || !(config_.velocity_tau_s > 0.0)) {
    throw std::invalid_argument("invalid kinematics config");
  }
}

void KinematicModel::set_setpoint(const Setpoint& setpoint) {
  if (!finite_setpoint(setpoint)) {
    throw std::invalid_argument("setpoint must be finite");
  }
  setpoint_ = setpoint;
  setpoint_.yaw = wrap_pi(setpoint.yaw);
  has_setpoint_ = true;
}

void KinematicModel::set_pose(const Pose& pose) {
  if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.z) ||
      !std::isfinite(pose.yaw) || !std::isfinite(pose.vx) || !std::isfinite(pose.vy) ||
      !std::isfinite(pose.vz) || !std::isfinite(pose.yaw_rate)) {
    throw std::invalid_argument("pose must be finite");
  }
  pose_ = pose;
  pose_.yaw = wrap_pi(pose.yaw);
}

void KinematicModel::step(double dt) {
  if (!(dt >= 0.0) || !std::isfinite(dt)) {
    throw std::invalid_argument("dt must be finite and non-negative");
  }
  if (dt == 0.0) {
    pose_.yaw_rate = 0.0;
    return;
  }

  Setpoint goal = setpoint_;
  if (!has_setpoint_) {
    goal.x = pose_.x;
    goal.y = pose_.y;
    goal.z = pose_.z;
    goal.yaw = pose_.yaw;
    goal.max_speed_m_s = 0.0;
  }

  double speed_cap = config_.max_speed_m_s;
  if (goal.max_speed_m_s > 0.0) {
    speed_cap = goal.max_speed_m_s;
  }

  const double dx = goal.x - pose_.x;
  const double dy = goal.y - pose_.y;
  const double dz = goal.z - pose_.z;
  const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

  double desired_vx = 0.0;
  double desired_vy = 0.0;
  double desired_vz = 0.0;
  if (dist > 1e-9) {
    const double desired_speed = std::min(speed_cap, config_.position_gain * dist);
    const double scale = desired_speed / dist;
    desired_vx = dx * scale;
    desired_vy = dy * scale;
    desired_vz = dz * scale;
  }

  const double tau = std::max(config_.velocity_tau_s, 1e-3);
  const double alpha = std::clamp(dt / tau, 0.0, 1.0);
  pose_.vx += (desired_vx - pose_.vx) * alpha;
  pose_.vy += (desired_vy - pose_.vy) * alpha;
  pose_.vz += (desired_vz - pose_.vz) * alpha;

  const double speed = std::sqrt(pose_.vx * pose_.vx + pose_.vy * pose_.vy + pose_.vz * pose_.vz);
  if (speed > speed_cap && speed > 0.0) {
    const double scale = speed_cap / speed;
    pose_.vx *= scale;
    pose_.vy *= scale;
    pose_.vz *= scale;
  }

  pose_.x += pose_.vx * dt;
  pose_.y += pose_.vy * dt;
  pose_.z += pose_.vz * dt;

  const double yaw_error = wrap_pi(goal.yaw - pose_.yaw);
  const double yaw_step_cap = config_.max_yaw_rate_rad_s * dt;
  const double yaw_step = std::clamp(yaw_error, -yaw_step_cap, yaw_step_cap);
  pose_.yaw = wrap_pi(pose_.yaw + yaw_step);
  pose_.yaw_rate = yaw_step / dt;
}

}  // namespace drone_sim
