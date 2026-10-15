// SPDX-License-Identifier: MIT
#pragma once

namespace drone_sim {

struct KinematicsConfig {
  double max_speed_m_s{3.0};
  double max_yaw_rate_rad_s{1.5};
  double position_gain{1.0};
  double velocity_tau_s{0.25};
};

struct Setpoint {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
  // Non-positive values use KinematicsConfig::max_speed_m_s.
  double max_speed_m_s{0.0};
};

struct Pose {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
  double vx{0.0};
  double vy{0.0};
  double vz{0.0};
  double yaw_rate{0.0};
};

struct BodyVelocity {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

// Point-mass plant: desired velocity points at the setpoint, is speed-limited,
// and the actual velocity follows it with a first-order lag. Yaw rate is limited.
class KinematicModel {
 public:
  explicit KinematicModel(KinematicsConfig config = {});

  void set_setpoint(const Setpoint& setpoint);
  void set_pose(const Pose& pose);
  void step(double dt);

  const Pose& pose() const { return pose_; }
  const KinematicsConfig& config() const { return config_; }
  bool has_setpoint() const { return has_setpoint_; }

 private:
  KinematicsConfig config_;
  Setpoint setpoint_{};
  bool has_setpoint_{false};
  Pose pose_{};
};

double wrap_pi(double angle);

// ENU world velocity to a body frame whose x axis points along yaw.
BodyVelocity world_velocity_to_body(double vx, double vy, double vz, double yaw);

}  // namespace drone_sim
