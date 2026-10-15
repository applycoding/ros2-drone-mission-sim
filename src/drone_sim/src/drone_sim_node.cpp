// SPDX-License-Identifier: MIT
#include "drone_sim/battery_model.hpp"
#include "drone_sim/kinematics.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "interfaces/msg/battery_status.hpp"
#include "interfaces/msg/setpoint.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace drone_sim {
namespace {

geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw) {
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.x = 0.0;
  quaternion.y = 0.0;
  quaternion.z = std::sin(0.5 * yaw);
  quaternion.w = std::cos(0.5 * yaw);
  return quaternion;
}

double require_non_negative(double value, const std::string& name) {
  if (!std::isfinite(value) || value < 0.0) {
    throw std::invalid_argument(name + " must be finite and non-negative");
  }
  return value;
}

double require_positive(double value, const std::string& name) {
  if (!std::isfinite(value) || !(value > 0.0)) {
    throw std::invalid_argument(name + " must be finite and positive");
  }
  return value;
}

}  // namespace

class DroneSimNode : public rclcpp::Node {
 public:
  DroneSimNode() : rclcpp::Node("drone_sim_node") {
    const double rate_hz = require_positive(declare_parameter<double>("rate_hz", 50.0), "rate_hz");
    nominal_dt_ = 1.0 / rate_hz;

    KinematicsConfig kinematics_config;
    kinematics_config.max_speed_m_s = require_non_negative(
        declare_parameter<double>("max_speed_m_s", 3.0), "max_speed_m_s");
    kinematics_config.max_yaw_rate_rad_s = require_non_negative(
        declare_parameter<double>("max_yaw_rate_rad_s", 1.2), "max_yaw_rate_rad_s");
    kinematics_config.position_gain = require_non_negative(
        declare_parameter<double>("position_gain", 1.5), "position_gain");
    kinematics_config.velocity_tau_s =
        require_positive(declare_parameter<double>("velocity_tau_s", 0.2), "velocity_tau_s");

    BatteryConfig battery_config;
    battery_config.percent = declare_parameter<double>("battery_percent", 100.0);
    battery_config.hover_drain_pct_per_s = require_non_negative(
        declare_parameter<double>("hover_drain_pct_per_s", 0.02), "hover_drain_pct_per_s");
    battery_config.move_drain_pct_per_s = require_non_negative(
        declare_parameter<double>("move_drain_pct_per_s", 0.05), "move_drain_pct_per_s");
    battery_config.low_percent = declare_parameter<double>("low_percent", 25.0);
    battery_config.critical_percent = declare_parameter<double>("critical_percent", 10.0);

    frame_id_ = declare_parameter<std::string>("frame_id", "odom");
    child_frame_id_ = declare_parameter<std::string>("child_frame_id", "base_link");
    const auto setpoint_topic = declare_parameter<std::string>("setpoint_topic", "/drone/setpoint");
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/drone/odom");
    const auto battery_topic = declare_parameter<std::string>("battery_topic", "/drone/battery");
    if (frame_id_.empty() || child_frame_id_.empty()) {
      throw std::invalid_argument("frame ids must be non-empty");
    }

    kinematics_ = std::make_unique<KinematicModel>(kinematics_config);
    battery_ = std::make_unique<BatteryModel>(battery_config);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    setpoint_sub_ = create_subscription<interfaces::msg::Setpoint>(
        setpoint_topic, rclcpp::QoS(10),
        std::bind(&DroneSimNode::on_setpoint, this, std::placeholders::_1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic, rclcpp::QoS(10));
    battery_pub_ =
        create_publisher<interfaces::msg::BatteryStatus>(battery_topic, rclcpp::QoS(10));

    const auto period = std::chrono::duration<double>(nominal_dt_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&DroneSimNode::on_timer, this));

    RCLCPP_INFO(
        get_logger(), "Kinematic sim at %.1f Hz, odom on %s, setpoints on %s", rate_hz,
        odom_topic.c_str(), setpoint_topic.c_str());
  }

 private:
  void on_setpoint(const interfaces::msg::Setpoint& msg) {
    if (!std::isfinite(msg.x) || !std::isfinite(msg.y) || !std::isfinite(msg.z) ||
        !std::isfinite(msg.yaw) || !std::isfinite(msg.max_speed_m_s)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring non-finite setpoint");
      return;
    }
    Setpoint setpoint;
    setpoint.x = msg.x;
    setpoint.y = msg.y;
    setpoint.z = msg.z;
    setpoint.yaw = msg.yaw;
    setpoint.max_speed_m_s = msg.max_speed_m_s;
    kinematics_->set_setpoint(setpoint);
  }

  void on_timer() {
    const auto now = get_clock()->now();
    double step_dt = nominal_dt_;
    if (have_last_time_) {
      step_dt = (now - last_time_).seconds();
      if (!std::isfinite(step_dt) || step_dt <= 0.0) {
        step_dt = nominal_dt_;
      }
      step_dt = std::min(step_dt, 0.25);
    }
    have_last_time_ = true;
    last_time_ = now;

    if (!kinematics_->has_setpoint()) {
      const Pose& pose = kinematics_->pose();
      Setpoint hold;
      hold.x = pose.x;
      hold.y = pose.y;
      hold.z = pose.z;
      hold.yaw = pose.yaw;
      kinematics_->set_setpoint(hold);
    }

    kinematics_->step(step_dt);
    const Pose& pose = kinematics_->pose();
    const double speed = std::sqrt(pose.vx * pose.vx + pose.vy * pose.vy + pose.vz * pose.vz);
    battery_->update(speed, step_dt);
    publish(now, pose);
  }

  void publish(const rclcpp::Time& stamp, const Pose& pose) {
    const auto rotation = quaternion_from_yaw(pose.yaw);
    const BodyVelocity body = world_velocity_to_body(pose.vx, pose.vy, pose.vz, pose.yaw);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = child_frame_id_;
    odom.pose.pose.position.x = pose.x;
    odom.pose.pose.position.y = pose.y;
    odom.pose.pose.position.z = pose.z;
    odom.pose.pose.orientation = rotation;
    odom.twist.twist.linear.x = body.x;
    odom.twist.twist.linear.y = body.y;
    odom.twist.twist.linear.z = body.z;
    odom.twist.twist.angular.z = pose.yaw_rate;
    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = frame_id_;
    transform.child_frame_id = child_frame_id_;
    transform.transform.translation.x = pose.x;
    transform.transform.translation.y = pose.y;
    transform.transform.translation.z = pose.z;
    transform.transform.rotation = rotation;
    tf_broadcaster_->sendTransform(transform);

    interfaces::msg::BatteryStatus battery;
    battery.header.stamp = stamp;
    battery.header.frame_id = frame_id_;
    battery.percent = static_cast<float>(battery_->percent());
    const double fraction = battery_->percent() / 100.0;
    battery.voltage = static_cast<float>(11.1 + (12.6 - 11.1) * fraction);
    battery.drain_pct_per_s = static_cast<float>(battery_->last_drain_pct_per_s());
    battery.low = battery_->low();
    battery.critical = battery_->critical();
    battery_pub_->publish(battery);
  }

  std::string frame_id_;
  std::string child_frame_id_;
  double nominal_dt_{0.02};
  bool have_last_time_{false};
  rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};

  std::unique_ptr<KinematicModel> kinematics_;
  std::unique_ptr<BatteryModel> battery_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<interfaces::msg::Setpoint>::SharedPtr setpoint_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<interfaces::msg::BatteryStatus>::SharedPtr battery_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace drone_sim

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<drone_sim::DroneSimNode>();
    rclcpp::spin(node);
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(rclcpp::get_logger("drone_sim_node"), "%s", ex.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
