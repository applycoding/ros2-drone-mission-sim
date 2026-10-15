// Copyright 2026 Drone Waypoint Demo

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "drone_msgs/msg/drone_state.hpp"
#include "drone_msgs/msg/waypoint.hpp"
#include "drone_sim_core/kinematics.hpp"

namespace
{

drone_sim_core::SimParams read_params(rclcpp::Node & node)
{
  drone_sim_core::SimParams params;
  params.home.x = node.declare_parameter<double>("home_x", 0.0);
  params.home.y = node.declare_parameter<double>("home_y", 0.0);
  params.home.z = node.declare_parameter<double>("home_z", 0.0);
  params.max_horizontal_speed = node.declare_parameter<double>("max_horizontal_speed", 4.0);
  params.max_vertical_speed = node.declare_parameter<double>("max_vertical_speed", 2.0);
  params.max_yaw_rate = node.declare_parameter<double>("max_yaw_rate", 1.2);
  params.position_p_gain = node.declare_parameter<double>("position_p_gain", 1.4);
  params.yaw_p_gain = node.declare_parameter<double>("yaw_p_gain", 2.5);
  params.geofence_radius = node.declare_parameter<double>("geofence_radius", 40.0);
  params.geofence_min_alt = node.declare_parameter<double>("geofence_min_alt", -0.5);
  params.geofence_max_alt = node.declare_parameter<double>("geofence_max_alt", 30.0);
  params.battery_initial = node.declare_parameter<double>("battery_initial", 100.0);
  params.battery_rtl_percent = node.declare_parameter<double>("battery_rtl_percent", 25.0);
  params.battery_critical_percent =
    node.declare_parameter<double>("battery_critical_percent", 12.0);
  params.battery_idle_percent_per_sec =
    node.declare_parameter<double>("battery_idle_percent_per_sec", 0.01);
  params.battery_flight_percent_per_sec =
    node.declare_parameter<double>("battery_flight_percent_per_sec", 0.25);
  params.land_altitude = node.declare_parameter<double>("land_altitude", 0.2);
  params.rtl_acceptance = node.declare_parameter<double>("rtl_acceptance", 0.6);
  params.recharge_percent_per_sec =
    node.declare_parameter<double>("recharge_percent_per_sec", 0.5);
  return params;
}

}  // namespace

class SimDroneNode : public rclcpp::Node
{
public:
  SimDroneNode()
  : Node("sim_drone")
  {
    drone_ = std::make_unique<drone_sim_core::KinematicDrone>(read_params(*this));
    const double rate = declare_parameter<double>("publish_rate", 20.0);
    if (!(rate > 0.0)) {
      throw std::invalid_argument("publish_rate must be positive");
    }
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    setpoint_sub_ = create_subscription<drone_msgs::msg::Waypoint>(
      "drone/setpoint", qos,
      std::bind(&SimDroneNode::on_setpoint, this, std::placeholders::_1));
    state_pub_ = create_publisher<drone_msgs::msg::DroneState>("drone/state", qos);
    const auto period = std::chrono::duration<double>(1.0 / rate);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&SimDroneNode::on_timer, this));
    RCLCPP_INFO(
      get_logger(),
      "Kinematic simulator ready (geofence %.1f m, RTL at %.0f%% battery)",
      drone_->params().geofence_radius,
      drone_->params().battery_rtl_percent);
  }

private:
  void on_setpoint(const drone_msgs::msg::Waypoint::SharedPtr msg)
  {
    drone_sim_core::Setpoint setpoint;
    setpoint.position = {msg->x, msg->y, msg->z};
    setpoint.yaw = msg->yaw;
    setpoint.return_to_home = msg->return_to_home;
    drone_->set_setpoint(setpoint);
  }

  void on_timer()
  {
    const rclcpp::Time stamp = now();
    double dt = 0.0;
    if (last_time_.nanoseconds() != 0) {
      dt = (stamp - last_time_).seconds();
    }
    last_time_ = stamp;
    drone_->update(dt);
    publish(stamp);
  }

  void publish(const rclcpp::Time & stamp)
  {
    const drone_sim_core::SimState state = drone_->state();
    const std::string mode = drone_sim_core::flight_mode_name(state.mode);
    if (mode != last_mode_) {
      RCLCPP_INFO(get_logger(), "Flight mode %s", mode.c_str());
      last_mode_ = mode;
    }
    drone_msgs::msg::DroneState msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "map";
    msg.x = state.position.x;
    msg.y = state.position.y;
    msg.z = state.position.z;
    msg.yaw = state.yaw;
    msg.vx = state.velocity.x;
    msg.vy = state.velocity.y;
    msg.vz = state.velocity.z;
    msg.speed = std::hypot(std::hypot(state.velocity.x, state.velocity.y), state.velocity.z);
    msg.battery_percent = state.battery_percent;
    msg.battery_rtl_percent = drone_->params().battery_rtl_percent;
    msg.battery_critical_percent = drone_->params().battery_critical_percent;
    msg.distance_home = state.distance_home;
    msg.home_x = drone_->params().home.x;
    msg.home_y = drone_->params().home.y;
    msg.home_z = drone_->params().home.z;
    msg.geofence_radius = drone_->params().geofence_radius;
    msg.geofence_max_alt = drone_->params().geofence_max_alt;
    msg.mode = mode;
    msg.geofence_breached = state.geofence_breached;
    msg.failsafe_active = state.failsafe_active;
    state_pub_->publish(msg);
  }

  std::unique_ptr<drone_sim_core::KinematicDrone> drone_;
  rclcpp::Subscription<drone_msgs::msg::Waypoint>::SharedPtr setpoint_sub_;
  rclcpp::Publisher<drone_msgs::msg::DroneState>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};
  std::string last_mode_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<SimDroneNode>());
  } catch (const std::exception & err) {
    RCLCPP_FATAL(rclcpp::get_logger("sim_drone"), "%s", err.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
