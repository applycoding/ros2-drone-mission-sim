// Copyright 2026 Drone Waypoint Demo
//
// First-order kinematic vehicle used by the desktop demo.
// Velocity is commanded directly from position error. There is no mass,
// drag, rotor model, or estimator. Battery and the geofence are bookkeeping
// so the failsafe paths can be exercised without a flight controller.

#ifndef DRONE_SIM_CORE__KINEMATICS_HPP_
#define DRONE_SIM_CORE__KINEMATICS_HPP_

namespace drone_sim_core
{

struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Setpoint
{
  Vec3 position;
  double yaw{0.0};
  bool return_to_home{false};
  bool valid{true};
};

struct SimParams
{
  Vec3 home;
  double max_horizontal_speed{4.0};
  double max_vertical_speed{2.0};
  double max_yaw_rate{1.2};
  double position_p_gain{1.4};
  double yaw_p_gain{2.5};
  double geofence_radius{40.0};
  double geofence_min_alt{-0.5};
  double geofence_max_alt{30.0};
  double battery_initial{100.0};
  double battery_rtl_percent{25.0};
  double battery_critical_percent{12.0};
  double battery_idle_percent_per_sec{0.01};
  double battery_flight_percent_per_sec{0.25};
  double land_altitude{0.2};
  double rtl_acceptance{0.6};
  // Simulator convenience so a landed demo can be started again.
  double recharge_percent_per_sec{0.0};
};

enum class FlightMode
{
  Disarmed,
  Flying,
  ReturnToHome,
  Failsafe,
  Landed
};

struct SimState
{
  Vec3 position;
  Vec3 velocity;
  double yaw{0.0};
  double battery_percent{100.0};
  FlightMode mode{FlightMode::Disarmed};
  bool geofence_breached{false};
  bool failsafe_active{false};
  double distance_home{0.0};
};

inline const char * flight_mode_name(FlightMode mode)
{
  switch (mode) {
    case FlightMode::Disarmed:
      return "DISARMED";
    case FlightMode::Flying:
      return "FLYING";
    case FlightMode::ReturnToHome:
      return "RTL";
    case FlightMode::Failsafe:
      return "FAILSAFE";
    case FlightMode::Landed:
      return "LANDED";
  }
  return "UNKNOWN";
}

bool outside_geofence(const Vec3 & position, const SimParams & params);

class KinematicDrone
{
public:
  explicit KinematicDrone(SimParams params);

  // False when the pack is at or below the critical threshold, the vehicle
  // is already outside the fence, or the setpoint is rejected.
  bool set_setpoint(const Setpoint & setpoint);
  void command_rtl();
  void update(double dt);

  SimState state() const;
  const SimParams & params() const;

private:
  bool arm();
  void integrate(double dt);
  void update_battery(double dt, bool airborne);
  void apply_automatic_mode_changes();
  void maybe_land();
  double transit_altitude() const;
  double yaw_toward(const Vec3 & target) const;

  SimParams params_;
  Vec3 position_;
  Vec3 velocity_;
  double yaw_{0.0};
  double battery_{100.0};
  FlightMode mode_{FlightMode::Disarmed};
  bool failsafe_latched_{false};
  bool emergency_land_{false};
  Setpoint active_setpoint_;
};

}  // namespace drone_sim_core

#endif  // DRONE_SIM_CORE__KINEMATICS_HPP_
