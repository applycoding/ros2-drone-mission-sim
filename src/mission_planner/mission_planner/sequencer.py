# Copyright 2026 Drone Waypoint Demo
"""Pure mission sequencer. The ROS node is a thin wrapper around this."""

import math

from mission_planner.models import Waypoint


class MissionState:
    """String states published on mission/status."""

    IDLE = "IDLE"
    LOADED = "LOADED"
    RUNNING = "RUNNING"
    COMPLETE = "COMPLETE"
    ABORTED = "ABORTED"
    RTL = "RTL"
    FAILSAFE = "FAILSAFE"


_TERMINAL = {
    MissionState.COMPLETE,
    MissionState.ABORTED,
    MissionState.RTL,
    MissionState.FAILSAFE,
}


class Sequencer:
    """Advance a waypoint list from vehicle position.

    Hold timers run only while the vehicle mode is FLYING, so a return-to-home
    in the simulator cannot be mistaken for progress along the plan.
    """

    def __init__(self, mission=None):
        self.mission = None
        self.state = MissionState.IDLE
        self.index = 0
        self.phase = "NAVIGATE"
        self.hold_elapsed = 0.0
        self.detail = "no mission loaded"
        if mission is not None:
            self.load(mission)

    def load(self, mission):
        """Replace the plan. Refuses to do so while a mission is running."""
        if self.state == MissionState.RUNNING:
            raise ValueError("refuse to replace a running mission")
        if mission is None or not mission.waypoints:
            raise ValueError("mission has no waypoints")
        self.mission = mission
        self.state = MissionState.LOADED
        self.index = 0
        self.phase = "NAVIGATE"
        self.hold_elapsed = 0.0
        self.detail = "loaded %d waypoints" % len(mission.waypoints)

    def start(self):
        """Begin at the first waypoint. Returns success and a short message."""
        if self.mission is None or not self.mission.waypoints:
            return False, "no mission loaded"
        if self.state == MissionState.RUNNING:
            return False, "mission already running"
        self.index = 0
        self.phase = "NAVIGATE"
        self.hold_elapsed = 0.0
        self.state = MissionState.RUNNING
        self.detail = "enroute %s" % self.mission.waypoints[0].name
        return True, "started"

    def abort(self):
        """Stop the plan and ask the vehicle to return home."""
        if self.mission is None:
            return False, "no mission loaded"
        self.state = MissionState.ABORTED
        self.detail = "aborted; returning home"
        return True, self.detail

    def command_rtl(self):
        """Ask for a pilot-initiated return without marking a vehicle failsafe."""
        if self.mission is None:
            return False, "no mission loaded"
        self.state = MissionState.RTL
        self.detail = "return to home requested"
        return True, self.detail

    def notify_vehicle_failsafe(self):
        """Record that the vehicle itself decided to stop the mission."""
        if self.state not in (MissionState.RUNNING, MissionState.RTL):
            return
        self.state = MissionState.FAILSAFE
        self.detail = "vehicle failsafe; returning home"

    def update(self, x, y, z, dt, vehicle_mode):
        """Return the setpoint to stream, or None when no plan is active."""
        if self.state in (MissionState.IDLE, MissionState.LOADED):
            return None
        if self.state in _TERMINAL:
            return self._home_waypoint()
        if self.mission is None:
            return None
        if vehicle_mode != "FLYING":
            if isinstance(vehicle_mode, str) and vehicle_mode:
                label = vehicle_mode.lower()
            else:
                label = "unknown"
            self.detail = "waiting for vehicle (%s)" % label
            return self._active_waypoint()
        safe_dt = dt
        if not isinstance(safe_dt, (int, float)) or isinstance(safe_dt, bool):
            safe_dt = 0.0
        if not math.isfinite(safe_dt) or safe_dt < 0.0:
            safe_dt = 0.0
        safe_dt = min(float(safe_dt), 1.0)
        waypoint = self._active_waypoint()
        distance = _distance(x, y, z, waypoint)
        if self.phase == "NAVIGATE" and distance <= waypoint.acceptance_radius:
            self.phase = "HOLD"
            self.hold_elapsed = 0.0
        if self.phase == "HOLD":
            if distance > waypoint.acceptance_radius:
                self.phase = "NAVIGATE"
                self.hold_elapsed = 0.0
                self.detail = "enroute %s" % waypoint.name
                return waypoint
            self.hold_elapsed += safe_dt
            self.detail = "holding %s" % waypoint.name
            if self.hold_elapsed + 1e-9 >= waypoint.hold_sec:
                self.index += 1
                self.phase = "NAVIGATE"
                self.hold_elapsed = 0.0
                if self.index >= len(self.mission.waypoints):
                    self.state = MissionState.COMPLETE
                    self.detail = "mission complete; returning home"
                    return self._home_waypoint()
                waypoint = self._active_waypoint()
                self.detail = "enroute %s" % waypoint.name
        else:
            self.detail = "enroute %s" % waypoint.name
        return waypoint

    def progress(self):
        """Fraction of waypoints completed, in the range 0 to 1."""
        if self.mission is None or not self.mission.waypoints:
            return 0.0
        count = len(self.mission.waypoints)
        if self.state == MissionState.COMPLETE or self.index >= count:
            return 1.0
        partial = 0.5 if self.phase == "HOLD" else 0.0
        return (self.index + partial) / count

    def current_index(self):
        """Active waypoint index, or the waypoint count once the plan is done."""
        if self.mission is None:
            return 0
        return min(self.index, len(self.mission.waypoints))

    def current_name(self):
        """Name of the active waypoint, or home after the plan completes."""
        if self.mission is None or not self.mission.waypoints:
            return ""
        if self.state == MissionState.COMPLETE or self.index >= len(self.mission.waypoints):
            return "home"
        return self.mission.waypoints[self.index].name

    def waypoint_count(self):
        if self.mission is None:
            return 0
        return len(self.mission.waypoints)

    def _active_waypoint(self):
        return self.mission.waypoints[self.index]

    def _home_waypoint(self):
        mission = self.mission
        return Waypoint(
            name="home",
            x=0.0 if mission is None else mission.home_x,
            y=0.0 if mission is None else mission.home_y,
            z=0.0 if mission is None else mission.home_z,
            yaw=0.0,
            hold_sec=0.0,
            acceptance_radius=0.6,
        )


def _distance(x, y, z, waypoint):
    return math.sqrt(
        (x - waypoint.x) ** 2 +
        (y - waypoint.y) ** 2 +
        (z - waypoint.z) ** 2
    )
