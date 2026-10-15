# Copyright 2026 Drone Waypoint Demo
"""Thread-safe snapshot shared by the ROS node and the web server."""

import copy
import threading


def _default_drone():
    return {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
        "yaw": 0.0,
        "vx": 0.0,
        "vy": 0.0,
        "vz": 0.0,
        "speed": 0.0,
        "battery_percent": 0.0,
        "battery_rtl_percent": 25.0,
        "battery_critical_percent": 12.0,
        "distance_home": 0.0,
        "home_x": 0.0,
        "home_y": 0.0,
        "home_z": 0.0,
        "geofence_radius": 0.0,
        "geofence_max_alt": 0.0,
        "mode": "UNKNOWN",
        "geofence_breached": False,
        "failsafe_active": False,
    }


def _default_mission():
    return {
        "state": "UNKNOWN",
        "current_index": 0,
        "waypoint_count": 0,
        "current_name": "",
        "progress": 0.0,
        "detail": "waiting for mission planner",
    }


def _default_plan():
    return {
        "frame": "",
        "source": "",
        "home": {"x": 0.0, "y": 0.0, "z": 0.0},
        "waypoints": [],
    }


class StateStore:
    """Latest telemetry. ROS callbacks write it; HTTP handlers read it."""

    def __init__(self):
        self._lock = threading.Lock()
        self._drone = _default_drone()
        self._mission = _default_mission()
        self._plan = _default_plan()
        self._last_command = None
        self._have_drone = False
        self._have_mission = False

    def update_drone(self, drone):
        with self._lock:
            self._drone = dict(drone)
            self._have_drone = True

    def update_mission(self, mission):
        with self._lock:
            self._mission = dict(mission)
            self._have_mission = True

    def update_plan(self, plan):
        if not isinstance(plan, dict):
            return
        with self._lock:
            self._plan = plan

    def set_last_command(self, command, success, message):
        with self._lock:
            self._last_command = {
                "command": command,
                "success": bool(success),
                "message": message,
            }

    def snapshot(self):
        with self._lock:
            return copy.deepcopy(
                {
                    "drone": self._drone,
                    "mission": self._mission,
                    "plan": self._plan,
                    "last_command": self._last_command,
                    "have_drone": self._have_drone,
                    "have_mission": self._have_mission,
                }
            )

    def health(self):
        with self._lock:
            return {
                "ok": True,
                "service": "telemetry_bridge",
                "have_drone": self._have_drone,
                "have_mission": self._have_mission,
            }


class CommandQueue:
    """Commands accepted by HTTP and drained on the ROS executor."""

    def __init__(self):
        self._lock = threading.Lock()
        self._items = []

    def push(self, command):
        with self._lock:
            self._items.append(command)

    def drain(self):
        with self._lock:
            items = list(self._items)
            self._items = []
            return items
