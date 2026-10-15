# Copyright 2026 Drone Waypoint Demo
"""Shared mission types for the planner and its tests."""

from dataclasses import dataclass


@dataclass(frozen=True)
class Waypoint:
    """One setpoint in local ENU meters. Yaw is radians from east."""

    name: str
    x: float
    y: float
    z: float
    yaw: float = 0.0
    hold_sec: float = 0.0
    acceptance_radius: float = 0.75


@dataclass(frozen=True)
class Mission:
    """Waypoints already expressed in the simulator's local ENU frame."""

    frame: str
    waypoints: list
    home_x: float = 0.0
    home_y: float = 0.0
    home_z: float = 0.0
    source: str = ""


def mission_to_dict(mission):
    """Serialize a mission for the ground station."""
    return {
        "frame": mission.frame,
        "source": mission.source,
        "home": {
            "x": mission.home_x,
            "y": mission.home_y,
            "z": mission.home_z,
        },
        "waypoints": [
            {
                "name": item.name,
                "x": item.x,
                "y": item.y,
                "z": item.z,
                "yaw": item.yaw,
                "hold_sec": item.hold_sec,
                "acceptance_radius": item.acceptance_radius,
            }
            for item in mission.waypoints
        ],
    }
