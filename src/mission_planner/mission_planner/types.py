# SPDX-License-Identifier: MIT
"""Shared mission types. No ROS imports."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, IntEnum


class MissionError(ValueError):
    """A mission file or state-machine transition is invalid."""


class CommandType(Enum):
    TAKEOFF = "takeoff"
    WAYPOINT = "waypoint"
    RTL = "rtl"
    LAND = "land"


class Phase(IntEnum):
    """Numeric values match interfaces/msg/MissionState.msg."""

    IDLE = 0
    ARMED = 1
    TAKEOFF = 2
    WAYPOINT = 3
    RTL = 4
    LAND = 5
    COMPLETE = 6
    FAILSAFE = 7


@dataclass(frozen=True)
class VehicleState:
    x: float
    y: float
    z: float
    yaw: float = 0.0


@dataclass(frozen=True)
class Setpoint:
    x: float
    y: float
    z: float
    yaw: float
    max_speed_m_s: float


@dataclass(frozen=True)
class Geofence:
    center_x: float
    center_y: float
    radius_m: float
    min_alt_m: float
    max_alt_m: float


@dataclass(frozen=True)
class MissionItem:
    kind: CommandType
    x: float
    y: float
    z: float
    yaw: float = 0.0
    hold_s: float = 0.0


@dataclass(frozen=True)
class Mission:
    name: str
    home: VehicleState
    items: tuple[MissionItem, ...]
    geofence: Geofence
    acceptance_radius_m: float = 0.5
    cruise_speed_m_s: float = 2.0
    battery_failsafe_percent: float = 25.0
    battery_critical_percent: float = 10.0
    rtl_altitude_m: float = 5.0
    frame: str = "local_enu"

    def __post_init__(self) -> None:
        if not self.items:
            raise MissionError("mission has no items")
        if self.acceptance_radius_m <= 0.0:
            raise MissionError("acceptance_radius_m must be positive")
        if self.cruise_speed_m_s <= 0.0:
            raise MissionError("cruise_speed_m_s must be positive")
        if self.battery_critical_percent > self.battery_failsafe_percent:
            raise MissionError(
                "battery_critical_percent must be <= battery_failsafe_percent"
            )
        if self.geofence.radius_m <= 0.0:
            raise MissionError("geofence radius must be positive")
        if self.geofence.min_alt_m > self.geofence.max_alt_m:
            raise MissionError("geofence min_alt_m exceeds max_alt_m")
        if self.frame != "local_enu":
            raise MissionError("only the local_enu frame is supported")
