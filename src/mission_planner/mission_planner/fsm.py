# SPDX-License-Identifier: MIT
"""Mission state machine.

Airborne phases watch two failsafes:

* battery at or below the low threshold skips the rest of the plan, returns
  home at the RTL altitude, then lands
* a geofence breach or a critical battery lands at the current horizontal
  position

A failsafe latches. Touchdown after one ends in FAILSAFE. A plan that runs
to its own land item ends in COMPLETE. The machine is not thread-safe.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from mission_planner.geofence import GeofenceCheck, evaluate_geofence
from mission_planner.types import (
    CommandType,
    Mission,
    MissionError,
    MissionItem,
    Phase,
    Setpoint,
    VehicleState,
)

_AIRBORNE = frozenset({Phase.TAKEOFF, Phase.WAYPOINT, Phase.RTL})
_PHASE_FOR_COMMAND = {
    CommandType.TAKEOFF: Phase.TAKEOFF,
    CommandType.WAYPOINT: Phase.WAYPOINT,
    CommandType.RTL: Phase.RTL,
    CommandType.LAND: Phase.LAND,
}


@dataclass(frozen=True)
class TickResult:
    setpoint: Setpoint
    phase: Phase
    waypoint_index: int
    waypoint_count: int
    waypoints_reached: int
    distance_to_target: float
    failsafe_reason: str
    geofence_inside: bool
    geofence_breach: str
    geofence_margin_m: float


class MissionFSM:
    def __init__(self, mission: Mission) -> None:
        self.mission = mission
        self.phase = Phase.IDLE
        self.failsafe_reason = ""
        self.waypoints_reached = 0
        self._items = tuple(mission.items)
        self._index = 0
        self._override: tuple[MissionItem, ...] | None = None
        self._override_index = 0
        self._hold_elapsed = 0.0
        self._failsafe_latched = False
        self._waypoint_count = sum(
            1 for item in self._items if item.kind is CommandType.WAYPOINT
        )

    @property
    def waypoint_count(self) -> int:
        return self._waypoint_count

    def arm(self) -> None:
        if self.phase is not Phase.IDLE:
            raise MissionError(f"cannot arm from {self.phase.name}")
        self.phase = Phase.ARMED

    def start(self, battery_percent: float = 100.0) -> None:
        if self.phase is not Phase.ARMED:
            raise MissionError(f"cannot start from {self.phase.name}")
        if battery_percent <= self.mission.battery_critical_percent:
            raise MissionError("refusing to start: battery is critical")
        self.phase = _phase_for(self._items[0])

    def request_land(self, vehicle: VehicleState, reason: str) -> None:
        if self.phase in (Phase.COMPLETE, Phase.FAILSAFE, Phase.IDLE):
            return
        if self.phase is Phase.ARMED:
            self.phase = Phase.IDLE
            return
        self._land_in_place(vehicle, reason)

    def tick(self, vehicle: VehicleState, battery_percent: float, dt: float) -> TickResult:
        if dt < 0.0 or not math.isfinite(dt):
            raise MissionError("dt must be finite and non-negative")
        battery = min(100.0, max(0.0, battery_percent))
        fence = evaluate_geofence(self.mission.geofence, vehicle.x, vehicle.y, vehicle.z)
        self._apply_failsafes(vehicle, battery, fence.inside)

        if self.phase in (Phase.IDLE, Phase.ARMED, Phase.COMPLETE, Phase.FAILSAFE):
            return self._result(self._hold(vehicle), vehicle, fence)

        self._advance_if_arrived(vehicle, dt)
        if self.phase in (Phase.COMPLETE, Phase.FAILSAFE):
            return self._result(self._hold(vehicle), vehicle, fence)

        item = self._current_item()
        if item is None:
            self._finish()
            return self._result(self._hold(vehicle), vehicle, fence)
        return self._result(self._setpoint_for(item), vehicle, fence)

    def _apply_failsafes(self, vehicle: VehicleState, battery: float, inside: bool) -> None:
        if self.phase not in _AIRBORNE:
            return
        if not inside or battery <= self.mission.battery_critical_percent:
            if not inside and battery <= self.mission.battery_critical_percent:
                reason = "geofence_and_battery_critical"
            elif not inside:
                reason = "geofence"
            else:
                reason = "battery_critical"
            self._land_in_place(vehicle, reason)
            return
        if battery <= self.mission.battery_failsafe_percent and self.phase in (
            Phase.TAKEOFF,
            Phase.WAYPOINT,
        ):
            self._return_home("battery_low")

    def _return_home(self, reason: str) -> None:
        if self._failsafe_latched:
            return
        self._failsafe_latched = True
        self.failsafe_reason = reason
        self._hold_elapsed = 0.0
        home = self.mission.home
        self._override = (
            MissionItem(CommandType.RTL, home.x, home.y, self.mission.rtl_altitude_m, home.yaw),
            MissionItem(CommandType.LAND, home.x, home.y, 0.0, home.yaw),
        )
        self._override_index = 0
        self.phase = Phase.RTL

    def _land_in_place(self, vehicle: VehicleState, reason: str) -> None:
        self._failsafe_latched = True
        self.failsafe_reason = reason
        self._hold_elapsed = 0.0
        self._override = (
            MissionItem(CommandType.LAND, vehicle.x, vehicle.y, 0.0, vehicle.yaw),
        )
        self._override_index = 0
        self.phase = Phase.LAND

    def _advance_if_arrived(self, vehicle: VehicleState, dt: float) -> None:
        item = self._current_item()
        if item is None:
            self._finish()
            return
        if not _reached(vehicle, item, self.mission.acceptance_radius_m):
            self._hold_elapsed = 0.0
            return
        self._hold_elapsed += dt
        if self._hold_elapsed + 1e-9 < item.hold_s:
            return
        self._complete_item(item)

    def _complete_item(self, item: MissionItem) -> None:
        if item.kind is CommandType.WAYPOINT:
            self.waypoints_reached += 1
        self._hold_elapsed = 0.0
        if self._override is not None:
            self._override_index += 1
            if self._override_index >= len(self._override):
                self._finish()
                return
            self.phase = _phase_for(self._override[self._override_index])
            return
        self._index += 1
        if self._index >= len(self._items):
            self._finish()
            return
        self.phase = _phase_for(self._items[self._index])

    def _finish(self) -> None:
        self.phase = Phase.FAILSAFE if self.failsafe_reason else Phase.COMPLETE

    def _current_item(self) -> MissionItem | None:
        if self._override is not None:
            if self._override_index >= len(self._override):
                return None
            return self._override[self._override_index]
        if self._index >= len(self._items):
            return None
        return self._items[self._index]

    def _setpoint_for(self, item: MissionItem) -> Setpoint:
        return Setpoint(
            item.x,
            item.y,
            item.z,
            item.yaw,
            self.mission.cruise_speed_m_s,
        )

    def _hold(self, vehicle: VehicleState) -> Setpoint:
        return Setpoint(
            vehicle.x,
            vehicle.y,
            vehicle.z,
            vehicle.yaw,
            self.mission.cruise_speed_m_s,
        )

    def _result(self, setpoint: Setpoint, vehicle: VehicleState, fence: GeofenceCheck) -> TickResult:
        item = self._current_item()
        terminal = self.phase in (Phase.IDLE, Phase.ARMED, Phase.COMPLETE, Phase.FAILSAFE)
        if item is None or terminal:
            distance = 0.0
        else:
            distance = math.dist((vehicle.x, vehicle.y, vehicle.z), (item.x, item.y, item.z))
        return TickResult(
            setpoint=setpoint,
            phase=self.phase,
            waypoint_index=self.waypoints_reached,
            waypoint_count=self._waypoint_count,
            waypoints_reached=self.waypoints_reached,
            distance_to_target=distance,
            failsafe_reason=self.failsafe_reason,
            geofence_inside=fence.inside,
            geofence_breach=fence.breach_type,
            geofence_margin_m=fence.distance_to_boundary_m,
        )


def _phase_for(item: MissionItem) -> Phase:
    try:
        return _PHASE_FOR_COMMAND[item.kind]
    except KeyError as exc:
        raise MissionError(f"unhandled command {item.kind}") from exc


def _reached(vehicle: VehicleState, item: MissionItem, radius: float) -> bool:
    horizontal = math.hypot(vehicle.x - item.x, vehicle.y - item.y)
    vertical = abs(vehicle.z - item.z)
    return horizontal <= radius and vertical <= radius
