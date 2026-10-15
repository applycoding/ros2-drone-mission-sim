# SPDX-License-Identifier: MIT

import math
from pathlib import Path

import pytest

from mission_planner.fsm import MissionFSM
from mission_planner.loader import load_mission
from mission_planner.types import (
    CommandType,
    Geofence,
    Mission,
    MissionError,
    MissionItem,
    Phase,
    Setpoint,
    VehicleState,
)

MISSIONS = Path(__file__).resolve().parents[1] / "missions"


def _mission(items: list[MissionItem], **overrides: float) -> Mission:
    return Mission(
        name="test",
        home=VehicleState(0.0, 0.0, 0.0),
        items=tuple(items),
        geofence=Geofence(0.0, 0.0, 1000.0, -5.0, 100.0),
        acceptance_radius_m=0.3,
        cruise_speed_m_s=2.0,
        battery_failsafe_percent=float(overrides.get("battery_failsafe_percent", 25.0)),
        battery_critical_percent=float(overrides.get("battery_critical_percent", 10.0)),
        rtl_altitude_m=float(overrides.get("rtl_altitude_m", 5.0)),
    )


def _square() -> Mission:
    return _mission(
        [
            MissionItem(CommandType.TAKEOFF, 0.0, 0.0, 5.0),
            MissionItem(CommandType.WAYPOINT, 4.0, 0.0, 5.0, hold_s=0.2),
            MissionItem(CommandType.WAYPOINT, 4.0, 4.0, 5.0),
            MissionItem(CommandType.RTL, 0.0, 0.0, 5.0),
            MissionItem(CommandType.LAND, 0.0, 0.0, 0.0),
        ]
    )


def pursue(vehicle: VehicleState, setpoint: Setpoint, speed: float, dt: float) -> VehicleState:
    dx = setpoint.x - vehicle.x
    dy = setpoint.y - vehicle.y
    dz = setpoint.z - vehicle.z
    dist = math.hypot(dx, dy, dz)
    if dist <= 1e-9:
        return VehicleState(setpoint.x, setpoint.y, setpoint.z, setpoint.yaw)
    step = min(speed * dt, dist)
    scale = step / dist
    return VehicleState(
        vehicle.x + dx * scale,
        vehicle.y + dy * scale,
        vehicle.z + dz * scale,
        setpoint.yaw,
    )


def fly(fsm: MissionFSM, battery: float = 100.0, limit: int = 800):
    vehicle = VehicleState(0.0, 0.0, 0.0)
    result = None
    for _ in range(limit):
        result = fsm.tick(vehicle, battery, 0.05)
        if result.phase in (Phase.COMPLETE, Phase.FAILSAFE):
            return vehicle, result
        vehicle = pursue(vehicle, result.setpoint, 25.0, 0.05)
    raise AssertionError(f"mission did not finish in {result.phase.name if result else 'no'} phase")


def test_arm_and_start_gates():
    fsm = MissionFSM(_square())
    with pytest.raises(MissionError, match="cannot start"):
        fsm.start()
    fsm.arm()
    with pytest.raises(MissionError, match="cannot arm"):
        fsm.arm()
    with pytest.raises(MissionError, match="battery is critical"):
        fsm.start(5.0)
    fsm.start(100.0)
    with pytest.raises(MissionError, match="cannot start"):
        fsm.start()


def test_idle_and_armed_do_not_consume_the_plan():
    fsm = MissionFSM(_square())
    fsm.arm()
    for _ in range(10):
        result = fsm.tick(VehicleState(0.0, 0.0, 5.0), 100.0, 0.2)
    assert result.phase is Phase.ARMED
    assert fsm.waypoints_reached == 0
    assert result.setpoint.z == pytest.approx(5.0)


def test_nominal_mission_completes():
    fsm = MissionFSM(_square())
    fsm.arm()
    fsm.start()
    vehicle, result = fly(fsm)
    assert result.phase is Phase.COMPLETE
    assert result.failsafe_reason == ""
    assert result.waypoints_reached == 2
    assert vehicle.z == pytest.approx(0.0, abs=0.3)
    assert math.hypot(vehicle.x, vehicle.y) == pytest.approx(0.0, abs=0.3)


def test_sample_square_yaml_completes():
    fsm = MissionFSM(load_mission(MISSIONS / "square.yaml"))
    fsm.arm()
    fsm.start()
    _, result = fly(fsm, limit=1200)
    assert result.phase is Phase.COMPLETE
    assert result.waypoints_reached == 4
    assert result.waypoint_count == 4


def test_hold_resets_when_the_vehicle_leaves_the_waypoint():
    fsm = MissionFSM(
        _mission(
            [
                MissionItem(CommandType.WAYPOINT, 0.0, 0.0, 0.0, hold_s=1.0),
                MissionItem(CommandType.LAND, 0.0, 0.0, 0.0),
            ]
        )
    )
    fsm.arm()
    fsm.start()
    here = VehicleState(0.0, 0.0, 0.0)
    fsm.tick(here, 100.0, 0.4)
    assert fsm.phase is Phase.WAYPOINT
    fsm.tick(VehicleState(5.0, 0.0, 0.0), 100.0, 0.4)
    fsm.tick(here, 100.0, 0.4)
    assert fsm.phase is Phase.WAYPOINT
    assert fsm.waypoints_reached == 0
    result = fsm.tick(here, 100.0, 0.7)
    assert result.phase is Phase.LAND
    assert fsm.waypoints_reached == 1


def test_low_battery_returns_home_then_fails_safe():
    fsm = MissionFSM(_square())
    fsm.arm()
    fsm.start()
    vehicle = VehicleState(0.0, 0.0, 0.0)
    for _ in range(200):
        result = fsm.tick(vehicle, 100.0, 0.05)
        if result.phase is Phase.WAYPOINT and vehicle.x > 2.0:
            break
        vehicle = pursue(vehicle, result.setpoint, 25.0, 0.05)
    else:
        raise AssertionError("never reached an away waypoint")

    result = fsm.tick(vehicle, 20.0, 0.05)
    assert result.phase is Phase.RTL
    assert result.failsafe_reason == "battery_low"
    assert result.setpoint.x == pytest.approx(0.0)
    assert result.setpoint.y == pytest.approx(0.0)
    assert result.setpoint.z == pytest.approx(5.0)

    _, finished = fly_from(fsm, vehicle, battery=20.0)
    assert finished.phase is Phase.FAILSAFE
    assert finished.failsafe_reason == "battery_low"


def test_geofence_and_critical_battery_land_in_place():
    mission = Mission(
        name="tight",
        home=VehicleState(0.0, 0.0, 0.0),
        items=(MissionItem(CommandType.TAKEOFF, 0.0, 0.0, 5.0),),
        geofence=Geofence(0.0, 0.0, 30.0, -1.0, 40.0),
        rtl_altitude_m=5.0,
    )
    fsm = MissionFSM(mission)
    fsm.arm()
    fsm.start()
    outside = VehicleState(80.0, -2.0, 8.0)
    result = fsm.tick(outside, 100.0, 0.05)
    assert result.phase is Phase.LAND
    assert result.failsafe_reason == "geofence"
    assert result.setpoint.x == pytest.approx(outside.x)
    assert result.setpoint.y == pytest.approx(outside.y)
    assert result.setpoint.z == pytest.approx(0.0)
    assert not result.geofence_inside

    critical = MissionFSM(_square())
    critical.arm()
    critical.start()
    vehicle = VehicleState(3.0, 4.0, 6.0)
    result = critical.tick(vehicle, 5.0, 0.05)
    assert result.phase is Phase.LAND
    assert result.failsafe_reason == "battery_critical"
    assert result.setpoint.x == pytest.approx(3.0)
    assert result.setpoint.y == pytest.approx(4.0)
    assert result.setpoint.z == pytest.approx(0.0)


def test_cancel_lands_and_a_ground_abort_does_not():
    fsm = MissionFSM(_square())
    fsm.arm()
    fsm.request_land(VehicleState(0.0, 0.0, 0.0), "canceled")
    assert fsm.phase is Phase.IDLE

    fsm.arm()
    fsm.start()
    fsm.tick(VehicleState(0.0, 0.0, 1.0), 100.0, 0.05)
    fsm.request_land(VehicleState(1.0, 2.0, 4.0), "canceled")
    assert fsm.phase is Phase.LAND
    _, result = fly_from(fsm, VehicleState(1.0, 2.0, 4.0))
    assert result.phase is Phase.FAILSAFE
    assert result.failsafe_reason == "canceled"


def test_negative_dt_and_inconsistent_battery_thresholds():
    fsm = MissionFSM(_square())
    with pytest.raises(MissionError):
        fsm.tick(VehicleState(0.0, 0.0, 0.0), 100.0, -0.1)
    with pytest.raises(MissionError, match="battery_critical_percent"):
        _mission(
            [MissionItem(CommandType.LAND, 0.0, 0.0, 0.0)],
            battery_critical_percent=40.0,
            battery_failsafe_percent=20.0,
        )


def fly_from(fsm: MissionFSM, vehicle: VehicleState, battery: float = 100.0, limit: int = 800):
    result = None
    for _ in range(limit):
        result = fsm.tick(vehicle, battery, 0.05)
        if result.phase in (Phase.COMPLETE, Phase.FAILSAFE):
            return vehicle, result
        vehicle = pursue(vehicle, result.setpoint, 25.0, 0.05)
    raise AssertionError("mission did not finish")
