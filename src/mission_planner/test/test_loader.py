# SPDX-License-Identifier: MIT

import math
from pathlib import Path

import pytest

from mission_planner.geodesy import WGS84_A_M, lla_to_enu
from mission_planner.loader import load_mission
from mission_planner.types import CommandType, MissionError

MISSIONS = Path(__file__).resolve().parents[1] / "missions"
SQUARE = [(8.0, 0.0), (8.0, 8.0), (0.0, 8.0), (0.0, 0.0)]


def _waypoint_xy(mission):
    return [
        (item.x, item.y, item.z)
        for item in mission.items
        if item.kind is CommandType.WAYPOINT
    ]


def test_geodesy_moves_north_and_east():
    lat0 = 37.0
    lon0 = -122.0
    north = 100.0
    east = 100.0
    lat = lat0 + math.degrees(north / WGS84_A_M)
    lon = lon0 + math.degrees(east / (WGS84_A_M * math.cos(math.radians(lat0))))
    x, y, z = lla_to_enu(lat, lon, 15.0, lat0, lon0, 10.0)
    assert x == pytest.approx(east, abs=1e-6)
    assert y == pytest.approx(north, abs=1e-6)
    assert z == pytest.approx(5.0)


def test_yaml_square_synthesizes_takeoff_rtl_and_land():
    mission = load_mission(MISSIONS / "square.yaml")
    assert mission.name == "square"
    assert [item.kind for item in mission.items] == [
        CommandType.TAKEOFF,
        CommandType.WAYPOINT,
        CommandType.WAYPOINT,
        CommandType.WAYPOINT,
        CommandType.WAYPOINT,
        CommandType.RTL,
        CommandType.LAND,
    ]
    assert mission.items[0].z == pytest.approx(5.0)
    assert mission.items[-1].z == pytest.approx(0.0)
    assert mission.geofence.radius_m == pytest.approx(40.0)
    assert mission.items[2].yaw == pytest.approx(math.radians(90.0))
    assert _waypoint_xy(mission) == [
        (pytest.approx(8.0), pytest.approx(0.0), pytest.approx(5.0)),
        (pytest.approx(8.0), pytest.approx(8.0), pytest.approx(5.0)),
        (pytest.approx(0.0), pytest.approx(8.0), pytest.approx(5.0)),
        (pytest.approx(0.0), pytest.approx(0.0), pytest.approx(5.0)),
    ]


def test_qgc_plan_and_wpl_match_the_yaml_square():
    yaml_mission = load_mission(MISSIONS / "square.yaml")
    plan = load_mission(MISSIONS / "square.plan")
    wpl = load_mission(MISSIONS / "square.waypoints")
    assert plan.geofence.radius_m == pytest.approx(40.0)
    assert plan.geofence.center_x == pytest.approx(0.0, abs=1e-3)
    assert plan.geofence.center_y == pytest.approx(0.0, abs=1e-3)
    for loaded in (plan, wpl):
        points = _waypoint_xy(loaded)
        assert len(points) == 4
        for got, want in zip(points, SQUARE):
            assert got[0] == pytest.approx(want[0], abs=0.05)
            assert got[1] == pytest.approx(want[1], abs=0.05)
            assert got[2] == pytest.approx(5.0, abs=1e-6)
        assert loaded.items[0].kind is CommandType.TAKEOFF
        assert loaded.items[-2].kind is CommandType.RTL
        assert loaded.items[-2].x == pytest.approx(0.0, abs=0.05)
        assert loaded.items[-1].kind is CommandType.LAND
        assert loaded.items[-1].z == pytest.approx(0.0)
    assert yaml_mission.cruise_speed_m_s == pytest.approx(plan.cruise_speed_m_s)


def test_local_ned_waypoint_converts_to_enu(tmp_path: Path):
    plan = tmp_path / "ned.plan"
    plan.write_text(
        """
        {
          "fileType": "Plan",
          "mission": {
            "plannedHomePosition": [0, 0, 0],
            "items": [
              {"type": "SimpleItem", "command": 16, "frame": 1,
               "params": [0, 0, 0, 0, 5, 3, 2]}
            ]
          }
        }
        """,
        encoding="utf-8",
    )
    mission = load_mission(plan)
    item = mission.items[0]
    assert item.x == pytest.approx(3.0)
    assert item.y == pytest.approx(5.0)
    assert item.z == pytest.approx(-2.0)


def test_explicit_yaml_items_are_not_rewritten(tmp_path: Path):
    mission_path = tmp_path / "explicit.yaml"
    mission_path.write_text(
        """
        name: hover
        items:
          - {command: takeoff, z: 3}
          - {command: land}
        """,
        encoding="utf-8",
    )
    mission = load_mission(mission_path)
    assert [item.kind for item in mission.items] == [CommandType.TAKEOFF, CommandType.LAND]
    assert mission.items[0].z == pytest.approx(3.0)
    assert mission.items[1].x == pytest.approx(0.0)
    assert mission.items[1].z == pytest.approx(0.0)


def test_rejects_bad_files(tmp_path: Path):
    with pytest.raises(MissionError):
        load_mission(tmp_path / "missing.yaml")

    both = tmp_path / "both.yaml"
    both.write_text("waypoints: [{x: 1, y: 0, z: 1}]\nitems: [{command: land}]\n", encoding="utf-8")
    with pytest.raises(MissionError):
        load_mission(both)

    complex_item = tmp_path / "survey.plan"
    complex_item.write_text(
        '{"fileType":"Plan","mission":{"items":[{"type":"ComplexItem","command":16}]}}',
        encoding="utf-8",
    )
    with pytest.raises(MissionError, match="unsupported QGC item type"):
        load_mission(complex_item)

    bad_command = tmp_path / "bad.waypoints"
    bad_command.write_text("QGC WPL 110\n0 0 3 99 0 0 0 0 0 0 5 1\n", encoding="utf-8")
    with pytest.raises(MissionError, match="unsupported MAVLink command"):
        load_mission(bad_command)


def test_global_amsl_altitude_is_relative_to_home(tmp_path: Path):
    plan = tmp_path / "amsl.plan"
    plan.write_text(
        """
        {"fileType":"Plan","mission":{
          "plannedHomePosition":[10,20,100],
          "items":[{"type":"SimpleItem","command":16,"frame":0,
                    "params":[1,0,0,0,10,20,112]}]
        }}
        """,
        encoding="utf-8",
    )
    mission = load_mission(plan)
    assert mission.items[0].z == pytest.approx(12.0)
    assert mission.items[0].hold_s == pytest.approx(1.0)
    assert mission.items[0].x == pytest.approx(0.0, abs=1e-6)
    assert mission.items[0].y == pytest.approx(0.0, abs=1e-6)
