# Copyright 2026 Drone Waypoint Demo
"""The missions shipped with the bringup package load and fit the demo fence."""

from pathlib import Path

from mission_planner.loaders import load_mission

_REPO = Path(__file__).resolve().parents[3]
_CONFIG = _REPO / "src" / "drone_bringup" / "config"


def test_sample_missions_fit_inside_the_demo_geofence():
    yaml_mission = load_mission(_CONFIG / "survey_square.yaml")
    geo_mission = load_mission(_CONFIG / "survey_square.geojson", "geojson")
    assert len(yaml_mission.waypoints) >= 4
    assert len(geo_mission.waypoints) >= 4
    for mission in (yaml_mission, geo_mission):
        for waypoint in mission.waypoints:
            assert abs(waypoint.x) < 20.0
            assert abs(waypoint.y) < 20.0
            assert 0.0 <= waypoint.z <= 20.0
