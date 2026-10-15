# Copyright 2026 Drone Waypoint Demo
"""Loader tests using temporary mission files."""

import json

import pytest
import yaml

from mission_planner.loaders import detect_format, load_mission


def test_yaml_round_trip_fields(tmp_path):
    path = tmp_path / "mission.yaml"
    path.write_text(
        yaml.safe_dump(
            {
                "frame": "local_enu",
                "home": {"x": 1.0, "y": 2.0, "z": 0.5},
                "waypoints": [
                    {
                        "name": "pad",
                        "x": 1.0,
                        "y": 2.0,
                        "z": 5.0,
                        "yaw": 0.3,
                        "hold_sec": 1.5,
                        "acceptance_radius": 0.8,
                    }
                ],
            }
        ),
        encoding="utf-8",
    )
    mission = load_mission(path, "auto")
    assert mission.frame == "local_enu"
    assert mission.home_x == 1.0
    assert mission.waypoints[0].name == "pad"
    assert mission.waypoints[0].z == 5.0
    assert mission.waypoints[0].hold_sec == 1.5


def test_geojson_projects_about_the_declared_home(tmp_path):
    path = tmp_path / "mission.geojson"
    path.write_text(
        json.dumps(
            {
                "type": "FeatureCollection",
                "properties": {"home": {"lat": 0.0, "lon": 0.0, "alt": 2.0}},
                "features": [
                    {
                        "type": "Feature",
                        "properties": {"name": "east", "hold_sec": 0.0},
                        "geometry": {
                            "type": "Point",
                            "coordinates": [0.0001, 0.0, 7.0],
                        },
                    }
                ],
            }
        ),
        encoding="utf-8",
    )
    mission = load_mission(str(path), "geojson")
    assert mission.waypoints[0].name == "east"
    assert mission.waypoints[0].x == pytest.approx(11.131949, abs=1e-3)
    assert mission.waypoints[0].y == pytest.approx(0.0, abs=1e-6)
    assert mission.waypoints[0].z == pytest.approx(5.0)


def test_missing_home_uses_the_first_point_as_origin(tmp_path):
    path = tmp_path / "mission.json"
    document = {
        "type": "FeatureCollection",
        "features": [
            {
                "type": "Feature",
                "properties": {"alt": 4.0},
                "geometry": {"type": "Point", "coordinates": [-122.0, 37.0]},
            }
        ],
    }
    path.write_text(json.dumps(document), encoding="utf-8")
    mission = load_mission(path)
    assert mission.waypoints[0].name == "wp1"
    assert mission.waypoints[0].x == pytest.approx(0.0, abs=1e-6)
    assert mission.waypoints[0].y == pytest.approx(0.0, abs=1e-6)
    assert mission.waypoints[0].z == pytest.approx(4.0)


def test_rejects_bad_files(tmp_path):
    empty = tmp_path / "empty.yaml"
    empty.write_text("waypoints: []\n", encoding="utf-8")
    with pytest.raises(ValueError):
        load_mission(empty)

    missing = tmp_path / "missing.yaml"
    missing.write_text("waypoints:\n  - name: a\n    x: 1\n", encoding="utf-8")
    with pytest.raises(ValueError):
        load_mission(missing)

    line = tmp_path / "line.geojson"
    line.write_text(
        json.dumps(
            {
                "type": "FeatureCollection",
                "features": [
                    {
                        "type": "Feature",
                        "geometry": {"type": "LineString", "coordinates": [[0, 0], [1, 1]]},
                    }
                ],
            }
        ),
        encoding="utf-8",
    )
    with pytest.raises(ValueError):
        load_mission(line)

    bad_radius = tmp_path / "radius.yaml"
    bad_radius.write_text(
        "waypoints:\n  - x: 0\n    y: 0\n    z: 1\n    acceptance_radius: 0\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError):
        load_mission(bad_radius)

    with pytest.raises(FileNotFoundError):
        load_mission(tmp_path / "nope.yaml")


def test_format_detection(tmp_path):
    path = tmp_path / "mission.yaml"
    assert detect_format(path, "auto").value == "yaml"
    assert detect_format(path, "geojson").value == "geojson"
    with pytest.raises(ValueError):
        detect_format(tmp_path / "mission.txt", "auto")
    with pytest.raises(ValueError):
        detect_format(path, "kml")
