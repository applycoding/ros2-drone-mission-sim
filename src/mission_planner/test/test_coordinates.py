# Copyright 2026 Drone Waypoint Demo
"""Coordinate conversion checks with an independently written expectation."""

import math

import pytest

from mission_planner.coordinates import EARTH_RADIUS_M, wgs84_to_local_enu


def test_home_projects_to_the_origin():
    east, north = wgs84_to_local_enu(37.0, -122.0, 37.0, -122.0)
    assert east == pytest.approx(0.0, abs=1e-9)
    assert north == pytest.approx(0.0, abs=1e-9)


def test_equator_offset_matches_the_radius_definition():
    delta_deg = 0.0001
    expected = math.radians(delta_deg) * EARTH_RADIUS_M
    east, north = wgs84_to_local_enu(0.0, delta_deg, 0.0, 0.0)
    assert east == pytest.approx(expected, rel=1e-12)
    assert north == pytest.approx(0.0, abs=1e-9)
    east, north = wgs84_to_local_enu(delta_deg, 0.0, 0.0, 0.0)
    assert north == pytest.approx(expected, rel=1e-12)
    assert east == pytest.approx(0.0, abs=1e-9)


def test_rejects_coordinates_outside_wgs84():
    with pytest.raises(ValueError):
        wgs84_to_local_enu(91.0, 0.0, 0.0, 0.0)
    with pytest.raises(ValueError):
        wgs84_to_local_enu(0.0, 0.0, 0.0, float("nan"))
