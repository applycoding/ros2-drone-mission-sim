# SPDX-License-Identifier: MIT

from mission_planner.geofence import evaluate_geofence
from mission_planner.types import Geofence

FENCE = Geofence(center_x=0.0, center_y=0.0, radius_m=10.0, min_alt_m=0.0, max_alt_m=20.0)


def test_inside_and_on_the_boundary():
    inside = evaluate_geofence(FENCE, 3.0, 4.0, 10.0)
    assert inside.inside
    assert inside.breach_type == "none"
    assert inside.distance_to_boundary_m == 5.0

    boundary = evaluate_geofence(FENCE, 10.0, 0.0, 20.0)
    assert boundary.inside
    assert boundary.distance_to_boundary_m == 0.0


def test_horizontal_altitude_and_combined_breaches():
    horizontal = evaluate_geofence(FENCE, 12.0, 0.0, 5.0)
    assert not horizontal.inside
    assert horizontal.breach_type == "horizontal"
    assert horizontal.distance_to_boundary_m == -2.0

    altitude = evaluate_geofence(FENCE, 0.0, 0.0, 25.0)
    assert altitude.breach_type == "altitude"
    assert altitude.distance_to_boundary_m == -5.0

    both = evaluate_geofence(FENCE, 0.0, 40.0, -2.0)
    assert both.breach_type == "horizontal_and_altitude"
    assert both.distance_to_boundary_m < 0.0
