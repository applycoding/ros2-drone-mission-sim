# SPDX-License-Identifier: MIT
"""Vertical-cylinder geofence in the local ENU frame."""

from __future__ import annotations

import math
from dataclasses import dataclass

from mission_planner.types import Geofence


@dataclass(frozen=True)
class GeofenceCheck:
    inside: bool
    distance_to_boundary_m: float
    breach_type: str


def evaluate_geofence(fence: Geofence, x: float, y: float, z: float) -> GeofenceCheck:
    horizontal = math.hypot(x - fence.center_x, y - fence.center_y)
    horizontal_margin = fence.radius_m - horizontal
    altitude_margin = min(z - fence.min_alt_m, fence.max_alt_m - z)
    horizontal_breach = horizontal_margin < 0.0
    altitude_breach = altitude_margin < 0.0
    if horizontal_breach and altitude_breach:
        breach = "horizontal_and_altitude"
    elif horizontal_breach:
        breach = "horizontal"
    elif altitude_breach:
        breach = "altitude"
    else:
        breach = "none"
    return GeofenceCheck(
        inside=breach == "none",
        distance_to_boundary_m=min(horizontal_margin, altitude_margin),
        breach_type=breach,
    )
