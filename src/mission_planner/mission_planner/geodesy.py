# SPDX-License-Identifier: MIT
"""Small-area WGS84 to local ENU conversion.

Equirectangular projection about a home point. Accurate enough for missions
of a few kilometres. It is not a geodesy library.
"""

from __future__ import annotations

import math

WGS84_A_M = 6378137.0


def lla_to_enu(
    lat_deg: float,
    lon_deg: float,
    alt_m: float,
    lat0_deg: float,
    lon0_deg: float,
    alt0_m: float,
) -> tuple[float, float, float]:
    lat0 = math.radians(lat0_deg)
    east = WGS84_A_M * math.radians(lon_deg - lon0_deg) * math.cos(lat0)
    north = WGS84_A_M * math.radians(lat_deg - lat0_deg)
    up = alt_m - alt0_m
    return east, north, up
