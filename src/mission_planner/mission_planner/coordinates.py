# Copyright 2026 Drone Waypoint Demo
"""Local tangent-plane conversion for demo missions.

The conversion is an equirectangular projection about a home latitude and
longitude. Over a few kilometers it is accurate to roughly a meter. It is
not a geodesy library and it is not suitable for navigation.
"""

import math

EARTH_RADIUS_M = 6378137.0


def wgs84_to_local_enu(lat_deg, lon_deg, home_lat_deg, home_lon_deg):
    """Return east and north meters relative to home.

    Raises ValueError when a coordinate is non-finite or outside WGS84 range.
    """
    _validate_lat_lon(lat_deg, lon_deg, "waypoint")
    _validate_lat_lon(home_lat_deg, home_lon_deg, "home")
    dlat = math.radians(lat_deg - home_lat_deg)
    dlon = math.radians(lon_deg - home_lon_deg)
    home_lat = math.radians(home_lat_deg)
    east = dlon * math.cos(home_lat) * EARTH_RADIUS_M
    north = dlat * EARTH_RADIUS_M
    return east, north


def _validate_lat_lon(lat_deg, lon_deg, label):
    if isinstance(lat_deg, bool) or isinstance(lon_deg, bool):
        raise ValueError("%s latitude/longitude must be numbers" % label)
    if not isinstance(lat_deg, (int, float)) or not isinstance(lon_deg, (int, float)):
        raise ValueError("%s latitude/longitude must be numbers" % label)
    if not math.isfinite(float(lat_deg)) or not math.isfinite(float(lon_deg)):
        raise ValueError("%s latitude/longitude must be finite" % label)
    if lat_deg < -90.0 or lat_deg > 90.0 or lon_deg < -180.0 or lon_deg > 180.0:
        raise ValueError("%s latitude/longitude is outside WGS84 range" % label)
