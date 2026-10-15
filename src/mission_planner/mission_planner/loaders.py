# Copyright 2026 Drone Waypoint Demo
"""Load waypoint missions from YAML or GeoJSON into local ENU meters."""

import enum
import json
import math
from pathlib import Path

import yaml

from mission_planner.coordinates import wgs84_to_local_enu
from mission_planner.models import Mission, Waypoint


class MissionFormat(enum.Enum):
    """On-disk encodings this demo knows how to read."""

    YAML = "yaml"
    GEOJSON = "geojson"


def load_mission(path, declared_format="auto"):
    """Load a mission file.

    declared_format is auto, yaml, or geojson. auto uses the file suffix.
    """
    mission_format = detect_format(path, declared_format)
    if mission_format is MissionFormat.YAML:
        return load_yaml(path)
    if mission_format is MissionFormat.GEOJSON:
        return load_geojson(path)
    raise AssertionError("unhandled mission format %s" % mission_format)


def detect_format(path, declared_format="auto"):
    """Resolve auto to a concrete format, or reject an unknown declaration."""
    declared = (declared_format or "auto").strip().lower()
    if declared in ("yaml", "yml"):
        return MissionFormat.YAML
    if declared in ("geojson", "json"):
        return MissionFormat.GEOJSON
    if declared != "auto":
        raise ValueError("unsupported mission format '%s'" % declared_format)
    suffix = Path(path).suffix.lower()
    if suffix in (".yaml", ".yml"):
        return MissionFormat.YAML
    if suffix in (".geojson", ".json"):
        return MissionFormat.GEOJSON
    raise ValueError("cannot detect mission format from '%s'" % path)


def load_yaml(path):
    """Load a local-ENU YAML mission."""
    raw = _read_yaml(path)
    if not isinstance(raw, dict):
        raise ValueError("YAML mission must be a mapping")
    frame = raw.get("frame", "local_enu")
    if frame != "local_enu":
        raise ValueError("YAML missions use frame local_enu")
    home = raw.get("home") or {}
    if not isinstance(home, dict):
        raise ValueError("home must be a mapping")
    home_x = _number(home.get("x", 0.0), "home.x")
    home_y = _number(home.get("y", 0.0), "home.y")
    home_z = _number(home.get("z", 0.0), "home.z")
    waypoints = _yaml_waypoints(raw.get("waypoints"))
    return Mission(
        frame="local_enu",
        waypoints=tuple(waypoints),
        home_x=home_x,
        home_y=home_y,
        home_z=home_z,
        source=str(path),
    )


def load_geojson(path):
    """Load a GeoJSON FeatureCollection and project it about its home.

    Altitude is meters above the home altitude. Collection properties.home
    supplies the origin. When it is omitted, the first point is the origin
    and that point's own local position is east=0, north=0.
    """
    raw = _read_json(path)
    if not isinstance(raw, dict) or raw.get("type") != "FeatureCollection":
        raise ValueError("GeoJSON mission must be a FeatureCollection")
    features = raw.get("features")
    if not isinstance(features, list) or not features:
        raise ValueError("GeoJSON mission needs at least one Point feature")
    for feature in features:
        _require_point(feature)
    home_lat, home_lon, home_alt = _geo_home(raw, features)
    waypoints = [
        _geo_waypoint(feature, index, home_lat, home_lon, home_alt)
        for index, feature in enumerate(features, start=1)
    ]
    return Mission(
        frame="wgs84",
        waypoints=tuple(waypoints),
        home_x=0.0,
        home_y=0.0,
        home_z=0.0,
        source=str(path),
    )


def _yaml_waypoints(raw_waypoints):
    if not isinstance(raw_waypoints, list) or not raw_waypoints:
        raise ValueError("waypoints must be a non-empty list")
    waypoints = []
    for index, item in enumerate(raw_waypoints, start=1):
        if not isinstance(item, dict):
            raise ValueError("waypoint %d must be a mapping" % index)
        for key in ("x", "y", "z"):
            if key not in item:
                raise ValueError("waypoint %d is missing %s" % (index, key))
        waypoints.append(
            _make_waypoint(
                item.get("name"),
                index,
                _number(item["x"], "waypoint %d x" % index),
                _number(item["y"], "waypoint %d y" % index),
                _number(item["z"], "waypoint %d z" % index),
                item.get("yaw", 0.0),
                item.get("hold_sec", 0.0),
                item.get("acceptance_radius", 0.75),
            )
        )
    return waypoints


def _geo_home(raw, features):
    props = raw.get("properties") or {}
    if not isinstance(props, dict):
        raise ValueError("FeatureCollection properties must be a mapping")
    home = props.get("home")
    if home is None:
        lon, lat, _alt = _coordinates(features[0])
        return lat, lon, 0.0
    if not isinstance(home, dict):
        raise ValueError("properties.home must be a mapping")
    return (
        _number(home.get("lat"), "home.lat"),
        _number(home.get("lon"), "home.lon"),
        _number(home.get("alt", 0.0), "home.alt"),
    )


def _geo_waypoint(feature, index, home_lat, home_lon, home_alt):
    lon, lat, _listed_alt = _coordinates(feature)
    east, north = wgs84_to_local_enu(lat, lon, home_lat, home_lon)
    altitude = _feature_altitude(feature, index) - home_alt
    props = feature.get("properties") or {}
    return _make_waypoint(
        props.get("name"),
        index,
        east,
        north,
        altitude,
        props.get("yaw", 0.0),
        props.get("hold_sec", 0.0),
        props.get("acceptance_radius", 0.75),
    )


def _make_waypoint(name, index, x, y, z, yaw, hold_sec, acceptance_radius):
    label = name.strip() if isinstance(name, str) else ""
    if not label:
        label = "wp%d" % index
    yaw_value = _number(yaw, "%s yaw" % label)
    hold = _number(hold_sec, "%s hold_sec" % label)
    radius = _number(acceptance_radius, "%s acceptance_radius" % label)
    if hold < 0.0:
        raise ValueError("%s hold_sec must be >= 0" % label)
    if radius <= 0.0:
        raise ValueError("%s acceptance_radius must be > 0" % label)
    return Waypoint(
        name=label,
        x=x,
        y=y,
        z=z,
        yaw=yaw_value,
        hold_sec=hold,
        acceptance_radius=radius,
    )


def _feature_altitude(feature, index):
    props = feature.get("properties") or {}
    if isinstance(props, dict) and "alt" in props:
        return _number(props["alt"], "waypoint %d alt" % index)
    _lon, _lat, alt = _coordinates(feature)
    if alt is None:
        raise ValueError("waypoint %d is missing an altitude" % index)
    return _number(alt, "waypoint %d altitude" % index)


def _coordinates(feature):
    geometry = feature.get("geometry") or {}
    coordinates = geometry.get("coordinates")
    if not isinstance(coordinates, (list, tuple)) or len(coordinates) < 2:
        raise ValueError("Point coordinates must be [lon, lat] or [lon, lat, alt]")
    lon = _number(coordinates[0], "longitude")
    lat = _number(coordinates[1], "latitude")
    alt = None
    if len(coordinates) >= 3 and coordinates[2] is not None:
        alt = coordinates[2]
    return lon, lat, alt


def _require_point(feature):
    if not isinstance(feature, dict) or feature.get("type") != "Feature":
        raise ValueError("each GeoJSON entry must be a Feature")
    geometry = feature.get("geometry") or {}
    if not isinstance(geometry, dict) or geometry.get("type") != "Point":
        raise ValueError("only Point features are accepted")


def _number(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("%s must be a number" % label)
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("%s must be finite" % label)
    return number


def _read_yaml(path):
    file_path = _existing_file(path)
    with file_path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def _read_json(path):
    file_path = _existing_file(path)
    with file_path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def _existing_file(path):
    file_path = Path(path)
    if not file_path.is_file():
        raise FileNotFoundError("mission file not found: %s" % path)
    return file_path
