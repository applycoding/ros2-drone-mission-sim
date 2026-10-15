# SPDX-License-Identifier: MIT
"""Load a local YAML mission or a small subset of QGroundControl plans.

YAML waypoint lists gain an implicit takeoff, RTL, and land. QGC files are
followed as written. Supported QGC commands are NAV_WAYPOINT (16),
NAV_RETURN_TO_LAUNCH (20), NAV_LAND (21), and NAV_TAKEOFF (22). Supported
frames are global AMSL (0), local NED (1), and global-relative altitude (3).
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import yaml

from mission_planner.geodesy import lla_to_enu
from mission_planner.types import (
    CommandType,
    Geofence,
    Mission,
    MissionError,
    MissionItem,
    VehicleState,
)

MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_NAV_RTL = 20
MAV_CMD_NAV_LAND = 21
MAV_CMD_NAV_TAKEOFF = 22

MAV_FRAME_GLOBAL = 0
MAV_FRAME_LOCAL_NED = 1
MAV_FRAME_GLOBAL_RELATIVE_ALT = 3

DEFAULT_ACCEPTANCE_M = 0.5
DEFAULT_CRUISE_M_S = 2.0
DEFAULT_BATTERY_FAILSAFE_PERCENT = 25.0
DEFAULT_BATTERY_CRITICAL_PERCENT = 10.0
DEFAULT_FENCE_RADIUS_M = 100.0
DEFAULT_MIN_ALT_M = -1.0
DEFAULT_MAX_ALT_M = 120.0
DEFAULT_TAKEOFF_ALT_M = 5.0
DEFAULT_RTL_ALT_M = 5.0

_MAV_COMMANDS = {
    MAV_CMD_NAV_WAYPOINT: CommandType.WAYPOINT,
    MAV_CMD_NAV_RTL: CommandType.RTL,
    MAV_CMD_NAV_LAND: CommandType.LAND,
    MAV_CMD_NAV_TAKEOFF: CommandType.TAKEOFF,
}

_YAML_COMMANDS = {kind.value: kind for kind in CommandType}


def load_mission(path: str | Path) -> Mission:
    mission_path = Path(path)
    if not mission_path.is_file():
        raise MissionError(f"mission file not found: {mission_path}")
    text = mission_path.read_text(encoding="utf-8")
    kind = _detect_format(mission_path, text)
    name = mission_path.stem
    if kind == "yaml":
        return _load_yaml(text, name)
    if kind == "plan":
        return _load_plan(text, name)
    if kind == "wpl":
        return _load_wpl(text, name)
    raise MissionError(f"unrecognized mission format: {kind}")


def _detect_format(path: Path, text: str) -> str:
    suffix = path.suffix.lower()
    if suffix in {".yaml", ".yml"}:
        return "yaml"
    if suffix in {".plan", ".json"}:
        return "plan"
    if suffix in {".waypoints", ".wpl", ".txt"}:
        return "wpl"
    stripped = text.lstrip()
    if stripped.startswith("QGC WPL"):
        return "wpl"
    if stripped.startswith("{"):
        return "plan"
    return "yaml"


def _load_yaml(text: str, fallback_name: str) -> Mission:
    raw = yaml.safe_load(text)
    if not isinstance(raw, dict):
        raise MissionError("YAML mission must be a mapping")
    frame = str(raw.get("frame", "local_enu"))
    if frame != "local_enu":
        raise MissionError("YAML missions must use frame local_enu")
    home = _home_from_yaml(raw.get("home", {}))
    acceptance = _optional_float(raw, "acceptance_radius_m", DEFAULT_ACCEPTANCE_M)
    cruise = _optional_float(raw, "cruise_speed_m_s", DEFAULT_CRUISE_M_S)
    battery_low = _optional_float(
        raw, "battery_failsafe_percent", DEFAULT_BATTERY_FAILSAFE_PERCENT
    )
    battery_critical = _optional_float(
        raw, "battery_critical_percent", DEFAULT_BATTERY_CRITICAL_PERCENT
    )
    takeoff_alt = _optional_float(raw, "takeoff_altitude_m", DEFAULT_TAKEOFF_ALT_M)
    rtl_alt = _optional_float(raw, "rtl_altitude_m", DEFAULT_RTL_ALT_M)
    fence = _fence_from_yaml(raw.get("geofence"), home)
    if "items" in raw and "waypoints" in raw:
        raise MissionError("use either items or waypoints, not both")
    if "items" in raw:
        items = _explicit_items(raw["items"], home, takeoff_alt, rtl_alt)
    elif "waypoints" in raw:
        items = _items_from_waypoints(raw["waypoints"], home, takeoff_alt, rtl_alt)
    else:
        raise MissionError("YAML mission needs items or waypoints")
    return _build(
        name=str(raw.get("name", fallback_name)),
        home=home,
        items=items,
        geofence=fence,
        acceptance=acceptance,
        cruise=cruise,
        battery_low=battery_low,
        battery_critical=battery_critical,
        rtl_alt=rtl_alt,
        frame=frame,
    )


def _load_plan(text: str, fallback_name: str) -> Mission:
    try:
        raw = json.loads(text)
    except json.JSONDecodeError as exc:
        raise MissionError(f"invalid QGC plan JSON: {exc}") from exc
    if not isinstance(raw, dict) or raw.get("fileType") != "Plan":
        raise MissionError("QGC plan fileType must be Plan")
    mission = raw.get("mission")
    if not isinstance(mission, dict):
        raise MissionError("QGC plan is missing mission")
    home_ll = mission.get("plannedHomePosition") or [0.0, 0.0, 0.0]
    if not isinstance(home_ll, list) or len(home_ll) < 2:
        raise MissionError("plannedHomePosition must be [lat, lon, alt]")
    home_lat = _number(home_ll[0], "home latitude")
    home_lon = _number(home_ll[1], "home longitude")
    home_alt = _number(home_ll[2], "home altitude") if len(home_ll) > 2 else 0.0
    home = VehicleState(0.0, 0.0, 0.0, 0.0)
    items = []
    for entry in mission.get("items") or []:
        if not isinstance(entry, dict):
            raise MissionError("QGC mission item must be an object")
        item_type = entry.get("type", "SimpleItem")
        if item_type != "SimpleItem":
            raise MissionError(f"unsupported QGC item type {item_type}")
        items.append(_item_from_mav(entry, home_lat, home_lon, home_alt, home))
    cruise = _optional_float(mission, "cruiseSpeed", DEFAULT_CRUISE_M_S)
    fence = _fence_from_plan(raw.get("geoFence"), home_lat, home_lon)
    return _build(
        name=str(mission.get("name", fallback_name)),
        home=home,
        items=items,
        geofence=fence,
        acceptance=DEFAULT_ACCEPTANCE_M,
        cruise=cruise if cruise > 0.0 else DEFAULT_CRUISE_M_S,
        battery_low=DEFAULT_BATTERY_FAILSAFE_PERCENT,
        battery_critical=DEFAULT_BATTERY_CRITICAL_PERCENT,
        rtl_alt=DEFAULT_RTL_ALT_M,
        frame="local_enu",
    )


def _load_wpl(text: str, fallback_name: str) -> Mission:
    lines = [line.strip() for line in text.splitlines() if line.strip()]
    if not lines or not lines[0].startswith("QGC WPL"):
        raise MissionError("WPL mission must start with a QGC WPL header")
    rows = []
    for line in lines[1:]:
        if line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) < 11:
            raise MissionError(f"WPL row has {len(parts)} columns, expected at least 11")
        rows.append(parts)
    if not rows:
        raise MissionError("WPL mission has no items")
    home_lat, home_lon, home_alt = _wpl_home(rows)
    home = VehicleState(0.0, 0.0, 0.0, 0.0)
    items = []
    for index, parts in enumerate(rows):
        command = int(float(parts[3]))
        current = int(float(parts[1]))
        if index == 0 and command == MAV_CMD_NAV_WAYPOINT and current == 1:
            continue
        entry = {
            "command": command,
            "frame": int(float(parts[2])),
            "params": [float(parts[i]) for i in range(4, 11)],
        }
        items.append(_item_from_mav(entry, home_lat, home_lon, home_alt, home))
    return _build(
        name=fallback_name,
        home=home,
        items=items,
        geofence=Geofence(0.0, 0.0, DEFAULT_FENCE_RADIUS_M, DEFAULT_MIN_ALT_M, DEFAULT_MAX_ALT_M),
        acceptance=DEFAULT_ACCEPTANCE_M,
        cruise=DEFAULT_CRUISE_M_S,
        battery_low=DEFAULT_BATTERY_FAILSAFE_PERCENT,
        battery_critical=DEFAULT_BATTERY_CRITICAL_PERCENT,
        rtl_alt=DEFAULT_RTL_ALT_M,
        frame="local_enu",
    )


def _item_from_mav(
    entry: dict,
    home_lat: float,
    home_lon: float,
    home_alt: float,
    home: VehicleState,
) -> MissionItem:
    command = int(entry["command"])
    try:
        kind = _MAV_COMMANDS[command]
    except KeyError as exc:
        raise MissionError(f"unsupported MAVLink command {command}") from exc
    frame = int(entry.get("frame", MAV_FRAME_GLOBAL_RELATIVE_ALT))
    params = list(entry.get("params") or [])
    lat_or_x = _param(params, 4)
    lon_or_y = _param(params, 5)
    alt_or_z = _param(params, 6)
    yaw_deg = _param(params, 3)
    hold_s = _param(params, 0) if kind is CommandType.WAYPOINT else 0.0
    x, y, z = _mav_to_enu(
        frame, lat_or_x, lon_or_y, alt_or_z, home_lat, home_lon, home_alt, home
    )
    if kind is CommandType.RTL and z <= 0.0:
        z = DEFAULT_RTL_ALT_M
    if kind is CommandType.TAKEOFF and z <= 0.0:
        z = DEFAULT_TAKEOFF_ALT_M
    if kind is CommandType.LAND:
        z = 0.0
    if kind in (CommandType.RTL, CommandType.LAND) and frame != MAV_FRAME_LOCAL_NED:
        # QGC often leaves RTL/LAND coordinates at zero. Fly home instead.
        if abs(lat_or_x) < 1e-8 and abs(lon_or_y) < 1e-8:
            x, y = home.x, home.y
    return MissionItem(kind, x, y, z, math.radians(yaw_deg), max(0.0, hold_s))


def _mav_to_enu(
    frame: int,
    lat_or_x: float,
    lon_or_y: float,
    alt_or_z: float,
    home_lat: float,
    home_lon: float,
    home_alt: float,
    home: VehicleState,
) -> tuple[float, float, float]:
    if frame == MAV_FRAME_LOCAL_NED:
        return lon_or_y + home.x, lat_or_x + home.y, -alt_or_z
    if frame == MAV_FRAME_GLOBAL_RELATIVE_ALT:
        east, north, _ = lla_to_enu(lat_or_x, lon_or_y, 0.0, home_lat, home_lon, 0.0)
        return east, north, alt_or_z
    if frame == MAV_FRAME_GLOBAL:
        return lla_to_enu(lat_or_x, lon_or_y, alt_or_z, home_lat, home_lon, home_alt)
    raise MissionError(f"unsupported MAVLink frame {frame}")


def _wpl_home(rows: list[list[str]]) -> tuple[float, float, float]:
    parts = rows[0]
    return float(parts[8]), float(parts[9]), float(parts[10])


def _fence_from_plan(raw: object, home_lat: float, home_lon: float) -> Geofence:
    if isinstance(raw, dict):
        for circle in raw.get("circles") or []:
            if not isinstance(circle, dict) or circle.get("inclusion", True) is False:
                continue
            nested = circle.get("circle")
            source = nested if isinstance(nested, dict) else circle
            center = source.get("center") or [home_lat, home_lon]
            if not isinstance(center, list) or len(center) < 2:
                raise MissionError("geofence circle center must be [lat, lon]")
            east, north, _ = lla_to_enu(
                _number(center[0], "geofence latitude"),
                _number(center[1], "geofence longitude"),
                0.0,
                home_lat,
                home_lon,
                0.0,
            )
            radius = _number(source.get("radius", DEFAULT_FENCE_RADIUS_M), "geofence radius")
            return Geofence(east, north, radius, DEFAULT_MIN_ALT_M, DEFAULT_MAX_ALT_M)
    return Geofence(0.0, 0.0, DEFAULT_FENCE_RADIUS_M, DEFAULT_MIN_ALT_M, DEFAULT_MAX_ALT_M)


def _home_from_yaml(raw: object) -> VehicleState:
    if raw is None:
        raw = {}
    if not isinstance(raw, dict):
        raise MissionError("home must be a mapping")
    return VehicleState(
        _optional_float(raw, "x", 0.0),
        _optional_float(raw, "y", 0.0),
        _optional_float(raw, "z", 0.0),
        math.radians(_optional_float(raw, "yaw_deg", 0.0)),
    )


def _fence_from_yaml(raw: object, home: VehicleState) -> Geofence:
    if raw is None:
        raw = {}
    if not isinstance(raw, dict):
        raise MissionError("geofence must be a mapping")
    return Geofence(
        _optional_float(raw, "center_x", home.x),
        _optional_float(raw, "center_y", home.y),
        _optional_float(raw, "radius_m", DEFAULT_FENCE_RADIUS_M),
        _optional_float(raw, "min_alt_m", DEFAULT_MIN_ALT_M),
        _optional_float(raw, "max_alt_m", DEFAULT_MAX_ALT_M),
    )


def _items_from_waypoints(
    raw: object, home: VehicleState, takeoff_alt: float, rtl_alt: float
) -> list[MissionItem]:
    if not isinstance(raw, list) or not raw:
        raise MissionError("waypoints must be a non-empty list")
    items = [MissionItem(CommandType.TAKEOFF, home.x, home.y, takeoff_alt, home.yaw, 0.0)]
    for index, entry in enumerate(raw):
        if not isinstance(entry, dict):
            raise MissionError(f"waypoint {index} must be a mapping")
        items.append(
            MissionItem(
                CommandType.WAYPOINT,
                _required_float(entry, "x", f"waypoint {index}"),
                _required_float(entry, "y", f"waypoint {index}"),
                _required_float(entry, "z", f"waypoint {index}"),
                math.radians(_optional_float(entry, "yaw_deg", 0.0)),
                _optional_float(entry, "hold_s", 0.0),
            )
        )
    items.append(MissionItem(CommandType.RTL, home.x, home.y, rtl_alt, home.yaw, 0.0))
    items.append(MissionItem(CommandType.LAND, home.x, home.y, 0.0, home.yaw, 0.0))
    return items


def _explicit_items(
    raw: object, home: VehicleState, takeoff_alt: float, rtl_alt: float
) -> list[MissionItem]:
    if not isinstance(raw, list) or not raw:
        raise MissionError("items must be a non-empty list")
    items = []
    for index, entry in enumerate(raw):
        if not isinstance(entry, dict):
            raise MissionError(f"item {index} must be a mapping")
        command_name = str(entry.get("command", "")).lower()
        try:
            kind = _YAML_COMMANDS[command_name]
        except KeyError as exc:
            raise MissionError(f"item {index} has unsupported command {command_name}") from exc
        if kind is CommandType.TAKEOFF:
            default_z = takeoff_alt
        elif kind is CommandType.WAYPOINT:
            default_z = takeoff_alt
        elif kind is CommandType.RTL:
            default_z = rtl_alt
        elif kind is CommandType.LAND:
            default_z = 0.0
        else:
            raise MissionError(f"unhandled command {kind}")
        items.append(
            MissionItem(
                kind,
                _optional_float(entry, "x", home.x),
                _optional_float(entry, "y", home.y),
                _optional_float(entry, "z", default_z),
                math.radians(_optional_float(entry, "yaw_deg", math.degrees(home.yaw))),
                _optional_float(entry, "hold_s", 0.0),
            )
        )
    return items


def _build(
    name: str,
    home: VehicleState,
    items: list[MissionItem],
    geofence: Geofence,
    acceptance: float,
    cruise: float,
    battery_low: float,
    battery_critical: float,
    rtl_alt: float,
    frame: str,
) -> Mission:
    if not items:
        raise MissionError("mission has no items")
    return Mission(
        name=name,
        home=home,
        items=tuple(items),
        geofence=geofence,
        acceptance_radius_m=acceptance,
        cruise_speed_m_s=cruise,
        battery_failsafe_percent=battery_low,
        battery_critical_percent=battery_critical,
        rtl_altitude_m=rtl_alt,
        frame=frame,
    )


def _optional_float(raw: dict, key: str, default: float) -> float:
    if key not in raw or raw[key] is None:
        return default
    return _number(raw[key], key)


def _required_float(raw: dict, key: str, where: str) -> float:
    if key not in raw:
        raise MissionError(f"{where} is missing {key}")
    return _number(raw[key], f"{where}.{key}")


def _number(value: object, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise MissionError(f"{field} must be a number")
    number = float(value)
    if not math.isfinite(number):
        raise MissionError(f"{field} must be finite")
    return number


def _param(params: list, index: int) -> float:
    if index >= len(params) or params[index] is None:
        return 0.0
    return _number(params[index], f"params[{index}]")
