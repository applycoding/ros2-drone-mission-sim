#!/usr/bin/env python3
# Copyright 2026 Drone Waypoint Demo
"""Confirm a running demo serves the page, accepts start, and leaves the ground."""

import json
import sys
import time
import urllib.error
import urllib.request

BASE = "http://127.0.0.1:8080"


def _request(method, path, payload=None):
    data = None
    headers = {}
    if payload is not None:
        data = json.dumps(payload).encode("utf-8")
        headers["Content-Type"] = "application/json"
    request = urllib.request.Request(
        BASE + path, data=data, headers=headers, method=method
    )
    with urllib.request.urlopen(request, timeout=2) as response:
        raw = response.read().decode("utf-8")
        if path == "/":
            return response.status, raw
        return response.status, json.loads(raw)


def _wait(check, timeout, label):
    deadline = time.time() + timeout
    last = None
    while time.time() < deadline:
        try:
            last = check()
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError, RuntimeError) as exc:
            last = exc
        else:
            if last:
                return last
        time.sleep(0.25)
    print("timed out waiting for %s" % label, file=sys.stderr)
    print("last observation: %r" % (last,), file=sys.stderr)
    return None


def _page_ready():
    status, body = _request("GET", "/")
    return status == 200 and "Kinematic desktop demo" in body


def _plan_ready():
    status, body = _request("GET", "/api/state")
    if status != 200:
        return False
    waypoints = body.get("plan", {}).get("waypoints") or []
    mission = body.get("mission", {})
    if len(waypoints) < 4 or mission.get("waypoint_count", 0) < 4:
        return False
    drone = body.get("drone", {})
    if abs(float(drone.get("geofence_radius", 0.0)) - 40.0) > 0.1:
        return False
    return body


def _airborne():
    status, body = _request("GET", "/api/state")
    if status != 200:
        return False
    drone = body.get("drone", {})
    mission = body.get("mission", {})
    if mission.get("state") != "RUNNING":
        return False
    if drone.get("mode") != "FLYING":
        return False
    if float(drone.get("z", 0.0)) <= 0.5:
        return False
    return body


def main():
    if not _wait(_page_ready, 25, "ground station page"):
        return 1
    print("ground station page is up")
    if not _wait(_plan_ready, 20, "loaded mission"):
        return 1
    print("mission loaded")
    status, body = _request("POST", "/api/command", {"command": "start"})
    if status != 202 or not body.get("accepted"):
        print("start was not accepted: %s %r" % (status, body), file=sys.stderr)
        return 1
    airborne = _wait(_airborne, 25, "vehicle to fly")
    if not airborne:
        return 1
    drone = airborne["drone"]
    print(
        "flying: mode=%s z=%.2f battery=%.1f"
        % (drone["mode"], drone["z"], drone["battery_percent"])
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
