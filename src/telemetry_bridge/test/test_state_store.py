# Copyright 2026 Drone Waypoint Demo
"""State store behavior."""

from telemetry_bridge.state_store import CommandQueue, StateStore


def test_snapshot_tracks_updates_without_sharing_the_internal_dict():
    store = StateStore()
    health = store.health()
    assert health["ok"] is True
    assert health["have_drone"] is False
    store.update_drone({"mode": "FLYING", "z": 4.0, "battery_percent": 90.0})
    store.update_mission({"state": "RUNNING", "waypoint_count": 4, "detail": "enroute"})
    store.update_plan({"waypoints": [{"name": "a", "x": 1.0}]})
    store.set_last_command("start", True, "started")
    snap = store.snapshot()
    snap["drone"]["z"] = 0.0
    snap["plan"]["waypoints"].append({"name": "other"})
    again = store.snapshot()
    assert again["drone"]["z"] == 4.0
    assert again["have_drone"] is True
    assert again["have_mission"] is True
    assert again["mission"]["state"] == "RUNNING"
    assert len(again["plan"]["waypoints"]) == 1
    assert again["last_command"]["success"] is True
    store.update_plan("not-a-plan")
    assert store.snapshot()["plan"]["waypoints"][0]["name"] == "a"


def test_command_queue_drains_once():
    queue = CommandQueue()
    queue.push("start")
    queue.push("rtl")
    assert queue.drain() == ["start", "rtl"]
    assert queue.drain() == []
