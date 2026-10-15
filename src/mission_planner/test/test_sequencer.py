# Copyright 2026 Drone Waypoint Demo
"""Sequencer behavior without a ROS graph."""

import pytest

from mission_planner.models import Mission, Waypoint
from mission_planner.sequencer import MissionState, Sequencer


def _mission(*waypoints, home_x=0.0, home_y=0.0, home_z=0.0):
    return Mission(
        frame="local_enu",
        waypoints=tuple(waypoints),
        home_x=home_x,
        home_y=home_y,
        home_z=home_z,
    )


def _wp(name, x, y, z, hold=0.0, radius=1.0):
    return Waypoint(name, x, y, z, hold_sec=hold, acceptance_radius=radius)


def test_start_requires_a_mission_and_rejects_a_second_start():
    sequencer = Sequencer()
    ok, _message = sequencer.start()
    assert not ok
    sequencer.load(_mission(_wp("a", 0, 0, 1, hold=1.0)))
    assert sequencer.state == MissionState.LOADED
    assert sequencer.start()[0]
    assert not sequencer.start()[0]


def test_does_not_advance_while_far_or_while_the_vehicle_is_not_flying():
    sequencer = Sequencer(_mission(_wp("a", 0, 0, 5, hold=1.0)))
    sequencer.start()
    stayed = sequencer.update(0, 0, 0, 5.0, "FLYING")
    assert stayed.name == "a"
    assert sequencer.state == MissionState.RUNNING
    waiting = sequencer.update(0, 0, 5, 5.0, "RTL")
    assert waiting.name == "a"
    assert sequencer.hold_elapsed == 0.0
    assert "waiting" in sequencer.detail


def test_hold_then_complete_and_zero_hold_advances_one_per_update():
    sequencer = Sequencer(_mission(_wp("a", 0, 0, 1, hold=2.0)))
    sequencer.start()
    sequencer.update(0, 0, 1, 0.0, "FLYING")
    assert sequencer.phase == "HOLD"
    assert sequencer.state == MissionState.RUNNING
    # Hold time is applied in steps of at most one second.
    sequencer.update(0, 0, 1, 1.0, "FLYING")
    assert sequencer.state == MissionState.RUNNING
    done = sequencer.update(0, 0, 1, 1.0, "FLYING")
    assert sequencer.state == MissionState.COMPLETE
    assert done.name == "home"
    assert done.z == 0.0
    assert sequencer.progress() == 1.0

    pair = Sequencer(
        _mission(
            _wp("a", 0, 0, 1, hold=0.0),
            _wp("b", 0, 0, 1, hold=0.0),
        )
    )
    pair.start()
    first = pair.update(0, 0, 1, 0.1, "FLYING")
    assert first.name == "b"
    assert pair.state == MissionState.RUNNING
    second = pair.update(0, 0, 1, 0.1, "FLYING")
    assert pair.state == MissionState.COMPLETE
    assert second.name == "home"


def test_leaving_the_acceptance_radius_resets_the_hold():
    sequencer = Sequencer(_mission(_wp("a", 0, 0, 0, hold=2.0, radius=1.0)))
    sequencer.start()
    sequencer.update(0, 0, 0, 0.0, "FLYING")
    sequencer.update(0, 0, 0, 1.0, "FLYING")
    assert sequencer.hold_elapsed == pytest.approx(1.0)
    sequencer.update(5, 0, 0, 3.0, "FLYING")
    assert sequencer.phase == "NAVIGATE"
    assert sequencer.hold_elapsed == 0.0
    sequencer.update(0, 0, 0, 0.0, "FLYING")
    sequencer.update(0, 0, 0, 1.0, "FLYING")
    assert sequencer.state == MissionState.RUNNING
    assert sequencer.index == 0


def test_abort_and_rtl_stream_home_and_running_load_is_refused():
    mission = _mission(_wp("a", 2, 0, 3, hold=1.0), home_x=1.0)
    sequencer = Sequencer(mission)
    sequencer.start()
    with pytest.raises(ValueError):
        sequencer.load(mission)
    assert sequencer.abort()[0]
    home = sequencer.update(2, 0, 3, 0.0, "FLYING")
    assert home.name == "home"
    assert home.x == 1.0
    assert sequencer.command_rtl()[0]
    assert sequencer.update(0, 0, 0, 0.0, "RTL").name == "home"


def test_vehicle_failsafe_interrupts_a_running_mission():
    sequencer = Sequencer(_mission(_wp("a", 0, 0, 1)))
    sequencer.start()
    sequencer.notify_vehicle_failsafe()
    assert sequencer.state == MissionState.FAILSAFE
    home = sequencer.update(0, 0, 1, 0.0, "FAILSAFE")
    assert home.name == "home"
    sequencer.notify_vehicle_failsafe()
    assert sequencer.state == MissionState.FAILSAFE


def test_loaded_update_publishes_nothing():
    sequencer = Sequencer(_mission(_wp("a", 0, 0, 1)))
    assert sequencer.update(0, 0, 0, 0.1, "DISARMED") is None
    assert sequencer.progress() == 0.0
