# SPDX-License-Identifier: MIT

import math

import pytest

from mission_planner.pose_math import quaternion_from_yaw, yaw_from_quaternion


def test_yaw_round_trip():
    samples = (0.0, 0.4, -1.2, math.pi - 0.01, -math.pi + 0.01, 2.0)
    for yaw in samples:
        recovered = yaw_from_quaternion(*quaternion_from_yaw(yaw))
        assert recovered == pytest.approx(yaw, abs=1e-9)
