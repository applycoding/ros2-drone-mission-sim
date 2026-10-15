# Copyright 2026 Drone Waypoint Demo
"""Same demo graph, with a battery that hits return-to-home during the flight."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    share = get_package_share_directory("drone_bringup")
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, "launch", "mission_demo.launch.py")
            ),
            launch_arguments={"profile": "failsafe"}.items(),
        ),
    ])
