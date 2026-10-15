# Copyright 2026 Drone Waypoint Demo
"""Launch the kinematic waypoint demo and its ground-station page."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    share = get_package_share_directory("drone_bringup")
    profile = LaunchConfiguration("profile").perform(context)
    if profile == "nominal":
        params = os.path.join(share, "config", "sim_params.yaml")
    elif profile == "failsafe":
        params = os.path.join(share, "config", "failsafe_params.yaml")
    else:
        raise RuntimeError("profile must be nominal or failsafe, got '%s'" % profile)
    mission_file = LaunchConfiguration("mission_file").perform(context)
    mission_format = LaunchConfiguration("mission_format").perform(context)
    web_port = int(LaunchConfiguration("web_port").perform(context))
    return [
        Node(
            package="sim_drone",
            executable="sim_drone_node",
            name="sim_drone",
            parameters=[params],
            output="screen",
        ),
        Node(
            package="mission_planner",
            executable="mission_planner",
            name="mission_planner",
            parameters=[{
                "mission_file": mission_file,
                "mission_format": mission_format,
                "autoload": True,
                "publish_rate": 10.0,
            }],
            output="screen",
        ),
        Node(
            package="telemetry_bridge",
            executable="telemetry_bridge",
            name="telemetry_bridge",
            parameters=[{
                "web_host": "0.0.0.0",
                "web_port": web_port,
            }],
            output="screen",
        ),
    ]


def generate_launch_description():
    share = get_package_share_directory("drone_bringup")
    default_mission = os.path.join(share, "config", "survey_square.yaml")
    return LaunchDescription([
        DeclareLaunchArgument(
            "profile",
            default_value="nominal",
            description="nominal flies the sample with a full battery; failsafe drains faster",
        ),
        DeclareLaunchArgument(
            "mission_file",
            default_value=default_mission,
            description="YAML or GeoJSON mission passed to the planner",
        ),
        DeclareLaunchArgument(
            "mission_format",
            default_value="auto",
            description="auto, yaml, or geojson",
        ),
        DeclareLaunchArgument(
            "web_port",
            default_value="8080",
            description="Ground-station HTTP and WebSocket port",
        ),
        OpaqueFunction(function=_nodes),
    ])
