from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    planner_share = FindPackageShare("mission_planner")
    sim_share = FindPackageShare("drone_sim")
    default_mission = PathJoinSubstitution([planner_share, "missions", "square.yaml"])
    planner_config = PathJoinSubstitution([planner_share, "config", "planner.yaml"])
    sim_config = PathJoinSubstitution([sim_share, "config", "sim.yaml"])
    rviz_config = PathJoinSubstitution([planner_share, "rviz", "mission.rviz"])

    use_rviz = LaunchConfiguration("use_rviz")
    mission_file = LaunchConfiguration("mission_file")
    auto_start = LaunchConfiguration("auto_start")
    battery_percent = LaunchConfiguration("battery_percent")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="Start RViz when ros-jazzy-rviz2 is installed",
            ),
            DeclareLaunchArgument(
                "mission_file",
                default_value=default_mission,
                description="YAML, QGC .plan, or QGC WPL 110 mission",
            ),
            DeclareLaunchArgument(
                "auto_start",
                default_value="true",
                description="Arm and start after the first odometry sample",
            ),
            DeclareLaunchArgument(
                "battery_percent",
                default_value="100.0",
                description="Simulator battery at startup. 22 or below trips the RTL failsafe",
            ),
            Node(
                package="drone_sim",
                executable="drone_sim_node",
                name="drone_sim_node",
                output="screen",
                parameters=[
                    sim_config,
                    {
                        "battery_percent": ParameterValue(battery_percent, value_type=float),
                    },
                ],
            ),
            Node(
                package="mission_planner",
                executable="mission_planner_node",
                name="mission_planner_node",
                output="screen",
                parameters=[
                    planner_config,
                    {
                        "mission_file": mission_file,
                        "auto_start": ParameterValue(auto_start, value_type=bool),
                    },
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
                output="screen",
            ),
        ]
    )
