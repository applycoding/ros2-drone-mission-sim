from setuptools import setup

package_name = "mission_planner"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/demo.launch.py"]),
        ("share/" + package_name + "/config", ["config/planner.yaml"]),
        ("share/" + package_name + "/rviz", ["rviz/mission.rviz"]),
        (
            "share/" + package_name + "/missions",
            [
                "missions/square.yaml",
                "missions/square.plan",
                "missions/square.waypoints",
            ],
        ),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="Arpan Bom",
    maintainer_email="applycoding@users.noreply.github.com",
    description=(
        "YAML and QGroundControl mission loader with an arm, takeoff, "
        "waypoint, RTL, and land state machine."
    ),
    license="MIT",
    entry_points={
        "console_scripts": [
            "mission_planner_node = mission_planner.planner_node:main",
        ],
    },
)
