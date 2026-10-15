# Copyright 2026 Drone Waypoint Demo

from setuptools import find_packages, setup

package_name = "mission_planner"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Drone Waypoint Demo",
    maintainer_email="demo@example.com",
    description="Load waypoint missions and publish setpoints.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mission_planner = mission_planner.planner_node:main",
        ],
    },
)
