# Copyright 2026 Drone Waypoint Demo

from setuptools import find_packages, setup

package_name = "telemetry_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/static", ["static/index.html"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Drone Waypoint Demo",
    maintainer_email="demo@example.com",
    description="Serve simulator telemetry over HTTP and WebSocket.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "telemetry_bridge = telemetry_bridge.bridge_node:main",
        ],
    },
)
