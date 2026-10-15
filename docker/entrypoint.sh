#!/usr/bin/env bash
# Copyright 2026 Drone Waypoint Demo
# Source ROS 2 Humble and this workspace, then run the container command.
set -euo pipefail
source /opt/ros/humble/setup.bash
if [[ -f /ws/install/setup.bash ]]; then
  source /ws/install/setup.bash
fi
exec "$@"
