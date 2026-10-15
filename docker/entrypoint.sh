#!/bin/bash
set -e
# The official image already sources the distro under /ros_entrypoint.sh.
# This wrapper also sources the workspace overlay.
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source /ws/install/setup.bash
exec "$@"
