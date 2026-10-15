# Demo image. It runs the kinematic mission on a desktop or CI runner.
# It does not contain PX4, Gazebo, or flight-controller firmware.
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONDONTWRITEBYTECODE=1

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-aiohttp \
        python3-pytest \
        python3-yaml \
        ros-humble-ament-cmake-gtest \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY . /ws
RUN chmod +x /ws/docker/entrypoint.sh \
    && . /opt/ros/humble/setup.sh \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

EXPOSE 8080
ENTRYPOINT ["/ws/docker/entrypoint.sh"]
CMD ["ros2", "launch", "drone_bringup", "mission_demo.launch.py"]
