FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-pytest \
        python3-yaml \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY src /ws/src
COPY scripts /ws/scripts
COPY README.md LICENSE /ws/

RUN source /opt/ros/jazzy/setup.bash \
    && (rosdep init || true) \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src --rosdistro jazzy -y \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && colcon test --event-handlers console_direct+ \
    && colcon test-result --verbose

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "mission_planner", "demo.launch.py", "use_rviz:=false"]
