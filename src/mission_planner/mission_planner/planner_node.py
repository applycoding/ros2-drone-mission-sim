# Copyright 2026 Drone Waypoint Demo
"""ROS 2 wrapper that streams the sequencer setpoint to the simulator."""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String

from drone_msgs.msg import DroneState, MissionStatus, Waypoint as WaypointMsg
from drone_msgs.srv import LoadMission, MissionCommand
from mission_planner.loaders import load_mission
from mission_planner.models import mission_to_dict
from mission_planner.sequencer import MissionState, Sequencer


PLAN_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
_RTL_STATES = {
    MissionState.COMPLETE,
    MissionState.ABORTED,
    MissionState.RTL,
    MissionState.FAILSAFE,
}


class MissionPlanner(Node):
    """Load a mission file and publish the active setpoint."""

    def __init__(self):
        super().__init__("mission_planner")
        self._mission_file = self.declare_parameter("mission_file", "").value
        self._mission_format = self.declare_parameter("mission_format", "auto").value
        autoload = self.declare_parameter("autoload", True).value
        rate = self.declare_parameter("publish_rate", 10.0).value
        if not isinstance(rate, (int, float)) or rate <= 0.0:
            raise ValueError("publish_rate must be positive")
        self._sequencer = Sequencer()
        self._drone = None
        self._prev_stamp = None
        self._pending = None
        self._pending_rtl = False
        qos = QoSProfile(depth=10)
        self._setpoint_pub = self.create_publisher(WaypointMsg, "drone/setpoint", qos)
        self._status_pub = self.create_publisher(MissionStatus, "mission/status", qos)
        self._plan_pub = self.create_publisher(String, "mission/plan", PLAN_QOS)
        self.create_subscription(DroneState, "drone/state", self._on_state, qos)
        self.create_service(LoadMission, "mission/load", self._handle_load)
        self.create_service(MissionCommand, "mission/command", self._handle_command)
        self.create_timer(1.0 / float(rate), self._on_timer)
        if autoload and self._mission_file:
            try:
                ok, message = self._load_from_path(self._mission_file, self._mission_format)
                if ok:
                    self.get_logger().info(message)
                else:
                    self.get_logger().error(message)
            except (OSError, ValueError) as exc:
                self.get_logger().error("failed to autoload mission: %s" % exc)
        elif autoload:
            self.get_logger().warning("autoload is set but mission_file is empty")
        self._publish_status()
        self._publish_plan()

    def _on_timer(self):
        self._publish_setpoint()
        self._publish_status()
        self._publish_plan()

    def _on_state(self, msg):
        dt = self._dt_from(msg)
        self._drone = msg
        if msg.failsafe_active and self._sequencer.state == MissionState.RUNNING:
            self._sequencer.notify_vehicle_failsafe()
        self._pending = self._sequencer.update(msg.x, msg.y, msg.z, dt, msg.mode)
        self._pending_rtl = self._sequencer.state in _RTL_STATES
        self._publish_setpoint()
        self._publish_status()

    def _dt_from(self, msg):
        stamp = Time.from_msg(msg.header.stamp)
        if self._prev_stamp is None:
            self._prev_stamp = stamp
            return 0.0
        dt = (stamp - self._prev_stamp).nanoseconds / 1e9
        self._prev_stamp = stamp
        if not math.isfinite(dt) or dt < 0.0:
            return 0.0
        return min(dt, 1.0)

    def _handle_load(self, request, response):
        path = request.path.strip() or self._mission_file
        declared = request.format.strip() or "auto"
        try:
            ok, message = self._load_from_path(path, declared)
        except (OSError, ValueError) as exc:
            ok, message = False, str(exc)
        response.success = ok
        response.message = message
        response.waypoint_count = self._sequencer.waypoint_count()
        self._publish_status()
        self._publish_plan()
        return response

    def _handle_command(self, request, response):
        ok, message = self._apply_command(request.command)
        response.success = ok
        response.message = message
        response.state = self._sequencer.state
        self._publish_status()
        return response

    def _apply_command(self, command):
        normalized = command.strip().lower()
        if normalized == "start":
            return self._sequencer.start()
        if normalized == "abort":
            return self._sequencer.abort()
        if normalized == "rtl":
            return self._sequencer.command_rtl()
        if normalized == "reload":
            return self._reload()
        return False, "unknown command '%s'" % command

    def _reload(self):
        if not self._mission_file:
            return False, "no mission file configured"
        try:
            return self._load_from_path(self._mission_file, self._mission_format)
        except (OSError, ValueError) as exc:
            return False, str(exc)

    def _load_from_path(self, path, declared_format):
        if self._sequencer.state == MissionState.RUNNING:
            return False, "refuse to replace a running mission"
        mission = load_mission(path, declared_format)
        self._sequencer.load(mission)
        self._mission_file = path
        self._mission_format = declared_format
        self._pending = None
        self._pending_rtl = False
        self._publish_plan()
        return True, "loaded %d waypoints from %s" % (len(mission.waypoints), path)

    def _publish_setpoint(self):
        if self._pending is None:
            return
        msg = WaypointMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.name = self._pending.name
        msg.x = float(self._pending.x)
        msg.y = float(self._pending.y)
        msg.z = float(self._pending.z)
        msg.yaw = float(self._pending.yaw)
        msg.hold_sec = float(self._pending.hold_sec)
        msg.acceptance_radius = float(self._pending.acceptance_radius)
        msg.return_to_home = self._pending_rtl
        self._setpoint_pub.publish(msg)

    def _publish_status(self):
        msg = MissionStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.state = self._sequencer.state
        msg.current_index = int(self._sequencer.current_index())
        msg.waypoint_count = int(self._sequencer.waypoint_count())
        msg.current_name = self._sequencer.current_name()
        msg.progress = float(self._sequencer.progress())
        msg.detail = self._sequencer.detail
        self._status_pub.publish(msg)

    def _publish_plan(self):
        mission = self._sequencer.mission
        payload = mission_to_dict(mission) if mission is not None else {
            "frame": "",
            "source": "",
            "home": {"x": 0.0, "y": 0.0, "z": 0.0},
            "waypoints": [],
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._plan_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
