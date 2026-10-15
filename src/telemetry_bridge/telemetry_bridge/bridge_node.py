# Copyright 2026 Drone Waypoint Demo
"""Subscribe to demo telemetry and serve it to the ground-station page."""

import asyncio
import json
import threading
from pathlib import Path

from aiohttp import web
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from drone_msgs.msg import DroneState, MissionStatus
from drone_msgs.srv import MissionCommand
from telemetry_bridge.state_store import CommandQueue, StateStore
from telemetry_bridge.web_server import create_app


PLAN_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class TelemetryBridge(Node):
    """REST and WebSocket front end. It does not make flight decisions."""

    def __init__(self):
        super().__init__("telemetry_bridge")
        host = self.declare_parameter("web_host", "0.0.0.0").value
        port = self.declare_parameter("web_port", 8080).value
        if not isinstance(port, int) or isinstance(port, bool) or port <= 0 or port > 65535:
            raise ValueError("web_port must be an integer from 1 to 65535")
        self._store = StateStore()
        self._commands = CommandQueue()
        self._loop = None
        self._thread = None
        qos = QoSProfile(depth=10)
        self.create_subscription(DroneState, "drone/state", self._on_drone, qos)
        self.create_subscription(MissionStatus, "mission/status", self._on_mission, qos)
        self.create_subscription(String, "mission/plan", self._on_plan, PLAN_QOS)
        self._command_client = self.create_client(MissionCommand, "mission/command")
        self.create_timer(0.1, self._flush_commands)
        index = Path(get_package_share_directory("telemetry_bridge")) / "static" / "index.html"
        app = create_app(self._store, self._commands, index)
        self._thread = threading.Thread(
            target=self._serve,
            args=(app, host, port),
            name="telemetry_http",
            daemon=True,
        )
        self._thread.start()

    def _serve(self, app, host, port):
        loop = asyncio.new_event_loop()
        self._loop = loop
        asyncio.set_event_loop(loop)
        try:
            runner = web.AppRunner(app)
            loop.run_until_complete(runner.setup())
            site = web.TCPSite(runner, host, port)
            loop.run_until_complete(site.start())
        except OSError as exc:
            self.get_logger().fatal("web server failed: %s" % exc)
            return
        self.get_logger().info("Ground station at http://%s:%s" % (host, port))
        loop.run_forever()
        loop.close()

    def _on_drone(self, msg):
        self._store.update_drone(
            {
                "x": msg.x,
                "y": msg.y,
                "z": msg.z,
                "yaw": msg.yaw,
                "vx": msg.vx,
                "vy": msg.vy,
                "vz": msg.vz,
                "speed": msg.speed,
                "battery_percent": msg.battery_percent,
                "battery_rtl_percent": msg.battery_rtl_percent,
                "battery_critical_percent": msg.battery_critical_percent,
                "distance_home": msg.distance_home,
                "home_x": msg.home_x,
                "home_y": msg.home_y,
                "home_z": msg.home_z,
                "geofence_radius": msg.geofence_radius,
                "geofence_max_alt": msg.geofence_max_alt,
                "mode": msg.mode,
                "geofence_breached": msg.geofence_breached,
                "failsafe_active": msg.failsafe_active,
            }
        )

    def _on_mission(self, msg):
        self._store.update_mission(
            {
                "state": msg.state,
                "current_index": msg.current_index,
                "waypoint_count": msg.waypoint_count,
                "current_name": msg.current_name,
                "progress": msg.progress,
                "detail": msg.detail,
            }
        )

    def _on_plan(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning("mission/plan was not valid JSON")
            return
        self._store.update_plan(payload)

    def _flush_commands(self):
        if not self._command_client.service_is_ready():
            return
        for command in self._commands.drain():
            request = MissionCommand.Request()
            request.command = command
            future = self._command_client.call_async(request)
            future.add_done_callback(
                lambda done, command=command: self._on_command_done(command, done)
            )

    def _on_command_done(self, command, future):
        try:
            response = future.result()
        except Exception as exc:  # service failures must not kill the executor
            self._store.set_last_command(command, False, str(exc))
            self.get_logger().error("mission command %s failed: %s" % (command, exc))
            return
        self._store.set_last_command(command, response.success, response.message)
        log = self.get_logger().info if response.success else self.get_logger().warning
        log("mission command %s: %s" % (command, response.message))

    def destroy_node(self):
        if self._loop is not None and self._loop.is_running():
            self._loop.call_soon_threadsafe(self._loop.stop)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
