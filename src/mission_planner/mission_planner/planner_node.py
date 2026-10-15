# SPDX-License-Identifier: MIT
"""ROS 2 node that tracks a mission and publishes position setpoints."""

from __future__ import annotations

import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from interfaces.action import ExecuteMission
from interfaces.msg import BatteryStatus, GeofenceStatus, MissionState, Setpoint
from interfaces.srv import LoadMission
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from mission_planner.fsm import MissionFSM, TickResult
from mission_planner.loader import load_mission
from mission_planner.pose_math import quaternion_from_yaw, yaw_from_quaternion
from mission_planner.types import Mission, MissionError, Phase, VehicleState

_TERMINAL = frozenset({Phase.COMPLETE, Phase.FAILSAFE})
_CAN_LOAD = frozenset({Phase.IDLE, Phase.COMPLETE, Phase.FAILSAFE})


def _state_code(phase: Phase) -> int:
    mapping = {
        Phase.IDLE: MissionState.IDLE,
        Phase.ARMED: MissionState.ARMED,
        Phase.TAKEOFF: MissionState.TAKEOFF,
        Phase.WAYPOINT: MissionState.WAYPOINT,
        Phase.RTL: MissionState.RTL,
        Phase.LAND: MissionState.LAND,
        Phase.COMPLETE: MissionState.COMPLETE,
        Phase.FAILSAFE: MissionState.FAILSAFE,
    }
    try:
        code = mapping[phase]
    except KeyError as exc:
        raise RuntimeError(f"unhandled mission phase {phase}") from exc
    if int(phase) != int(code):
        raise RuntimeError(f"{phase.name} value does not match MissionState.{phase.name}")
    return int(code)


def _as_bool(value: object) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in {"1", "true", "yes", "on"}:
            return True
        if lowered in {"0", "false", "no", "off"}:
            return False
    raise ValueError(f"expected a boolean parameter, got {value!r}")


class MissionPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("mission_planner_node")
        for phase in Phase:
            _state_code(phase)

        rate = float(self.declare_parameter("tick_rate_hz", 20.0).value)
        if rate <= 0.0:
            raise ValueError("tick_rate_hz must be positive")
        self._dt = 1.0 / rate
        auto_start = _as_bool(self.declare_parameter("auto_start", False).value)
        mission_file = str(self.declare_parameter("mission_file", "").value)
        odom_topic = str(self.declare_parameter("odom_topic", "/drone/odom").value)
        battery_topic = str(self.declare_parameter("battery_topic", "/drone/battery").value)
        setpoint_topic = str(self.declare_parameter("setpoint_topic", "/drone/setpoint").value)
        pose_topic = str(self.declare_parameter("pose_topic", "/drone/setpoint_pose").value)

        self._lock = threading.Lock()
        self._mission: Mission | None = None
        self._mission_path = ""
        self._fsm: MissionFSM | None = None
        self._vehicle = VehicleState(0.0, 0.0, 0.0, 0.0)
        self._battery = 100.0
        self._have_vehicle = False
        self._last: TickResult | None = None
        self._logged_phase: Phase | None = None
        self._auto_start_pending = False

        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._control_group = MutuallyExclusiveCallbackGroup()
        self._action_group = ReentrantCallbackGroup()

        self._setpoint_pub = self.create_publisher(Setpoint, setpoint_topic, 10)
        self._pose_pub = self.create_publisher(PoseStamped, pose_topic, 10)
        self._state_pub = self.create_publisher(MissionState, "/mission/state", 10)
        self._fence_pub = self.create_publisher(GeofenceStatus, "/mission/geofence", 10)
        self._path_pub = self.create_publisher(Path, "/mission/path", latched)

        self.create_subscription(
            Odometry, odom_topic, self._on_odom, 10, callback_group=self._control_group
        )
        self.create_subscription(
            BatteryStatus,
            battery_topic,
            self._on_battery,
            10,
            callback_group=self._control_group,
        )
        self.create_service(
            LoadMission, "/mission/load", self._on_load, callback_group=self._control_group
        )
        self._action = ActionServer(
            self,
            ExecuteMission,
            "/mission/execute",
            execute_callback=self._on_execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=self._action_group,
        )
        self.create_timer(self._dt, self._on_timer, callback_group=self._control_group)

        if mission_file:
            try:
                self._load_mission(mission_file)
            except MissionError as exc:
                self.get_logger().error(str(exc))
        if auto_start and self._fsm is not None:
            self._auto_start_pending = True
            self.get_logger().info("Auto-start armed; waiting for the first odometry sample")

    def _load_mission(self, path: str) -> None:
        mission = load_mission(path)
        self._mission = mission
        self._mission_path = path
        self._fsm = MissionFSM(mission)
        self._last = None
        self._logged_phase = None
        self._publish_path(mission)
        self.get_logger().info(
            f"Loaded '{mission.name}' with {len(mission.items)} items from {path}"
        )

    def _on_odom(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        vehicle = VehicleState(
            float(position.x),
            float(position.y),
            float(position.z),
            yaw_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w),
        )
        with self._lock:
            self._vehicle = vehicle
            self._have_vehicle = True

    def _on_battery(self, msg: BatteryStatus) -> None:
        with self._lock:
            self._battery = float(msg.percent)

    def _on_load(
        self, request: LoadMission.Request, response: LoadMission.Response
    ) -> LoadMission.Response:
        with self._lock:
            if self._fsm is not None and self._fsm.phase not in _CAN_LOAD:
                response.success = False
                response.message = "cannot load while a mission is active"
                return response
            try:
                self._load_mission(request.mission_path)
            except MissionError as exc:
                response.success = False
                response.message = str(exc)
                return response
            mission = self._mission
            response.success = True
            response.message = "loaded"
            response.item_count = 0 if mission is None else len(mission.items)
            response.mission_name = "" if mission is None else mission.name
            return response

    def _on_goal(self, goal_request: ExecuteMission.Goal) -> GoalResponse:
        with self._lock:
            active = self._fsm is not None and self._fsm.phase not in _CAN_LOAD
        if active:
            self.get_logger().warning("Rejected a mission goal because a mission is already active")
            return GoalResponse.REJECT
        label = goal_request.mission_path or "the loaded mission"
        self.get_logger().info(f"Accepted mission goal for {label}")
        return GoalResponse.ACCEPT

    def _on_cancel(self, _goal_handle: ServerGoalHandle) -> CancelResponse:
        self.get_logger().info("Mission cancel requested")
        return CancelResponse.ACCEPT

    def _on_execute(self, goal_handle: ServerGoalHandle) -> ExecuteMission.Result:
        result = ExecuteMission.Result()
        with self._lock:
            try:
                self._begin_goal(goal_handle.request.mission_path)
            except MissionError as exc:
                goal_handle.abort()
                result.success = False
                result.message = str(exc)
                result.final_state = "ERROR"
                result.waypoints_reached = 0
                return result

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                with self._lock:
                    if self._fsm is not None:
                        self._fsm.request_land(self._vehicle, "canceled")
                        result.waypoints_reached = self._fsm.waypoints_reached
                goal_handle.canceled()
                result.success = False
                result.message = "canceled"
                result.final_state = Phase.LAND.name
                return result

            with self._lock:
                feedback = self._make_feedback()
                phase = Phase.IDLE if self._fsm is None else self._fsm.phase
                reached = 0 if self._fsm is None else self._fsm.waypoints_reached
                reason = "" if self._fsm is None else self._fsm.failsafe_reason
            goal_handle.publish_feedback(feedback)
            if phase in _TERMINAL:
                goal_handle.succeed()
                result.success = phase is Phase.COMPLETE
                result.message = "completed" if result.success else (reason or phase.name)
                result.waypoints_reached = reached
                result.final_state = phase.name
                return result
            time.sleep(0.05)

        result.success = False
        result.message = "rclpy shutdown"
        result.final_state = "SHUTDOWN"
        return result

    def _begin_goal(self, mission_path: str) -> None:
        if self._fsm is not None and self._fsm.phase not in _CAN_LOAD:
            raise MissionError("mission is already active")
        if mission_path:
            self._load_mission(mission_path)
        elif self._fsm is None:
            raise MissionError("no mission loaded")
        elif self._fsm.phase is not Phase.IDLE:
            if not self._mission_path:
                raise MissionError("no mission loaded")
            self._load_mission(self._mission_path)
        if self._fsm is None:
            raise MissionError("no mission loaded")
        self._fsm.arm()
        self._fsm.start(self._battery)
        self._auto_start_pending = False

    def _on_timer(self) -> None:
        with self._lock:
            if self._fsm is None or not self._have_vehicle:
                return
            if self._auto_start_pending and self._fsm.phase is Phase.IDLE:
                try:
                    self._fsm.arm()
                    self._fsm.start(self._battery)
                except MissionError as exc:
                    self.get_logger().error(str(exc))
                self._auto_start_pending = False
            result = self._fsm.tick(self._vehicle, self._battery, self._dt)
            self._last = result
            phase_changed = result.phase is not self._logged_phase
            self._logged_phase = result.phase
        self._publish(result)
        if phase_changed:
            reason = result.failsafe_reason or "none"
            self.get_logger().info(
                f"{result.phase.name} waypoints="
                f"{result.waypoints_reached}/{result.waypoint_count} failsafe={reason}"
            )

    def _make_feedback(self) -> ExecuteMission.Feedback:
        feedback = ExecuteMission.Feedback()
        phase = Phase.IDLE if self._fsm is None else self._fsm.phase
        feedback.state = _state_code(phase)
        feedback.state_name = phase.name
        feedback.waypoint_index = 0 if self._fsm is None else self._fsm.waypoints_reached
        feedback.waypoint_count = 0 if self._fsm is None else self._fsm.waypoint_count
        feedback.distance_to_target = 0.0 if self._last is None else self._last.distance_to_target
        feedback.battery_percent = float(self._battery)
        feedback.failsafe_reason = "" if self._fsm is None else self._fsm.failsafe_reason
        return feedback

    def _publish(self, result: TickResult) -> None:
        stamp = self.get_clock().now().to_msg()
        setpoint = Setpoint()
        setpoint.header.stamp = stamp
        setpoint.header.frame_id = "odom"
        setpoint.x = result.setpoint.x
        setpoint.y = result.setpoint.y
        setpoint.z = result.setpoint.z
        setpoint.yaw = result.setpoint.yaw
        setpoint.max_speed_m_s = result.setpoint.max_speed_m_s
        self._setpoint_pub.publish(setpoint)

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "odom"
        pose.pose.position.x = result.setpoint.x
        pose.pose.position.y = result.setpoint.y
        pose.pose.position.z = result.setpoint.z
        qx, qy, qz, qw = quaternion_from_yaw(result.setpoint.yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self._pose_pub.publish(pose)

        state = MissionState()
        state.header.stamp = stamp
        state.header.frame_id = "odom"
        state.state = _state_code(result.phase)
        state.state_name = result.phase.name
        state.waypoint_index = result.waypoint_index
        state.waypoint_count = result.waypoint_count
        state.waypoints_reached = result.waypoints_reached
        state.failsafe_reason = result.failsafe_reason
        state.distance_to_target = result.distance_to_target
        state.geofence_inside = result.geofence_inside
        self._state_pub.publish(state)

        fence = GeofenceStatus()
        fence.header.stamp = stamp
        fence.header.frame_id = "odom"
        fence.inside = result.geofence_inside
        fence.distance_to_boundary_m = result.geofence_margin_m
        fence.breach_type = result.geofence_breach
        self._fence_pub.publish(fence)

    def _publish_path(self, mission: Mission) -> None:
        stamp = self.get_clock().now().to_msg()
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = "odom"
        for item in mission.items:
            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = "odom"
            pose.pose.position.x = item.x
            pose.pose.position.y = item.y
            pose.pose.position.z = item.z
            qx, qy, qz, qw = quaternion_from_yaw(item.yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            path.poses.append(pose)
        self._path_pub.publish(path)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MissionPlannerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
