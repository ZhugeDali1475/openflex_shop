#!/usr/bin/env python3

import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from controller_manager_msgs.srv import ListControllers, SwitchController
from control_msgs.action import GripperCommand
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from interface.srv import SaveMaps
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Odometry, Path as NavPath
from openarmx_task_msgs.action import (
    DetectObject,
    ExecutePick,
    MoveToShelfView,
    PickObject,
    PlanPick,
    ReadOrder,
    RetreatFromStation,
    ShelfFineAlign,
    ShelfTraverse,
)
from openarmx_integrated_moveit_msgs.action import CartesianApproach, PlanPregrasp
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import JointState, LaserScan, PointCloud2
from std_msgs.msg import Bool, Empty, Float64MultiArray, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from PyQt5.QtCore import (
    QProcess,
    QRectF,
    QSettings,
    QSize,
    Qt,
    QTimer,
    QUrl,
    pyqtSignal,
)
from PyQt5.QtGui import QColor, QDesktopServices, QImage, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSpinBox,
    QSplitter,
    QStyle,
    QTabWidget,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from .model import (
    RECORD_TOPICS,
    command_text,
    discover_map_names,
    nearest_path_error,
    normalize_angle,
    quaternion_from_yaw,
    resolve_map_paths,
    yaw_from_quaternion,
)
from .map_editor import MapEditorDialog
from .goal_safety import CostmapSnapshot, GoalSafetyGeometry, GoalSafetyMap
from .map_editor_core import parse_map_yaml, read_pgm


APP_TITLE = "OpenFlex Mapping and Navigation Console"
DEFAULT_MAP_ROOT = "/home/openflex/openflex_shop/openflex_maps"
DEFAULT_GOAL_SAFETY_GEOMETRY = GoalSafetyGeometry(
    half_length_m=0.46,
    half_width_m=0.45,
    clearance_m=0.05,
)
WHEEL_NAMES = ("FL", "FR", "BL", "BR")


@dataclass
class WheelTelemetry:
    requested_angle: float = 0.0
    commanded_angle: float = 0.0
    actual_angle: float = 0.0
    angle_error: float = 0.0
    target_speed: float = 0.0
    commanded_speed: float = 0.0
    actual_speed: float = 0.0
    steering_scale: float = 1.0


@dataclass
class Telemetry:
    map_x: float = 0.0
    map_y: float = 0.0
    map_yaw: float = 0.0
    has_map_pose: bool = False
    map_to_odom_x: float = 0.0
    map_to_odom_y: float = 0.0
    map_to_odom_yaw: float = 0.0
    icp_step_xy: float = 0.0
    icp_step_yaw: float = 0.0
    odom_x: float = 0.0
    odom_y: float = 0.0
    odom_yaw: float = 0.0
    base_linear_speed: float = 0.0
    base_angular_speed: float = 0.0
    wheel_x: float = 0.0
    wheel_y: float = 0.0
    wheel_yaw: float = 0.0
    lio_x: float = 0.0
    lio_y: float = 0.0
    lio_yaw: float = 0.0
    cmd_nav: tuple[float, float, float] = (0.0, 0.0, 0.0)
    cmd_raw: tuple[float, float, float] = (0.0, 0.0, 0.0)
    cmd_safe_in: tuple[float, float, float] = (0.0, 0.0, 0.0)
    cmd_safe: tuple[float, float, float] = (0.0, 0.0, 0.0)
    path: list[tuple[float, float, float]] = field(default_factory=list)
    trail: deque = field(default_factory=lambda: deque(maxlen=3000))
    wheels: list[WheelTelemetry] = field(
        default_factory=lambda: [WheelTelemetry() for _ in range(4)]
    )
    global_drive_scale: float = 1.0
    worst_angle_error: float = 0.0
    last_seen: dict[str, float] = field(default_factory=dict)
    node_names: set[str] = field(default_factory=set)
    action_servers: set[str] = field(default_factory=set)
    localization_hold_count: int = 0
    localization_ok: bool = False
    localization_state_seen: bool = False
    localization_true_since: Optional[float] = None
    global_costmap: Optional[CostmapSnapshot] = None
    action_status: str = "idle"
    action_remaining_distance: float = 0.0
    goal_x: float = 0.0
    goal_y: float = 0.0
    goal_yaw: float = 0.0
    goal_valid: bool = False
    shelf_align_status: str = "idle"
    shelf_align_phase: str = ""
    shelf_clearance: float = 0.0
    shelf_yaw_error: float = 0.0
    shelf_stable_samples: int = 0
    shelf_align_message: str = ""
    shelf_traverse_status: str = "idle"
    shelf_traverse_phase: str = ""
    shelf_traverse_remaining: float = 0.0
    shelf_traverse_clearance: float = 0.0
    shelf_traverse_yaw_error: float = 0.0
    shelf_traverse_message: str = ""
    shelf_view_status: str = "idle"
    shelf_view_phase: str = ""
    shelf_view_level: int = -1
    shelf_view_lift_position: float = 0.0
    shelf_view_message: str = ""
    right_gripper_status: str = "idle"
    right_gripper_position: float = 0.0
    right_gripper_reached_goal: bool = False
    right_gripper_stalled: bool = False
    right_gripper_message: str = ""
    pick_test_status: str = "idle"
    pick_test_phase: str = ""
    pick_test_progress: float = 0.0
    pick_test_attached: bool = False
    pick_test_message: str = ""
    retreat_test_status: str = "idle"
    retreat_test_phase: str = ""
    retreat_test_distance: float = 0.0
    retreat_test_message: str = ""
    joint_positions: dict[str, float] = field(default_factory=dict)
    joint_velocities: dict[str, float] = field(default_factory=dict)
    joint_efforts: dict[str, float] = field(default_factory=dict)
    lift_homing_state: str = "IDLE"
    lift_homing_detail: str = ""
    lift_homing_progress_seen: bool = False
    freedrive_home_state: str = "IDLE"
    freedrive_home_detail: str = ""


class NavigationRosNode(Node):
    CANCEL_RESPONSE_TIMEOUT_SEC = 2.0

    def __init__(self):
        super().__init__("openflex_navigation_gui")
        self.telemetry = Telemetry()
        self.tf_buffer = Buffer(cache_time=Duration(seconds=15.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.teleop_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.start_clearance_pub = self.create_publisher(Twist, "/cmd_vel_nav", 10)
        self.mapping_teleop_pub = self.create_publisher(Twist, "/cmd_vel_safe", 10)
        # Direct joint publishers must only exist while the operator explicitly
        # owns manual control. MoveIt rejects trajectories when another node is
        # publishing to either arm command topic.
        self.left_joint_command_pub = None
        self.right_joint_command_pub = None
        self.head_joint_command_pub = None
        self.lift_step_command_pub = None
        self._manual_joint_control_requested = False
        self._manual_joint_control_include_arms = True
        self._manual_joint_control_state = "released"
        self._manual_joint_control_detail = ""
        self._manual_joint_control_future = None
        self._manual_joint_control_operation = None
        self._lift_controller_list_client = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self._lift_controller_switch_client = self.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )
        self._arm_parameter_get_clients = {
            side: self.create_client(
                GetParameters,
                f"/openarmx_{side}_hardware_params/get_parameters",
            )
            for side in ("left", "right")
        }
        self._arm_parameter_set_clients = {
            side: self.create_client(
                SetParameters,
                f"/openarmx_{side}_hardware_params/set_parameters",
            )
            for side in ("left", "right")
        }
        self._last_lift_step_target = None
        self.lift_start_homing_client = self.create_client(
            Trigger, "/lift_slide_driver/start_homing"
        )
        self.freedrive_home_client = self.create_client(
            Trigger, "/openarmx_freedrive/home_arms"
        )
        self.freedrive_cancel_home_client = self.create_client(
            Trigger, "/openarmx_freedrive/cancel_home"
        )
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "/initialpose",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        # Diagnostic mirror only. Publishing to /goal_pose can make the RViz
        # Nav2 panel submit a second action goal.
        self.goal_pose_pub = self.create_publisher(
            PoseStamped, "/openflex_navigation_gui/goal_pose", 10)
        self.save_maps_client = self.create_client(SaveMaps, "/pgo/save_maps")
        self.nav_action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.shelf_align_action_client = ActionClient(
            self, ShelfFineAlign, "/shelf_fine_align"
        )
        self.shelf_traverse_action_client = ActionClient(
            self, ShelfTraverse, "/return_to_shelf_anchor"
        )
        self.shelf_view_action_client = ActionClient(
            self, MoveToShelfView, "/competition/move_to_shelf_view"
        )
        self.read_order_action_client = ActionClient(
            self, ReadOrder, "/read_delivery_order"
        )
        self.right_gripper_action_client = ActionClient(
            self, GripperCommand, "/right_gripper_controller/gripper_cmd"
        )
        self.mock_pick_action_client = ActionClient(
            self, PickObject, "/competition/mock_pick_object"
        )
        self.detect_object_action_client = ActionClient(
            self, DetectObject, "/detect_object"
        )
        self.pick_object_action_client = ActionClient(self, PickObject, "/pick_object")
        self.plan_pick_action_client = ActionClient(self, PlanPick, "/plan_pick")
        self.execute_pick_action_client = ActionClient(
            self, ExecutePick, "/execute_pick"
        )
        self.plan_pregrasp_action_client = ActionClient(
            self, PlanPregrasp, "/openflex/plan_pregrasp"
        )
        self.cartesian_approach_action_client = ActionClient(
            self, CartesianApproach, "/openflex/cartesian_approach"
        )
        self.grasp_debug_goal_handle = None
        self.grasp_debug_pick_goal_handle = None
        self.grasp_debug_execute_goal_handle = None
        self.grasp_debug_result = None
        self.grasp_debug_plan_result = None
        self.grasp_debug_plan_owner = ""
        self._grasp_plan_sequence = 0
        self._active_grasp_plan_sequence = 0
        self.grasp_debug_status = "idle"
        self.grasp_debug_message = ""
        self.retreat_action_client = ActionClient(
            self, RetreatFromStation, "/retreat_from_station"
        )
        self.goal_handle = None
        self._goal_sequence = 0
        self._active_goal_sequence = 0
        self._cancel_requested_sequence: Optional[int] = None
        self._cancel_future = None
        self._cancel_deadline: Optional[float] = None
        self._cancel_watchdog = self.create_timer(0.1, self._check_cancel_timeout)
        self.shelf_align_goal_handle = None
        self._shelf_align_sequence = 0
        self._active_shelf_align_sequence = 0
        self.shelf_traverse_goal_handle = None
        self._shelf_traverse_sequence = 0
        self._active_shelf_traverse_sequence = 0
        self.shelf_view_goal_handle = None
        self._shelf_view_sequence = 0
        self._active_shelf_view_sequence = 0
        self.right_gripper_goal_handle = None
        self._right_gripper_sequence = 0
        self._active_right_gripper_sequence = 0
        self.mock_pick_goal_handle = None
        self._pick_test_sequence = 0
        self._active_pick_test_sequence = 0
        self.retreat_goal_handle = None
        self._retreat_test_sequence = 0
        self._active_retreat_test_sequence = 0
        self.shopping_ticket_goal_handle = None
        self._shopping_ticket_sequence = 0
        self._active_shopping_ticket_sequence = 0
        self._shopping_ticket_cancel_requested_sequence: Optional[int] = None
        self.shopping_ticket_status = "idle"
        self.shopping_ticket_phase = ""
        self.shopping_ticket_message = ""
        self.shopping_ticket_lines = ()

        self.create_subscription(Odometry, "/odom_safe", self._odom_safe_cb, 20)
        self.create_subscription(Odometry, "/odom", self._wheel_odom_cb, 20)
        self.create_subscription(
            Odometry, "/fastlio2/lio_odom", self._lio_odom_cb, qos_profile_sensor_data
        )
        self.create_subscription(Twist, "/cmd_vel_nav", self._cmd_nav_cb, 20)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_raw_cb, 20)
        self.create_subscription(Twist, "/cmd_vel_safe_in", self._cmd_safe_in_cb, 20)
        self.create_subscription(Twist, "/cmd_vel_safe", self._cmd_safe_cb, 20)
        self.create_subscription(JointState, "/joint_states", self._joint_state_cb, 20)
        self.create_subscription(
            String,
            "/lift_slide_driver/homing_state",
            self._lift_homing_state_cb,
            10,
        )
        self.create_subscription(
            String,
            "/openarmx_freedrive/home_status",
            self._freedrive_home_state_cb,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "/swerve_drive_controller/debug_states",
            self._debug_cb,
            20,
        )
        self.create_subscription(LaserScan, "/scan", self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2,
            "/livox/body_cloud_safety",
            self._safety_cloud_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(Empty, "/localization_hold", self._hold_cb, 10)
        localization_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Bool, "/localization_ok", self._localization_state_cb, localization_qos
        )

        costmap_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            Costmap,
            "/global_costmap/costmap_raw",
            self._global_costmap_cb,
            costmap_qos,
        )

        path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            # Nav2 planner_server publishes /plan with volatile durability.
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(NavPath, "/plan", self._path_cb, path_qos)

    def _seen(self, topic: str) -> None:
        self.telemetry.last_seen[topic] = time.monotonic()

    @staticmethod
    def _twist_tuple(msg: Twist) -> tuple[float, float, float]:
        return msg.linear.x, msg.linear.y, msg.angular.z

    def _odom_safe_cb(self, msg: Odometry) -> None:
        state = self.telemetry
        state.odom_x = msg.pose.pose.position.x
        state.odom_y = msg.pose.pose.position.y
        state.odom_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        linear = msg.twist.twist.linear
        state.base_linear_speed = math.sqrt(
            linear.x * linear.x + linear.y * linear.y + linear.z * linear.z
        )
        state.base_angular_speed = abs(msg.twist.twist.angular.z)
        self._seen("/odom_safe")

    def _wheel_odom_cb(self, msg: Odometry) -> None:
        state = self.telemetry
        state.wheel_x = msg.pose.pose.position.x
        state.wheel_y = msg.pose.pose.position.y
        state.wheel_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self._seen("/odom")

    def _lio_odom_cb(self, msg: Odometry) -> None:
        state = self.telemetry
        state.lio_x = msg.pose.pose.position.x
        state.lio_y = msg.pose.pose.position.y
        state.lio_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self._seen("/fastlio2/lio_odom")

    def _cmd_nav_cb(self, msg: Twist) -> None:
        self.telemetry.cmd_nav = self._twist_tuple(msg)
        self._seen("/cmd_vel_nav")

    def _cmd_raw_cb(self, msg: Twist) -> None:
        self.telemetry.cmd_raw = self._twist_tuple(msg)
        self._seen("/cmd_vel")

    def _cmd_safe_in_cb(self, msg: Twist) -> None:
        self.telemetry.cmd_safe_in = self._twist_tuple(msg)
        self._seen("/cmd_vel_safe_in")

    def _cmd_safe_cb(self, msg: Twist) -> None:
        self.telemetry.cmd_safe = self._twist_tuple(msg)
        self._seen("/cmd_vel_safe")

    def _joint_state_cb(self, msg: JointState) -> None:
        state = self.telemetry
        for index, name in enumerate(msg.name):
            if index < len(msg.position):
                state.joint_positions[name] = float(msg.position[index])
            if index < len(msg.velocity):
                state.joint_velocities[name] = float(msg.velocity[index])
            if index < len(msg.effort):
                state.joint_efforts[name] = float(msg.effort[index])
        names = (
            "fl_steering_joint",
            "fr_steering_joint",
            "bl_steering_joint",
            "br_steering_joint",
        )
        for index, name in enumerate(names):
            if name in msg.name:
                position_index = msg.name.index(name)
                if position_index < len(msg.position):
                    self.telemetry.wheels[index].actual_angle = msg.position[position_index]
        self._seen("/joint_states")

    def request_shopping_ticket(
        self,
        station_id: str,
        *,
        expected_distinct_skus: int = 2,
        min_confidence: float = 0.8,
    ) -> bool:
        if (
            self._active_shopping_ticket_sequence > 0
            or not self.read_order_action_client.server_is_ready()
        ):
            return False
        goal = ReadOrder.Goal()
        goal.station_id = str(station_id).strip()
        goal.expected_distinct_skus = int(expected_distinct_skus)
        goal.min_confidence = float(min_confidence)
        # Shopping-ticket Qwen inference has no arbitrary local deadline.
        goal.timeout_sec = 0.0
        self._shopping_ticket_sequence += 1
        sequence = self._shopping_ticket_sequence
        self._active_shopping_ticket_sequence = sequence
        self._shopping_ticket_cancel_requested_sequence = None
        self.shopping_ticket_goal_handle = None
        self.shopping_ticket_status = "sending"
        self.shopping_ticket_phase = ""
        self.shopping_ticket_message = ""
        self.shopping_ticket_lines = ()
        future = self.read_order_action_client.send_goal_async(
            goal,
            feedback_callback=lambda feedback: self._shopping_ticket_feedback(
                feedback, sequence
            ),
        )
        future.add_done_callback(
            lambda response: self._shopping_ticket_response(response, sequence)
        )
        return True

    def _shopping_ticket_feedback(self, feedback, sequence: int) -> None:
        if sequence != self._active_shopping_ticket_sequence:
            return
        if self.shopping_ticket_status != "canceling":
            self.shopping_ticket_status = "running"
        self.shopping_ticket_phase = str(feedback.feedback.phase)

    def _shopping_ticket_response(self, future, sequence: int) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            if sequence == self._active_shopping_ticket_sequence:
                self.shopping_ticket_status = "error"
                self.shopping_ticket_message = str(exc)
                self._clear_shopping_ticket_state(sequence)
            return
        if sequence != self._active_shopping_ticket_sequence:
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        if not goal_handle.accepted:
            self.shopping_ticket_status = "rejected"
            self.shopping_ticket_message = "ReadOrder goal was rejected"
            self._clear_shopping_ticket_state(sequence)
            return
        self.shopping_ticket_goal_handle = goal_handle
        if self._shopping_ticket_cancel_requested_sequence == sequence:
            self._request_shopping_ticket_cancel(goal_handle, sequence)
            return
        self.shopping_ticket_status = "running"
        result = goal_handle.get_result_async()
        result.add_done_callback(
            lambda completed: self._shopping_ticket_result(completed, sequence)
        )

    def _shopping_ticket_result(self, future, sequence: int) -> None:
        if sequence != self._active_shopping_ticket_sequence:
            return
        try:
            wrapped = future.result()
            payload = wrapped.result
            if (
                wrapped.status == GoalStatus.STATUS_SUCCEEDED
                and bool(payload.success)
            ):
                self.shopping_ticket_status = "succeeded"
                self.shopping_ticket_lines = tuple(payload.lines)
            elif wrapped.status == GoalStatus.STATUS_CANCELED:
                self.shopping_ticket_status = "canceled"
                self.shopping_ticket_lines = ()
            else:
                self.shopping_ticket_status = "failed"
                self.shopping_ticket_lines = ()
            self.shopping_ticket_message = str(payload.message)
        except Exception as exc:
            self.shopping_ticket_status = "error"
            self.shopping_ticket_lines = ()
            self.shopping_ticket_message = str(exc)
        self._clear_shopping_ticket_state(sequence)

    def cancel_shopping_ticket(self) -> None:
        sequence = self._active_shopping_ticket_sequence
        if sequence <= 0:
            return
        self._shopping_ticket_cancel_requested_sequence = sequence
        self.shopping_ticket_status = "canceling"
        handle = self.shopping_ticket_goal_handle
        if handle is not None:
            self._request_shopping_ticket_cancel(handle, sequence)

    def _request_shopping_ticket_cancel(self, goal_handle, sequence: int) -> None:
        try:
            future = goal_handle.cancel_goal_async()
            future.add_done_callback(
                lambda completed: self._shopping_ticket_cancel_response(
                    completed, sequence
                )
            )
        except Exception as exc:
            if sequence == self._active_shopping_ticket_sequence:
                self.shopping_ticket_status = "cancel_error"
                self.shopping_ticket_message = str(exc)
                self._clear_shopping_ticket_state(sequence)

    def _shopping_ticket_cancel_response(self, future, sequence: int) -> None:
        if sequence != self._active_shopping_ticket_sequence:
            return
        try:
            accepted = bool(future.result().goals_canceling)
        except Exception as exc:
            self.shopping_ticket_status = "cancel_error"
            self.shopping_ticket_message = str(exc)
        else:
            self.shopping_ticket_status = (
                "canceled" if accepted else "cancel_rejected"
            )
            if not accepted:
                self.shopping_ticket_message = "ReadOrder cancellation was rejected"
        self.shopping_ticket_lines = ()
        self._clear_shopping_ticket_state(sequence)

    def _clear_shopping_ticket_state(self, sequence: int) -> None:
        if sequence != self._active_shopping_ticket_sequence:
            return
        self.shopping_ticket_goal_handle = None
        self._active_shopping_ticket_sequence = 0
        self._shopping_ticket_cancel_requested_sequence = None

    def _lift_homing_state_cb(self, msg: String) -> None:
        state, separator, detail = str(msg.data).partition(":")
        self.telemetry.lift_homing_state = state.strip() or "IDLE"
        self.telemetry.lift_homing_detail = detail.strip() if separator else ""
        if self.telemetry.lift_homing_state == "IN_PROGRESS":
            self.telemetry.lift_homing_progress_seen = True
        self._seen("/lift_slide_driver/homing_state")

    def _freedrive_home_state_cb(self, msg: String) -> None:
        state, separator, detail = str(msg.data).partition(":")
        self.telemetry.freedrive_home_state = state.strip() or "IDLE"
        self.telemetry.freedrive_home_detail = detail.strip() if separator else ""
        self._seen("/openarmx_freedrive/home_status")

    @staticmethod
    def _position_command(values) -> Float64MultiArray:
        message = Float64MultiArray()
        message.data = [float(value) for value in values]
        return message

    def publish_fullbody_joint_positions(
        self,
        left_arm,
        left_gripper: float,
        right_arm,
        right_gripper: float,
        lift: float,
        head,
    ) -> None:
        """Publish one coherent position target to the existing full-body controllers."""
        if len(left_arm) != 7 or len(right_arm) != 7 or len(head) != 2:
            raise ValueError("invalid full-body joint target dimensions")
        if not self.manual_joint_control_ready(include_arms=True):
            raise RuntimeError("manual joint control is not acquired")
        self.left_joint_command_pub.publish(
            self._position_command([*left_arm, left_gripper])
        )
        self.right_joint_command_pub.publish(
            self._position_command([*right_arm, right_gripper])
        )
        self.publish_auxiliary_joint_positions(lift, head)

    def publish_auxiliary_joint_positions(self, lift: float, head) -> None:
        """Publish lift and head targets without acquiring either arm topic."""
        if len(head) != 2:
            raise ValueError("invalid head joint target dimensions")
        if not self.manual_joint_control_ready(include_arms=False):
            raise RuntimeError("manual auxiliary joint control is not acquired")
        current_lift = self.telemetry.joint_positions.get("lift_joint")
        if (
            current_lift is not None
            and (
                self._last_lift_step_target is None
                or abs(float(lift) - self._last_lift_step_target) >= 0.0005
            )
        ):
            lift_delta = float(lift) - float(current_lift)
            if abs(lift_delta) >= 0.0005:
                self.lift_step_command_pub.publish(
                    self._position_command([lift_delta, 0.05])
                )
            self._last_lift_step_target = float(lift)
        # Controller order matches the existing RViz head slider panel: yaw, pitch.
        self.head_joint_command_pub.publish(self._position_command(head))

    def fullbody_position_controllers_ready(self) -> bool:
        return all(
            self.count_subscribers(topic) > 0
            for topic in (
                "/left_forward_position_controller/commands",
                "/right_forward_position_controller/commands",
                "/lift_manual_position_controller/step_command",
                "/head_forward_position_controller/commands",
            )
        )

    def request_manual_joint_control(
        self, include_arms: bool = True
    ) -> tuple[bool, str]:
        self._manual_joint_control_include_arms = bool(include_arms)
        self._manual_joint_control_requested = True
        if self._manual_joint_control_state == "acquired":
            self._create_manual_joint_publishers(
                include_arms=self._manual_joint_control_include_arms
            )
            self._manual_joint_control_detail = (
                "手动关节控制已获取"
                if self._manual_joint_control_include_arms
                else "立柱与头部手动控制已获取"
            )
            return True, self._manual_joint_control_detail
        self.poll_manual_joint_control()
        if self._manual_joint_control_state == "error":
            return False, self._manual_joint_control_detail
        return True, "正在切换升降柱并申请手动控制"

    def release_manual_joint_control(self) -> None:
        self._manual_joint_control_requested = False
        # Relinquish arm command ownership immediately. The lift handoff can
        # then complete asynchronously through controller_manager.
        self._destroy_manual_joint_publishers()
        self.poll_manual_joint_control()

    def shutdown_manual_joint_control(self, timeout_sec: float = 6.0) -> None:
        """Return lift ownership to MoveIt before the ROS executor stops."""
        self.release_manual_joint_control()
        deadline = time.monotonic() + timeout_sec
        while self._manual_joint_control_future is not None and time.monotonic() < deadline:
            time.sleep(0.05)
            self.poll_manual_joint_control()

    def manual_joint_control_ready(self, include_arms: bool = True) -> bool:
        auxiliary_ready = (
            getattr(self, "_manual_joint_control_state", "acquired") == "acquired"
            and self.head_joint_command_pub is not None
            and self.lift_step_command_pub is not None
        )
        if not auxiliary_ready:
            return False
        return not include_arms or (
            self.left_joint_command_pub is not None
            and self.right_joint_command_pub is not None
        )

    def manual_joint_control_status(self) -> tuple[str, str]:
        return self._manual_joint_control_state, self._manual_joint_control_detail

    def competition_actions_active(self) -> bool:
        """Report automatic workflow actions that still own robot motion."""
        sequenced = any(
            sequence > 0
            for sequence in (
                self._active_goal_sequence,
                self._active_shelf_align_sequence,
                self._active_shelf_traverse_sequence,
                self._active_shelf_view_sequence,
                self._active_right_gripper_sequence,
                self._active_pick_test_sequence,
                self._active_retreat_test_sequence,
            )
        )
        return sequenced or any(
            handle is not None
            for handle in (
                self.grasp_debug_goal_handle,
                self.grasp_debug_pick_goal_handle,
                self.grasp_debug_execute_goal_handle,
            )
        )

    def poll_manual_joint_control(self) -> None:
        future = self._manual_joint_control_future
        if future is not None and future.done():
            self._manual_joint_control_future = None
            operation = self._manual_joint_control_operation
            self._manual_joint_control_operation = None
            try:
                response = future.result()
            except Exception as exc:
                self._manual_joint_control_state = "error"
                self._manual_joint_control_detail = f"控制器切换异常：{exc}"
            else:
                if operation == "list":
                    self._handle_lift_controller_states(response)
                elif not response.ok:
                    self._manual_joint_control_state = "error"
                    self._manual_joint_control_detail = "controller_manager 拒绝控制器切换"
                elif self._manual_joint_control_state == "acquiring":
                    if self._manual_joint_control_requested:
                        self._create_manual_joint_publishers(
                            include_arms=self._manual_joint_control_include_arms
                        )
                        self._manual_joint_control_state = "acquired"
                        self._manual_joint_control_detail = (
                            "手动关节控制已获取"
                            if self._manual_joint_control_include_arms
                            else "立柱与头部手动控制已获取"
                        )
                    else:
                        self._manual_joint_control_state = "acquired"
                else:
                    self._manual_joint_control_state = "released"
                    self._manual_joint_control_detail = "已交还 MoveIt 控制"

        if self._manual_joint_control_future is not None:
            return
        if self._manual_joint_control_requested:
            if self._manual_joint_control_state in ("released", "error", "acquiring"):
                self._query_lift_controller_states(manual=True)
        elif self._manual_joint_control_state in ("acquired", "releasing"):
            self._query_lift_controller_states(manual=False)

    def _query_lift_controller_states(self, manual: bool) -> None:
        if not self._lift_controller_list_client.service_is_ready():
            self._manual_joint_control_state = "acquiring" if manual else "releasing"
            self._manual_joint_control_detail = (
                "/controller_manager/list_controllers 服务未就绪，等待中"
            )
            return
        self._manual_joint_control_state = "acquiring" if manual else "releasing"
        self._manual_joint_control_detail = (
            "正在检查手动关节控制" if manual else "正在检查 MoveIt 控制"
        )
        self._manual_joint_control_operation = "list"
        self._manual_joint_control_future = self._lift_controller_list_client.call_async(
            ListControllers.Request()
        )

    def _handle_lift_controller_states(self, response) -> None:
        states = {controller.name: controller.state for controller in response.controller}
        manual = self._manual_joint_control_requested
        target = (
            "lift_manual_position_controller" if manual else "lift_position_controller"
        )
        other = (
            "lift_position_controller" if manual else "lift_manual_position_controller"
        )
        if target not in states or other not in states:
            self._manual_joint_control_state = "error"
            self._manual_joint_control_detail = "升降柱控制器尚未加载完成"
            return
        if states[target] == "active" and states[other] != "active":
            if manual:
                self._create_manual_joint_publishers(
                    include_arms=self._manual_joint_control_include_arms
                )
                self._manual_joint_control_state = "acquired"
                self._manual_joint_control_detail = (
                    "手动关节控制已获取"
                    if self._manual_joint_control_include_arms
                    else "立柱与头部手动控制已获取"
                )
            else:
                self._manual_joint_control_state = "released"
                self._manual_joint_control_detail = "已交还 MoveIt 控制"
            return
        self._start_lift_controller_switch(manual, deactivate_other=states[other] == "active")

    def _start_lift_controller_switch(
        self, manual: bool, deactivate_other: bool = True
    ) -> None:
        if not self._lift_controller_switch_client.service_is_ready():
            self._manual_joint_control_state = "acquiring" if manual else "releasing"
            self._manual_joint_control_detail = "/controller_manager/switch_controller 服务未就绪，等待中"
            return
        request = SwitchController.Request()
        request.activate_controllers = [
            "lift_manual_position_controller" if manual else "lift_position_controller"
        ]
        request.deactivate_controllers = (
            ["lift_position_controller" if manual else "lift_manual_position_controller"]
            if deactivate_other
            else []
        )
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        request.timeout.sec = 5
        self._manual_joint_control_state = "acquiring" if manual else "releasing"
        self._manual_joint_control_detail = (
            "正在切换到手动关节控制" if manual else "正在交还 MoveIt 控制"
        )
        self._manual_joint_control_future = self._lift_controller_switch_client.call_async(
            request
        )
        self._manual_joint_control_operation = "switch"

    def _create_manual_joint_publishers(self, include_arms: bool = True) -> None:
        if include_arms:
            if self.left_joint_command_pub is None:
                self.left_joint_command_pub = self.create_publisher(
                    Float64MultiArray, "/left_forward_position_controller/commands", 10
                )
            if self.right_joint_command_pub is None:
                self.right_joint_command_pub = self.create_publisher(
                    Float64MultiArray, "/right_forward_position_controller/commands", 10
                )
        else:
            self._destroy_manual_arm_publishers()
        if self.head_joint_command_pub is None:
            self.head_joint_command_pub = self.create_publisher(
                Float64MultiArray, "/head_forward_position_controller/commands", 10
            )
        if self.lift_step_command_pub is None:
            self.lift_step_command_pub = self.create_publisher(
                Float64MultiArray, "/lift_manual_position_controller/step_command", 10
            )

    def _destroy_manual_arm_publishers(self) -> None:
        for attribute in ("left_joint_command_pub", "right_joint_command_pub"):
            publisher = getattr(self, attribute, None)
            if publisher is not None:
                self.destroy_publisher(publisher)
                setattr(self, attribute, None)

    def _destroy_manual_joint_publishers(self) -> None:
        for attribute in (
            "left_joint_command_pub",
            "right_joint_command_pub",
            "head_joint_command_pub",
            "lift_step_command_pub",
        ):
            publisher = getattr(self, attribute, None)
            if publisher is not None:
                self.destroy_publisher(publisher)
                setattr(self, attribute, None)
        self.reset_lift_command_target()

    def reset_lift_command_target(self) -> None:
        self._last_lift_step_target = None

    def lift_homing_ready(self) -> bool:
        return self.lift_start_homing_client.service_is_ready()

    def start_lift_homing(self):
        if not self.lift_homing_ready():
            return None
        return self.lift_start_homing_client.call_async(Trigger.Request())

    def freedrive_home_ready(self) -> bool:
        return self.freedrive_home_client.service_is_ready()

    def start_freedrive_home(self):
        if not self.freedrive_home_ready():
            return None
        return self.freedrive_home_client.call_async(Trigger.Request())

    def cancel_freedrive_home(self):
        if not self.freedrive_cancel_home_client.service_is_ready():
            return None
        return self.freedrive_cancel_home_client.call_async(Trigger.Request())

    @staticmethod
    def _validate_arm_side(side: str) -> str:
        selected = str(side).lower()
        if selected not in ("left", "right"):
            raise ValueError(f"unsupported arm side: {side}")
        return selected

    def arm_gain_services_ready(self, sides=("left", "right")) -> bool:
        return all(
            self._arm_parameter_get_clients[self._validate_arm_side(side)].service_is_ready()
            and self._arm_parameter_set_clients[self._validate_arm_side(side)].service_is_ready()
            for side in sides
        )

    def get_arm_gains(self, side: str):
        selected = self._validate_arm_side(side)
        client = self._arm_parameter_get_clients[selected]
        if not client.service_is_ready():
            return None
        request = GetParameters.Request()
        request.names = [
            f"{gain}_joint{joint}"
            for joint in range(1, 8)
            for gain in ("kp", "kd")
        ]
        return client.call_async(request)

    def set_arm_gains(self, side: str, gains: dict[str, float]):
        selected = self._validate_arm_side(side)
        client = self._arm_parameter_set_clients[selected]
        if not client.service_is_ready():
            return None
        request = SetParameters.Request()
        request.parameters = [
            Parameter(name=name, value=float(value)).to_parameter_msg()
            for name, value in gains.items()
        ]
        return client.call_async(request)

    def _debug_cb(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 45:
            return
        state = self.telemetry
        state.global_drive_scale = msg.data[3]
        state.worst_angle_error = msg.data[4]
        for index in range(4):
            base = 5 + index * 10
            wheel = state.wheels[index]
            wheel.requested_angle = msg.data[base]
            wheel.commanded_angle = msg.data[base + 1]
            wheel.actual_angle = msg.data[base + 2]
            wheel.angle_error = msg.data[base + 3]
            wheel.target_speed = msg.data[base + 5]
            wheel.commanded_speed = msg.data[base + 6]
            wheel.actual_speed = msg.data[base + 7]
            wheel.steering_scale = msg.data[base + 9]
        self._seen("/swerve_drive_controller/debug_states")

    def _scan_cb(self, _msg: LaserScan) -> None:
        self._seen("/scan")

    def _safety_cloud_cb(self, _msg: PointCloud2) -> None:
        self._seen("/livox/body_cloud_safety")

    def _hold_cb(self, _msg: Empty) -> None:
        self.telemetry.localization_hold_count += 1
        self._seen("/localization_hold")

    def _localization_state_cb(self, msg: Bool) -> None:
        state = self.telemetry
        now = time.monotonic()
        new_value = bool(msg.data)
        if new_value:
            if not state.localization_ok or state.localization_true_since is None:
                state.localization_true_since = now
        else:
            state.localization_true_since = None
        state.localization_ok = new_value
        state.localization_state_seen = True
        self._seen("/localization_ok")

    def _global_costmap_cb(self, msg: Costmap) -> None:
        metadata = msg.metadata
        self.telemetry.global_costmap = CostmapSnapshot(
            resolution=float(metadata.resolution),
            origin_x=float(metadata.origin.position.x),
            origin_y=float(metadata.origin.position.y),
            width=int(metadata.size_x),
            height=int(metadata.size_y),
            data=tuple(int(value) for value in msg.data),
            received_at=time.monotonic(),
        )
        self._seen("/global_costmap/costmap_raw")

    def _path_cb(self, msg: NavPath) -> None:
        self.telemetry.path = [
            (
                pose.pose.position.x,
                pose.pose.position.y,
                yaw_from_quaternion(pose.pose.orientation),
            )
            for pose in msg.poses
        ]
        self._seen("/plan")

    def sample_graph_and_tf(self) -> None:
        # ROS graph introspection is best-effort.  It must never prevent TF
        # sampling, because the latter drives the live chassis pose display.
        try:
            self.telemetry.node_names = {
                f"{namespace.rstrip('/')}/{name}" if namespace != "/" else f"/{name}"
                for name, namespace in self.get_node_names_and_namespaces()
            }
        except Exception as exc:
            self.get_logger().debug(f"node graph query failed: {exc}")

        try:
            action_suffix = "/_action/send_goal"
            self.telemetry.action_servers = {
                service_name[: -len(action_suffix)]
                for service_name, _types in self.get_service_names_and_types()
                if service_name.endswith(action_suffix)
            }
        except Exception as exc:
            self.get_logger().debug(f"action graph query failed: {exc}")

        # Always continue with the live transforms, even if either graph
        # query above is unavailable during ROS discovery or shutdown.
        self.sample_tf()

    def sample_tf(self) -> None:
        """Sample live localization transforms independently of graph discovery."""
        self._sample_transform("map", "base_link", base_pose=True)
        self._sample_transform("map", "odom", base_pose=False)

    def _sample_transform(self, parent: str, child: str, base_pose: bool) -> None:
        try:
            transform = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
        except TransformException:
            return
        translation = transform.transform.translation
        yaw = yaw_from_quaternion(transform.transform.rotation)
        state = self.telemetry
        if base_pose:
            state.map_x = translation.x
            state.map_y = translation.y
            state.map_yaw = yaw
            state.has_map_pose = True
            state.trail.append((translation.x, translation.y))
            self._seen("map->base_link")
            return

        if "map->odom" in state.last_seen:
            state.icp_step_xy = math.hypot(
                translation.x - state.map_to_odom_x,
                translation.y - state.map_to_odom_y,
            )
            state.icp_step_yaw = abs(normalize_angle(yaw - state.map_to_odom_yaw))
        state.map_to_odom_x = translation.x
        state.map_to_odom_y = translation.y
        state.map_to_odom_yaw = yaw
        self._seen("map->odom")

    def publish_teleop(self, vx: float, vy: float, wz: float) -> None:
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        self.teleop_pub.publish(msg)

    def publish_start_clearance(self, vx: float) -> None:
        """Feed the startup straight motion through Nav2's full safety chain."""
        msg = Twist()
        msg.linear.x = float(vx)
        self.start_clearance_pub.publish(msg)

    def publish_mapping_teleop(self, vx: float, vy: float, wz: float) -> None:
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        self.mapping_teleop_pub.publish(msg)

    def publish_initial_pose(self, x: float, y: float, yaw: float) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        self.initial_pose_pub.publish(msg)

    def request_save_map(self, directory: Path):
        request = SaveMaps.Request()
        request.file_path = str(directory)
        request.save_patches = True
        return self.save_maps_client.call_async(request)

    def send_navigation_goal(self, x: float, y: float, yaw: float) -> bool:
        if (not self.telemetry.localization_state_seen or
                not self.telemetry.localization_ok or
                not self.nav_action_client.server_is_ready()):
            return False
        goal = NavigateToPose.Goal()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        self.goal_pose_pub.publish(goal.pose)
        self.telemetry.goal_x = x
        self.telemetry.goal_y = y
        self.telemetry.goal_yaw = yaw
        self.telemetry.goal_valid = True
        self.telemetry.action_status = "sending"
        self._goal_sequence += 1
        sequence = self._goal_sequence
        self._active_goal_sequence = sequence
        self.goal_handle = None
        self._cancel_requested_sequence = None
        self._cancel_future = None
        self._cancel_deadline = None
        future = self.nav_action_client.send_goal_async(
            goal,
            feedback_callback=lambda feedback: self._goal_feedback(feedback, sequence),
        )
        future.add_done_callback(
            lambda response_future: self._goal_response(response_future, sequence)
        )
        return True

    def _goal_feedback(self, feedback, sequence: int) -> None:
        if sequence != self._active_goal_sequence:
            return
        if self._cancel_requested_sequence == sequence:
            return
        self.telemetry.action_status = "running"
        self.telemetry.action_remaining_distance = feedback.feedback.distance_remaining

    def _goal_response(self, future, sequence: int) -> None:
        try:
            goal_handle = future.result()
        except Exception:
            if sequence == self._active_goal_sequence:
                self.telemetry.action_status = "error"
                self._clear_goal_state(sequence)
            return

        if sequence != self._active_goal_sequence:
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return

        self.goal_handle = goal_handle
        if not goal_handle.accepted:
            self.telemetry.action_status = "rejected"
            self._clear_goal_state(sequence)
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda completed_future: self._goal_result(completed_future, sequence)
        )
        if self._cancel_requested_sequence == sequence:
            self._request_goal_cancel(sequence, goal_handle)
        else:
            self.telemetry.action_status = "running"

    def _goal_result(self, future, sequence: int) -> None:
        if sequence != self._active_goal_sequence:
            return
        try:
            status = future.result().status
        except Exception:
            self.telemetry.action_status = "error"
            self._clear_goal_state(sequence)
            return
        self.telemetry.action_status = "succeeded" if status == 4 else f"finished:{status}"
        self._clear_goal_state(sequence)

    def cancel_navigation_goal(self) -> None:
        sequence = self._active_goal_sequence
        if sequence <= 0 or self.telemetry.action_status not in (
            "sending",
            "running",
            "canceling",
        ):
            return
        if self._cancel_requested_sequence == sequence:
            return

        self._cancel_requested_sequence = sequence
        self._cancel_deadline = time.monotonic() + self.CANCEL_RESPONSE_TIMEOUT_SEC
        self.telemetry.action_status = "canceling"
        if self.goal_handle is not None:
            self._request_goal_cancel(sequence, self.goal_handle)

    def _request_goal_cancel(self, sequence: int, goal_handle) -> None:
        if sequence != self._active_goal_sequence or self._cancel_future is not None:
            return
        try:
            self._cancel_future = goal_handle.cancel_goal_async()
            self._cancel_future.add_done_callback(
                lambda cancel_future: self._cancel_response(cancel_future, sequence)
            )
        except Exception:
            self.telemetry.action_status = "cancel_error"
            self._clear_goal_state(sequence)

    def _cancel_response(self, future, sequence: int) -> None:
        if sequence != self._active_goal_sequence:
            return
        self._cancel_future = None
        try:
            accepted = bool(future.result().goals_canceling)
        except Exception:
            self.telemetry.action_status = "cancel_error"
        else:
            self.telemetry.action_status = "canceled" if accepted else "cancel_rejected"
        self._clear_goal_state(sequence)

    def _check_cancel_timeout(self) -> None:
        sequence = self._cancel_requested_sequence
        if (
            sequence is None
            or sequence != self._active_goal_sequence
            or self._cancel_deadline is None
            or time.monotonic() < self._cancel_deadline
        ):
            return
        self.telemetry.action_status = "cancel_timeout"
        self._clear_goal_state(sequence)

    def _clear_goal_state(self, sequence: int) -> None:
        if sequence != self._active_goal_sequence:
            return
        self.goal_handle = None
        self._cancel_requested_sequence = None
        self._cancel_future = None
        self._cancel_deadline = None
        self._active_goal_sequence = 0

    def send_shelf_alignment_goal(
        self,
        station_id: str,
        target_map_yaw: float,
        *,
        target_clearance: float = 0.05,
        clearance_tolerance: float = 0.01,
        yaw_tolerance: float = math.radians(1.0),
        timeout_sec: float = 20.0,
        measure_only: bool = False,
        yaw_only: bool = False,
    ) -> bool:
        if (
            not self.telemetry.localization_state_seen
            or not self.telemetry.localization_ok
            or not self.shelf_align_action_client.server_is_ready()
        ):
            return False
        goal = ShelfFineAlign.Goal()
        goal.station_id = station_id
        goal.target_clearance = float(target_clearance)
        goal.clearance_tolerance = float(clearance_tolerance)
        goal.target_map_yaw = float(target_map_yaw)
        goal.yaw_tolerance = float(yaw_tolerance)
        goal.timeout_sec = float(timeout_sec)
        goal.measure_only = bool(measure_only)
        goal.yaw_only = bool(yaw_only)
        state = self.telemetry
        state.shelf_align_status = "sending"
        state.shelf_align_phase = ""
        state.shelf_clearance = 0.0
        state.shelf_yaw_error = 0.0
        state.shelf_stable_samples = 0
        state.shelf_align_message = ""
        self._shelf_align_sequence += 1
        sequence = self._shelf_align_sequence
        self._active_shelf_align_sequence = sequence
        self.shelf_align_goal_handle = None
        future = self.shelf_align_action_client.send_goal_async(
            goal,
            feedback_callback=lambda feedback: self._shelf_align_feedback(
                feedback, sequence
            ),
        )
        future.add_done_callback(
            lambda response: self._shelf_align_response(response, sequence)
        )
        return True

    def _shelf_align_feedback(self, feedback, sequence: int) -> None:
        if sequence != self._active_shelf_align_sequence:
            return
        message = feedback.feedback
        state = self.telemetry
        state.shelf_align_status = "running"
        state.shelf_align_phase = message.phase
        state.shelf_clearance = float(message.measured_clearance)
        state.shelf_yaw_error = float(message.yaw_error)
        state.shelf_stable_samples = int(message.stable_samples)

    def _shelf_align_response(self, future, sequence: int) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            if sequence == self._active_shelf_align_sequence:
                self.telemetry.shelf_align_status = "error"
                self.telemetry.shelf_align_message = str(exc)
                self._clear_shelf_align_state(sequence)
            return
        if sequence != self._active_shelf_align_sequence:
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        if not goal_handle.accepted:
            self.telemetry.shelf_align_status = "rejected"
            self._clear_shelf_align_state(sequence)
            return
        self.shelf_align_goal_handle = goal_handle
        self.telemetry.shelf_align_status = "running"
        result = goal_handle.get_result_async()
        result.add_done_callback(
            lambda completed: self._shelf_align_result(completed, sequence)
        )

    def _shelf_align_result(self, future, sequence: int) -> None:
        if sequence != self._active_shelf_align_sequence:
            return
        try:
            wrapped = future.result()
            payload = wrapped.result
            succeeded = wrapped.status == 4 and bool(payload.success)
            state = self.telemetry
            state.shelf_align_status = "succeeded" if succeeded else "failed"
            state.shelf_clearance = float(payload.final_clearance)
            state.shelf_yaw_error = float(payload.final_yaw_error)
            state.shelf_align_message = str(payload.message)
        except Exception as exc:
            self.telemetry.shelf_align_status = "error"
            self.telemetry.shelf_align_message = str(exc)
        self._clear_shelf_align_state(sequence)

    def cancel_shelf_alignment_goal(self) -> None:
        sequence = self._active_shelf_align_sequence
        if sequence <= 0:
            return
        self.telemetry.shelf_align_status = "canceling"
        handle = self.shelf_align_goal_handle
        if handle is not None:
            try:
                future = handle.cancel_goal_async()
                future.add_done_callback(
                    lambda _future: self._clear_shelf_align_state(sequence)
                )
                return
            except Exception:
                self.telemetry.shelf_align_status = "cancel_error"
        self._clear_shelf_align_state(sequence)

    def _clear_shelf_align_state(self, sequence: int) -> None:
        if sequence != self._active_shelf_align_sequence:
            return
        self.shelf_align_goal_handle = None
        self._active_shelf_align_sequence = 0

    def send_shelf_traverse_goal(
        self,
        station_id: str,
        target_map_x: float,
        target_map_y: float,
        target_map_yaw: float,
        *,
        target_clearance: float = 0.05,
        clearance_tolerance: float = 0.01,
        lateral_tolerance: float = 0.03,
        yaw_tolerance: float = math.radians(1.0),
        max_lateral_speed: float = 0.04,
        timeout_sec: float = 35.0,
        lateral_only: bool = False,
    ) -> bool:
        if (
            not self.telemetry.localization_state_seen
            or not self.telemetry.localization_ok
            or not self.shelf_traverse_action_client.server_is_ready()
        ):
            return False
        goal = ShelfTraverse.Goal()
        goal.station_id = str(station_id).strip()
        goal.target_map_x = float(target_map_x)
        goal.target_map_y = float(target_map_y)
        goal.target_map_yaw = float(target_map_yaw)
        goal.target_clearance = float(target_clearance)
        goal.clearance_tolerance = float(clearance_tolerance)
        goal.lateral_tolerance = float(lateral_tolerance)
        goal.yaw_tolerance = float(yaw_tolerance)
        goal.max_lateral_speed = float(max_lateral_speed)
        goal.timeout_sec = float(timeout_sec)
        goal.lateral_only = bool(lateral_only)
        state = self.telemetry
        state.shelf_traverse_status = "sending"
        state.shelf_traverse_phase = ""
        state.shelf_traverse_remaining = 0.0
        state.shelf_traverse_clearance = 0.0
        state.shelf_traverse_yaw_error = 0.0
        state.shelf_traverse_message = ""
        self._shelf_traverse_sequence += 1
        sequence = self._shelf_traverse_sequence
        self._active_shelf_traverse_sequence = sequence
        self.shelf_traverse_goal_handle = None
        future = self.shelf_traverse_action_client.send_goal_async(
            goal,
            feedback_callback=lambda feedback: self._shelf_traverse_feedback(
                feedback, sequence
            ),
        )
        future.add_done_callback(
            lambda response: self._shelf_traverse_response(response, sequence)
        )
        return True

    def _shelf_traverse_feedback(self, feedback, sequence: int) -> None:
        if sequence != self._active_shelf_traverse_sequence:
            return
        message = feedback.feedback
        state = self.telemetry
        state.shelf_traverse_status = "running"
        state.shelf_traverse_phase = str(message.phase)
        state.shelf_traverse_remaining = float(
            message.remaining_lateral_distance
        )
        state.shelf_traverse_clearance = float(message.measured_clearance)
        state.shelf_traverse_yaw_error = float(message.yaw_error)

    def _shelf_traverse_response(self, future, sequence: int) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            if sequence == self._active_shelf_traverse_sequence:
                self.telemetry.shelf_traverse_status = "error"
                self.telemetry.shelf_traverse_message = str(exc)
                self._clear_shelf_traverse_state(sequence)
            return
        if sequence != self._active_shelf_traverse_sequence:
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        if not goal_handle.accepted:
            self.telemetry.shelf_traverse_status = "rejected"
            self.telemetry.shelf_traverse_message = "shelf traverse goal was rejected"
            self._clear_shelf_traverse_state(sequence)
            return
        self.shelf_traverse_goal_handle = goal_handle
        self.telemetry.shelf_traverse_status = "running"
        result = goal_handle.get_result_async()
        result.add_done_callback(
            lambda completed: self._shelf_traverse_result(completed, sequence)
        )

    def _shelf_traverse_result(self, future, sequence: int) -> None:
        if sequence != self._active_shelf_traverse_sequence:
            return
        try:
            wrapped = future.result()
            payload = wrapped.result
            succeeded = wrapped.status == 4 and bool(payload.success)
            state = self.telemetry
            state.shelf_traverse_status = "succeeded" if succeeded else "failed"
            state.shelf_traverse_remaining = float(payload.final_lateral_error)
            state.shelf_traverse_clearance = float(payload.final_clearance)
            state.shelf_traverse_yaw_error = float(payload.final_yaw_error)
            state.shelf_traverse_message = str(payload.message)
        except Exception as exc:
            self.telemetry.shelf_traverse_status = "error"
            self.telemetry.shelf_traverse_message = str(exc)
        self._clear_shelf_traverse_state(sequence)

    def _clear_shelf_traverse_state(self, sequence: int) -> None:
        if sequence != self._active_shelf_traverse_sequence:
            return
        self.shelf_traverse_goal_handle = None
        self._active_shelf_traverse_sequence = 0

    def cancel_shelf_traverse_goal(self) -> None:
        sequence = self._active_shelf_traverse_sequence
        if sequence <= 0:
            return
        self.telemetry.shelf_traverse_status = "canceling"
        handle = self.shelf_traverse_goal_handle
        if handle is not None:
            try:
                future = handle.cancel_goal_async()
                future.add_done_callback(
                    lambda _future: self._clear_shelf_traverse_state(sequence)
                )
                return
            except Exception as exc:
                self.telemetry.shelf_traverse_status = "cancel_error"
                self.telemetry.shelf_traverse_message = str(exc)
        self._clear_shelf_traverse_state(sequence)

    def send_shelf_view_goal(
        self,
        station_id: str,
        shelf_level: int,
        arm_name: str = "right",
        *,
        velocity_scale: float = 0.10,
        acceleration_scale: float = 0.10,
    ) -> bool:
        """Move to a configured shelf posture after the base is safely stopped."""
        if not self.shelf_view_action_client.server_is_ready():
            return False
        goal = MoveToShelfView.Goal()
        goal.station_id = str(station_id).strip()
        goal.shelf_level = int(shelf_level)
        normalized_arm = str(arm_name).strip().lower()
        if normalized_arm == "both":
            goal.arm = MoveToShelfView.Goal.BOTH
        elif normalized_arm == "left":
            goal.arm = MoveToShelfView.Goal.LEFT
        else:
            goal.arm = MoveToShelfView.Goal.RIGHT
        goal.use_explicit_lift_position = False
        goal.lift_position = 0.0
        goal.joint_names = []
        goal.joint_positions = []
        goal.velocity_scale = float(velocity_scale)
        goal.acceleration_scale = float(acceleration_scale)
        goal.planning_timeout_sec = 8.0
        goal.timeout_sec = 60.0
        goal.execute = True
        self.get_logger().info(
            "Sending taught posture: "
            f"station={goal.station_id}, level={goal.shelf_level}, arm={normalized_arm}"
        )

        state = self.telemetry
        state.shelf_view_status = "sending"
        state.shelf_view_phase = ""
        state.shelf_view_level = int(shelf_level)
        state.shelf_view_lift_position = 0.0
        state.shelf_view_message = ""
        self._shelf_view_sequence += 1
        sequence = self._shelf_view_sequence
        self._active_shelf_view_sequence = sequence
        self.shelf_view_goal_handle = None
        future = self.shelf_view_action_client.send_goal_async(
            goal,
            feedback_callback=lambda feedback: self._shelf_view_feedback(
                feedback, sequence
            ),
        )
        future.add_done_callback(
            lambda response: self._shelf_view_response(response, sequence)
        )
        return True

    def _shelf_view_feedback(self, feedback, sequence: int) -> None:
        if sequence != self._active_shelf_view_sequence:
            return
        message = feedback.feedback
        state = self.telemetry
        state.shelf_view_status = "running"
        state.shelf_view_phase = str(message.phase)
        state.shelf_view_lift_position = float(message.lift_position)

    def _shelf_view_response(self, future, sequence: int) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            if sequence == self._active_shelf_view_sequence:
                self.telemetry.shelf_view_status = "error"
                self.telemetry.shelf_view_message = str(exc)
                self._clear_shelf_view_state(sequence)
            return
        if sequence != self._active_shelf_view_sequence:
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        if not goal_handle.accepted:
            self.telemetry.shelf_view_status = "rejected"
            self.telemetry.shelf_view_message = "move_to_shelf_view goal was rejected"
            self.get_logger().error(self.telemetry.shelf_view_message)
            self._clear_shelf_view_state(sequence)
            return
        self.shelf_view_goal_handle = goal_handle
        self.telemetry.shelf_view_status = "running"
        result = goal_handle.get_result_async()
        result.add_done_callback(
            lambda completed: self._shelf_view_result(completed, sequence)
        )

    def _shelf_view_result(self, future, sequence: int) -> None:
        if sequence != self._active_shelf_view_sequence:
            return
        try:
            wrapped = future.result()
            payload = wrapped.result
            succeeded = wrapped.status == 4 and bool(payload.success)
            state = self.telemetry
            state.shelf_view_status = "succeeded" if succeeded else "failed"
            state.shelf_view_lift_position = float(payload.lift_position)
            state.shelf_view_message = str(payload.message)
            log_message = (
                "Taught posture result: "
                f"status={state.shelf_view_status}, message={state.shelf_view_message}"
            )
            # rclpy identifies a log call by its source location and rejects
            # changing that location's severity between calls.  Keep success
            # and failure calls on distinct lines so the real child-Action
            # error is never replaced by a logger exception.
            if succeeded:
                self.get_logger().info(log_message)
            else:
                self.get_logger().error(log_message)
        except Exception as exc:
            self.telemetry.shelf_view_status = "error"
            self.telemetry.shelf_view_message = str(exc)
        self._clear_shelf_view_state(sequence)

    def cancel_shelf_view_goal(self) -> None:
        sequence = self._active_shelf_view_sequence
        if sequence <= 0:
            return
        handle = self.shelf_view_goal_handle
        self.telemetry.shelf_view_status = "canceling"
        if handle is not None:
            try:
                future = handle.cancel_goal_async()
                future.add_done_callback(
                    lambda _future: self._clear_shelf_view_state(sequence)
                )
                return
            except Exception as exc:
                self.telemetry.shelf_view_status = "cancel_error"
                self.telemetry.shelf_view_message = str(exc)
        self._clear_shelf_view_state(sequence)

    def _clear_shelf_view_state(self, sequence: int) -> None:
        if sequence != self._active_shelf_view_sequence:
            return
        self.shelf_view_goal_handle = None
        self._active_shelf_view_sequence = 0

    def send_right_gripper_goal(
        self, position: float, *, max_effort: float = 20.0
    ) -> bool:
        """Command the right gripper through its dedicated ros2_control action."""
        target = float(position)
        effort = float(max_effort)
        if (
            not math.isfinite(target)
            or not 0.0 <= target <= 0.044
            or not math.isfinite(effort)
            or effort < 0.0
            or not self.right_gripper_action_client.server_is_ready()
        ):
            return False

        goal = GripperCommand.Goal()
        goal.command.position = target
        goal.command.max_effort = effort
        state = self.telemetry
        state.right_gripper_status = "sending"
        state.right_gripper_position = float(
            state.joint_positions.get("openarmx_right_finger_joint1", 0.0)
        )
        state.right_gripper_reached_goal = False
        state.right_gripper_stalled = False
        state.right_gripper_message = ""
        self._right_gripper_sequence += 1
        sequence = self._right_gripper_sequence
        self._active_right_gripper_sequence = sequence
        self.right_gripper_goal_handle = None
        future = self.right_gripper_action_client.send_goal_async(
            goal,
            feedback_callback=lambda feedback: self._right_gripper_feedback(
                feedback, sequence
            ),
        )
        future.add_done_callback(
            lambda response: self._right_gripper_response(response, sequence)
        )
        return True

    def _right_gripper_feedback(self, feedback, sequence: int) -> None:
        if sequence != self._active_right_gripper_sequence:
            return
        message = feedback.feedback
        state = self.telemetry
        state.right_gripper_status = "running"
        state.right_gripper_position = float(message.position)
        state.right_gripper_reached_goal = bool(message.reached_goal)
        state.right_gripper_stalled = bool(message.stalled)

    def _right_gripper_response(self, future, sequence: int) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            if sequence == self._active_right_gripper_sequence:
                self.telemetry.right_gripper_status = "error"
                self.telemetry.right_gripper_message = str(exc)
                self._clear_right_gripper_state(sequence)
            return
        if sequence != self._active_right_gripper_sequence:
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        if not goal_handle.accepted:
            self.telemetry.right_gripper_status = "rejected"
            self.telemetry.right_gripper_message = "right gripper goal was rejected"
            self._clear_right_gripper_state(sequence)
            return
        self.right_gripper_goal_handle = goal_handle
        self.telemetry.right_gripper_status = "running"
        result = goal_handle.get_result_async()
        result.add_done_callback(
            lambda completed: self._right_gripper_result(completed, sequence)
        )

    def _right_gripper_result(self, future, sequence: int) -> None:
        if sequence != self._active_right_gripper_sequence:
            return
        try:
            wrapped = future.result()
            payload = wrapped.result
            reached = bool(payload.reached_goal)
            stalled = bool(payload.stalled)
            succeeded = wrapped.status == 4 and reached and not stalled
            state = self.telemetry
            state.right_gripper_status = "succeeded" if succeeded else "failed"
            state.right_gripper_position = float(payload.position)
            state.right_gripper_reached_goal = reached
            state.right_gripper_stalled = stalled
            state.right_gripper_message = (
                "right gripper reached release position"
                if succeeded
                else f"right gripper release incomplete (reached={reached}, stalled={stalled})"
            )
        except Exception as exc:
            self.telemetry.right_gripper_status = "error"
            self.telemetry.right_gripper_message = str(exc)
        self._clear_right_gripper_state(sequence)

    def _clear_right_gripper_state(self, sequence: int) -> None:
        if sequence != self._active_right_gripper_sequence:
            return
        self.right_gripper_goal_handle = None
        self._active_right_gripper_sequence = 0

    def cancel_right_gripper_goal(self) -> None:
        sequence = self._active_right_gripper_sequence
        if sequence <= 0:
            return
        handle = self.right_gripper_goal_handle
        self.telemetry.right_gripper_status = "canceling"
        if handle is not None:
            try:
                future = handle.cancel_goal_async()
                future.add_done_callback(
                    lambda _future: self._clear_right_gripper_state(sequence)
                )
                return
            except Exception as exc:
                self.telemetry.right_gripper_status = "cancel_error"
                self.telemetry.right_gripper_message = str(exc)
        self._clear_right_gripper_state(sequence)

    def send_mock_pick_goal(self, object_id: str, arm_name: str) -> bool:
        """Run the isolated mock action; this never commands manipulation hardware."""
        if not self.mock_pick_action_client.server_is_ready():
            return False
        normalized_arm = str(arm_name).strip().lower()
        if normalized_arm not in ("left", "right"):
            return False
        goal = PickObject.Goal()
        goal.object_id = str(object_id).strip() or "mock_object"
        goal.detection_id = f"mock:{normalized_arm}:{goal.object_id}"
        goal.object_pose.header.stamp = self.get_clock().now().to_msg()
        goal.object_pose.header.frame_id = "base_link"
        goal.object_pose.pose.position.x = 0.55
        goal.object_pose.pose.position.z = 0.80
        goal.object_pose.pose.orientation.w = 1.0
        goal.arm_name = normalized_arm
        goal.grasp_strategy = "gui_mock_success"
        goal.retreat_to_transport = True
        goal.timeout_sec = 5.0
        state = self.telemetry
        state.pick_test_status = "sending"
        state.pick_test_phase = ""
        state.pick_test_progress = 0.0
        state.pick_test_attached = False
        state.pick_test_message = ""
        self._pick_test_sequence += 1
        sequence = self._pick_test_sequence
        self._active_pick_test_sequence = sequence
        self.mock_pick_goal_handle = None
        future = self.mock_pick_action_client.send_goal_async(
            goal,
            feedback_callback=lambda feedback: self._mock_pick_feedback(
                feedback, sequence
            ),
        )
        future.add_done_callback(
            lambda response: self._mock_pick_response(response, sequence)
        )
        return True

    def invalidate_grasp_plan(self, plan_owner: Optional[str] = None) -> bool:
        """Clear only the plan owned by ``plan_owner``.

        The competition and debug pages share one ROS node, but a debug-page
        parameter edit must not invalidate a live competition plan.
        """
        requested_owner = str(plan_owner or "").strip()
        current_owner = str(getattr(self, "grasp_debug_plan_owner", "") or "")
        if requested_owner and current_owner and requested_owner != current_owner:
            return False
        self._grasp_plan_sequence = int(getattr(self, "_grasp_plan_sequence", 0)) + 1
        self._active_grasp_plan_sequence = 0
        self.grasp_debug_plan_result = None
        self.grasp_debug_plan_owner = ""
        if getattr(self, "grasp_debug_status", "") == "plan_ready":
            self.grasp_debug_status = "idle"
            self.grasp_debug_message = "抓取计划已清除，请重新规划"
        return True

    def send_grasp_detect_goal(
        self,
        object_id: str,
        arm_name: str,
        qwen_grounding_prompt: str = "",
        *,
        plan_owner: str = "debug",
    ) -> bool:
        if not self.invalidate_grasp_plan(plan_owner):
            return False
        if not self.detect_object_action_client.server_is_ready() and not (
            self.detect_object_action_client.wait_for_server(timeout_sec=1.0)
        ):
            self.grasp_debug_status = "unavailable"
            self.grasp_debug_message = "/detect_object 服务端未就绪，请检查抓取后端"
            return False
        goal = DetectObject.Goal()
        goal.object_id = str(object_id).strip()
        goal.sku_name = goal.object_id
        goal.qwen_grounding_prompt = str(qwen_grounding_prompt).strip()
        goal.arm_name = str(arm_name).strip().lower()
        goal.min_confidence = 0.0
        goal.timeout_sec = 30.0
        self.grasp_debug_result = None
        self.grasp_debug_plan_owner = str(plan_owner).strip() or "debug"
        self.grasp_debug_status = "sending"
        future = self.detect_object_action_client.send_goal_async(
            goal, feedback_callback=self._grasp_detect_feedback
        )
        future.add_done_callback(self._grasp_detect_response)
        return True

    def _grasp_detect_feedback(self, wrapped) -> None:
        self.grasp_debug_status = str(wrapped.feedback.phase)
        self.grasp_debug_message = f"confidence={wrapped.feedback.confidence:.3f}"

    def _grasp_detect_response(self, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:
            self.grasp_debug_status, self.grasp_debug_message = "error", str(exc)
            self.grasp_debug_plan_owner = ""
            return
        if not handle.accepted:
            self.grasp_debug_status, self.grasp_debug_message = "rejected", "检测请求被拒绝"
            self.grasp_debug_plan_owner = ""
            return
        self.grasp_debug_goal_handle = handle
        final = handle.get_result_async()
        final.add_done_callback(self._grasp_detect_result)

    def _grasp_detect_result(self, future) -> None:
        try:
            wrapped = future.result()
            payload = wrapped.result
            self.grasp_debug_result = payload if payload.found else None
            self.grasp_debug_status = "detected" if payload.found else "failed"
            self.grasp_debug_message = str(payload.message)
        except Exception as exc:
            self.grasp_debug_status, self.grasp_debug_message = "error", str(exc)
            self.grasp_debug_plan_owner = ""
        if self.grasp_debug_status == "failed":
            self.grasp_debug_plan_owner = ""
        self.grasp_debug_goal_handle = None

    def send_grasp_plan_goal(
        self,
        object_id: str,
        arm_name: str,
        *,
        initial_raise_m: float = 0.0,
        pregrasp_height_offset_m: float = 0.0,
        grasp_height_offset_m: float = 0.0,
        retreat_height_offset_m: float = 0.0,
        approach_distance_m: float = 0.0,
        plan_owner: str = "debug",
    ) -> bool:
        requested_owner = str(plan_owner).strip() or "debug"
        current_owner = str(getattr(self, "grasp_debug_plan_owner", "") or "")
        if current_owner and current_owner != requested_owner:
            return False
        detected = self.grasp_debug_result
        if detected is None or not detected.detection_id:
            self.grasp_debug_status, self.grasp_debug_message = "invalid", "请先完成仅检测"
            return False
        if not self.plan_pick_action_client.server_is_ready() and not (
            self.plan_pick_action_client.wait_for_server(timeout_sec=1.0)
        ):
            self.grasp_debug_status, self.grasp_debug_message = (
                "unavailable",
                "/plan_pick 服务端未就绪，请检查抓取序列节点",
            )
            return False
        if not self.plan_pregrasp_action_client.server_is_ready() and not (
            self.plan_pregrasp_action_client.wait_for_server(timeout_sec=1.0)
        ):
            self.grasp_debug_status, self.grasp_debug_message = (
                "unavailable",
                "/openflex/plan_pregrasp 未就绪，请检查 MoveIt 抓取规划节点",
            )
            return False
        if not self.cartesian_approach_action_client.server_is_ready() and not (
            self.cartesian_approach_action_client.wait_for_server(timeout_sec=1.0)
        ):
            self.grasp_debug_status, self.grasp_debug_message = (
                "unavailable",
                "/openflex/cartesian_approach 未就绪，请检查 Cartesian 规划节点",
            )
            return False
        goal = PlanPick.Goal()
        goal.object_id = str(object_id).strip()
        goal.detection_id = detected.detection_id
        goal.arm_name = str(arm_name).strip().lower()
        goal.grasp_strategy = "graspnet_shelf"
        goal.height_offset_m = 0.0
        goal.initial_raise_m = float(initial_raise_m)
        goal.pregrasp_height_offset_m = float(pregrasp_height_offset_m)
        goal.grasp_height_offset_m = float(grasp_height_offset_m)
        goal.retreat_height_offset_m = float(retreat_height_offset_m)
        goal.approach_distance_m = float(approach_distance_m)
        # Keep the base fixed while jointly planning the selected arm and lift.
        goal.max_dof = PlanPick.Goal.ARM_WITH_LIFT
        goal.velocity_scale = 0.08
        goal.acceleration_scale = 0.08
        goal.timeout_sec = 60.0
        self.grasp_debug_plan_result = None
        self.grasp_debug_plan_owner = requested_owner
        self._grasp_plan_sequence = int(
            getattr(self, "_grasp_plan_sequence", 0)
        ) + 1
        sequence = self._grasp_plan_sequence
        self._active_grasp_plan_sequence = sequence
        self.grasp_debug_status = "planning"
        self.grasp_debug_message = (
            "分阶段抬高："
            f"起始 {goal.initial_raise_m * 1000.0:.1f} / "
            f"预抓取 {goal.pregrasp_height_offset_m * 1000.0:.1f} / "
            f"抓取 {goal.grasp_height_offset_m * 1000.0:.1f} / "
            f"回撤 {goal.retreat_height_offset_m * 1000.0:.1f} mm；"
            + (
                "进给距离自动"
                if goal.approach_distance_m <= 0.0
                else f"进给距离 {goal.approach_distance_m * 1000.0:.1f} mm"
            )
        )
        future = self.plan_pick_action_client.send_goal_async(
            goal, feedback_callback=self._grasp_plan_feedback
        )
        future.add_done_callback(
            lambda response: self._grasp_plan_response(
                response, sequence, requested_owner
            )
        )
        return True

    def _grasp_plan_feedback(self, wrapped) -> None:
        self.grasp_debug_status = str(wrapped.feedback.phase)
        self.grasp_debug_message = f"progress={wrapped.feedback.progress:.0%}"

    def _grasp_plan_response(self, future, sequence=None, plan_owner=None) -> None:
        if sequence is not None and sequence != self._active_grasp_plan_sequence:
            try:
                handle = future.result()
                if handle.accepted:
                    handle.cancel_goal_async()
            except Exception:
                pass
            return
        try:
            handle = future.result()
        except Exception as exc:
            self.grasp_debug_status, self.grasp_debug_message = "error", str(exc)
            self.grasp_debug_plan_owner = ""
            return
        if not handle.accepted:
            self.grasp_debug_status, self.grasp_debug_message = "rejected", "规划请求被拒绝"
            if sequence is None or sequence == self._active_grasp_plan_sequence:
                self.grasp_debug_plan_owner = ""
            return
        self.grasp_debug_pick_goal_handle = handle
        final = handle.get_result_async()
        final.add_done_callback(
            lambda completed: self._grasp_plan_result(
                completed, sequence, plan_owner
            )
        )

    def _grasp_plan_result(self, future, sequence=None, plan_owner=None) -> None:
        if sequence is not None and sequence != self._active_grasp_plan_sequence:
            return
        try:
            payload = future.result().result
            self.grasp_debug_plan_result = payload if payload.success else None
            self.grasp_debug_plan_owner = (
                str(plan_owner or "") if payload.success else ""
            )
            self.grasp_debug_status = "plan_ready" if payload.success else "failed"
            self.grasp_debug_message = str(payload.message)
        except Exception as exc:
            self.grasp_debug_status, self.grasp_debug_message = "error", str(exc)
            self.grasp_debug_plan_owner = ""
        self.grasp_debug_pick_goal_handle = None

    def send_grasp_execute_goal(
        self, object_id: str, arm_name: str, *, plan_owner: str = "debug"
    ) -> bool:
        requested_owner = str(plan_owner).strip() or "debug"
        current_owner = str(getattr(self, "grasp_debug_plan_owner", "") or "")
        if current_owner and current_owner != requested_owner:
            return False
        planned = self.grasp_debug_plan_result
        if planned is None or not planned.plan_id:
            self.grasp_debug_status, self.grasp_debug_message = (
                "invalid",
                "请先完成仅规划，并在计划有效期内执行",
            )
            return False
        if not self.execute_pick_action_client.server_is_ready() and not (
            self.execute_pick_action_client.wait_for_server(timeout_sec=1.0)
        ):
            self.grasp_debug_status, self.grasp_debug_message = (
                "unavailable",
                "/execute_pick 服务端未就绪，请检查抓取序列节点",
            )
            return False
        goal = ExecutePick.Goal()
        goal.plan_id = planned.plan_id
        goal.object_id = str(object_id).strip()
        goal.arm_name = str(arm_name).strip().lower()
        goal.timeout_sec = 120.0
        self.grasp_debug_plan_result = None
        self.grasp_debug_status = "executing"
        self.grasp_debug_message = "已提交单次缓存计划"
        future = self.execute_pick_action_client.send_goal_async(
            goal, feedback_callback=self._grasp_execute_feedback
        )
        future.add_done_callback(self._grasp_execute_response)
        return True

    def _grasp_execute_feedback(self, wrapped) -> None:
        self.grasp_debug_status = str(wrapped.feedback.phase)
        self.grasp_debug_message = f"progress={wrapped.feedback.progress:.0%}"

    def _grasp_execute_response(self, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:
            self.grasp_debug_status, self.grasp_debug_message = "error", str(exc)
            return
        if not handle.accepted:
            self.grasp_debug_status, self.grasp_debug_message = (
                "rejected",
                "执行请求被拒绝，计划可能已失效或已使用",
            )
            self.grasp_debug_plan_owner = ""
            return
        self.grasp_debug_execute_goal_handle = handle
        final = handle.get_result_async()
        final.add_done_callback(self._grasp_execute_result)

    def _grasp_execute_result(self, future) -> None:
        try:
            payload = future.result().result
            self.grasp_debug_status = "complete" if payload.success else "failed"
            self.grasp_debug_message = str(payload.message)
        except Exception as exc:
            self.grasp_debug_status, self.grasp_debug_message = "error", str(exc)
            self.grasp_debug_plan_owner = ""
        self.grasp_debug_plan_owner = ""
        self.grasp_debug_execute_goal_handle = None

    def cancel_grasp_debug_goal(self) -> None:
        for handle in (
            self.grasp_debug_goal_handle,
            self.grasp_debug_pick_goal_handle,
            self.grasp_debug_execute_goal_handle,
        ):
            if handle is not None:
                handle.cancel_goal_async()
        self.grasp_debug_status = "canceling"
        self.invalidate_grasp_plan()

    def _mock_pick_feedback(self, feedback, sequence: int) -> None:
        if sequence != self._active_pick_test_sequence:
            return
        message = feedback.feedback
        self.telemetry.pick_test_status = "running"
        self.telemetry.pick_test_phase = str(message.phase)
        self.telemetry.pick_test_progress = float(message.progress)

    def _mock_pick_response(self, future, sequence: int) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            if sequence == self._active_pick_test_sequence:
                self.telemetry.pick_test_status = "error"
                self.telemetry.pick_test_message = str(exc)
                self._clear_pick_test_state(sequence)
            return
        if sequence != self._active_pick_test_sequence:
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        if not goal_handle.accepted:
            self.telemetry.pick_test_status = "rejected"
            self._clear_pick_test_state(sequence)
            return
        self.mock_pick_goal_handle = goal_handle
        self.telemetry.pick_test_status = "running"
        result = goal_handle.get_result_async()
        result.add_done_callback(
            lambda completed: self._mock_pick_result(completed, sequence)
        )

    def _mock_pick_result(self, future, sequence: int) -> None:
        if sequence != self._active_pick_test_sequence:
            return
        try:
            wrapped = future.result()
            payload = wrapped.result
            succeeded = (
                wrapped.status == 4
                and bool(payload.success)
                and bool(payload.object_attached)
            )
            state = self.telemetry
            state.pick_test_status = "succeeded" if succeeded else "failed"
            state.pick_test_progress = 1.0 if succeeded else state.pick_test_progress
            state.pick_test_attached = bool(payload.object_attached)
            state.pick_test_message = str(payload.message)
        except Exception as exc:
            self.telemetry.pick_test_status = "error"
            self.telemetry.pick_test_message = str(exc)
        self._clear_pick_test_state(sequence)

    def _clear_pick_test_state(self, sequence: int) -> None:
        if sequence != self._active_pick_test_sequence:
            return
        self.mock_pick_goal_handle = None
        self._active_pick_test_sequence = 0

    def cancel_mock_pick_goal(self) -> None:
        """Cancel an active mock goal, including one still awaiting acceptance."""
        sequence = self._active_pick_test_sequence
        if sequence <= 0:
            return
        handle = self.mock_pick_goal_handle
        if handle is None:
            self.telemetry.pick_test_status = "canceled"
            self._clear_pick_test_state(sequence)
            return
        self.telemetry.pick_test_status = "canceling"
        try:
            future = handle.cancel_goal_async()
            future.add_done_callback(
                lambda completed: self._mock_pick_cancel_response(
                    completed, sequence
                )
            )
        except Exception as exc:
            self.telemetry.pick_test_status = "cancel_error"
            self.telemetry.pick_test_message = str(exc)
            self._clear_pick_test_state(sequence)

    def _mock_pick_cancel_response(self, future, sequence: int) -> None:
        if sequence != self._active_pick_test_sequence:
            return
        try:
            accepted = bool(future.result().goals_canceling)
        except Exception as exc:
            self.telemetry.pick_test_status = "cancel_error"
            self.telemetry.pick_test_message = str(exc)
        else:
            self.telemetry.pick_test_status = (
                "canceled" if accepted else "cancel_rejected"
            )
        self._clear_pick_test_state(sequence)

    def send_station_retreat_goal(
        self,
        station_id: str,
        *,
        distance: float = 0.15,
        max_speed: float = 0.04,
        timeout_sec: float = 8.0,
    ) -> bool:
        if (
            not self.telemetry.localization_state_seen
            or not self.telemetry.localization_ok
            or not self.retreat_action_client.server_is_ready()
        ):
            return False
        goal = RetreatFromStation.Goal()
        goal.station_id = str(station_id).strip()
        goal.distance = float(distance)
        goal.max_speed = float(max_speed)
        goal.timeout_sec = float(timeout_sec)
        state = self.telemetry
        state.retreat_test_status = "sending"
        state.retreat_test_phase = ""
        state.retreat_test_distance = 0.0
        state.retreat_test_message = ""
        self._retreat_test_sequence += 1
        sequence = self._retreat_test_sequence
        self._active_retreat_test_sequence = sequence
        self.retreat_goal_handle = None
        future = self.retreat_action_client.send_goal_async(
            goal,
            feedback_callback=lambda feedback: self._retreat_feedback(
                feedback, sequence
            ),
        )
        future.add_done_callback(
            lambda response: self._retreat_response(response, sequence)
        )
        return True

    def _retreat_feedback(self, feedback, sequence: int) -> None:
        if sequence != self._active_retreat_test_sequence:
            return
        message = feedback.feedback
        self.telemetry.retreat_test_status = "running"
        self.telemetry.retreat_test_phase = str(message.phase)
        self.telemetry.retreat_test_distance = float(message.traveled_distance)

    def _retreat_response(self, future, sequence: int) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            if sequence == self._active_retreat_test_sequence:
                self.telemetry.retreat_test_status = "error"
                self.telemetry.retreat_test_message = str(exc)
                self._clear_retreat_test_state(sequence)
            return
        if sequence != self._active_retreat_test_sequence:
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        if not goal_handle.accepted:
            self.telemetry.retreat_test_status = "rejected"
            self._clear_retreat_test_state(sequence)
            return
        self.retreat_goal_handle = goal_handle
        self.telemetry.retreat_test_status = "running"
        result = goal_handle.get_result_async()
        result.add_done_callback(
            lambda completed: self._retreat_result(completed, sequence)
        )

    def _retreat_result(self, future, sequence: int) -> None:
        if sequence != self._active_retreat_test_sequence:
            return
        try:
            wrapped = future.result()
            payload = wrapped.result
            succeeded = wrapped.status == 4 and bool(payload.success)
            state = self.telemetry
            state.retreat_test_status = "succeeded" if succeeded else "failed"
            state.retreat_test_distance = float(payload.traveled_distance)
            state.retreat_test_message = str(payload.message)
        except Exception as exc:
            self.telemetry.retreat_test_status = "error"
            self.telemetry.retreat_test_message = str(exc)
        self._clear_retreat_test_state(sequence)

    def _clear_retreat_test_state(self, sequence: int) -> None:
        if sequence != self._active_retreat_test_sequence:
            return
        self.retreat_goal_handle = None
        self._active_retreat_test_sequence = 0

    def cancel_station_retreat_goal(self) -> None:
        """Cancel retreat immediately when the GUI navigation workflow stops."""
        sequence = self._active_retreat_test_sequence
        if sequence <= 0:
            return
        handle = self.retreat_goal_handle
        if handle is None:
            self.telemetry.retreat_test_status = "canceled"
            self._clear_retreat_test_state(sequence)
            return
        self.telemetry.retreat_test_status = "canceling"
        try:
            future = handle.cancel_goal_async()
            future.add_done_callback(
                lambda completed: self._retreat_cancel_response(
                    completed, sequence
                )
            )
        except Exception as exc:
            self.telemetry.retreat_test_status = "cancel_error"
            self.telemetry.retreat_test_message = str(exc)
            self._clear_retreat_test_state(sequence)

    def _retreat_cancel_response(self, future, sequence: int) -> None:
        if sequence != self._active_retreat_test_sequence:
            return
        try:
            accepted = bool(future.result().goals_canceling)
        except Exception as exc:
            self.telemetry.retreat_test_status = "cancel_error"
            self.telemetry.retreat_test_message = str(exc)
        else:
            self.telemetry.retreat_test_status = (
                "canceled" if accepted else "cancel_rejected"
            )
        self._clear_retreat_test_state(sequence)


class RosExecutorThread:
    """Continuously drain ROS callbacks without blocking the Qt event loop."""

    def __init__(self, node: NavigationRosNode):
        self.executor = SingleThreadedExecutor(context=node.context)
        self.executor.add_node(node)
        self.thread = threading.Thread(
            target=self._spin,
            name="openflex-navigation-gui-ros",
            daemon=True,
        )

    def start(self) -> None:
        self.thread.start()

    def _spin(self) -> None:
        try:
            self.executor.spin()
        except ExternalShutdownException:
            pass

    def shutdown(self) -> None:
        self.executor.shutdown(timeout_sec=2.0)
        self.thread.join(timeout=2.0)


class StatusDot(QLabel):
    COLORS = {
        "idle": "#777777",
        "running": "#27ae60",
        "warning": "#d4a017",
        "error": "#d64545",
    }

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(12, 12)
        self.set_state("idle")

    def set_state(self, state: str) -> None:
        color = self.COLORS.get(state, self.COLORS["idle"])
        self.setStyleSheet(f"background:{color}; border-radius:6px; border:1px solid #202020;")


class MetricWidget(QFrame):
    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        self.setObjectName("metric")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 7, 10, 7)
        layout.setSpacing(2)
        title_label = QLabel(title)
        title_label.setObjectName("metricTitle")
        self.value = QLabel("--")
        self.value.setObjectName("metricValue")
        layout.addWidget(title_label)
        layout.addWidget(self.value)

    def set_value(self, text: str, level: str = "normal") -> None:
        colors = {
            "normal": "#f0f0f0",
            "ok": "#5fd38d",
            "warning": "#f1c75b",
            "error": "#ff6b6b",
        }
        self.value.setText(text)
        self.value.setStyleSheet(f"color:{colors.get(level, colors['normal'])};")


class PathPlot(QWidget):
    goal_selected = pyqtSignal(float, float)
    pose_selected = pyqtSignal(float, float, float)
    pose_rejected = pyqtSignal(str)

    def __init__(self, telemetry: Telemetry, parent=None):
        super().__init__(parent)
        self.telemetry = telemetry
        self.reference_points: list[tuple[float, float, str]] = []
        self.setMinimumSize(520, 380)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._bounds = (-2.0, 2.0, -2.0, 2.0)
        self._plot_rect = None
        self._map_image: Optional[QImage] = None
        self._map_resolution = 0.05
        self._map_origin = (0.0, 0.0, 0.0)
        self._map_world_bounds: Optional[tuple[float, float, float, float]] = None
        self._goal_safety: Optional[GoalSafetyMap] = None
        self._mouse_world: Optional[tuple[float, float]] = None
        self._selected_pose: Optional[tuple[float, float, float]] = None
        self._drag_start_world: Optional[tuple[float, float]] = None
        self._drag_current_world: Optional[tuple[float, float]] = None
        self._view_zoom = 1.0
        self._view_center: Optional[tuple[float, float]] = None
        self._pan_start_pixel = None
        self._pan_start_center: Optional[tuple[float, float]] = None
        self._selected_yaw = 0.0
        self.setMouseTracking(True)
        self.setToolTip(
            "滚轮缩放；右键拖动平移；右键双击恢复全图；左键拖动选择目标与车头方向"
        )

    def set_selected_pose(self, x: float, y: float, yaw: float) -> None:
        self._selected_pose = (x, y, yaw)
        self._selected_yaw = yaw
        self.update()

    def clear_map(self) -> None:
        self._map_image = None
        self._map_world_bounds = None
        self._goal_safety = None
        self.reset_view()
        self.update()

    def reset_view(self) -> None:
        self._view_zoom = 1.0
        self._view_center = None
        self._pan_start_pixel = None
        self._pan_start_center = None
        self.unsetCursor()
        self.update()

    def zoom_view(self, factor: float) -> None:
        xmin, xmax, ymin, ymax = self._bounds
        if self._view_center is None:
            self._view_center = ((xmin + xmax) * 0.5, (ymin + ymax) * 0.5)
        self._view_zoom = max(1.0, min(20.0, self._view_zoom * factor))
        self.update()

    def set_map(
        self,
        pgm_path: Path,
        yaml_path: Path,
        safety_geometry: GoalSafetyGeometry | None = None,
    ) -> None:
        """Load a ROS map and display it in the same world coordinate frame."""
        grid = read_pgm(Path(pgm_path))
        metadata = parse_map_yaml(Path(yaml_path))
        if metadata.resolution is None or metadata.origin is None:
            raise ValueError("地图 YAML 缺少 resolution 或 origin")
        self._map_image = QImage(
            bytes(grid.pixels),
            grid.width,
            grid.height,
            grid.width,
            QImage.Format_Grayscale8,
        ).copy()
        self._map_resolution = metadata.resolution
        self._map_origin = metadata.origin
        self._goal_safety = (
            GoalSafetyMap(grid, metadata, safety_geometry)
            if safety_geometry is not None
            else None
        )
        ox, oy, _ = metadata.origin
        self._map_world_bounds = (
            ox,
            ox + grid.width * metadata.resolution,
            oy,
            oy + grid.height * metadata.resolution,
        )
        self.reset_view()
        self.update()

    def paintEvent(self, _event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor("#171717"))
        margin = 34
        rect = self.rect().adjusted(margin, 18, -18, -margin)
        self._plot_rect = rect
        xmin, xmax, ymin, ymax = self._calculate_bounds()
        self._bounds = xmin, xmax, ymin, ymax

        if self._map_image is not None and self._map_world_bounds is not None:
            mxmin, mxmax, mymin, mymax = self._map_world_bounds
            top_left_x, top_left_y = self._to_pixel(mxmin, mymax)
            bottom_right_x, bottom_right_y = self._to_pixel(mxmax, mymin)
            map_rect = QRectF(
                min(top_left_x, bottom_right_x),
                min(top_left_y, bottom_right_y),
                abs(bottom_right_x - top_left_x),
                abs(bottom_right_y - top_left_y),
            )
            painter.drawImage(map_rect, self._map_image)
            self._draw_goal_safety_overlay(painter, map_rect)

        painter.setPen(QPen(QColor("#303030"), 1))
        gx = math.ceil(xmin)
        while gx <= math.floor(xmax):
            px, _ = self._to_pixel(gx, ymin)
            painter.drawLine(int(px), rect.top(), int(px), rect.bottom())
            painter.setPen(QColor("#777777"))
            painter.drawText(int(px + 3), rect.bottom() + 18, f"{gx}m")
            painter.setPen(QPen(QColor("#303030"), 1))
            gx += 1
        gy = math.ceil(ymin)
        while gy <= math.floor(ymax):
            _, py = self._to_pixel(xmin, gy)
            painter.drawLine(rect.left(), int(py), rect.right(), int(py))
            painter.setPen(QColor("#777777"))
            label_rect = QRectF(0, py - 9, margin - 5, 18)
            painter.drawText(label_rect, Qt.AlignRight | Qt.AlignVCenter, f"{gy}m")
            painter.setPen(QPen(QColor("#303030"), 1))
            gy += 1

        # World-frame axes make the origin and the sign of each coordinate
        # unambiguous when selecting a goal on the map.
        if ymin <= 0.0 <= ymax:
            _, py = self._to_pixel(0.0, 0.0)
            painter.setPen(QPen(QColor("#d6a84f"), 2))
            painter.drawLine(rect.left(), int(py), rect.right(), int(py))
            painter.drawText(rect.right() - 28, int(py - 5), "+X")
        if xmin <= 0.0 <= xmax:
            px, _ = self._to_pixel(0.0, 0.0)
            painter.setPen(QPen(QColor("#72b7e8"), 2))
            painter.drawLine(int(px), rect.top(), int(px), rect.bottom())
            painter.drawText(int(px + 5), rect.top() + 14, "+Y")
        if xmin <= 0.0 <= xmax and ymin <= 0.0 <= ymax:
            ox, oy = self._to_pixel(0.0, 0.0)
            label_rect = QRectF(ox + 5, oy - 24, 68, 19)
            painter.fillRect(label_rect, QColor(12, 12, 12, 210))
            painter.setPen(QColor("#ffffff"))
            painter.drawText(label_rect, Qt.AlignCenter, "O (0, 0)")

        self._draw_polyline(painter, self.telemetry.path, QColor("#e25b52"), 2.2)
        trail = [(x, y, 0.0) for x, y in list(self.telemetry.trail)[-1200:]]
        self._draw_polyline(painter, trail, QColor("#49a6d8"), 2.0)
        self._draw_reference_points(painter)

        if self._selected_pose is not None:
            selected_yaw = math.degrees(self._selected_pose[2])
            self._draw_pose_arrow(
                painter,
                self._selected_pose,
                QColor("#5fd3e8"),
                f"选中 {selected_yaw:.1f}°",
            )
        if self._drag_start_world is not None and self._drag_current_world is not None:
            dx = self._drag_current_world[0] - self._drag_start_world[0]
            dy = self._drag_current_world[1] - self._drag_start_world[1]
            if math.hypot(dx, dy) >= 0.02:
                yaw = math.atan2(dy, dx)
                self._draw_pose_arrow(
                    painter,
                    (self._drag_start_world[0], self._drag_start_world[1], yaw),
                    QColor("#ff77d4"),
                    f"Yaw {math.degrees(yaw):.1f}°",
                )

        if self.telemetry.path:
            gx, gy, _ = self.telemetry.path[-1]
            px, py = self._to_pixel(gx, gy)
            painter.setPen(QPen(QColor("#f1c75b"), 2))
            painter.setBrush(Qt.NoBrush)
            painter.drawEllipse(int(px - 7), int(py - 7), 14, 14)

        if self.telemetry.has_map_pose:
            self._draw_robot(painter)

        painter.setPen(QColor("#aaaaaa"))
        painter.drawText(
            rect.right() - 96,
            14,
            f"MAP  {self._view_zoom:.1f}x",
        )
        if self._mouse_world is not None:
            mx, my = self._mouse_world
            painter.setPen(QColor("#ffffff"))
            painter.drawText(rect.left() + 6, 14, f"鼠标 X {mx:.3f} m   Y {my:.3f} m")
        if self._goal_safety is not None:
            painter.setPen(QColor("#ff6b6b"))
            painter.drawText(
                rect.left() + 6,
                rect.bottom() + 29,
                "红区：车体外沿距障碍不足 "
                f"{self._goal_safety.geometry.clearance_m * 100:.0f} cm，"
                f"当前方向 {math.degrees(self._selected_yaw):.1f}°",
            )

    def _calculate_bounds(self) -> tuple[float, float, float, float]:
        if self._map_world_bounds is not None:
            xmin, xmax, ymin, ymax = self._map_world_bounds
            width = max(xmax - xmin, 1e-3)
            height = max(ymax - ymin, 1e-3)
            aspect = max(self.width(), 1) / max(self.height(), 1)
            if width / height < aspect:
                width = height * aspect
            else:
                height = width / aspect
            cx, cy = (
                self._view_center
                if self._view_center is not None
                else ((xmin + xmax) * 0.5, (ymin + ymax) * 0.5)
            )
            width /= self._view_zoom
            height /= self._view_zoom
            return (
                cx - width * 0.5,
                cx + width * 0.5,
                cy - height * 0.5,
                cy + height * 0.5,
            )
        points = [(x, y) for x, y, _ in self.telemetry.path]
        points.extend(list(self.telemetry.trail)[-1200:])
        points.extend((x, y) for x, y, _ in self.reference_points)
        if self.telemetry.has_map_pose:
            points.append((self.telemetry.map_x, self.telemetry.map_y))
        if not points:
            return -2.0, 2.0, -2.0, 2.0
        xs = [point[0] for point in points]
        ys = [point[1] for point in points]
        center_x = (min(xs) + max(xs)) * 0.5
        center_y = (min(ys) + max(ys)) * 0.5
        width = max(max(xs) - min(xs), 4.0) * 1.25
        height = max(max(ys) - min(ys), 4.0) * 1.25
        aspect = max(self.width(), 1) / max(self.height(), 1)
        if width / height < aspect:
            width = height * aspect
        else:
            height = width / aspect
        return (
            center_x - width * 0.5,
            center_x + width * 0.5,
            center_y - height * 0.5,
            center_y + height * 0.5,
        )

    def _to_pixel(self, x: float, y: float) -> tuple[float, float]:
        if self._plot_rect is None:
            return 0.0, 0.0
        xmin, xmax, ymin, ymax = self._bounds
        px = self._plot_rect.left() + (x - xmin) / max(xmax - xmin, 1e-6) * self._plot_rect.width()
        py = self._plot_rect.bottom() - (y - ymin) / max(ymax - ymin, 1e-6) * self._plot_rect.height()
        return px, py

    def _from_pixel(self, px: float, py: float) -> tuple[float, float]:
        xmin, xmax, ymin, ymax = self._bounds
        x = xmin + (px - self._plot_rect.left()) / max(self._plot_rect.width(), 1) * (xmax - xmin)
        y = ymin + (self._plot_rect.bottom() - py) / max(self._plot_rect.height(), 1) * (ymax - ymin)
        return x, y

    def _draw_polyline(self, painter, points, color, width) -> None:
        if len(points) < 2:
            return
        polygon = QPolygonF()
        step = max(1, len(points) // 800)
        from PyQt5.QtCore import QPointF

        for x, y, _ in points[::step]:
            px, py = self._to_pixel(x, y)
            polygon.append(QPointF(px, py))
        painter.setPen(QPen(color, width))
        painter.setBrush(Qt.NoBrush)
        painter.drawPolyline(polygon)

    def _draw_goal_safety_overlay(self, painter, map_rect: QRectF) -> None:
        if self._goal_safety is None:
            return
        yaw = self._selected_yaw
        if self._drag_start_world is not None and self._drag_current_world is not None:
            delta_x = self._drag_current_world[0] - self._drag_start_world[0]
            delta_y = self._drag_current_world[1] - self._drag_start_world[1]
            if math.hypot(delta_x, delta_y) >= 0.04:
                yaw = math.atan2(delta_y, delta_x)
        forbidden, boundary = self._goal_safety.masks(yaw)
        grid = self._goal_safety.grid
        overlay = QImage(grid.width, grid.height, QImage.Format_ARGB32)
        overlay.fill(Qt.transparent)
        for index, blocked in enumerate(forbidden):
            if not blocked:
                continue
            overlay.setPixelColor(
                index % grid.width,
                index // grid.width,
                QColor(255, 30, 30, 235 if boundary[index] else 50),
            )
        painter.drawImage(map_rect, overlay)

    def _draw_robot(self, painter) -> None:
        px, py = self._to_pixel(self.telemetry.map_x, self.telemetry.map_y)
        yaw = self.telemetry.map_yaw
        painter.setPen(QPen(QColor("#5fd38d"), 2))
        painter.setBrush(QColor("#244d35"))
        painter.drawEllipse(int(px - 8), int(py - 8), 16, 16)
        hx = px + 22 * math.cos(yaw)
        hy = py - 22 * math.sin(yaw)
        painter.drawLine(int(px), int(py), int(hx), int(hy))

    def _draw_reference_points(self, painter) -> None:
        painter.setPen(QPen(QColor("#f1c75b"), 2))
        painter.setBrush(QColor("#6b5718"))
        for x, y, label in self.reference_points:
            px, py = self._to_pixel(x, y)
            painter.drawEllipse(int(px - 5), int(py - 5), 10, 10)
            painter.drawText(int(px + 8), int(py - 7), label)

    def _draw_pose_arrow(self, painter, pose, color: QColor, label: str) -> None:
        x, y, yaw = pose
        px, py = self._to_pixel(x, y)
        length = 28.0
        ex = px + length * math.cos(yaw)
        ey = py - length * math.sin(yaw)
        painter.setPen(QPen(color, 2.5))
        painter.setBrush(color)
        painter.drawEllipse(int(px - 5), int(py - 5), 10, 10)
        painter.drawLine(int(px), int(py), int(ex), int(ey))
        # Small triangular arrow head, kept in screen coordinates so the
        # direction is obvious even when the map is zoomed to fit.
        left = (ex - 7 * math.cos(yaw - 0.55), ey + 7 * math.sin(yaw - 0.55))
        right = (ex - 7 * math.cos(yaw + 0.55), ey + 7 * math.sin(yaw + 0.55))
        from PyQt5.QtCore import QPointF

        painter.drawLine(int(ex), int(ey), int(left[0]), int(left[1]))
        painter.drawLine(int(ex), int(ey), int(right[0]), int(right[1]))
        if label:
            painter.drawText(int(ex + 5), int(ey - 5), label)

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.RightButton and self._plot_rect is not None:
            if self._plot_rect.contains(event.pos()):
                self._pan_start_pixel = event.pos()
                xmin, xmax, ymin, ymax = self._bounds
                self._pan_start_center = (
                    (xmin + xmax) * 0.5,
                    (ymin + ymax) * 0.5,
                )
                self.setCursor(Qt.ClosedHandCursor)
            return
        if event.button() != Qt.LeftButton or self._plot_rect is None:
            return
        if not self._plot_rect.contains(event.pos()):
            return
        self._drag_start_world = self._from_pixel(event.x(), event.y())
        self._drag_current_world = self._drag_start_world
        self._mouse_world = self._drag_start_world
        self.update()

    def mouseMoveEvent(self, event) -> None:
        if (
            self._pan_start_pixel is not None
            and self._pan_start_center is not None
            and self._plot_rect is not None
        ):
            xmin, xmax, ymin, ymax = self._bounds
            delta_x = (
                event.x() - self._pan_start_pixel.x()
            ) / max(self._plot_rect.width(), 1) * (xmax - xmin)
            delta_y = (
                event.y() - self._pan_start_pixel.y()
            ) / max(self._plot_rect.height(), 1) * (ymax - ymin)
            self._view_center = (
                self._pan_start_center[0] - delta_x,
                self._pan_start_center[1] + delta_y,
            )
            self.update()
            return
        if self._plot_rect is not None and self._plot_rect.contains(event.pos()):
            self._mouse_world = self._from_pixel(event.x(), event.y())
            if self._drag_start_world is not None:
                self._drag_current_world = self._mouse_world
        else:
            self._mouse_world = None
        self.update()

    def mouseReleaseEvent(self, event) -> None:
        if event.button() == Qt.RightButton and self._pan_start_pixel is not None:
            self._pan_start_pixel = None
            self._pan_start_center = None
            self.unsetCursor()
            return
        if event.button() != Qt.LeftButton or self._drag_start_world is None:
            return
        start = self._drag_start_world
        end = self._from_pixel(event.x(), event.y())
        self._drag_start_world = None
        self._drag_current_world = None
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        yaw = self._selected_yaw
        if math.hypot(dx, dy) >= 0.04:
            yaw = math.atan2(dy, dx)
        if self._goal_safety is not None and not self._goal_safety.is_safe(
            start[0], start[1], yaw
        ):
            self._selected_yaw = yaw
            self.pose_rejected.emit(
                self._goal_safety.rejection_reason(start[0], start[1], yaw)
            )
            self.update()
            return
        self.set_selected_pose(start[0], start[1], yaw)
        self.goal_selected.emit(start[0], start[1])
        self.pose_selected.emit(start[0], start[1], yaw)
        self.update()

    def leaveEvent(self, _event) -> None:
        self._mouse_world = None
        self.update()

    def mouseDoubleClickEvent(self, event) -> None:
        if event.button() == Qt.RightButton:
            self.reset_view()
            event.accept()
            return
        super().mouseDoubleClickEvent(event)

    def wheelEvent(self, event) -> None:
        if self._plot_rect is None or not self._plot_rect.contains(event.pos()):
            super().wheelEvent(event)
            return
        old_world = self._from_pixel(event.x(), event.y())
        factor = 1.25 if event.angleDelta().y() > 0 else 0.8
        new_zoom = max(1.0, min(20.0, self._view_zoom * factor))
        if math.isclose(new_zoom, self._view_zoom):
            event.accept()
            return
        old_zoom = self._view_zoom
        xmin, xmax, ymin, ymax = self._bounds
        center_x = (xmin + xmax) * 0.5
        center_y = (ymin + ymax) * 0.5
        ratio = old_zoom / new_zoom
        self._view_center = (
            old_world[0] - (old_world[0] - center_x) * ratio,
            old_world[1] - (old_world[1] - center_y) * ratio,
        )
        self._view_zoom = new_zoom
        self.update()
        event.accept()


class MainWindow(QMainWindow):
    log_signal = pyqtSignal(str)
    metadata_done_signal = pyqtSignal(str)

    def __init__(self, ros_node: Optional[NavigationRosNode]):
        super().__init__()
        self.ros_node = ros_node
        self.telemetry = ros_node.telemetry if ros_node else Telemetry()
        self.settings = QSettings("OpenArmX", "OpenFlexNavigationGUI")
        self.processes: dict[str, QProcess] = {}
        self.process_group_ids: dict[str, int] = {}
        self.process_labels: dict[str, str] = {}
        self.process_output_tail: dict[str, list[str]] = {}
        self.process_stop_requested: set[str] = set()
        self.save_future = None
        self.after_save_action = None
        self.current_record_dir: Optional[Path] = None
        self.teleop_command = (0.0, 0.0, 0.0)
        self.teleop_active_key = None
        self._closing = False

        self.setWindowTitle("OpenFlex 建图定位导航工作台")
        self.setMinimumSize(1180, 760)
        self.resize(1440, 900)
        self.setFocusPolicy(Qt.StrongFocus)

        self.log_signal.connect(self._append_log)
        self.metadata_done_signal.connect(self._append_log)
        self._build_ui()
        self._apply_style()
        self._load_settings()
        self._refresh_maps()

        self.future_timer = QTimer(self)
        self.future_timer.timeout.connect(self._poll_ros_futures)
        self.future_timer.start(50)
        self.telemetry_timer = QTimer(self)
        self.telemetry_timer.timeout.connect(self._update_telemetry)
        self.telemetry_timer.start(200)
        self.graph_timer = QTimer(self)
        self.graph_timer.timeout.connect(self._sample_graph)
        self.graph_timer.start(1000)
        self.teleop_timer = QTimer(self)
        self.teleop_timer.timeout.connect(self._publish_teleop)
        self.teleop_timer.start(50)

    def _build_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(12, 10, 12, 10)
        root.setSpacing(8)

        header = QHBoxLayout()
        title = QLabel("OpenFlex 建图定位导航工作台")
        title.setObjectName("title")
        header.addWidget(title)
        header.addStretch(1)
        self.mode_dot = StatusDot()
        self.mode_label = QLabel("空闲")
        self.record_dot = StatusDot()
        self.record_label = QLabel("未录制")
        self.stop_motion_button = QPushButton("停止速度")
        self.stop_motion_button.setObjectName("dangerButton")
        self.stop_motion_button.setIcon(self.style().standardIcon(QStyle.SP_MediaStop))
        self.stop_motion_button.clicked.connect(self._stop_motion)
        header.addWidget(self.mode_dot)
        header.addWidget(self.mode_label)
        header.addSpacing(16)
        header.addWidget(self.record_dot)
        header.addWidget(self.record_label)
        header.addSpacing(16)
        header.addWidget(self.stop_motion_button)
        root.addLayout(header)

        splitter = QSplitter(Qt.Horizontal)
        splitter.setChildrenCollapsible(False)
        root.addWidget(splitter, 1)

        self.control_tabs = QTabWidget()
        self.control_tabs.setMinimumWidth(440)
        self.control_tabs.setMaximumWidth(510)
        self.control_tabs.addTab(self._build_mapping_tab(), "建图")
        self.control_tabs.addTab(self._build_navigation_tab(), "导航")
        self.control_tabs.addTab(self._build_system_tab(), "系统")
        splitter.addWidget(self.control_tabs)

        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.setContentsMargins(4, 0, 0, 0)
        right_layout.setSpacing(8)

        metrics = QGridLayout()
        metrics.setSpacing(6)
        self.metric_pose = MetricWidget("地图位姿")
        self.metric_goal = MetricWidget("目标误差")
        self.metric_cte = MetricWidget("路径横向误差")
        self.metric_icp = MetricWidget("ICP 单步修正")
        self.metric_cmd = MetricWidget("最终速度")
        self.metric_odom = MetricWidget("LIO / 轮式差异")
        for index, widget in enumerate(
            (
                self.metric_pose,
                self.metric_goal,
                self.metric_cte,
                self.metric_icp,
                self.metric_cmd,
                self.metric_odom,
            )
        ):
            metrics.addWidget(widget, index // 3, index % 3)
        right_layout.addLayout(metrics)

        self.path_plot = PathPlot(self.telemetry)
        self.path_plot.goal_selected.connect(self._plot_goal_selected)
        self.path_plot.pose_selected.connect(self._plot_pose_selected)
        self.path_plot.pose_rejected.connect(self._goal_pose_rejected)
        map_view_controls = QHBoxLayout()
        map_view_controls.setContentsMargins(0, 0, 0, 0)
        map_view_controls.addWidget(QLabel("2D 地图视图"))
        zoom_in = QPushButton("放大 +")
        zoom_in.clicked.connect(lambda: self.path_plot.zoom_view(1.25))
        zoom_out = QPushButton("缩小 −")
        zoom_out.clicked.connect(lambda: self.path_plot.zoom_view(0.8))
        reset_view = QPushButton("显示全图")
        reset_view.clicked.connect(self.path_plot.reset_view)
        map_view_controls.addWidget(zoom_in)
        map_view_controls.addWidget(zoom_out)
        map_view_controls.addWidget(reset_view)
        map_view_controls.addStretch(1)
        map_view_controls.addWidget(QLabel("滚轮缩放｜右键拖动平移"))
        right_layout.addLayout(map_view_controls)
        right_layout.addWidget(self.path_plot, 1)

        self.detail_tabs = QTabWidget()
        self.detail_tabs.setMaximumHeight(230)
        self.detail_tabs.addTab(self._build_wheel_table(), "舵轮状态")
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.document().setMaximumBlockCount(2500)
        self.detail_tabs.addTab(self.log_view, "运行日志")
        right_layout.addWidget(self.detail_tabs)
        splitter.addWidget(right)
        splitter.setSizes([470, 930])

        self.statusBar().showMessage("就绪")

    def _build_map_selector(self, parent_layout) -> None:
        group = QGroupBox("地图")
        layout = QGridLayout(group)
        layout.setColumnStretch(1, 1)
        self.map_root_edit = QComboBox()
        self.map_root_edit.setEditable(True)
        self.map_root_edit.setMinimumContentsLength(8)
        self.map_root_edit.setSizeAdjustPolicy(QComboBox.AdjustToMinimumContentsLengthWithIcon)
        self.map_root_edit.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)
        self.map_root_edit.addItems(
            [
                str(Path(DEFAULT_MAP_ROOT).expanduser()),
                str(Path("~/MID_360_nav/openflex_maps").expanduser()),
            ]
        )
        browse = QPushButton()
        browse.setFixedWidth(44)
        browse.setIcon(self.style().standardIcon(QStyle.SP_DirOpenIcon))
        browse.setToolTip("选择地图根目录")
        browse.clicked.connect(self._choose_map_root)
        self.map_name_combo = QComboBox()
        self.map_name_combo.setEditable(True)
        self.map_name_combo.setMinimumContentsLength(8)
        self.map_name_combo.setSizeAdjustPolicy(QComboBox.AdjustToMinimumContentsLengthWithIcon)
        self.map_name_combo.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)
        self.map_name_combo.currentTextChanged.connect(self._update_map_file_status)
        refresh = QPushButton()
        refresh.setFixedWidth(44)
        refresh.setIcon(self.style().standardIcon(QStyle.SP_BrowserReload))
        refresh.setToolTip("刷新地图列表")
        refresh.clicked.connect(self._refresh_maps)
        self.pcd_status = QLabel("PCD --")
        self.yaml_status = QLabel("2D --")
        layout.addWidget(QLabel("根目录"), 0, 0)
        layout.addWidget(self.map_root_edit, 0, 1)
        layout.addWidget(browse, 0, 2)
        layout.addWidget(QLabel("名称"), 1, 0)
        layout.addWidget(self.map_name_combo, 1, 1)
        layout.addWidget(refresh, 1, 2)
        status_layout = QHBoxLayout()
        status_layout.addWidget(self.pcd_status)
        status_layout.addStretch(1)
        status_layout.addWidget(self.yaml_status)
        layout.addLayout(status_layout, 2, 1, 1, 2)
        self.map_root_edit.currentTextChanged.connect(self._refresh_maps)
        parent_layout.addWidget(group)

    def _build_mapping_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)
        self._build_map_selector(layout)

        launch_group = QGroupBox("建图流程")
        launch_layout = QGridLayout(launch_group)
        self.mapping_speed = self._double_spin(0.1, 2.0, 0.4, 0.1, " m/s")
        self.mapping_accel = self._double_spin(0.1, 2.0, 0.4, 0.1, " m/s2")
        self.mapping_rviz = QCheckBox("RViz")
        self.mapping_rviz.setChecked(True)
        self.start_mapping_button = QPushButton("开始建图")
        self.start_mapping_button.setObjectName("primaryButton")
        self.start_mapping_button.setIcon(self.style().standardIcon(QStyle.SP_MediaPlay))
        self.start_mapping_button.clicked.connect(self._start_mapping)
        self.stop_mapping_button = QPushButton("停止")
        self.stop_mapping_button.setIcon(self.style().standardIcon(QStyle.SP_MediaStop))
        self.stop_mapping_button.clicked.connect(lambda: self._stop_process("mapping"))
        self.save_3d_button = QPushButton("保存 3D 地图")
        self.save_3d_button.setIcon(self.style().standardIcon(QStyle.SP_DialogSaveButton))
        self.save_3d_button.clicked.connect(self._save_3d_map)
        self.finish_map_button = QPushButton("保存并生成 2D 地图")
        self.finish_map_button.setObjectName("primaryButton")
        self.finish_map_button.clicked.connect(self._save_and_convert)
        self.edit_2d_button = QPushButton("编辑 2D 地图")
        self.edit_2d_button.clicked.connect(self._open_map_editor)
        launch_layout.addWidget(QLabel("轮速上限"), 0, 0)
        launch_layout.addWidget(self.mapping_speed, 0, 1)
        launch_layout.addWidget(QLabel("加速度"), 1, 0)
        launch_layout.addWidget(self.mapping_accel, 1, 1)
        launch_layout.addWidget(self.mapping_rviz, 2, 0, 1, 2)
        launch_layout.addWidget(self.start_mapping_button, 3, 0)
        launch_layout.addWidget(self.stop_mapping_button, 3, 1)
        launch_layout.addWidget(self.save_3d_button, 4, 0)
        launch_layout.addWidget(self.finish_map_button, 4, 1)
        launch_layout.addWidget(self.edit_2d_button, 5, 0, 1, 2)
        layout.addWidget(launch_group)

        convert_group = QGroupBox("二维地图参数")
        convert_layout = QFormLayout(convert_group)
        self.map_resolution = self._double_spin(0.01, 0.2, 0.05, 0.01, " m")
        self.projection_min_z = self._double_spin(-2.0, 3.0, -0.35, 0.05, " m")
        self.projection_max_z = self._double_spin(-1.0, 5.0, 2.80, 0.05, " m")
        self.obstacle_min_z = self._double_spin(-1.0, 3.0, -0.18, 0.01, " m")
        self.obstacle_max_z = self._double_spin(0.0, 5.0, 1.80, 0.05, " m")
        self.inflation_radius = self._double_spin(0.0, 1.0, 0.00, 0.05, " m")
        self.obstacle_min_hits = self._int_spin(1, 100, 2)
        self.free_min_hits = self._int_spin(1, 100, 3)
        self.sor_mean_k = self._int_spin(0, 200, 16)
        self.sor_stddev = self._double_spin(0.1, 10.0, 1.5, 0.1, "", 1)
        self.min_obstacle_cluster = self._int_spin(0, 1000, 3)
        self.pose_crop_margin = self._double_spin(0.0, 20.0, 2.5, 0.1, " m", 1)
        self.raycast_max_range = self._double_spin(0.0, 100.0, 4.5, 0.1, " m", 1)
        self.map_parameter_controls = [
            self.map_resolution,
            self.projection_min_z,
            self.projection_max_z,
            self.obstacle_min_z,
            self.obstacle_max_z,
            self.inflation_radius,
            self.obstacle_min_hits,
            self.free_min_hits,
            self.sor_mean_k,
            self.sor_stddev,
            self.min_obstacle_cluster,
            self.pose_crop_margin,
            self.raycast_max_range,
        ]
        self.map_parameters_unlocked = QCheckBox("解锁参数编辑")
        self.map_parameters_unlocked.setChecked(False)
        self.map_parameters_unlocked.toggled.connect(self._set_map_parameters_enabled)
        convert_layout.addRow(self.map_parameters_unlocked)
        convert_layout.addRow("分辨率", self.map_resolution)
        convert_layout.addRow("投影高度下限", self.projection_min_z)
        convert_layout.addRow("投影高度上限", self.projection_max_z)
        convert_layout.addRow("障碍高度下限", self.obstacle_min_z)
        convert_layout.addRow("障碍高度上限", self.obstacle_max_z)
        convert_layout.addRow("地图膨胀", self.inflation_radius)
        convert_layout.addRow("障碍最少命中", self.obstacle_min_hits)
        convert_layout.addRow("空闲最少命中", self.free_min_hits)
        convert_layout.addRow("SOR 邻居数", self.sor_mean_k)
        convert_layout.addRow("SOR 标准差倍数", self.sor_stddev)
        convert_layout.addRow("最小障碍簇", self.min_obstacle_cluster)
        convert_layout.addRow("位姿裁剪边界", self.pose_crop_margin)
        convert_layout.addRow("射线最大距离", self.raycast_max_range)
        self._set_map_parameters_enabled(False)
        layout.addWidget(convert_group)

        teleop_group = QGroupBox("手动控制")
        teleop_layout = QVBoxLayout(teleop_group)
        self.teleop_enabled = QCheckBox("启用遥控")
        self.teleop_enabled.toggled.connect(self._teleop_enabled_changed)
        teleop_layout.addWidget(self.teleop_enabled)
        button_grid = QGridLayout()
        self.teleop_buttons = []
        up = self._motion_button(QStyle.SP_ArrowUp, "前进", (1.0, 0.0, 0.0))
        down = self._motion_button(QStyle.SP_ArrowDown, "后退", (-1.0, 0.0, 0.0))
        left = self._motion_button(QStyle.SP_ArrowLeft, "左移", (0.0, 1.0, 0.0))
        right = self._motion_button(QStyle.SP_ArrowRight, "右移", (0.0, -1.0, 0.0))
        rotate_left = self._text_motion_button("左转", (0.0, 0.0, 1.0))
        rotate_right = self._text_motion_button("右转", (0.0, 0.0, -1.0))
        stop = QPushButton("STOP")
        stop.setObjectName("dangerButton")
        stop.clicked.connect(self._stop_motion)
        button_grid.addWidget(up, 0, 1)
        button_grid.addWidget(rotate_left, 1, 0)
        button_grid.addWidget(stop, 1, 1)
        button_grid.addWidget(rotate_right, 1, 2)
        button_grid.addWidget(left, 2, 0)
        button_grid.addWidget(down, 2, 1)
        button_grid.addWidget(right, 2, 2)
        teleop_layout.addLayout(button_grid)
        speed_form = QFormLayout()
        self.teleop_linear = self._double_spin(0.05, 0.5, 0.20, 0.05, " m/s")
        self.teleop_angular = self._double_spin(0.05, 0.8, 0.30, 0.05, " rad/s")
        speed_form.addRow("线速度", self.teleop_linear)
        speed_form.addRow("角速度", self.teleop_angular)
        teleop_layout.addLayout(speed_form)
        layout.addWidget(teleop_group)
        layout.addStretch(1)
        return self._scroll_page(tab)

    def _build_navigation_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        pose_group = QGroupBox("初始位姿")
        pose_form = QFormLayout(pose_group)
        self.initial_x = self._double_spin(-100.0, 100.0, 0.0, 0.1, " m", 3)
        self.initial_y = self._double_spin(-100.0, 100.0, 0.0, 0.1, " m", 3)
        self.initial_yaw = self._double_spin(-math.pi, math.pi, 0.0, 0.05, " rad", 3)
        pose_form.addRow("X", self.initial_x)
        pose_form.addRow("Y", self.initial_y)
        pose_form.addRow("Yaw", self.initial_yaw)
        self.publish_initial_button = QPushButton("发送初始位姿")
        self.publish_initial_button.clicked.connect(self._publish_initial_pose)
        pose_form.addRow(self.publish_initial_button)
        layout.addWidget(pose_group)

        navigation_group = QGroupBox("导航系统")
        navigation_layout = QGridLayout(navigation_group)
        self.nav_speed = self._double_spin(0.1, 2.0, 0.5, 0.1, " m/s")
        self.nav_accel = self._double_spin(0.1, 2.0, 0.4, 0.1, " m/s2")
        self.nav_rviz = QCheckBox("RViz")
        self.nav_rviz.setChecked(True)
        self.auto_record = QCheckBox("自动录制")
        self.auto_record.setChecked(True)
        self.start_nav_button = QPushButton("启动定位导航")
        self.start_nav_button.setObjectName("primaryButton")
        self.start_nav_button.setIcon(self.style().standardIcon(QStyle.SP_MediaPlay))
        self.start_nav_button.clicked.connect(self._start_navigation)
        self.stop_nav_button = QPushButton("停止导航")
        self.stop_nav_button.setIcon(self.style().standardIcon(QStyle.SP_MediaStop))
        self.stop_nav_button.clicked.connect(self._stop_navigation)
        navigation_layout.addWidget(QLabel("轮速上限"), 0, 0)
        navigation_layout.addWidget(self.nav_speed, 0, 1)
        navigation_layout.addWidget(QLabel("加速度"), 1, 0)
        navigation_layout.addWidget(self.nav_accel, 1, 1)
        navigation_layout.addWidget(self.nav_rviz, 2, 0)
        navigation_layout.addWidget(self.auto_record, 2, 1)
        navigation_layout.addWidget(self.start_nav_button, 3, 0)
        navigation_layout.addWidget(self.stop_nav_button, 3, 1)
        layout.addWidget(navigation_group)

        goal_group = QGroupBox("坐标目标")
        goal_form = QFormLayout(goal_group)
        self.goal_x = self._double_spin(-100.0, 100.0, 0.0, 0.1, " m", 3)
        self.goal_y = self._double_spin(-100.0, 100.0, 0.0, 0.1, " m", 3)
        self.goal_yaw = self._double_spin(-math.pi, math.pi, 0.0, 0.05, " rad", 3)
        goal_form.addRow("X", self.goal_x)
        goal_form.addRow("Y", self.goal_y)
        goal_form.addRow("Yaw", self.goal_yaw)
        goal_buttons = QHBoxLayout()
        send_goal = QPushButton("发送目标")
        send_goal.setObjectName("primaryButton")
        send_goal.clicked.connect(self._send_goal)
        cancel_goal = QPushButton("取消目标")
        cancel_goal.clicked.connect(self._cancel_goal)
        goal_buttons.addWidget(send_goal)
        goal_buttons.addWidget(cancel_goal)
        goal_form.addRow(goal_buttons)
        layout.addWidget(goal_group)

        record_group = QGroupBox("诊断数据")
        record_layout = QGridLayout(record_group)
        self.start_record_button = QPushButton("开始录制")
        self.start_record_button.setIcon(self.style().standardIcon(QStyle.SP_DialogSaveButton))
        self.start_record_button.clicked.connect(self._start_recording)
        self.stop_record_button = QPushButton("停止录制")
        self.stop_record_button.clicked.connect(lambda: self._stop_process("recording"))
        open_records = QPushButton("打开数据目录")
        open_records.setIcon(self.style().standardIcon(QStyle.SP_DirOpenIcon))
        open_records.clicked.connect(self._open_record_root)
        record_layout.addWidget(self.start_record_button, 0, 0)
        record_layout.addWidget(self.stop_record_button, 0, 1)
        record_layout.addWidget(open_records, 1, 0, 1, 2)
        layout.addWidget(record_group)
        layout.addStretch(1)
        return self._scroll_page(tab)

    def _build_system_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)
        group = QGroupBox("运行检查")
        grid = QGridLayout(group)
        checks = (
            ("can4", "驱动 CAN"),
            ("can5", "转向 CAN"),
            ("lidar_config", "雷达配置"),
            ("pcd", "3D 地图"),
            ("yaml", "2D 地图"),
            ("scan", "激光扫描"),
            ("odom", "安全里程计"),
            ("tf", "地图定位 TF"),
        )
        self.check_widgets = {}
        for row, (key, title) in enumerate(checks):
            dot = StatusDot()
            value = QLabel("--")
            grid.addWidget(dot, row, 0)
            grid.addWidget(QLabel(title), row, 1)
            grid.addWidget(value, row, 2)
            self.check_widgets[key] = (dot, value)
        refresh = QPushButton("立即检查")
        refresh.setIcon(self.style().standardIcon(QStyle.SP_BrowserReload))
        refresh.clicked.connect(self._update_preflight)
        grid.addWidget(refresh, len(checks), 0, 1, 3)
        layout.addWidget(group)

        active_group = QGroupBox("ROS 节点")
        active_layout = QVBoxLayout(active_group)
        self.active_nodes_label = QLabel("未连接")
        self.active_nodes_label.setWordWrap(True)
        active_layout.addWidget(self.active_nodes_label)
        layout.addWidget(active_group)
        layout.addStretch(1)
        return self._scroll_page(tab)

    @staticmethod
    def _scroll_page(content: QWidget) -> QScrollArea:
        content.setMinimumHeight(content.sizeHint().height())
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setWidget(content)
        return scroll

    def _build_wheel_table(self) -> QWidget:
        self.wheel_table = QTableWidget(4, 7)
        self.wheel_table.setHorizontalHeaderLabels(
            ["轮", "请求角", "实际角", "角误差", "目标速度", "实际速度", "转向比例"]
        )
        self.wheel_table.verticalHeader().setVisible(False)
        self.wheel_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.wheel_table.setSelectionMode(QTableWidget.NoSelection)
        header = self.wheel_table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.Stretch)
        return self.wheel_table

    def _double_spin(
        self,
        minimum: float,
        maximum: float,
        value: float,
        step: float,
        suffix: str,
        decimals: int = 2,
    ) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(minimum, maximum)
        spin.setValue(value)
        spin.setSingleStep(step)
        spin.setDecimals(decimals)
        spin.setSuffix(suffix)
        return spin

    def _int_spin(self, minimum: int, maximum: int, value: int) -> QSpinBox:
        spin = QSpinBox()
        spin.setRange(minimum, maximum)
        spin.setValue(value)
        return spin

    def _set_map_parameters_enabled(self, enabled: bool) -> None:
        for control in self.map_parameter_controls:
            control.setEnabled(enabled)

    def _motion_button(self, icon, tooltip: str, command) -> QPushButton:
        button = QPushButton()
        button.setIcon(self.style().standardIcon(icon))
        button.setIconSize(QSize(24, 24))
        button.setMinimumHeight(42)
        button.setToolTip(tooltip)
        button.pressed.connect(lambda: self._set_motion(command))
        button.released.connect(self._stop_motion)
        self.teleop_buttons.append(button)
        return button

    def _text_motion_button(self, text: str, command) -> QPushButton:
        button = QPushButton(text)
        button.setMinimumHeight(42)
        button.pressed.connect(lambda: self._set_motion(command))
        button.released.connect(self._stop_motion)
        self.teleop_buttons.append(button)
        return button

    def _apply_style(self) -> None:
        self.setStyleSheet(
            """
            QMainWindow, QWidget { background:#202020; color:#e8e8e8; font-size:13px; }
            QLabel#title { font-size:20px; font-weight:700; color:#ffffff; }
            QGroupBox {
              border:1px solid #454545; border-radius:6px; margin-top:10px;
              padding-top:8px; font-weight:600;
            }
            QGroupBox::title { subcontrol-origin:margin; left:10px; padding:0 5px; color:#dcdcdc; }
            QPushButton {
              background:#383838; border:1px solid #555; border-radius:5px;
              padding:7px 10px; min-height:22px;
            }
            QPushButton:hover { background:#454545; border-color:#6c6c6c; }
            QPushButton:pressed { background:#2c2c2c; }
            QPushButton:disabled { color:#777; background:#2a2a2a; }
            QPushButton#primaryButton {
              background:#176b87; border-color:#2b91b2; color:white; font-weight:700;
            }
            QPushButton#primaryButton:hover { background:#1d7f9f; }
            QPushButton#dangerButton {
              background:#8f3030; border-color:#bd4747; color:white; font-weight:700;
            }
            QLineEdit, QComboBox, QDoubleSpinBox, QPlainTextEdit, QTableWidget {
              background:#171717; border:1px solid #484848; border-radius:4px; padding:4px;
            }
            QTabWidget::pane { border:1px solid #454545; }
            QTabBar::tab { background:#2b2b2b; padding:8px 14px; border:1px solid #454545; }
            QTabBar::tab:selected { background:#176b87; color:white; }
            QFrame#metric { background:#292929; border:1px solid #444; border-radius:6px; }
            QLabel#metricTitle { color:#a9a9a9; font-size:11px; }
            QLabel#metricValue { color:#f0f0f0; font-size:15px; font-weight:700; }
            QHeaderView::section {
              background:#303030; color:#ddd; padding:5px;
              border:0; border-right:1px solid #444;
            }
            QStatusBar { background:#181818; color:#bdbdbd; }
            """
        )

    def _load_settings(self) -> None:
        root = self.settings.value("map_root", str(Path(DEFAULT_MAP_ROOT).expanduser()))
        self.map_root_edit.setCurrentText(str(root))
        if self.map_root_edit.lineEdit() is not None:
            self.map_root_edit.lineEdit().setCursorPosition(0)
        self.map_name_combo.setEditText(str(self.settings.value("map_name", "supermarket_test_01")))
        self.initial_x.setValue(float(self.settings.value("initial_x", 0.0)))
        self.initial_y.setValue(float(self.settings.value("initial_y", 0.0)))
        self.initial_yaw.setValue(float(self.settings.value("initial_yaw", 0.0)))
        auto_record = str(self.settings.value("auto_record", "true")).lower() == "true"
        self.auto_record.setChecked(auto_record)

    def _save_settings(self) -> None:
        self.settings.setValue("map_root", self.map_root_edit.currentText())
        self.settings.setValue("map_name", self.map_name_combo.currentText())
        self.settings.setValue("initial_x", self.initial_x.value())
        self.settings.setValue("initial_y", self.initial_y.value())
        self.settings.setValue("initial_yaw", self.initial_yaw.value())
        self.settings.setValue("auto_record", self.auto_record.isChecked())

    def _choose_map_root(self) -> None:
        selected = QFileDialog.getExistingDirectory(
            self,
            "选择地图根目录",
            self.map_root_edit.currentText(),
        )
        if selected:
            self.map_root_edit.setCurrentText(selected)

    def _map_paths(self):
        return resolve_map_paths(self.map_root_edit.currentText(), self.map_name_combo.currentText())

    def _refresh_maps(self) -> None:
        current = self.map_name_combo.currentText().strip() if hasattr(self, "map_name_combo") else ""
        if not hasattr(self, "map_name_combo"):
            return
        names = discover_map_names(self.map_root_edit.currentText())
        self.map_name_combo.blockSignals(True)
        self.map_name_combo.clear()
        self.map_name_combo.addItems(names)
        self.map_name_combo.setEditText(current or (names[0] if names else "supermarket_test_01"))
        if self.map_name_combo.lineEdit() is not None:
            self.map_name_combo.lineEdit().setCursorPosition(0)
        self.map_name_combo.blockSignals(False)
        self._update_map_file_status()

    def _update_map_file_status(self) -> None:
        try:
            paths = self._map_paths()
        except ValueError:
            self.pcd_status.setText("PCD 无效")
            self.yaml_status.setText("2D 无效")
            self.path_plot.clear_map()
            return
        self._set_file_label(self.pcd_status, "PCD", paths.pcd.is_file())
        map_ready = paths.yaml.is_file() and paths.pgm.is_file()
        self._set_file_label(self.yaml_status, "2D", map_ready)
        if not map_ready:
            self.path_plot.clear_map()
            return
        try:
            self.path_plot.set_map(
                paths.pgm,
                paths.yaml,
                self._active_goal_safety_geometry(),
            )
        except Exception as exc:
            self.path_plot.clear_map()
            self._append_log(f"二维地图加载失败: {exc}")

    def _active_goal_safety_geometry(self) -> GoalSafetyGeometry:
        return getattr(self, "_goal_safety_geometry", DEFAULT_GOAL_SAFETY_GEOMETRY)

    def _goal_pose_rejected(self, reason: str) -> None:
        self.statusBar().showMessage(reason, 5000)
        self._append_log(f"拒绝地图目标: {reason}")

    @staticmethod
    def _set_file_label(label: QLabel, name: str, exists: bool) -> None:
        label.setText(f"{name} {'就绪' if exists else '缺失'}")
        label.setStyleSheet(f"color:{'#5fd38d' if exists else '#f1c75b'};")

    def _start_mapping(self) -> None:
        start_robot_hardware = self._mapping_starts_robot_hardware()
        if start_robot_hardware and self._has_external_chassis_stack():
            self._warn_external_stack()
            return
        try:
            paths = self._map_paths()
        except ValueError as exc:
            QMessageBox.warning(self, "地图名称", str(exc))
            return
        paths.map_3d_dir.mkdir(parents=True, exist_ok=True)
        args = [
            "launch",
            "swerve_navigation",
            "mapping.launch.py",
            f"use_rviz:={'true' if self.mapping_rviz.isChecked() else 'false'}",
            "use_camera:=false",
            "use_gps:=false",
            f"start_robot_hardware:={'true' if start_robot_hardware else 'false'}",
            f"mapping_max_wheel_speed:={self.mapping_speed.value():.3f}",
            f"mapping_wheel_accel_limit:={self.mapping_accel.value():.3f}",
        ]
        self._start_process("mapping", "建图", "ros2", args)

    def _save_3d_map(self) -> None:
        self._begin_save_map(after_action=None)

    def _save_and_convert(self) -> None:
        self._begin_save_map(after_action="convert")

    def _open_map_editor(self) -> None:
        if not self._map_edit_allowed():
            QMessageBox.warning(
                self,
                "请先停止导航",
                "导航运行期间不允许改写正在使用的静态地图。请停止导航后再编辑。",
            )
            return
        try:
            paths = self._map_paths()
        except ValueError as exc:
            QMessageBox.warning(self, "地图名称", str(exc))
            return
        if not paths.pgm.is_file() or not paths.yaml.is_file():
            QMessageBox.warning(self, "地图不完整", f"缺少 {paths.pgm} 或 {paths.yaml}")
            return
        try:
            dialog = MapEditorDialog(
                paths.pgm,
                paths.yaml,
                self,
                save_allowed=self._map_edit_allowed,
            )
        except Exception as exc:
            QMessageBox.critical(self, "打开地图失败", str(exc))
            return
        dialog.exec_()
        if dialog.saved:
            self._append_log(f"已编辑 2D 地图: {paths.pgm}")
            if dialog.backup_path:
                self._append_log(f"原图备份: {dialog.backup_path}")
            self._update_map_file_status()

    def _map_edit_allowed(self) -> bool:
        return not self._process_running("navigation") and not any(
            name in self.telemetry.node_names
            for name in ("/map_server", "/planner_server", "/controller_server", "/bt_navigator")
        )

    def _begin_save_map(self, after_action: Optional[str]) -> None:
        if not self.ros_node:
            return
        if self.save_future is not None:
            QMessageBox.information(self, "正在保存", "地图保存请求尚未完成")
            return
        if not self.ros_node.save_maps_client.service_is_ready():
            QMessageBox.warning(self, "PGO 未就绪", "没有发现 /pgo/save_maps，请先启动建图")
            return
        try:
            paths = self._map_paths()
        except ValueError as exc:
            QMessageBox.warning(self, "地图名称", str(exc))
            return
        paths.map_3d_dir.mkdir(parents=True, exist_ok=True)
        self.after_save_action = after_action
        self.save_future = self.ros_node.request_save_map(paths.map_3d_dir)
        self.save_3d_button.setEnabled(False)
        self.finish_map_button.setEnabled(False)
        self._append_log(f"保存 3D 地图到 {paths.map_3d_dir}")

    def _handle_save_future(self) -> None:
        if self.save_future is None or not self.save_future.done():
            return
        future = self.save_future
        self.save_future = None
        self.save_3d_button.setEnabled(True)
        self.finish_map_button.setEnabled(True)
        try:
            response = future.result()
        except Exception as exc:
            self._append_log(f"3D 地图保存失败: {exc}")
            QMessageBox.critical(self, "保存失败", str(exc))
            self.after_save_action = None
            return
        self._append_log(f"PGO: {response.message}")
        if not response.success:
            QMessageBox.critical(self, "保存失败", response.message)
            self.after_save_action = None
            return
        self._refresh_maps()
        if self.after_save_action == "convert":
            self.after_save_action = None
            self._stop_process("mapping")
            QTimer.singleShot(1200, self._start_conversion)
        else:
            QMessageBox.information(self, "保存完成", response.message)

    def _start_conversion(self) -> None:
        try:
            paths = self._map_paths()
        except ValueError as exc:
            QMessageBox.warning(self, "地图名称", str(exc))
            return
        if not paths.pcd.is_file():
            QMessageBox.warning(self, "缺少 PCD", str(paths.pcd))
            return
        paths.map_2d_dir.mkdir(parents=True, exist_ok=True)
        args = [
            "run",
            "pgo",
            "pcd_to_nav2_map",
            "--pcd",
            str(paths.pcd),
            "--out-dir",
            str(paths.map_2d_dir),
            "--resolution",
            f"{self.map_resolution.value():.3f}",
            "--z-min",
            f"{self.projection_min_z.value():.3f}",
            "--z-max",
            f"{self.projection_max_z.value():.3f}",
            "--obstacle-z-min",
            f"{self.obstacle_min_z.value():.3f}",
            "--obstacle-z-max",
            f"{self.obstacle_max_z.value():.3f}",
            "--inflation-radius",
            f"{self.inflation_radius.value():.3f}",
            "--obstacle-min-hits",
            str(self.obstacle_min_hits.value()),
            "--free-min-hits",
            str(self.free_min_hits.value()),
            "--sor-mean-k",
            str(self.sor_mean_k.value()),
            "--sor-stddev",
            f"{self.sor_stddev.value():.2f}",
            "--min-obstacle-cluster",
            str(self.min_obstacle_cluster.value()),
            "--raycast-max-range",
            f"{self.raycast_max_range.value():.2f}",
        ]
        if paths.poses.is_file():
            args.extend(
                [
                    "--poses",
                    str(paths.poses),
                    "--crop-to-poses-margin",
                    f"{self.pose_crop_margin.value():.2f}",
                ]
            )
        self._start_process("conversion", "地图转换", "ros2", args)

    def _start_navigation(self) -> None:
        if self._has_external_chassis_stack():
            self._warn_external_stack()
            return
        try:
            paths = self._map_paths()
        except ValueError as exc:
            QMessageBox.warning(self, "地图名称", str(exc))
            return
        missing = [str(path) for path in (paths.pcd, paths.yaml, paths.pgm) if not path.is_file()]
        if missing:
            QMessageBox.warning(self, "地图不完整", "\n".join(missing))
            return
        if self.auto_record.isChecked():
            self._start_recording()
        pose = (
            f"[{self.initial_x.value():.6f},{self.initial_y.value():.6f},0.0,"
            f"0.0,0.0,{self.initial_yaw.value():.6f}]"
        )
        args = [
            "launch",
            "swerve_navigation",
            "full_system.launch.py",
            f"pcd_path:={paths.pcd}",
            f"map_yaml:={paths.yaml}",
            f"initial_pose:={pose}",
            f"use_rviz:={'true' if self.nav_rviz.isChecked() else 'false'}",
            f"nav_controller_max_wheel_speed:={self.nav_speed.value():.3f}",
            f"nav_controller_wheel_accel_limit:={self.nav_accel.value():.3f}",
        ]
        if self._start_process("navigation", "定位导航", "ros2", args):
            QTimer.singleShot(8000, self._dump_metadata_async)

    def _stop_navigation(self) -> None:
        self._cancel_goal()
        self._stop_motion()
        self._stop_process("navigation")
        if self.auto_record.isChecked():
            QTimer.singleShot(800, lambda: self._stop_process("recording"))

    def _publish_initial_pose(self) -> None:
        if not self.ros_node:
            return
        self.ros_node.publish_initial_pose(
            self.initial_x.value(), self.initial_y.value(), self.initial_yaw.value()
        )
        self._append_log("已发布 /initialpose")

    def _send_goal(self) -> None:
        if not self.ros_node:
            return
        state = self.ros_node.telemetry
        if not state.localization_state_seen or not state.localization_ok:
            QMessageBox.warning(
                self,
                "定位未就绪",
                "尚未收到稳定的 /localization_ok=true，已禁止发送导航目标。",
            )
            return
        if not self.ros_node.send_navigation_goal(
            self.goal_x.value(), self.goal_y.value(), self.goal_yaw.value()
        ):
            QMessageBox.warning(self, "Nav2 未就绪", "没有发现 /navigate_to_pose Action")
            return
        self._append_log(
            f"发送目标: x={self.goal_x.value():.3f}, y={self.goal_y.value():.3f}, "
            f"yaw={self.goal_yaw.value():.3f}"
        )

    def _cancel_goal(self) -> None:
        if self.ros_node:
            self.ros_node.cancel_navigation_goal()

    def _plot_goal_selected(self, x: float, y: float) -> None:
        self.goal_x.setValue(x)
        self.goal_y.setValue(y)
        self.control_tabs.setCurrentIndex(1)

    def _plot_pose_selected(self, x: float, y: float, yaw: float) -> None:
        self.goal_x.setValue(x)
        self.goal_y.setValue(y)
        self.goal_yaw.setValue(yaw)
        self.control_tabs.setCurrentIndex(1)

    def _start_recording(self) -> None:
        if self._process_running("recording"):
            return
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        root = self._record_root()
        output = root / f"nav_{stamp}"
        bag = output / "bag"
        output.mkdir(parents=True, exist_ok=True)
        self.current_record_dir = output
        (output / "recorded_at.txt").write_text(
            datetime.now().astimezone().isoformat() + "\n", encoding="utf-8"
        )
        args = [
            "bag",
            "record",
            "--include-hidden-topics",
            "--max-cache-size",
            "52428800",
            "-o",
            str(bag),
            *RECORD_TOPICS,
        ]
        if self._start_process("recording", "诊断录制", "ros2", args):
            self.record_dot.set_state("running")
            self.record_label.setText(output.name)
            self._dump_metadata_async()

    def _dump_metadata_async(self) -> None:
        if self.current_record_dir is None:
            return
        output = self.current_record_dir
        manifest = {
            "map_root": self.map_root_edit.currentText(),
            "map_name": self.map_name_combo.currentText(),
            "initial_pose": [
                self.initial_x.value(),
                self.initial_y.value(),
                self.initial_yaw.value(),
            ],
            "record_topics": list(RECORD_TOPICS),
        }

        def worker() -> None:
            params_dir = output / "params"
            params_dir.mkdir(parents=True, exist_ok=True)
            commands = {
                "topics.txt": ["ros2", "topic", "list", "-t"],
                "nodes.txt": ["ros2", "node", "list"],
                "actions.txt": ["ros2", "action", "list", "-t"],
            }
            for filename, command in commands.items():
                result = subprocess.run(command, capture_output=True, text=True, timeout=15)
                (output / filename).write_text(
                    result.stdout + result.stderr, encoding="utf-8"
                )
            nodes = (
                "/controller_server",
                "/swerve_drive_controller",
                "/icp_registration",
                "/safe_odom_mux",
                "/cmd_vel_safety_gate",
                "/localization_costmap_guard",
                "/fastlio2/lio_bridge",
                "/collision_monitor",
                "/local_costmap/local_costmap",
                "/global_costmap/global_costmap",
                "/bt_navigator",
            )
            for node in nodes:
                result = subprocess.run(
                    ["ros2", "param", "dump", node],
                    capture_output=True,
                    text=True,
                    timeout=15,
                )
                filename = node.strip("/").replace("/", "_") + ".yaml"
                (params_dir / filename).write_text(
                    result.stdout + result.stderr, encoding="utf-8"
                )
            (output / "manifest.json").write_text(
                json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
                encoding="utf-8",
            )
            self.metadata_done_signal.emit(f"诊断参数已保存到 {output}")

        threading.Thread(target=worker, daemon=True).start()

    def _finalize_recording(self) -> None:
        if not self.current_record_dir:
            return
        output = self.current_record_dir

        def worker() -> None:
            bag = output / "bag"
            if bag.is_dir():
                result = subprocess.run(
                    ["ros2", "bag", "info", str(bag)],
                    capture_output=True,
                    text=True,
                    timeout=30,
                )
                (output / "bag_info.txt").write_text(
                    result.stdout + result.stderr, encoding="utf-8"
                )
            self.metadata_done_signal.emit(f"录制完成: {output}")

        threading.Thread(target=worker, daemon=True).start()

    def _open_record_root(self) -> None:
        root = self._record_root()
        root.mkdir(parents=True, exist_ok=True)
        QDesktopServices.openUrl(QUrl.fromLocalFile(str(root)))

    @staticmethod
    def _record_root() -> Path:
        return Path(os.environ.get("NAV_RECORD_ROOT", "~/openflex_nav_records")).expanduser()

    def _set_motion(self, normalized_command) -> None:
        if not self.teleop_enabled.isChecked() or not self._mapping_available():
            return
        vx, vy, wz = normalized_command
        self.teleop_command = (
            vx * self.teleop_linear.value(),
            vy * self.teleop_linear.value(),
            wz * self.teleop_angular.value(),
        )

    def _stop_motion(self) -> None:
        self.teleop_command = (0.0, 0.0, 0.0)
        if self.ros_node:
            if self._mapping_uses_external_hardware():
                self.ros_node.publish_mapping_teleop(0.0, 0.0, 0.0)
            else:
                self.ros_node.publish_teleop(0.0, 0.0, 0.0)

    def _publish_teleop(self) -> None:
        if not self.ros_node:
            return
        if self.teleop_enabled.isChecked() and self._mapping_available():
            if self._mapping_uses_external_hardware():
                self.ros_node.publish_mapping_teleop(*self.teleop_command)
            else:
                self.ros_node.publish_teleop(*self.teleop_command)

    def _mapping_starts_robot_hardware(self) -> bool:
        return True

    def _mapping_uses_external_hardware(self) -> bool:
        return False

    def _teleop_enabled_changed(self, enabled: bool) -> None:
        if not enabled:
            self._stop_motion()

    def _mapping_available(self) -> bool:
        return self._process_running("mapping") or any(
            name.startswith("/pgo/") or name == "/pgo" for name in self.telemetry.node_names
        )

    def keyPressEvent(self, event) -> None:
        if event.isAutoRepeat() or not self.teleop_enabled.isChecked():
            super().keyPressEvent(event)
            return
        mapping = {
            Qt.Key_W: (1.0, 0.0, 0.0),
            Qt.Key_S: (-1.0, 0.0, 0.0),
            Qt.Key_A: (0.0, 1.0, 0.0),
            Qt.Key_D: (0.0, -1.0, 0.0),
            Qt.Key_Q: (0.0, 0.0, 1.0),
            Qt.Key_E: (0.0, 0.0, -1.0),
        }
        if event.key() in mapping:
            self.teleop_active_key = event.key()
            self._set_motion(mapping[event.key()])
            event.accept()
            return
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event) -> None:
        if not event.isAutoRepeat() and event.key() == self.teleop_active_key:
            self.teleop_active_key = None
            self._stop_motion()
            event.accept()
            return
        super().keyReleaseEvent(event)

    def _start_process(self, key: str, label: str, program: str, args: list[str]) -> bool:
        if self._process_running(key):
            self._append_log(f"{label} 已在运行")
            return False
        process = QProcess(self)
        process.setProcessChannelMode(QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(lambda key=key: self._read_process_output(key))
        process.started.connect(lambda key=key: self._process_started(key))
        process.finished.connect(
            lambda code, status, key=key: self._process_finished(key, code, status)
        )
        self.processes[key] = process
        self.process_labels[key] = label
        self.process_output_tail[key] = []
        self.process_stop_requested.discard(key)
        self._append_log(f"$ {command_text(program, args)}")
        process.start("setsid", [program, *args])
        if not process.waitForStarted(3000):
            self._append_log(f"{label} 启动失败: {process.errorString()}")
            return False
        # setsid makes the launched program the leader of a dedicated process
        # group.  Keep the PGID even if the launch parent exits before one of
        # its ROS children, so Stop can still clean up the whole owned stack.
        self.process_group_ids[key] = int(process.processId())
        return True

    def _read_process_output(self, key: str) -> None:
        process = self.processes.get(key)
        if not process:
            return
        data = bytes(process.readAllStandardOutput()).decode("utf-8", errors="replace")
        if data:
            for line in data.rstrip().splitlines():
                self._append_log(f"[{self.process_labels.get(key, key)}] {line}")
                tail = self.process_output_tail.setdefault(key, [])
                tail.append(line)
                del tail[:-12]

    def _process_started(self, key: str) -> None:
        self._append_log(f"{self.process_labels.get(key, key)} 已启动")
        self._update_mode()

    def _process_finished(self, key: str, code: int, _status) -> None:
        label = self.process_labels.get(key, key)
        self._append_log(f"{label} 已退出，代码 {code}")
        stop_requested = key in self.process_stop_requested
        self.process_stop_requested.discard(key)
        if key in ("mapping", "navigation", "fullbody") and code != 0 and not stop_requested:
            detail = "\n".join(self.process_output_tail.get(key, [])[-8:])
            QMessageBox.critical(
                self,
                f"{label}启动失败",
                f"进程已退出，代码 {code}。\n\n"
                f"{detail or '请查看窗口下方的运行日志。'}",
            )
        if key == "conversion":
            self._refresh_maps()
            if code == 0:
                try:
                    paths = self._map_paths()
                    conversion_ready = paths.yaml.is_file() and paths.pgm.is_file()
                except ValueError:
                    conversion_ready = False
                if conversion_ready:
                    answer = QMessageBox.question(
                        self,
                        "地图完成",
                        "2D 导航地图已经生成。是否立即检查并编辑栅格地图？",
                        QMessageBox.Yes | QMessageBox.No,
                        QMessageBox.Yes,
                    )
                    if answer == QMessageBox.Yes:
                        self._open_map_editor()
                else:
                    QMessageBox.critical(
                        self,
                        "转换失败",
                        "转换进程已退出，但缺少 map.yaml 或 map.pgm",
                    )
        if key == "recording":
            self.record_dot.set_state("idle" if code == 0 else "warning")
            self.record_label.setText("未录制")
            self._finalize_recording()
        if key == "mapping" and not self._mapping_uses_external_hardware():
            self.teleop_enabled.setChecked(False)
        if key == "navigation" and self.auto_record.isChecked():
            QTimer.singleShot(500, lambda: self._stop_process("recording"))
        if key == "navigation" and stop_requested:
            # A launch parent can exit before all ROS children have completed
            # their shutdown handlers.  Verify the graph shortly afterwards so
            # the competition console does not silently report a partial stop.
            QTimer.singleShot(10500, self._verify_competition_shutdown)
        self._update_mode()

    def _verify_competition_shutdown(self) -> None:
        if self._process_running("navigation"):
            return
        expected_external = self._process_running("fullbody")
        remaining_candidates = [
            "/planner_server",
            "/controller_server",
            "/bt_navigator",
            "/collision_monitor",
        ]
        if not expected_external:
            remaining_candidates.extend(
                [
                    "/controller_manager",
                    "/move_group",
                    "/openarmx_task_executor",
                    "/station_docking",
                ]
            )
        remaining = sorted(
            name
            for name in remaining_candidates
            if name in self.telemetry.node_names
        )
        actions = sorted(
            name
            for name in (
                "/navigate_to_pose",
                "/execute_transport",
                "/dock_to_station",
            )
            if name in self.telemetry.action_servers
        )
        if remaining or actions:
            self._append_log(
                "停止后仍检测到 ROS 工作流: "
                + ", ".join([*remaining, *actions])
            )

    def _stop_process(self, key: str) -> None:
        process = self.processes.get(key)
        process_group_id = self.process_group_ids.get(key)
        if not process or process_group_id is None:
            return
        if process.state() == QProcess.NotRunning:
            try:
                os.killpg(process_group_id, 0)
            except (ProcessLookupError, PermissionError):
                return
        label = self.process_labels.get(key, key)
        self.process_stop_requested.add(key)
        self._append_log(f"正在停止 {label}")
        try:
            os.killpg(process_group_id, signal.SIGINT)
        except (ProcessLookupError, PermissionError):
            if process.state() != QProcess.NotRunning:
                process.terminate()

        def signal_remaining(sig: int, description: str) -> None:
            try:
                os.killpg(process_group_id, 0)
            except (ProcessLookupError, PermissionError):
                return
            self._append_log(f"{label} 仍有子进程，正在{description}")
            try:
                os.killpg(process_group_id, sig)
            except (ProcessLookupError, PermissionError):
                return

        # Do not use QProcess.state() here.  The launch parent may already be
        # gone while a Nav2 or driver child remains in the owned process group.
        QTimer.singleShot(
            6000,
            lambda: signal_remaining(signal.SIGTERM, "强制结束"),
        )
        QTimer.singleShot(
            9000,
            lambda: signal_remaining(signal.SIGKILL, "最终清理"),
        )

    def _process_running(self, key: str) -> bool:
        process = self.processes.get(key)
        if process and process.state() != QProcess.NotRunning:
            return True
        process_group_id = self.process_group_ids.get(key)
        if process_group_id is None:
            return False
        try:
            os.killpg(process_group_id, 0)
        except (ProcessLookupError, PermissionError):
            return False
        return True

    def _has_external_chassis_stack(self) -> bool:
        if self._process_running("mapping") or self._process_running("navigation"):
            return False
        return "/controller_manager" in self.telemetry.node_names

    def _warn_external_stack(self) -> None:
        QMessageBox.warning(
            self,
            "检测到已有系统",
            "当前 ROS 图中已有 /controller_manager。请先在原启动终端停止现有建图或导航，"
            "避免重复启动硬件控制。",
        )

    def _poll_ros_futures(self) -> None:
        self._handle_save_future()

    def _sample_graph(self) -> None:
        if self.ros_node:
            try:
                self.ros_node.sample_graph_and_tf()
            except Exception as exc:
                self._append_log(f"状态采样失败: {exc}")
        self._update_preflight()
        self._update_mode()

    def _update_telemetry(self) -> None:
        # The graph is intentionally queried at 1 Hz, but chassis pose and
        # goal error must follow the 200 ms telemetry refresh.  Looking up the
        # latest buffered transforms is local and does not perform discovery.
        if self.ros_node:
            self.ros_node.sample_tf()
        state = self.telemetry
        if state.has_map_pose:
            self.metric_pose.set_value(
                f"{state.map_x:+.3f}, {state.map_y:+.3f}, {math.degrees(state.map_yaw):+.1f} deg",
                "ok",
            )
        else:
            self.metric_pose.set_value("等待 map->base_link", "warning")

        if state.goal_valid and state.has_map_pose:
            # Use the pose actually sent to Nav2.  The planner path is a
            # diagnostic stream and may arrive late or retain a previous goal.
            goal_x, goal_y, goal_yaw = state.goal_x, state.goal_y, state.goal_yaw
            goal_xy = math.hypot(state.map_x - goal_x, state.map_y - goal_y)
            goal_heading = abs(normalize_angle(state.map_yaw - goal_yaw))
            level = "ok" if goal_xy < 0.10 and goal_heading < 0.10 else "normal"
            self.metric_goal.set_value(
                f"{goal_xy * 100:.1f} cm / {math.degrees(goal_heading):.1f} deg", level
            )
            cte, heading = nearest_path_error(
                state.map_x, state.map_y, state.map_yaw, state.path
            )
            cte_level = "ok" if cte < 0.05 else "warning" if cte < 0.12 else "error"
            self.metric_cte.set_value(
                f"{cte * 100:.1f} cm / {math.degrees(heading):+.1f} deg", cte_level
            )
        else:
            self.metric_goal.set_value("--")
            self.metric_cte.set_value("--")

        icp_level = (
            "error"
            if state.icp_step_xy > 0.10 or state.icp_step_yaw > math.radians(5)
            else "warning"
            if state.icp_step_xy > 0.03 or state.icp_step_yaw > math.radians(1)
            else "ok"
        )
        self.metric_icp.set_value(
            f"{state.icp_step_xy * 100:.2f} cm / {math.degrees(state.icp_step_yaw):.2f} deg",
            icp_level,
        )
        vx, vy, wz = state.cmd_safe
        self.metric_cmd.set_value(f"{vx:+.3f}, {vy:+.3f}, {wz:+.3f}")
        odom_xy = math.hypot(state.lio_x - state.wheel_x, state.lio_y - state.wheel_y)
        odom_yaw = abs(normalize_angle(state.lio_yaw - state.wheel_yaw))
        odom_level = "ok" if odom_xy < 0.08 and odom_yaw < 0.10 else "warning"
        self.metric_odom.set_value(
            f"{odom_xy * 100:.1f} cm / {math.degrees(odom_yaw):.1f} deg", odom_level
        )

        for row, wheel in enumerate(state.wheels):
            values = (
                WHEEL_NAMES[row],
                f"{math.degrees(wheel.requested_angle):+.1f}",
                f"{math.degrees(wheel.actual_angle):+.1f}",
                f"{math.degrees(wheel.angle_error):+.1f}",
                f"{wheel.target_speed:+.3f}",
                f"{wheel.actual_speed:+.3f}",
                f"{wheel.steering_scale:.2f}",
            )
            for column, value in enumerate(values):
                item = self.wheel_table.item(row, column)
                if item is None:
                    item = QTableWidgetItem()
                    item.setTextAlignment(Qt.AlignCenter)
                    self.wheel_table.setItem(row, column, item)
                item.setText(value)
        self.path_plot.update()
        self.statusBar().showMessage(
            f"Nav2: {state.action_status} | remaining {state.action_remaining_distance:.2f} m | "
            f"localization {'OK' if state.localization_ok else 'NOT READY'} | "
            f"hold {state.localization_hold_count}"
        )

    def _update_preflight(self) -> None:
        now = time.monotonic()
        for interface in ("can4", "can5"):
            path = Path(f"/sys/class/net/{interface}")
            state = "缺失"
            ok = False
            if path.exists():
                try:
                    state = (path / "operstate").read_text(encoding="utf-8").strip()
                    ok = state in ("up", "unknown")
                except OSError:
                    state = "存在"
                    ok = True
            self._set_check(interface, ok, state)
        lidar_config = Path("~/.openflex/lidar_config.yaml").expanduser()
        self._set_check("lidar_config", lidar_config.is_file(), str(lidar_config))
        try:
            paths = self._map_paths()
            self._set_check("pcd", paths.pcd.is_file(), paths.pcd.name)
            self._set_check("yaml", paths.yaml.is_file(), paths.yaml.name)
        except ValueError:
            self._set_check("pcd", False, "地图名无效")
            self._set_check("yaml", False, "地图名无效")
        scan_age = self._age(now, "/scan")
        odom_age = self._age(now, "/odom_safe")
        tf_age = self._age(now, "map->base_link")
        self._set_check("scan", scan_age is not None and scan_age < 1.0, self._age_text(scan_age))
        self._set_check("odom", odom_age is not None and odom_age < 1.0, self._age_text(odom_age))
        self._set_check("tf", tf_age is not None and tf_age < 1.5, self._age_text(tf_age))

        important = [
            name
            for name in (
                "/controller_manager",
                "/pgo/pgo_node",
                "/icp_registration",
                "/planner_server",
                "/controller_server",
                "/bt_navigator",
                "/collision_monitor",
            )
            if name in self.telemetry.node_names
        ]
        self.active_nodes_label.setText("  ".join(important) if important else "未发现工作流节点")

    def _age(self, now: float, key: str) -> Optional[float]:
        stamp = self.telemetry.last_seen.get(key)
        return None if stamp is None else now - stamp

    @staticmethod
    def _age_text(age: Optional[float]) -> str:
        return "无数据" if age is None else f"{age:.1f}s"

    def _set_check(self, key: str, ok: bool, text: str) -> None:
        if key not in self.check_widgets:
            return
        dot, label = self.check_widgets[key]
        dot.set_state("running" if ok else "error")
        label.setText(text)
        label.setStyleSheet(f"color:{'#5fd38d' if ok else '#ff6b6b'};")

    def _update_mode(self) -> None:
        if self._process_running("mapping"):
            mode, state = "建图运行中", "running"
        elif self._process_running("navigation"):
            mode, state = "定位导航运行中", "running"
        elif self._process_running("fullbody"):
            mode, state = "全身控制运行中", "running"
        elif "/controller_server" in self.telemetry.node_names:
            mode, state = "外部导航已连接", "warning"
        elif any(name.startswith("/pgo/") for name in self.telemetry.node_names):
            mode, state = "外部建图已连接", "warning"
        else:
            mode, state = "空闲", "idle"
        self.mode_label.setText(mode)
        self.mode_dot.set_state(state)

    def _append_log(self, text: str) -> None:
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_view.appendPlainText(f"[{timestamp}] {text.rstrip()}")

    def closeEvent(self, event) -> None:
        running = [key for key in self.processes if self._process_running(key)]
        if running and not self._closing:
            answer = QMessageBox.question(
                self,
                "退出工作台",
                "仍有工作流进程运行。停止这些进程并退出？",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if answer != QMessageBox.Yes:
                event.ignore()
                return
        self._closing = True
        self._save_settings()
        self._stop_motion()
        for key in running:
            self._stop_process(key)
        for key in running:
            process = self.processes.get(key)
            if process is None:
                continue
            if process.state() != QProcess.NotRunning:
                process.waitForFinished(5000)
            process_group_id = self.process_group_ids.get(key)
            if process_group_id is not None:
                try:
                    os.killpg(process_group_id, signal.SIGTERM)
                except (ProcessLookupError, PermissionError):
                    pass
        event.accept()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavigationRosNode()
    ros_executor = RosExecutorThread(node)
    ros_executor.start()
    app = QApplication(sys.argv)
    app.setApplicationName(APP_TITLE)
    window = MainWindow(node)
    window.show()
    exit_code = app.exec_()
    window._stop_motion()
    node.shutdown_manual_joint_control()
    ros_executor.shutdown()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
