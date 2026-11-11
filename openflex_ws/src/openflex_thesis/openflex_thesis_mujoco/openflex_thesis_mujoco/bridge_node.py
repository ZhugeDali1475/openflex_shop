"""ROS 2 hardware-compatible bridge for the thesis MuJoCo scene."""

from copy import deepcopy
import json
import math
import random
import threading
import time
from typing import Dict, Optional, Sequence, Tuple
import uuid

from builtin_interfaces.msg import Time
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from openflex_thesis_msgs.msg import (
    ControlMode,
    FixedPlaceControl,
    GraspStatus,
    ObjectMemory,
    ObjectObservation,
    PlaceStatus,
)
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from .math_utils import clamp, compose_pose, normalize_angle, relative_pose, yaw_to_quaternion
from .model_builder import build_thesis_model
from .camera_spec import HEAD_CAMERA
from .trajectory import TrajectoryPoint, TrajectorySampler, duration_to_seconds, measured_goal_reached
from .contact_audit import forbidden_environment_contact, forbidden_payload_contact
from .release_verification import object_settled_on_surface, release_request_rejection

try:
    import mujoco
except ImportError:  # The package can still be built and pure helpers can be tested.
    mujoco = None


LEFT_ARM_JOINTS = tuple(f"openarmx_left_joint{index}" for index in range(1, 8))
RIGHT_ARM_JOINTS = tuple(f"openarmx_right_joint{index}" for index in range(1, 8))
CONTROLLERS = {
    "lift": ("lift_joint",),
    "left_arm": LEFT_ARM_JOINTS,
    "right_arm": RIGHT_ARM_JOINTS,
}
GRIPPER_JOINTS = {
    "left": "openarmx_left_finger_joint1",
    "right": "openarmx_right_finger_joint1",
}
CAMERAS = {
    "head": (
        "thesis_head_camera",
        "/thesis_sim/head",
        HEAD_CAMERA.optical_frame,
    ),
    "left_wrist": (
        "thesis_left_wrist_camera",
        "/thesis_sim/left_wrist",
        "openarmx_left_link7_pico",
    ),
    "right_wrist": (
        "thesis_right_wrist_camera",
        "/thesis_sim/right_wrist",
        "openarmx_right_link7_pico",
    ),
}


def _depth_to_uint16_mm(depth):
    """Convert MuJoCo metre depth to a RealSense-compatible 16UC1 image."""
    import numpy as np

    depth = np.asarray(depth, dtype=np.float32)
    valid = np.isfinite(depth) & (depth > 0.0) & (depth <= 65.535)
    output = np.zeros(depth.shape, dtype=np.uint16)
    output[valid] = np.rint(depth[valid] * 1000.0).astype(np.uint16)
    return np.ascontiguousarray(output)


class MujocoBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("openflex_thesis_mujoco_bridge")
        if mujoco is None:
            raise RuntimeError(
                "Python package 'mujoco' is not installed; run "
                "python3 -m pip install 'mujoco>=3.3,<4'"
            )
        self._declare_parameters()
        model_path = build_thesis_model(
            str(self.get_parameter("upstream_model_path").value),
            str(self.get_parameter("derived_model_path").value),
        )
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.model.opt.timestep = float(self.get_parameter("timestep_sec").value)
        self.data = mujoco.MjData(self.model)
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.callback_group = ReentrantCallbackGroup()
        self.random = random.Random(int(self.get_parameter("random_seed").value))
        self.reset_index = 0
        self.sequence = 0
        self.base_pose = [0.0, 0.0, 0.0]
        self.actual_twist = [0.0, 0.0, 0.0]
        self.commanded_twist = [0.0, 0.0, 0.0]
        self.last_twist_wall = 0.0
        self.trajectories: Dict[str, tuple] = {}
        self.trajectory_done: Dict[str, bool] = {}
        self.active_controllers = set()
        self.active_grippers = set()
        self.gripper_targets = {"left": 0.04, "right": 0.04}
        self.attached_arm: Optional[str] = None
        self.attachment_offset = None
        self.latest_memory: Optional[ObjectMemory] = None
        self.occluded = False
        self.current_object_nominal = [1.12, 0.0, 0.84, math.pi]
        self.place_status: Optional[PlaceStatus] = None
        self.place_status_until = 0.0
        self.pending_place = None
        self.control_mode = None
        self.control_mode_receipt = 0.0
        self.last_publish_stamp_ns = 0
        self.safety_stop = None

        self._cache_model_indices()
        # base_link sits above the floor plane the map frame uses; the URDF
        # states the same offset in base_footprint_joint (xyz="0 0 -0.17").
        # The offset is published as map -> odom, because the MoveIt SRDF
        # declares odom -> base_link as a *planar* virtual joint (x, y, yaw
        # only), i.e. MoveIt's base_link is modelled in odom's own plane. See
        # _publish_tf.
        self.base_link_height = 0.0
        self.attach_geometry = self._build_attach_geometry()
        self._create_ros_interfaces()
        self._reset_scene(randomize=False)
        self.viewer = None
        if bool(self.get_parameter("enable_viewer").value):
            from mujoco import viewer

            self.viewer = viewer.launch_passive(self.model, self.data)
        self.sim_thread = threading.Thread(
            target=self._simulation_loop, name="mujoco-step", daemon=True
        )
        self.sim_thread.start()
        self.renderers = {}
        self.camera_thread = None
        if bool(self.get_parameter("publish_rgbd").value):
            self.camera_thread = threading.Thread(
                target=self._camera_loop, name="mujoco-rgbd", daemon=True
            )
            self.camera_thread.start()
        rate = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / rate, self._publish_state)
        self.get_logger().warn(
            "MUJOCO DIGITAL EXPERIMENT active: ground-truth observations and oracle "
            "grasps are non-physical evidence"
        )
        self.get_logger().info(f"loaded derived model: {model_path}")

    def _declare_parameters(self) -> None:
        parameters = {
            "upstream_model_path": "",
            "derived_model_path": "/tmp/openflex_thesis_mujoco_scene.xml",
            "timestep_sec": 0.002,
            "real_time_factor": 1.0,
            "enable_viewer": False,
            "publish_rate_hz": 50.0,
            "publish_rgbd": True,
            "camera_rate_hz": 5.0,
            "image_width": 320,
            "image_height": 240,
            "command_timeout_sec": 0.25,
            "base_velocity_tau_sec": 0.08,
            "max_linear_mps": 0.50,
            "max_angular_rps": 0.80,
            "object_id": "thesis_item_01",
            "initial_base_pose": [0.0, 0.0, 0.0],
            "inactive_arm_park_positions": [-0.7, -1.2, 0.4, 1.2, 0.0, 0.0, 0.0],
            "hold_left_arm_park": True,
            "inactive_gripper_open_position": 0.04,
            "object_nominal_pose": [1.12, 0.0, 0.84, math.pi],
            "object_randomization_xy_m": [0.08, 0.18],
            "object_randomization_yaw_rad": 0.20,
            "occlusion_probability": 0.0,
            "observation_position_noise_std_m": 0.0,
            "observation_confidence": 0.99,
            "random_seed": 7,
            "grasp_attach_distance_m": 0.13,
            # Jaw region measured in the TCP frame from the MuJoCo hand meshes
            # with the fingers closed: pads span z in [-0.069, +0.016] and the
            # closed gap is 0.5 mm, so the outer span is 0.0855 m.
            "grasp_attach_reach_min_m": -0.069,
            "grasp_attach_reach_max_m": 0.016,
            "grasp_attach_max_opening_m": 0.0855,
            "grasp_attach_lateral_tolerance_m": 0.020,
            # Object-vs-TCP proximity log radius. Logging only; the
            # accept/reject decision does not read this value.
            "grasp_proximity_log_radius_m": 0.30,
            # The converted MJCF uses plain PD position actuators. Measured
            # offline at the G5 grasp pose they droop 30 mm (lift, kp=10 000
            # against a ~300 N load) and ~0.1 rad (shoulder, kp=180), i.e.
            # 60-120 mm of TCP error, so every approach missed the object.
            # Feeding the bias force forward for the actuated joints is what
            # the real arm's gravity compensation does; free bodies (the
            # grasped object) keep their own gravity.
            "gravity_compensation": True,
            "gripper_closed_threshold_m": 0.008,
            "gripper_goal_tolerance_m": 0.003,
            "gripper_action_timeout_sec": 4.0,
            "delivery_surface_z_m": 0.56,
        }
        for name, value in parameters.items():
            self.declare_parameter(name, value)

    def _id(self, object_type, name: str) -> int:
        identifier = mujoco.mj_name2id(self.model, object_type, name)
        if identifier < 0:
            raise ValueError(f"derived MuJoCo model is missing {name}")
        return int(identifier)

    def _cache_model_indices(self) -> None:
        robot_root = self._id(mujoco.mjtObj.mjOBJ_BODY, "openflex_robot_yaw_root")
        self.robot_body_ids = {
            index for index in range(self.model.nbody)
            if int(self.model.body_rootid[index]) == robot_root
        }
        self.environment_body_ids = {
            self._id(mujoco.mjtObj.mjOBJ_BODY, name)
            for name in ("thesis_shelf", "thesis_delivery_table")
        }
        self.joint_ids = {
            name: self._id(mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in (
                "base_x",
                "base_y",
                "base_yaw",
                "thesis_object_free",
                "thesis_occluder_free",
                "lift_joint",
                *LEFT_ARM_JOINTS,
                *RIGHT_ARM_JOINTS,
                *GRIPPER_JOINTS.values(),
            )
        }
        self.actuator_ids = {}
        for name in ("lift_joint", *LEFT_ARM_JOINTS, *RIGHT_ARM_JOINTS, *GRIPPER_JOINTS.values()):
            self.actuator_ids[name] = self._id(
                mujoco.mjtObj.mjOBJ_ACTUATOR, f"{name}_pos"
            )
        self.actuated_dofs = [
            int(self.model.jnt_dofadr[self.joint_ids[name]]) for name in self.actuator_ids
        ]
        self.base_link_id = self._id(mujoco.mjtObj.mjOBJ_BODY, "base_link")
        self.object_body_id = self._id(mujoco.mjtObj.mjOBJ_BODY, "thesis_object")
        self.hand_body_ids = {
            arm: self._id(mujoco.mjtObj.mjOBJ_BODY, f"openarmx_{arm}_hand_tcp")
            for arm in ("left", "right")
        }
        self.scalar_joint_names = []
        for joint_id in range(self.model.njnt):
            joint_type = int(self.model.jnt_type[joint_id])
            if joint_type not in (
                int(mujoco.mjtJoint.mjJNT_HINGE),
                int(mujoco.mjtJoint.mjJNT_SLIDE),
            ):
                continue
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
            if name not in ("base_x", "base_y", "base_yaw"):
                self.scalar_joint_names.append(name)

    def _latched_qos(self) -> QoSProfile:
        return QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

    def _create_ros_interfaces(self) -> None:
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 20)
        self.odom_pub = self.create_publisher(
            Odometry, "/swerve_drive_controller/odom", 20
        )
        self.observation_pub = self.create_publisher(
            ObjectObservation, "/thesis/object_observation", self._latched_qos()
        )
        self.camera_publishers = {}
        for key, (_camera_name, topic_prefix, _frame) in CAMERAS.items():
            self.camera_publishers[key] = {
                "color": self.create_publisher(
                    Image, f"{topic_prefix}/color/image_raw", 5
                ),
                "depth": self.create_publisher(
                    Image, f"{topic_prefix}/aligned_depth_to_color/image_raw", 5
                ),
                "info": self.create_publisher(
                    CameraInfo, f"{topic_prefix}/color/camera_info", 5
                ),
            }
        self.grasp_pub = self.create_publisher(GraspStatus, "/thesis/grasp_status", 20)
        self.place_pub = self.create_publisher(PlaceStatus, "/thesis/place/status", 20)
        self.metadata_pub = self.create_publisher(
            String, "/thesis_sim/trial_metadata", self._latched_qos()
        )
        self.diagnostic_state_pub = self.create_publisher(
            String, "/thesis_sim/diagnostic_state", self._latched_qos()
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Twist, "/thesis/cmd_vel_selected", self._on_twist, 20
        )
        self.create_subscription(
            ObjectMemory,
            "/thesis/object_memory",
            self._on_memory,
            self._latched_qos(),
            callback_group=self.callback_group,
        )
        self.create_subscription(
            FixedPlaceControl,
            "/thesis/place/control",
            self._on_place_control,
            self._latched_qos(),
            callback_group=self.callback_group,
        )
        self.create_subscription(ControlMode, '/thesis/control_mode', self._on_control_mode, self._latched_qos(),
                                 callback_group=self.callback_group)
        self.create_service(Trigger, "/thesis_sim/reset_nominal", self._on_reset_nominal)
        self.create_service(Trigger, "/thesis_sim/randomize", self._on_randomize)
        self.create_service(Trigger, "/thesis_sim/drop_object", self._on_drop_object)
        self.trajectory_servers = []
        for key, action_name in (
            ("lift", "/lift_joint_controller/follow_joint_trajectory"),
            ("left_arm", "/left_joint_trajectory_controller/follow_joint_trajectory"),
            ("right_arm", "/right_joint_trajectory_controller/follow_joint_trajectory"),
        ):
            self.trajectory_servers.append(
                ActionServer(
                    self,
                    FollowJointTrajectory,
                    action_name,
                    execute_callback=lambda handle, controller=key: self._execute_trajectory(
                        handle, controller
                    ),
                    goal_callback=lambda request, controller=key: self._trajectory_goal(
                        request, controller
                    ),
                    cancel_callback=lambda _: CancelResponse.ACCEPT,
                    callback_group=self.callback_group,
                )
            )
        self.gripper_servers = []
        for arm in ("left", "right"):
            self.gripper_servers.append(
                ActionServer(
                    self,
                    GripperCommand,
                    f"/{arm}_gripper_controller/gripper_cmd",
                    execute_callback=lambda handle, selected=arm: self._execute_gripper(
                        handle, selected
                    ),
                    goal_callback=lambda request, selected=arm: self._gripper_goal(request, selected),
                    cancel_callback=lambda _: CancelResponse.ACCEPT,
                    callback_group=self.callback_group,
                )
            )

    def _simulation_stamp(self) -> Time:
        seconds = max(0.0, float(self.data.time))
        stamp = Time()
        stamp.sec = int(seconds)
        stamp.nanosec = int((seconds - stamp.sec) * 1.0e9)
        return stamp

    def _next_publish_stamp(self) -> Time:
        """Return a stamp that never repeats or moves backwards."""
        stamp = self._simulation_stamp()
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        if stamp_ns <= self.last_publish_stamp_ns:
            stamp_ns = self.last_publish_stamp_ns + 1
        self.last_publish_stamp_ns = stamp_ns
        stamp.sec = stamp_ns // 1_000_000_000
        stamp.nanosec = stamp_ns % 1_000_000_000
        return stamp

    def _joint_position(self, name: str) -> float:
        joint_id = self.joint_ids.get(name)
        if joint_id is None:
            joint_id = self._id(mujoco.mjtObj.mjOBJ_JOINT, name)
            self.joint_ids[name] = joint_id
        return float(self.data.qpos[int(self.model.jnt_qposadr[joint_id])])

    def _set_free_joint_pose(
        self, joint_name: str, position: Sequence[float], orientation_xyzw: Sequence[float]
    ) -> None:
        joint_id = self.joint_ids[joint_name]
        qpos = int(self.model.jnt_qposadr[joint_id])
        dof = int(self.model.jnt_dofadr[joint_id])
        x, y, z, w = (float(value) for value in orientation_xyzw)
        self.data.qpos[qpos : qpos + 7] = [*position, w, x, y, z]
        self.data.qvel[dof : dof + 6] = 0.0

    def _initialize_actuators(self) -> None:
        gripper_open = float(
            self.get_parameter("inactive_gripper_open_position").value
        )
        for joint_name, actuator_id in self.actuator_ids.items():
            value = self._joint_position(joint_name)
            if "finger_joint1" in joint_name:
                value = gripper_open
            self.data.ctrl[actuator_id] = value
        self.gripper_targets = {"left": gripper_open, "right": gripper_open}

    def _set_left_arm_park_pose(self) -> None:
        positions = [
            float(value)
            for value in self.get_parameter("inactive_arm_park_positions").value
        ]
        if len(positions) != len(LEFT_ARM_JOINTS):
            raise ValueError(
                "inactive_arm_park_positions must contain seven left-arm joint values"
            )
        for name, value in zip(LEFT_ARM_JOINTS, positions):
            joint_id = self.joint_ids[name]
            qpos = int(self.model.jnt_qposadr[joint_id])
            dof = int(self.model.jnt_dofadr[joint_id])
            lower, upper = (float(item) for item in self.model.jnt_range[joint_id])
            if not lower <= value <= upper:
                raise ValueError(
                    f"inactive-arm park position for {name} is outside "
                    f"[{lower:.6f}, {upper:.6f}]"
                )
            self.data.qpos[qpos] = value
            self.data.qvel[dof] = 0.0

    def _reset_scene(self, randomize: bool) -> dict:
        with self.lock:
            mujoco.mj_resetData(self.model, self.data)
            base = [float(value) for value in self.get_parameter("initial_base_pose").value]
            nominal = [float(value) for value in self.get_parameter("object_nominal_pose").value]
            if len(base) != 3 or len(nominal) != 4:
                raise ValueError("initial_base_pose needs 3 and object_nominal_pose needs 4 values")
            if randomize:
                xy = [
                    float(value)
                    for value in self.get_parameter("object_randomization_xy_m").value
                ]
                if len(xy) != 2:
                    raise ValueError("object_randomization_xy_m must contain x/y ranges")
                nominal[0] += self.random.uniform(-xy[0], xy[0])
                nominal[1] += self.random.uniform(-xy[1], xy[1])
                yaw_range = float(self.get_parameter("object_randomization_yaw_rad").value)
                nominal[3] = normalize_angle(
                    nominal[3] + self.random.uniform(-yaw_range, yaw_range)
                )
                self.occluded = self.random.random() < float(
                    self.get_parameter("occlusion_probability").value
                )
            else:
                self.occluded = False
            self.base_pose = base
            self.actual_twist = [0.0, 0.0, 0.0]
            self.commanded_twist = [0.0, 0.0, 0.0]
            self.current_object_nominal = nominal
            self.attached_arm = None
            self.attachment_offset = None
            self.trajectories.clear()
            self.trajectory_done.clear()
            self.place_status = None
            self.pending_place = None
            self.safety_stop = None
            self.environment_contact_checks = 0
            self.last_contact_audit_time = -5.0
            self._set_base_state()
            oq = yaw_to_quaternion(nominal[3])
            self._set_free_joint_pose("thesis_object_free", nominal[:3], oq)
            occluder_position = (
                (nominal[0] - 0.14, nominal[1], nominal[2])
                if self.occluded
                else (nominal[0], 3.0, nominal[2])
            )
            self._set_free_joint_pose(
                "thesis_occluder_free", occluder_position, (0.0, 0.0, 0.0, 1.0)
            )
            self._set_left_arm_park_pose()
            self._initialize_actuators()
            mujoco.mj_forward(self.model, self.data)
            self.base_link_height = float(self.data.xpos[self.base_link_id][2])
            self.reset_index += 1
            metadata = {
                "simulator": "mujoco",
                "evidence_class": "digital_oracle_non_physical",
                "reset_index": self.reset_index,
                "randomized": bool(randomize),
                "object_pose_xyzyaw": nominal,
                "inactive_arm_park_positions": [
                    float(value)
                    for value in self.get_parameter(
                        "inactive_arm_park_positions"
                    ).value
                ],
                "occluded": self.occluded,
                "seed": int(self.get_parameter("random_seed").value),
            }
        self.metadata_pub.publish(String(data=json.dumps(metadata, sort_keys=True)))
        return metadata

    def _on_reset_nominal(self, _request, response):
        metadata = self._reset_scene(False)
        response.success = True
        response.message = json.dumps(metadata, sort_keys=True)
        return response

    def _on_randomize(self, _request, response):
        metadata = self._reset_scene(True)
        response.success = True
        response.message = json.dumps(metadata, sort_keys=True)
        return response

    def _on_drop_object(self, _request, response):
        with self.lock:
            self.attached_arm = None
            self.attachment_offset = None
        response.success = True
        response.message = "object attachment disabled"
        return response

    def _on_twist(self, msg: Twist) -> None:
        maximum_linear = float(self.get_parameter("max_linear_mps").value)
        maximum_angular = float(self.get_parameter("max_angular_rps").value)
        with self.lock:
            if self.safety_stop is not None:
                return
            self.commanded_twist = [
                clamp(float(msg.linear.x), -maximum_linear, maximum_linear),
                clamp(float(msg.linear.y), -maximum_linear, maximum_linear),
                clamp(float(msg.angular.z), -maximum_angular, maximum_angular),
            ]
            self.last_twist_wall = time.monotonic()

    def _on_memory(self, msg: ObjectMemory) -> None:
        self.latest_memory = deepcopy(msg)

    def _trajectory_goal(self, request, controller: str) -> GoalResponse:
        with self.lock:
            if (tuple(request.trajectory.joint_names) != CONTROLLERS[controller]
                    or not request.trajectory.points or self.safety_stop is not None
                    or controller in self.active_controllers or not self._arm_control_valid()):
                return GoalResponse.REJECT
            try:
                TrajectorySampler(request.trajectory.joint_names,
                    {name: self._joint_position(name) for name in request.trajectory.joint_names},
                    [TrajectoryPoint(duration_to_seconds(point.time_from_start), tuple(point.positions))
                     for point in request.trajectory.points])
            except (KeyError, ValueError):
                return GoalResponse.REJECT
            self.active_controllers.add(controller)
            return GoalResponse.ACCEPT

    def _arm_control_valid(self):
        return bool(self.control_mode is not None and self.control_mode.mode == ControlMode.MOVEIT
                    and self.control_mode.inputs_healthy and time.monotonic() - self.control_mode_receipt <= 0.5
                    and math.hypot(*self.actual_twist[:2]) <= 0.02 and abs(self.actual_twist[2]) <= 0.03)

    def _hold_controller_measured(self, controller, token):
        with self.lock:
            current = self.trajectories.get(controller)
            if current is not None and current[0] == token:
                self.trajectories.pop(controller, None)
            for name in CONTROLLERS[controller]:
                self.data.ctrl[self.actuator_ids[name]] = self._joint_position(name)

    def _execute_trajectory(self, goal_handle, controller: str):
        result = FollowJointTrajectory.Result()
        goal = goal_handle.request
        token = None
        succeeded = False
        try:
            with self.lock:
                start_positions = {
                    name: self._joint_position(name) for name in goal.trajectory.joint_names
                }
                points = [
                    TrajectoryPoint(
                        duration_to_seconds(point.time_from_start),
                        tuple(float(value) for value in point.positions),
                    )
                    for point in goal.trajectory.points
                ]
                sampler = TrajectorySampler(
                    goal.trajectory.joint_names, start_positions, points
                )
                token = uuid.uuid4().hex
                self.trajectories[controller] = (token, sampler, float(self.data.time))
                self.trajectory_done[token] = False
            wall_timeout = max(5.0, sampler.duration_sec / max(
                0.01, float(self.get_parameter("real_time_factor").value)
            ) + 5.0)
            started = time.monotonic()
            settled_since = None
            while time.monotonic() - started <= wall_timeout:
                if self.safety_stop is not None or not self._arm_control_valid():
                    goal_handle.abort()
                    result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                    result.error_string = "MuJoCo safety stop or fresh exclusive MoveIt control revoked"
                    return result
                if goal_handle.is_cancel_requested:
                    self._hold_controller_measured(controller, token)
                    goal_handle.canceled()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = "trajectory canceled"
                    return result
                with self.lock:
                    done = self.trajectory_done.get(token, False)
                    positions = [self._joint_position(name) for name in sampler.joint_names]
                    velocities = [float(self.data.qvel[self.model.jnt_dofadr[self.joint_ids[name]]])
                                  for name in sampler.joint_names]
                    reached = done and measured_goal_reached(sampler.joint_names,
                        sampler.points[-1].positions, positions, velocities)
                if not reached:
                    settled_since = None
                else:
                    settled_since = settled_since or time.monotonic()
                if settled_since is not None and time.monotonic() - settled_since >= 0.2:
                    goal_handle.succeed()
                    succeeded = True
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = "MuJoCo measured joint target and stationary velocities verified for 0.2s"
                    return result
                time.sleep(0.01)
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
            result.error_string = "MuJoCo trajectory wall timeout"
            return result
        except (KeyError, ValueError) as error:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = str(error)
            return result
        finally:
            with self.lock:
                if not succeeded and token is not None:
                    self._hold_controller_measured(controller, token)
                self.trajectory_done.pop(token, None)
                self.active_controllers.discard(controller)

    def _gripper_goal(self, request, arm):
        with self.lock:
            if (self.safety_stop is not None or arm in self.active_grippers or self.pending_place is not None
                    or not self._arm_control_valid()
                    or not math.isfinite(request.command.position) or not 0 <= request.command.position <= 0.044
                    or not math.isfinite(request.command.max_effort) or request.command.max_effort < 0):
                return GoalResponse.REJECT
            self.active_grippers.add(arm)
            return GoalResponse.ACCEPT

    def _execute_gripper(self, goal_handle, arm: str):
        succeeded = False
        try:
            result = self._run_gripper(goal_handle, arm)
            succeeded = bool(result.reached_goal or result.stalled)
            return result
        finally:
            with self.lock:
                if not succeeded:
                    self.gripper_targets[arm] = self._joint_position(GRIPPER_JOINTS[arm])
                    self.data.ctrl[self.actuator_ids[GRIPPER_JOINTS[arm]]] = self.gripper_targets[arm]
                self.active_grippers.discard(arm)

    def _run_gripper(self, goal_handle, arm: str):
        result = GripperCommand.Result()
        if self.safety_stop is not None or not self._arm_control_valid():
            goal_handle.abort()
            return result
        target = clamp(float(goal_handle.request.command.position), 0.0, 0.044)
        with self.lock:
            self.gripper_targets[arm] = target
            self.data.ctrl[self.actuator_ids[GRIPPER_JOINTS[arm]]] = target
        tolerance = float(self.get_parameter("gripper_goal_tolerance_m").value)
        timeout = float(self.get_parameter("gripper_action_timeout_sec").value)
        started = time.monotonic()
        while time.monotonic() - started <= timeout:
            if self.safety_stop is not None or not self._arm_control_valid():
                goal_handle.abort()
                return result
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return result
            with self.lock:
                position = self._joint_position(GRIPPER_JOINTS[arm])
            if abs(position - target) <= tolerance:
                goal_handle.succeed()
                result.position = position
                result.reached_goal = True
                return result
            time.sleep(0.01)
        # Contact can prevent an ideal position while still representing a valid grasp.
        with self.lock:
            result.position = self._joint_position(GRIPPER_JOINTS[arm])
            result.stalled = self.attached_arm == arm
        if result.stalled:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _set_base_state(self) -> None:
        for index, name in enumerate(("base_x", "base_y", "base_yaw")):
            joint_id = self.joint_ids[name]
            qpos = int(self.model.jnt_qposadr[joint_id])
            dof = int(self.model.jnt_dofadr[joint_id])
            self.data.qpos[qpos] = self.base_pose[index]
            if index == 0:
                yaw = self.base_pose[2]
                self.data.qvel[dof] = (
                    math.cos(yaw) * self.actual_twist[0]
                    - math.sin(yaw) * self.actual_twist[1]
                )
            elif index == 1:
                yaw = self.base_pose[2]
                self.data.qvel[dof] = (
                    math.sin(yaw) * self.actual_twist[0]
                    + math.cos(yaw) * self.actual_twist[1]
                )
            else:
                self.data.qvel[dof] = self.actual_twist[2]

    def _step_base(self, dt: float) -> None:
        if time.monotonic() - self.last_twist_wall > float(
            self.get_parameter("command_timeout_sec").value
        ):
            command = (0.0, 0.0, 0.0)
        else:
            command = self.commanded_twist
        tau = max(1.0e-4, float(self.get_parameter("base_velocity_tau_sec").value))
        alpha = 1.0 - math.exp(-dt / tau)
        for index in range(3):
            self.actual_twist[index] += alpha * (command[index] - self.actual_twist[index])
        yaw = self.base_pose[2]
        self.base_pose[0] += dt * (
            math.cos(yaw) * self.actual_twist[0]
            - math.sin(yaw) * self.actual_twist[1]
        )
        self.base_pose[1] += dt * (
            math.sin(yaw) * self.actual_twist[0]
            + math.cos(yaw) * self.actual_twist[1]
        )
        self.base_pose[2] = normalize_angle(
            self.base_pose[2] + dt * self.actual_twist[2]
        )
        self._set_base_state()

    def _step_trajectories(self) -> None:
        for controller, entry in list(self.trajectories.items()):
            token, sampler, start_time = entry
            values, done = sampler.sample(float(self.data.time) - start_time)
            for name, value in values.items():
                actuator = self.actuator_ids[name]
                low, high = self.model.actuator_ctrlrange[actuator]
                self.data.ctrl[actuator] = clamp(value, float(low), float(high))
            if done:
                self.trajectories.pop(controller, None)
                self.trajectory_done[token] = True

    @staticmethod
    def _mujoco_body_pose(data, body_id: int) -> Tuple[tuple, tuple]:
        position = tuple(float(value) for value in data.xpos[body_id])
        qw, qx, qy, qz = (float(value) for value in data.xquat[body_id])
        return position, (qx, qy, qz, qw)

    def _try_attach(self) -> None:
        """Attach the object only when the closed jaws actually straddle it.

        The first implementation attached whenever the TCP came within
        `grasp_attach_distance_m` (0.13-0.16 m) of the object centre, which
        succeeds even when the object is not between the fingers. That made
        every G4 run report `verified=true` regardless of the grasp geometry.
        The gate below requires the object centre to sit inside the jaw region
        expressed in the TCP frame, and the jaw opening to bracket the object.
        """
        if self.attached_arm is not None:
            return
        closed_threshold = float(self.get_parameter("gripper_closed_threshold_m").value)
        max_distance = float(self.get_parameter("grasp_attach_distance_m").value)
        geometry = self.attach_geometry
        object_position, object_orientation = self._mujoco_body_pose(
            self.data, self.object_body_id
        )
        for arm in ("left", "right"):
            if self.gripper_targets[arm] > closed_threshold:
                continue
            hand_position, hand_orientation = self._mujoco_body_pose(
                self.data, self.hand_body_ids[arm]
            )
            distance = math.sqrt(
                sum((object_position[i] - hand_position[i]) ** 2 for i in range(3))
            )
            if distance > max_distance:
                # Also worth seeing: the jaw gate is only consulted when the
                # object is already inside this pre-screen radius, so a gate
                # rejection and a distance rejection have different fixes.
                self.get_logger().info(
                    f"grasp candidate on {arm} rejected: object {distance:.3f} m away "
                    f"> grasp_attach_distance_m {max_distance:.3f}",
                    throttle_duration_sec=1.0,
                )
                continue
            offset, orientation = relative_pose(
                hand_position, hand_orientation, object_position, object_orientation
            )
            reason = self.attach_rejection(offset, geometry)
            if reason:
                # Info, not debug: a rejected close is the difference between
                # "the fingers shut on nothing" and a recorded G4 grasp, so it
                # has to be visible in a normal mission log.
                self.get_logger().info(
                    f"grasp candidate on {arm} rejected: {reason} "
                    f"(distance {distance:.3f} m, object in TCP frame "
                    f"{tuple(round(value, 4) for value in offset)})",
                    throttle_duration_sec=1.0,
                )
                continue
            self.attached_arm = arm
            self.attachment_offset = (offset, orientation)
            self.get_logger().info(
                f"simulated grasp attached {self.get_parameter('object_id').value} "
                f"to {arm} TCP at distance {distance:.3f} m "
                f"(object in TCP frame {tuple(round(value, 4) for value in offset)})"
            )
            break

    def attach_rejection(self, offset, geometry) -> str:
        """Return why this TCP-relative object pose is not a real grasp."""
        x, y, z = (float(value) for value in offset)
        reach_min, reach_max, half_gap = geometry
        if z < reach_min:
            return f"object behind the fingertips (z={z:.4f} < {reach_min:.4f})"
        if z > reach_max:
            return f"object beyond the finger pads (z={z:.4f} > {reach_max:.4f})"
        if abs(y) > half_gap:
            return f"object outside the jaw span (|y|={abs(y):.4f} > {half_gap:.4f})"
        if abs(x) > float(self.get_parameter("grasp_attach_lateral_tolerance_m").value):
            return f"object laterally off the jaw axis (|x|={abs(x):.4f})"
        return ""

    def _contact_summary(self) -> str:
        """Names of the bodies currently in contact, for approach diagnosis.

        A PD position servo cannot hold a commanded pose against an
        unmodelled contact; when the hand stops short of the object this line
        says whether the arm was resting on the shelf.
        """
        pairs = []
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            names = []
            for geom in (int(contact.geom1), int(contact.geom2)):
                body = int(self.model.geom_bodyid[geom])
                name = (
                    mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body)
                    if body > 0
                    else "world"
                )
                names.append(name or f"body{body}")
            pair = "--".join(sorted(names))
            if pair not in pairs:
                pairs.append(pair)
        if not pairs:
            return " contacts=none"
        pairs.sort()
        return " contacts=" + ";".join(pairs)

    def _diagnostic_contact_pairs(self):
        pairs = []
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            names = []
            for geom in (int(contact.geom1), int(contact.geom2)):
                body = int(self.model.geom_bodyid[geom])
                body_name = (
                    mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body)
                    if body > 0 else "world"
                )
                geom_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, geom)
                names.append({"body": body_name or f"body{body}",
                              "geom": geom_name or f"geom{geom}"})
            pairs.append({"first": names[0], "second": names[1],
                          "distance_m": float(contact.dist)})
        return pairs

    def _log_grasp_proximity(self) -> None:
        """Record where the object sits in each TCP frame while the hand is near.

        `_try_attach` only speaks once the jaws are commanded closed, so the
        moment the object starts moving away from the TCP - the difference
        between "the approach missed it" and "the closed hand shoved it" - is
        otherwise invisible. This line is throttled and never changes the
        accept/reject decision.
        """
        radius = float(self.get_parameter("grasp_proximity_log_radius_m").value)
        if radius <= 0.0:
            return
        object_position, object_orientation = self._mujoco_body_pose(
            self.data, self.object_body_id
        )
        for arm in ("left", "right"):
            hand_position, hand_orientation = self._mujoco_body_pose(
                self.data, self.hand_body_ids[arm]
            )
            distance = math.sqrt(
                sum((object_position[i] - hand_position[i]) ** 2 for i in range(3))
            )
            if distance > radius:
                continue
            offset, _ = relative_pose(
                hand_position, hand_orientation, object_position, object_orientation
            )
            self.get_logger().info(
                f"grasp proximity on {arm}: object {distance:.3f} m from the TCP "
                f"(object in TCP frame "
                f"{tuple(round(float(value), 4) for value in offset)}, "
                f"gripper command {self.gripper_targets[arm]:.4f}, "
                f"attached={self.attached_arm}){self._contact_summary()}",
                throttle_duration_sec=1.0,
            )

    def _build_attach_geometry(self) -> tuple:
        """Measured jaw region in the TCP frame, from the MuJoCo hand meshes.

        With the right finger joints at the closed opening the finger pads span
        z in [-0.069, +0.016] and the closed jaws leave a 1 mm gap; the pads
        are 40 mm deep in x. Values live in parameters so a different hand can
        be re-measured without editing code.
        """
        return (
            float(self.get_parameter("grasp_attach_reach_min_m").value),
            float(self.get_parameter("grasp_attach_reach_max_m").value),
            0.5 * float(self.get_parameter("grasp_attach_max_opening_m").value),
        )

    def _step_attachment(self) -> None:
        if self.attached_arm is None or self.attachment_offset is None:
            return
        if (self.gripper_targets[self.attached_arm] >= 0.03
                and self._joint_position(GRIPPER_JOINTS[self.attached_arm]) >= 0.02):
            self.attached_arm = None
            self.attachment_offset = None
            return
        hand_position, hand_orientation = self._mujoco_body_pose(
            self.data, self.hand_body_ids[self.attached_arm]
        )
        position, orientation = compose_pose(
            hand_position,
            hand_orientation,
            self.attachment_offset[0],
            self.attachment_offset[1],
        )
        self._set_free_joint_pose("thesis_object_free", position, orientation)
        mujoco.mj_forward(self.model, self.data)

    def _apply_gravity_compensation(self) -> None:
        """Feed the gravity/Coriolis load forward for the actuated joints.

        Without this the PD position servos of the converted MJCF hold a
        configuration that is 60-120 mm away from the commanded one at the
        grasp pose, which is larger than the object itself. Only actuated
        joints are compensated; the base slides and the free bodies keep the
        load they should have.
        """
        if not bool(self.get_parameter("gravity_compensation").value):
            return
        bias = self.data.qfrc_bias
        applied = self.data.qfrc_applied
        for dof in self.actuated_dofs:
            applied[dof] = bias[dof]

    def _hold_left_arm_park(self) -> None:
        """Keep the inactive-arm baseline posture fixed in MuJoCo.

        A physical deployment requires an equivalent controller or brake. This
        lock is a digital baseline constraint and must be reported as such.
        """
        if bool(self.get_parameter("hold_left_arm_park").value):
            self._set_left_arm_park_pose()

    def _audit_environment_contacts(self) -> None:
        if self.safety_stop is not None:
            return
        self.environment_contact_checks += 1
        contacts = []
        for contact in self.data.contact[:self.data.ncon]:
            geoms = (int(contact.geom1), int(contact.geom2))
            bodies = tuple(int(self.model.geom_bodyid[geom]) for geom in geoms)
            if forbidden_environment_contact(
                bodies, float(contact.dist), self.robot_body_ids, self.environment_body_ids
            ) or forbidden_payload_contact(bodies, float(contact.dist),
                self.object_body_id if getattr(self, 'attached_arm', None) is not None else None,
                self.environment_body_ids):
                contacts.append({
                    "bodies": [mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body)
                               for body in bodies],
                    "geoms": [mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, geom)
                              for geom in geoms],
                    "distance_m": float(contact.dist),
                })
        if contacts:
            self.safety_stop = {
                "sim_time_sec": float(self.data.time), "base_xy_yaw": list(self.base_pose),
                "joints": {name: self._joint_position(name) for name in self.scalar_joint_names},
                "contacts": contacts, "response": "freeze_simulation_until_reset",
            }
            self.actual_twist = [0.0, 0.0, 0.0]
            self.commanded_twist = [0.0, 0.0, 0.0]
            self.data.qvel[:] = 0.0
            self.get_logger().error("MUJOCO_SAFETY_STOP " + json.dumps(self.safety_stop))
        elif self.data.time - self.last_contact_audit_time >= 5.0:
            self.last_contact_audit_time = float(self.data.time)
            self.get_logger().info("MUJOCO_SAFETY_AUDIT " + json.dumps({
                "checks": self.environment_contact_checks, "sim_time_sec": float(self.data.time),
                "scope": "robot_bodies_vs_shelf_and_delivery_table", "violation": False,
            }))

    def _simulation_loop(self) -> None:
        dt = float(self.model.opt.timestep)
        real_time_factor = max(0.01, float(self.get_parameter("real_time_factor").value))
        wall_step = dt / real_time_factor
        deadline = time.monotonic()
        while not self.stop_event.is_set():
            with self.lock:
                if self.safety_stop is not None:
                    self.data.time += dt
                else:
                    self._step_base(dt)
                    self._step_trajectories()
                    for arm, joint in GRIPPER_JOINTS.items():
                        self.data.ctrl[self.actuator_ids[joint]] = self.gripper_targets[arm]
                    self._apply_gravity_compensation()
                    mujoco.mj_step(self.model, self.data)
                    self._hold_left_arm_park()
                    mujoco.mj_forward(self.model, self.data)
                    self._audit_environment_contacts()
                    if self.safety_stop is None:
                        self._try_attach()
                        self._log_grasp_proximity()
                        self._step_attachment()
                self._step_placement()
            deadline += wall_step
            wait = deadline - time.monotonic()
            if wait > 0.0:
                self.stop_event.wait(wait)
            elif wait < -0.1:
                deadline = time.monotonic()

    def _camera_info(self, stamp: Time, frame_id: str, camera_name: str) -> CameraInfo:
        camera_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name
        )
        if camera_id < 0:
            raise ValueError(f"MuJoCo model is missing camera {camera_name}")
        width = int(self.get_parameter("image_width").value)
        height = int(self.get_parameter("image_height").value)
        fovy = math.radians(float(self.model.cam_fovy[camera_id]))
        fy = 0.5 * height / math.tan(0.5 * fovy)
        fx = fy
        cx = 0.5 * width
        cy = 0.5 * height
        message = CameraInfo()
        message.header.stamp = deepcopy(stamp)
        message.header.frame_id = frame_id
        message.width = width
        message.height = height
        message.distortion_model = "plumb_bob"
        message.d = [0.0] * 5
        message.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        message.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        message.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return message

    def _publish_cameras(self, render_data, stamp: Time) -> None:
        optical_id = self._id(mujoco.mjtObj.mjOBJ_BODY, HEAD_CAMERA.optical_frame)
        position, orientation = self._mujoco_body_pose(render_data, optical_id)
        transform = TransformStamped()
        transform.header.stamp = deepcopy(stamp)
        transform.header.frame_id = "map"
        transform.child_frame_id = HEAD_CAMERA.optical_frame
        transform.transform.translation.x, transform.transform.translation.y, \
            transform.transform.translation.z = position
        transform.transform.rotation.x, transform.transform.rotation.y, \
            transform.transform.rotation.z, transform.transform.rotation.w = orientation
        self.tf_broadcaster.sendTransform(transform)
        messages = []
        for key, (camera_name, _topic_prefix, frame_id) in CAMERAS.items():
            renderer = self.renderers[key]
            renderer.disable_depth_rendering()
            renderer.update_scene(render_data, camera=camera_name)
            color = renderer.render().copy()
            renderer.enable_depth_rendering()
            renderer.update_scene(render_data, camera=camera_name)
            depth = _depth_to_uint16_mm(renderer.render())
            color_message = Image()
            color_message.header.stamp = deepcopy(stamp)
            color_message.header.frame_id = frame_id
            color_message.height, color_message.width = color.shape[:2]
            color_message.encoding = "rgb8"
            color_message.is_bigendian = False
            color_message.step = color_message.width * 3
            color_message.data = color.tobytes()
            depth_message = Image()
            depth_message.header.stamp = deepcopy(stamp)
            depth_message.header.frame_id = frame_id
            depth_message.height, depth_message.width = depth.shape
            depth_message.encoding = "16UC1"
            depth_message.is_bigendian = False
            depth_message.step = depth_message.width * 2
            depth_message.data = depth.tobytes()
            messages.append(
                (
                    key,
                    color_message,
                    depth_message,
                    self._camera_info(stamp, frame_id, camera_name),
                )
            )
        for key, color, depth, info in messages:
            if not rclpy.ok():
                return
            publishers = self.camera_publishers[key]
            publishers["color"].publish(color)
            publishers["depth"].publish(depth)
            publishers["info"].publish(info)

    def _camera_loop(self) -> None:
        width = int(self.get_parameter("image_width").value)
        height = int(self.get_parameter("image_height").value)
        period = 1.0 / float(self.get_parameter("camera_rate_hz").value)
        if width <= 0 or height <= 0 or period <= 0.0:
            self.get_logger().error("RGB-D dimensions and rate must be positive")
            return
        try:
            render_data = mujoco.MjData(self.model)
            self.renderers = {
                key: mujoco.Renderer(self.model, height=height, width=width)
                for key in CAMERAS
            }
            deadline = time.monotonic()
            while not self.stop_event.is_set() and rclpy.ok():
                with self.lock:
                    qpos = self.data.qpos.copy()
                    qvel = self.data.qvel.copy()
                    render_time = float(self.data.time)
                render_data.qpos[:] = qpos
                render_data.qvel[:] = qvel
                render_data.time = render_time
                mujoco.mj_forward(self.model, render_data)
                stamp = Time()
                stamp.sec = int(render_time)
                stamp.nanosec = int((render_time - stamp.sec) * 1.0e9)
                self._publish_cameras(render_data, stamp)
                deadline += period
                remaining = deadline - time.monotonic()
                if remaining > 0.0:
                    self.stop_event.wait(remaining)
                elif remaining < -period:
                    deadline = time.monotonic()
        except Exception as error:
            if rclpy.ok():
                self.get_logger().error(f"MuJoCo RGB-D rendering stopped: {error}")
        finally:
            for renderer in self.renderers.values():
                renderer.close()
            self.renderers = {}

    def _on_control_mode(self, msg: ControlMode) -> None:
        with self.lock:
            self.control_mode = deepcopy(msg)
            self.control_mode_receipt = time.monotonic()

    def _place_guard_valid(self) -> bool:
        return bool(self.control_mode is not None and self.control_mode.mode == ControlMode.PLACE
                    and time.monotonic() - self.control_mode_receipt <= 0.5
                    and math.hypot(*self.actual_twist[:2]) <= 0.02 and abs(self.actual_twist[2]) <= 0.03
                    and math.hypot(*self.commanded_twist[:2]) <= 0.02 and abs(self.commanded_twist[2]) <= 0.03
                    and not self.trajectories and self.safety_stop is None)

    def _delivery_geometry(self):
        surface = self._id(mujoco.mjtObj.mjOBJ_GEOM, 'thesis_delivery_top')
        object_geometry = self._id(mujoco.mjtObj.mjOBJ_GEOM, 'thesis_object_geom')
        return (surface, tuple(float(value) for value in self.model.geom_size[object_geometry]),
                tuple(float(value) for value in self.data.geom_xpos[surface]),
                tuple(float(value) for value in self.model.geom_size[surface]))

    def _place_failure(self, code, detail):
        self.place_status.header.stamp = self._simulation_stamp()
        self.place_status.failure_code = code
        self.place_status.verified = False
        self.place_status.detail = detail
        if self.pending_place is not None:
            self.place_status.released = self.attached_arm is None
        if self.pending_place is not None and self.attached_arm is not None:
            self.gripper_targets[self.pending_place['arm']] = self.pending_place['previous_gripper_target']
        self.pending_place = None
        self.place_status_until = time.monotonic() + 2.0

    def _on_place_control(self, msg: FixedPlaceControl) -> None:
        with self.lock:
            if not msg.enabled:
                if self.pending_place is not None and msg.object_id == self.place_status.object_id:
                    self._place_failure(93, 'release verification canceled; cancellation is not successful placement')
                return
            if self.pending_place is not None:
                return
            self.place_status = PlaceStatus()
            self.place_status.header.stamp = self._simulation_stamp()
            self.place_status.header.frame_id = 'map'
            self.place_status.object_id = msg.object_id
            self.place_status_until = time.monotonic() + 2.0
            if not self._place_guard_valid():
                self._place_failure(92, 'fresh exclusive PLACE mode, stopped base and stationary arm required')
                return
            if (msg.object_id != str(self.get_parameter('object_id').value)
                    or msg.active_arm not in (1, 2) or msg.delivery_pose.header.frame_id != 'map'):
                self._place_failure(94, 'release object, arm or map-frame contract mismatch')
                return
            arm = 'left' if msg.active_arm == 1 else 'right'
            if self.attached_arm != arm:
                self._place_failure(91, 'requested object is not attached to the requested arm')
                return
            pose = msg.delivery_pose.pose
            target = (pose.position.x, pose.position.y, pose.position.z)
            position, orientation = self._mujoco_body_pose(self.data, self.object_body_id)
            surface, half_size, center, surface_size = self._delivery_geometry()
            reason = release_request_rejection(position, orientation, target, half_size, center, surface_size)
            if reason:
                self._place_failure(95, reason)
                return
            self.pending_place = {'arm': arm, 'target': target, 'surface': surface, 'half_size': half_size,
                                  'surface_center': center, 'surface_half_size': surface_size,
                                  'started': time.monotonic(), 'stable_since': None,
                                  'previous_gripper_target': self.gripper_targets[arm]}
            self.gripper_targets[arm] = 0.04
            self.place_status.target_reached = True
            self.place_status.detail = 'opening measured gripper at actual object pose; awaiting physical settling'

    def _step_placement(self) -> None:
        pending = self.pending_place
        if pending is None:
            return
        self.place_status.header.stamp = self._simulation_stamp()
        self.place_status_until = time.monotonic() + 2.0
        if not self._place_guard_valid():
            self._place_failure(92, 'release safety ownership or base-stop guard revoked')
            return
        if time.monotonic() - pending['started'] > 10.0:
            self._place_failure(96, 'release settling deadline exceeded')
            return
        self.place_status.released = self.attached_arm is None
        position, orientation = self._mujoco_body_pose(self.data, self.object_body_id)
        object_geometry = self._id(mujoco.mjtObj.mjOBJ_GEOM, 'thesis_object_geom')
        supported = any({int(contact.geom1), int(contact.geom2)} == {object_geometry, pending['surface']}
                        and -0.001 <= contact.dist <= 0.001 for contact in self.data.contact[:self.data.ncon])
        dof = int(self.model.jnt_dofadr[self.joint_ids['thesis_object_free']])
        settled = object_settled_on_surface(position, orientation, pending['half_size'], pending['target'],
            pending['surface_center'], pending['surface_half_size'], self.data.qvel[dof:dof + 6], supported,
            self.attached_arm is not None, self._joint_position(GRIPPER_JOINTS[pending['arm']]) >= 0.03)
        if not settled:
            pending['stable_since'] = None
            return
        if pending['stable_since'] is None:
            pending['stable_since'] = float(self.data.time)
        if self.data.time - pending['stable_since'] >= 0.5:
            self.place_status.verified = True
            self.place_status.detail = 'MuJoCo actual-pose release: open gripper, detached box, table contact and 0.5s settled'
            self.pending_place = None

    def _publish_state(self) -> None:
        with self.lock:
            stamp = self._next_publish_stamp()
            clock = Clock(clock=deepcopy(stamp))
            joint = JointState()
            joint.header.stamp = deepcopy(stamp)
            joint.name = list(self.scalar_joint_names)
            joint.position = [self._joint_position(name) for name in joint.name]
            joint.velocity = [
                float(self.data.qvel[int(self.model.jnt_dofadr[
                    self._id(mujoco.mjtObj.mjOBJ_JOINT, name)
                ])])
                for name in joint.name
            ]
            odom = Odometry()
            odom.header.stamp = deepcopy(stamp)
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.pose.pose.position.x = self.base_pose[0]
            odom.pose.pose.position.y = self.base_pose[1]
            q = yaw_to_quaternion(self.base_pose[2])
            odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, \
                odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = q
            odom.twist.twist.linear.x = self.actual_twist[0]
            odom.twist.twist.linear.y = self.actual_twist[1]
            odom.twist.twist.angular.z = self.actual_twist[2]
            object_position, object_orientation = self._mujoco_body_pose(
                self.data, self.object_body_id
            )
            observation = self._object_observation(stamp, object_position, object_orientation)
            grasp = self._grasp_status(stamp, object_position)
            simulation_diagnostic = {
                "schema_version": 1,
                "source": "mujoco_diagnostic_only_NOT_PHYSICAL",
                "stamp": {"sec": int(stamp.sec), "nanosec": int(stamp.nanosec)},
                "simulation_time_sec": float(self.data.time),
                "object_id": str(self.get_parameter("object_id").value),
                "object_pose": {
                    "frame_id": "map",
                    "position_m": [float(value) for value in object_position],
                    "orientation_xyzw": [float(value) for value in object_orientation],
                },
                "joint_state": {
                    "names": list(joint.name),
                    "position_rad_or_m": [float(value) for value in joint.position],
                    "velocity_rad_or_m_s": [float(value) for value in joint.velocity],
                },
                "attached_arm": self.attached_arm,
                "control_ack": None if self.control_mode is None else {
                    "mode": int(self.control_mode.mode),
                    "epoch": int(self.control_mode.epoch),
                    "inputs_healthy": bool(self.control_mode.inputs_healthy),
                    "fresh": time.monotonic() - self.control_mode_receipt <= 0.5,
                    "arm_control_valid": self._arm_control_valid(),
                    "place_control_valid": self._place_guard_valid(),
                },
                "object_velocity_linear_angular": [float(value) for value in self.data.qvel[
                    int(self.model.jnt_dofadr[self.joint_ids['thesis_object_free']]):
                    int(self.model.jnt_dofadr[self.joint_ids['thesis_object_free']]) + 6]],
                "delivery_support_contact": any(
                    {int(contact.geom1), int(contact.geom2)} == {
                        self._id(mujoco.mjtObj.mjOBJ_GEOM, 'thesis_object_geom'), self._delivery_geometry()[0]}
                    and -0.001 <= contact.dist <= 0.001 for contact in self.data.contact[:self.data.ncon]),
                "contacts": self._diagnostic_contact_pairs(),
                "safety_stop": deepcopy(self.safety_stop),
            }
            place = deepcopy(self.place_status)
            place_valid = place is not None and time.monotonic() <= self.place_status_until
            base_pose = tuple(self.base_pose)
            if self.viewer is not None and self.viewer.is_running():
                self.viewer.sync()
        self.clock_pub.publish(clock)
        self.joint_pub.publish(joint)
        self.odom_pub.publish(odom)
        self.observation_pub.publish(observation)
        self.grasp_pub.publish(grasp)
        diagnostic = String()
        diagnostic.data = json.dumps(simulation_diagnostic, sort_keys=True)
        self.diagnostic_state_pub.publish(diagnostic)
        if place_valid:
            self.place_pub.publish(place)
        self._publish_tf(stamp, base_pose)

    def _object_observation(self, stamp, position, orientation) -> ObjectObservation:
        message = ObjectObservation()
        message.header.stamp = deepcopy(stamp)
        message.header.frame_id = "map"
        message.object_id = str(self.get_parameter("object_id").value)
        noise = float(self.get_parameter("observation_position_noise_std_m").value)
        values = [position[i] + self.random.gauss(0.0, noise) for i in range(3)]
        message.pose.pose.position.x, message.pose.pose.position.y, \
            message.pose.pose.position.z = values
        message.pose.pose.orientation.x, message.pose.pose.orientation.y, \
            message.pose.pose.orientation.z, message.pose.pose.orientation.w = orientation
        variance = max(noise * noise, 1.0e-6)
        message.pose.covariance[0] = variance
        message.pose.covariance[7] = variance
        message.pose.covariance[14] = variance
        message.confidence = (
            0.0 if self.occluded else float(self.get_parameter("observation_confidence").value)
        )
        message.visible = not self.occluded
        self.sequence += 1
        message.sequence = self.sequence
        message.source = "mujoco_ground_truth_oracle_NOT_PHYSICAL"
        return message

    def _grasp_status(self, stamp, object_position) -> GraspStatus:
        message = GraspStatus()
        message.header.stamp = deepcopy(stamp)
        message.header.frame_id = "map"
        message.object_id = str(self.get_parameter("object_id").value)
        if self.latest_memory is not None and self.latest_memory.object_id == message.object_id:
            message.map_version = self.latest_memory.map_version
        attached = self.attached_arm is not None
        message.contact_detected = attached
        message.gripper_closed = bool(
            attached
            and self.gripper_targets[self.attached_arm]
            <= float(self.get_parameter("gripper_closed_threshold_m").value)
        )
        message.object_retained = attached
        message.transport_safe = bool(
            attached and object_position[2] > 0.20 and self.safety_stop is None
        )
        # Attachment now requires the object to sit inside the closed jaws, so
        # `attached` is real geometry instead of a proximity proxy. Re-check the
        # live pose rather than the stored grasp-time offset so a displaced
        # object cannot keep reporting `verified=true`.
        inside_jaws = False
        live_offset = None
        if attached:
            hand_position, hand_orientation = self._mujoco_body_pose(
                self.data, self.hand_body_ids[self.attached_arm]
            )
            object_position_now, object_orientation_now = self._mujoco_body_pose(
                self.data, self.object_body_id
            )
            live_offset, _ = relative_pose(
                hand_position,
                hand_orientation,
                object_position_now,
                object_orientation_now,
            )
            inside_jaws = not self.attach_rejection(live_offset, self.attach_geometry)
        message.verified = bool(
            message.contact_detected and message.gripper_closed and inside_jaws
            and self.safety_stop is None
        )
        message.detail = (
            f"MuJoCo jaw-bracketed attachment on {self.attached_arm} TCP "
            f"(object in TCP frame {tuple(round(float(v), 4) for v in live_offset)})"
            if attached
            else "MuJoCo object is not attached"
        )
        return message

    def _publish_tf(self, stamp, base_pose) -> None:
        # The SRDF declares the chassis as
        #   <virtual_joint name="base_planar_joint" type="planar"
        #                  parent_frame="odom" child_link="base_link"/>
        # A planar virtual joint has x, y and yaw only, so MoveIt models
        # base_link in the *same plane* as odom and converts poses between the
        # two frames with zero z offset. The chassis does ride
        # `base_link_height` (0.17 m) above the floor, so that fixed offset must
        # live in map -> odom: keeping it in odom -> base_link made MoveIt add
        # 0.17 m to every base_link IK target (measured with /compute_ik: a
        # base_link target at z = 0.6739 produced a TCP at z = 0.8439, i.e. the
        # arm missed the object by exactly the mount height). The composite
        # map -> base_link transform is unchanged either way.
        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = deepcopy(stamp)
        map_to_odom.header.frame_id = "map"
        map_to_odom.child_frame_id = "odom"
        map_to_odom.transform.translation.z = self.base_link_height
        map_to_odom.transform.rotation.w = 1.0
        odom_to_base = TransformStamped()
        odom_to_base.header.stamp = deepcopy(stamp)
        odom_to_base.header.frame_id = "odom"
        odom_to_base.child_frame_id = "base_link"
        odom_to_base.transform.translation.x = base_pose[0]
        odom_to_base.transform.translation.y = base_pose[1]
        odom_to_base.transform.translation.z = 0.0
        q = yaw_to_quaternion(base_pose[2])
        odom_to_base.transform.rotation.x, odom_to_base.transform.rotation.y, \
            odom_to_base.transform.rotation.z, odom_to_base.transform.rotation.w = q
        self.tf_broadcaster.sendTransform([map_to_odom, odom_to_base])

    def destroy_node(self):
        self.stop_event.set()
        if hasattr(self, "sim_thread"):
            self.sim_thread.join(timeout=2.0)
        if getattr(self, "camera_thread", None) is not None:
            self.camera_thread.join(timeout=5.0)
        if getattr(self, "viewer", None) is not None:
            self.viewer.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = MujocoBridgeNode()
        executor = MultiThreadedExecutor(num_threads=8)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
