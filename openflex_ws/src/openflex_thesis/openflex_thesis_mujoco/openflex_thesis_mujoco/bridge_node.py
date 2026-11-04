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
    FixedPlaceControl,
    GraspStatus,
    ObjectMemory,
    ObjectObservation,
    PlaceStatus,
)
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from .math_utils import clamp, compose_pose, normalize_angle, relative_pose, yaw_to_quaternion
from .model_builder import build_thesis_model
from .trajectory import TrajectoryPoint, TrajectorySampler, duration_to_seconds

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
        "d435_color_optical_frame",
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
        self.gripper_targets = {"left": 0.04, "right": 0.04}
        self.attached_arm: Optional[str] = None
        self.attachment_offset = None
        self.latest_memory: Optional[ObjectMemory] = None
        self.occluded = False
        self.current_object_nominal = [1.12, 0.0, 0.84, math.pi]
        self.place_status: Optional[PlaceStatus] = None
        self.place_status_until = 0.0
        self.last_publish_stamp_ns = 0

        self._cache_model_indices()
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
            "inactive_arm_park_positions": [0.0, -0.15, 0.0, 0.0, 0.0, 0.0, 0.0],
            "inactive_gripper_open_position": 0.04,
            "object_nominal_pose": [1.12, 0.0, 0.84, math.pi],
            "object_randomization_xy_m": [0.08, 0.18],
            "object_randomization_yaw_rad": 0.20,
            "occlusion_probability": 0.0,
            "observation_position_noise_std_m": 0.0,
            "observation_confidence": 0.99,
            "random_seed": 7,
            "grasp_attach_distance_m": 0.13,
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
                    goal_callback=lambda _: GoalResponse.ACCEPT,
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
            self.commanded_twist = [
                clamp(float(msg.linear.x), -maximum_linear, maximum_linear),
                clamp(float(msg.linear.y), -maximum_linear, maximum_linear),
                clamp(float(msg.angular.z), -maximum_angular, maximum_angular),
            ]
            self.last_twist_wall = time.monotonic()

    def _on_memory(self, msg: ObjectMemory) -> None:
        self.latest_memory = deepcopy(msg)

    def _trajectory_goal(self, request, controller: str) -> GoalResponse:
        return (
            GoalResponse.ACCEPT
            if tuple(request.trajectory.joint_names) == CONTROLLERS[controller]
            and bool(request.trajectory.points)
            else GoalResponse.REJECT
        )

    def _execute_trajectory(self, goal_handle, controller: str):
        result = FollowJointTrajectory.Result()
        goal = goal_handle.request
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
            while time.monotonic() - started <= wall_timeout:
                if goal_handle.is_cancel_requested:
                    with self.lock:
                        current = self.trajectories.get(controller)
                        if current is not None and current[0] == token:
                            self.trajectories.pop(controller, None)
                    goal_handle.canceled()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = "trajectory canceled"
                    return result
                with self.lock:
                    done = self.trajectory_done.get(token, False)
                if done:
                    goal_handle.succeed()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = "MuJoCo position trajectory completed"
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

    def _execute_gripper(self, goal_handle, arm: str):
        result = GripperCommand.Result()
        target = clamp(float(goal_handle.request.command.position), 0.0, 0.044)
        with self.lock:
            self.gripper_targets[arm] = target
            self.data.ctrl[self.actuator_ids[GRIPPER_JOINTS[arm]]] = target
        tolerance = float(self.get_parameter("gripper_goal_tolerance_m").value)
        timeout = float(self.get_parameter("gripper_action_timeout_sec").value)
        started = time.monotonic()
        while time.monotonic() - started <= timeout:
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
        if self.attached_arm is not None:
            return
        closed_threshold = float(self.get_parameter("gripper_closed_threshold_m").value)
        attach_distance = float(self.get_parameter("grasp_attach_distance_m").value)
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
            if distance <= attach_distance:
                self.attached_arm = arm
                self.attachment_offset = relative_pose(
                    hand_position, hand_orientation, object_position, object_orientation
                )
                self.get_logger().info(
                    f"simulated grasp attached {self.get_parameter('object_id').value} "
                    f"to {arm} TCP at distance {distance:.3f} m"
                )
                break

    def _step_attachment(self) -> None:
        if self.attached_arm is None or self.attachment_offset is None:
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

    def _simulation_loop(self) -> None:
        dt = float(self.model.opt.timestep)
        real_time_factor = max(0.01, float(self.get_parameter("real_time_factor").value))
        wall_step = dt / real_time_factor
        deadline = time.monotonic()
        while not self.stop_event.is_set():
            with self.lock:
                self._step_base(dt)
                self._step_trajectories()
                for arm, joint in GRIPPER_JOINTS.items():
                    self.data.ctrl[self.actuator_ids[joint]] = self.gripper_targets[arm]
                mujoco.mj_step(self.model, self.data)
                self._try_attach()
                self._step_attachment()
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

    def _on_place_control(self, msg: FixedPlaceControl) -> None:
        if not msg.enabled:
            return
        status = PlaceStatus()
        with self.lock:
            status.header.stamp = self._simulation_stamp()
            status.header.frame_id = msg.delivery_pose.header.frame_id or "map"
            status.object_id = msg.object_id
            if self.attached_arm is None:
                status.failure_code = 91
                status.detail = "no object is attached in MuJoCo"
            else:
                pose = msg.delivery_pose.pose
                z = float(pose.position.z)
                if z <= 0.05:
                    z = float(self.get_parameter("delivery_surface_z_m").value)
                q = pose.orientation
                orientation = (q.x, q.y, q.z, q.w)
                if sum(value * value for value in orientation) < 0.5:
                    orientation = (0.0, 0.0, 0.0, 1.0)
                self.attached_arm = None
                self.attachment_offset = None
                self._set_free_joint_pose(
                    "thesis_object_free",
                    (pose.position.x, pose.position.y, z),
                    orientation,
                )
                arm = "left" if int(msg.active_arm) == 1 else "right"
                self.gripper_targets[arm] = 0.04
                mujoco.mj_forward(self.model, self.data)
                status.target_reached = True
                status.released = True
                status.verified = True
                status.detail = "MuJoCo fixed-placement release verified"
            self.place_status = status
            self.place_status_until = time.monotonic() + 2.0

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
        if place_valid:
            place.header.stamp = deepcopy(stamp)
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
        message.transport_safe = bool(attached and object_position[2] > 0.20)
        message.verified = bool(message.contact_detected and message.gripper_closed)
        message.detail = (
            f"MuJoCo deterministic attachment on {self.attached_arm} TCP"
            if attached
            else "MuJoCo object is not attached"
        )
        return message

    def _publish_tf(self, stamp, base_pose) -> None:
        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = deepcopy(stamp)
        map_to_odom.header.frame_id = "map"
        map_to_odom.child_frame_id = "odom"
        map_to_odom.transform.rotation.w = 1.0
        odom_to_base = TransformStamped()
        odom_to_base.header.stamp = deepcopy(stamp)
        odom_to_base.header.frame_id = "odom"
        odom_to_base.child_frame_id = "base_link"
        odom_to_base.transform.translation.x = base_pose[0]
        odom_to_base.transform.translation.y = base_pose[1]
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
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = MujocoBridgeNode()
        executor = MultiThreadedExecutor(num_threads=8)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
