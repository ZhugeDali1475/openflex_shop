"""MuJoCo physics, controller and RGB-D bridge for the supermarket competition."""

from copy import deepcopy
import math
import threading
import time
import uuid

from builtin_interfaces.msg import Time
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TransformStamped, Twist
import mujoco
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from .competition_model_builder import build_competition_model


LEFT_ARM_JOINTS = tuple(f"openarmx_left_joint{index}" for index in range(1, 8))
RIGHT_ARM_JOINTS = tuple(f"openarmx_right_joint{index}" for index in range(1, 8))
GRIPPER_JOINTS = {
    "left": "openarmx_left_finger_joint1",
    "right": "openarmx_right_finger_joint1",
}
CONTROLLERS = {
    "lift": ("lift_joint",),
    "left_arm": LEFT_ARM_JOINTS,
    "right_arm": RIGHT_ARM_JOINTS,
}
CAMERAS = {
    # The competition model uses the chassis D435 as the global/DREAM camera.
    # These frames already exist in the authoritative robot description.
    "head": (
        "competition_head_camera",
        ("/camera",),
        "d435_color_optical_frame",
    ),
    "left": (
        "competition_left_wrist_camera",
        ("/cam_left", "/camera/left_camera"),
        "openarmx_left_link7_pico",
    ),
    "right": (
        "competition_right_wrist_camera",
        ("/cam_right", "/camera/right_camera"),
        "openarmx_right_link7_pico",
    ),
}


def _clamp(value, low, high):
    return max(low, min(high, value))


def _normalize_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def _duration_seconds(duration):
    return float(duration.sec) + float(duration.nanosec) * 1.0e-9


def _quat_conjugate(quaternion):
    w, x, y, z = quaternion
    return np.array((w, -x, -y, -z), dtype=np.float64)


def _quat_multiply(first, second):
    aw, ax, ay, az = first
    bw, bx, by, bz = second
    return np.array(
        (
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ),
        dtype=np.float64,
    )


def _quat_rotate(quaternion, vector):
    pure = np.array((0.0, *vector), dtype=np.float64)
    return _quat_multiply(
        _quat_multiply(quaternion, pure), _quat_conjugate(quaternion)
    )[1:]


def _depth_to_uint16_mm(depth):
    """Convert MuJoCo metre depth to the RealSense-compatible 16UC1 contract."""
    depth = np.asarray(depth, dtype=np.float32)
    valid = np.isfinite(depth) & (depth > 0.0) & (depth <= 65.535)
    output = np.zeros(depth.shape, dtype=np.uint16)
    output[valid] = np.rint(depth[valid] * 1000.0).astype(np.uint16)
    return np.ascontiguousarray(output)


class CompetitionPhysicsBridge(Node):
    def __init__(self):
        super().__init__("openarmx_supermarket_physics")
        self._declare_parameters()
        model_path = build_competition_model(
            str(self.get_parameter("robot_model_path").value),
            str(self.get_parameter("scene_model_path").value),
            str(self.get_parameter("combined_model_path").value),
        )
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.model.opt.timestep = float(self.get_parameter("timestep_sec").value)
        self.data = mujoco.MjData(self.model)
        self.render_qpos = self.data.qpos.copy()
        self.render_qvel = self.data.qvel.copy()
        self.render_time = 0.0
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.callback_group = ReentrantCallbackGroup()
        self.commanded_twist = [0.0, 0.0, 0.0]
        self.actual_twist = [0.0, 0.0, 0.0]
        self.base_pose = [float(value) for value in self.get_parameter("initial_base_pose").value]
        self.last_twist_wall = 0.0
        self.trajectories = {}
        self.trajectory_done = {}
        self.gripper_targets = {"left": 0.04, "right": 0.04}
        self.attached = None
        self.attachment_pose = None
        self.collision_count = 0
        self.last_collision_pair = ""
        self.last_collision_wall = 0.0
        self._cache_indices()
        self._create_interfaces()
        self._reset_physics()
        self.renderers = {}
        self.camera_thread = None
        self.viewer = None
        if bool(self.get_parameter("enable_viewer").value):
            from mujoco import viewer

            self.viewer = viewer.launch_passive(self.model, self.data)
        self.sim_thread = threading.Thread(target=self._simulation_loop, daemon=True)
        self.sim_thread.start()
        self.create_timer(
            1.0 / float(self.get_parameter("state_rate_hz").value), self._publish_state
        )
        if bool(self.get_parameter("publish_rgbd").value):
            self.camera_thread = threading.Thread(
                target=self._camera_loop, name="mujoco-rgbd", daemon=True
            )
            self.camera_thread.start()
        self.get_logger().info(
            f"MuJoCo competition physics ready: {model_path}; "
            f"bodies={self.model.nbody}, products={len(self.product_bodies)}"
        )

    def _declare_parameters(self):
        values = {
            "robot_model_path": "",
            "scene_model_path": "",
            "combined_model_path": "/tmp/openarmx_supermarket_competition.xml",
            "timestep_sec": 0.005,
            "real_time_factor": 1.0,
            "state_rate_hz": 50.0,
            "camera_rate_hz": 5.0,
            "image_width": 320,
            "image_height": 240,
            "publish_rgbd": True,
            "enable_viewer": False,
            "initial_base_pose": [0.0, -3.50, 1.57079632679],
            "cmd_vel_topic": "/cmd_vel",
            "command_timeout_sec": 0.3,
            "max_linear_mps": 0.50,
            "max_angular_rps": 0.80,
            "grasp_attach_distance_m": 0.14,
            "collision_debounce_sec": 0.5,
        }
        for name, value in values.items():
            self.declare_parameter(name, value)

    def _id(self, object_type, name):
        identifier = int(mujoco.mj_name2id(self.model, object_type, name))
        if identifier < 0:
            raise ValueError(f"MuJoCo model is missing {name}")
        return identifier

    def _cache_indices(self):
        scalar_names = []
        self.joint_ids = {}
        for joint_id in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
            self.joint_ids[name] = joint_id
            if int(self.model.jnt_type[joint_id]) in (
                int(mujoco.mjtJoint.mjJNT_HINGE),
                int(mujoco.mjtJoint.mjJNT_SLIDE),
            ) and name not in ("base_x", "base_y", "base_yaw"):
                scalar_names.append(name)
        self.scalar_joint_names = scalar_names
        controlled = (
            "lift_joint",
            *LEFT_ARM_JOINTS,
            *RIGHT_ARM_JOINTS,
            *GRIPPER_JOINTS.values(),
        )
        self.actuator_ids = {
            name: self._id(mujoco.mjtObj.mjOBJ_ACTUATOR, f"{name}_pos")
            for name in controlled
        }
        self.hand_body_ids = {
            arm: self._id(mujoco.mjtObj.mjOBJ_BODY, f"openarmx_{arm}_hand_tcp")
            for arm in ("left", "right")
        }
        self.product_bodies = {}
        for body_id in range(self.model.nbody):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body_id) or ""
            if name.startswith("product_"):
                joint_id = int(self.model.body_jntadr[body_id])
                if joint_id >= 0 and int(self.model.jnt_type[joint_id]) == int(
                    mujoco.mjtJoint.mjJNT_FREE
                ):
                    self.product_bodies[name] = (body_id, joint_id)

    def _create_interfaces(self):
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 20)
        self.odom_pub = self.create_publisher(Odometry, "/odom_safe", 20)
        self.raw_odom_pub = self.create_publisher(
            Odometry, "/swerve_drive_controller/odom", 20
        )
        self.event_pub = self.create_publisher(String, "/sim_competition/physics_events", 20)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Twist, str(self.get_parameter("cmd_vel_topic").value), self._on_twist, 20
        )
        self.create_service(Trigger, "/sim_competition/reset_physics", self._reset_service)
        self.trajectory_servers = []
        for controller, action_name in (
            ("lift", "/lift_joint_controller/follow_joint_trajectory"),
            ("left_arm", "/left_joint_trajectory_controller/follow_joint_trajectory"),
            ("right_arm", "/right_joint_trajectory_controller/follow_joint_trajectory"),
        ):
            self.trajectory_servers.append(
                ActionServer(
                    self,
                    FollowJointTrajectory,
                    action_name,
                    execute_callback=lambda goal, key=controller: self._execute_trajectory(
                        goal, key
                    ),
                    goal_callback=lambda request, key=controller: self._trajectory_goal(
                        request, key
                    ),
                    cancel_callback=lambda _goal: CancelResponse.ACCEPT,
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
                    execute_callback=lambda goal, key=arm: self._execute_gripper(goal, key),
                    goal_callback=lambda _request: GoalResponse.ACCEPT,
                    cancel_callback=lambda _goal: CancelResponse.ACCEPT,
                    callback_group=self.callback_group,
                )
            )
        self.camera_publishers = {}
        for key, (_camera, prefixes, frame) in CAMERAS.items():
            self.camera_publishers[key] = {
                "frame": frame,
                "color": [
                    self.create_publisher(Image, f"{prefix}/color/image_raw", 5)
                    for prefix in prefixes
                ],
                "info": [
                    self.create_publisher(CameraInfo, f"{prefix}/color/camera_info", 5)
                    for prefix in prefixes
                ],
                "depth": [
                    self.create_publisher(
                        Image, f"{prefix}/aligned_depth_to_color/image_raw", 5
                    )
                    for prefix in prefixes
                ],
            }
        # DREAM/RTAB-Map deployments use both aligned-depth spellings.
        self.head_depth_alias_pub = self.create_publisher(
            Image, "/camera/depth/image_raw", 5
        )
        self.moveit_depth_pub = self.create_publisher(
            Image, "/head_mount_d435/depth_registered/image_raw", 5
        )

    def _joint_position(self, name):
        joint_id = self.joint_ids[name]
        return float(self.data.qpos[int(self.model.jnt_qposadr[joint_id])])

    def _reset_physics(self):
        with self.lock:
            mujoco.mj_resetData(self.model, self.data)
            self.actual_twist = [0.0, 0.0, 0.0]
            self.commanded_twist = [0.0, 0.0, 0.0]
            self.trajectories.clear()
            self.trajectory_done.clear()
            self.attached = None
            self.attachment_pose = None
            for index, name in enumerate(("base_x", "base_y", "base_yaw")):
                joint_id = self.joint_ids[name]
                self.data.qpos[int(self.model.jnt_qposadr[joint_id])] = self.base_pose[index]
            for name, actuator_id in self.actuator_ids.items():
                target = 0.04 if "finger_joint1" in name else self._joint_position(name)
                self.data.ctrl[actuator_id] = target
            mujoco.mj_forward(self.model, self.data)
            self._update_render_snapshot()

    def _reset_service(self, _request, response):
        self._reset_physics()
        response.success = True
        response.message = "MuJoCo competition physics reset"
        return response

    def _on_twist(self, message):
        linear = float(self.get_parameter("max_linear_mps").value)
        angular = float(self.get_parameter("max_angular_rps").value)
        with self.lock:
            self.commanded_twist = [
                _clamp(float(message.linear.x), -linear, linear),
                _clamp(float(message.linear.y), -linear, linear),
                _clamp(float(message.angular.z), -angular, angular),
            ]
            self.last_twist_wall = time.monotonic()

    def _trajectory_goal(self, request, controller):
        return (
            GoalResponse.ACCEPT
            if tuple(request.trajectory.joint_names) == CONTROLLERS[controller]
            and request.trajectory.points
            else GoalResponse.REJECT
        )

    def _execute_trajectory(self, goal_handle, controller):
        result = FollowJointTrajectory.Result()
        request = goal_handle.request
        times = [_duration_seconds(point.time_from_start) for point in request.trajectory.points]
        if not times or any(right <= left for left, right in zip(times, times[1:])):
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "trajectory times must be strictly increasing"
            return result
        with self.lock:
            start = tuple(self._joint_position(name) for name in request.trajectory.joint_names)
            token = uuid.uuid4().hex
            points = [
                tuple(float(value) for value in point.positions)
                for point in request.trajectory.points
            ]
            self.trajectories[controller] = (
                token,
                tuple(request.trajectory.joint_names),
                start,
                tuple(times),
                tuple(points),
                float(self.data.time),
            )
            self.trajectory_done[token] = False
        timeout = times[-1] / max(
            0.01, float(self.get_parameter("real_time_factor").value)
        ) + 5.0
        started = time.monotonic()
        while time.monotonic() - started < timeout:
            if goal_handle.is_cancel_requested:
                with self.lock:
                    self.trajectories.pop(controller, None)
                goal_handle.canceled()
                return result
            with self.lock:
                if self.trajectory_done.get(token, False):
                    goal_handle.succeed()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    return result
            time.sleep(0.01)
        goal_handle.abort()
        result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
        result.error_string = "MuJoCo trajectory timeout"
        return result

    def _execute_gripper(self, goal_handle, arm):
        target = _clamp(float(goal_handle.request.command.position), 0.0, 0.044)
        with self.lock:
            self.gripper_targets[arm] = target
            if target >= 0.025 and self.attached and self.attached[0] == arm:
                self.attached = None
                self.attachment_pose = None
        started = time.monotonic()
        result = GripperCommand.Result()
        while time.monotonic() - started < 4.0:
            with self.lock:
                position = self._joint_position(GRIPPER_JOINTS[arm])
            if abs(position - target) < 0.004:
                goal_handle.succeed()
                result.position = position
                result.reached_goal = True
                return result
            time.sleep(0.01)
        goal_handle.abort()
        result.position = position
        return result

    def _step_base(self, dt):
        command = self.commanded_twist
        if time.monotonic() - self.last_twist_wall > float(
            self.get_parameter("command_timeout_sec").value
        ):
            command = (0.0, 0.0, 0.0)
        alpha = 1.0 - math.exp(-dt / 0.08)
        for index in range(3):
            self.actual_twist[index] += alpha * (command[index] - self.actual_twist[index])
        yaw = self.base_pose[2]
        world_velocity = (
            math.cos(yaw) * self.actual_twist[0] - math.sin(yaw) * self.actual_twist[1],
            math.sin(yaw) * self.actual_twist[0] + math.cos(yaw) * self.actual_twist[1],
        )
        self.base_pose[0] += dt * world_velocity[0]
        self.base_pose[1] += dt * world_velocity[1]
        self.base_pose[2] = _normalize_angle(self.base_pose[2] + dt * self.actual_twist[2])
        for index, name in enumerate(("base_x", "base_y", "base_yaw")):
            joint_id = self.joint_ids[name]
            qpos = int(self.model.jnt_qposadr[joint_id])
            dof = int(self.model.jnt_dofadr[joint_id])
            self.data.qpos[qpos] = self.base_pose[index]
            self.data.qvel[dof] = (*world_velocity, self.actual_twist[2])[index]

    def _step_trajectories(self):
        for controller, entry in list(self.trajectories.items()):
            token, names, start, times, points, start_time = entry
            elapsed = float(self.data.time) - start_time
            if elapsed >= times[-1]:
                values = points[-1]
                done = True
            else:
                segment = next(index for index, value in enumerate(times) if value > elapsed)
                left_time = 0.0 if segment == 0 else times[segment - 1]
                left = start if segment == 0 else points[segment - 1]
                ratio = (elapsed - left_time) / (times[segment] - left_time)
                values = tuple(a + ratio * (b - a) for a, b in zip(left, points[segment]))
                done = False
            for name, value in zip(names, values):
                actuator = self.actuator_ids[name]
                low, high = self.model.actuator_ctrlrange[actuator]
                self.data.ctrl[actuator] = _clamp(value, float(low), float(high))
            if done:
                self.trajectories.pop(controller, None)
                self.trajectory_done[token] = True

    def _try_attach(self):
        if self.attached is not None:
            return
        threshold = float(self.get_parameter("grasp_attach_distance_m").value)
        for arm in ("left", "right"):
            if self.gripper_targets[arm] > 0.012:
                continue
            hand_id = self.hand_body_ids[arm]
            hand_position = np.array(self.data.xpos[hand_id], dtype=np.float64)
            nearest = None
            for name, (body_id, joint_id) in self.product_bodies.items():
                distance = float(np.linalg.norm(self.data.xpos[body_id] - hand_position))
                if distance <= threshold and (nearest is None or distance < nearest[0]):
                    nearest = (distance, name, body_id, joint_id)
            if nearest is None:
                continue
            _, name, body_id, joint_id = nearest
            hand_q = np.array(self.data.xquat[hand_id], dtype=np.float64)
            object_q = np.array(self.data.xquat[body_id], dtype=np.float64)
            rel_position = _quat_rotate(
                _quat_conjugate(hand_q), self.data.xpos[body_id] - hand_position
            )
            rel_quaternion = _quat_multiply(_quat_conjugate(hand_q), object_q)
            self.attached = (arm, name, joint_id)
            self.attachment_pose = (rel_position, rel_quaternion)
            self.event_pub.publish(String(data=f"grasp_attached:{arm}:{name}"))
            break

    def _step_attachment(self):
        if self.attached is None:
            return
        arm, _name, joint_id = self.attached
        hand_id = self.hand_body_ids[arm]
        hand_position = np.array(self.data.xpos[hand_id], dtype=np.float64)
        hand_q = np.array(self.data.xquat[hand_id], dtype=np.float64)
        rel_position, rel_quaternion = self.attachment_pose
        position = hand_position + _quat_rotate(hand_q, rel_position)
        quaternion = _quat_multiply(hand_q, rel_quaternion)
        qpos = int(self.model.jnt_qposadr[joint_id])
        dof = int(self.model.jnt_dofadr[joint_id])
        self.data.qpos[qpos:qpos + 7] = (*position, *quaternion)
        self.data.qvel[dof:dof + 6] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def _record_collisions(self):
        now = time.monotonic()
        if now - self.last_collision_wall < float(
            self.get_parameter("collision_debounce_sec").value
        ):
            return
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            first = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
            second = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
            first = first or "unnamed"
            second = second or "unnamed"
            robot_contact = first.startswith("packages_") or second.startswith("packages_")
            static_contact = any(
                token in first or token in second
                for token in ("shelf_group", "counter_", "wall_")
            )
            if robot_contact and static_contact:
                pair = f"{first}|{second}"
                self.collision_count += 1
                self.last_collision_pair = pair
                self.last_collision_wall = now
                self.event_pub.publish(String(data=f"collision:{pair}"))
                break

    def _simulation_loop(self):
        dt = float(self.model.opt.timestep)
        factor = max(0.01, float(self.get_parameter("real_time_factor").value))
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
                self._record_collisions()
                self._update_render_snapshot()
            deadline += dt / factor
            remaining = deadline - time.monotonic()
            if remaining > 0.0:
                self.stop_event.wait(remaining)
            elif remaining < -0.1:
                deadline = time.monotonic()

    def _stamp(self):
        value = max(0.0, float(self.data.time))
        stamp = Time()
        stamp.sec = int(value)
        stamp.nanosec = int((value - stamp.sec) * 1.0e9)
        return stamp

    def _publish_state(self):
        with self.lock:
            stamp = self._stamp()
            joint = JointState()
            joint.header.stamp = deepcopy(stamp)
            joint.name = list(self.scalar_joint_names)
            joint.position = [self._joint_position(name) for name in joint.name]
            odom = Odometry()
            odom.header.stamp = deepcopy(stamp)
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.pose.pose.position.x = self.base_pose[0]
            odom.pose.pose.position.y = self.base_pose[1]
            half = 0.5 * self.base_pose[2]
            odom.pose.pose.orientation.z = math.sin(half)
            odom.pose.pose.orientation.w = math.cos(half)
            odom.twist.twist.linear.x = self.actual_twist[0]
            odom.twist.twist.linear.y = self.actual_twist[1]
            odom.twist.twist.angular.z = self.actual_twist[2]
            base_pose = tuple(self.base_pose)
            if self.viewer is not None and self.viewer.is_running():
                self.viewer.sync()
        self.clock_pub.publish(Clock(clock=deepcopy(stamp)))
        self.joint_pub.publish(joint)
        self.odom_pub.publish(odom)
        self.raw_odom_pub.publish(odom)
        self._publish_tf(stamp, base_pose)

    def _publish_tf(self, stamp, base_pose):
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
        half = 0.5 * base_pose[2]
        odom_to_base.transform.rotation.z = math.sin(half)
        odom_to_base.transform.rotation.w = math.cos(half)
        self.tf_broadcaster.sendTransform((map_to_odom, odom_to_base))

    def _camera_info(self, stamp, frame, camera_name):
        width = int(self.get_parameter("image_width").value)
        height = int(self.get_parameter("image_height").value)
        camera_id = self._id(mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
        fovy = math.radians(float(self.model.cam_fovy[camera_id]))
        focal = 0.5 * height / math.tan(0.5 * fovy)
        message = CameraInfo()
        message.header.stamp = deepcopy(stamp)
        message.header.frame_id = frame
        message.width = width
        message.height = height
        message.k = [
            focal, 0.0, width / 2.0,
            0.0, focal, height / 2.0,
            0.0, 0.0, 1.0,
        ]
        message.p = [
            focal, 0.0, width / 2.0, 0.0,
            0.0, focal, height / 2.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        message.distortion_model = "plumb_bob"
        message.d = [0.0] * 5
        return message

    def _update_render_snapshot(self):
        self.render_qpos = self.data.qpos.copy()
        self.render_qvel = self.data.qvel.copy()
        self.render_time = float(self.data.time)

    @staticmethod
    def _time_from_seconds(value):
        stamp = Time()
        stamp.sec = int(value)
        stamp.nanosec = int((value - stamp.sec) * 1.0e9)
        return stamp

    def _publish_cameras(self, render_data, stamp):
        messages = []
        for key, (camera_name, _prefixes, frame) in CAMERAS.items():
            renderer = self.renderers[key]
            renderer.disable_depth_rendering()
            renderer.update_scene(render_data, camera=camera_name)
            rgb = np.ascontiguousarray(renderer.render())
            renderer.enable_depth_rendering()
            renderer.update_scene(render_data, camera=camera_name)
            depth = _depth_to_uint16_mm(renderer.render())
            color_message = Image()
            color_message.header.stamp = deepcopy(stamp)
            color_message.header.frame_id = frame
            color_message.height, color_message.width = rgb.shape[:2]
            color_message.encoding = "rgb8"
            color_message.is_bigendian = False
            color_message.step = color_message.width * 3
            color_message.data = rgb.tobytes()
            depth_message = Image()
            depth_message.header.stamp = deepcopy(stamp)
            depth_message.header.frame_id = frame
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
                    self._camera_info(stamp, frame, camera_name),
                )
            )
        for key, color, depth, info in messages:
            if not rclpy.ok():
                return
            publishers = self.camera_publishers[key]
            for publisher in publishers["color"]:
                publisher.publish(color)
            for publisher in publishers["depth"]:
                publisher.publish(depth)
            for publisher in publishers["info"]:
                publisher.publish(info)
            if key == "head":
                self.head_depth_alias_pub.publish(depth)
                self.moveit_depth_pub.publish(depth)

    def _camera_loop(self):
        width = int(self.get_parameter("image_width").value)
        height = int(self.get_parameter("image_height").value)
        period = 1.0 / float(self.get_parameter("camera_rate_hz").value)
        try:
            render_data = mujoco.MjData(self.model)
            self.renderers = {
                key: mujoco.Renderer(self.model, height=height, width=width)
                for key in CAMERAS
            }
            deadline = time.monotonic()
            while not self.stop_event.is_set() and rclpy.ok():
                qpos = self.render_qpos
                qvel = self.render_qvel
                render_time = self.render_time
                render_data.qpos[:] = qpos
                render_data.qvel[:] = qvel
                render_data.time = render_time
                mujoco.mj_forward(self.model, render_data)
                self._publish_cameras(
                    render_data, self._time_from_seconds(render_time)
                )
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

    def destroy_node(self):
        self.stop_event.set()
        if hasattr(self, "sim_thread"):
            self.sim_thread.join(timeout=2.0)
        if self.camera_thread is not None:
            self.camera_thread.join(timeout=5.0)
        if self.viewer is not None:
            self.viewer.close()
        return super().destroy_node()


def main(args=None):
    # Keep the ROS context alive until both the physics and EGL rendering
    # threads have stopped.  rclpy's default SIGINT handler shuts the context
    # down before destroy_node(), which can race an in-flight RGB-D publish.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = CompetitionPhysicsBridge()
    executor = MultiThreadedExecutor(num_threads=12)
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
