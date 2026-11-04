#!/usr/bin/env python3
"""ROS2 adapter for OpenFlex Pico relative VR teleoperation."""

import ast
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from openarmx_arm_driver import TeleopConfig
from openarmx_arm_driver._lib.teleop_core import (
    AbsoluteTeleopInputFrame,
    PinocchioTeleopCore,
    PoseInput,
    RelativeTeleopInputFrame,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Float64MultiArray


class OpenArmXTeleopVRNode(Node):
    """OpenFlex ROS wrapper around the Pinocchio relative teleop core."""

    def __init__(self):
        super().__init__("openarmx_teleop_vr_node")

        self.cb_group = ReentrantCallbackGroup()
        self._declare_parameters()

        urdf_path = self.get_parameter("urdf_path").value
        if not urdf_path:
            self.get_logger().error("URDF path not provided")
            raise ValueError("urdf_path parameter is required")

        self.control_rate = float(self.get_parameter("control_rate").value)
        self.grip_threshold = float(self.get_parameter("grip_threshold").value)
        self.resync_threshold_deg = float(self.get_parameter("resync_threshold_deg").value)
        self.ik_iterations = int(self.get_parameter("ik_iterations").value)
        self.sync_joint_states_each_cycle = bool(
            self.get_parameter("sync_joint_states_each_cycle").value
        )

        # 步长限制参数 - 逐关节配置
        max_step_joint1_2 = float(self.get_parameter("max_step_deg_joint1_2").value)
        max_step_joint3_4 = float(self.get_parameter("max_step_deg_joint3_4").value)
        max_step_joint5_7 = float(self.get_parameter("max_step_deg_joint5_7").value)

        threshold_joint1_2 = float(self.get_parameter("threshold_deg_joint1_2").value)
        threshold_joint3_4 = float(self.get_parameter("threshold_deg_joint3_4").value)
        threshold_joint5_7 = float(self.get_parameter("threshold_deg_joint5_7").value)

        # 回零模式参数
        home_step_deg = float(self.get_parameter("home_step_deg").value)
        self.max_step_rad_home = np.deg2rad(home_step_deg)

        # 构建逐关节步长限制数组（14 个关节：左臂 7 + 右臂 7）
        # 只限制 Joint 1-2，其余关节设为无穷大（不限制）
        self.max_step_rad_per_joint = np.array([
            # 左臂
            np.deg2rad(max_step_joint1_2),  # joint1 (RS04)
            np.deg2rad(max_step_joint1_2),  # joint2 (RS04)
            np.inf,  # joint3 (不限制)
            np.inf,  # joint4 (不限制)
            np.inf,  # joint5 (不限制)
            np.inf,  # joint6 (不限制)
            np.inf,  # joint7 (不限制)
            # 右臂
            np.deg2rad(max_step_joint1_2),  # joint1 (RS04)
            np.deg2rad(max_step_joint1_2),  # joint2 (RS04)
            np.inf,  # joint3 (不限制)
            np.inf,  # joint4 (不限制)
            np.inf,  # joint5 (不限制)
            np.inf,  # joint6 (不限制)
            np.inf,  # joint7 (不限制)
        ], dtype=np.float64)

        # 构建逐关节阈值数组（14 个关节：左臂 7 + 右臂 7）
        # 只对 Joint 1-2 设置阈值，其余关节设为无穷大（始终直接通过）
        self.threshold_rad_per_joint = np.array([
            # 左臂
            np.deg2rad(threshold_joint1_2),  # joint1 (RS04)
            np.deg2rad(threshold_joint1_2),  # joint2 (RS04)
            np.inf,  # joint3 (不限制)
            np.inf,  # joint4 (不限制)
            np.inf,  # joint5 (不限制)
            np.inf,  # joint6 (不限制)
            np.inf,  # joint7 (不限制)
            # 右臂
            np.deg2rad(threshold_joint1_2),  # joint1 (RS04)
            np.deg2rad(threshold_joint1_2),  # joint2 (RS04)
            np.inf,  # joint3 (不限制)
            np.inf,  # joint4 (不限制)
            np.inf,  # joint5 (不限制)
            np.inf,  # joint6 (不限制)
            np.inf,  # joint7 (不限制)
        ], dtype=np.float64)

        self.pose_mutex = threading.Lock()
        self.control_lock = threading.Lock()

        self.joint_positions = {}
        self.joint_states_received = False

        self.current_mode = None
        self._init_pose_state()

        core_config = TeleopConfig(
            urdf_path=urdf_path,
            grip_threshold=self.grip_threshold,
            body_anchor_offset=np.asarray(
                self.get_parameter("body_anchor_offset").value, dtype=np.float64
            ),
            position_scale_xyz=np.asarray(
                self.get_parameter("position_scale_xyz").value, dtype=np.float64
            ),
            left_axis_matrix=self._matrix_from_parameter("left_axis_matrix"),
            right_axis_matrix=self._matrix_from_parameter("right_axis_matrix"),
            left_orientation_matrix=self._matrix_from_parameter("left_orientation_matrix"),
            right_orientation_matrix=self._matrix_from_parameter("right_orientation_matrix"),
            slow_max_step_deg=0.0,
            fast_max_step_deg=0.0,
            resync_threshold_deg=self.resync_threshold_deg,
            ik_iterations=self.ik_iterations,
            sync_joint_states_each_cycle=self.sync_joint_states_each_cycle,
            stream_timeout_sec=0.3,
        )
        self.core = PinocchioTeleopCore(core_config)
        self.get_logger().info("Pinocchio relative teleop core initialized successfully")

        self._setup_ros_io()
        self.timer = self.create_timer(
            1.0 / self.control_rate,
            self._control_loop,
            callback_group=self.cb_group,
        )
        self.get_logger().info(f"Node initialized - control rate: {self.control_rate} Hz")

    def _declare_parameters(self):
        self.declare_parameter("urdf_path", "")
        self.declare_parameter("control_rate", 100.0)
        self.declare_parameter("grip_threshold", 0.5)
        self.declare_parameter("resync_threshold_deg", 5.0)
        self.declare_parameter("ik_iterations", 3)
        self.declare_parameter("sync_joint_states_each_cycle", True)

        # 步长限制参数 - 逐关节配置
        # 根据电机型号设置不同的步长限制
        self.declare_parameter("max_step_deg_joint1_2", 8.0)  # RS04 大扭矩电机
        self.declare_parameter("max_step_deg_joint3_4", 5.0)  # RS03 中扭矩电机：激进
        self.declare_parameter("max_step_deg_joint5_7", 5.0)  # RS00 小扭矩电机：激进

        # 逐关节阈值参数 - 根据电机型号设置不同的启用阈值
        self.declare_parameter("threshold_deg_joint1_2", 4.0)   # RS04 大关节：较小阈值
        self.declare_parameter("threshold_deg_joint3_4", 12.0)  # RS03 中关节：较大阈值
        self.declare_parameter("threshold_deg_joint5_7", 12.0)  # RS00 小关节：较大阈值

        # 回零模式参数
        self.declare_parameter("home_step_deg", 8.0)  # 回零模式的步长限制

        self.declare_parameter("left_pose_topic", "/pico_left_controller/pose")
        self.declare_parameter("right_pose_topic", "/pico_right_controller/pose")
        self.declare_parameter("left_grip_topic", "/pico_left_controller/grip")
        self.declare_parameter("right_grip_topic", "/pico_right_controller/grip")
        self.declare_parameter("left_trigger_topic", "/pico_left_controller/trigger")
        self.declare_parameter("right_trigger_topic", "/pico_right_controller/trigger")
        self.declare_parameter("button_a_topic", "/pico_right_controller/button_a")
        self.declare_parameter("joint_states_topic", "/joint_states")

        self.declare_parameter("left_cmd_topic", "/left_forward_position_controller/commands")
        self.declare_parameter("right_cmd_topic", "/right_forward_position_controller/commands")

        self.declare_parameter("body_anchor_offset", [0.0, -0.18, 0.08])
        self.declare_parameter("position_scale_xyz", [1.0, 1.0, 0.9])
        self.declare_parameter(
            "left_axis_matrix",
            [0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        )
        self.declare_parameter(
            "right_axis_matrix",
            [0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        )
        self.declare_parameter(
            "left_orientation_matrix",
            [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        )
        self.declare_parameter(
            "right_orientation_matrix",
            [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        )

    def _init_pose_state(self):
        self.relative_left_pose = None
        self.relative_right_pose = None
        self.relative_left_grip = 0.0
        self.relative_right_grip = 0.0
        self.relative_left_trigger = 0.0
        self.relative_right_trigger = 0.0
        self.button_a_pressed = False
        self.homing_mode = False  # 回零模式标志
        self.returning_home = False  # A键回零状态

    def _matrix_from_parameter(self, name: str) -> np.ndarray:
        raw_value = self.get_parameter(name).value
        if isinstance(raw_value, str):
            raw_value = ast.literal_eval(raw_value)
        values = np.asarray(list(raw_value), dtype=np.float64)
        if values.size != 9:
            raise ValueError(f"{name} must contain 9 values, got {values.size}")
        return values.reshape(3, 3)

    def _pose_from_msg(self, msg: PoseStamped) -> PoseInput:
        position = np.array(
            [
                float(msg.pose.position.x),
                float(msg.pose.position.y),
                float(msg.pose.position.z),
            ],
            dtype=np.float64,
        )
        orientation_xyzw = (
            float(msg.pose.orientation.x),
            float(msg.pose.orientation.y),
            float(msg.pose.orientation.z),
            float(msg.pose.orientation.w),
        )
        timestamp = time.monotonic()
        try:
            return PoseInput(
                position=position,
                orientation_xyzw=orientation_xyzw,
                timestamp=timestamp,
            )
        except TypeError:
            pose = PoseInput()
            pose.position = position
            pose.orientation_xyzw = orientation_xyzw
            pose.timestamp = timestamp
            return pose

    @staticmethod
    def _make_relative_input(
        left_pose,
        right_pose,
        left_grip,
        right_grip,
        left_trigger,
        right_trigger,
    ) -> RelativeTeleopInputFrame:
        try:
            return RelativeTeleopInputFrame(
                left_pose=left_pose,
                right_pose=right_pose,
                left_grip=left_grip,
                right_grip=right_grip,
                left_trigger=left_trigger,
                right_trigger=right_trigger,
            )
        except TypeError:
            frame = RelativeTeleopInputFrame()
            frame.left_pose = left_pose
            frame.right_pose = right_pose
            frame.left_grip = left_grip
            frame.right_grip = right_grip
            frame.left_trigger = left_trigger
            frame.right_trigger = right_trigger
            return frame

    @staticmethod
    def _make_absolute_input(
        head_pose,
        left_pose,
        right_pose,
        left_grip,
        right_grip,
        left_trigger,
        right_trigger,
        mode_selected,
    ) -> AbsoluteTeleopInputFrame:
        try:
            return AbsoluteTeleopInputFrame(
                head_pose=head_pose,
                left_pose=left_pose,
                right_pose=right_pose,
                left_grip=left_grip,
                right_grip=right_grip,
                left_trigger=left_trigger,
                right_trigger=right_trigger,
                mode_selected=mode_selected,
            )
        except TypeError:
            frame = AbsoluteTeleopInputFrame()
            frame.head_pose = head_pose
            frame.left_pose = left_pose
            frame.right_pose = right_pose
            frame.left_grip = left_grip
            frame.right_grip = right_grip
            frame.left_trigger = left_trigger
            frame.right_trigger = right_trigger
            frame.mode_selected = mode_selected
            return frame

    def _setup_ros_io(self):
        p = self.get_parameter

        self.relative_left_pose_sub = self.create_subscription(
            PoseStamped,
            p("left_pose_topic").value,
            self._relative_left_pose_callback,
            10,
            callback_group=self.cb_group,
        )
        self.relative_right_pose_sub = self.create_subscription(
            PoseStamped,
            p("right_pose_topic").value,
            self._relative_right_pose_callback,
            10,
            callback_group=self.cb_group,
        )
        self.relative_left_grip_sub = self.create_subscription(
            Float32,
            p("left_grip_topic").value,
            self._relative_left_grip_callback,
            10,
            callback_group=self.cb_group,
        )
        self.relative_right_grip_sub = self.create_subscription(
            Float32,
            p("right_grip_topic").value,
            self._relative_right_grip_callback,
            10,
            callback_group=self.cb_group,
        )
        self.relative_left_trigger_sub = self.create_subscription(
            Float32,
            p("left_trigger_topic").value,
            self._relative_left_trigger_callback,
            10,
            callback_group=self.cb_group,
        )
        self.relative_right_trigger_sub = self.create_subscription(
            Float32,
            p("right_trigger_topic").value,
            self._relative_right_trigger_callback,
            10,
            callback_group=self.cb_group,
        )
        self.button_a_sub = self.create_subscription(
            Bool,
            p("button_a_topic").value,
            self._button_a_callback,
            10,
            callback_group=self.cb_group,
        )
        self.joint_state_sub = self.create_subscription(
            JointState,
            p("joint_states_topic").value,
            self._joint_state_callback,
            10,
            callback_group=self.cb_group,
        )

        self.left_cmd_pub = self.create_publisher(
            Float64MultiArray, p("left_cmd_topic").value, 10
        )
        self.right_cmd_pub = self.create_publisher(
            Float64MultiArray, p("right_cmd_topic").value, 10
        )
        self.get_logger().info("ROS2 relative teleop interfaces configured")

    def _relative_left_pose_callback(self, msg: PoseStamped):
        with self.pose_mutex:
            self.relative_left_pose = self._pose_from_msg(msg)

    def _relative_right_pose_callback(self, msg: PoseStamped):
        with self.pose_mutex:
            self.relative_right_pose = self._pose_from_msg(msg)

    def _relative_left_grip_callback(self, msg: Float32):
        self.relative_left_grip = max(0.0, min(1.0, float(msg.data)))

    def _relative_right_grip_callback(self, msg: Float32):
        self.relative_right_grip = max(0.0, min(1.0, float(msg.data)))

    def _relative_left_trigger_callback(self, msg: Float32):
        self.relative_left_trigger = max(0.0, min(1.0, float(msg.data)))

    def _relative_right_trigger_callback(self, msg: Float32):
        self.relative_right_trigger = max(0.0, min(1.0, float(msg.data)))

    def _button_a_callback(self, msg: Bool):
        # 边沿检测：只在按键从未按下变为按下时触发
        if msg.data and not self.button_a_pressed:
            self.returning_home = not self.returning_home
            status = "启用" if self.returning_home else "禁用"
            self.get_logger().info(f"A键按下 - 回零位模式已{status}")
        self.button_a_pressed = msg.data

    def _joint_state_callback(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self.joint_positions[name] = float(pos)
        self.joint_states_received = True
        self.core.update_joint_states(msg.name, msg.position)

    def _current_relative_input(self) -> RelativeTeleopInputFrame:
        with self.pose_mutex:
            left_pose = self.relative_left_pose
            right_pose = self.relative_right_pose
        return self._make_relative_input(
            left_pose=left_pose,
            right_pose=right_pose,
            left_grip=self.relative_left_grip,
            right_grip=self.relative_right_grip,
            left_trigger=self.relative_left_trigger,
            right_trigger=self.relative_right_trigger,
        )

    @staticmethod
    def _disabled_absolute_input() -> AbsoluteTeleopInputFrame:
        return OpenArmXTeleopVRNode._make_absolute_input(
            head_pose=None,
            left_pose=None,
            right_pose=None,
            left_grip=0.0,
            right_grip=0.0,
            left_trigger=0.0,
            right_trigger=0.0,
            mode_selected=False,
        )

    def _control_loop(self):
        if not self.control_lock.acquire(blocking=False):
            return
        try:
            # 检查是否正在回零位
            if self.returning_home:
                current_q = self._get_current_joint_positions()
                if current_q is None:
                    return

                # 目标位置：零位（14个关节全为0）
                home_target_q = np.zeros(14, dtype=np.float64)

                # 使用回零专用步长限制
                limited_arm_q = self._limit_joint_step_home(home_target_q, current_q)

                # 检查是否到达零位（所有关节误差 < 0.01 rad）
                if np.all(np.abs(limited_arm_q - current_q) < 0.01):
                    self.returning_home = False
                    self.get_logger().info("Reached home position")

                # 发布回零命令（夹爪保持当前状态）
                left_gripper = self._get_current_gripper_command("left")
                right_gripper = self._get_current_gripper_command("right")
                self._publish_joint_commands(
                    limited_arm_q[:7].tolist(),
                    limited_arm_q[7:14].tolist(),
                    left_gripper,
                    right_gripper,
                )
                return

            # 正常遥操作模式
            relative_input = self._current_relative_input()
            result = self.core.step(relative_input, self._disabled_absolute_input())
            if result.active_mode != self.current_mode:
                self.current_mode = result.active_mode
                if self.current_mode is None:
                    self.get_logger().info("Teleop mode idle - no active relative stream")
                else:
                    self.get_logger().info(f"Teleop mode switched to: {self.current_mode}")

            if result.waiting_for_joint_states or result.calibration_required or result.target_q is None:
                return

            current_q = self._get_current_joint_positions()
            if current_q is None:
                return
            target_q = np.asarray(result.target_q, dtype=np.float64)

            # 调试：打印维度信息（首次）
            if not hasattr(self, '_dimension_logged'):
                self.get_logger().info(f"target_q shape: {target_q.shape}, current_q shape: {current_q.shape}")
                self._dimension_logged = True

            # 只对机械臂的14个关节应用步长限制（忽略夹爪等额外关节）
            arm_target_q = target_q[:14]
            limited_arm_q = self._limit_joint_step(arm_target_q, current_q)

            # 如果 target_q 包含夹爪关节，保留它们
            if len(target_q) > 14:
                limited_q = np.concatenate([limited_arm_q, target_q[14:]])
            else:
                limited_q = limited_arm_q

            left_gripper = (
                self._map_trigger_to_gripper(self.relative_left_trigger)
                if result.left_active
                else self._get_current_gripper_command("left")
            )
            right_gripper = (
                self._map_trigger_to_gripper(self.relative_right_trigger)
                if result.right_active
                else self._get_current_gripper_command("right")
            )

            self._publish_joint_commands(
                limited_q[:7].tolist(),
                limited_q[7:14].tolist(),
                left_gripper,
                right_gripper,
            )
        except Exception as exc:
            self.get_logger().error(f"Control loop error: {exc}")
        finally:
            self.control_lock.release()

    def _get_current_joint_positions(self):
        if not self.joint_states_received:
            return None
        joint_names = [
            "openarmx_left_joint1",
            "openarmx_left_joint2",
            "openarmx_left_joint3",
            "openarmx_left_joint4",
            "openarmx_left_joint5",
            "openarmx_left_joint6",
            "openarmx_left_joint7",
            "openarmx_right_joint1",
            "openarmx_right_joint2",
            "openarmx_right_joint3",
            "openarmx_right_joint4",
            "openarmx_right_joint5",
            "openarmx_right_joint6",
            "openarmx_right_joint7",
        ]
        values = []
        for name in joint_names:
            current = self.joint_positions.get(name)
            if current is None:
                return None
            values.append(float(current))
        return np.asarray(values, dtype=np.float64)

    def _limit_joint_step(self, target_q, current_q):
        """
        限制相邻控制周期之间的关节步长，提高安全性和平滑性。

        逐关节配置策略：
        - Joint 1-2 (RS04): 1.5°/cycle → 150°/s (大扭矩，保守)
        - Joint 3-4 (RS03): 2.5°/cycle → 250°/s (中扭矩，平衡)
        - Joint 5-7 (RS00): 5.0°/cycle → 500°/s (小扭矩，激进)

        智能阈值策略：
        - 小误差（< threshold）：直接通过，不限制 → 高精度定位
        - 大误差（> threshold）：限制步长 → 安全平滑运动

        参数:
            target_q: IK 求解的目标关节角 (14,)
            current_q: 当前关节角 (14,)

        返回:
            限制后的目标关节角 (14,)
        """
        if current_q is None:
            return target_q

        # 计算增量
        delta = np.asarray(target_q, dtype=np.float64) - np.asarray(current_q, dtype=np.float64)

        # 逐关节智能阈值：只对大误差应用限制
        # 小误差直接通过，避免收敛慢的问题
        apply_limit_mask = np.abs(delta) > self.threshold_rad_per_joint

        # 逐关节限制（每个关节有自己的步长限制）
        limited_delta = np.clip(delta, -self.max_step_rad_per_joint, self.max_step_rad_per_joint)

        # 选择性应用：小误差直接通过，大误差限制
        delta = np.where(apply_limit_mask, limited_delta, delta)

        return np.asarray(current_q, dtype=np.float64) + delta

    def _limit_joint_step_home(self, target_q, current_q):
        """
        回零专用步长限制：使用统一的较大步长实现快速回零。

        参数:
            target_q: 目标关节角（零位） (14,)
            current_q: 当前关节角 (14,)

        返回:
            限制后的目标关节角 (14,)
        """
        if current_q is None:
            return target_q

        # 计算增量
        delta = np.asarray(target_q, dtype=np.float64) - np.asarray(current_q, dtype=np.float64)

        # 统一步长限制（所有关节使用相同的回零步长）
        limited_delta = np.clip(delta, -self.max_step_rad_home, self.max_step_rad_home)

        return np.asarray(current_q, dtype=np.float64) + limited_delta

    @staticmethod
    def _map_trigger_to_gripper(trigger_value):
        return max(0.0, min(1.0, float(trigger_value))) * 0.04

    def _publish_joint_commands(self, left_joints, right_joints, left_gripper, right_gripper):
        left_cmd = Float64MultiArray()
        left_cmd.data = list(left_joints) + [float(left_gripper)]
        self.left_cmd_pub.publish(left_cmd)

        right_cmd = Float64MultiArray()
        right_cmd.data = list(right_joints) + [float(right_gripper)]
        self.right_cmd_pub.publish(right_cmd)

    def _get_current_gripper_command(self, arm: str) -> float:
        joint_name = f"openarmx_{arm}_finger_joint1"
        current = self.joint_positions.get(joint_name)
        return 0.0 if current is None else float(current)


def main(args=None):
    rclpy.init(args=args)
    node = OpenArmXTeleopVRNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
