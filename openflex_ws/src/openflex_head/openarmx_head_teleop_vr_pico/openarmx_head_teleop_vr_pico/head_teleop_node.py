# Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
#
# Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd.
# https://www.openarmx.com
#
# This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike
# 4.0 International License (CC BY-NC-SA 4.0).
#
# To view a copy of this license, visit:
# http://creativecommons.org/licenses/by-nc-sa/4.0/
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Float64MultiArray
from tf2_ros import TransformBroadcaster


# Joint names matching head_controllers.yaml order
YAW_JOINT = 'openarmx_head_yaw_joint'
PITCH_JOINT = 'openarmx_head_pitch_joint'

# Physical joint limits from URDF (±90°)
PHYSICAL_JOINT_LIMIT = 1.5708


def quat_inverse(x, y, z, w):
    """Inverse of a unit quaternion (conjugate)."""
    return (-x, -y, -z, w)


def quat_multiply(ax, ay, az, aw, bx, by, bz, bw):
    """Hamilton product of two quaternions."""
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def euler_from_quaternion_yxz(x, y, z, w):
    """Extract yaw (Y-axis) and pitch (X-axis) from quaternion using YXZ order.

    OpenXR convention: X=right, Y=up, Z=back.
    - Rotation around Y-axis = yaw (left/right turn)
    - Rotation around X-axis = pitch (up/down nod)
    """
    # YXZ intrinsic rotation decomposition
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    # pitch from X-axis rotation
    pitch = math.atan2(sinr_cosp, cosr_cosp)

    # yaw from Y-axis rotation
    siny_cosp = 2.0 * (w * y - z * x)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return yaw, pitch


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def resolve_command_limit(physical_limit, margin_deg):
    margin_rad = math.radians(max(0.0, margin_deg))
    return max(0.0, physical_limit - margin_rad)


def apply_soft_limit(value, limit, blend_width):
    """Preserve linear response in the center and ease into the command limit."""
    limit = abs(limit)
    if limit <= 0.0:
        return 0.0

    blend_width = clamp(blend_width, 0.0, limit)
    abs_value = abs(value)

    if blend_width <= 1e-9:
        return clamp(value, -limit, limit)

    linear_limit = limit - blend_width
    if abs_value <= linear_limit:
        return value
    if abs_value >= limit:
        return math.copysign(limit, value)

    u = (abs_value - linear_limit) / blend_width
    eased = linear_limit + blend_width * (u + u * u - u * u * u)
    return math.copysign(eased, value)


class HeadTeleopNode(Node):
    def __init__(self):
        super().__init__('head_teleop_vr_pico')
        self.cb_group = ReentrantCallbackGroup()

        # --- Parameters ---
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('button_b_topic', '/pico_right_controller/button_b')
        self.declare_parameter('head_pose_topic', '/pico_head/pose')
        self.declare_parameter('rate_topic', '/pico_left_controller/rate')
        self.declare_parameter('command_topic',
                               '/head_forward_position_controller/commands')
        self.declare_parameter('slow_max_step_deg', 2.0)
        self.declare_parameter('fast_max_step_deg', 5.0)
        self.declare_parameter('yaw_scale', 1.0)
        self.declare_parameter('pitch_scale', 1.0)
        self.declare_parameter('invert_yaw', False)
        self.declare_parameter('invert_pitch', False)
        self.declare_parameter('enable_soft_limits', True)
        self.declare_parameter('yaw_soft_margin_deg', 10.0)
        self.declare_parameter('pitch_soft_margin_deg', 10.0)
        self.declare_parameter('soft_limit_blend_deg', 10.0)
        self.declare_parameter('publish_visualization_tf', False)
        self.declare_parameter('startup_home_enabled', True)
        self.declare_parameter('startup_home_step_deg', 1.0)
        self.declare_parameter('startup_home_tolerance_deg', 1.0)

        control_rate = self.get_parameter('control_rate').value
        button_b_topic = self.get_parameter('button_b_topic').value
        head_pose_topic = self.get_parameter('head_pose_topic').value
        rate_topic = self.get_parameter('rate_topic').value
        command_topic = self.get_parameter('command_topic').value

        # --- State (protected by mutex) ---
        self.mutex = threading.Lock()

        # VR head pose quaternion (x, y, z, w)
        self.vr_quat = (0.0, 0.0, 0.0, 1.0)

        # Right-hand B button toggles head relative control on/off.
        self.button_b_pressed = False
        self.button_b_toggle_requested = False

        # Rate value for speed mode
        self.rate_value = 0.1
        self.estop_active = False

        # Current joint positions from /joint_states
        self.current_yaw = 0.0
        self.current_pitch = 0.0
        self.joint_states_received = False

        # --- Relative-mode state ---
        self.control_enabled = False
        self.reference_quat = (0.0, 0.0, 0.0, 1.0)
        self.anchor_yaw = 0.0
        self.anchor_pitch = 0.0

        # Last commanded position (for step limiting)
        self.commanded_yaw = 0.0
        self.commanded_pitch = 0.0

        # Startup homing: ease the head back to zero after power-on.
        self.startup_home_enabled = bool(self.get_parameter('startup_home_enabled').value)
        self.startup_home_done = not self.startup_home_enabled

        # Control lock (non-blocking)
        self.control_lock = threading.Lock()

        # --- Subscriptions ---
        self.create_subscription(
            PoseStamped, head_pose_topic,
            self._head_pose_cb, 10,
            callback_group=self.cb_group)

        self.create_subscription(
            Bool, button_b_topic,
            self._button_b_cb, 10,
            callback_group=self.cb_group)

        self.create_subscription(
            Float32, rate_topic,
            self._rate_cb, 10,
            callback_group=self.cb_group)

        self.create_subscription(
            JointState, '/joint_states',
            self._joint_states_cb, 10,
            callback_group=self.cb_group)

        # 全局紧停订阅
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        estop_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.create_subscription(
            Bool, '/vr_estop_active',
            self._estop_cb, estop_qos,
            callback_group=self.cb_group)

        # --- Publisher ---
        self.cmd_pub = self.create_publisher(
            Float64MultiArray, command_topic, 10)

        # --- Optional TF broadcaster ---
        self.publish_tf = self.get_parameter('publish_visualization_tf').value
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # --- Control timer ---
        period = 1.0 / control_rate
        self.create_timer(period, self._control_loop,
                          callback_group=self.cb_group)

        self.get_logger().info(
            f'Head teleop node started at {control_rate} Hz, '
            f'button B: {button_b_topic}, '
            f'pose: {head_pose_topic}')

    # ------------------------------------------------------------------ callbacks
    def _head_pose_cb(self, msg: PoseStamped):
        o = msg.pose.orientation
        with self.mutex:
            self.vr_quat = (o.x, o.y, o.z, o.w)

    def _button_b_cb(self, msg: Bool):
        with self.mutex:
            pressed = bool(msg.data)
            if pressed and not self.button_b_pressed:
                # Latch the rising edge in the subscription callback so
                # short button taps are not missed by the control timer.
                self.button_b_toggle_requested = True
            self.button_b_pressed = pressed

    def _rate_cb(self, msg: Float32):
        with self.mutex:
            self.rate_value = clamp(msg.data, 0.0, 1.0)

    def _joint_states_cb(self, msg: JointState):
        try:
            yaw_idx = msg.name.index(YAW_JOINT)
            pitch_idx = msg.name.index(PITCH_JOINT)
        except ValueError:
            return
        with self.mutex:
            self.current_yaw = msg.position[yaw_idx]
            self.current_pitch = msg.position[pitch_idx]
            if not self.joint_states_received:
                self.commanded_yaw = self.current_yaw
                self.commanded_pitch = self.current_pitch
                self.joint_states_received = True

    def _estop_cb(self, msg: Bool):
        with self.mutex:
            was_active = self.estop_active
            self.estop_active = bool(msg.data)
        if self.estop_active and not was_active:
            self.get_logger().warn("E-STOP: head control suspended")
        elif not self.estop_active and was_active:
            self.get_logger().info("E-STOP released: head control resumed")

    # ------------------------------------------------------------------ control loop
    def _control_loop(self):
        if not self.control_lock.acquire(blocking=False):
            return
        try:
            self._control_step()
        finally:
            self.control_lock.release()

    def _control_step(self):
        # Read parameters that may change at runtime
        slow_max_step_deg = self.get_parameter('slow_max_step_deg').value
        fast_max_step_deg = self.get_parameter('fast_max_step_deg').value
        yaw_scale = self.get_parameter('yaw_scale').value
        pitch_scale = self.get_parameter('pitch_scale').value
        invert_yaw = self.get_parameter('invert_yaw').value
        invert_pitch = self.get_parameter('invert_pitch').value
        enable_soft_limits = self.get_parameter('enable_soft_limits').value
        yaw_soft_margin_deg = self.get_parameter('yaw_soft_margin_deg').value
        pitch_soft_margin_deg = self.get_parameter('pitch_soft_margin_deg').value
        soft_limit_blend_deg = self.get_parameter('soft_limit_blend_deg').value
        startup_home_step_deg = self.get_parameter('startup_home_step_deg').value
        startup_home_tolerance_deg = self.get_parameter('startup_home_tolerance_deg').value

        blend_width = math.radians(max(0.0, soft_limit_blend_deg))
        yaw_command_limit = PHYSICAL_JOINT_LIMIT
        pitch_command_limit = PHYSICAL_JOINT_LIMIT
        if enable_soft_limits:
            yaw_command_limit = resolve_command_limit(
                PHYSICAL_JOINT_LIMIT, yaw_soft_margin_deg)
            pitch_command_limit = resolve_command_limit(
                PHYSICAL_JOINT_LIMIT, pitch_soft_margin_deg)

        # Snapshot state under lock
        with self.mutex:
            vr_quat = self.vr_quat
            button_b_toggle_requested = self.button_b_toggle_requested
            self.button_b_toggle_requested = False
            rate_value = self.rate_value
            cur_yaw = self.current_yaw
            cur_pitch = self.current_pitch
            joint_states_received = self.joint_states_received
            estop_active = self.estop_active

        if not joint_states_received:
            return

        if estop_active:
            return

        if not self.startup_home_done:
            self._run_startup_home_step(
                target_yaw=0.0,
                target_pitch=0.0,
                max_step_deg=startup_home_step_deg,
                tolerance_deg=startup_home_tolerance_deg,
            )
            if button_b_toggle_requested:
                with self.mutex:
                    self.button_b_toggle_requested = True
            return

        if button_b_toggle_requested:
            self.control_enabled = not self.control_enabled
            if self.control_enabled:
                self.reference_quat = vr_quat
                self.anchor_yaw = cur_yaw
                self.anchor_pitch = cur_pitch
                self.get_logger().info(
                    f'Button B toggled ON — anchor yaw={math.degrees(cur_yaw):.1f}° '
                    f'pitch={math.degrees(cur_pitch):.1f}°')
            else:
                self.get_logger().info('Button B toggled OFF — head holds current position')

        if not self.control_enabled:
            return

        # Compute relative rotation: q_rel = q_ref⁻¹ * q_current
        ref_inv = quat_inverse(*self.reference_quat)
        q_rel = quat_multiply(*ref_inv, *vr_quat)

        # Extract yaw/pitch from relative quaternion
        rel_yaw, rel_pitch = euler_from_quaternion_yxz(*q_rel)

        # Apply inversion
        if invert_yaw:
            rel_yaw = -rel_yaw
        if invert_pitch:
            rel_pitch = -rel_pitch

        # Target = anchor + relative * scale
        target_yaw = self.anchor_yaw + rel_yaw * yaw_scale
        target_pitch = self.anchor_pitch + rel_pitch * pitch_scale

        if enable_soft_limits:
            target_yaw = apply_soft_limit(
                target_yaw, yaw_command_limit, blend_width)
            target_pitch = apply_soft_limit(
                target_pitch, pitch_command_limit, blend_width)
        else:
            target_yaw = clamp(target_yaw, -yaw_command_limit, yaw_command_limit)
            target_pitch = clamp(
                target_pitch, -pitch_command_limit, pitch_command_limit)

        # Step limiting based on speed mode
        is_fast = rate_value >= 0.999
        max_step_deg = fast_max_step_deg if is_fast else slow_max_step_deg
        max_step_rad = math.radians(max_step_deg)

        delta_yaw = target_yaw - self.commanded_yaw
        delta_pitch = target_pitch - self.commanded_pitch

        delta_yaw = clamp(delta_yaw, -max_step_rad, max_step_rad)
        delta_pitch = clamp(delta_pitch, -max_step_rad, max_step_rad)

        self.commanded_yaw = self.commanded_yaw + delta_yaw
        self.commanded_pitch = self.commanded_pitch + delta_pitch

        # Final clamp
        self.commanded_yaw = clamp(
            self.commanded_yaw, -yaw_command_limit, yaw_command_limit)
        self.commanded_pitch = clamp(
            self.commanded_pitch, -pitch_command_limit, pitch_command_limit)

        self._publish_command(self.commanded_yaw, self.commanded_pitch)

        # Optional visualization TF
        if self.publish_tf:
            self._publish_debug_tf()

    def _publish_command(self, yaw, pitch):
        msg = Float64MultiArray()
        msg.data = [float(yaw), float(pitch)]
        self.cmd_pub.publish(msg)

    def _run_startup_home_step(self, target_yaw, target_pitch, max_step_deg, tolerance_deg):
        max_step_rad = math.radians(max(0.0, max_step_deg))
        tolerance_rad = math.radians(max(0.0, tolerance_deg))

        delta_yaw = target_yaw - self.commanded_yaw
        delta_pitch = target_pitch - self.commanded_pitch

        if abs(delta_yaw) <= tolerance_rad and abs(delta_pitch) <= tolerance_rad:
            self.commanded_yaw = target_yaw
            self.commanded_pitch = target_pitch
            self.startup_home_done = True
            self.control_enabled = False
            self.button_b_toggle_requested = False

            self._publish_command(self.commanded_yaw, self.commanded_pitch)
            self.get_logger().info('Startup homing complete — head returned to zero')
            return

        if max_step_rad <= 0.0:
            return

        delta_yaw = clamp(delta_yaw, -max_step_rad, max_step_rad)
        delta_pitch = clamp(delta_pitch, -max_step_rad, max_step_rad)

        self.commanded_yaw += delta_yaw
        self.commanded_pitch += delta_pitch

        self._publish_command(self.commanded_yaw, self.commanded_pitch)

    def _publish_debug_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'head_base_link'
        t.child_frame_id = 'head_teleop_target'
        # Encode commanded angles as a simple rotation for visualization
        cy = math.cos(self.commanded_yaw * 0.5)
        sy = math.sin(self.commanded_yaw * 0.5)
        cp = math.cos(self.commanded_pitch * 0.5)
        sp = math.sin(self.commanded_pitch * 0.5)
        # yaw around Z, pitch around Y
        t.transform.rotation.x = -sy * sp
        t.transform.rotation.y = cy * sp
        t.transform.rotation.z = sy * cp
        t.transform.rotation.w = cy * cp
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = HeadTeleopNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
