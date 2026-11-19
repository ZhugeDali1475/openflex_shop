"""Exclusive, fail-closed velocity mux for Nav2, ACT and recovery."""

from copy import deepcopy
import json
import time
import threading

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from openflex_thesis_msgs.msg import AffordanceState, ControlMode, ObjectMemory
from openflex_thesis_msgs.srv import SetControlMode
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .arbiter import (
    ACT,
    HOLD,
    NAV2,
    MOVEIT,
    PLACE,
    RECOVERY,
    ArbiterConfig,
    Velocity,
    VelocityArbiter,
)

# A locked base stays healthy while the measured twist is zero. Long arm trajectories
# (tens of seconds) must not be aborted by a momentary telemetry hiccup, but a base
# whose measured velocity is persistently unknown still fails closed.
CONTROL_FEED_STALE_SEC = 2.0


class VelocityArbiterNode(Node):
    def __init__(self) -> None:
        super().__init__("velocity_arbiter")
        self.declare_parameter("nav_input_topic", "/thesis/nav_cmd_vel")
        self.declare_parameter("act_input_topic", "/thesis/act_cmd_vel")
        self.declare_parameter("recovery_input_topic", "/thesis/recovery_cmd_vel")
        self.declare_parameter("output_topic", "/thesis/cmd_vel_selected")
        self.declare_parameter("measured_velocity_topic", "/swerve_drive_controller/odom")
        self.declare_parameter("affordance_topic", "/thesis/tc_wam/state")
        self.declare_parameter("memory_topic", "/thesis/object_memory")
        self.declare_parameter("mode_topic", "/thesis/control_mode")
        self.declare_parameter("set_mode_service", "/thesis/set_control_mode")
        self.declare_parameter("input_timeout_sec", 0.20)
        self.declare_parameter("measured_timeout_sec", 0.25)
        self.declare_parameter("memory_stream_timeout_sec", 0.25)
        self.declare_parameter("stop_confirm_sec", 0.50)
        self.declare_parameter("stopped_linear_mps", 0.02)
        self.declare_parameter("stopped_angular_rps", 0.03)
        self.declare_parameter("max_linear_mps", 0.25)
        self.declare_parameter("max_angular_rps", 0.40)
        self.declare_parameter("require_physical_affordance", True)
        self.declare_parameter("publish_rate_hz", 50.0)
        self.arbiter = VelocityArbiter(
            ArbiterConfig(
                input_timeout_sec=float(self.get_parameter("input_timeout_sec").value),
                measured_timeout_sec=float(
                    self.get_parameter("measured_timeout_sec").value
                ),
                stop_confirm_sec=float(self.get_parameter("stop_confirm_sec").value),
                stopped_linear_mps=float(self.get_parameter("stopped_linear_mps").value),
                stopped_angular_rps=float(self.get_parameter("stopped_angular_rps").value),
                max_linear_mps=float(self.get_parameter("max_linear_mps").value),
                max_angular_rps=float(self.get_parameter("max_angular_rps").value),
                require_physical_affordance=bool(
                    self.get_parameter("require_physical_affordance").value
                ),
            )
        )
        mode_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.output_publisher = self.create_publisher(
            Twist, str(self.get_parameter("output_topic").value), 10
        )
        self.mode_publisher = self.create_publisher(
            ControlMode, str(self.get_parameter("mode_topic").value), mode_qos
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("nav_input_topic").value),
            lambda msg: self.on_source(NAV2, msg),
            10,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("act_input_topic").value),
            lambda msg: self.on_source(ACT, msg),
            10,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("recovery_input_topic").value),
            lambda msg: self.on_source(RECOVERY, msg),
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("measured_velocity_topic").value),
            self.on_odometry,
            20,
        )
        self.create_subscription(
            AffordanceState,
            str(self.get_parameter("affordance_topic").value),
            self.on_affordance,
            mode_qos,
        )
        self.create_subscription(
            ObjectMemory,
            str(self.get_parameter("memory_topic").value),
            self.on_memory,
            mode_qos,
        )
        self.create_service(
            SetControlMode,
            str(self.get_parameter("set_mode_service").value),
            self.on_set_mode,
        )
        self.latest_affordance = None
        self.latest_memory = None
        self.latest_memory_receipt_sec = 0.0
        self.last_published_epoch = -1
        rate = float(self.get_parameter("publish_rate_hz").value)
        if rate <= 0.0:
            raise ValueError("publish_rate_hz must be positive")
        self.create_timer(1.0 / rate, self.on_timer)
        self.publish_mode("startup_hold")
        # The mode feed is a live exclusivity heartbeat: consumers reject control when
        # it goes stale on the wall clock. A sim-clock stall (long probe, CPU spike)
        # would otherwise starve the timer and abort an in-flight MoveIt trajectory,
        # so republish the same mode from a wall-clock watchdog.
        self.mode_watchdog = threading.Thread(target=self.mode_watchdog_loop, daemon=True)
        self.mode_watchdog.start()

    def mode_watchdog_loop(self) -> None:
        while rclpy.ok():
            time.sleep(0.2)
            try:
                self.publish_mode(self.last_mode_reason)
            except Exception as error:  # pragma: no cover - defensive only
                self.get_logger().error(f"control_mode watchdog republish failed: {error}")

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def on_source(self, mode: int, msg: Twist) -> None:
        self.arbiter.update_source(
            mode, Velocity(msg.linear.x, msg.linear.y, msg.angular.z), self.now_sec()
        )

    def on_odometry(self, msg: Odometry) -> None:
        twist = msg.twist.twist
        now_sec = self.now_sec()
        if self.arbiter.last_measured_sec is not None and now_sec - self.arbiter.last_measured_sec > 0.3:
            self.get_logger().warn(f"odometry feed gap {now_sec - self.arbiter.last_measured_sec:.3f}s of sim time")
        self.arbiter.update_measured_velocity(
            Velocity(twist.linear.x, twist.linear.y, twist.angular.z), now_sec
        )

    def on_affordance(self, msg: AffordanceState) -> None:
        self.latest_affordance = deepcopy(msg)
        self.update_safety()

    def on_memory(self, msg: ObjectMemory) -> None:
        self.latest_memory = deepcopy(msg)
        self.latest_memory_receipt_sec = self.now_sec()
        self.update_safety()

    def update_safety(self) -> None:
        affordance = self.latest_affordance
        memory = self.latest_memory
        versions_match = bool(
            affordance is not None
            and memory is not None
            and affordance.object_id == memory.object_id
            and affordance.map_version == memory.map_version
        )
        memory_stream_fresh = bool(
            memory is not None
            and self.now_sec() - self.latest_memory_receipt_sec
            <= float(self.get_parameter("memory_stream_timeout_sec").value)
        )
        previous_epoch = self.arbiter.epoch
        self.arbiter.update_safety(
            bool(versions_match and affordance.valid),
            bool(versions_match and affordance.physical_evaluation),
            bool(memory_stream_fresh and memory.visible),
            bool(memory_stream_fresh and memory.grasp_authorized),
        )
        if self.arbiter.epoch != previous_epoch:
            self.publish_mode("safety invalidated active policy")

    def on_set_mode(self, request, response):
        accepted, message = self.arbiter.request_mode(
            int(request.requested_mode), self.now_sec()
        )
        response.accepted = accepted
        response.current_mode = self.arbiter.mode
        response.epoch = self.arbiter.epoch
        response.message = message
        if accepted:
            self.output_publisher.publish(Twist())
            self.publish_mode(request.reason or "mode request")
        return response

    @staticmethod
    def owner(mode: int) -> str:
        return {HOLD: "base_lock", NAV2: "nav2", ACT: "act", RECOVERY: "recovery", PLACE: "place", MOVEIT: "moveit_base_lock"}.get(
            mode, "unknown"
        )

    def publish_mode(self, reason: str) -> None:
        wall_now = time.monotonic()
        previous = getattr(self, 'last_mode_publish_wall', None)
        if previous is not None and wall_now - previous > 0.3:
            self.get_logger().warn(
                f"control_mode publication stalled {wall_now - previous:.3f}s of wall time; "
                f"sim clock {self.now_sec():.3f}, reason={reason}")
        self.last_mode_publish_wall = wall_now
        self.last_mode_publish_sec = self.now_sec()
        self.last_mode_reason = reason
        msg = ControlMode()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mode = self.arbiter.mode
        msg.epoch = self.arbiter.epoch
        msg.owner = self.owner(self.arbiter.mode)
        msg.reason = reason
        if self.arbiter.mode == ACT:
            msg.inputs_healthy = self.arbiter.act_safety_valid()
        elif self.arbiter.mode in (MOVEIT, PLACE):
            msg.inputs_healthy = self.arbiter.confirmed_stopped(self.now_sec(), stale_sec=CONTROL_FEED_STALE_SEC)
        else:
            msg.inputs_healthy = True
        signature = (int(msg.mode), int(msg.epoch), bool(msg.inputs_healthy))
        if signature != getattr(self, 'last_mode_signature', None):
            self.get_logger().warn(f"control_mode -> mode={msg.mode} epoch={msg.epoch} "
                                   f"inputs_healthy={msg.inputs_healthy} reason={reason}"
                                   + ("" if msg.inputs_healthy else " "
                                      + json.dumps(self.arbiter.stop_diagnostics(self.now_sec()))))
            self.last_mode_signature = signature
        self.mode_publisher.publish(msg)
        self.last_published_epoch = self.arbiter.epoch

    def on_timer(self) -> None:
        self.update_safety()
        selected = self.arbiter.selected_velocity(self.now_sec())
        msg = Twist()
        msg.linear.x = selected.vx
        msg.linear.y = selected.vy
        msg.angular.z = selected.wz
        self.output_publisher.publish(msg)
        if self.last_published_epoch != self.arbiter.epoch:
            self.publish_mode("automatic fail-closed transition")
        elif self.now_sec() - self.last_mode_publish_sec >= 0.1:
            self.publish_mode(self.last_mode_reason)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VelocityArbiterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.output_publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
