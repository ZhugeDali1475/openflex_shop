"""PickObject adapter: validates/consumes detection UUID and requests resident GraspNet.

Physical execution is deliberately disabled here. The returned plan-only result lets
the higher layer validate the non-executing chain without commanding hardware.
"""

from __future__ import annotations

import copy
import json
import math
import threading

import rclpy
from control_msgs.action import GripperCommand
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from nav_msgs.msg import Path as NavPath
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from openarmx_task_msgs.action import PickObject
from openarmx_task_msgs.msg import TaskError
from openflex_grasp_interfaces.action import FitGrasp
from openflex_grasp_interfaces.srv import GetDetectionSnapshot
from openarmx_integrated_moveit_msgs.action import CartesianApproach, PlanPregrasp

from .arm_context import arm_context
from .backend_owner import BackendOwner
from .detection_store import DetectionStore
from .pointcloud_safety import (
    pointcloud_xyz,
    require_pose_above_minimum,
    transformed_minimum_z,
)
from .product_moveit_bridge import transform_pose_via_planar_base


class PickObjectServer(Node):
    def __init__(self):
        super().__init__("openflex_pick_object")
        self._owner = BackendOwner("pick_object")
        self.declare_parameter("execute", False)
        self.declare_parameter("snapshot_service", "/openflex_grasp/get_detection_snapshot")
        self.declare_parameter("fit_action", "/openflex_grasp/fit_grasp")
        self.declare_parameter("plan_action", "/openflex/plan_pregrasp")
        self.declare_parameter("cartesian_action", "/openflex/cartesian_approach")
        self.declare_parameter("target_frame", "odom")
        self.declare_parameter("velocity_scale", 0.08)
        self.declare_parameter("acceleration_scale", 0.08)
        self.declare_parameter("object_bottom_clearance_m", 0.01)
        self.declare_parameter("gripper_max_effort", 10.0)
        self.declare_parameter("gripper_attached_min_position_m", 0.002)
        self._busy = threading.Lock()
        # Preserve detection-time transforms across slow model inference.
        self._tf_buffer = Buffer(cache_time=Duration(seconds=240.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._snapshots = self.create_client(
            GetDetectionSnapshot, str(self.get_parameter("snapshot_service").value)
        )
        self._fit = ActionClient(self, FitGrasp, str(self.get_parameter("fit_action").value))
        self._plan = ActionClient(self, PlanPregrasp, str(self.get_parameter("plan_action").value))
        self._cartesian = ActionClient(
            self, CartesianApproach, str(self.get_parameter("cartesian_action").value)
        )
        self._grippers = {
            arm: ActionClient(self, GripperCommand, arm_context(arm).gripper_action)
            for arm in ("left", "right")
        }
        debug_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._debug_grasp = self.create_publisher(
            String, "/openflex_grasp/debug/grasp_result", debug_qos
        )
        self._debug_execution = self.create_publisher(
            String, "/openflex_grasp/debug/execution_result", debug_qos
        )
        self._debug_path = self.create_publisher(
            NavPath, "/openflex_grasp/debug/grasp_path", debug_qos
        )
        self._server = ActionServer(
            self, PickObject, "/pick_object", execute_callback=self._execute,
            goal_callback=self._goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
        )

    def _goal(self, goal):
        try:
            DetectionStore.validate_id(goal.detection_id)
            arm_context(goal.arm_name)
        except Exception:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    async def _execute(self, handle):
        result, acquired = PickObject.Result(), False
        try:
            if not self._busy.acquire(blocking=False):
                raise RuntimeError("Pick requests are serialized")
            acquired = True
            goal = handle.request
            context = arm_context(goal.arm_name)
            if not self._snapshots.wait_for_service(timeout_sec=2.0):
                raise RuntimeError("detection snapshot service unavailable")
            request = GetDetectionSnapshot.Request()
            request.detection_id, request.consume = goal.detection_id, False
            timeout_s = max(1.0, float(goal.timeout_sec))
            snapshot = await self._snapshots.call_async(request)
            if snapshot is None or not snapshot.success:
                code = TaskError.DETECTION_EXPIRED if snapshot and snapshot.error_code == TaskError.DETECTION_EXPIRED else TaskError.DETECTION_MISMATCH
                raise RuntimeError(f"snapshot read failed [{code}]: {'' if snapshot is None else snapshot.message}")
            if snapshot.arm_name != context.arm_name or snapshot.object_id != goal.object_id:
                raise RuntimeError("detection snapshot arm or object does not match Pick goal")
            if not self._fit.wait_for_server(timeout_sec=2.0):
                raise RuntimeError("resident GraspNet worker unavailable")
            fit_goal = FitGrasp.Goal()
            fit_goal.detection_id, fit_goal.arm_name = goal.detection_id, context.arm_name
            fit_goal.grasp_strategy, fit_goal.product_points, fit_goal.scene_points = goal.grasp_strategy, snapshot.product_points, snapshot.scene_points
            fit_goal.timeout_sec = float(goal.timeout_sec)
            fitted = await self._fit.send_goal_async(fit_goal)
            if fitted is None or not fitted.accepted:
                raise RuntimeError("GraspNet worker rejected FitGrasp goal")
            final = await fitted.get_result_async()
            fit_result = final.result if final else None
            if fit_result is None or not fit_result.success:
                raise RuntimeError(f"GraspNet fitting failed: {'' if fit_result is None else fit_result.message}")
            try:
                self._publish_grasp_debug(goal, snapshot.object_pose, fit_result)
            except Exception as debug_error:
                self.get_logger().warning(
                    f"Grasp fitting succeeded but debug visualization publish failed: {debug_error}"
                )
            execute = bool(self.get_parameter("execute").value)
            pregrasp = self._transform_pose(fit_result.pregrasp_pose)
            grasp = self._transform_pose(fit_result.grasp_pose)
            minimum_tcp_z = self._minimum_product_tcp_z(snapshot.product_points)
            require_pose_above_minimum(pregrasp, minimum_tcp_z, "pregrasp")
            require_pose_above_minimum(grasp, minimum_tcp_z, "grasp")
            current_pose = self._lookup_current_tcp_pose(context)
            safe_start = copy.deepcopy(current_pose)
            safe_start.pose.position.z = max(
                float(current_pose.pose.position.z), minimum_tcp_z
            )
            predicted_start = None
            if safe_start.pose.position.z > current_pose.pose.position.z + 1e-6:
                raise_result = await self._cartesian_segment(
                    current_pose,
                    safe_start,
                    context.arm_name,
                    execute,
                )
                if not execute:
                    predicted_start = self._trajectory_end_state(
                        raise_result.trajectory
                    )
            pregrasp_result = await self._cartesian_segment(
                safe_start,
                pregrasp,
                context.arm_name,
                execute,
                start_joint_state=predicted_start,
                minimum_tcp_z=minimum_tcp_z,
            )
            selected_stage = PlanPregrasp.Result.STAGE_B
            if not execute:
                await self._cartesian_segment(
                    pregrasp,
                    grasp,
                    context.arm_name,
                    False,
                    start_joint_state=self._cartesian_start_state(
                        self._trajectory_end_state(pregrasp_result.trajectory),
                        context,
                    ),
                    minimum_tcp_z=minimum_tcp_z,
                )
            consume = GetDetectionSnapshot.Request()
            consume.detection_id, consume.consume = goal.detection_id, True
            consumed = await self._snapshots.call_async(consume)
            if not consumed.success:
                raise RuntimeError(
                    f"detection snapshot could not be consumed: {consumed.message}"
                )
            attached = False
            if execute:
                gripper = self._grippers[context.arm_name]
                if not gripper.wait_for_server(timeout_sec=2.0):
                    raise RuntimeError(f"{context.arm_name} gripper Action unavailable")
                await self._gripper(gripper, 0.044)
                approached = False
                try:
                    await self._cartesian_segment(
                        pregrasp,
                        grasp,
                        context.arm_name,
                        True,
                        minimum_tcp_z=minimum_tcp_z,
                    )
                    approached = True
                    closed = await self._gripper(gripper, 0.0)
                    attached = bool(closed.stalled) and float(closed.position) >= float(
                        self.get_parameter("gripper_attached_min_position_m").value
                    )
                    if not attached:
                        raise RuntimeError(
                            "gripper close did not provide object-attached evidence"
                        )
                finally:
                    if approached:
                        try:
                            await self._cartesian_segment(
                                grasp,
                                pregrasp,
                                context.arm_name,
                                True,
                                minimum_tcp_z=minimum_tcp_z,
                            )
                        except Exception as retreat_error:
                            raise RuntimeError(
                                f"grasp retreat failed: {retreat_error}"
                            ) from retreat_error
            result.success, result.object_attached = True, attached
            result.error_code = TaskError.OK
            result.message = (
                f"executed and attached on {context.arm_name}"
                if execute else
                f"plan-only MoveIt+Cartesian chain ready for {context.arm_name}; no hardware command was sent; "
                f"stage={selected_stage} width={fit_result.required_width_m:.3f}m "
                f"candidates={fit_result.candidate_count}"
            )
            self._publish_execution_debug(
                True,
                result.message,
                "executed" if execute else "plan_only_complete",
                execute,
                selected_stage,
            )
            handle.succeed()
        except Exception as error:
            result.success, result.object_attached = False, False
            result.error_code = TaskError.CANCELLED if handle.is_cancel_requested else TaskError.PICK_FAILED
            result.message = str(error)
            self._publish_execution_debug(
                False, str(error), "cancelled" if handle.is_cancel_requested else "failed", False, -1
            )
            if handle.is_cancel_requested:
                handle.canceled()
            else:
                handle.abort()
        finally:
            if acquired:
                self._busy.release()
        return result

    @staticmethod
    def _pose_payload(pose):
        return {
            "frame_id": pose.header.frame_id,
            "position": [
                float(pose.pose.position.x),
                float(pose.pose.position.y),
                float(pose.pose.position.z),
            ],
            "orientation": [
                float(pose.pose.orientation.x),
                float(pose.pose.orientation.y),
                float(pose.pose.orientation.z),
                float(pose.pose.orientation.w),
            ],
        }

    def _publish_grasp_debug(self, goal, object_pose, fit_result):
        payload = {
            "valid": True,
            "message": str(fit_result.message) or "GraspNet 位姿已就绪",
            "detection_id": goal.detection_id,
            "arm_name": goal.arm_name,
            "object_pose": self._pose_payload(object_pose),
            "pregrasp_pose": self._pose_payload(fit_result.pregrasp_pose),
            "grasp_pose": self._pose_payload(fit_result.grasp_pose),
            "required_width_m": float(fit_result.required_width_m),
            "score": float(fit_result.score),
            "candidate_count": int(fit_result.candidate_count),
            "constrained_candidate_count": int(fit_result.constrained_candidate_count),
        }
        self._debug_grasp.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )
        path = NavPath()
        path.header = copy.deepcopy(fit_result.pregrasp_pose.header)
        path.poses = [
            copy.deepcopy(fit_result.pregrasp_pose),
            copy.deepcopy(fit_result.grasp_pose),
        ]
        self._debug_path.publish(path)

    def _publish_execution_debug(self, accepted, message, stage, execute, selected_stage):
        payload = {
            "accepted": bool(accepted),
            "message": str(message),
            "stage": str(stage),
            "execute": bool(execute),
            "moveit_stage": int(selected_stage),
        }
        try:
            self._debug_execution.publish(
                String(data=json.dumps(payload, ensure_ascii=False))
            )
        except Exception as debug_error:
            self.get_logger().warning(
                f"Execution result debug publish failed: {debug_error}"
            )

    def _transform_pose(self, source):
        target = str(self.get_parameter("target_frame").value).strip()
        try:
            return transform_pose_via_planar_base(
                self._tf_buffer,
                source,
                target_frame=target,
                lookup_time=Time(),
                timeout=Duration(seconds=1.0),
            )
        except TransformException as error:
            raise RuntimeError(
                f"failed to transform grasp pose {source.header.frame_id}->{target}: {error}"
            ) from error

    def _lookup_current_tcp_pose(self, context):
        source = PoseStamped()
        source.header.frame_id = context.tcp_frame
        source.pose.orientation.w = 1.0
        try:
            return transform_pose_via_planar_base(
                self._tf_buffer,
                source,
                target_frame=str(self.get_parameter("target_frame").value),
                lookup_time=Time(),
                timeout=Duration(seconds=1.0),
            )
        except TransformException as error:
            raise RuntimeError(
                f"current TCP pose {context.tcp_frame} unavailable: {error}"
            ) from error

    def _minimum_product_tcp_z(self, cloud):
        source_frame = str(cloud.header.frame_id).strip()
        if not source_frame:
            raise RuntimeError("product point cloud has no frame_id")
        target_frame = str(self.get_parameter("target_frame").value).strip()
        transform_target = "base_link" if target_frame == "odom" else target_frame
        points = pointcloud_xyz(cloud)
        if source_frame == transform_target:
            translation = (0.0, 0.0, 0.0)
            rotation = (0.0, 0.0, 0.0, 1.0)
        else:
            try:
                transform = self._tf_buffer.lookup_transform(
                    transform_target,
                    source_frame,
                    Time.from_msg(cloud.header.stamp),
                    timeout=Duration(seconds=1.0),
                )
            except TransformException as error:
                raise RuntimeError(
                    f"product-cloud TF {source_frame}->{transform_target} "
                    f"unavailable: {error}"
                ) from error
            t = transform.transform.translation
            q = transform.transform.rotation
            translation = (t.x, t.y, t.z)
            rotation = (q.x, q.y, q.z, q.w)
        clearance = float(self.get_parameter("object_bottom_clearance_m").value)
        if not math.isfinite(clearance) or clearance < 0.0 or clearance > 0.10:
            raise RuntimeError("object_bottom_clearance_m must be in [0.0, 0.10] m")
        return transformed_minimum_z(points, translation, rotation) + clearance

    @staticmethod
    def _trajectory_end_state(trajectory):
        names = trajectory.joint_trajectory.joint_names
        points = trajectory.joint_trajectory.points
        if not names or not points or len(names) != len(points[-1].positions):
            raise RuntimeError("planned trajectory has no valid endpoint state")
        return JointState(
            name=list(names),
            position=list(points[-1].positions),
        )

    async def _plan_pregrasp(self, pose, arm_name, max_dof, execute):
        if not self._plan.wait_for_server(timeout_sec=2.0):
            raise RuntimeError("PlanPregrasp Action unavailable")
        if int(max_dof) != PlanPregrasp.Goal.ARM_WITH_LIFT:
            raise RuntimeError("PickObject pregrasp requires ARM_WITH_LIFT=8")
        goal = PlanPregrasp.Goal()
        goal.arm = PlanPregrasp.Goal.LEFT if arm_name == "left" else PlanPregrasp.Goal.RIGHT
        goal.max_dof = PlanPregrasp.Goal.ARM_WITH_LIFT
        goal.tcp_pregrasp_pose = pose
        goal.base_yaw_mode = PlanPregrasp.Goal.CURRENT_STATE
        goal.velocity_scale = float(self.get_parameter("velocity_scale").value)
        goal.acceleration_scale = float(self.get_parameter("acceleration_scale").value)
        goal.planning_timeout_sec = 8.0
        goal.execute = bool(execute)
        sent = await self._plan.send_goal_async(goal)
        if not sent.accepted:
            raise RuntimeError("PlanPregrasp goal rejected")
        wrapped = await sent.get_result_async()
        if wrapped.result.error_code.val != 1:
            raise RuntimeError(f"PlanPregrasp failed: {wrapped.result.message}")
        if wrapped.result.selected_stage not in (
            wrapped.result.STAGE_A,
            wrapped.result.STAGE_B,
        ):
            raise RuntimeError("PlanPregrasp selected forbidden whole-body Stage C")
        return wrapped.result

    @staticmethod
    def _cartesian_start_state(planned_state, context):
        allowed = set(context.planning_joint_names)
        result = JointState()
        result.header = planned_state.header
        for name, position in zip(planned_state.name, planned_state.position):
            if name in allowed:
                result.name.append(name)
                result.position.append(position)
        missing = set(context.joint_names) - set(result.name)
        if missing:
            raise RuntimeError(
                "PlanPregrasp result is missing arm joints: "
                + ", ".join(sorted(missing))
            )
        return result

    async def _cartesian_segment(
        self,
        start,
        end,
        arm_name,
        execute,
        *,
        start_joint_state=None,
        minimum_tcp_z=None,
    ):
        if not self._cartesian.wait_for_server(timeout_sec=2.0):
            raise RuntimeError("CartesianApproach Action unavailable")
        goal = CartesianApproach.Goal()
        goal.arm = CartesianApproach.Goal.LEFT if arm_name == "left" else CartesianApproach.Goal.RIGHT
        goal.start_pose, goal.end_pose = start, end
        if start_joint_state is not None:
            goal.use_start_joint_state = True
            goal.start_joint_state = start_joint_state
        goal.eef_step, goal.jump_threshold, goal.min_fraction = 0.005, 0.0, 0.99
        goal.start_position_tolerance, goal.start_orientation_tolerance_deg = 0.04, 20.0
        goal.velocity_scale = float(self.get_parameter("velocity_scale").value)
        goal.acceleration_scale = float(self.get_parameter("acceleration_scale").value)
        goal.orientation_mode = CartesianApproach.Goal.INTERPOLATE
        goal.path_mode = CartesianApproach.Goal.STRICT_CARTESIAN
        if minimum_tcp_z is not None:
            goal.use_minimum_tcp_z = True
            goal.minimum_tcp_z = float(minimum_tcp_z)
        goal.execute = bool(execute)
        sent = await self._cartesian.send_goal_async(goal)
        if not sent.accepted:
            raise RuntimeError("CartesianApproach goal rejected")
        wrapped = await sent.get_result_async()
        if wrapped.result.error_code.val != 1 or wrapped.result.path_fraction < 0.99:
            raise RuntimeError(f"CartesianApproach failed: {wrapped.result.message}")
        return wrapped.result

    async def _gripper(self, client, position):
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(self.get_parameter("gripper_max_effort").value)
        sent = await client.send_goal_async(goal)
        if not sent.accepted:
            raise RuntimeError("gripper goal rejected")
        wrapped = await sent.get_result_async()
        return wrapped.result

    def destroy_node(self):
        self._owner.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PickObjectServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
