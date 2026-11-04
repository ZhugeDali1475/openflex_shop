"""One serialized, resident GraspNet worker for both wrists."""

from __future__ import annotations

import struct
import threading

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from openarmx_task_msgs.msg import TaskError
from openflex_grasp_interfaces.action import FitGrasp

from .arm_context import arm_context
from .backend_owner import BackendOwner
from .graspnet_backend import GraspNetBackend, fit_graspnet_straight_grasp
from .paths import GRASPNET_CHECKPOINT, GRASPNET_REPO


def _points(message):
    if message.point_step < 12 or not message.data:
        raise ValueError("point cloud is empty or has no XYZ fields")
    fields = {item.name: item.offset for item in message.fields}
    if not {"x", "y", "z"}.issubset(fields):
        raise ValueError("point cloud must contain x/y/z fields")
    count = int(message.width) * max(1, int(message.height))
    values = np.empty((count, 3), dtype=np.float32)
    for index in range(count):
        base = index * message.point_step
        values[index] = [struct.unpack_from("<f", message.data, base + fields[key])[0] for key in ("x", "y", "z")]
    return values[np.isfinite(values).all(axis=1)]


def _rotate_vector(vector, quaternion):
    vector = np.asarray(vector, dtype=np.float64)
    q = np.asarray(quaternion, dtype=np.float64)
    q /= np.linalg.norm(q)
    xyz, w = q[:3], q[3]
    return (
        vector
        + 2.0 * w * np.cross(xyz, vector)
        + 2.0 * np.cross(xyz, np.cross(xyz, vector))
    )


class GraspNetWorker(Node):
    def __init__(self):
        super().__init__("openflex_graspnet_worker")
        self._owner = BackendOwner("graspnet_worker")
        self.declare_parameter("graspnet_repo", str(GRASPNET_REPO))
        self.declare_parameter("graspnet_checkpoint", str(GRASPNET_CHECKPOINT))
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("axis_frame", "base_link")
        self.declare_parameter("num_point", 20000)
        self.declare_parameter("max_width_m", 0.086)
        self.declare_parameter("pregrasp_distance_m", 0.10)
        self.declare_parameter("grasp_depth_offset_m", 0.05)
        self.declare_parameter("enable_corridor_check", False)
        self._busy = threading.Lock()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._backend = GraspNetBackend(
            str(self.get_parameter("graspnet_repo").value),
            str(self.get_parameter("graspnet_checkpoint").value),
            num_point=int(self.get_parameter("num_point").value), device=str(self.get_parameter("device").value),
        )
        self._server = ActionServer(
            self, FitGrasp, "/openflex_grasp/fit_grasp", execute_callback=self._execute,
            goal_callback=lambda _: GoalResponse.ACCEPT, cancel_callback=lambda _: CancelResponse.ACCEPT,
        )
        self.get_logger().info("Resident GraspNet worker ready for both wrists")

    @staticmethod
    def _feedback(handle, phase, progress):
        feedback = FitGrasp.Feedback()
        feedback.phase, feedback.progress = phase, float(progress)
        handle.publish_feedback(feedback)

    def _execute(self, handle):
        result, acquired = FitGrasp.Result(), False
        try:
            if not self._busy.acquire(blocking=False):
                raise RuntimeError("GraspNet worker is busy; left/right requests are serialized")
            acquired = True
            goal, context = handle.request, arm_context(handle.request.arm_name)
            if handle.is_cancel_requested:
                raise RuntimeError("FitGrasp cancelled")
            self._feedback(handle, "LOAD_SNAPSHOT", 0.15)
            product, scene = _points(goal.product_points), _points(goal.scene_points)
            if product.shape[0] < 100 or scene.shape[0] < 100:
                raise ValueError("detection point clouds have insufficient points")
            self._feedback(handle, "GRASPNET_INFERENCE", 0.45)
            candidates = self._backend.predict(scene)
            self._feedback(handle, "FIT_STRAIGHT_GRASP", 0.75)
            approach = self._axis_in_cloud_frame(
                context.approach_axis, goal.product_points.header.frame_id
            )
            closing = self._axis_in_cloud_frame(
                context.closing_axis, goal.product_points.header.frame_id
            )
            collision_scene = (
                scene
                if bool(self.get_parameter("enable_corridor_check").value)
                else None
            )
            fit = fit_graspnet_straight_grasp(
                product, collision_scene, candidates,
                approach=approach, closing_axis=closing,
                pregrasp_distance=float(self.get_parameter("pregrasp_distance_m").value),
                grasp_depth_offset=float(
                    self.get_parameter("grasp_depth_offset_m").value
                ),
                gripper_max_width=float(self.get_parameter("max_width_m").value), enforce_gripper_width=True,
            )
            if not fit.valid:
                raise RuntimeError(fit.reason)
            header = goal.product_points.header
            grasp, pregrasp = PoseStamped(), PoseStamped()
            for pose, location in ((grasp, fit.center), (pregrasp, fit.pregrasp)):
                pose.header = header
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, location)
                pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = map(float, fit.quaternion_xyzw)
            result.success, result.grasp_pose, result.pregrasp_pose = True, grasp, pregrasp
            result.required_width_m, result.score = float(fit.required_width), float(fit.score)
            result.candidate_count, result.constrained_candidate_count = int(fit.candidate_count), int(fit.constrained_candidate_count)
            result.error_code, result.message = TaskError.OK, "ok"
            self._feedback(handle, "FIT_READY", 1.0)
            handle.succeed()
        except Exception as error:
            result.success, result.error_code, result.message = False, TaskError.PICK_FAILED, str(error)
            if handle.is_cancel_requested:
                result.error_code = TaskError.CANCELLED
                handle.canceled()
            else:
                handle.abort()
        finally:
            if acquired:
                self._busy.release()
        return result

    def _axis_in_cloud_frame(self, axis, cloud_frame: str):
        axis_frame = str(self.get_parameter("axis_frame").value).strip()
        if not cloud_frame:
            raise RuntimeError("product point cloud has no frame_id")
        if cloud_frame == axis_frame:
            return np.asarray(axis, dtype=np.float64)
        try:
            transform = self._tf_buffer.lookup_transform(
                cloud_frame, axis_frame, Time(), timeout=Duration(seconds=1.0)
            )
        except TransformException as error:
            raise RuntimeError(
                f"failed to transform grasp axis {axis_frame}->{cloud_frame}: {error}"
            ) from error
        q = transform.transform.rotation
        return _rotate_vector(axis, (q.x, q.y, q.z, q.w))

    def destroy_node(self):
        self._owner.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GraspNetWorker()
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
