import json

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .grasp_geometry import (
    align_symmetric_axis,
    grasp_mode_profile,
    robust_cloud_center,
)
from .graspnet_backend import (
    GraspNetBackend,
    fit_graspnet_straight_grasp,
    summarize_graspnet_candidates,
)
from .product_moveit_bridge import rotate_vector_by_quaternion
from .paths import GRASPNET_CHECKPOINT, GRASPNET_REPO


def pointcloud_xyz(message: PointCloud2) -> np.ndarray:
    fields = {field.name: field for field in message.fields}
    missing = {"x", "y", "z"} - fields.keys()
    if missing:
        raise ValueError(f"PointCloud2 is missing fields: {sorted(missing)}")
    dtype = np.dtype({
        "names": ["x", "y", "z"],
        "formats": ["<f4", "<f4", "<f4"],
        "offsets": [fields[axis].offset for axis in ("x", "y", "z")],
        "itemsize": message.point_step,
    })
    count = int(message.width * message.height)
    records = np.frombuffer(message.data, dtype=dtype, count=count)
    return np.column_stack((records["x"], records["y"], records["z"])).astype(np.float64)


class StraightGraspNode(Node):
    def __init__(self):
        super().__init__("straight_grasp")
        self.declare_parameter("backend", "graspnet")
        self.declare_parameter("grasp_mode", "shelf")
        self.declare_parameter("approach_axis", [1.0, 0.0, 0.0])
        self.declare_parameter("approach_frame", "base_link")
        self.declare_parameter("closing_axis", [0.0, -1.0, 0.0])
        self.declare_parameter("closing_axis_frame", "base_link")
        self.declare_parameter("basket_approach_axis", [0.0, 0.0, -1.0])
        self.declare_parameter("basket_approach_frame", "base_link")
        self.declare_parameter("tcp_frame", "openarmx_right_hand_tcp")
        self.declare_parameter("approach_transform_timeout_s", 1.0)
        self.declare_parameter("pregrasp_distance", 0.12)
        self.declare_parameter("grasp_depth_offset", 0.015)
        self.declare_parameter("gripper_max_width", 0.086)
        self.declare_parameter("enforce_gripper_width", False)
        self.declare_parameter("width_margin", 0.008)
        self.declare_parameter("corridor_radius", 0.055)
        self.declare_parameter("object_clearance", 0.04)
        self.declare_parameter("collision_point_limit", 20)
        self.declare_parameter("min_points", 100)
        self.declare_parameter("product_cloud_topic", "/product_perception/points")
        self.declare_parameter(
            "graspnet_repo",
            str(GRASPNET_REPO),
        )
        self.declare_parameter(
            "graspnet_checkpoint",
            str(GRASPNET_CHECKPOINT),
        )
        self.declare_parameter("graspnet_device", "cuda:0")
        self.declare_parameter("graspnet_num_point", 20000)
        self.declare_parameter("graspnet_num_view", 300)
        self.declare_parameter("candidate_center_tolerance", 0.06)
        self.declare_parameter("candidate_approach_max_angle_deg", 25.0)
        self.declare_parameter("candidate_diagnostic_limit", 80)

        requested_backend = str(self.get_parameter("backend").value).lower()
        if requested_backend != "graspnet":
            raise ValueError(
                "only the license-free GraspNetBaseline backend is supported; "
                f"got {requested_backend!r}. Set backend:=graspnet."
            )
        self._grasp_profile = grasp_mode_profile(
            str(self.get_parameter("grasp_mode").value)
        )
        self._graspnet = GraspNetBackend(
            str(self.get_parameter("graspnet_repo").value),
            str(self.get_parameter("graspnet_checkpoint").value),
            device=str(self.get_parameter("graspnet_device").value),
            num_point=int(self.get_parameter("graspnet_num_point").value),
            num_view=int(self.get_parameter("graspnet_num_view").value),
        )
        self._active_backend = "graspnet"
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("product_cloud_topic").value),
            self._on_product,
            1,
        )
        self.grasp_pub = self.create_publisher(PoseStamped, "/product_perception/grasp_pose", 1)
        self.pregrasp_pub = self.create_publisher(PoseStamped, "/product_perception/pregrasp_pose", 1)
        self.path_pub = self.create_publisher(Path, "/product_perception/grasp_path", 1)
        self.width_pub = self.create_publisher(Float32, "/product_perception/gripper_width", 1)
        self.result_pub = self.create_publisher(String, "/product_perception/grasp_result", 10)
        self.candidate_pub = self.create_publisher(
            String, "/product_perception/grasp_candidates", 10
        )
        self.marker_pub = self.create_publisher(MarkerArray, "/product_perception/grasp_markers", 1)
        self.get_logger().info(
            f"Straight grasp fitter is ready (requested={requested_backend}, "
            f"active={self._active_backend}, mode={self._grasp_profile.name})"
        )

    def _on_product(self, message: PointCloud2):
        try:
            points = pointcloud_xyz(message)
            estimate = self._fit(points, message.header.frame_id)
            self._publish(message, estimate)
        except Exception as error:
            self.get_logger().error(f"Grasp fitting failed: {error}")
            payload = {
                "valid": False,
                "reason": str(error),
                "backend": self._active_backend,
                "grasp_mode": self._grasp_profile.name,
                "frame_id": message.header.frame_id,
                "execution_enabled": False,
            }
            result = String()
            result.data = json.dumps(payload, ensure_ascii=False)
            self.result_pub.publish(result)

    def _fit_kwargs(self):
        return {
            "pregrasp_distance": float(self.get_parameter("pregrasp_distance").value),
            "grasp_depth_offset": float(self.get_parameter("grasp_depth_offset").value),
            "gripper_max_width": float(self.get_parameter("gripper_max_width").value),
            "enforce_gripper_width": bool(
                self.get_parameter("enforce_gripper_width").value
            ),
            "width_margin": float(self.get_parameter("width_margin").value),
            "corridor_radius": float(self.get_parameter("corridor_radius").value),
            "object_clearance": float(self.get_parameter("object_clearance").value),
            "collision_point_limit": int(self.get_parameter("collision_point_limit").value),
            "min_points": int(self.get_parameter("min_points").value),
        }

    def _axis_in_cloud_frame(
        self,
        axis_parameter: str,
        frame_parameter: str,
        cloud_frame: str,
    ) -> np.ndarray:
        axis = np.asarray(self.get_parameter(axis_parameter).value, dtype=np.float64)
        if axis.shape != (3,) or not np.isfinite(axis).all():
            raise ValueError(f"{axis_parameter} must contain three finite values")
        source_frame = str(self.get_parameter(frame_parameter).value).strip()
        if not source_frame or source_frame == cloud_frame:
            return axis
        try:
            transform = self._tf_buffer.lookup_transform(
                cloud_frame,
                source_frame,
                Time(),
                timeout=Duration(
                    seconds=float(self.get_parameter("approach_transform_timeout_s").value)
                ),
            )
        except TransformException as error:
            raise RuntimeError(
                f"failed to transform {axis_parameter} from "
                f"{source_frame} to {cloud_frame}: {error}"
            ) from error
        rotation = transform.transform.rotation
        return np.asarray(
            rotate_vector_by_quaternion(
                axis,
                (rotation.x, rotation.y, rotation.z, rotation.w),
            ),
            dtype=np.float64,
        )

    def _approach_in_cloud_frame(self, cloud_frame: str) -> np.ndarray:
        return self._axis_in_cloud_frame(
            self._grasp_profile.approach_axis_parameter,
            self._grasp_profile.approach_frame_parameter,
            cloud_frame,
        )

    def _current_closing_axis_in_cloud_frame(
        self, cloud_frame: str
    ) -> np.ndarray | None:
        tcp_frame = str(self.get_parameter("tcp_frame").value).strip()
        if not tcp_frame:
            return None
        try:
            transform = self._tf_buffer.lookup_transform(
                cloud_frame,
                tcp_frame,
                Time(),
                timeout=Duration(
                    seconds=float(
                        self.get_parameter("approach_transform_timeout_s").value
                    )
                ),
            )
        except TransformException as error:
            self.get_logger().warning(
                f"failed to select the nearest equivalent gripper roll from "
                f"{tcp_frame}: {error}; using the fitted closing-axis sign"
            )
            return None
        rotation = transform.transform.rotation
        return np.asarray(
            rotate_vector_by_quaternion(
                (0.0, 1.0, 0.0),
                (rotation.x, rotation.y, rotation.z, rotation.w),
            ),
            dtype=np.float64,
        )

    def _closing_axes_in_cloud_frame(
        self, cloud_frame: str
    ) -> tuple[np.ndarray | None, np.ndarray | None]:
        current = self._current_closing_axis_in_cloud_frame(cloud_frame)
        parameter = self._grasp_profile.closing_axis_parameter
        frame_parameter = self._grasp_profile.closing_axis_frame_parameter
        if parameter is None or frame_parameter is None:
            return None, current
        configured = self._axis_in_cloud_frame(
            parameter,
            frame_parameter,
            cloud_frame,
        )
        return (
            configured
            if current is None
            else align_symmetric_axis(configured, current),
            None,
        )

    def _fit(self, points: np.ndarray, cloud_frame: str):
        approach = self._approach_in_cloud_frame(cloud_frame)
        closing_axis, closing_axis_reference = self._closing_axes_in_cloud_frame(
            cloud_frame
        )
        candidates = self._graspnet.predict(points)
        fit_kwargs = self._fit_kwargs()
        _, _, target_center = robust_cloud_center(
            points,
            min_points=int(fit_kwargs.get("min_points", 100)),
        )
        diagnostics = summarize_graspnet_candidates(
            candidates,
            target_center,
            approach=approach,
            max_center_distance=float(self.get_parameter("candidate_center_tolerance").value),
            max_approach_angle_deg=float(
                self.get_parameter("candidate_approach_max_angle_deg").value
            ),
            max_width=None,
            limit=int(self.get_parameter("candidate_diagnostic_limit").value),
        )
        diagnostics.update({
            "backend": "graspnet",
            "grasp_mode": self._grasp_profile.name,
            "frame_id": cloud_frame,
            "inference_cloud": "segmented_product",
            "orientation_policy": self._grasp_profile.orientation_policy,
            "fit_method": self._grasp_profile.fit_method,
            "network_width_policy": "diagnostic_only",
            "robot_gripper_max_width_m": float(fit_kwargs["gripper_max_width"]),
            "gripper_width_limit_enforced": bool(
                fit_kwargs["enforce_gripper_width"]
            ),
            "required_closing_axis": (
                None if closing_axis is None else closing_axis.tolist()
            ),
            "closing_axis_policy": (
                "product_cloud_box_minor_axis"
                if self._grasp_profile.name == "basket"
                else "configured_axis"
            ),
            "closing_axis_reference_frame": (
                cloud_frame
                if self._grasp_profile.closing_axis_frame_parameter is None
                else str(
                    self.get_parameter(
                        self._grasp_profile.closing_axis_frame_parameter
                    ).value
                )
            ),
            "pointcloud_collision_check": "disabled",
        })
        diagnostic_message = String()
        diagnostic_message.data = json.dumps(diagnostics, ensure_ascii=False)
        self.candidate_pub.publish(diagnostic_message)
        return fit_graspnet_straight_grasp(
            points,
            None,
            candidates,
            approach=approach,
            closing_axis=closing_axis,
            closing_axis_reference=closing_axis_reference,
            use_cloud_minor_axis=self._grasp_profile.name == "basket",
            candidate_center_tolerance=float(
                self.get_parameter("candidate_center_tolerance").value
            ),
            candidate_approach_max_angle_deg=float(
                self.get_parameter("candidate_approach_max_angle_deg").value
            ),
            **fit_kwargs,
        )

    @staticmethod
    def _pose(header, position, quaternion) -> PoseStamped:
        pose = PoseStamped()
        pose.header = header
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, position)
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = map(
            float, quaternion
        )
        return pose

    def _publish(self, cloud_message, estimate):
        grasp = self._pose(cloud_message.header, estimate.center, estimate.quaternion_xyzw)
        pregrasp = self._pose(cloud_message.header, estimate.pregrasp, estimate.quaternion_xyzw)
        if estimate.valid:
            self.grasp_pub.publish(grasp)
            self.pregrasp_pub.publish(pregrasp)
            width = Float32()
            width.data = float(estimate.required_width)
            self.width_pub.publish(width)

        path = Path()
        path.header = cloud_message.header
        path.poses = [pregrasp, grasp]
        self.path_pub.publish(path)
        self.marker_pub.publish(self._markers(cloud_message.header, estimate))

        payload = {
            "valid": estimate.valid,
            "reason": estimate.reason,
            "backend": estimate.backend,
            "grasp_mode": self._grasp_profile.name,
            "frame_id": cloud_message.header.frame_id,
            "center_m": estimate.center.tolist(),
            "surface_center_m": (
                estimate.surface_center.tolist()
                if estimate.surface_center is not None
                else estimate.center.tolist()
            ),
            "grasp_depth_offset_m": float(
                np.linalg.norm(
                    estimate.center
                    - (
                        estimate.surface_center
                        if estimate.surface_center is not None
                        else estimate.center
                    )
                )
            ),
            "pregrasp_m": estimate.pregrasp.tolist(),
            "quaternion_xyzw": estimate.quaternion_xyzw.tolist(),
            "approach_axis": estimate.approach.tolist(),
            "approach_reference_frame": str(
                self.get_parameter(
                    self._grasp_profile.approach_frame_parameter
                ).value
            ),
            "closing_axis": estimate.closing_axis.tolist(),
            "closing_axis_reference_frame": (
                cloud_message.header.frame_id
                if self._grasp_profile.closing_axis_frame_parameter is None
                else str(
                    self.get_parameter(
                        self._grasp_profile.closing_axis_frame_parameter
                    ).value
                )
            ),
            "orientation_policy": self._grasp_profile.orientation_policy,
            "grasp_width_m": estimate.grasp_width,
            "network_grasp_width_m": estimate.network_grasp_width,
            "required_width_m": estimate.required_width,
            "object_spans_m": estimate.object_spans.tolist(),
            "collision_checked": estimate.collision_checked,
            "obstacle_count": estimate.obstacle_count,
            "score": estimate.score,
            "network_score": estimate.network_score,
            "candidate_count": estimate.candidate_count,
            "constrained_candidate_count": estimate.constrained_candidate_count,
            "fit_contributor_count": estimate.constrained_candidate_count,
            "fit_method": self._grasp_profile.fit_method,
            "candidate_center_distance_m": estimate.candidate_center_distance,
            "candidate_approach_alignment": estimate.candidate_approach_alignment,
            "execution_enabled": False,
            "pointcloud_collision_check": "disabled",
        }
        result = String()
        result.data = json.dumps(payload, ensure_ascii=False)
        self.result_pub.publish(result)
        if estimate.valid:
            self.get_logger().info(result.data)
        else:
            self.get_logger().warning(result.data)

    @staticmethod
    def _markers(header, estimate):
        color = (0.1, 0.85, 0.2) if estimate.valid else (0.9, 0.15, 0.1)
        arrow = Marker()
        arrow.header = header
        arrow.ns = "straight_grasp"
        arrow.id = 0
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.points = [
            Point(x=float(estimate.pregrasp[0]), y=float(estimate.pregrasp[1]), z=float(estimate.pregrasp[2])),
            Point(x=float(estimate.center[0]), y=float(estimate.center[1]), z=float(estimate.center[2])),
        ]
        arrow.scale.x = 0.008
        arrow.scale.y = 0.016
        arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = (*color, 1.0)

        gripper = Marker()
        gripper.header = header
        gripper.ns = "straight_grasp"
        gripper.id = 1
        gripper.type = Marker.CUBE
        gripper.action = Marker.ADD
        gripper.pose.position.x, gripper.pose.position.y, gripper.pose.position.z = map(float, estimate.center)
        quaternion = estimate.quaternion_xyzw
        gripper.pose.orientation.x, gripper.pose.orientation.y = map(float, quaternion[:2])
        gripper.pose.orientation.z, gripper.pose.orientation.w = map(float, quaternion[2:])
        gripper.scale.x = 0.02
        gripper.scale.y = float(estimate.required_width)
        gripper.scale.z = 0.06
        gripper.color.r, gripper.color.g, gripper.color.b, gripper.color.a = (*color, 0.45)
        return MarkerArray(markers=[arrow, gripper])


def main(args=None):
    rclpy.init(args=args)
    node = StraightGraspNode()
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
