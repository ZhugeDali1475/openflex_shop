import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


OPENFLEX_ROOT = os.environ.get("OPENFLEX_ROOT", "/home/openflex/openflex_shop")
WORKSPACE_ROOT = os.environ.get("OPENFLEX_WORKSPACE", os.path.join(OPENFLEX_ROOT, "openflex_ws"))
GRASP_SUITE_ROOT = os.environ.get(
    "OPENFLEX_GRASP_SUITE_ROOT", os.path.join(WORKSPACE_ROOT, "src", "openflex_grasp")
)
MODEL_ROOT = os.environ.get("OPENFLEX_MODEL_ROOT", os.path.join(OPENFLEX_ROOT, "openflex_models"))


def generate_launch_description():
    start_camera = LaunchConfiguration("start_camera")
    target = LaunchConfiguration("target")
    qwen_model = LaunchConfiguration("qwen_model")
    sam_model = LaunchConfiguration("sam_model")
    serial_no = LaunchConfiguration("serial_no")
    grasp_backend = LaunchConfiguration("grasp_backend")
    graspnet_repo = LaunchConfiguration("graspnet_repo")
    graspnet_checkpoint = LaunchConfiguration("graspnet_checkpoint")
    start_moveit_bridge = LaunchConfiguration("start_moveit_bridge")
    start_grasp_executor = LaunchConfiguration("start_grasp_executor")
    grasp_product = LaunchConfiguration("grasp_product")
    start_web_visualizer = LaunchConfiguration("start_web_visualizer")

    camera = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="cam_right",
        namespace="cam_right",
        condition=IfCondition(start_camera),
        output="screen",
        parameters=[{
            "serial_no": ParameterValue(serial_no, value_type=str),
            "camera_name": "right_camera",
            "wait_for_device_timeout": -1.0,
            "reconnect_timeout": 2.0,
            "depth_module.depth_profile": "480x270x15",
            "depth_module.color_profile": "480x270x15",
            "color_qos": "SENSOR_DATA",
            "base_frame_id": "right_camera_link",
            "depth_frame_id": "right_camera_depth_frame",
            "depth_optical_frame_id": "right_camera_depth_optical_frame",
            "color_frame_id": "right_camera_color_frame",
            "color_optical_frame_id": "right_camera_color_optical_frame",
            "enable_color": True,
            "enable_depth": True,
            "align_depth.enable": True,
            "enable_infra1": False,
            "enable_infra2": False,
            "enable_gyro": False,
            "enable_accel": False,
            "pointcloud.enable": False,
            "publish_tf": True,
        }],
        remappings=[
            ("/cam_right/cam_right/color/image_raw", "/cam_right/color/image"),
            ("/cam_right/cam_right/color/image_rect_raw", "/cam_right/color/image"),
            ("/cam_right/cam_right/aligned_depth_to_color/image_raw", "/cam_right/depth/image"),
            ("/cam_right/cam_right/aligned_depth_to_color/camera_info", "/cam_right/depth/camera_info"),
        ],
    )

    perception = Node(
        package="openflex_grasp",
        executable="product_pointcloud_node",
        output="screen",
        parameters=[{
            "target": ParameterValue(target, value_type=str),
            "qwen_model": qwen_model,
            "sam_model": sam_model,
            "depth_scale": 0.001,
            "rgb_topic": "/cam_right/color/image",
            "depth_topic": "/cam_right/depth/image",
            "camera_info_topic": "/cam_right/depth/camera_info",
        }],
    )

    grasp_fitter = Node(
        package="openflex_grasp",
        executable="straight_grasp_node",
        output="screen",
        parameters=[{
            "backend": grasp_backend,
            "grasp_mode": LaunchConfiguration("grasp_mode"),
            "approach_axis": [1.0, 0.0, 0.0],
            "approach_frame": LaunchConfiguration("grasp_approach_frame"),
            "closing_axis": [0.0, -1.0, 0.0],
            "closing_axis_frame": LaunchConfiguration("grasp_closing_axis_frame"),
            "tcp_frame": LaunchConfiguration("grasp_tcp_frame"),
            "pregrasp_distance": ParameterValue(
                LaunchConfiguration("grasp_pregrasp_distance_m"), value_type=float
            ),
            "grasp_depth_offset": ParameterValue(
                LaunchConfiguration("grasp_depth_offset_m"), value_type=float
            ),
            "gripper_max_width": ParameterValue(
                LaunchConfiguration("grasp_gripper_max_opening_m"), value_type=float
            ),
            "width_margin": ParameterValue(
                LaunchConfiguration("grasp_width_margin_m"), value_type=float
            ),
            "graspnet_repo": graspnet_repo,
            "graspnet_checkpoint": graspnet_checkpoint,
            "candidate_center_tolerance": ParameterValue(
                LaunchConfiguration("grasp_candidate_center_tolerance_m"), value_type=float
            ),
            "candidate_approach_max_angle_deg": ParameterValue(
                LaunchConfiguration("grasp_candidate_approach_max_angle_deg"), value_type=float
            ),
        }],
    )

    moveit_bridge = Node(
        package="openflex_grasp",
        executable="product_moveit_bridge",
        name="right_wrist_product_moveit_bridge",
        output="screen",
        condition=IfCondition(start_moveit_bridge),
        parameters=[{
            "arm": "right",
            "pregrasp_topic": "/product_perception/pregrasp_pose",
            "target_frame": LaunchConfiguration("moveit_target_frame"),
            "plan_action": LaunchConfiguration("moveit_action"),
            "execute": ParameterValue(LaunchConfiguration("moveit_execute"), value_type=bool),
            "max_dof": ParameterValue(LaunchConfiguration("moveit_max_dof"), value_type=str),
            "base_yaw_mode": LaunchConfiguration("moveit_base_yaw_mode"),
            "desired_base_yaw_odom": ParameterValue(
                LaunchConfiguration("moveit_desired_base_yaw_odom"),
                value_type=float,
            ),
            "base_yaw_tolerance": ParameterValue(
                LaunchConfiguration("moveit_base_yaw_tolerance"),
                value_type=float,
            ),
            "base_yaw_samples": ParameterValue(
                LaunchConfiguration("moveit_base_yaw_samples"),
                value_type=int,
            ),
            "front_distance": ParameterValue(
                LaunchConfiguration("moveit_front_distance"),
                value_type=float,
            ),
            "velocity_scale": ParameterValue(
                LaunchConfiguration("moveit_velocity_scale"),
                value_type=float,
            ),
            "acceleration_scale": ParameterValue(
                LaunchConfiguration("moveit_acceleration_scale"),
                value_type=float,
            ),
            "planning_timeout_sec": ParameterValue(
                LaunchConfiguration("moveit_planning_timeout_sec"),
                value_type=float,
            ),
            "use_latest_transform": ParameterValue(
                LaunchConfiguration("moveit_use_latest_transform"),
                value_type=bool,
            ),
            "transform_timeout_s": ParameterValue(
                LaunchConfiguration("moveit_transform_timeout_s"),
                value_type=float,
            ),
            "action_wait_timeout_s": ParameterValue(
                LaunchConfiguration("moveit_action_wait_timeout_s"),
                value_type=float,
            ),
        }],
    )

    grasp_executor = Node(
        package="openflex_grasp",
        executable="product_grasp_executor",
        name="right_product_grasp_executor",
        output="screen",
        condition=IfCondition(start_grasp_executor),
        parameters=[{
            "enabled": ParameterValue(LaunchConfiguration("grasp_enabled"), value_type=bool),
            "execute": ParameterValue(LaunchConfiguration("grasp_execute"), value_type=bool),
            "arm": "right",
            "grasp_mode": LaunchConfiguration("grasp_mode"),
            "product": ParameterValue(grasp_product, value_type=str),
            "target_topic": "/product_perception/target",
            "target_frame": LaunchConfiguration("moveit_target_frame"),
            "tcp_frame": LaunchConfiguration("grasp_tcp_frame"),
            "cartesian_action": LaunchConfiguration("cartesian_action"),
            "plan_action": LaunchConfiguration("moveit_action"),
            "gripper_action": "/right_gripper_controller/gripper_cmd",
            "send_once": True,
            "photo_pose_timeout_s": ParameterValue(
                LaunchConfiguration("grasp_photo_pose_timeout_s"), value_type=float
            ),
            "action_wait_timeout_s": ParameterValue(
                LaunchConfiguration("grasp_action_wait_timeout_s"), value_type=float
            ),
            "perception_wait_timeout_s": ParameterValue(
                LaunchConfiguration("grasp_perception_wait_timeout_s"), value_type=float
            ),
            "camera_settle_s": ParameterValue(
                LaunchConfiguration("grasp_camera_settle_s"), value_type=float
            ),
            "return_to_photo": ParameterValue(
                LaunchConfiguration("grasp_return_to_photo"), value_type=bool
            ),
            "postgrasp_lift_m": ParameterValue(
                LaunchConfiguration("grasp_postgrasp_lift_m"), value_type=float
            ),
            "basket_final_descent_reduction_m": ParameterValue(
                LaunchConfiguration("grasp_basket_final_descent_reduction_m"),
                value_type=float,
            ),
            "pregrasp_max_dof": ParameterValue(
                LaunchConfiguration("grasp_pregrasp_max_dof"), value_type=str
            ),
            "unconstrained_moveit": ParameterValue(
                LaunchConfiguration("grasp_unconstrained_moveit"), value_type=bool
            ),
            "prefer_straight_grasp_feed": ParameterValue(
                LaunchConfiguration("grasp_prefer_straight_feed"), value_type=bool
            ),
            "straight_line_tolerance_m": ParameterValue(
                LaunchConfiguration("grasp_straight_line_tolerance_m"), value_type=float
            ),
            "cartesian_eef_step_m": ParameterValue(
                LaunchConfiguration("grasp_cartesian_eef_step_m"), value_type=float
            ),
            "cartesian_min_fraction": ParameterValue(
                LaunchConfiguration("grasp_cartesian_min_fraction"), value_type=float
            ),
            "cartesian_velocity_scale": ParameterValue(
                LaunchConfiguration("grasp_cartesian_velocity_scale"), value_type=float
            ),
            "cartesian_acceleration_scale": ParameterValue(
                LaunchConfiguration("grasp_cartesian_acceleration_scale"), value_type=float
            ),
            "photo_pregrasp_corridor_radius_m": ParameterValue(
                LaunchConfiguration("grasp_corridor_radius_m"), value_type=float
            ),
            "corridor_max_path_scale": ParameterValue(
                LaunchConfiguration("grasp_corridor_max_path_scale"), value_type=float
            ),
            "corridor_max_extra_path_m": ParameterValue(
                LaunchConfiguration("grasp_corridor_max_extra_path_m"), value_type=float
            ),
            "corridor_max_backtracking_m": ParameterValue(
                LaunchConfiguration("grasp_corridor_max_backtracking_m"), value_type=float
            ),
            "corridor_planning_timeout_s": ParameterValue(
                LaunchConfiguration("grasp_corridor_planning_timeout_s"), value_type=float
            ),
            "cartesian_start_position_tolerance_m": ParameterValue(
                LaunchConfiguration("grasp_cartesian_start_position_tolerance_m"), value_type=float
            ),
            "cartesian_start_orientation_tolerance_deg": ParameterValue(
                LaunchConfiguration("grasp_cartesian_start_orientation_tolerance_deg"), value_type=float
            ),
            "gripper_close_position_m": ParameterValue(
                LaunchConfiguration("grasp_gripper_close_position_m"), value_type=float
            ),
            "gripper_joint_max_position_m": ParameterValue(
                LaunchConfiguration("grasp_gripper_joint_max_position_m"), value_type=float
            ),
            "gripper_open_stall_tolerance_m": ParameterValue(
                LaunchConfiguration("grasp_gripper_open_stall_tolerance_m"), value_type=float
            ),
            "gripper_opening_scale": ParameterValue(
                LaunchConfiguration("grasp_gripper_opening_scale"), value_type=float
            ),
            "gripper_preopen_margin_m": ParameterValue(
                LaunchConfiguration("grasp_width_margin_m"), value_type=float
            ),
            "gripper_grip_compression_m": ParameterValue(
                LaunchConfiguration("grasp_gripper_grip_compression_m"), value_type=float
            ),
            "payload_compensation_enabled": ParameterValue(
                LaunchConfiguration("grasp_payload_compensation_enabled"), value_type=bool
            ),
            "payload_mass_kg": ParameterValue(
                LaunchConfiguration("grasp_payload_mass_kg"), value_type=float
            ),
            "payload_com_x_m": ParameterValue(
                LaunchConfiguration("grasp_payload_com_x_m"), value_type=float
            ),
            "payload_com_y_m": ParameterValue(
                LaunchConfiguration("grasp_payload_com_y_m"), value_type=float
            ),
            "payload_com_z_m": ParameterValue(
                LaunchConfiguration("grasp_payload_com_z_m"), value_type=float
            ),
        }],
    )

    web_visualizer = Node(
        package="openflex_grasp",
        executable="product_grasp_web",
        name="right_grasp_web_visualizer",
        output="screen",
        condition=IfCondition(start_web_visualizer),
        parameters=[{
            "product": ParameterValue(grasp_product, value_type=str),
            "port": ParameterValue(LaunchConfiguration("web_port"), value_type=int),
            "open_browser": ParameterValue(
                LaunchConfiguration("web_open_browser"), value_type=bool
            ),
            "rgb_topic": "/cam_right/color/image",
            "overlay_topic": "/product_perception/overlay",
            "mask_topic": "/product_perception/mask",
            "product_cloud_topic": "/product_perception/points",
            "scene_cloud_topic": "/product_perception/scene_points",
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("start_camera", default_value="true"),
        DeclareLaunchArgument("target", default_value="water bottle"),
        DeclareLaunchArgument(
            "serial_no",
            default_value=os.environ.get("OPENFLEX_RIGHT_CAMERA_SERIAL", ""),
        ),
        DeclareLaunchArgument("grasp_backend", default_value="graspnet"),
        DeclareLaunchArgument(
            "grasp_mode",
            default_value="shelf",
            description="Grasp direction profile: shelf (front approach) or basket (top-down approach).",
        ),
        DeclareLaunchArgument("start_moveit_bridge", default_value="false"),
        DeclareLaunchArgument("start_grasp_executor", default_value="false"),
        DeclareLaunchArgument("grasp_enabled", default_value="false"),
        DeclareLaunchArgument("grasp_execute", default_value="false"),
        DeclareLaunchArgument("start_web_visualizer", default_value="false"),
        DeclareLaunchArgument("web_port", default_value="8765"),
        DeclareLaunchArgument("web_open_browser", default_value="true"),
        DeclareLaunchArgument("grasp_product", default_value=target),
        DeclareLaunchArgument("grasp_tcp_frame", default_value="openarmx_right_hand_tcp"),
        DeclareLaunchArgument("grasp_photo_pose_timeout_s", default_value="15.0"),
        DeclareLaunchArgument("grasp_action_wait_timeout_s", default_value="30.0"),
        DeclareLaunchArgument("grasp_perception_wait_timeout_s", default_value="300.0"),
        DeclareLaunchArgument("grasp_camera_settle_s", default_value="1.0"),
        DeclareLaunchArgument("grasp_return_to_photo", default_value="true"),
        DeclareLaunchArgument("grasp_postgrasp_lift_m", default_value="0.01"),
        DeclareLaunchArgument(
            "grasp_basket_final_descent_reduction_m", default_value="0.0"
        ),
        DeclareLaunchArgument("grasp_pregrasp_max_dof", default_value="auto"),
        DeclareLaunchArgument("grasp_unconstrained_moveit", default_value="true"),
        DeclareLaunchArgument("grasp_prefer_straight_feed", default_value="true"),
        DeclareLaunchArgument("grasp_approach_frame", default_value="base_link"),
        DeclareLaunchArgument("grasp_closing_axis_frame", default_value="base_link"),
        DeclareLaunchArgument("grasp_pregrasp_distance_m", default_value="0.12"),
        DeclareLaunchArgument("grasp_depth_offset_m", default_value="0.015"),
        DeclareLaunchArgument("grasp_candidate_center_tolerance_m", default_value="0.06"),
        DeclareLaunchArgument("grasp_candidate_approach_max_angle_deg", default_value="25.0"),
        DeclareLaunchArgument("grasp_straight_line_tolerance_m", default_value="0.005"),
        DeclareLaunchArgument("grasp_cartesian_eef_step_m", default_value="0.005"),
        DeclareLaunchArgument("grasp_cartesian_min_fraction", default_value="0.99"),
        DeclareLaunchArgument("grasp_cartesian_velocity_scale", default_value="0.08"),
        DeclareLaunchArgument("grasp_cartesian_acceleration_scale", default_value="0.08"),
        DeclareLaunchArgument("grasp_corridor_radius_m", default_value="0.06"),
        DeclareLaunchArgument("grasp_corridor_max_path_scale", default_value="2.5"),
        DeclareLaunchArgument("grasp_corridor_max_extra_path_m", default_value="0.10"),
        DeclareLaunchArgument("grasp_corridor_max_backtracking_m", default_value="0.03"),
        DeclareLaunchArgument("grasp_corridor_planning_timeout_s", default_value="8.0"),
        DeclareLaunchArgument("grasp_cartesian_start_position_tolerance_m", default_value="0.015"),
        DeclareLaunchArgument("grasp_cartesian_start_orientation_tolerance_deg", default_value="10.0"),
        DeclareLaunchArgument("grasp_width_margin_m", default_value="0.004"),
        DeclareLaunchArgument("grasp_gripper_max_opening_m", default_value="0.086"),
        DeclareLaunchArgument("grasp_gripper_joint_max_position_m", default_value="0.043"),
        DeclareLaunchArgument("grasp_gripper_open_stall_tolerance_m", default_value="0.002"),
        DeclareLaunchArgument("grasp_gripper_opening_scale", default_value="2.0"),
        DeclareLaunchArgument("grasp_gripper_grip_compression_m", default_value="0.003"),
        DeclareLaunchArgument("grasp_gripper_close_position_m", default_value="0.0"),
        DeclareLaunchArgument("grasp_payload_compensation_enabled", default_value="false"),
        DeclareLaunchArgument("grasp_payload_mass_kg", default_value="0.55"),
        DeclareLaunchArgument("grasp_payload_com_x_m", default_value="0.0"),
        DeclareLaunchArgument("grasp_payload_com_y_m", default_value="0.0"),
        DeclareLaunchArgument("grasp_payload_com_z_m", default_value="0.05"),
        DeclareLaunchArgument("moveit_execute", default_value="false"),
        DeclareLaunchArgument("moveit_max_dof", default_value="auto"),
        DeclareLaunchArgument("moveit_target_frame", default_value="odom"),
        DeclareLaunchArgument("moveit_action", default_value="/openflex/plan_pregrasp"),
        DeclareLaunchArgument("cartesian_action", default_value="/openflex/cartesian_approach"),
        DeclareLaunchArgument("moveit_base_yaw_mode", default_value="current"),
        DeclareLaunchArgument("moveit_desired_base_yaw_odom", default_value="0.0"),
        DeclareLaunchArgument("moveit_base_yaw_tolerance", default_value="0.0"),
        DeclareLaunchArgument("moveit_base_yaw_samples", default_value="0"),
        DeclareLaunchArgument("moveit_front_distance", default_value="0.0"),
        DeclareLaunchArgument("moveit_velocity_scale", default_value="0.0"),
        DeclareLaunchArgument("moveit_acceleration_scale", default_value="0.0"),
        DeclareLaunchArgument("moveit_planning_timeout_sec", default_value="0.0"),
        DeclareLaunchArgument("moveit_use_latest_transform", default_value="true"),
        DeclareLaunchArgument("moveit_transform_timeout_s", default_value="1.0"),
        DeclareLaunchArgument("moveit_action_wait_timeout_s", default_value="5.0"),
        DeclareLaunchArgument(
            "graspnet_repo",
            default_value=os.environ.get(
                "OPENFLEX_GRASPNET_REPO", os.path.join(GRASP_SUITE_ROOT, "graspnet-baseline")
            ),
        ),
        DeclareLaunchArgument(
            "graspnet_checkpoint",
            default_value=os.environ.get(
                "OPENFLEX_GRASPNET_CHECKPOINT",
                os.path.join(MODEL_ROOT, "graspnet-baseline", "checkpoint-rs.tar"),
            ),
        ),
        DeclareLaunchArgument(
            "qwen_model",
            default_value=os.environ.get(
                "OPENFLEX_QWEN_MODEL", os.path.join(MODEL_ROOT, "Qwen3.5-4B")
            ),
        ),
        DeclareLaunchArgument(
            "sam_model",
            default_value=os.environ.get(
                "OPENFLEX_SAM_MODEL", os.path.join(MODEL_ROOT, "sam2.1-hiera-small")
            ),
        ),
        camera,
        grasp_fitter,
        moveit_bridge,
        grasp_executor,
        web_visualizer,
        perception,
    ])
