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

    camera = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="cam_left",
        namespace="cam_left",
        condition=IfCondition(start_camera),
        output="screen",
        parameters=[{
            "serial_no": ParameterValue(serial_no, value_type=str),
            "camera_name": "left_camera",
            "wait_for_device_timeout": -1.0,
            "reconnect_timeout": 2.0,
            "depth_module.depth_profile": "480x270x15",
            "depth_module.color_profile": "480x270x15",
            "color_qos": "SENSOR_DATA",
            "base_frame_id": "left_camera_link",
            "depth_frame_id": "left_camera_depth_frame",
            "depth_optical_frame_id": "left_camera_depth_optical_frame",
            "color_frame_id": "left_camera_color_frame",
            "color_optical_frame_id": "left_camera_color_optical_frame",
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
            ("/cam_left/cam_left/color/image_raw", "/cam_left/color/image"),
            ("/cam_left/cam_left/color/image_rect_raw", "/cam_left/color/image"),
            ("/cam_left/cam_left/aligned_depth_to_color/image_raw", "/cam_left/depth/image"),
            ("/cam_left/cam_left/aligned_depth_to_color/camera_info", "/cam_left/depth/camera_info"),
        ],
    )

    perception = Node(
        package="openflex_grasp",
        executable="product_pointcloud_node",
        output="screen",
        parameters=[{
            "target": target,
            "qwen_model": qwen_model,
            "sam_model": sam_model,
            "depth_scale": 0.001,
            "rgb_topic": "/cam_left/color/image",
            "depth_topic": "/cam_left/depth/image",
            "camera_info_topic": "/cam_left/depth/camera_info",
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
            "approach_frame": "base_link",
            "closing_axis": [0.0, 1.0, 0.0],
            "closing_axis_frame": "base_link",
            "pregrasp_distance": 0.12,
            "gripper_max_width": 0.088,
            "graspnet_repo": graspnet_repo,
            "graspnet_checkpoint": graspnet_checkpoint,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("start_camera", default_value="true"),
        DeclareLaunchArgument("target", default_value="water bottle"),
        DeclareLaunchArgument(
            "serial_no",
            default_value=os.environ.get("OPENFLEX_LEFT_CAMERA_SERIAL", ""),
        ),
        DeclareLaunchArgument("grasp_backend", default_value="graspnet"),
        DeclareLaunchArgument(
            "grasp_mode",
            default_value="shelf",
            description="Grasp direction profile: shelf or basket.",
        ),
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
        perception,
    ])
