import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    camera = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="right_camera",
        namespace="cam_right",
        output="screen",
        parameters=[{
            "serial_no": ParameterValue(
                LaunchConfiguration("serial_no"), value_type=str
            ),
            "camera_name": "right_camera",
            "wait_for_device_timeout": -1.0,
            "reconnect_timeout": 2.0,
            "depth_module.depth_profile": "480x270x15",
            "depth_module.color_profile": "480x270x15",
            "color_qos": "SENSOR_DATA",
            "enable_color": True,
            "enable_depth": True,
            "align_depth.enable": True,
            "pointcloud.enable": True,
            "pointcloud.ordered_pc": False,
            "enable_infra1": False,
            "enable_infra2": False,
            "enable_gyro": False,
            "enable_accel": False,
            # URDF owns wrist -> camera_link; RealSense owns internal frames.
            "publish_tf": True,
        }],
        remappings=[
            ("/cam_right/right_camera/color/image_raw", "/cam_right/color/image"),
            (
                "/cam_right/right_camera/aligned_depth_to_color/image_raw",
                "/cam_right/depth/image",
            ),
            (
                "/cam_right/right_camera/aligned_depth_to_color/camera_info",
                "/cam_right/depth/camera_info",
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_no",
            default_value=os.environ.get("OPENFLEX_RIGHT_CAMERA_SERIAL", ""),
        ),
        camera,
    ])
