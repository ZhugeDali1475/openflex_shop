import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cartesian = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("openflex_grasp_moveit"),
                "launch", "cartesian_approach_executor.launch.py",
            )
        )
    )
    cameras = []
    for arm in ("right", "left"):
        cameras.append(Node(
            package="realsense2_camera", executable="realsense2_camera_node",
            namespace=f"cam_{arm}", name=f"cam_{arm}", output="screen",
            condition=IfCondition(LaunchConfiguration("start_cameras")),
            parameters=[{
                "serial_no": ParameterValue(
                    LaunchConfiguration(f"{arm}_camera_serial"), value_type=str
                ),
                "camera_name": f"{arm}_camera", "enable_color": True,
                "enable_depth": True, "align_depth.enable": True,
                "pointcloud.enable": False, "publish_tf": True,
            }],
            remappings=[
                (f"/cam_{arm}/cam_{arm}/color/image_raw", f"/cam_{arm}/color/image"),
                (f"/cam_{arm}/cam_{arm}/aligned_depth_to_color/image_raw", f"/cam_{arm}/depth/image"),
                (f"/cam_{arm}/cam_{arm}/aligned_depth_to_color/camera_info", f"/cam_{arm}/depth/camera_info"),
            ],
        ))
    return LaunchDescription([
        DeclareLaunchArgument("start_cameras", default_value="false"),
        DeclareLaunchArgument("right_camera_serial", default_value=os.environ.get("OPENFLEX_RIGHT_CAMERA_SERIAL", "")),
        DeclareLaunchArgument("left_camera_serial", default_value=os.environ.get("OPENFLEX_LEFT_CAMERA_SERIAL", "")),
        DeclareLaunchArgument("start_detect_server", default_value="true"),
        DeclareLaunchArgument("start_graspnet_worker", default_value="true"),
        DeclareLaunchArgument("start_pick_server", default_value="true"),
        DeclareLaunchArgument("execute", default_value="false"),
        DeclareLaunchArgument("grasp_depth_offset_m", default_value="0.05"),
        DeclareLaunchArgument("object_bottom_clearance_m", default_value="0.01"),
        *cameras,
        cartesian,
        Node(
            package="openflex_grasp", executable="detect_object_server",
            name="openflex_detect_object", output="screen",
            condition=IfCondition(LaunchConfiguration("start_detect_server")),
            parameters=[{
                "detection_ttl_s": 180.0,
                "right_camera_serial": ParameterValue(
                    LaunchConfiguration("right_camera_serial"), value_type=str
                ),
                "left_camera_serial": ParameterValue(
                    LaunchConfiguration("left_camera_serial"), value_type=str
                ),
            }],
        ),
        Node(
            package="openflex_grasp", executable="graspnet_worker",
            name="openflex_graspnet_worker", output="screen",
            condition=IfCondition(LaunchConfiguration("start_graspnet_worker")),
            parameters=[{
                "grasp_depth_offset_m": ParameterValue(
                    LaunchConfiguration("grasp_depth_offset_m"), value_type=float
                ),
            }],
        ),
        Node(
            package="openflex_grasp", executable="pick_object_server",
            name="openflex_pick_object", output="screen",
            condition=IfCondition(LaunchConfiguration("start_pick_server")),
            parameters=[{
                "execute": ParameterValue(
                    LaunchConfiguration("execute"), value_type=bool
                ),
                "object_bottom_clearance_m": ParameterValue(
                    LaunchConfiguration("object_bottom_clearance_m"), value_type=float
                ),
            }],
        ),
        Node(
            package="openflex_grasp", executable="pick_sequence_server",
            name="openflex_pick_sequence", output="screen",
            condition=IfCondition(LaunchConfiguration("start_pick_server")),
            parameters=[{
                "object_bottom_clearance_m": ParameterValue(
                    LaunchConfiguration("object_bottom_clearance_m"), value_type=float
                ),
            }],
        ),
        Node(
            package="openflex_grasp", executable="product_grasp_web",
            name="openflex_grasp_readonly_web", output="screen",
            parameters=[{"open_browser": False}],
        ),
    ])
