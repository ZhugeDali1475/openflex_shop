"""supermarket_test_05 competition navigation entry point."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    nav_dir = Path(get_package_share_directory("swerve_navigation"))
    map_root = Path.home() / "openflex_all" / "openflex_maps"
    args = {
        "map_yaml": LaunchConfiguration("map_yaml"),
        "pcd_path": LaunchConfiguration("pcd_path"),
        "initial_pose": LaunchConfiguration("initial_pose"),
        "use_rviz": LaunchConfiguration("use_rviz"),
        "nav_controller_max_wheel_speed": LaunchConfiguration(
            "nav_controller_max_wheel_speed"
        ),
        "nav_controller_wheel_accel_limit": LaunchConfiguration(
            "nav_controller_wheel_accel_limit"
        ),
        "start_robot_hardware": LaunchConfiguration("start_robot_hardware"),
        "start_safe_odom_mux": LaunchConfiguration("start_safe_odom_mux"),
        "reverse_authorization_topic": LaunchConfiguration(
            "reverse_authorization_topic"
        ),
        "reverse_authorization_timeout": LaunchConfiguration(
            "reverse_authorization_timeout"
        ),
    }
    full_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(nav_dir / "launch" / "full_system.launch.py")),
        launch_arguments={
            **args,
            "use_gps": "false",
            "nav2_params_file": str(
                nav_dir / "config" / "nav2_params_supermarket_test_03.yaml"
            ),
            "allow_reverse": "false",
            "collision_monitor_params_file": str(
                nav_dir / "config" / "collision_monitor_supermarket_test_03.yaml"
            ),
            "nav_to_pose_bt_xml": str(
                nav_dir / "behavior_trees" / "navigate_to_pose_supermarket_test_03.xml"
            ),
            "nav_through_poses_bt_xml": str(
                nav_dir
                / "behavior_trees"
                / "navigate_through_poses_supermarket_test_03.xml"
            ),
        }.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_yaml",
                default_value=str(map_root / "2d" / "supermarket_test_05" / "map.yaml"),
            ),
            DeclareLaunchArgument(
                "pcd_path",
                default_value=str(map_root / "3d" / "supermarket_test_05" / "map.pcd"),
            ),
            DeclareLaunchArgument(
                "initial_pose",
                default_value="[-2.434000,-1.172000,0.0,0.0,0.0,0.0]",
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("start_robot_hardware", default_value="true"),
            DeclareLaunchArgument("start_safe_odom_mux", default_value="true"),
            DeclareLaunchArgument(
                "reverse_authorization_topic",
                default_value="/competition/post_pick_reverse_authorized",
            ),
            DeclareLaunchArgument(
                "reverse_authorization_timeout", default_value="0.25"
            ),
            DeclareLaunchArgument("nav_controller_max_wheel_speed", default_value="0.60"),
            DeclareLaunchArgument(
                "nav_controller_wheel_accel_limit", default_value="0.50"
            ),
            full_system,
        ]
    )
