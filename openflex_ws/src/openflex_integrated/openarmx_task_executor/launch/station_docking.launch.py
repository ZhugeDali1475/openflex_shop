from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("marker_frame_prefix", default_value="station_marker_"),
            DeclareLaunchArgument("target_marker_x", default_value="0.65"),
            DeclareLaunchArgument("target_marker_y", default_value="0.0"),
            DeclareLaunchArgument("target_marker_yaw", default_value="0.0"),
            Node(
                package="openarmx_task_executor",
                executable="station_docking",
                name="station_docking",
                output="screen",
                parameters=[
                    {
                        "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                        "marker_frame_prefix": LaunchConfiguration("marker_frame_prefix"),
                        "target_marker_x": LaunchConfiguration("target_marker_x"),
                        "target_marker_y": LaunchConfiguration("target_marker_y"),
                        "target_marker_yaw": LaunchConfiguration("target_marker_yaw"),
                    }
                ],
            ),
        ]
    )
