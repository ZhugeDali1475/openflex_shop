"""Guarded task-level reverse primitive behind collision and localization safety."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("odom_topic", default_value="/odom_safe"),
            DeclareLaunchArgument(
                "reverse_authorization_topic",
                default_value="/competition/post_pick_reverse_authorized",
            ),
            Node(
                package="openarmx_task_executor",
                executable="station_retreat",
                name="station_retreat",
                output="screen",
                parameters=[
                    {
                        "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                        "odom_topic": LaunchConfiguration("odom_topic"),
                        "reverse_authorization_topic": LaunchConfiguration(
                            "reverse_authorization_topic"
                        ),
                    }
                ],
            ),
        ]
    )
