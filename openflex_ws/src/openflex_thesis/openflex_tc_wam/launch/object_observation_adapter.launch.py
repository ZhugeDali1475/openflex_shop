"""Adapt a timestamped external/DREAM object pose into thesis object memory."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("openflex_tc_wam")
    default_config = os.path.join(share, "config", "tc_wam.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument(
                "pose_topic", default_value="/dream/active_object_pose"
            ),
            DeclareLaunchArgument("object_id", default_value="thesis_item_01"),
            Node(
                package="openflex_tc_wam",
                executable="object_observation_adapter",
                name="object_observation_adapter",
                parameters=[
                    LaunchConfiguration("config"),
                    {
                        "pose_topic": LaunchConfiguration("pose_topic"),
                        "object_id": LaunchConfiguration("object_id"),
                    },
                ],
                output="screen",
            ),
        ]
    )
