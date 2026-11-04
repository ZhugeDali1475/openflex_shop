#!/usr/bin/env python3

"""Start only the Cartesian approach Action server alongside an existing MoveIt stack."""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

moveit_share = get_package_share_directory("openarmx_integrated_moveit_config")
sys.path.insert(0, os.path.join(moveit_share, "launch"))
from surface_collision_profile import build_surface_collision_robot_description


def generate_launch_description():
    description_share = get_package_share_directory("openarmx_integrated_description")
    urdf_path = os.path.join(description_share, "urdf", "openarmx_integrated_robot.urdf.xacro")
    use_sim_time = LaunchConfiguration("use_sim_time")
    moveit_params = (
        MoveItConfigsBuilder("openarmx_integrated", package_name="openarmx_integrated_moveit_config")
        .robot_description(
            file_path=urdf_path,
            mappings={"use_fake_hardware": "false", "enable_head": "true"},
        )
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
        .to_dict()
    )
    moveit_params["robot_description"] = build_surface_collision_robot_description(
        urdf_path, {"use_fake_hardware": "false", "enable_head": "true"}
    )
    moveit_params["use_sim_time"] = use_sim_time

    executor = Node(
        package="openflex_grasp_moveit",
        executable="cartesian_approach_executor",
        name="cartesian_approach_executor",
        output="screen",
        parameters=[moveit_params],
    )
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        executor,
    ])
