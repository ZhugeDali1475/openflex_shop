#!/usr/bin/env python3

"""Generate a collision-free FK target dataset for a selectable robot group."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    description_share = get_package_share_directory("openarmx_integrated_description")
    urdf_path = os.path.join(description_share, "urdf", "openarmx_integrated_robot.urdf.xacro")
    moveit_params = (
        MoveItConfigsBuilder("openarmx_integrated", package_name="openarmx_integrated_moveit_config")
        .robot_description(
            file_path=urdf_path,
            mappings={"use_fake_hardware": "false", "enable_head": "true"},
        )
        .to_moveit_configs()
        .to_dict()
    )
    generator = Node(
        package="openarmx_integrated_moveit_config",
        executable="generate_pose_dataset",
        output="screen",
        parameters=[
            moveit_params,
            {
                "output_path": LaunchConfiguration("output_path"),
                "group_name": LaunchConfiguration("group_name"),
                "tip_link": LaunchConfiguration("tip_link"),
                "frame_id": LaunchConfiguration("frame_id"),
                "sampling_joint_names": LaunchConfiguration("sampling_joint_names"),
                "joint_step_rad": ParameterValue(LaunchConfiguration("joint_step_rad"), value_type=float),
                "sampling_joint_step_overrides": LaunchConfiguration("sampling_joint_step_overrides"),
                "sampling_joint_extra_values": LaunchConfiguration("sampling_joint_extra_values"),
                "sampling_mode": LaunchConfiguration("sampling_mode"),
                "target_count": ParameterValue(LaunchConfiguration("target_count"), value_type=int),
                "random_seed": ParameterValue(LaunchConfiguration("random_seed"), value_type=int),
                "max_random_attempts": ParameterValue(
                    LaunchConfiguration("max_random_attempts"), value_type=int
                ),
                "sampling_joint_position_ranges": LaunchConfiguration("sampling_joint_position_ranges"),
                "fixed_joint_positions": LaunchConfiguration("fixed_joint_positions"),
                "joint_limit_margin": ParameterValue(LaunchConfiguration("joint_limit_margin"), value_type=float),
                "max_combinations": ParameterValue(LaunchConfiguration("max_combinations"), value_type=int),
                "report_first_collision": ParameterValue(LaunchConfiguration("report_first_collision"), value_type=bool),
            },
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("output_path", default_value="/tmp/openarmx_left_arm_poses.jsonl"),
            DeclareLaunchArgument("group_name", default_value="left_arm"),
            DeclareLaunchArgument("tip_link", default_value="openarmx_left_hand"),
            DeclareLaunchArgument("frame_id", default_value="odom"),
            DeclareLaunchArgument("sampling_joint_names", default_value=""),
            DeclareLaunchArgument("joint_step_rad", default_value="1.0471975512"),
            DeclareLaunchArgument("sampling_joint_step_overrides", default_value=""),
            DeclareLaunchArgument("sampling_joint_extra_values", default_value=""),
            DeclareLaunchArgument("sampling_mode", default_value="grid"),
            DeclareLaunchArgument("target_count", default_value="0"),
            DeclareLaunchArgument("random_seed", default_value="20260720"),
            DeclareLaunchArgument("max_random_attempts", default_value="200000"),
            DeclareLaunchArgument("sampling_joint_position_ranges", default_value=""),
            DeclareLaunchArgument("fixed_joint_positions", default_value=""),
            DeclareLaunchArgument("joint_limit_margin", default_value="0.02"),
            DeclareLaunchArgument("max_combinations", default_value="200000"),
            DeclareLaunchArgument("report_first_collision", default_value="false"),
            generator,
        ]
    )
