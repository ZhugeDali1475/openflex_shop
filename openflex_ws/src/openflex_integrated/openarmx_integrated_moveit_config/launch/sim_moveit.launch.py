#!/usr/bin/env python3

"""Attach MoveIt to an already running MuJoCo full-body simulation."""

import os
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder

sys.path.insert(0, str(Path(__file__).parent))
from surface_collision_profile import build_surface_collision_robot_description


def generate_launch_description():
    moveit_share = get_package_share_directory("openarmx_integrated_moveit_config")
    description_share = get_package_share_directory("openarmx_integrated_description")
    urdf_path = os.path.join(description_share, "urdf", "openarmx_integrated_robot.urdf.xacro")
    moveit_params = (
        MoveItConfigsBuilder(
            "openarmx_integrated", package_name="openarmx_integrated_moveit_config"
        )
        .robot_description(
            file_path=urdf_path,
            mappings={"use_fake_hardware": "false", "enable_head": "true"},
        )
        .to_moveit_configs()
        .to_dict()
    )
    moveit_params["robot_description"] = build_surface_collision_robot_description(
        urdf_path, {"use_fake_hardware": "false", "enable_head": "true"}
    )

    readiness = Node(
        package="openarmx_integrated_moveit_config",
        executable="wait_for_robot_state.py",
        name="wait_for_simulation_state",
        output="both",
        parameters=[
            {
                "use_sim_time": True,
                "require_controllers": ParameterValue(
                    LaunchConfiguration("require_controllers"), value_type=bool
                ),
            }
        ],
    )
    bridge = Node(
        package="openarmx_integrated_moveit_config",
        executable="moveit_command_bridge.py",
        name="moveit_command_bridge",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_command_bridge")),
    )
    base_navigation_bridge = Node(
        package="openarmx_integrated_moveit_config",
        executable="base_navigation_bridge.py",
        name="base_navigation_bridge",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_base_navigation_bridge")),
    )
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_params, {"use_sim_time": True}],
    )
    pick_task_executor = Node(
        package="openarmx_integrated_moveit_config",
        executable="openflex_pick_task_executor",
        name="openflex_pick_task_executor",
        output="screen",
        parameters=[
            moveit_params,
            {"use_sim_time": True},
        ],
        condition=IfCondition(LaunchConfiguration("start_pick_task_executor")),
    )
    joint_posture_executor = Node(
        package="openarmx_integrated_moveit_config",
        executable="openflex_joint_posture_executor",
        name="openflex_joint_posture_executor",
        output="screen",
        parameters=[moveit_params, {"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("start_joint_posture_executor")),
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", os.path.join(moveit_share, "config", "integrated_robot_moveit.rviz")],
        parameters=[moveit_params, {"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )
    start_moveit = RegisterEventHandler(
        OnProcessExit(
            target_action=readiness,
            on_exit=[
                base_navigation_bridge,
                bridge,
                move_group,
                TimerAction(
                    period=2.0,
                    actions=[pick_task_executor, joint_posture_executor],
                ),
                rviz,
            ],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "require_controllers",
                default_value="true",
                description=(
                    "Require ros2_control controllers. Disable for bridges that "
                    "provide trajectory Actions directly."
                ),
            ),
            DeclareLaunchArgument(
                "start_base_navigation_bridge",
                default_value="true",
                description="Serve /navigate_to_pose from odometry and /cmd_vel.",
            ),
            DeclareLaunchArgument(
                "start_command_bridge",
                default_value="true",
                description=(
                    "Start the real-hardware command bridge. Disable it when the "
                    "simulator already owns the controller Action names."
                ),
            ),
            DeclareLaunchArgument(
                "start_pick_task_executor",
                default_value="true",
                description="Serve /openflex/plan_pregrasp task goals.",
            ),
            DeclareLaunchArgument(
                "start_joint_posture_executor",
                default_value="true",
                description="Serve /openflex/execute_joint_posture goals.",
            ),
            LogInfo(
                msg="Waiting for MuJoCo simulation: /joint_states and TF odom -> base_link"
            ),
            readiness,
            start_moveit,
        ]
    )
