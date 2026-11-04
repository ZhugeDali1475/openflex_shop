#!/usr/bin/env python3

"""Bring up the integrated robot and its MoveIt online-planning frontend."""

import os
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

sys.path.insert(0, str(Path(__file__).parent))
from surface_collision_profile import build_surface_collision_robot_description


def generate_launch_description():
    bringup_share = get_package_share_directory("openarmx_integrated_bringup")
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
    # Match the desktop workspace execution policy: MoveIt delegates endpoint
    # convergence to the command bridge and does not reject the next segment
    # on a duplicate start-state or expected-duration gate.
    moveit_params["trajectory_execution"] = {
        "allowed_execution_duration_scaling": 1.2,
        "allowed_goal_duration_margin": 8.0,
        "allowed_start_tolerance": 0.0,
        "execution_duration_monitoring": False,
    }
    moveit_params["robot_description"] = build_surface_collision_robot_description(
        urdf_path, {"use_fake_hardware": "false", "enable_head": "true"}
    )

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "integrated_robot_bringup.launch.py")
        ),
        launch_arguments={
            "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
            "use_rviz": "false",
            "control_mode": LaunchConfiguration("control_mode"),
            "gripper_control_mode": LaunchConfiguration("gripper_control_mode"),
            # This launch owns controller loading through controller_recovery.
            # Running the timed spawners as well makes controller_manager
            # transitions nondeterministic on slower real-hardware startup.
            "start_controller_spawners": "false",
        }.items(),
    )
    readiness = Node(
        package="openarmx_integrated_moveit_config",
        executable="wait_for_robot_state.py",
        name="wait_for_robot_state",
        output="both",
    )
    controller_recovery = Node(
        package="openarmx_integrated_moveit_config",
        executable="ensure_controllers_ready.py",
        name="ensure_position_controllers_ready",
        arguments=[
            "--active", "joint_state_broadcaster",
            "--active", "swerve_drive_controller",
            "--active", "left_forward_position_controller",
            "--active", "right_forward_position_controller",
            "--active", "head_forward_position_controller",
            "--active", "left_forward_effort_controller",
            "--active", "right_forward_effort_controller",
            "--active", "lift_state_controller",
            "--configured", "lift_position_controller",
            "--configured", "lift_manual_position_controller",
            "--configured", "velocity_controller",
            "--timeout", "90.0",
            "--repair-delay", "0.0",
        ],
        output="both",
    )
    activate_moveit_lift = Node(
        package="openarmx_integrated_moveit_config",
        executable="switch_lift_controller.py",
        name="switch_lift_controller_for_moveit",
        arguments=["--mode", "moveit"],
        output="both",
    )
    bridge = Node(
        package="openarmx_integrated_moveit_config",
        executable="moveit_command_bridge.py",
        name="moveit_command_bridge",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_execution_bridge")),
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
        parameters=[moveit_params],
    )
    move_group_ready = Node(
        package="openarmx_integrated_moveit_config",
        executable="wait_for_moveit.py",
        name="wait_for_moveit",
        arguments=["--timeout", "60.0"],
        output="both",
    )
    pick_task_executor = Node(
        package="openarmx_integrated_moveit_config",
        executable="openflex_pick_task_executor",
        name="openflex_pick_task_executor",
        output="screen",
        parameters=[
            moveit_params,
        ],
        condition=IfCondition(LaunchConfiguration("start_pick_task_executor")),
    )
    joint_posture_executor = Node(
        package="openarmx_integrated_moveit_config",
        executable="openflex_joint_posture_executor",
        name="openflex_joint_posture_executor",
        output="screen",
        parameters=[moveit_params],
        condition=IfCondition(LaunchConfiguration("start_joint_posture_executor")),
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", os.path.join(moveit_share, "config", "integrated_robot_moveit.rviz")],
        parameters=[moveit_params],
        condition=IfCondition(LaunchConfiguration("_moveit_rviz")),
    )

    def start_readiness_after_controller_recovery(event, _context):
        if event.returncode != 0:
            return [
                LogInfo(
                    msg=(
                        "Controller recovery failed (return code "
                        f"{event.returncode}); MoveIt frontend will not start"
                    )
                )
            ]
        return [readiness]

    start_readiness = RegisterEventHandler(
        OnProcessExit(
            target_action=controller_recovery,
            on_exit=start_readiness_after_controller_recovery,
        )
    )
    switch_lift_for_moveit = RegisterEventHandler(
        OnProcessExit(target_action=readiness, on_exit=[activate_moveit_lift])
    )
    def start_frontend_after_lift_switch(event, _context):
        if event.returncode != 0:
            return [
                LogInfo(
                    msg="Lift controller handoff failed; MoveIt frontend will not start"
                )
            ]
        return [
            base_navigation_bridge,
            bridge,
            move_group,
            move_group_ready,
            rviz,
        ]

    def start_clients_after_move_group(event, _context):
        if event.returncode != 0:
            return [
                LogInfo(
                    msg=(
                        "MoveIt planning scene was not ready (return code "
                        f"{event.returncode}); manipulation clients will not start"
                    )
                )
            ]
        # MoveGroupInterface construction is synchronous. Stagger the two
        # clients after MoveIt is ready so they do not compete for the same
        # planning-scene initialization window.
        return [
            TimerAction(period=1.0, actions=[pick_task_executor]),
            TimerAction(period=4.0, actions=[joint_posture_executor]),
        ]

    start_clients = RegisterEventHandler(
        OnProcessExit(
            target_action=move_group_ready,
            on_exit=start_clients_after_move_group,
        )
    )

    start_frontend = RegisterEventHandler(
        OnProcessExit(
            target_action=activate_moveit_lift,
            on_exit=start_frontend_after_lift_switch,
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_fake_hardware",
                default_value="false",
                description="Use fake hardware instead of CAN-connected hardware.",
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "control_mode", default_value="mit", choices=["mit", "csp"]
            ),
            DeclareLaunchArgument(
                "gripper_control_mode",
                default_value=LaunchConfiguration("control_mode"),
                choices=["mit", "csp"],
            ),
            DeclareLaunchArgument("start_execution_bridge", default_value="true"),
            DeclareLaunchArgument(
                "start_base_navigation_bridge",
                default_value="true",
                description="Serve /navigate_to_pose from odometry and /cmd_vel.",
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
            # Preserve the user-facing RViz choice before the included bringup
            # receives use_rviz:=false to suppress its legacy layout.
            SetLaunchConfiguration("_moveit_rviz", LaunchConfiguration("use_rviz")),
            # Register the event chain before starting either process.  This
            # matters for fake hardware, where recovery can finish immediately.
            start_readiness,
            switch_lift_for_moveit,
            start_frontend,
            start_clients,
            robot,
            controller_recovery,
        ]
    )
