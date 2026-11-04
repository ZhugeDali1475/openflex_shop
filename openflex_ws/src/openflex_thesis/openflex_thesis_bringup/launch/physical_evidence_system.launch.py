"""Fail-closed entry point for physical thesis evidence collection."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import FindExecutable, LaunchConfiguration


def generate_launch_description():
    system_launch = os.path.join(
        get_package_share_directory("openflex_thesis_bringup"),
        "launch",
        "thesis_system.launch.py",
    )
    scope = LaunchConfiguration("scope")
    through_gate = LaunchConfiguration("through_gate")
    start_move_group = LaunchConfiguration("start_move_group")
    start_object_adapter = LaunchConfiguration("start_object_adapter")
    object_pose_topic = LaunchConfiguration("object_pose_topic")
    object_id = LaunchConfiguration("object_id")

    preflight = ExecuteProcess(
        cmd=[
            FindExecutable(name="ros2"),
            "run",
            "openflex_thesis_experiments",
            "thesis_preflight",
            "--scope",
            scope,
            "--through-gate",
            through_gate,
        ],
        output="screen",
    )

    def after_preflight(event, _context):
        if event.returncode != 0:
            return [
                LogInfo(
                    msg=(
                        "Physical evidence preflight failed. Thesis nodes were not "
                        "started; fix the reported calibration/freeze gates first."
                    )
                ),
                Shutdown(reason="physical thesis evidence preflight failed"),
            ]
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(system_launch),
                launch_arguments={
                    "start_move_group": start_move_group,
                    "start_object_adapter": start_object_adapter,
                    "object_pose_topic": object_pose_topic,
                    "object_id": object_id,
                }.items(),
            )
        ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scope",
                default_value="end_to_end",
                choices=["perception", "tcwam", "act", "end_to_end"],
            ),
            DeclareLaunchArgument("through_gate", default_value="G7_end_to_end"),
            DeclareLaunchArgument("start_move_group", default_value="false"),
            DeclareLaunchArgument("start_object_adapter", default_value="true"),
            DeclareLaunchArgument(
                "object_pose_topic", default_value="/dream/active_object_pose"
            ),
            DeclareLaunchArgument("object_id", default_value="thesis_item_01"),
            preflight,
            RegisterEventHandler(
                OnProcessExit(target_action=preflight, on_exit=after_preflight)
            ),
        ]
    )
