"""Run the complete three-phase competition against truth-mode capabilities."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim_share = Path(get_package_share_directory("openarmx_supermarket_sim"))
    executor_share = Path(get_package_share_directory("openarmx_task_executor"))
    station_config = executor_share / "config" / "stations.yaml"
    competition_config = executor_share / "config" / "competition.yaml"

    simulation = Node(
        package="openarmx_supermarket_sim",
        executable="competition_sim",
        name="openarmx_supermarket_sim",
        output="screen",
        parameters=[
            str(sim_share / "config" / "simulation.yaml"),
            {
                "world_config": str(sim_share / "config" / "world.yaml"),
                "station_config": str(station_config),
                "random_seed": ParameterValue(
                    LaunchConfiguration("random_seed"), value_type=int
                ),
                "auto_run": ParameterValue(
                    LaunchConfiguration("auto_run"), value_type=bool
                ),
                "exit_on_complete": ParameterValue(
                    LaunchConfiguration("exit_on_complete"), value_type=bool
                ),
                "time_scale": ParameterValue(
                    LaunchConfiguration("time_scale"), value_type=float
                ),
            },
        ],
    )
    transport_executor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(executor_share / "launch" / "task_executor.launch.py")
        ),
        launch_arguments={
            "station_config": str(station_config),
            "enable_docking": "true",
            "enable_detection": "true",
            "allow_nominal_detection_fallback": "false",
            "odom_topic": "/odom_safe",
        }.items(),
    )
    coordinator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(executor_share / "launch" / "competition_coordinator.launch.py")
        ),
        launch_arguments={
            "competition_config": str(competition_config),
            "station_config": str(station_config),
            "odom_topic": "/odom_safe",
        }.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("random_seed", default_value="20260729"),
            DeclareLaunchArgument("auto_run", default_value="false"),
            DeclareLaunchArgument("exit_on_complete", default_value="true"),
            DeclareLaunchArgument("time_scale", default_value="0.02"),
            simulation,
            transport_executor,
            coordinator,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=simulation,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(reason="competition simulation exited")
                        )
                    ],
                )
            ),
        ]
    )
