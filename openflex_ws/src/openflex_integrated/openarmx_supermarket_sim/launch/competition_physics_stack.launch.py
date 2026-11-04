"""Bring up the registered MuJoCo field, Nav2, MoveIt and optional DREAM."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = Path(get_package_share_directory("openarmx_supermarket_sim"))
    moveit_share = Path(get_package_share_directory("openarmx_integrated_moveit_config"))
    executor_share = Path(get_package_share_directory("openarmx_task_executor"))

    physics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(sim_share / "launch" / "mujoco_physics.launch.py")),
        launch_arguments={
            "enable_viewer": LaunchConfiguration("enable_viewer"),
            "publish_rgbd": "true",
            "mujoco_gl": LaunchConfiguration("mujoco_gl"),
        }.items(),
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(sim_share / "launch" / "supermarket_nav2.launch.py")),
        condition=IfCondition(LaunchConfiguration("start_nav2")),
    )
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(moveit_share / "launch" / "sim_moveit.launch.py")),
        condition=IfCondition(LaunchConfiguration("start_moveit")),
        launch_arguments={
            "use_rviz": LaunchConfiguration("use_moveit_rviz"),
            # Nav2 is the sole owner of /navigate_to_pose in this stack.
            "start_base_navigation_bridge": "false",
            # MuJoCo owns all trajectory and gripper controller actions.
            "start_command_bridge": "false",
            "start_pick_task_executor": "true",
        }.items(),
    )
    dream_rgbd_sync = Node(
        package="rtabmap_sync",
        executable="rgbd_sync",
        namespace="dream_rtabmap",
        name="rgbd_sync",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_dream")),
        parameters=[
            {
                "use_sim_time": True,
                "approx_sync": True,
                "approx_sync_max_interval": 0.05,
                "topic_queue_size": 20,
                "sync_queue_size": 20,
                "qos": 1,
            }
        ],
        remappings=[
            ("rgb/image", "/camera/color/image_raw"),
            ("rgb/camera_info", "/camera/color/camera_info"),
            ("depth/image", "/camera/aligned_depth_to_color/image_raw"),
        ],
    )
    dream_rtabmap = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        namespace="dream_rtabmap",
        name="rtabmap",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_dream")),
        parameters=[
            {
                "use_sim_time": True,
                "frame_id": "base_link",
                "odom_frame_id": "odom",
                "map_frame_id": "dream_map",
                "publish_tf": False,
                "subscribe_rgbd": True,
                "subscribe_depth": False,
                "subscribe_scan": False,
                "subscribe_scan_cloud": False,
                "rgbd_cameras": 1,
                "queue_size": 20,
                "approx_sync": True,
                "Reg/Strategy": "0",
                "Reg/Force3DoF": "true",
                "RGBD/ForceOdom3DoF": "true",
                "RGBD/CreateOccupancyGrid": "false",
                "RGBD/MaxDepth": "4.0",
                "Grid/Sensor": "1",
            }
        ],
        remappings=[("odom", "/odom_safe")],
        arguments=["-d"],
    )
    dream_root = Path("/home/openflex/openflex_shop/DREAM-realtime")
    dream_python = Path("/home/openarmx/anaconda3/envs/dream/bin/python")
    dream_pythonpath = os.pathsep.join(
        [
            str(dream_root / "src"),
            str(dream_root / "build" / "dream_ros2_bridge"),
            os.environ.get("PYTHONPATH", ""),
        ]
    )
    dream_environment = {
        "PYTHONPATH": dream_pythonpath,
        "LD_LIBRARY_PATH": os.pathsep.join(
            [
                "/home/openarmx/anaconda3/envs/dream/lib",
                os.environ.get("LD_LIBRARY_PATH", ""),
            ]
        ),
    }
    dream_server = ExecuteProcess(
        cmd=[
            str(dream_python),
            "-m",
            "dream_ros2_bridge.remote.server",
            "--rtabmap-namespace",
            "/dream_rtabmap",
            "--observation-only",
            "--ros-args",
            "-p",
            "use_sim_time:=true",
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_dream")),
        additional_env=dream_environment,
    )
    dream_tf_pose = ExecuteProcess(
        cmd=[
            str(dream_python),
            "-m",
            "dream_ros2_bridge.nodes.tf_pose_publisher",
            "--ros-args",
            "-p",
            "use_sim_time:=true",
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_dream")),
        additional_env=dream_environment,
    )
    task_api = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(executor_share / "launch" / "competition_task_api.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("start_task_api")),
        launch_arguments={
            "station_config": str(
                executor_share / "config" / "stations_supermarket_mujoco.yaml"
            ),
            "enable_docking": "false",
            "start_docking_server": "false",
            "start_competition_coordinator": "true",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("enable_viewer", default_value="false"),
            DeclareLaunchArgument("mujoco_gl", default_value="egl"),
            DeclareLaunchArgument("start_nav2", default_value="true"),
            DeclareLaunchArgument("start_moveit", default_value="true"),
            DeclareLaunchArgument("use_moveit_rviz", default_value="true"),
            DeclareLaunchArgument(
                "start_dream",
                default_value="false",
                description=(
                    "Start RGB-D-only RTAB-Map plus the DREAM observation bridge. "
                    "The semantic client still requires the DREAM GPU environment."
                ),
            ),
            DeclareLaunchArgument(
                "start_task_api",
                default_value="false",
                description=(
                    "Start the competition coordinator/executor. Keep false until the "
                    "physical DetectObject/PickObject/PlaceObject adapters are installed."
                ),
            ),
            physics,
            TimerAction(period=2.0, actions=[nav2]),
            TimerAction(period=3.0, actions=[moveit]),
            TimerAction(
                period=5.0,
                actions=[dream_rgbd_sync, dream_rtabmap, dream_tf_pose, dream_server],
            ),
            TimerAction(period=8.0, actions=[task_api]),
        ]
    )
