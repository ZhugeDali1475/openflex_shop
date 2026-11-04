"""Start the complete supermarket MuJoCo physics and RGB-D bridge."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("openarmx_supermarket_sim"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("enable_viewer", default_value="false"),
            DeclareLaunchArgument("publish_rgbd", default_value="true"),
            DeclareLaunchArgument("mujoco_gl", default_value="egl"),
            SetEnvironmentVariable("MUJOCO_GL", LaunchConfiguration("mujoco_gl")),
            Node(
                package="openarmx_supermarket_sim",
                executable="competition_physics",
                name="openarmx_supermarket_physics",
                output="screen",
                parameters=[
                    str(share / "config" / "physics.yaml"),
                    {
                        "enable_viewer": ParameterValue(
                            LaunchConfiguration("enable_viewer"), value_type=bool
                        ),
                        "publish_rgbd": ParameterValue(
                            LaunchConfiguration("publish_rgbd"), value_type=bool
                        ),
                    },
                ],
            ),
        ]
    )
