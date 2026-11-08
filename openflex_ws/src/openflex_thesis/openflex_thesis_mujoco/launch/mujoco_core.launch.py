import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("openflex_thesis_mujoco")
    description_share = get_package_share_directory("openarmx_integrated_description")
    xacro_file = os.path.join(
        description_share, "urdf", "openarmx_integrated_robot.urdf.xacro"
    )
    robot_description = ParameterValue(
        Command(["xacro ", xacro_file, " use_fake_hardware:=false enable_head:=true"]),
        value_type=str,
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("upstream_model_path", default_value=""),
            DeclareLaunchArgument("experiment_config", default_value=os.path.join(share, "config", "mujoco.yaml")),
            DeclareLaunchArgument("enable_viewer", default_value="false"),
            DeclareLaunchArgument("publish_rgbd", default_value="true"),
            Node(
                package="openflex_thesis_mujoco",
                executable="mujoco_bridge",
                name="openflex_thesis_mujoco_bridge",
                output="screen",
                parameters=[
                    os.path.join(share, "config", "mujoco.yaml"),
                    {
                        "upstream_model_path": LaunchConfiguration("upstream_model_path"),
                        "enable_viewer": ParameterValue(
                            LaunchConfiguration("enable_viewer"), value_type=bool
                        ),
                        "publish_rgbd": ParameterValue(
                            LaunchConfiguration("publish_rgbd"), value_type=bool
                        ),
                        "use_sim_time": False,
                    },
                    LaunchConfiguration("experiment_config"),
                ],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="thesis_mujoco_robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description, "use_sim_time": True}],
            ),
        ]
    )
