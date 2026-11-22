import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
            DeclareLaunchArgument("scene_model_path", default_value=""),
            DeclareLaunchArgument("experiment_config", default_value=os.path.join(share, "config", "mujoco.yaml")),
            DeclareLaunchArgument("enable_viewer", default_value="false"),
            DeclareLaunchArgument("publish_rgbd", default_value="true"),
            DeclareLaunchArgument("publish_lidar", default_value="false"),
            DeclareLaunchArgument("lidar_rate_hz", default_value="10.0"),
            DeclareLaunchArgument("lidar_azimuth_bins", default_value="240"),
            DeclareLaunchArgument("lidar_channels", default_value="8"),
            DeclareLaunchArgument("publish_ground_truth_tf", default_value="true"),
            DeclareLaunchArgument("start_lio_odom_tf_bridge", default_value="false"),
            Node(
                package="openflex_thesis_mujoco",
                executable="mujoco_bridge",
                name="openflex_thesis_mujoco_bridge",
                output="screen",
                parameters=[
                    os.path.join(share, "config", "mujoco.yaml"),
                    LaunchConfiguration("experiment_config"),
                    {
                        "upstream_model_path": LaunchConfiguration("upstream_model_path"),
                        "scene_model_path": LaunchConfiguration("scene_model_path"),
                        "enable_viewer": ParameterValue(
                            LaunchConfiguration("enable_viewer"), value_type=bool
                        ),
                        "publish_rgbd": ParameterValue(
                            LaunchConfiguration("publish_rgbd"), value_type=bool
                        ),
                        "publish_lidar": ParameterValue(
                            LaunchConfiguration("publish_lidar"), value_type=bool
                        ),
                        "lidar_rate_hz": ParameterValue(
                            LaunchConfiguration("lidar_rate_hz"), value_type=float
                        ),
                        "lidar_azimuth_bins": ParameterValue(
                            LaunchConfiguration("lidar_azimuth_bins"), value_type=int
                        ),
                        "lidar_channels": ParameterValue(
                            LaunchConfiguration("lidar_channels"), value_type=int
                        ),
                        "publish_ground_truth_tf": ParameterValue(
                            LaunchConfiguration("publish_ground_truth_tf"), value_type=bool
                        ),
                        "use_sim_time": False,
                    },
                ],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="thesis_mujoco_robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description, "use_sim_time": True}],
            ),
            Node(
                package="openflex_thesis_mujoco",
                executable="lio_odom_tf_bridge",
                name="lio_odom_tf_bridge",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_lio_odom_tf_bridge")),
                parameters=[{"use_sim_time": True}],
            ),
        ]
    )
