from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("openarmx_task_executor")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "competition_config",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "competition.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "station_config",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "stations.yaml"]
                ),
            ),
            DeclareLaunchArgument("station_mission", default_value=""),
            DeclareLaunchArgument("odom_topic", default_value="/odom_safe"),
            Node(
                package="openarmx_task_executor",
                executable="competition_coordinator",
                name="openarmx_competition_coordinator",
                output="screen",
                parameters=[
                    {
                        "competition_config": LaunchConfiguration(
                            "competition_config"
                        ),
                        "station_config": LaunchConfiguration("station_config"),
                        "station_mission": LaunchConfiguration("station_mission"),
                        "odom_topic": LaunchConfiguration("odom_topic"),
                    }
                ],
            ),
        ]
    )
