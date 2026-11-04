from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rviz_default = (
        get_package_share_directory("openarmx_battery_monitor")
        + "/config/single_battery_overlay.rviz"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=rviz_default),
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument(
                "serial_port_candidates",
                default_value="/dev/ttyUSB0,/dev/ttyUSB1,/dev/ttyUSB2,/dev/ttyUSB3",
            ),
            DeclareLaunchArgument("baudrate", default_value="19200"),
            DeclareLaunchArgument("slave_id", default_value="3"),
            DeclareLaunchArgument("poll_hz", default_value="5.0"),
            DeclareLaunchArgument("timeout_sec", default_value="0.2"),
            DeclareLaunchArgument("read_retries", default_value="0"),
            DeclareLaunchArgument("offline_after_failures", default_value="1"),
            DeclareLaunchArgument("battery_state_topic", default_value="/battery_state"),
            Node(
                package="openarmx_battery_monitor",
                executable="jd_battery_node",
                output="screen",
                parameters=[
                    {
                        "serial_port": LaunchConfiguration("serial_port"),
                        "serial_port_candidates": LaunchConfiguration("serial_port_candidates"),
                        "baudrate": LaunchConfiguration("baudrate"),
                        "slave_id": LaunchConfiguration("slave_id"),
                        "poll_hz": LaunchConfiguration("poll_hz"),
                        "timeout_sec": LaunchConfiguration("timeout_sec"),
                        "read_retries": LaunchConfiguration("read_retries"),
                        "offline_after_failures": LaunchConfiguration("offline_after_failures"),
                        "battery_state_topic": LaunchConfiguration("battery_state_topic"),
                    }
                ],
            ),
            Node(
                condition=IfCondition(LaunchConfiguration("start_rviz")),
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", LaunchConfiguration("rviz_config")],
            ),
        ]
    )
