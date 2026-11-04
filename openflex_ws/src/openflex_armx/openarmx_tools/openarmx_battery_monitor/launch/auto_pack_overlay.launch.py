from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rviz_default = (
        get_package_share_directory("openarmx_battery_monitor")
        + "/config/auto_pack_overlay.rviz"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=rviz_default),
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument(
                "serial_port_candidates",
                default_value=(
                    "/dev/ttyACM0,/dev/ttyUSB0,/dev/ttyACM1,/dev/ttyUSB1,"
                    "/dev/ttyACM2,/dev/ttyUSB2,/dev/ttyACM3,/dev/ttyUSB3"
                ),
            ),
            DeclareLaunchArgument("baudrate", default_value="19200"),
            DeclareLaunchArgument("master_slave_id", default_value="3"),
            DeclareLaunchArgument("battery_topic", default_value="/battery_state"),
            DeclareLaunchArgument("poll_hz", default_value="6.0"),
            DeclareLaunchArgument("timeout_sec", default_value="0.25"),
            DeclareLaunchArgument("flush_before_write", default_value="false"),
            DeclareLaunchArgument("read_retries", default_value="1"),
            DeclareLaunchArgument("offline_after_failures", default_value="3"),
            DeclareLaunchArgument("hold_on_read_fail_sec", default_value="0.6"),
            DeclareLaunchArgument("current_neutral_deadband_a", default_value="0.1"),
            DeclareLaunchArgument("battery_location", default_value=""),
            DeclareLaunchArgument("auto_discover_serial_port", default_value="true"),
            DeclareLaunchArgument("port_rescan_interval_sec", default_value="2.0"),
            DeclareLaunchArgument("fix_serial_permission", default_value="true"),
            Node(
                package="openarmx_battery_monitor",
                executable="jd_battery_master_dual_node",
                output="screen",
                parameters=[
                    {
                        "serial_port": LaunchConfiguration("serial_port"),
                        "serial_port_candidates": LaunchConfiguration(
                            "serial_port_candidates"
                        ),
                        "baudrate": LaunchConfiguration("baudrate"),
                        "master_slave_id": LaunchConfiguration("master_slave_id"),
                        "battery_topic": LaunchConfiguration("battery_topic"),
                        "poll_hz": LaunchConfiguration("poll_hz"),
                        "timeout_sec": LaunchConfiguration("timeout_sec"),
                        "flush_before_write": LaunchConfiguration("flush_before_write"),
                        "read_retries": LaunchConfiguration("read_retries"),
                        "offline_after_failures": LaunchConfiguration(
                            "offline_after_failures"
                        ),
                        "hold_on_read_fail_sec": LaunchConfiguration("hold_on_read_fail_sec"),
                        "current_neutral_deadband_a": LaunchConfiguration(
                            "current_neutral_deadband_a"
                        ),
                        "battery_location": LaunchConfiguration("battery_location"),
                        "auto_discover_serial_port": LaunchConfiguration(
                            "auto_discover_serial_port"
                        ),
                        "port_rescan_interval_sec": LaunchConfiguration(
                            "port_rescan_interval_sec"
                        ),
                        "fix_serial_permission": LaunchConfiguration("fix_serial_permission"),
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
