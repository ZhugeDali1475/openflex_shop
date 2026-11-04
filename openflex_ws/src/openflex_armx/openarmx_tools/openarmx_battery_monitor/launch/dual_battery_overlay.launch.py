from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rviz_default = (
        get_package_share_directory("openarmx_battery_monitor")
        + "/config/dual_battery_overlay.rviz"
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
            DeclareLaunchArgument("poll_hz", default_value="5.0"),
            DeclareLaunchArgument("timeout_sec", default_value="0.2"),
            DeclareLaunchArgument("flush_before_write", default_value="false"),
            DeclareLaunchArgument("read_retries", default_value="0"),
            DeclareLaunchArgument("offline_after_failures", default_value="3"),
            DeclareLaunchArgument("online_after_successes", default_value="1"),
            DeclareLaunchArgument("stale_timeout_sec", default_value="0.8"),
            DeclareLaunchArgument("inter_request_gap_sec", default_value="0.004"),
            DeclareLaunchArgument("reset_port_after_failures", default_value="4"),
            DeclareLaunchArgument("start_register", default_value="114"),
            DeclareLaunchArgument("register_count", default_value="5"),
            DeclareLaunchArgument("battery_frame_id", default_value="battery_link"),
            DeclareLaunchArgument(
                "battery_ids_csv", default_value="battery_1,battery_2"
            ),
            DeclareLaunchArgument(
                "battery_topics_csv",
                default_value="/battery_1/battery_state,/battery_2/battery_state",
            ),
            DeclareLaunchArgument(
                "slave_ids_csv",
                default_value="",
                description="Comma-separated slave IDs. Leave empty to auto discover.",
            ),
            DeclareLaunchArgument("auto_discover_slave_ids", default_value="true"),
            DeclareLaunchArgument("scan_min_id", default_value="1"),
            DeclareLaunchArgument("scan_max_id", default_value="16"),
            DeclareLaunchArgument("discover_interval_sec", default_value="2.0"),
            Node(
                package="openarmx_battery_monitor",
                executable="jd_battery_multi_node",
                output="screen",
                parameters=[
                    {
                        "serial_port": LaunchConfiguration("serial_port"),
                        "serial_port_candidates": LaunchConfiguration(
                            "serial_port_candidates"
                        ),
                        "baudrate": LaunchConfiguration("baudrate"),
                        "poll_hz": LaunchConfiguration("poll_hz"),
                        "timeout_sec": LaunchConfiguration("timeout_sec"),
                        "flush_before_write": LaunchConfiguration("flush_before_write"),
                        "read_retries": LaunchConfiguration("read_retries"),
                        "offline_after_failures": LaunchConfiguration(
                            "offline_after_failures"
                        ),
                        "online_after_successes": LaunchConfiguration(
                            "online_after_successes"
                        ),
                        "stale_timeout_sec": LaunchConfiguration("stale_timeout_sec"),
                        "inter_request_gap_sec": LaunchConfiguration(
                            "inter_request_gap_sec"
                        ),
                        "reset_port_after_failures": LaunchConfiguration(
                            "reset_port_after_failures"
                        ),
                        "start_register": LaunchConfiguration("start_register"),
                        "register_count": LaunchConfiguration("register_count"),
                        "battery_frame_id": LaunchConfiguration("battery_frame_id"),
                        "battery_ids_csv": LaunchConfiguration("battery_ids_csv"),
                        "battery_topics_csv": LaunchConfiguration("battery_topics_csv"),
                        "slave_ids_csv": LaunchConfiguration("slave_ids_csv"),
                        "auto_discover_slave_ids": LaunchConfiguration(
                            "auto_discover_slave_ids"
                        ),
                        "scan_min_id": LaunchConfiguration("scan_min_id"),
                        "scan_max_id": LaunchConfiguration("scan_max_id"),
                        "discover_interval_sec": LaunchConfiguration(
                            "discover_interval_sec"
                        ),
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
