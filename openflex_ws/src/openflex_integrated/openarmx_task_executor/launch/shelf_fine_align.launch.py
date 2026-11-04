"""Shelf fine alignment server; commands remain behind the safety chain."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            Node(
                package="openarmx_task_executor",
                executable="shelf_fine_align",
                name="shelf_fine_align",
                output="screen",
                parameters=[
                    {
                        "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                        "localization_guard_state_topic": (
                            "/shelf_fine_align/localization_guard_state"
                        ),
                        "half_length": 0.46,
                        "half_width": 0.45,
                        "maximum_linear_speed": 0.040,
                        "minimum_linear_speed": 0.030,
                        "maximum_angular_speed": 0.10,
                        "minimum_angular_speed": 0.070,
                        "minimum_safe_clearance": 0.035,
                        "plane_fit_station_ids": ["jiaofu2"],
                        "plane_minimum_points": 5,
                        "plane_minimum_lateral_span": 0.30,
                        "plane_minimum_points_per_side": 2,
                        # The delivery plane still requires five points spanning
                        # both sides, followed by the 5-of-7 temporal filter.
                        "plane_inlier_tolerance": 0.040,
                        "plane_maximum_normal_yaw": 0.2094395102,
                        "plane_filter_quantile": 0.50,
                        "plane_acquisition_timeout": 3.00,
                        "plane_measurement_timeout": 1.50,
                        "acquisition_timeout": 1.50,
                        "measurement_timeout": 0.80,
                    }
                ],
            ),
        ]
    )
