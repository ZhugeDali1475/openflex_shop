#!/usr/bin/env python3

"""Run the complete right-arm product grasp pipeline beside an existing MoveIt stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    grasp_share = get_package_share_directory("openflex_grasp")
    grasp_moveit_share = get_package_share_directory("openflex_grasp_moveit")
    execute = LaunchConfiguration("execute")
    visualize = LaunchConfiguration("visualize")
    unconstrained_moveit = LaunchConfiguration("unconstrained_moveit")

    cartesian_executor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(grasp_moveit_share, "launch", "cartesian_approach_executor.launch.py")
        )
    )
    product_grasp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                grasp_share,
                "launch",
                "right_wrist_product_pointcloud.launch.py",
            )
        ),
        launch_arguments={
            "start_camera": "true",
            "serial_no": LaunchConfiguration("camera_serial"),
            "target": "",
            "grasp_product": LaunchConfiguration("product"),
            "grasp_backend": "graspnet",
            "grasp_mode": LaunchConfiguration("grasp_mode"),
            "start_moveit_bridge": "false",
            "start_grasp_executor": "true",
            "start_web_visualizer": visualize,
            "web_port": LaunchConfiguration("web_port"),
            "web_open_browser": LaunchConfiguration("web_open_browser"),
            "grasp_enabled": "true",
            "grasp_execute": execute,
            "grasp_return_to_photo": "true",
            "grasp_postgrasp_lift_m": LaunchConfiguration("postgrasp_lift_m"),
            "grasp_pregrasp_max_dof": "auto",
            "grasp_unconstrained_moveit": unconstrained_moveit,
            "grasp_prefer_straight_feed": "true",
            "grasp_closing_axis_frame": "base_link",
            "grasp_pregrasp_distance_m": LaunchConfiguration("pregrasp_distance_m"),
            "grasp_depth_offset_m": LaunchConfiguration("grasp_depth_offset_m"),
            "grasp_basket_final_descent_reduction_m": LaunchConfiguration(
                "basket_final_descent_reduction_m"
            ),
            "grasp_candidate_center_tolerance_m": "0.08",
            "grasp_candidate_approach_max_angle_deg": "35.0",
            "grasp_straight_line_tolerance_m": "0.005",
            "grasp_cartesian_velocity_scale": LaunchConfiguration(
                "trajectory_velocity_scale"
            ),
            "grasp_cartesian_acceleration_scale": LaunchConfiguration(
                "trajectory_acceleration_scale"
            ),
            "grasp_cartesian_start_position_tolerance_m": "0.04",
            "grasp_cartesian_start_orientation_tolerance_deg": "20.0",
            "grasp_corridor_radius_m": LaunchConfiguration("corridor_radius"),
            "grasp_corridor_planning_timeout_s": LaunchConfiguration(
                "corridor_planning_timeout"
            ),
            "grasp_width_margin_m": "0.004",
            "grasp_gripper_max_opening_m": "0.086",
            "grasp_gripper_joint_max_position_m": "0.043",
            "grasp_gripper_open_stall_tolerance_m": "0.002",
            "grasp_gripper_opening_scale": "2.0",
            "grasp_gripper_grip_compression_m": "0.003",
            "grasp_gripper_close_position_m": "0.0",
            "grasp_payload_compensation_enabled": "false",
            "grasp_payload_mass_kg": LaunchConfiguration("payload_mass_kg"),
            "grasp_payload_com_z_m": LaunchConfiguration("payload_com_z_m"),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "product",
            description="Natural-language product name, for example 'water bottle'.",
        ),
        DeclareLaunchArgument(
            "camera_serial",
            default_value=os.environ.get("OPENFLEX_RIGHT_CAMERA_SERIAL", ""),
            description="Right wrist RealSense serial; override this for each robot.",
        ),
        DeclareLaunchArgument(
            "execute",
            default_value="true",
            description="Execute the right-arm grasp sequence.",
        ),
        DeclareLaunchArgument(
            "grasp_mode",
            default_value="shelf",
            description="Use shelf front approach or basket top-down approach.",
        ),
        DeclareLaunchArgument(
            "unconstrained_moveit",
            default_value="true",
            description="Use standard right-arm MoveIt pose planning without custom path constraints.",
        ),
        DeclareLaunchArgument(
            "grasp_depth_offset_m",
            default_value="0.015",
            description="Push the final grasp point into the product along the configured approach.",
        ),
        DeclareLaunchArgument(
            "pregrasp_distance_m",
            default_value="0.08",
            description="Distance from the segmented product surface to the pregrasp pose.",
        ),
        DeclareLaunchArgument(
            "basket_final_descent_reduction_m",
            default_value="0.0",
            description="Reduce only the basket pregrasp-to-grasp vertical descent; zero uses the full descent.",
        ),
        DeclareLaunchArgument(
            "postgrasp_lift_m",
            default_value="0.01",
            description="Vertical lift before returning straight to the photo pose.",
        ),
        DeclareLaunchArgument(
            "trajectory_velocity_scale",
            default_value="0.40",
            description="MoveIt velocity scaling against the 1.0 rad/s arm and 0.1 m/s lift limits.",
        ),
        DeclareLaunchArgument(
            "trajectory_acceleration_scale",
            default_value="0.20",
            description="MoveIt acceleration scaling for CSP trajectory tracking.",
        ),
        DeclareLaunchArgument(
            "payload_mass_kg",
            default_value="0.55",
            description="Expected held product mass used for right-arm gravity compensation.",
        ),
        DeclareLaunchArgument(
            "payload_com_z_m",
            default_value="0.05",
            description="Held product COM offset along the right hand TCP Z axis.",
        ),
        DeclareLaunchArgument(
            "visualize",
            default_value="true",
            description="Start the local segmentation and grasp inspection page.",
        ),
        DeclareLaunchArgument("web_port", default_value="8765"),
        DeclareLaunchArgument("web_open_browser", default_value="true"),
        DeclareLaunchArgument(
            "corridor_radius",
            default_value="0.06",
            description="Maximum TCP deviation from the direct photo/pregrasp line in metres.",
        ),
        DeclareLaunchArgument("corridor_planning_timeout", default_value="8.0"),
        cartesian_executor,
        product_grasp,
    ])
