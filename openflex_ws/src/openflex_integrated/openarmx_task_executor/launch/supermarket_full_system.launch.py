"""Single-owner supermarket competition bringup.

The integrated MoveIt bringup owns robot_state_publisher, ros2_control, and all
hardware.  The navigation include owns only sensing, localization, Nav2, and
the velocity safety chain.  The task API composes the teammate-owned action
servers without starting a second robot stack.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    moveit_share = Path(
        get_package_share_directory("openarmx_integrated_moveit_config")
    )
    navigation_share = Path(get_package_share_directory("swerve_navigation"))
    executor_share = Path(get_package_share_directory("openarmx_task_executor"))
    head_vision_share = Path(
        get_package_share_directory("openarmx_head_vision_h264")
    )
    map_root = Path.home() / "openflex_all" / "openflex_maps"

    robot_and_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                moveit_share
                / "launch"
                / "integrated_robot_bringup_moveit.launch.py"
            )
        ),
        launch_arguments={
            "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
            "use_rviz": LaunchConfiguration("use_moveit_rviz"),
            "start_execution_bridge": "true",
            # Nav2 is the only /navigate_to_pose server in competition mode.
            "start_base_navigation_bridge": "false",
            "start_pick_task_executor": "true",
            "start_joint_posture_executor": "true",
            # safe_odom_mux is the only odom -> base_link TF publisher.
            "enable_odom_tf": "false",
            # Hardware receives commands only after collision/localization gates.
            "chassis_cmd_vel_topic": "/cmd_vel_safe",
            "arm_controller": "forward",
            "control_mode": LaunchConfiguration("control_mode"),
            "gripper_control_mode": LaunchConfiguration("gripper_control_mode"),
            "auto_homing": LaunchConfiguration("auto_homing"),
            "right_poe_calibrated": LaunchConfiguration(
                "right_poe_calibrated"
            ),
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                navigation_share
                / "launch"
                / "supermarket_test_05_benchmark.launch.py"
            )
        ),
        launch_arguments={
            "map_yaml": LaunchConfiguration("map_yaml"),
            "pcd_path": LaunchConfiguration("pcd_path"),
            "initial_pose": LaunchConfiguration("initial_pose"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "nav_controller_max_wheel_speed": LaunchConfiguration(
                "nav_controller_max_wheel_speed"
            ),
            "nav_controller_wheel_accel_limit": LaunchConfiguration(
                "nav_controller_wheel_accel_limit"
            ),
            # Full-body bringup above is the sole robot/hardware owner.
            "start_robot_hardware": "false",
            "reverse_authorization_topic": LaunchConfiguration(
                "reverse_authorization_topic"
            ),
        }.items(),
    )

    head_receipt_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(head_vision_share / "launch" / "d435i_source.launch.py")
        ),
        condition=IfCondition(
            LaunchConfiguration("start_head_receipt_camera")
        ),
        launch_arguments={
            "serial_number": LaunchConfiguration("head_camera_serial"),
            "color_width": "1280",
            "color_height": "720",
            "color_fps": "6",
            "enable_depth": "false",
            "enable_imu": "false",
        }.items(),
    )

    task_api = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(executor_share / "launch" / "competition_task_api.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("start_task_api")),
        launch_arguments={
            "station_config": LaunchConfiguration("station_config"),
            "station_mission": LaunchConfiguration("station_mission"),
            "competition_config": LaunchConfiguration("competition_config"),
            "enable_docking": "true",
            "enable_detection": "true",
            "start_docking_server": "true",
            "start_competition_coordinator": "true",
            "start_shopping_ticket_reader": LaunchConfiguration(
                "start_shopping_ticket_reader"
            ),
            "start_lift_level_test_server": LaunchConfiguration(
                "start_lift_level_test_server"
            ),
            "start_shelf_view_server": LaunchConfiguration(
                "start_shelf_view_server"
            ),
            "enable_shelf_view": LaunchConfiguration("enable_shelf_view"),
            "docking_cmd_vel_topic": "/cmd_vel",
            "enable_post_pick_retreat": LaunchConfiguration(
                "enable_post_pick_retreat"
            ),
            "post_pick_retreat_speed": LaunchConfiguration(
                "post_pick_retreat_speed"
            ),
            "post_pick_retreat_timeout_sec": LaunchConfiguration(
                "post_pick_retreat_timeout_sec"
            ),
            "reverse_authorization_topic": LaunchConfiguration(
                "reverse_authorization_topic"
            ),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_fake_hardware", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_moveit_rviz", default_value="false"),
            DeclareLaunchArgument(
                "control_mode", default_value="csp", choices=["mit", "csp"]
            ),
            DeclareLaunchArgument(
                "gripper_control_mode", default_value="mit", choices=["mit", "csp"]
            ),
            DeclareLaunchArgument("start_task_api", default_value="true"),
            DeclareLaunchArgument(
                "start_shopping_ticket_reader", default_value="true"
            ),
            DeclareLaunchArgument(
                "start_head_receipt_camera", default_value="true"
            ),
            DeclareLaunchArgument(
                "head_camera_serial", default_value="261922070132"
            ),
            DeclareLaunchArgument(
                "start_lift_level_test_server", default_value="false"
            ),
            DeclareLaunchArgument(
                "start_shelf_view_server", default_value="false"
            ),
            DeclareLaunchArgument("enable_shelf_view", default_value="false"),
            DeclareLaunchArgument("enable_post_pick_retreat", default_value="true"),
            DeclareLaunchArgument(
                "post_pick_retreat_speed", default_value="0.04"
            ),
            DeclareLaunchArgument(
                "post_pick_retreat_timeout_sec", default_value="5.0"
            ),
            DeclareLaunchArgument(
                "reverse_authorization_topic",
                default_value="/competition/post_pick_reverse_authorized",
            ),
            DeclareLaunchArgument("auto_homing", default_value="false"),
            DeclareLaunchArgument(
                "right_poe_calibrated", default_value="false"
            ),
            DeclareLaunchArgument(
                "map_yaml",
                default_value=str(
                    map_root / "2d" / "supermarket_test_05" / "map.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "pcd_path",
                default_value=str(
                    map_root / "3d" / "supermarket_test_05" / "map.pcd"
                ),
            ),
            DeclareLaunchArgument(
                "initial_pose",
                default_value="[-2.434000,-1.172000,0.0,0.0,0.0,0.0]",
            ),
            DeclareLaunchArgument(
                "nav_controller_max_wheel_speed", default_value="0.60"
            ),
            DeclareLaunchArgument(
                "nav_controller_wheel_accel_limit", default_value="0.50"
            ),
            DeclareLaunchArgument(
                "station_config",
                default_value=str(executor_share / "config" / "stations.yaml"),
            ),
            DeclareLaunchArgument("station_mission", default_value=""),
            DeclareLaunchArgument(
                "competition_config",
                default_value=str(executor_share / "config" / "competition.yaml"),
            ),
            robot_and_moveit,
            navigation,
            head_receipt_camera,
            task_api,
        ]
    )
