"""Full-body competition runtime without mapping, localization, or Nav2."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    moveit_share = Path(
        get_package_share_directory("openarmx_integrated_moveit_config")
    )
    executor_share = Path(get_package_share_directory("openarmx_task_executor"))
    head_vision_share = Path(
        get_package_share_directory("openarmx_head_vision_h264")
    )

    wrist_cameras = []
    for arm in ("right", "left"):
        camera_name = f"{arm}_camera"
        namespace = f"cam_{arm}"
        wrist_cameras.append(
            Node(
                package="realsense2_camera",
                executable="realsense2_camera_node",
                namespace=namespace,
                name=camera_name,
                output="screen",
                condition=IfCondition(
                    LaunchConfiguration("start_grasp_cameras")
                ),
                parameters=[{
                    "serial_no": ParameterValue(
                        LaunchConfiguration(f"{arm}_camera_serial"),
                        value_type=str,
                    ),
                    "camera_name": camera_name,
                    "wait_for_device_timeout": -1.0,
                    "reconnect_timeout": 2.0,
                    "enable_color": True,
                    "enable_depth": True,
                    "align_depth.enable": True,
                    "pointcloud.enable": False,
                    "enable_infra1": False,
                    "enable_infra2": False,
                    # The calibrated URDF joint owns wrist -> camera_link;
                    # RealSense uniquely publishes camera_link -> optical frames.
                    "publish_tf": True,
                }],
                remappings=[
                    (
                        f"/{namespace}/{camera_name}/color/image_raw",
                        f"/{namespace}/color/image",
                    ),
                    (
                        f"/{namespace}/{camera_name}/aligned_depth_to_color/image_raw",
                        f"/{namespace}/depth/image",
                    ),
                    (
                        f"/{namespace}/{camera_name}/aligned_depth_to_color/camera_info",
                        f"/{namespace}/depth/camera_info",
                    ),
                ],
            )
        )

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
            "start_base_navigation_bridge": "false",
            "start_pick_task_executor": "true",
            "start_joint_posture_executor": "true",
            # safe_odom_mux below is the sole odom -> base_link TF publisher.
            "enable_odom_tf": "false",
            # Mapping teleop and navigation both target the final safe output.
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

    cartesian_approach = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                Path(get_package_share_directory("openflex_grasp_moveit"))
                / "launch"
                / "cartesian_approach_executor.launch.py"
            )
        ),
        condition=IfCondition(LaunchConfiguration("start_cartesian_approach")),
        launch_arguments={"use_sim_time": "false"}.items(),
    )
    moveit_ready_for_cartesian = Node(
        package="openarmx_integrated_moveit_config",
        executable="wait_for_moveit.py",
        name="wait_for_moveit_for_cartesian",
        arguments=["--timeout", "60.0"],
        output="both",
        condition=IfCondition(LaunchConfiguration("start_cartesian_approach")),
    )

    def start_cartesian_after_moveit(event, _context):
        if event.returncode != 0:
            return [
                LogInfo(
                    msg=(
                        "MoveIt was not ready; Cartesian approach executor will not start "
                        f"(return code {event.returncode})"
                    )
                )
            ]
        return [cartesian_approach]

    start_cartesian = RegisterEventHandler(
        OnProcessExit(
            target_action=moveit_ready_for_cartesian,
            on_exit=start_cartesian_after_moveit,
        )
    )

    safe_odom_mux = Node(
        package="swerve_navigation",
        executable="safe_odom_mux.py",
        name="safe_odom_mux",
        parameters=[{
            "use_sim_time": False,
            "lio_odom_topic": "/fastlio2/lio_odom",
            "wheel_odom_topic": "/odom",
            "output_odom_topic": "/odom_safe",
            "odom_frame_id": "odom",
            "base_frame_id": "base_link",
            "publish_tf": True,
        }],
        output="screen",
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
            "start_shelf_fine_align_server": "true",
            "start_competition_coordinator": LaunchConfiguration(
                "start_competition_coordinator"
            ),
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
            "start_mock_pick_server": "false",
        }.items(),
    )

    grasp_detect = Node(
        package="openflex_grasp", executable="detect_object_server",
        name="openflex_detect_object", output="screen",
        condition=IfCondition(LaunchConfiguration("start_grasp_backend")),
        parameters=[{
            "detection_ttl_s": 180.0,
            "right_camera_serial": ParameterValue(
                LaunchConfiguration("right_camera_serial"), value_type=str
            ),
            "left_camera_serial": ParameterValue(
                LaunchConfiguration("left_camera_serial"), value_type=str
            ),
        }],
    )
    grasp_worker = Node(
        package="openflex_grasp", executable="graspnet_worker",
        name="openflex_graspnet_worker", output="screen",
        condition=IfCondition(LaunchConfiguration("start_grasp_backend")),
        parameters=[{
            "grasp_depth_offset_m": ParameterValue(
                LaunchConfiguration("grasp_depth_offset_m"), value_type=float
            ),
        }],
    )
    grasp_pick = Node(
        package="openflex_grasp", executable="pick_object_server",
        name="openflex_pick_object", output="screen",
        condition=IfCondition(LaunchConfiguration("start_grasp_backend")),
        parameters=[{
            "execute": ParameterValue(
                LaunchConfiguration("execute_grasp"), value_type=bool
            ),
            "object_bottom_clearance_m": ParameterValue(
                LaunchConfiguration("object_bottom_clearance_m"), value_type=float
            ),
        }],
    )
    grasp_sequence = Node(
        package="openflex_grasp", executable="pick_sequence_server",
        name="openflex_pick_sequence", output="screen",
        condition=IfCondition(LaunchConfiguration("start_grasp_backend")),
        parameters=[{
            "object_bottom_clearance_m": ParameterValue(
                LaunchConfiguration("object_bottom_clearance_m"), value_type=float
            ),
        }],
    )
    grasp_visualizer = Node(
        package="openflex_grasp", executable="product_grasp_web",
        name="openflex_grasp_readonly_web", output="screen",
        condition=IfCondition(LaunchConfiguration("start_grasp_backend")),
        parameters=[{"open_browser": False, "port": 8765}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_fake_hardware", default_value="false"),
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
            DeclareLaunchArgument("start_grasp_backend", default_value="true"),
            DeclareLaunchArgument(
                "start_cartesian_approach",
                default_value=LaunchConfiguration("start_grasp_backend"),
            ),
            DeclareLaunchArgument("start_grasp_cameras", default_value="true"),
            DeclareLaunchArgument(
                "right_camera_serial",
                default_value=os.environ.get(
                    "OPENFLEX_RIGHT_CAMERA_SERIAL", "260322275347"
                ),
            ),
            DeclareLaunchArgument(
                "left_camera_serial",
                default_value=os.environ.get(
                    "OPENFLEX_LEFT_CAMERA_SERIAL", "260322276903"
                ),
            ),
            DeclareLaunchArgument("execute_grasp", default_value="false"),
            DeclareLaunchArgument(
                "grasp_depth_offset_m", default_value="0.05"
            ),
            DeclareLaunchArgument(
                "object_bottom_clearance_m", default_value="0.01"
            ),
            DeclareLaunchArgument(
                # Test-9 phase 1 is driven by the competition-flow GUI.  The
                # separate legacy three-phase coordinator requires its own
                # complete station contract and must only be enabled with a
                # matching competition_config.
                "start_competition_coordinator", default_value="false"
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
                "station_config",
                default_value=str(executor_share / "config" / "stations.yaml"),
            ),
            DeclareLaunchArgument("station_mission", default_value=""),
            DeclareLaunchArgument(
                "competition_config",
                default_value=str(executor_share / "config" / "competition.yaml"),
            ),
            robot_and_moveit,
            head_receipt_camera,
            start_cartesian,
            moveit_ready_for_cartesian,
            safe_odom_mux,
            task_api,
            *wrist_cameras,
            grasp_detect,
            grasp_worker,
            grasp_pick,
            grasp_sequence,
            grasp_visualizer,
        ]
    )
