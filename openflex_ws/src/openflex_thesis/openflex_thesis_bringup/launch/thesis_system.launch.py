from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    tc_share = get_package_share_directory("openflex_tc_wam")
    control_share = get_package_share_directory("openflex_thesis_control")
    tc_config = os.path.join(tc_share, "config", "tc_wam.yaml")
    control_launch = os.path.join(control_share, "launch", "thesis_control.launch.py")
    moveit_launch = os.path.join(
        get_package_share_directory("openflex_thesis_bringup"),
        "launch",
        "moveit_execution_services.launch.py",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_object_adapter",
                default_value="false",
                description="Adapt an external/DREAM PoseStamped into ObjectObservation",
            ),
            DeclareLaunchArgument(
                "object_pose_topic", default_value="/dream/active_object_pose"
            ),
            DeclareLaunchArgument("object_id", default_value="thesis_item_01"),
            DeclareLaunchArgument(
                "start_move_group",
                default_value="false",
                description="Start execution-capable move_group without duplicate robot_state_publisher",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(moveit_launch),
                condition=IfCondition(LaunchConfiguration("start_move_group")),
            ),
            Node(
                package="openflex_tc_wam",
                executable="object_observation_adapter",
                name="object_observation_adapter",
                parameters=[
                    tc_config,
                    {
                        "pose_topic": LaunchConfiguration("object_pose_topic"),
                        "object_id": LaunchConfiguration("object_id"),
                    },
                ],
                condition=IfCondition(LaunchConfiguration("start_object_adapter")),
                output="screen",
            ),
            # Robot hardware, localization and Nav2 are intentionally external.
            # MoveIt is external unless start_move_group=true. No duplicate RSP/odom/controller.
            Node(
                package="openflex_tc_wam",
                executable="object_memory",
                name="object_memory",
                parameters=[tc_config],
                output="screen",
            ),
            Node(
                package="openflex_tc_wam",
                executable="anygrasp_gateway",
                name="anygrasp_gateway",
                parameters=[tc_config],
                output="screen",
            ),
            Node(
                package="openflex_tc_wam",
                executable="moveit_evaluator",
                name="moveit_evaluator",
                parameters=[tc_config],
                output="screen",
            ),
            Node(
                package="openflex_tc_wam",
                executable="tc_wam_server",
                name="tc_wam_server",
                parameters=[tc_config],
                output="screen",
            ),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(control_launch)),
        ]
    )
