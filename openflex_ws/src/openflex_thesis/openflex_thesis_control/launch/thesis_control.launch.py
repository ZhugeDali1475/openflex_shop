from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("openflex_thesis_control"), "config", "control.yaml"
    )
    return LaunchDescription(
        [
            Node(
                package="openflex_thesis_control",
                executable="velocity_arbiter",
                name="velocity_arbiter",
                parameters=[config],
                output="screen",
            ),
            Node(
                package="openflex_thesis_control",
                executable="local_alignment_supervisor",
                name="local_alignment_supervisor",
                parameters=[config],
                output="screen",
            ),
            Node(
                package="openflex_thesis_control",
                executable="moveit_grasp_executor",
                name="moveit_grasp_executor",
                parameters=[config],
                output="screen",
            ),
            Node(
                package="openflex_thesis_control",
                executable="fixed_place_supervisor",
                name="fixed_place_supervisor",
                parameters=[config],
                output="screen",
            ),
            Node(
                package="openflex_thesis_control",
                executable="mission_orchestrator",
                name="mission_orchestrator",
                parameters=[config],
                output="screen",
            ),
        ]
    )
