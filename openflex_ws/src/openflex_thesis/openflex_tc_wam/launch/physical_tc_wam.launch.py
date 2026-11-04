from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("openflex_tc_wam"), "config", "tc_wam.yaml"
    )
    return LaunchDescription(
        [
            Node(
                package="openflex_tc_wam",
                executable="object_memory",
                name="object_memory",
                parameters=[config],
                output="screen",
            ),
            Node(
                package="openflex_tc_wam",
                executable="anygrasp_gateway",
                name="anygrasp_gateway",
                parameters=[config],
                output="screen",
            ),
            Node(
                package="openflex_tc_wam",
                executable="moveit_evaluator",
                name="moveit_evaluator",
                parameters=[config],
                output="screen",
            ),
            Node(
                package="openflex_tc_wam",
                executable="tc_wam_server",
                name="tc_wam_server",
                parameters=[config],
                output="screen",
            ),
            Node(
                package="openflex_tc_wam",
                executable="auto_compute",
                name="auto_compute",
                parameters=[config],
                output="screen",
            ),
        ]
    )
