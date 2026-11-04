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
                package="tf2_ros",
                executable="static_transform_publisher",
                name="offline_base_tf",
                arguments=[
                    "--x", "0", "--y", "0", "--z", "0",
                    "--yaw", "0", "--pitch", "0", "--roll", "0",
                    "--frame-id", "map", "--child-frame-id", "base_link",
                ],
                output="screen",
            ),
            Node(
                package="openflex_tc_wam",
                executable="manual_object_source",
                name="manual_object_source",
                parameters=[config],
                output="screen",
            ),
            Node(
                package="openflex_tc_wam",
                executable="object_memory",
                name="object_memory",
                parameters=[config],
                output="screen",
            ),
            Node(
                package="openflex_tc_wam",
                executable="mock_anygrasp",
                name="mock_anygrasp",
                parameters=[config, {"allow_mock": True}],
                output="screen",
            ),
            Node(
                package="openflex_tc_wam",
                executable="mock_evaluator",
                name="mock_evaluator",
                parameters=[config, {"allow_mock": True}],
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
                parameters=[config, {
                    "require_physical_perception": False,
                    "require_physical_evaluation": False,
                }],
                output="screen",
            ),
        ]
    )
