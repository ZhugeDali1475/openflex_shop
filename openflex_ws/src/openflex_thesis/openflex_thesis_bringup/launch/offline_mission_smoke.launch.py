from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
import os


def generate_launch_description():
    tc_config = os.path.join(
        get_package_share_directory("openflex_tc_wam"), "config", "tc_wam.yaml"
    )
    control_config = os.path.join(
        get_package_share_directory("openflex_thesis_control"), "config", "control.yaml"
    )
    synthetic = {
        "require_physical_affordance": False,
        "require_physical_perception": False,
    }
    mission_client = Node(
        package="openflex_thesis_control",
        executable="demo_mission_client",
        name="demo_mission_client",
        output="screen",
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
            Node(package="openflex_tc_wam", executable="manual_object_source",
                 name="manual_object_source", parameters=[tc_config], output="screen"),
            Node(package="openflex_tc_wam", executable="object_memory",
                 name="object_memory", parameters=[tc_config], output="screen"),
            Node(package="openflex_tc_wam", executable="mock_anygrasp",
                 name="mock_anygrasp", parameters=[tc_config, {"allow_mock": True}], output="screen"),
            Node(package="openflex_tc_wam", executable="mock_evaluator",
                 name="mock_evaluator", parameters=[tc_config, {"allow_mock": True}], output="screen"),
            Node(package="openflex_tc_wam", executable="tc_wam_server",
                 name="tc_wam_server", parameters=[tc_config], output="screen"),
            Node(package="openflex_thesis_control", executable="velocity_arbiter",
                 name="velocity_arbiter", parameters=[control_config, synthetic], output="screen"),
            Node(package="openflex_thesis_control", executable="fixed_place_supervisor",
                 name="fixed_place_supervisor", parameters=[control_config], output="screen"),
            Node(package="openflex_thesis_control", executable="mission_orchestrator",
                 name="mission_orchestrator", parameters=[control_config, synthetic], output="screen"),
            Node(package="openflex_thesis_control", executable="mock_dependencies",
                 name="mock_dependencies", output="screen"),
            mission_client,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=mission_client,
                    on_exit=[Shutdown(reason="offline smoke mission client completed")],
                )
            ),
        ]
    )
