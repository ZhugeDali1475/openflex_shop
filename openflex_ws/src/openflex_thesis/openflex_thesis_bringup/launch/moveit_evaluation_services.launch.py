"""Start MoveIt services only, reusing the robot_state_publisher from robot bringup."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory("openarmx_integrated_description"),
        "urdf",
        "openarmx_integrated_robot.urdf.xacro",
    )
    moveit_config = (
        MoveItConfigsBuilder(
            "openarmx_integrated", package_name="openarmx_integrated_moveit_config"
        )
        .robot_description(
            file_path=urdf_path,
            mappings={"use_fake_hardware": "false", "enable_head": "true"},
        )
        .to_moveit_configs()
    )
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                # TC-WAM needs IK and planning-scene services, not trajectory execution.
                "allow_trajectory_execution": False,
            },
        ],
    )
    # Deliberately no robot_state_publisher and no static world->base_link node.
    return LaunchDescription([move_group])
