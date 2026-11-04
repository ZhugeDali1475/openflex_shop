"""Start the bimanual MoveGroup against an already-running robot stack."""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    description_path = os.path.join(
        get_package_share_directory("openarmx_description"),
        "urdf",
        "robot",
        "v10.urdf.xacro",
    )
    robot_description = xacro.process_file(
        description_path,
        mappings={
            "arm_type": "v10",
            "bimanual": "true",
            "use_fake_hardware": "false",
            "ros2_control": "true",
            "can_fd": "false",
            "control_mode": "mit",
            "left_can_interface": "can1",
            "right_can_interface": "can0",
            "right_wrist_camera": "true",
            # Integrated robot mounting at lift_joint=0.033638 m. The lift is
            # held fixed for this right-arm-only hardware test.
            "left_arm_base_xyz": "0.16755 0.075 0.938638",
            "right_arm_base_xyz": "0.16755 -0.075 0.938638",
        },
    ).toprettyxml(indent="  ")

    moveit_config = MoveItConfigsBuilder(
        "openarmx_bimanual",
        package_name="openarmx_bimanual_moveit_config",
    ).to_moveit_configs()
    moveit_parameters = moveit_config.to_dict()
    moveit_parameters["robot_description"] = robot_description

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[moveit_parameters],
        remappings=[("/joint_states", "/bimanual_joint_states")],
    )
    joint_state_filter = Node(
        package="openarmx_bimanual_moveit_config",
        executable="bimanual_joint_state_filter.py",
        name="bimanual_joint_state_filter",
        output="screen",
    )
    return LaunchDescription([joint_state_filter, move_group])
