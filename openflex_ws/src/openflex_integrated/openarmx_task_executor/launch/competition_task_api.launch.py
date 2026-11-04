"""One competition API launch without duplicating robot/nav/MoveIt bringup."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    package_share = FindPackageShare("openarmx_task_executor")
    executor_launch = PathJoinSubstitution(
        [package_share, "launch", "task_executor.launch.py"]
    )
    docking_launch = PathJoinSubstitution(
        [package_share, "launch", "station_docking.launch.py"]
    )
    shelf_align_launch = PathJoinSubstitution(
        [package_share, "launch", "shelf_fine_align.launch.py"]
    )
    retreat_launch = PathJoinSubstitution(
        [package_share, "launch", "station_retreat.launch.py"]
    )
    coordinator_launch = PathJoinSubstitution(
        [package_share, "launch", "competition_coordinator.launch.py"]
    )
    station_config = PathJoinSubstitution(
        [package_share, "config", "stations.yaml"]
    )

    station_config_arg = DeclareLaunchArgument(
        "station_config", default_value=station_config
    )
    station_mission_arg = DeclareLaunchArgument(
        "station_mission", default_value=""
    )
    competition_config_arg = DeclareLaunchArgument(
        "competition_config",
        default_value=PathJoinSubstitution(
            [package_share, "config", "competition.yaml"]
        ),
    )
    enable_docking_arg = DeclareLaunchArgument(
        "enable_docking", default_value="true"
    )
    enable_detection_arg = DeclareLaunchArgument(
        "enable_detection", default_value="true"
    )
    start_docking_arg = DeclareLaunchArgument(
        "start_docking_server", default_value="true"
    )
    start_shelf_align_arg = DeclareLaunchArgument(
        "start_shelf_fine_align_server", default_value="true"
    )
    start_retreat_arg = DeclareLaunchArgument(
        "start_station_retreat_server", default_value="true"
    )
    start_mock_pick_arg = DeclareLaunchArgument(
        "start_mock_pick_server", default_value="true"
    )
    start_coordinator_arg = DeclareLaunchArgument(
        "start_competition_coordinator", default_value="true"
    )
    start_shopping_ticket_reader_arg = DeclareLaunchArgument(
        "start_shopping_ticket_reader", default_value="true"
    )
    start_lift_level_test_arg = DeclareLaunchArgument(
        "start_lift_level_test_server", default_value="false"
    )
    start_shelf_view_arg = DeclareLaunchArgument(
        "start_shelf_view_server", default_value="false"
    )
    enable_shelf_view_arg = DeclareLaunchArgument(
        "enable_shelf_view", default_value="false"
    )
    docking_cmd_arg = DeclareLaunchArgument(
        "docking_cmd_vel_topic", default_value="/cmd_vel"
    )
    enable_retreat_arg = DeclareLaunchArgument(
        "enable_post_pick_retreat", default_value="true"
    )
    retreat_speed_arg = DeclareLaunchArgument(
        "post_pick_retreat_speed", default_value="0.04"
    )
    retreat_timeout_arg = DeclareLaunchArgument(
        "post_pick_retreat_timeout_sec", default_value="5.0"
    )
    reverse_authorization_topic_arg = DeclareLaunchArgument(
        "reverse_authorization_topic",
        default_value="/competition/post_pick_reverse_authorized",
    )

    executor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(executor_launch),
        launch_arguments={
            "station_config": LaunchConfiguration("station_config"),
            "station_mission": LaunchConfiguration("station_mission"),
            "enable_docking": LaunchConfiguration("enable_docking"),
            "enable_detection": LaunchConfiguration("enable_detection"),
            "enable_shelf_view": LaunchConfiguration("enable_shelf_view"),
            "stop_cmd_topic": LaunchConfiguration("docking_cmd_vel_topic"),
            "enable_post_pick_retreat": LaunchConfiguration(
                "enable_post_pick_retreat"
            ),
            "post_pick_retreat_speed": LaunchConfiguration(
                "post_pick_retreat_speed"
            ),
            "post_pick_retreat_timeout_sec": LaunchConfiguration(
                "post_pick_retreat_timeout_sec"
            ),
        }.items(),
    )
    docking = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(docking_launch),
        condition=IfCondition(LaunchConfiguration("start_docking_server")),
        launch_arguments={
            "cmd_vel_topic": LaunchConfiguration("docking_cmd_vel_topic")
        }.items(),
    )
    shelf_align = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(shelf_align_launch),
        condition=IfCondition(LaunchConfiguration("start_shelf_fine_align_server")),
        launch_arguments={
            "cmd_vel_topic": LaunchConfiguration("docking_cmd_vel_topic")
        }.items(),
    )
    retreat = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(retreat_launch),
        condition=IfCondition(LaunchConfiguration("start_station_retreat_server")),
        launch_arguments={
            "cmd_vel_topic": LaunchConfiguration("docking_cmd_vel_topic"),
            "reverse_authorization_topic": LaunchConfiguration(
                "reverse_authorization_topic"
            ),
        }.items(),
    )
    mock_pick = Node(
        package="openarmx_task_executor",
        executable="competition_mock_pick",
        name="competition_mock_pick",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_mock_pick_server")),
    )
    shopping_ticket_reader = Node(
        package="openarmx_task_executor",
        executable="shopping_ticket_reader",
        name="shopping_ticket_reader",
        output="screen",
        condition=IfCondition(
            LaunchConfiguration("start_shopping_ticket_reader")
        ),
        parameters=[
            {
                "competition_config": LaunchConfiguration(
                    "competition_config"
                ),
            }
        ],
    )
    lift_level_test = Node(
        package="openarmx_task_executor",
        executable="lift_level_test",
        name="lift_level_test",
        output="screen",
        condition=IfCondition(
            LaunchConfiguration("start_lift_level_test_server")
        ),
        parameters=[
            {
                "station_config": LaunchConfiguration("station_config"),
                "station_mission": LaunchConfiguration("station_mission"),
                "action_name": "/competition/move_lift_to_level",
                "trajectory_action_name": "/lift_joint_controller/follow_joint_trajectory",
                "odom_topic": "/odom_safe",
            }
        ],
    )
    shelf_view = Node(
        package="openarmx_task_executor",
        executable="shelf_view",
        name="shelf_view",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_shelf_view_server")),
        parameters=[
            {
                "station_config": LaunchConfiguration("station_config"),
                "station_mission": LaunchConfiguration("station_mission"),
                "action_name": "/competition/move_to_shelf_view",
                "left_trajectory_action_name": (
                    "/left_joint_trajectory_controller/follow_joint_trajectory"
                ),
                "right_trajectory_action_name": (
                    "/right_joint_trajectory_controller/follow_joint_trajectory"
                ),
                "lift_trajectory_action_name": (
                    "/lift_joint_controller/follow_joint_trajectory"
                ),
                "odom_topic": "/odom_safe",
            }
        ],
    )
    coordinator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(coordinator_launch),
        condition=IfCondition(LaunchConfiguration("start_competition_coordinator")),
        launch_arguments={
            "station_config": LaunchConfiguration("station_config"),
            "station_mission": LaunchConfiguration("station_mission"),
            "competition_config": LaunchConfiguration("competition_config"),
        }.items(),
    )
    return LaunchDescription(
        [
            station_config_arg,
            station_mission_arg,
            competition_config_arg,
            enable_docking_arg,
            enable_detection_arg,
            start_docking_arg,
            start_shelf_align_arg,
            start_retreat_arg,
            start_mock_pick_arg,
            start_coordinator_arg,
            start_shopping_ticket_reader_arg,
            start_lift_level_test_arg,
            start_shelf_view_arg,
            enable_shelf_view_arg,
            docking_cmd_arg,
            enable_retreat_arg,
            retreat_speed_arg,
            retreat_timeout_arg,
            reverse_authorization_topic_arg,
            executor,
            docking,
            shelf_align,
            retreat,
            mock_pick,
            shopping_ticket_reader,
            lift_level_test,
            shelf_view,
            coordinator,
        ]
    )
