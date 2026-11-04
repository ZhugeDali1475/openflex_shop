from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    station_config = PathJoinSubstitution(
        [FindPackageShare("openarmx_task_executor"), "config", "stations.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("station_config", default_value=station_config),
            DeclareLaunchArgument("station_mission", default_value=""),
            DeclareLaunchArgument("enable_docking", default_value="true"),
            DeclareLaunchArgument("enable_detection", default_value="true"),
            DeclareLaunchArgument("enable_shelf_view", default_value="false"),
            DeclareLaunchArgument("allow_nominal_detection_fallback", default_value="false"),
            DeclareLaunchArgument("nav_action_name", default_value="/navigate_to_pose"),
            DeclareLaunchArgument("dock_action_name", default_value="/dock_to_station"),
            DeclareLaunchArgument(
                "shelf_align_action_name", default_value="/shelf_fine_align"
            ),
            DeclareLaunchArgument(
                "retreat_action_name", default_value="/retreat_from_station"
            ),
            DeclareLaunchArgument("detect_action_name", default_value="/detect_object"),
            DeclareLaunchArgument("pick_action_name", default_value="/pick_object"),
            DeclareLaunchArgument("place_action_name", default_value="/place_object"),
            DeclareLaunchArgument(
                "shelf_view_action_name",
                default_value="/competition/move_to_shelf_view",
            ),
            DeclareLaunchArgument("odom_topic", default_value="/odom_safe"),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("default_arm_name", default_value="left"),
            DeclareLaunchArgument("max_manipulation_dof", default_value="8"),
            DeclareLaunchArgument("lift_joint_min_position", default_value="-0.650"),
            DeclareLaunchArgument("lift_joint_max_position", default_value="0.300"),
            DeclareLaunchArgument("publish_stop_cmd", default_value="false"),
            DeclareLaunchArgument("stop_cmd_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("enable_post_pick_retreat", default_value="false"),
            DeclareLaunchArgument("post_pick_retreat_speed", default_value="0.04"),
            DeclareLaunchArgument(
                "post_pick_retreat_timeout_sec", default_value="5.0"
            ),
            Node(
                package="openarmx_task_executor",
                executable="task_executor",
                name="openarmx_task_executor",
                output="screen",
                parameters=[
                    {
                        "station_config": LaunchConfiguration("station_config"),
                        "station_mission": LaunchConfiguration("station_mission"),
                        "enable_docking": LaunchConfiguration("enable_docking"),
                        "enable_detection": LaunchConfiguration("enable_detection"),
                        "enable_shelf_view": LaunchConfiguration("enable_shelf_view"),
                        "allow_nominal_detection_fallback": LaunchConfiguration(
                            "allow_nominal_detection_fallback"
                        ),
                        "nav_action_name": LaunchConfiguration("nav_action_name"),
                        "dock_action_name": LaunchConfiguration("dock_action_name"),
                        "shelf_align_action_name": LaunchConfiguration(
                            "shelf_align_action_name"
                        ),
                        "retreat_action_name": LaunchConfiguration(
                            "retreat_action_name"
                        ),
                        "detect_action_name": LaunchConfiguration("detect_action_name"),
                        "pick_action_name": LaunchConfiguration("pick_action_name"),
                        "place_action_name": LaunchConfiguration("place_action_name"),
                        "shelf_view_action_name": LaunchConfiguration(
                            "shelf_view_action_name"
                        ),
                        "odom_topic": LaunchConfiguration("odom_topic"),
                        "map_frame": LaunchConfiguration("map_frame"),
                        "base_frame": LaunchConfiguration("base_frame"),
                        "default_arm_name": LaunchConfiguration("default_arm_name"),
                        "max_manipulation_dof": LaunchConfiguration("max_manipulation_dof"),
                        "lift_joint_min_position": LaunchConfiguration(
                            "lift_joint_min_position"
                        ),
                        "lift_joint_max_position": LaunchConfiguration(
                            "lift_joint_max_position"
                        ),
                        "publish_stop_cmd": LaunchConfiguration("publish_stop_cmd"),
                        "stop_cmd_topic": LaunchConfiguration("stop_cmd_topic"),
                        "enable_post_pick_retreat": LaunchConfiguration(
                            "enable_post_pick_retreat"
                        ),
                        "post_pick_retreat_speed": LaunchConfiguration(
                            "post_pick_retreat_speed"
                        ),
                        "post_pick_retreat_timeout_sec": LaunchConfiguration(
                            "post_pick_retreat_timeout_sec"
                        ),
                    }
                ],
            ),
        ]
    )
