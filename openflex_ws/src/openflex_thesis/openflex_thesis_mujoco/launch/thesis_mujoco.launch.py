import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim_share = get_package_share_directory("openflex_thesis_mujoco")
    tc_share = get_package_share_directory("openflex_tc_wam")
    control_share = get_package_share_directory("openflex_thesis_control")
    tc_config = os.path.join(tc_share, "config", "tc_wam.yaml")
    control_config = os.path.join(control_share, "config", "control.yaml")
    sim_config = os.path.join(sim_share, "config", "mujoco.yaml")
    sim_scene = os.path.join(sim_share, "config", "sim_shelf_scene.yaml")
    core_launch = os.path.join(sim_share, "launch", "mujoco_core.launch.py")
    moveit_launch = os.path.join(sim_share, "launch", "mujoco_moveit.launch.py")
    use_moveit_evaluator = LaunchConfiguration("use_moveit_evaluator")
    return LaunchDescription(
        [
            DeclareLaunchArgument("upstream_model_path", default_value=""),
            DeclareLaunchArgument("experiment_config", default_value=sim_config),
            DeclareLaunchArgument("enable_viewer", default_value="false"),
            DeclareLaunchArgument("publish_rgbd", default_value="true"),
            DeclareLaunchArgument("start_moveit", default_value="false"),
            DeclareLaunchArgument("use_moveit_evaluator", default_value="false"),
            DeclareLaunchArgument("start_scripted_policy", default_value="true"),
            DeclareLaunchArgument("start_sim_navigation", default_value="true"),
            DeclareLaunchArgument("run_mission", default_value="false"),
            DeclareLaunchArgument("start_auto_compute", default_value="true"),
            DeclareLaunchArgument("start_moveit_executor", default_value="true"),
            DeclareLaunchArgument(
                "auto_evaluate_robustness", default_value="true"
            ),
            DeclareLaunchArgument(
                "candidate_distance_step_m", default_value="0.10"
            ),
            DeclareLaunchArgument(
                "candidate_lateral_step_m", default_value="0.10"
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(core_launch),
                launch_arguments={
                    "upstream_model_path": LaunchConfiguration("upstream_model_path"),
                    "enable_viewer": LaunchConfiguration("enable_viewer"),
                    "publish_rgbd": LaunchConfiguration("publish_rgbd"),
                    "experiment_config": LaunchConfiguration("experiment_config"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(moveit_launch),
                condition=IfCondition(LaunchConfiguration("start_moveit")),
            ),
            Node(
                package="openflex_tc_wam",
                executable="object_memory",
                name="object_memory",
                parameters=[tc_config, {"use_sim_time": True}],
                output="screen",
            ),
            Node(
                package="openflex_tc_wam",
                executable="anygrasp_gateway",
                name="anygrasp_gateway",
                parameters=[tc_config, {"use_sim_time": True}],
                output="screen",
            ),
            Node(
                package="openflex_thesis_mujoco",
                executable="oracle_anygrasp",
                name="mujoco_oracle_anygrasp",
                parameters=[sim_config, {"use_sim_time": True}, LaunchConfiguration("experiment_config")],
                output="screen",
            ),
            Node(
                package="openflex_tc_wam",
                executable="mock_evaluator",
                name="mock_evaluator",
                condition=UnlessCondition(use_moveit_evaluator),
                parameters=[tc_config, {"allow_mock": True, "use_sim_time": True}],
                output="screen",
            ),
            Node(
                package="openflex_tc_wam",
                executable="moveit_evaluator",
                name="moveit_evaluator",
                condition=IfCondition(use_moveit_evaluator),
                parameters=[
                    tc_config,
                    {
                        "shelf_scene_file": sim_scene,
                        "require_calibrated_scene": False,
                        "use_sim_time": True,
                    },
                ],
                output="screen",
            ),
            Node(
                package="openflex_tc_wam",
                executable="tc_wam_server",
                name="tc_wam_server",
                parameters=[
                    tc_config,
                    {
                        "distance_step_m": ParameterValue(
                            LaunchConfiguration("candidate_distance_step_m"),
                            value_type=float,
                        ),
                        "lateral_step_m": ParameterValue(
                            LaunchConfiguration("candidate_lateral_step_m"),
                            value_type=float,
                        ),
                        "use_sim_time": True,
                    },
                    LaunchConfiguration("experiment_config"),
                ],
                output="screen",
            ),
            Node(
                package="openflex_tc_wam",
                executable="auto_compute",
                name="auto_compute",
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            LaunchConfiguration("start_auto_compute"),
                            "'.lower() == 'true' and '",
                            LaunchConfiguration("run_mission"),
                            "'.lower() != 'true'",
                        ]
                    )
                ),
                parameters=[
                    tc_config,
                    {
                        "require_physical_perception": False,
                        "require_physical_evaluation": False,
                        "evaluate_robustness": ParameterValue(
                            LaunchConfiguration("auto_evaluate_robustness"),
                            value_type=bool,
                        ),
                        "use_sim_time": True,
                    },
                ],
                output="screen",
            ),
            Node(
                package="openflex_thesis_control",
                executable="velocity_arbiter",
                name="velocity_arbiter",
                parameters=[
                    control_config,
                    {"require_physical_affordance": False, "use_sim_time": True},
                ],
                output="screen",
            ),
            Node(
                package="openflex_thesis_control",
                executable="local_alignment_supervisor",
                name="local_alignment_supervisor",
                parameters=[
                    control_config,
                    {"require_physical_affordance": False, "use_sim_time": True},
                ],
                output="screen",
            ),
            Node(
                package="openflex_thesis_control",
                executable="moveit_grasp_executor",
                name="moveit_grasp_executor",
                condition=IfCondition(LaunchConfiguration("start_moveit_executor")),
                parameters=[
                    control_config,
                    {
                        "require_calibrated_object_geometry": False,
                        "require_physical_plan": False,
                        "object_box_dimensions_m": [0.07, 0.06, 0.16],
                        "shelf_scene_file": sim_scene,
                        "require_shelf_scene": True,
                        "require_calibrated_shelf_scene": False,
                        "use_sim_time": True,
                    },
                ],
                output="screen",
            ),
            Node(
                package="openflex_thesis_control",
                executable="fixed_place_supervisor",
                name="fixed_place_supervisor",
                parameters=[control_config, {"use_sim_time": True}],
                output="screen",
            ),
            Node(
                package="openflex_thesis_control",
                executable="mission_orchestrator",
                name="mission_orchestrator",
                parameters=[
                    control_config,
                    {
                        "require_physical_affordance": False,
                        "require_physical_perception": False,
                        "use_sim_time": True,
                    },
                    LaunchConfiguration("experiment_config"),
                ],
                output="screen",
            ),
            Node(
                package="openflex_thesis_mujoco",
                executable="scripted_local_policy",
                name="scripted_local_alignment_baseline",
                condition=IfCondition(LaunchConfiguration("start_scripted_policy")),
                parameters=[sim_config, {"use_sim_time": True}],
                output="screen",
            ),
            Node(
                package="openflex_thesis_mujoco",
                executable="sim_navigation",
                name="sim_navigation_baseline_not_nav2",
                condition=IfCondition(LaunchConfiguration("start_sim_navigation")),
                parameters=[sim_config, {"use_sim_time": True}],
                output="screen",
            ),
            Node(
                package="openflex_thesis_mujoco",
                executable="sim_mission_client",
                name="thesis_mujoco_mission_client",
                condition=IfCondition(LaunchConfiguration("run_mission")),
                parameters=[sim_config, {"use_sim_time": True}, LaunchConfiguration("experiment_config")],
                output="screen",
            ),
        ]
    )
