from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import os
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from openflex_preflight_config import resolve_shelf_scene


def generate_launch_description():
    root = Path(__file__).resolve().parents[1]
    thesis = root / "openflex_ws/src/openflex_thesis"
    tc_config = str(thesis / "openflex_tc_wam/config/tc_wam.yaml")
    sim_config = str(thesis / "openflex_thesis_mujoco/config/mujoco.yaml")
    scene = resolve_shelf_scene(
        default_scene=thesis / "openflex_thesis_mujoco/config/sim_shelf_scene.yaml",
        environment=os.environ,
    )
    urdf = Path(get_package_share_directory("openarmx_integrated_description")) / "urdf/openarmx_integrated_robot.urdf.xacro"
    configuration = (
        MoveItConfigsBuilder("openarmx_integrated", package_name="openarmx_integrated_moveit_config")
        .robot_description(file_path=str(urdf), mappings={"use_fake_hardware": "false", "enable_head": "true"})
        .to_moveit_configs()
    )
    ompl = configuration.planning_pipelines['ompl']
    group = ompl.setdefault('right_arm_with_lift', {})
    group['longest_valid_segment_fraction'] = 0.001
    group['maximum_waypoint_distance'] = 0.005
    # The default profile is visual-only.  Ground-truth object memory and the
    # MuJoCo grasp oracle are opt-in comparison infrastructure, never an
    # implicit dependency of a visual run.
    use_oracle = os.environ.get("OPENFLEX_ENABLE_ORACLE_GRASP") == "1"
    actions = [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             name="openflex_preflight_robot_state_publisher", output="screen",
             parameters=[configuration.robot_description, {"use_sim_time": True}]),
        Node(package="moveit_ros_move_group", executable="move_group", name="move_group", output="screen",
             parameters=[configuration.to_dict(), {"use_sim_time": True, "allow_trajectory_execution": False,
                                                  "moveit_manage_controllers": False}]),
        Node(package="openflex_tc_wam", executable="anygrasp_gateway", name="anygrasp_gateway", output="screen",
             parameters=[tc_config, {"use_sim_time": True}]),
        Node(package="openflex_tc_wam", executable="moveit_evaluator", name="moveit_evaluator", output="screen",
             parameters=[tc_config, {"use_sim_time": True, "shelf_scene_file": scene, "require_calibrated_scene": False}]),
        # Standoff window must clear the protruding gondola base level in the
        # single-red fixture; a 0.60 m standoff put the chassis into the shelf.
        Node(package="openflex_tc_wam", executable="tc_wam_server", name="tc_wam_server", output="screen",
             parameters=[tc_config, {"use_sim_time": True, "distance_min_m": 0.70,
                                     "distance_step_m": 0.10, "lateral_step_m": 0.15}]),
    ]
    if use_oracle:
        actions[2:2] = [
            Node(package="openflex_tc_wam", executable="object_memory", name="object_memory", output="screen",
                 parameters=[tc_config, {"use_sim_time": True}]),
            Node(package="openflex_thesis_mujoco", executable="oracle_anygrasp", name="mujoco_oracle_anygrasp", output="screen",
                 parameters=[sim_config, {"use_sim_time": True}]),
        ]
    return LaunchDescription(actions)
