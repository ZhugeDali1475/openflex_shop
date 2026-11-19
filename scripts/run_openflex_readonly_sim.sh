#!/usr/bin/env bash
set -eo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/humble/setup.bash
source "${root}/openflex_ws/install_v2/local_setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-83}"
export MUJOCO_GL="${MUJOCO_GL:-egl}"
export OPENFLEX_ROOT="${root}"
export OPENFLEX_WORKSPACE="${root}/openflex_ws"
export OPENFLEX_MODEL_ROOT="${root}/openflex_models"
export OPENFLEX_MUJOCO_MODEL="${root}/openflex_mujoco-1.0_basic/openflex_mujoco.xml"
export PYTHONPATH="${root}/openflex_ws/src/openflex_thesis/openflex_thesis_mujoco:${PYTHONPATH:-}"
export PYTHONUNBUFFERED=1
selected_velocity_topic="/openflex_readonly_disabled/cmd_vel"
lift_action="/openflex_readonly_disabled/lift"
right_arm_action="/openflex_readonly_disabled/right_arm"
right_gripper_action="/openflex_readonly_disabled/right_gripper"
memory_topic="/openflex_readonly_disabled/object_memory"
place_topic="/openflex_readonly_disabled/place_control"
if [[ "${OPENFLEX_ENABLE_SIM_NAV:-0}" == "1" ]]; then
    [[ "${ROS_DOMAIN_ID}" == "83" ]] || { echo "Simulation navigation requires domain 83" >&2; exit 1; }
    selected_velocity_topic="/thesis/cmd_vel_selected"
fi
if [[ "${OPENFLEX_ENABLE_SIM_MANIPULATION:-0}" == "1" ]]; then
    [[ "${ROS_DOMAIN_ID}" == "83" && "${OPENFLEX_ENABLE_SIM_NAV:-0}" == "1" ]] || exit 1
    lift_action="/lift_joint_controller/follow_joint_trajectory"
    right_arm_action="/right_joint_trajectory_controller/follow_joint_trajectory"
    right_gripper_action="/right_gripper_controller/gripper_cmd"
    memory_topic="/thesis/object_memory"
    echo 'Explicit simulation manipulation opt-in: right arm/lift/gripper only; legacy placement and left arm remain disabled.'
fi
if [[ "${OPENFLEX_ENABLE_SIM_PLACE:-0}" == "1" ]]; then
    [[ "${OPENFLEX_ENABLE_SIM_MANIPULATION:-0}" == "1" && "${ROS_DOMAIN_ID}" == "83" ]] || exit 1
    place_topic="/thesis/place/control"
    echo 'Explicit simulation placement opt-in: guarded actual-pose release only; scene mutation remains disabled.'
fi
mkdir -p "${root}/local_dependencies/openflex_roboos"
# Optional audited scene override: an explicit runtime profile replaces the legacy
# derived scene generator. Both paths must be provided together.
scene_params=()
if [[ -n "${OPENFLEX_SIM_SCENE_MODEL:-}" ]]; then
    [[ -n "${OPENFLEX_SIM_SCENE_CONFIG:-}" ]] || { echo "OPENFLEX_SIM_SCENE_MODEL requires OPENFLEX_SIM_SCENE_CONFIG" >&2; exit 1; }
    scene_params=(--params-file "${OPENFLEX_SIM_SCENE_CONFIG}" -p scene_model_path:="${OPENFLEX_SIM_SCENE_MODEL}")
    echo "Bridge scene override: ${OPENFLEX_SIM_SCENE_MODEL}"
fi
echo "Starting only the existing MuJoCo bridge in ROS domain ${ROS_DOMAIN_ID}."
echo "No mission, Nav2, MoveIt executor, oracle grasp planner or automatic client."
exec /usr/bin/python3 -c 'from openflex_thesis_mujoco.bridge_node import main; main()' \
    --ros-args \
    --params-file "${root}/openflex_ws/src/openflex_thesis/openflex_thesis_mujoco/config/mujoco.yaml" \
    -p enable_viewer:=false -p publish_rgbd:=true -p image_width:=320 -p image_height:=240 \
    -p use_sim_time:=false \
    "${scene_params[@]}" \
    -p derived_model_path:="${root}/local_dependencies/openflex_roboos/readonly_scene.xml" \
    -r /thesis/cmd_vel_selected:="${selected_velocity_topic}" \
    -r /thesis/place/control:="${place_topic}" \
    -r /thesis/object_memory:="${memory_topic}" \
    -r /thesis_sim/reset_nominal:=/openflex_readonly_disabled/reset_nominal \
    -r /thesis_sim/randomize:=/openflex_readonly_disabled/randomize \
    -r /thesis_sim/drop_object:=/openflex_readonly_disabled/drop_object \
    -r /lift_joint_controller/follow_joint_trajectory:="${lift_action}" \
    -r /left_joint_trajectory_controller/follow_joint_trajectory:=/openflex_readonly_disabled/left_arm \
    -r /right_joint_trajectory_controller/follow_joint_trajectory:="${right_arm_action}" \
    -r /left_gripper_controller/gripper_cmd:=/openflex_readonly_disabled/left_gripper \
    -r /right_gripper_controller/gripper_cmd:="${right_gripper_action}"
