#!/usr/bin/env bash
set -eo pipefail

root="/home/wgb/openflex_shop"
workspace="${root}/openflex_ws"

set +u
source /opt/ros/humble/setup.bash
source "${workspace}/install_v2/setup.bash"
set -u

export OPENFLEX_ROOT="${root}"
export OPENFLEX_WORKSPACE="${workspace}"
export OPENFLEX_MODEL_ROOT="${root}/openflex_models"
export OPENFLEX_MUJOCO_MODEL="${root}/openflex_mujoco-1.0_basic/openflex_mujoco.xml"
export MUJOCO_GL="${MUJOCO_GL:-egl}"
export PYTHONUNBUFFERED=1

exec ros2 launch openflex_thesis_mujoco thesis_mujoco.launch.py \
  enable_viewer:=false \
  publish_rgbd:=false \
  start_moveit:=true \
  start_moveit_executor:=true \
  use_moveit_evaluator:=true \
  start_scripted_policy:=true \
  start_sim_navigation:=true \
  run_mission:=true \
  start_auto_compute:=false \
  auto_evaluate_robustness:=false \
  candidate_distance_step_m:=0.20 \
  candidate_lateral_step_m:=0.15 \
  "$@"

