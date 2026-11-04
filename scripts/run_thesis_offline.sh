#!/usr/bin/env bash
set -eo pipefail

workspace="/home/wgb/openflex_shop/openflex_ws"

set +u
source /opt/ros/humble/setup.bash
source "${workspace}/install_v2/setup.bash"
set -u

export OPENFLEX_ROOT="/home/wgb/openflex_shop"
export OPENFLEX_WORKSPACE="${workspace}"
export OPENFLEX_MODEL_ROOT="${OPENFLEX_ROOT}/openflex_models"
export OPENFLEX_MUJOCO_MODEL="${OPENFLEX_ROOT}/openflex_mujoco-1.0_basic/openflex_mujoco.xml"
export MUJOCO_GL="${MUJOCO_GL:-egl}"

exec ros2 launch openflex_thesis_mujoco thesis_mujoco.launch.py \
  enable_viewer:=false \
  start_moveit:=false \
  start_moveit_executor:=false \
  use_moveit_evaluator:=false \
  start_scripted_policy:=true \
  start_sim_navigation:=true \
  run_mission:=false \
  start_auto_compute:=true \
  "$@"
