#!/usr/bin/env bash
set -eo pipefail

workspace="/home/wgb/openflex_shop/openflex_ws"

set +u
source /opt/ros/humble/setup.bash
source "${workspace}/install_v2/setup.bash"
set -u

exec ros2 launch openarmx_integrated_moveit_config sim_moveit.launch.py \
  use_rviz:=false \
  require_controllers:=false \
  start_command_bridge:=false \
  start_base_navigation_bridge:=false \
  start_pick_task_executor:=false \
  start_joint_posture_executor:=false \
  "$@"
