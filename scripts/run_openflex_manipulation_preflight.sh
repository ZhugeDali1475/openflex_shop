#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/humble/setup.bash
source "${root}/openflex_ws/install_v2/local_setup.bash"
set -u
export ROS_DOMAIN_ID=83
export PYTHONPATH="${root}/openflex_ws/src/openflex_thesis/openflex_tc_wam:${root}/openflex_ws/src/openflex_thesis/openflex_thesis_mujoco:${PYTHONPATH:-}"
echo "Planning queries only: trajectory execution disabled; no grasp executor or automatic mission."
exec ros2 launch "${root}/scripts/openflex_manipulation_preflight.launch.py"
