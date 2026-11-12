#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${root}/scripts/openflex_sim_resources.sh"
source /opt/ros/humble/setup.bash
source "${root}/openflex_ws/install_v2/local_setup.bash"
export ROS_DOMAIN_ID=83
export MUJOCO_GL=glfw
exec /usr/bin/python3 "${root}/scripts/view_openflex_live_sim.py"
