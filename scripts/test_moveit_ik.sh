#!/usr/bin/env bash
set -eo pipefail

workspace="/home/wgb/openflex_shop/openflex_ws"

set +u
source /opt/ros/humble/setup.bash
source "${workspace}/install_v2/setup.bash"
set -u

exec python3 /home/wgb/openflex_shop/scripts/test_moveit_ik.py "$@"
