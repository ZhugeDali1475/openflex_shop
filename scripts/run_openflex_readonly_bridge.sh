#!/usr/bin/env bash
set -eo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python="${root}/.venvs/openflex-mcp/bin/python"
if [[ ! -x "${python}" ]]; then
    echo "Run python3 scripts/setup_openflex_readonly.py first" >&2
    exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source "${root}/openflex_ws/install_v2/local_setup.bash"
set -u
export PYTHONPATH="${root}/openflex_ws/src/openflex_thesis/openflex_roboos_bridge:${PYTHONPATH:-}"
export PYTHONUNBUFFERED=1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-83}"
snapshot_pid=""
mcp_pid=""
cleanup() {
    trap - EXIT INT TERM
    [[ -z "${mcp_pid}" ]] || kill "${mcp_pid}" 2>/dev/null || true
    [[ -z "${snapshot_pid}" ]] || kill -INT "${snapshot_pid}" 2>/dev/null || true
    [[ -z "${mcp_pid}" ]] || wait "${mcp_pid}" 2>/dev/null || true
    [[ -z "${snapshot_pid}" ]] || wait "${snapshot_pid}" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

/usr/bin/python3 -s -m openflex_roboos_bridge.ros_snapshot --ros-args -p use_sim_time:=true \
    -p scene_config:="${OPENFLEX_SIM_SCENE_CONFIG:-${root}/openflex_ws/src/openflex_thesis/openflex_thesis_mujoco/config/mujoco.yaml}" \
    -p scene_model:="${OPENFLEX_SIM_SCENE_MODEL:-${root}/local_dependencies/openflex_roboos/readonly_scene.xml}" &
snapshot_pid=$!
env -u ALL_PROXY -u all_proxy -u HTTP_PROXY -u http_proxy -u HTTPS_PROXY -u https_proxy \
    "${python}" -I "${root}/scripts/openflex_mcp_entry.py" &
mcp_pid=$!
echo "Read-only MCP: http://127.0.0.1:18551/mcp; ROS snapshots: 127.0.0.1:18552"
echo "Only get_robot_state / observe_scene; no simulation, missions or motion started."
wait -n "${snapshot_pid}" "${mcp_pid}"
