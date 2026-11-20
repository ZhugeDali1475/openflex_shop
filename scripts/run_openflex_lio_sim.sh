#!/usr/bin/env bash
# Reproducible read-only smoke run: simulated MID-360 -> FAST-LIO2.
#
# Starts the existing read-only MuJoCo bridge with the simulated MID-360 lidar
# opt-in, then runs fastlio2/lio_node on /livox/lidar + /livox/imu with the
# simulation config, probes the result, and tears both processes down.  It never
# sends motion and never touches the ground-truth TF tree (the bridge owns
# map -> odom -> base_link; LIO runs with publish_tf: false).
#
# Requires the LIO overlay: build it once with
#   colcon build --packages-select livox_ros_driver2 fastlio2 \
#     --build-base build_lio --install-base install_lio \
#     --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=<livox_sdk2>
set -eo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/humble/setup.bash
source "${root}/openflex_ws/install_v2/local_setup.bash"
source "${root}/openflex_ws/install_lio/local_setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-83}"
[[ "${ROS_DOMAIN_ID}" == "83" ]] || { echo "LIO simulation requires ROS_DOMAIN_ID=83" >&2; exit 1; }

config="${LIO_SIM_CONFIG:-${root}/openflex_ws/src/openflex_thesis/openflex_thesis_mujoco/config/lio_sim_mujoco.yaml}"
[[ -f "${config}" ]] || { echo "missing simulation LIO config: ${config}" >&2; exit 1; }

log_dir="${root}/local_dependencies/openflex_roboos/runs"
mkdir -p "${log_dir}"
sim_log="${log_dir}/lio_sim_bridge.log"
lio_log="${log_dir}/lio_sim_node.log"

sim_pid=""
lio_pid=""
stop_job() {
    # A bash script launched with ``&`` from a non-interactive shell inherits
    # SIGINT as ignored, so a plain SIGINT never reaches its trap.  Escalate to
    # SIGTERM for robustness (the bridge itself is a python process whose rclpy
    # handler already reacts to SIGINT).
    local pid="$1"
    [[ -z "${pid}" ]] && return 0
    kill -INT "${pid}" 2>/dev/null || true
    sleep 1
    kill -TERM "${pid}" 2>/dev/null || true
}
cleanup() {
    stop_job "${lio_pid}"
    stop_job "${sim_pid}"
    # ``ros2 run`` (used as a fallback) forks the node, so make sure the real
    # executable is gone too before waiting on the tracked pids.
    pkill -INT -f "fastlio2/lib/fastlio2/lio_node" 2>/dev/null || true
    [[ -n "${lio_pid}" ]] && wait "${lio_pid}" 2>/dev/null || true
    [[ -n "${sim_pid}" ]] && wait "${sim_pid}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" OPENFLEX_ENABLE_SIM_LIDAR=1 \
    bash "${root}/scripts/run_openflex_readonly_sim.sh" > "${sim_log}" 2>&1 &
sim_pid=$!
echo "Started read-only MuJoCo bridge with simulated MID-360 (pid ${sim_pid}); log ${sim_log}"

# Wait for the bridge itself (not a stale ROS graph entry) before trusting the
# topic list, otherwise a cached /livox/lidar from a previous run starts LIO
# against a bridge that has not loaded its model yet.
deadline=$((SECONDS + 120))
until grep -q "loaded derived model" "${sim_log}" 2>/dev/null; do
    if (( SECONDS >= deadline )); then
        echo "timed out waiting for the bridge model (see ${sim_log})" >&2
        exit 1
    fi
    sleep 1
done
until ros2 topic list 2>/dev/null | grep -qx "/livox/lidar"; do
    if (( SECONDS >= deadline )); then
        echo "timed out waiting for /livox/lidar (see ${sim_log})" >&2
        exit 1
    fi
    sleep 1
done
echo "/livox/lidar is up; starting fastlio2/lio_node"

# Launch the installed executable directly so the tracked pid really is the node
# (``ros2 run`` would only track its own wrapper).  Fall back to ``ros2 run``.
lio_exe="${root}/openflex_ws/install_lio/fastlio2/lib/fastlio2/lio_node"
if [[ -x "${lio_exe}" ]]; then
    "${lio_exe}" --ros-args -p config_path:="${config}" > "${lio_log}" 2>&1 &
else
    ros2 run fastlio2 lio_node --ros-args -p config_path:="${config}" > "${lio_log}" 2>&1 &
fi
lio_pid=$!
echo "Started lio_node (pid ${lio_pid}); log ${lio_log}"
echo "Waiting ${LIO_SIM_WARMUP_SECONDS:-6}s for IMU initialisation before probing."
sleep "${LIO_SIM_WARMUP_SECONDS:-6}"

python3 "${root}/scripts/probe_openflex_lio_sim.py" --seconds "${LIO_SIM_SECONDS:-12}"
