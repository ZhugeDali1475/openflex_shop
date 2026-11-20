#!/usr/bin/env bash
# Reproducible base-motion audit: simulated MID-360 -> FAST-LIO2 vs ground truth.
#
# Starts the simulation navigation profile with the simulated MID-360 lidar,
# runs fastlio2/lio_node on /livox/lidar + /livox/imu, then drives the base only
# through the audited velocity arbiter (NAV2 mode) with a short scripted pattern
# and compares /lio_odom against the MuJoCo ground-truth odometry.  Motion is
# base-only, low speed and bounded; the arm, lift and gripper are never
# commanded.  Requires the LIO overlay (see scripts/run_openflex_lio_sim.sh).
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
nav_log="${log_dir}/lio_motion_nav.log"
lio_log="${log_dir}/lio_motion_node.log"

nav_pid=""
lio_pid=""
stop_job() {
    # A bash script launched with ``&`` from a non-interactive shell inherits
    # SIGINT as ignored, so a plain SIGINT never reaches its trap.  Escalate to
    # SIGTERM (honoured by the navigation launcher's TERM trap, which then stops
    # its own children cleanly).
    local pid="$1"
    [[ -z "${pid}" ]] && return 0
    kill -INT "${pid}" 2>/dev/null || true
    sleep 1
    kill -TERM "${pid}" 2>/dev/null || true
}
cleanup() {
    pkill -INT -f "fastlio2/lib/fastlio2/lio_node" 2>/dev/null || true
    stop_job "${lio_pid}"
    stop_job "${nav_pid}"
    [[ -n "${lio_pid}" ]] && wait "${lio_pid}" 2>/dev/null || true
    [[ -n "${nav_pid}" ]] && wait "${nav_pid}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

OPENFLEX_ENABLE_SIM_NAV=1 OPENFLEX_ENABLE_SIM_LIDAR=1 \
    bash "${root}/scripts/run_openflex_sim_navigation.sh" > "${nav_log}" 2>&1 &
nav_pid=$!
echo "Started simulation navigation profile with simulated MID-360 (pid ${nav_pid}); log ${nav_log}"

deadline=$((SECONDS + 180))
until grep -q "loaded derived model" "${nav_log}" 2>/dev/null; do
    if (( SECONDS >= deadline )); then
        echo "timed out waiting for the bridge model (see ${nav_log})" >&2
        exit 1
    fi
    sleep 1
done
until ros2 topic list 2>/dev/null | grep -qx "/livox/lidar"; do
    (( SECONDS < deadline )) || { echo "timed out waiting for /livox/lidar" >&2; exit 1; }
    sleep 1
done
until ros2 service list 2>/dev/null | grep -qx "/thesis/set_control_mode"; do
    (( SECONDS < deadline )) || { echo "timed out waiting for the velocity arbiter" >&2; exit 1; }
    sleep 1
done
echo "/livox/lidar and the velocity arbiter are up; starting fastlio2/lio_node"

lio_exe="${root}/openflex_ws/install_lio/fastlio2/lib/fastlio2/lio_node"
if [[ -x "${lio_exe}" ]]; then
    "${lio_exe}" --ros-args -p config_path:="${config}" > "${lio_log}" 2>&1 &
else
    ros2 run fastlio2 lio_node --ros-args -p config_path:="${config}" > "${lio_log}" 2>&1 &
fi
lio_pid=$!
echo "Started lio_node (pid ${lio_pid}); log ${lio_log}"
echo "Waiting ${LIO_SIM_WARMUP_SECONDS:-6}s for IMU initialisation before driving."
sleep "${LIO_SIM_WARMUP_SECONDS:-6}"

set +e
python3 "${root}/scripts/audit_openflex_lio_motion.py"
audit_rc=$?
set -e
if grep -q "MUJOCO_SAFETY_STOP" "${nav_log}"; then
    echo "MUJOCO safety stop detected during the motion batch (see ${nav_log})" >&2
    audit_rc=1
fi
exit "${audit_rc}"
