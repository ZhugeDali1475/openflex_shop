#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${OPENFLEX_ENABLE_SIM_NAV:-0}" != "1" ]]; then
    echo "Explicit opt-in required: OPENFLEX_ENABLE_SIM_NAV=1 bash scripts/run_openflex_sim_navigation.sh" >&2
    exit 1
fi
export ROS_DOMAIN_ID=83
export OPENFLEX_ENABLE_SIM_NAV=1
export PYTHONUNBUFFERED=1
set +u
source /opt/ros/humble/setup.bash
source "${root}/openflex_ws/install_v2/local_setup.bash"
set -u
export PYTHONPATH="${root}/openflex_ws/src/openflex_thesis/openflex_roboos_bridge:${root}/openflex_ws/src/openflex_thesis/openflex_thesis_control:${root}/openflex_ws/src/openflex_thesis/openflex_thesis_mujoco:${PYTHONPATH:-}"
runtime="${root}/local_dependencies/openflex_roboos"
mkdir -p "${runtime}"
exec 9>"${runtime}/navigation.lock"
flock -n 9 || { echo "Another OpenFlex navigation launcher holds the lock" >&2; exit 1; }
children=()
cleanup() {
    trap - EXIT INT TERM
    for child in "${children[@]}"; do kill -INT "${child}" 2>/dev/null || true; done
    local deadline=$((SECONDS + 8))
    while (( SECONDS < deadline )); do
        local alive=0
        for child in "${children[@]}"; do kill -0 "${child}" 2>/dev/null && alive=1; done
        (( alive == 0 )) && break
        sleep 0.1
    done
    for child in "${children[@]}"; do kill -TERM "${child}" 2>/dev/null || true; done
    for child in "${children[@]}"; do wait "${child}" 2>/dev/null || true; done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
bash "${root}/scripts/run_openflex_readonly_sim.sh" &
children+=("$!")
/usr/bin/python3 -c 'from openflex_thesis_control.velocity_arbiter_node import main; main()' \
    --ros-args --params-file "${root}/openflex_ws/src/openflex_thesis/openflex_thesis_control/config/control.yaml" \
    -p use_sim_time:=true &
children+=("$!")
/usr/bin/python3 -c 'from openflex_thesis_mujoco.sim_navigation_node import main; main()' \
    --ros-args -p use_sim_time:=true -p max_linear_mps:=0.15 -p max_angular_rps:=0.25 \
    -p position_tolerance_m:=0.008 -p yaw_tolerance_rad:=0.01 &
children+=("$!")
sleep 4
/usr/bin/python3 -m openflex_roboos_bridge.navigation_node --ros-args -p use_sim_time:=true \
    -p scene_config:="${root}/openflex_ws/src/openflex_thesis/openflex_thesis_mujoco/config/mujoco.yaml" \
    -p scene_model:="${runtime}/readonly_scene.xml" \
    -p journal:="${runtime}/navigation_operations.jsonl" &
children+=("$!")
if [[ "${OPENFLEX_ENABLE_SIM_PLACE:-0}" == "1" ]]; then
    echo 'Isolated simulation navigation/grasp/actual-pose placement gateway: 127.0.0.1:18553. No legacy scheduler or scene mutation.'
elif [[ "${OPENFLEX_ENABLE_SIM_MANIPULATION:-0}" == "1" ]]; then
    echo 'Simulation shared navigation/grasp gateway: 127.0.0.1:18553. No place or old mission scheduler.'
else
    echo 'Simulation navigation gateway: 127.0.0.1:18553. No grasp, place or old mission scheduler.'
fi
wait -n "${children[@]}"
