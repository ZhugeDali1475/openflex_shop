#!/usr/bin/env bash

set -Eeuo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_PROFILE="$WORKSPACE/src/openflex_chassis/navigation_layer/swerve_navigation/config/supermarket_test_09_competition.yaml"
DEFAULT_STATIONS_TEST09="$WORKSPACE/src/openflex_integrated/openarmx_task_executor/config/stations_supermarket_test_09.yaml"
DEFAULT_STATIONS_TEST08="$WORKSPACE/src/openflex_integrated/openarmx_task_executor/config/stations_supermarket_test_08.yaml"
DEFAULT_STATIONS="${DEFAULT_STATIONS_TEST09}"
[[ -f "$DEFAULT_STATIONS" ]] || DEFAULT_STATIONS="$DEFAULT_STATIONS_TEST08"
DEFAULT_SCENE_DB="$WORKSPACE/../supermarket_scene/shelf_inventory.db"
SCENE_ROOT="$(readlink -f "$WORKSPACE/../supermarket_scene")"
QTWEB_ROOT="$(readlink -f "$WORKSPACE/../local_dependencies/qtwebengine/root")"
PROFILE="${OPENFLEX_COMPETITION_PROFILE:-$DEFAULT_PROFILE}"
STATION_CONFIG="${OPENFLEX_STATION_CONFIG:-$DEFAULT_STATIONS}"
SCENE_DB="${SUPERMARKET_SCENE_DB_PATH:-$DEFAULT_SCENE_DB}"
RECORD_ROOT="${NAV_RECORD_ROOT:-$HOME/openflex_nav_records/fullbody_competition}"
CHECK_ONLY=false
START_SCENE_VISION=false
ROS_ARGS=()

# PickIK is a runtime MoveIt plugin rather than a build-time dependency.  The
# competition console starts ROS in a fresh terminal, so it must explicitly
# source the plugin overlay before sourcing this workspace.  The first two
# paths are the supported deployments; the final path preserves the currently
# installed handoff copy as a last-resort recovery source.
PICK_IK_SETUP=""
for candidate in \
    "${OPENFLEX_PICK_IK_SETUP:-}" \
    "$WORKSPACE/local_dependencies/pick_ik_1.1.2/opt/ros/humble/share/pick_ik/local_setup.bash" \
    "$WORKSPACE/../local_dependencies/pick_ik_1.1.2/opt/ros/humble/share/pick_ik/local_setup.bash" \
    "$HOME/openflex_all/local_dependencies/pick_ik_1.1.2/opt/ros/humble/share/pick_ik/local_setup.bash" \
    "$HOME/handeye_handoff_20260806/payload/home/openarmx/openflex_all/local_dependencies/pick_ik_1.1.2/opt/ros/humble/share/pick_ik/local_setup.bash"; do
    if [[ -n "$candidate" && -r "$candidate" ]]; then
        PICK_IK_SETUP="$candidate"
        break
    fi
done

usage() {
    cat <<EOF
Usage: $(basename "$0") [options] [-- ROS arguments]

Options:
  --profile PATH         Competition navigation profile
  --station-config PATH  Team semantic station database
  --record-root PATH     Navigation record directory
  --scene-db PATH        Supermarket scene SQLite database
  --start-scene-vision   Explicitly start the optional OWLv2/DINO scene vision node
  --no-scene-vision      Do not start the scene vision node
  --check                Validate files and installed launch packages only
  -h, --help             Show this help
EOF
}

while (($#)); do
    case "$1" in
        --profile)
            [[ $# -ge 2 ]] || { echo "--profile requires a path" >&2; exit 2; }
            PROFILE="$2"
            shift 2
            ;;
        --station-config)
            [[ $# -ge 2 ]] || { echo "--station-config requires a path" >&2; exit 2; }
            STATION_CONFIG="$2"
            shift 2
            ;;
        --record-root)
            [[ $# -ge 2 ]] || { echo "--record-root requires a path" >&2; exit 2; }
            RECORD_ROOT="$2"
            shift 2
            ;;
        --scene-db)
            [[ $# -ge 2 ]] || { echo "--scene-db requires a path" >&2; exit 2; }
            SCENE_DB="$2"
            shift 2
            ;;
        --start-scene-vision)
            START_SCENE_VISION=true
            shift
            ;;
        --no-scene-vision)
            START_SCENE_VISION=false
            shift
            ;;
        --check)
            CHECK_ONLY=true
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        --)
            shift
            ROS_ARGS=("$@")
            break
            ;;
        *)
            echo "Unknown option: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

[[ -r /opt/ros/humble/setup.bash ]] || {
    echo "ROS 2 Humble setup was not found: /opt/ros/humble/setup.bash" >&2
    exit 1
}
[[ -r "$WORKSPACE/install/setup.bash" ]] || {
    echo "Workspace is not built: $WORKSPACE/install/setup.bash" >&2
    exit 1
}
[[ -n "$PICK_IK_SETUP" ]] || {
    echo "PickIK runtime overlay was not found. Set OPENFLEX_PICK_IK_SETUP to pick_ik/local_setup.bash." >&2
    exit 1
}

set +u
source /opt/ros/humble/setup.bash
source "$PICK_IK_SETUP"
source "$WORKSPACE/install/setup.bash"
set -u

ros2 pkg prefix pick_ik >/dev/null 2>&1 || {
    echo "PickIK was sourced but is not visible to ROS 2: $PICK_IK_SETUP" >&2
    exit 1
}
echo "PickIK runtime overlay: $PICK_IK_SETUP"

if [[ -d "$QTWEB_ROOT" ]]; then
    QTWEB_PYTHON="$QTWEB_ROOT/usr/bin/python3"
    export PYTHONPATH="$QTWEB_ROOT/usr/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
    export LD_LIBRARY_PATH="$QTWEB_ROOT/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    export QTWEBENGINEPROCESS_PATH="$QTWEB_ROOT/usr/lib/x86_64-linux-gnu/qt5/libexec/QtWebEngineProcess"
else
    QTWEB_PYTHON="/usr/bin/python3"
fi

PROFILE="$(readlink -f "$PROFILE")"
STATION_CONFIG="$(readlink -f "$STATION_CONFIG")"
SCENE_DB="$(readlink -f "$SCENE_DB")"
RECORD_ROOT="$(readlink -m "$RECORD_ROOT")"
export OPENFLEX_COMPETITION_PROFILE="$PROFILE"
export OPENFLEX_STATION_CONFIG="$STATION_CONFIG"
export OPENFLEX_COMPETITION_LAUNCH_PACKAGE="openarmx_task_executor"
export OPENFLEX_COMPETITION_LAUNCH_FILE="supermarket_full_system.launch.py"
export OPENFLEX_COMPETITION_RUNTIME_LAUNCH_PACKAGE="openarmx_task_executor"
export OPENFLEX_COMPETITION_RUNTIME_LAUNCH_FILE="supermarket_fullbody_runtime.launch.py"
export OPENFLEX_COMPETITION_NAV_LAUNCH_PACKAGE="swerve_navigation"
export OPENFLEX_COMPETITION_NAV_LAUNCH_FILE="supermarket_test_05_benchmark.launch.py"
export OPENFLEX_FULLBODY_COMPETITION=1
export OPENFLEX_WORKSPACE_ROOT="$WORKSPACE"
export SUPERMARKET_SCENE_DB_PATH="$SCENE_DB"
export NAV_RECORD_ROOT="$RECORD_ROOT"

[[ -f "$SCENE_DB" && -r "$SCENE_DB" && -w "$SCENE_DB" ]] || {
    echo "Scene database must exist and be readable/writable: $SCENE_DB" >&2
    exit 1
}
"$QTWEB_PYTHON" - <<'PY'
from PyQt5.QtWebChannel import QWebChannel
from PyQt5.QtWebEngineWidgets import QWebEngineView
from supermarket_scene_interfaces.srv import (
    ApplySceneLayout,
    FindSku,
    GetSceneSnapshot,
    InspectShelf,
    UpsertSkusBatch,
)
assert all((QWebChannel, QWebEngineView, ApplySceneLayout, FindSku, GetSceneSnapshot, InspectShelf, UpsertSkusBatch))
PY

python3 - "$PROFILE" "$STATION_CONFIG" <<'PY'
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from openarmx_task_executor.station_db import StationDatabase
from openarmx_task_msgs.action import PickObject, RetreatFromStation, ShelfTraverse
from openflex_navigation_gui.benchmark import load_benchmark_profile
from openflex_navigation_gui.competition_mission import load_mission, mission_path
from openflex_navigation_gui.model import resolve_map_paths

profile_path = Path(sys.argv[1])
station_config = Path(sys.argv[2])
if not profile_path.is_file():
    raise SystemExit(f"Competition profile was not found: {profile_path}")
if not station_config.is_file():
    raise SystemExit(f"Station database was not found: {station_config}")

profile = load_benchmark_profile(profile_path)
paths = resolve_map_paths(profile.map_root, profile.map_name)
missing = [path for path in (paths.pcd, paths.yaml, paths.pgm) if not path.is_file()]
if missing:
    raise SystemExit("Missing competition map files:\n  " + "\n  ".join(map(str, missing)))

launch_path = (
    Path(get_package_share_directory("openarmx_task_executor"))
    / "launch"
    / "supermarket_full_system.launch.py"
)
runtime_launch_path = (
    Path(get_package_share_directory("openarmx_task_executor"))
    / "launch"
    / "supermarket_fullbody_runtime.launch.py"
)
navigation_launch_path = (
    Path(get_package_share_directory("swerve_navigation"))
    / "launch"
    / "supermarket_test_05_benchmark.launch.py"
)
missing_launches = [
    path
    for path in (launch_path, runtime_launch_path, navigation_launch_path)
    if not path.is_file()
]
if missing_launches:
    raise SystemExit(
        "Competition launch files are not installed. Build the affected packages first:\n  "
        + "\n  ".join(map(str, missing_launches))
    )

# Importing these generated classes is part of the preflight: an old build
# must not open a GUI whose mock-pick/retreat buttons can never connect.
assert PickObject is not None and RetreatFromStation is not None and ShelfTraverse is not None
mission_file = mission_path(paths)
mission = load_mission(mission_file, paths.name) if mission_file.is_file() else []
stations = StationDatabase.from_yaml(
    str(station_config),
    str(mission_file) if mission_file.is_file() else "",
)
retreat_stations = [
    station_id
    for station_id, station in stations.items()
    if station.retreat_after_pick or station.retreat_after_place
]
shelf_a = stations.get("SHELF-A")
if shelf_a is None:
    raise SystemExit("Test-9 station database is missing SHELF-A")
for level in (2, 3):
    if stations.level_lift_position("SHELF-A", None, level) is None:
        raise SystemExit(f"SHELF-A is missing calibrated lift level {level}")
    if len(stations.level_arm_joint_positions("SHELF-A", level, "right")) != 7:
        raise SystemExit(f"SHELF-A is missing the seven-joint right-arm level {level} pose")

print(f"Profile: {profile.profile_id} ({profile_path})")
print(f"Map: {profile.map_name}")
print(f"Station database: {station_config}")
print("Runtime owner: independent full-body bringup (one controller_manager)")
print("Mapping/navigation: separate workflows reusing the full-body runtime")
print("Navigation owner: Nav2 (one /navigate_to_pose server)")
print("Velocity chain: /cmd_vel -> collision monitor -> localization gate -> /cmd_vel_safe")
print("Mock grasp endpoint: /competition/mock_pick_object (no hardware motion)")
print("Retreat endpoint: /retreat_from_station")
print("Shelf lateral traverse endpoint: /return_to_shelf_anchor")
print("Retreat-enabled stations: " + (", ".join(retreat_stations) or "none"))
if len(mission) < 2:
    print(
        "NOTICE: the selected map currently has fewer than two mission stations; "
        "save a second physically verified safe station in the GUI before running "
        "the mock-pick -> retreat -> next-navigation chain."
    )
PY

echo "Record root: $RECORD_ROOT"
echo "Scene database: $SCENE_DB"

if [[ "$CHECK_ONLY" == true ]]; then
    echo "Full-body competition GUI preflight passed."
    exit 0
fi

mkdir -p "$RECORD_ROOT"

# The GUI owns the process handles for the full-body/navigation launch.  Two
# GUI instances therefore cannot safely share one robot: a Stop button can
# only stop the launch started by that same window.  Keep the lock descriptor
# open across exec so exactly one competition console can run per login.
RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp}"
GUI_LOCK_FILE="$RUNTIME_DIR/openflex_supermarket_competition_gui.lock"
exec 9>"$GUI_LOCK_FILE"
if ! flock -n 9; then
    message="OpenFlex 商超比赛控制台已经在运行，请使用现有窗口。"
    echo "$message" >&2
    if command -v notify-send >/dev/null 2>&1; then
        notify-send "OpenFlex 商超比赛控制台" "$message" || true
    fi
    exit 3
fi

if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
    echo "No DISPLAY or WAYLAND_DISPLAY is set; the Qt GUI may not open." >&2
fi

SCENE_DATA_PID=""
SCENE_VISION_PID=""
GUI_PID=""
stop_owned_group() {
    local pid="$1"
    [[ -n "$pid" ]] || return 0
    local signal
    local attempts
    for signal in INT TERM KILL; do
        kill -0 -- "-$pid" 2>/dev/null || break
        kill -"$signal" -- "-$pid" 2>/dev/null || true
        attempts=20
        [[ "$signal" == INT ]] && attempts=40
        for _attempt in $(seq 1 "$attempts"); do
            kill -0 -- "-$pid" 2>/dev/null || break
            sleep 0.1
        done
    done
    wait "$pid" 2>/dev/null || true
}
service_available() {
    local service_name="$1"
    local services
    services="$(ros2 service list --no-daemon --spin-time 0.4 2>/dev/null || true)"
    # Some older ros2cli builds do not understand --no-daemon.  Keep the
    # preflight useful on those images instead of treating every service as
    # absent and silently starting a duplicate stack.
    if [[ -z "$services" ]]; then
        services="$(ros2 service list --spin-time 0.4 2>/dev/null || true)"
    fi
    grep -Fxq "$service_name" <<<"$services"
}
node_available() {
    local node_name="$1"
    local nodes
    nodes="$(ros2 node list --no-daemon --spin-time 0.4 2>/dev/null || true)"
    if [[ -z "$nodes" ]]; then
        nodes="$(ros2 node list --spin-time 0.4 2>/dev/null || true)"
    fi
    grep -Fxq "$node_name" <<<"$nodes"
}
assert_runtime_stack_absent() {
    local running_nodes
    local running_processes
    local stale_processes
    local node_name
    local -a owned_nodes=(
        /move_group
        /moveit_command_bridge
        /openflex_pick_task_executor
        /openflex_detect_object
        /openflex_pick_object
        /openflex_graspnet_worker
        /openflex_pick_sequence
        /openflex_grasp_readonly_web
        /cartesian_approach_executor
        /openflex_joint_posture_executor
        /openarmx_competition_coordinator
        /shopping_ticket_reader
    )

    running_nodes="$(ros2 node list --no-daemon --spin-time 0.6 2>/dev/null || true)"
    if [[ -z "$running_nodes" ]]; then
        running_nodes="$(ros2 node list --spin-time 0.6 2>/dev/null || true)"
    fi
    for node_name in "${owned_nodes[@]}"; do
        if grep -Fxq "$node_name" <<<"$running_nodes"; then
            echo "Refusing to start a second robot execution stack." >&2
            echo "ROS node is already running: $node_name" >&2
            echo "Stop the manually launched MoveIt/runtime stack, then start this console again." >&2
            return 1
        fi
    done

    # ROS graph discovery can miss orphaned nodes when an old launch parent
    # has exited or ros2cli is using a stale daemon/domain. Check the host
    # process table as a second source of truth before opening the console.
    running_processes="$(ps -eo pid=,args=)"
    stale_processes="$(grep -E \
        'controller_manager/ros2_control_node|moveit_ros_move_group/move_group|openflex_grasp\.(detect_object_server|graspnet_worker|pick_object_server|pick_sequence_server|grasp_web_visualizer)|openarmx_task_executor/lib/openarmx_task_executor/(task_executor|station_docking|shelf_fine_align|station_retreat|shopping_ticket_reader|shelf_view|competition_coordinator)|openarmx_integrated_moveit_config/lib/openarmx_integrated_moveit_config/(wait_for_robot_state\.py|ensure_controllers_ready\.py|switch_lift_controller\.py|moveit_command_bridge\.py|base_navigation_bridge\.py|openflex_pick_task_executor|openflex_joint_posture_executor)|openflex_grasp_moveit/lib/openflex_grasp_moveit/cartesian_approach_executor|openarmx_gravity_comp/lib/openarmx_gravity_comp/gravity_comp_node' \
        <<<"$running_processes" || true)"
    if [[ -n "$stale_processes" ]]; then
        echo "Refusing to start while an old robot execution process remains." >&2
        echo "$stale_processes" >&2
        echo "Stop the orphaned full-body/grasp process group, then start this console again." >&2
        return 1
    fi
}
start_scene_vision() {
    if [[ -x "$SCENE_ROOT/.venv/bin/python" ]]; then
        echo "Starting scene vision with project Python environment."
        setsid bash -c 'project_root="$1"; python="$2"; shift 2; cd "$project_root" && exec "$python" -m supermarket_scene_ros.vision_node "$@"' \
            _ "$SCENE_ROOT" "$SCENE_ROOT/.venv/bin/python" --ros-args -p db_path:="$SCENE_DB" 9>&- &
    else
        echo "Starting scene vision with ROS system Python (project environment unavailable)."
        setsid ros2 run supermarket_scene_ros supermarket_scene_vision_node \
            --ros-args -p db_path:="$SCENE_DB" 9>&- &
    fi
    SCENE_VISION_PID=$!
}
cleanup() {
    trap - EXIT HUP INT TERM
    if [[ -n "$GUI_PID" ]]; then
        stop_owned_group "$GUI_PID"
    fi
    if [[ -n "$SCENE_VISION_PID" ]]; then
        stop_owned_group "$SCENE_VISION_PID"
    fi
    if [[ -n "$SCENE_DATA_PID" ]]; then
        stop_owned_group "$SCENE_DATA_PID"
    fi
    exec 9>&-
}
handle_signal() {
    local status="$1"
    cleanup
    exit "$status"
}
trap cleanup EXIT
trap 'handle_signal 129' HUP
trap 'handle_signal 130' INT
trap 'handle_signal 143' TERM

# This console launches and owns MoveIt plus the command bridge.  Two stacks
# with the same action names make one trajectory reach multiple servers: one
# can accept while the other rejects it, producing code -4 and repeated
# gripper commands.  Fail before opening the GUI instead of sharing hardware.
assert_runtime_stack_absent

if node_available '/supermarket_scene_data_node'; then
    if service_available '/get_scene_snapshot' \
        && service_available '/upsert_skus_batch'; then
        echo "Reusing compatible supermarket_scene_data_node."
    else
        echo "A stale supermarket_scene_data_node is running with an incompatible interface." >&2
        echo "Stop that data-only node, then start the competition console again." >&2
        exit 1
    fi
else
    setsid ros2 run supermarket_scene_ros supermarket_scene_data_node \
        --ros-args -p db_path:="$SCENE_DB" 9>&- &
    SCENE_DATA_PID=$!
    for _attempt in $(seq 1 40); do
        if node_available '/supermarket_scene_data_node' \
            && service_available '/get_scene_snapshot' \
            && service_available '/upsert_skus_batch'; then
            break
        fi
        if ! kill -0 "$SCENE_DATA_PID" 2>/dev/null; then
            wait "$SCENE_DATA_PID"
            exit 1
        fi
        sleep 0.25
    done
    node_available '/supermarket_scene_data_node' \
        && service_available '/get_scene_snapshot' \
        && service_available '/upsert_skus_batch' || {
        echo "supermarket_scene_data_node did not become ready" >&2
        exit 1
    }
fi

if [[ "$START_SCENE_VISION" == true ]]; then
    if node_available '/supermarket_scene_vision_node' \
        && service_available '/inspect_shelf' && service_available '/find_sku'; then
        echo "Reusing running supermarket_scene_vision_node."
    else
        start_scene_vision
        for _attempt in $(seq 1 12); do
            if ! kill -0 "$SCENE_VISION_PID" 2>/dev/null; then
                wait "$SCENE_VISION_PID"
                exit 1
            fi
            sleep 0.25
        done
        echo "supermarket_scene_vision_node is loading models in the background."
    fi
else
    echo "Scene vision node auto-start is disabled."
fi

setsid "$QTWEB_PYTHON" -m openflex_navigation_gui.competition_window "${ROS_ARGS[@]}" 9>&- &
GUI_PID=$!
set +e
wait "$GUI_PID"
GUI_STATUS=$?
set -e
exit "$GUI_STATUS"
