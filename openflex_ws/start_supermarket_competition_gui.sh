#!/usr/bin/env bash

set -Eeuo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_PROFILE="$WORKSPACE/src/openflex_chassis/navigation_layer/swerve_navigation/config/supermarket_test_05_competition.yaml"
DEFAULT_SCENE_DB="$WORKSPACE/../supermarket_scene/shelf_inventory.db"
SCENE_ROOT="$(readlink -f "$WORKSPACE/../supermarket_scene")"
PROFILE="${OPENFLEX_COMPETITION_PROFILE:-$DEFAULT_PROFILE}"
SCENE_DB="${SUPERMARKET_SCENE_DB_PATH:-$DEFAULT_SCENE_DB}"
RECORD_ROOT="${NAV_RECORD_ROOT:-$HOME/openflex_nav_records/competition}"
CHECK_ONLY=false
START_SCENE_VISION=true
ROS_ARGS=()

usage() {
    cat <<EOF
Usage: $(basename "$0") [options] [-- ROS arguments]

Options:
  --profile PATH       Competition benchmark profile (default: supermarket_test_05)
  --record-root PATH   Navigation record directory
  --scene-db PATH      Supermarket scene SQLite database
  --start-scene-vision Start the scene vision node when it is not already running (default)
  --no-scene-vision    Do not start the scene vision node
  --check              Validate the workspace, profile, map, and launch files only
  -h, --help           Show this help
EOF
}

while (($#)); do
    case "$1" in
        --profile)
            [[ $# -ge 2 ]] || { echo "--profile requires a path" >&2; exit 2; }
            PROFILE="$2"
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

set +u
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
set -u

PROFILE="$(readlink -f "$PROFILE")"
SCENE_DB="$(readlink -f "$SCENE_DB")"
RECORD_ROOT="$(readlink -m "$RECORD_ROOT")"
export OPENFLEX_COMPETITION_PROFILE="$PROFILE"
export SUPERMARKET_SCENE_DB_PATH="$SCENE_DB"
export NAV_RECORD_ROOT="$RECORD_ROOT"

[[ -f "$SCENE_DB" && -r "$SCENE_DB" && -w "$SCENE_DB" ]] || {
    echo "Scene database must exist and be readable/writable: $SCENE_DB" >&2
    exit 1
}
python3 - <<'PY'
from supermarket_scene_interfaces.srv import (
    ApplySceneLayout,
    FindSku,
    GetSceneSnapshot,
    InspectShelf,
)
assert all((ApplySceneLayout, FindSku, GetSceneSnapshot, InspectShelf))
PY

python3 - "$PROFILE" <<'PY'
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from openflex_navigation_gui.benchmark import (
    find_waypoint_by_role,
    load_benchmark_profile,
)
from openflex_navigation_gui.model import resolve_map_paths

profile_path = Path(sys.argv[1])
if not profile_path.is_file():
    raise SystemExit(f"Competition profile was not found: {profile_path}")

profile = load_benchmark_profile(profile_path)
start = find_waypoint_by_role(profile, "start_position")

paths = resolve_map_paths(profile.map_root, profile.map_name)
missing = [path for path in (paths.pcd, paths.yaml, paths.pgm) if not path.is_file()]
if missing:
    raise SystemExit("Missing competition map files:\n  " + "\n  ".join(map(str, missing)))

launch_path = (
    Path(get_package_share_directory("swerve_navigation"))
    / "launch"
    / profile.launch_file
)
if not launch_path.is_file():
    raise SystemExit(f"Competition launch file was not found: {launch_path}")

print(f"Profile: {profile.profile_id} ({profile_path})")
print(f"Map: {profile.map_name}")
print(
    f"Localization seed: ({profile.initial_x:.3f}, {profile.initial_y:.3f}, "
    f"yaw={profile.initial_yaw:.3f})"
)
if start is None:
    print("NOTICE: no start_position waypoint; return-to-start will be disabled.")
else:
    print(
        f"Start waypoint: {start.waypoint_id} "
        f"({start.x:.3f}, {start.y:.3f}, yaw={start.yaw:.3f})"
    )
print(f"Targets: {', '.join(profile.waypoints)}")
PY

echo "Scene database: $SCENE_DB"

mkdir -p "$RECORD_ROOT"
echo "Record root: $RECORD_ROOT"

if [[ "$CHECK_ONLY" == true ]]; then
    echo "Competition GUI preflight passed."
    exit 0
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
    ros2 service list --no-daemon --spin-time 0.4 2>/dev/null \
        | grep -Fxq "$service_name"
}
node_available() {
    local node_name="$1"
    ros2 node list --no-daemon --spin-time 0.4 2>/dev/null \
        | grep -Fxq "$node_name"
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

if node_available '/supermarket_scene_data_node' \
    && service_available '/get_scene_snapshot'; then
    echo "Reusing running supermarket_scene_data_node."
else
    setsid ros2 run supermarket_scene_ros supermarket_scene_data_node \
        --ros-args -p db_path:="$SCENE_DB" 9>&- &
    SCENE_DATA_PID=$!
    for _attempt in $(seq 1 40); do
        if node_available '/supermarket_scene_data_node' \
            && service_available '/get_scene_snapshot'; then
            break
        fi
        if ! kill -0 "$SCENE_DATA_PID" 2>/dev/null; then
            wait "$SCENE_DATA_PID"
            exit 1
        fi
        sleep 0.25
    done
    node_available '/supermarket_scene_data_node' \
        && service_available '/get_scene_snapshot' || {
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

setsid ros2 run openflex_navigation_gui openflex_competition_navigation_gui "${ROS_ARGS[@]}" 9>&- &
GUI_PID=$!
set +e
wait "$GUI_PID"
GUI_STATUS=$?
set -e
exit "$GUI_STATUS"
