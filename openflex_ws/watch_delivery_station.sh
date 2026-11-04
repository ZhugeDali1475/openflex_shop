#!/usr/bin/env bash
# Live monitor for the delivery-station low-support fix (Layer A + B).
#
# Tails /rosout and prints only the lines that decide whether the fix worked,
# tagged and timestamped, to both the terminal and a log file.  Run this in its
# own terminal alongside the GUI's own bag recording.
#
# Usage:  ./watch_delivery_station.sh [run_label]

set -uo pipefail

LABEL="${1:-run}"
OUT_DIR="${NAV_RECORD_ROOT:-$HOME/openflex_nav_records}/guard_watch"
mkdir -p "$OUT_DIR"
LOG="$OUT_DIR/$(date +%Y%m%d_%H%M%S)_${LABEL}.log"

source /opt/ros/humble/setup.bash
source /home/openflex/openflex_shop/openflex_ws/install/setup.bash

echo "logging to $LOG"
echo "watching for guard decisions -- Ctrl-C to stop"
echo

{
  echo "# delivery-station guard watch"
  echo "# label: $LABEL"
  echo "# started: $(date --iso-8601=seconds)"
  echo "# expect: 'weak quality hold' during the turn (Layer A),"
  echo "#         'Low-support ICP accepted' after it stops (Layer B),"
  echo "#         and NO 'NOT READY' / no 3 consecutive 'quality rejected'."
  echo
} | tee "$LOG"

stdbuf -oL ros2 topic echo /rosout --field msg 2>/dev/null |
  stdbuf -oL grep --line-buffered -E \
    'Low-support ICP accepted|weak quality|quality rejected|Continuous ICP quality|NOT READY|Localization HEALTHY|latched|LIO yaw jump|divergence' |
  while IFS= read -r line; do
    ts="$(date +%H:%M:%S.%3N)"
    case "$line" in
      *"Low-support ICP accepted"*)   tag="[B-ACCEPT ]" ;;
      *"weak quality"*)               tag="[A-HOLD   ]" ;;
      *"quality rejected"*)           tag="[REJECT !!]" ;;
      *"NOT READY"*)                  tag="[LOST  !!!]" ;;
      *"Localization HEALTHY"*)       tag="[HEALTHY  ]" ;;
      *latched*)                      tag="[CMD-ZERO ]" ;;
      *"LIO yaw jump"*)               tag="[LIO-JUMP ]" ;;
      *divergence*)                   tag="[DIVERGE  ]" ;;
      *"accepted=0"*)                 tag="[FRAME-NG ]" ;;
      *)                              tag="[frame    ]" ;;
    esac
    printf '%s %s %s\n' "$ts" "$tag" "$line" | tee -a "$LOG"
  done
