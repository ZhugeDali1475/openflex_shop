#!/usr/bin/env bash
set -eo pipefail

ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
WS_SETUP="${WS_SETUP:-${HOME}/openflex_all/openflex_ws/install/setup.bash}"

SERIAL_PORT="${SERIAL_PORT:-/dev/ttyACM0}"
SERIAL_PORT_CANDIDATES="${SERIAL_PORT_CANDIDATES:-/dev/ttyACM0,/dev/ttyUSB0,/dev/ttyACM1,/dev/ttyUSB1,/dev/ttyACM2,/dev/ttyUSB2,/dev/ttyACM3,/dev/ttyUSB3}"
SLAVE_ID="${SLAVE_ID:-3}"

POLL_HZ="${POLL_HZ:-5.0}"
TIMEOUT_SEC="${TIMEOUT_SEC:-0.2}"
READ_RETRIES="${READ_RETRIES:-0}"
OFFLINE_AFTER_FAILURES="${OFFLINE_AFTER_FAILURES:-1}"
BATTERY_STATE_TOPIC="${BATTERY_STATE_TOPIC:-/battery_state}"

source "${ROS_SETUP}"
source "${WS_SETUP}"

exec ros2 launch openarmx_battery_monitor single_battery_overlay.launch.py \
  start_rviz:=false \
  serial_port:="${SERIAL_PORT}" \
  serial_port_candidates:="${SERIAL_PORT_CANDIDATES}" \
  slave_id:="${SLAVE_ID}" \
  poll_hz:="${POLL_HZ}" \
  timeout_sec:="${TIMEOUT_SEC}" \
  read_retries:="${READ_RETRIES}" \
  offline_after_failures:="${OFFLINE_AFTER_FAILURES}" \
  battery_state_topic:="${BATTERY_STATE_TOPIC}"
