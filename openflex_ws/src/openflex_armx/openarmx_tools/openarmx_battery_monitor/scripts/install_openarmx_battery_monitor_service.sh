#!/usr/bin/env bash
set -euo pipefail

SERVICE_NAME="openarmx-battery-monitor.service"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_SHARE_DEFAULT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PACKAGE_SHARE="${PACKAGE_SHARE:-${PACKAGE_SHARE_DEFAULT}}"
DEFAULT_ENV_FILE="/etc/default/openarmx-battery-monitor"

SERVICE_SRC="${PACKAGE_SHARE}/systemd/openarmx-battery-monitor.service"
ENV_SRC="${PACKAGE_SHARE}/systemd/openarmx-battery-monitor.env"
SERVICE_DST="/etc/systemd/system/${SERVICE_NAME}"
WORKSPACE_DIR="${WORKSPACE_DIR:-${HOME}/openflex_all/openflex_ws}"
WS_SETUP="${WS_SETUP:-${WORKSPACE_DIR}/install/setup.bash}"
RUN_SCRIPT="${PACKAGE_SHARE}/scripts/run_battery_monitor_systemd.sh"

if [[ ! -f "${SERVICE_SRC}" ]]; then
  echo "Missing file: ${SERVICE_SRC}"
  exit 1
fi

if [[ ! -f "${ENV_SRC}" ]]; then
  echo "Missing file: ${ENV_SRC}"
  exit 1
fi

tmp_service="$(mktemp)"
sed \
  -e "s|@OPENFLEX_USER@|${USER}|g" \
  -e "s|@OPENFLEX_GROUP@|$(id -gn)|g" \
  -e "s|@OPENFLEX_WORKSPACE@|${WORKSPACE_DIR}|g" \
  -e "s|@OPENFLEX_RUN_SCRIPT@|${RUN_SCRIPT}|g" \
  "${SERVICE_SRC}" > "${tmp_service}"
sudo install -m 0644 "${tmp_service}" "${SERVICE_DST}"
rm -f "${tmp_service}"

if [[ ! -f "${DEFAULT_ENV_FILE}" ]]; then
  tmp_env="$(mktemp)"
  sed \
    -e "s|@OPENFLEX_WS_SETUP@|${WS_SETUP}|g" \
    "${ENV_SRC}" > "${tmp_env}"
  sudo install -m 0644 "${tmp_env}" "${DEFAULT_ENV_FILE}"
  rm -f "${tmp_env}"
  echo "Created ${DEFAULT_ENV_FILE}"
else
  echo "Keep existing ${DEFAULT_ENV_FILE}"
fi

sudo systemctl daemon-reload
sudo systemctl enable --now "${SERVICE_NAME}"
sudo systemctl status "${SERVICE_NAME}" --no-pager -l || true
