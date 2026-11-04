#!/usr/bin/env bash
set -eo pipefail

root="/home/wgb/openflex_shop"

find "${root}/openflex_ws/src" "${root}/scripts" \
  -type d \( -name __pycache__ -o -name .pytest_cache \) \
  -prune -exec rm -rf {} +

find "${root}/openflex_ws/build_v2" "${root}/openflex_ws/install_v2" \
  -type d \( -name __pycache__ -o -name .pytest_cache \) \
  -prune -exec rm -rf {} +

find "${root}/supermarket_scene" \
  -path "${root}/supermarket_scene/.venv" -prune -o \
  -type d \( -name __pycache__ -o -name .pytest_cache \) \
  -prune -exec rm -rf {} +

rm -rf \
  "${root}/.cache/npm" \
  "${root}/openflex_ws/.pytest_cache" \
  "${root}/openflex_ws/log" \
  "${root}/openflex_ws/log_v2"

rmdir "${root}/.cache" 2>/dev/null || true

echo "generated_artifacts_cleaned"
