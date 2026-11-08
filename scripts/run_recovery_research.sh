#!/usr/bin/env bash
set -eo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${root}"

if [[ "${1:-}" == setup ]]; then
    "${root}/.local-uv" venv --python /usr/bin/python3 --system-site-packages "${root}/.venvs/recovery"
    "${root}/.local-uv" pip install --python "${root}/.venvs/recovery/bin/python" \
        -r openflex_ws/src/openflex_thesis/openflex_thesis_experiments/requirements-learning.txt
    exit 0
fi

set +u
source /opt/ros/humble/setup.bash
source "${root}/openflex_ws/install_v2/setup.bash"
set -u

for package in "${root}"/openflex_ws/src/openflex_thesis/*; do
    export PYTHONPATH="${package}:${PYTHONPATH:-}"
done

command="${1:-help}"
shift || true
case "${command}" in
    matrix) exec python3 -m openflex_thesis_experiments.recovery_protocol "$@" ;;
    run) exec python3 -m openflex_thesis_experiments.recovery_benchmark --root "${root}" "$@" ;;
    train) exec "${root}/.venvs/recovery/bin/python" -m openflex_thesis_experiments.recovery_training "$@" ;;
    report) exec python3 -m openflex_thesis_experiments.recovery_report "$@" ;;
    test) exec "${root}/.venvs/recovery/bin/python" -m pytest -p no:anyio -q "${root}/openflex_ws/src/openflex_thesis" "$@" ;;
    *) printf '%s\n' 'Usage: bash scripts/run_recovery_research.sh {setup|matrix|run|train|report|test} [arguments]'; exit 2 ;;
esac
