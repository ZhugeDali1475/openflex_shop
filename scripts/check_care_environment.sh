#!/usr/bin/env bash
set -eo pipefail

workspace="/home/wgb/openflex_shop/openflex_ws"
model="/home/wgb/openflex_shop/openflex_mujoco-1.0_basic/openflex_mujoco.xml"

set +u
source /opt/ros/humble/setup.bash
source "${workspace}/install_v2/setup.bash"
set -u

export MUJOCO_GL=egl
export PYTHONPATH="${workspace}/src/openflex_integrated/openarmx_supermarket_sim:${PYTHONPATH:-}"

python3 -c "import mujoco; model = mujoco.MjModel.from_xml_path('${model}'); print('mujoco_model', model.nbody, model.njnt, model.nu)"

for package in \
  openflex_care_agent \
  openflex_tc_wam \
  openflex_thesis_control \
  openflex_thesis_experiments \
  openflex_thesis_mujoco \
  openarmx_integrated_description \
  openarmx_head_description; do
  ros2 pkg prefix "${package}" >/dev/null
done

PYTHONPATH="${workspace}/src/openflex_thesis/openflex_care_agent" \
  python3 -m pytest -q "${workspace}/src/openflex_thesis/openflex_care_agent/test"

PYTHONPATH="${workspace}/src/openflex_thesis/openflex_thesis_mujoco" \
  python3 -m pytest -q "${workspace}/src/openflex_thesis/openflex_thesis_mujoco/test"

echo "care_environment_ok"
