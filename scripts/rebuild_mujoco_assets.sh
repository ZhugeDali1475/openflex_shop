#!/usr/bin/env bash
set -eo pipefail

root="/home/wgb/openflex_shop"
workspace="${root}/openflex_ws"
model_project="${root}/openflex_mujoco-1.0_basic"

mkdir -p "${model_project}/packages/openflex_chassis"

ln -sfn \
  "${workspace}/src/openflex_chassis/base_model_interface_layer/swerve_description" \
  "${model_project}/packages/openflex_chassis/swerve_description"
ln -sfn \
  "${workspace}/src/openflex_lift_slide/lift_slide_description" \
  "${model_project}/packages/lift_slide_description"
ln -sfn \
  "${workspace}/src/openflex_armx/openarmx_description" \
  "${model_project}/packages/openarmx_description"
ln -sfn \
  "${workspace}/src/openflex_head/openarmx_head_description" \
  "${model_project}/packages/openarmx_head_description"
ln -sfn \
  "${workspace}/src/openflex_integrated/openarmx_integrated_description" \
  "${model_project}/packages/openarmx_integrated_description"

ln -sfn "${model_project}" "${workspace}/src/openflex_mujoco"

cd "${model_project}"
conda run -n lerobot python build_urdf.py
conda run -n lerobot env MUJOCO_GL=egl python convert.py --check

echo "mujoco_assets_rebuilt"
