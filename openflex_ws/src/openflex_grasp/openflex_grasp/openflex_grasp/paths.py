"""Deployment paths configurable without editing the ROS package."""

import os
from pathlib import Path


OPENFLEX_ROOT = Path(
    os.environ.get("OPENFLEX_ROOT", "/home/openflex/openflex_shop")
).expanduser()
WORKSPACE_ROOT = Path(
    os.environ.get("OPENFLEX_WORKSPACE", str(OPENFLEX_ROOT / "openflex_ws"))
).expanduser()
GRASP_SUITE_ROOT = Path(
    os.environ.get(
        "OPENFLEX_GRASP_SUITE_ROOT", str(WORKSPACE_ROOT / "src" / "openflex_grasp")
    )
).expanduser()
MODEL_ROOT = Path(
    os.environ.get("OPENFLEX_MODEL_ROOT", str(OPENFLEX_ROOT / "openflex_models"))
).expanduser()
GRASPNET_REPO = Path(
    os.environ.get(
        "OPENFLEX_GRASPNET_REPO", str(GRASP_SUITE_ROOT / "graspnet-baseline")
    )
).expanduser()
GRASPNET_CHECKPOINT = Path(
    os.environ.get(
        "OPENFLEX_GRASPNET_CHECKPOINT",
        str(MODEL_ROOT / "graspnet-baseline" / "checkpoint-rs.tar"),
    )
).expanduser()
QWEN_MODEL = Path(
    os.environ.get("OPENFLEX_QWEN_MODEL", str(MODEL_ROOT / "Qwen3.5-4B"))
).expanduser()
SAM_MODEL = Path(
    os.environ.get("OPENFLEX_SAM_MODEL", str(MODEL_ROOT / "sam2.1-hiera-small"))
).expanduser()
