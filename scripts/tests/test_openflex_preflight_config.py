from pathlib import Path

import pytest

from openflex_preflight_config import resolve_shelf_scene


ROOT = Path(__file__).resolve().parents[2]
V7 = ROOT / "local_dependencies/openflex_thesis/retail_fixture_single_red_runtime_20261113_v7"
DEFAULT = ROOT / "openflex_ws/src/openflex_thesis/openflex_thesis_mujoco/config/sim_shelf_scene.yaml"


def test_legacy_mjcf_environment_resolves_adjacent_runtime_shelf_yaml():
    assert resolve_shelf_scene(DEFAULT, {"OPENFLEX_SIM_SHELF_SCENE": str(V7 / "scene.xml")}) == str(
        V7 / "shelf_scene.yaml"
    )


def test_scene_model_selects_its_matching_runtime_shelf_yaml():
    assert resolve_shelf_scene(DEFAULT, {"OPENFLEX_SIM_SCENE_MODEL": str(V7 / "scene.xml")}) == str(
        V7 / "shelf_scene.yaml"
    )


def test_explicit_yaml_override_is_supported():
    override = V7 / "shelf_scene.yaml"
    assert resolve_shelf_scene(DEFAULT, {"OPENFLEX_SIM_SHELF_SCENE_CONFIG": str(override)}) == str(override)


def test_mjcf_is_never_passed_as_yaml():
    with pytest.raises(ValueError, match="must be YAML"):
        resolve_shelf_scene(DEFAULT, {"OPENFLEX_SIM_SHELF_SCENE_CONFIG": str(V7 / "scene.xml")})
