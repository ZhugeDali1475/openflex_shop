import json
from pathlib import Path
import sys

import pytest

SCRIPT_ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(SCRIPT_ROOT))

from audit_openflex_scene_inventory import build_inventory


ROOT = Path(__file__).parents[2]


def test_inventory_identifies_single_red_target_without_ros(tmp_path):
    output = tmp_path / "inventory"
    result = build_inventory(ROOT / "local_dependencies/openflex_thesis", output)
    assert result["ros_accessed"] is False
    assert result["mujoco_steps"] == 0
    assert [Path(item).name for item in result["selected_runtime_regression_baseline"]] == [
        "retail_fixture_single_red_runtime_20261113_v7"
    ]
    assert [Path(item).name for item in result["selected_static_scene_source"]] == [
        "retail_fixture_single_red_20261113_v3"
    ]
    record = next(item for item in result["fixtures"]
                  if item["directory"].endswith("retail_fixture_single_red_runtime_20261113_v7"))
    assert record["shelf_top_level_bodies"] == ["shelf_group_1", "thesis_shelf"]
    assert record["delivery_top_level_bodies"] == ["counter_top_right", "thesis_delivery_table"]
    assert record["product_count_manifest_or_xml"] == 1
    assert record["red_product_count_manifest_or_xml"] == 1
    saved = json.loads((output / "scene_inventory.json").read_text())
    assert saved["target_topology"] == result["target_topology"]


def test_inventory_refuses_overwrite(tmp_path):
    output = tmp_path / "inventory"
    build_inventory(ROOT / "local_dependencies/openflex_thesis", output)
    with pytest.raises(FileExistsError):
        build_inventory(ROOT / "local_dependencies/openflex_thesis", output)
