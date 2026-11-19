#!/usr/bin/env python3
"""Create a read-only inventory of local MuJoCo retail fixture directories.

The audit never starts ROS, calls an action, or advances MuJoCo.  It refuses to
overwrite an existing output directory so that an inventory is reproducible.
"""

import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ROOT = ROOT / "local_dependencies/openflex_thesis"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _top_level_names(world, prefix=None):
    names = [body.get("name", "") for body in world.findall("body")]
    return sorted(name for name in names if name and (prefix is None or name.startswith(prefix)))


def inspect_fixture(directory: Path) -> dict:
    scene = directory / "scene.xml"
    manifest_path = directory / "manifest.json"
    if not scene.exists():
        return {"directory": str(directory), "usable": False, "reason": "missing scene.xml"}
    try:
        root = ET.parse(scene).getroot()
        world = root.find("worldbody")
        if world is None:
            raise ValueError("missing worldbody")
        manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else {}
        body_names = [body.get("name", "") for body in world.findall("body")]
        shelf_names = sorted(name for name in body_names
                             if name.startswith("shelf_group_") or name in {"thesis_shelf"})
        delivery_names = sorted(name for name in body_names if name.startswith("counter_")
                                or name in {"thesis_delivery_table"})
        products = manifest.get("products", [])
        manifest_product_count = (len(products) if manifest_path.exists() and isinstance(products, list)
                                  else None)
        manifest_red_count = sum(1 for item in products
                                 if isinstance(item, dict) and str(item.get("color", "")).lower() == "red")
        # Runtime profiles intentionally keep a smaller file set and may not
        # carry manifest.json.  Infer only the explicit single-red fixture
        # shape from XML; never infer an arbitrary SKU or visual identity.
        xml_products = [name for name in body_names if name.startswith("product_")]
        has_single_runtime_object = body_names.count("thesis_object") == 1 and not xml_products
        product_count = manifest_product_count if manifest_product_count is not None else (1 if has_single_runtime_object else None)
        red_count = manifest_red_count if manifest_product_count is not None else (1 if has_single_runtime_object else 0)
        target = (len(shelf_names) == 2 and len(delivery_names) == 2
                  and product_count == 1 and red_count == 1)
        if target:
            category = "single_red_two_shelves_two_deliveries"
        elif len(shelf_names) == 2 and len(delivery_names) == 2 and product_count and product_count > 1:
            category = "two_shelves_two_deliveries_multi_product_offline"
        else:
            category = "other_or_incomplete"
        fixture_name = directory.name
        if fixture_name == "retail_fixture_single_red_runtime_20261113_v7":
            role = "accepted_single_red_runtime_regression_baseline"
        elif fixture_name == "retail_fixture_single_red_20261113_v3":
            role = "single_red_static_scene_source"
        elif "single_red_runtime" in fixture_name:
            role = "historical_runtime_profile_do_not_select_implicitly"
        elif "single_red" in fixture_name:
            role = "historical_static_single_red_variant"
        elif "half_left" in fixture_name:
            role = "offline_parent_fixture_contains_multiple_products"
        else:
            role = "offline_or_historical_fixture"
        return {
            "directory": str(directory), "usable": True, "category": category,
            "role": role,
            "scene_sha256": sha256(scene), "scene_xml": str(scene),
            "shelf_top_level_bodies": shelf_names,
            "delivery_top_level_bodies": delivery_names,
            "product_count_manifest_or_xml": product_count,
            "red_product_count_manifest_or_xml": red_count,
            "manifest_present": manifest_path.exists(),
            "execution_enabled": manifest.get("execution_enabled"),
            "runtime_scene_replaced": manifest.get("runtime_scene_replaced"),
        }
    except (ET.ParseError, OSError, ValueError, json.JSONDecodeError) as error:
        return {"directory": str(directory), "usable": False, "reason": str(error)}


def build_inventory(root: Path, output: Path) -> dict:
    root = root.expanduser().resolve(strict=True)
    output = output.expanduser().resolve()
    if output.exists():
        raise FileExistsError(f"Refusing to overwrite existing output directory: {output}")
    fixtures = []
    for scene in sorted(root.glob("retail_fixture*/scene.xml")):
        fixtures.append(inspect_fixture(scene.parent))
    topology_matches = [item for item in fixtures
                        if item.get("category") == "single_red_two_shelves_two_deliveries"]
    selected_runtime = [item["directory"] for item in fixtures
                        if item.get("role") == "accepted_single_red_runtime_regression_baseline"]
    selected_static_source = [item["directory"] for item in fixtures
                              if item.get("role") == "single_red_static_scene_source"]
    result = {
        "schema_version": 1,
        "root": str(root),
        "ros_accessed": False,
        "mujoco_steps": 0,
        "target_topology": {
            "shelves": 2, "delivery_tables": 2, "red_test_objects": 1,
            "forbidden_scope": ["four shelves/four delivery tables", "multi-product execution"],
        },
        "topology_matches_not_all_runtime_validated": [item["directory"] for item in topology_matches],
        "selected_runtime_regression_baseline": selected_runtime,
        "selected_static_scene_source": selected_static_source,
        "fixtures": fixtures,
    }
    output.mkdir(parents=True, exist_ok=False)
    (output / "scene_inventory.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(build_inventory(args.root, args.output), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
