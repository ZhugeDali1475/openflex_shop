"""Export offline retail boxes for a future multi-object MoveIt adapter."""

import argparse
import json
from pathlib import Path
import sys

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from build_openflex_retail_scene import sha256


def export_objects(scene_path, manifest_path, output_directory):
    scene = Path(scene_path).expanduser().resolve(strict=True)
    manifest_file = Path(manifest_path).expanduser().resolve(strict=True)
    output = Path(output_directory).expanduser().resolve()
    if output.exists():
        raise FileExistsError(f"Refusing to overwrite an existing directory: {output}")
    manifest = json.loads(manifest_file.read_text(encoding="utf-8"))
    if sha256(scene) != manifest.get("scene_sha256"):
        raise ValueError("Scene hash does not match the retail manifest")
    if manifest.get("collision_mode") != "box":
        raise ValueError("MoveIt box export requires the audited box collision mode")

    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    data.qpos[:] = model.qpos0
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    objects = []
    seen_ids = set()
    for product in manifest.get("products", []):
        identifier = product.get("instance_id", "")
        if not identifier or identifier in seen_ids:
            raise ValueError("Product object IDs must be nonempty and unique")
        seen_ids.add(identifier)
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM,
                                    product.get("collision_geom", ""))
        if geom_id < 0 or model.geom_type[geom_id] != mujoco.mjtGeom.mjGEOM_BOX:
            raise ValueError(f"Missing box collision geometry for {identifier}")
        quaternion_wxyz = np.zeros(4, dtype=np.float64)
        mujoco.mju_mat2Quat(quaternion_wxyz, data.geom_xmat[geom_id])
        quaternion_norm = float(np.linalg.norm(quaternion_wxyz))
        if not np.isfinite(quaternion_norm) or quaternion_norm <= 0.0:
            raise ValueError(f"Invalid collision orientation for {identifier}")
        quaternion_wxyz /= quaternion_norm
        size = model.geom_size[geom_id]
        position = data.geom_xpos[geom_id]
        objects.append({
            "id": identifier,
            "simulation_sku": product["simulation_sku"],
            "instance_id": identifier,
            "slot_id": product["slot_id"],
            "role": "movable_product_world_object",
            "shape": "box",
            "dimensions_m": (2.0 * size).tolist(),
            "pose": {
                "position_m": position.tolist(),
                "orientation_xyzw": [float(value) for value in
                                     (quaternion_wxyz[1], quaternion_wxyz[2],
                                      quaternion_wxyz[3], quaternion_wxyz[0])],
            },
            "mass_kg": product["mass_kg"],
            "mass_provenance": product["mass_provenance"],
            "collision_geom": product["collision_geom"],
        })
    if len(objects) != len(manifest.get("products", [])) or not objects:
        raise ValueError("Manifest product/object count mismatch")
    result = {
        "schema_version": 1,
        "frame_id": "mujoco_sim_world",
        "frame_registered_to_live_map": False,
        "source_scene": str(scene),
        "source_scene_sha256": sha256(scene),
        "source_manifest": str(manifest_file),
        "source_manifest_sha256": sha256(manifest_file),
        "runtime_consumable": False,
        "execution_enabled": False,
        "lifecycle_contract": "future_adapter_must_remove_or_attach_only_selected_target_and_keep_other_products_as_obstacles",
        "objects": objects,
        "limitations": [
            "Not a shelf_scene configuration and must not be loaded by the current single-object bridge",
            "World frame is not calibrated or transformed to the live robot planning frame",
            "No MoveIt ApplyPlanningScene call or readback was performed",
            "Selected-target contact and attached-object transitions remain unimplemented",
        ],
    }
    output.mkdir(parents=True, exist_ok=False)
    destination = output / "planning_scene_products.json"
    destination.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    return destination, result


def main():
    root = Path(__file__).resolve().parents[1]
    default_scene = root / "local_dependencies/openflex_thesis/retail_fixture_20261112_v4/scene.xml"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=default_scene)
    parser.add_argument("--manifest", type=Path, default=default_scene.parent / "manifest.json")
    parser.add_argument("--output", required=True, type=Path,
                        help="new isolated output directory; must not already exist")
    options = parser.parse_args()
    path, result = export_objects(options.scene, options.manifest, options.output)
    print(json.dumps({"output": str(path), "objects": len(result["objects"]),
                      "frame_registered_to_live_map": result["frame_registered_to_live_map"],
                      "runtime_consumable": result["runtime_consumable"]}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
