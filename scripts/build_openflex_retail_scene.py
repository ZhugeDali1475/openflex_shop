"""Build and statically inspect an isolated supermarket fixture without ROS."""

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "openflex_ws/src/openflex_thesis/openflex_thesis_mujoco"))

from openflex_thesis_mujoco.retail_scene_builder import build_retail_scene, sha256


def render_segmentation(renderer, model, data, camera, scene_option, np, mujoco):
    original_flags = renderer._scene.flags.copy()
    renderer.enable_segmentation_rendering()
    renderer.update_scene(data, camera=camera, scene_option=scene_option)
    renderer._scene.flags[mujoco.mjtRndFlag.mjRND_SEGMENT] = True
    renderer._scene.flags[mujoco.mjtRndFlag.mjRND_IDCOLOR] = True
    raw = np.empty((renderer.height, renderer.width, 3), dtype=np.uint8)
    mujoco._render.mjr_render(renderer._rect, renderer._scene, renderer._mjr_context)
    mujoco._render.mjr_readPixels(raw, None, renderer._rect, renderer._mjr_context)
    packed = raw.astype(np.uint32)
    packed = packed[:, :, 0] + packed[:, :, 1] * (2 ** 8) + packed[:, :, 2] * (2 ** 16)
    segmentation = np.full((renderer.height, renderer.width, 2), -1, dtype=np.int32)
    try:
        mapping = np.full((model.ngeom + 1, 2), -1, dtype=np.int32)
        visible = [geom for geom in renderer._scene.geoms[:renderer._scene.ngeom]
                   if geom.segid != -1]
        visible_ids = np.asarray([geom.segid + 1 for geom in visible], dtype=np.int32)
        if np.any(visible_ids >= len(mapping)):
            raise ValueError("MuJoCo segmentation ID exceeds the compiled geometry table")
        mapping[visible_ids, 0] = [geom.objid for geom in visible]
        mapping[visible_ids, 1] = [geom.objtype for geom in visible]
        if packed.max(initial=0) >= len(mapping):
            raise ValueError("Rendered segmentation pixel exceeds the compiled geometry table")
        segmentation = mapping[packed]
        return np.flipud(segmentation)
    finally:
        renderer._scene.flags[:] = original_flags
        renderer.disable_segmentation_rendering()


def inspect_scene(scene_path, manifest, render=False):
    import mujoco
    import numpy as np

    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    contacts = []
    for contact in data.contact:
        contacts.append({"geom1_id": int(contact.geom1), "geom2_id": int(contact.geom2),
                         "body1": model.body(model.geom_bodyid[contact.geom1]).name,
                         "body2": model.body(model.geom_bodyid[contact.geom2]).name,
                         "geom1": model.geom(contact.geom1).name,
                         "geom2": model.geom(contact.geom2).name,
                         "distance_m": float(contact.dist)})
    products = []
    for product in manifest["products"]:
        body = model.body(product["body"])
        collision = model.geom(product["collision_geom"])
        support_name = product.get("support_geom")
        support = model.geom(support_name) if support_name else None
        products.append({
            "instance_id": product["instance_id"],
            "body_mass_kg": float(body.mass[0]),
            "mass_matches_assumption": bool(np.isclose(body.mass[0], product["mass_kg"])),
            "body_origin_matches_manifest": bool(np.allclose(data.xpos[body.id], product["initial_position"])),
            "support_top_m": (float(data.geom_xpos[support.id, 2] + support.size[2])
                              if support is not None else None),
            "collision_enabled": bool(collision.contype[0] and collision.conaffinity[0]),
            "initial_product_contacts": [contact for contact in contacts if
                                         product["collision_geom"] in (contact["geom1"], contact["geom2"])],
        })
    report = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "mujoco_version": mujoco.__version__, "scene_sha256": sha256(scene_path),
        "validation": "compile_and_mj_forward_only", "simulation_steps": 0,
        "ros_accessed": False, "live_runtime_modified": False,
        "execution_ready": False, "recognition_performed": False,
        "nq": model.nq, "nv": model.nv, "nbody": model.nbody, "ngeom": model.ngeom,
        "initial_state": "upstream_qpos0_not_a_safe_park_certificate",
        "contacts": contacts,
        "penetrations_over_1um": [contact for contact in contacts if contact["distance_m"] < -0.000001],
        "products": products, "visibility": {},
        "limitations": manifest["limitations"] + [
            "Segmentation is renderer ground truth, not RoboBrain recognition",
            "Static visibility is not reachable viewpoint or motion validation"],
    }
    if render:
        from PIL import Image

        option = mujoco.MjvOption()
        option.geomgroup[3] = 0
        overview = mujoco.MjvCamera()
        overview.lookat[:] = [0, 0, 0.7]
        overview.distance = 9.5
        overview.azimuth = 135
        overview.elevation = -40
        cameras = {"overview": overview, "shelf": "retail_fixture_close", "head": "thesis_head_camera",
                   "head_observation": "thesis_head_camera"}
        with mujoco.Renderer(model, height=720, width=960) as renderer:
            for name, camera in cameras.items():
                view_data = data
                if name == "head_observation":
                    view_data = mujoco.MjData(model)
                    configuration = {"base_x": 0.15, "base_y": 1.18, "base_yaw": float(np.pi),
                                     "openarmx_head_pitch_joint": -0.50}
                    for joint_name, position in configuration.items():
                        joint = model.joint(joint_name)
                        if joint.limited[0] and not joint.range[0] <= position <= joint.range[1]:
                            raise ValueError("Observation configuration violates joint limits")
                        view_data.qpos[joint.qposadr[0]] = position
                    mujoco.mj_forward(model, view_data)
                    report["observation_configuration"] = {
                        "joint_positions": configuration,
                        "scope": "separate_MjData_static_snapshot_not_executed_not_a_navigation_goal",
                        "transition_validated": False,
                        "penetrations_over_1um": [
                            {"geom1_id": int(contact.geom1), "geom2_id": int(contact.geom2),
                             "body1": model.body(model.geom_bodyid[contact.geom1]).name,
                             "body2": model.body(model.geom_bodyid[contact.geom2]).name,
                             "distance_m": float(contact.dist)}
                            for contact in view_data.contact if contact.dist < -0.000001],
                    }
                renderer.update_scene(view_data, camera=camera, scene_option=option)
                rgb = renderer.render().copy()
                Image.fromarray(rgb).save(scene_path.parent / f"{name}.png")
                segmentation = render_segmentation(renderer, model, view_data, camera, option, np, mujoco)
                report["visibility"][name] = {
                    product["instance_id"]: int(np.count_nonzero(
                        (segmentation[:, :, 0] == model.geom(product["visual_geom"]).id) &
                        (segmentation[:, :, 1] == int(mujoco.mjtObj.mjOBJ_GEOM))))
                    for product in manifest["products"]}
                np.save(scene_path.parent / f"{name}_segmentation.npy", segmentation)
        report["static_observation_all_instances_visible"] = all(
            count > 0 for count in report["visibility"]["head_observation"].values())
        report["visibility_source"] = "renderer_instance_segmentation_pixels_960x720_not_model_output"
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", type=Path, default=ROOT / "openflex_mujoco-1.0_basic/openflex_mujoco.xml")
    parser.add_argument("--supermarket", type=Path,
                        default=ROOT / "supermarket_scene/mujoco/supermarket_scene_6x8_competition.xml")
    parser.add_argument("--output", required=True, type=Path, help="New isolated output directory; must not exist")
    parser.add_argument("--collision-mode", choices=("box", "mesh"), default="box",
                        help="box: conservative OBJ bounds; mesh: high-detail convex collision baseline")
    parser.add_argument("--product-set", choices=("sample", "full"), default="sample",
                        help="sample: six controlled products; full: preserve all source-scene product instances")
    parser.add_argument("--assumed-product-mass-kg", type=float, default=0.25,
                        help="uniform explicitly assumed product mass for --product-set full")
    parser.add_argument("--render", action="store_true", help="Static RGB and ground-truth segmentation, no steps")
    options = parser.parse_args()
    os.environ.setdefault("MUJOCO_GL", "egl")
    scene_path, manifest = build_retail_scene(options.robot, options.supermarket, options.output,
                                              collision_mode=options.collision_mode,
                                              product_set=options.product_set,
                                              assumed_product_mass_kg=options.assumed_product_mass_kg)
    report = inspect_scene(scene_path, manifest, options.render)
    (scene_path.parent / "static_audit.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({"output": str(scene_path.parent), "products": len(report["products"]),
                      "initial_contacts": len(report["contacts"]),
                      "penetrations_over_1um": len(report["penetrations_over_1um"]),
                      "visibility": report["visibility"], "execution_ready": False}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
