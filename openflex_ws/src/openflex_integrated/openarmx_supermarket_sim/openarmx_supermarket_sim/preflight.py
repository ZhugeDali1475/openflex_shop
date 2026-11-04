"""Fail-closed readiness audit for the physical supermarket simulation stack."""

import argparse
import json
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_prefix, get_package_share_directory

from .competition_model_builder import build_competition_model
from .map_builder import build_nav2_map


def _check(name, function, required=True):
    try:
        detail = function()
        return {"name": name, "ok": True, "required": required, "detail": str(detail)}
    except Exception as error:
        return {"name": name, "ok": False, "required": required, "detail": str(error)}


def _package(name):
    return get_package_prefix(name)


def _file(path):
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(path)
    return path


def audit():
    sim_share = Path(get_package_share_directory("openarmx_supermarket_sim"))
    executor_share = Path(get_package_share_directory("openarmx_task_executor"))
    scene = Path("/home/openflex/openflex_shop/supermarket_scene/supermarket_scene_from_db.xml")
    output = Path(tempfile.gettempdir()) / "openarmx_supermarket_preflight.xml"

    def model_check():
        import mujoco

        model_path = build_competition_model("", str(scene), str(output))
        model = mujoco.MjModel.from_xml_path(str(model_path))
        product_count = sum(
            1
            for body_id in range(model.nbody)
            if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id) or "").startswith(
                "product_"
            )
        )
        if product_count < 200:
            raise RuntimeError(f"only {product_count} product bodies loaded")
        return f"{model.nbody} bodies, {product_count} products"

    def map_check():
        with tempfile.TemporaryDirectory(prefix="openarmx_map_preflight_") as directory:
            _yaml, _pgm, grid = build_nav2_map(
                str(scene),
                str(sim_share / "config" / "scene_layout_bottom.yaml"),
                str(executor_share / "config" / "stations_supermarket_mujoco.yaml"),
                directory,
            )
        return f"{grid.spec.width}x{grid.spec.height}, {grid.spec.resolution:.3f} m/cell"

    checks = [
        _check("mujoco_model", model_check),
        _check("registered_map_and_stations", map_check),
        _check("nav2", lambda: _package("nav2_bringup")),
        _check("moveit", lambda: _package("openarmx_integrated_moveit_config")),
        _check(
            "dream_workspace",
            lambda: _file("/home/openflex/openflex_shop/DREAM-realtime/install/setup.bash"),
        ),
        _check("nvidia_device", lambda: _file("/dev/nvidia0")),
    ]
    return checks


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", action="store_true")
    options = parser.parse_args(args)
    checks = audit()
    if options.json:
        print(json.dumps(checks, ensure_ascii=False, indent=2))
    else:
        for check in checks:
            mark = "PASS" if check["ok"] else "FAIL"
            print(f"[{mark}] {check['name']}: {check['detail']}")
    if any(check["required"] and not check["ok"] for check in checks):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
