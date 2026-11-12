"""Read-only research provenance and supermarket asset audit, without ROS."""

import argparse
import ast
from collections import Counter
import hashlib
import json
from pathlib import Path
import re
import sqlite3
import xml.etree.ElementTree as ElementTree
from zipfile import ZipFile


ROOT = Path(__file__).resolve().parents[1]
WORD_NS = {"word": "http://schemas.openxmlformats.org/wordprocessingml/2006/main"}
DRAW_NS = {"draw": "http://schemas.openxmlformats.org/drawingml/2006/main"}


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def declared_models(generator):
    models = set()
    for node in ast.parse(generator.read_text()).body:
        if isinstance(node, ast.Assign) and any(
            isinstance(target, ast.Name) and target.id in {"YCB_MODELS", "SCANNED_MODELS"}
            for target in node.targets
        ):
            models.update(ast.literal_eval(node.value))
    return models


def xml_audit(path):
    root = ElementTree.parse(path).getroot()
    products = [body for body in root.iter("body") if body.get("name", "").startswith("product_")]
    compiler = root.find("compiler")
    assets = []
    for tag in ("mesh", "texture"):
        directory = compiler.get(tag + "dir", ".") if compiler is not None else "."
        for asset in root.findall("./asset/" + tag):
            filename = asset.get("file")
            if filename:
                asset_path = (path.parent / directory / filename).resolve()
                assets.append({"name": asset.get("name"), "kind": tag,
                               "path": str(asset_path), "exists": asset_path.is_file()})
    return {
        "path": str(path), "sha256": sha256(path),
        "bodies": len(list(root.iter("body"))),
        "geoms": len(list(root.iter("geom"))),
        "freejoints": len(list(root.iter("freejoint"))),
        "product_bodies": len(products),
        "product_mesh_types": sorted({geom.get("mesh") for body in products
                                      for geom in body.iter("geom") if geom.get("mesh")}),
        "compiler_angle": compiler.get("angle", "degree") if compiler is not None else "degree",
        "assets": assets,
        "cameras": [camera.get("name") for camera in root.iter("camera")],
    }


def database_audit(path, model_names):
    connection = sqlite3.connect(path.resolve().as_uri() + "?mode=ro", uri=True)
    connection.row_factory = sqlite3.Row
    try:
        connection.execute("PRAGMA query_only=ON")
        connection.execute("BEGIN")
        inventory = [dict(row) for row in connection.execute("SELECT * FROM shelf_inventory")]
        catalog = [dict(row) for row in connection.execute("SELECT * FROM sku_catalog")]
        counts = {table: connection.execute("SELECT count(*) FROM " + table).fetchone()[0]
                  for table in ("shelf_groups", "delivery_tables", "sku_catalog", "shelf_inventory")}
        populated = [row for row in inventory if str(row["actual_sku"] or "").strip()]
        skus = Counter(str(row["actual_sku"]).strip() for row in populated)
        matched = {sku: count for sku, count in skus.items() if sku in model_names}
        return {
            "path": str(path), "access": "SQLite URI mode=ro; query_only; read transaction",
            "table_counts": counts, "populated_slots": len(populated),
            "distinct_actual_skus": len(skus), "actual_sku_counts": dict(sorted(skus.items())),
            "generator_exact_name_matched_slots": sum(matched.values()),
            "generator_exact_name_matched_skus": sorted(matched),
            "generator_unmapped_skus": sorted(set(skus) - model_names),
            "catalog_nonempty_mesh_fields": sum(bool(str(row["mesh_file"] or "").strip()) for row in catalog),
            "catalog_nonempty_texture_fields": sum(bool(str(row["tex_file"] or "").strip()) for row in catalog),
            "expected_actual_differences": sum(row["expected_sku"] != row["actual_sku"] for row in inventory),
            "identity_warning": "No implicit mapping from a Chinese SKU to a visually similar YCB object.",
        }
    finally:
        connection.close()


def document_extract(path):
    with ZipFile(path) as archive:
        if path.suffix.lower() == ".docx":
            root = ElementTree.fromstring(archive.read("word/document.xml"))
            paragraphs = []
            for index, paragraph in enumerate(root.findall("./word:body/word:p", WORD_NS), 1):
                text = "".join(node.text or "" for node in paragraph.iter()
                               if node.tag.rsplit("}", 1)[-1] == "t")
                if text.strip():
                    paragraphs.append({"paragraph": index, "text": text})
            tables = []
            for table in root.findall("./word:body/word:tbl", WORD_NS):
                tables.append([["".join(cell.itertext()) for cell in row.findall("word:tc", WORD_NS)]
                               for row in table.findall("word:tr", WORD_NS)])
            return {"source": str(path), "sha256": sha256(path),
                    "paragraphs": paragraphs, "tables": tables,
                    "index_note": "Top-level DOCX paragraph index; includes math text, not page numbers."}
        slides = sorted((name for name in archive.namelist()
                         if re.fullmatch(r"ppt/slides/slide\d+\.xml", name)),
                        key=lambda name: int(re.search(r"slide(\d+)", name).group(1)))
        return {"source": str(path), "sha256": sha256(path), "slides": [
            {"slide": index, "text": "\n".join(node.text or "" for node in
             ElementTree.fromstring(archive.read(name)).findall(".//draw:t", DRAW_NS))}
            for index, name in enumerate(slides, 1)]}


def render_preview(path, output):
    import mujoco
    from PIL import Image

    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    pictures = []
    with mujoco.Renderer(model, height=540, width=960) as renderer:
        for name in ("overview_angle", "shelf_close", "top_view"):
            if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name) < 0:
                continue
            renderer.update_scene(data, camera=name)
            destination = output / (name + ".png")
            Image.fromarray(renderer.render()).save(destination)
            pictures.append(destination.name)
    return {"mode": "Independent mj_forward only; no mj_step, ROS, controls or live scene replacement",
            "model_nbody": model.nbody, "model_ngeom": model.ngeom, "pictures": pictures,
            "dynamics_verified": False, "grasp_verified": False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--midterm", type=Path, default=Path("/home/wgb/中期"))
    parser.add_argument("--render", action="store_true")
    arguments = parser.parse_args()
    output = arguments.output.resolve()
    scene = ROOT / "supermarket_scene"
    if output == ROOT or output.is_relative_to(scene.resolve()) or output.is_relative_to(arguments.midterm.resolve()):
        parser.error("Output must not replace the repository root, original scene assets or midterm materials")
    output.mkdir(parents=True, exist_ok=False)
    models = declared_models(scene / "mujoco/generate_scene_from_database.py")
    database = scene / "shelf_inventory.db"
    database_before = sha256(database)
    report = {"scope": "Offline source and asset audit, not a robot experiment",
              "declared_generator_models": sorted(models),
              "database": database_audit(database, models),
              "scenes": [xml_audit(scene / "mujoco" / name) for name in (
                  "supermarket_scene_6x8_competition.xml", "supermarket_scene_from_db.xml")],
              "documents": []}
    for name in ("24S130317王国彬中期报告.docx", "ACT_PI05_底盘VLA_设计空间实验方案_v2.docx",
                 "OpenFlex_中期报告.pptx"):
        path = arguments.midterm / name
        if path.is_file():
            extraction = document_extract(path)
            destination = output / (path.stem + "_提取.json")
            destination.write_text(json.dumps(extraction, ensure_ascii=False, indent=2) + "\n")
            report["documents"].append({"source": str(path), "sha256": extraction["sha256"],
                                        "extraction": destination.name})
    if arguments.render:
        report["preview"] = render_preview(scene / "mujoco/supermarket_scene_6x8_competition.xml", output)
    report["database"]["sha256_before"] = database_before
    report["database"]["sha256_after"] = sha256(database)
    report["database"]["file_hash_unchanged"] = database_before == report["database"]["sha256_after"]
    destination = output / "asset_audit.json"
    destination.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({"output": str(output), "database_unchanged": report["database"]["file_hash_unchanged"],
                      "populated_slots": report["database"]["populated_slots"],
                      "exact_name_matched_slots": report["database"]["generator_exact_name_matched_slots"],
                      "products_per_xml": {Path(item["path"]).name: item["product_bodies"]
                                           for item in report["scenes"]}}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
