"""Export the source-audited review, separately from the thesis and live runtime."""

import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import shutil

from docx import Document
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm
from PIL import Image

from academic_support import chinese_count, validate_math_roundtrip
from build_thesis import convert, digest, render, setup


ROOT = Path(__file__).resolve().parents[1]
STEM = "OpenFlex商超研究主线与算法实验方案_原创归属修订_20261112"
PROFILES = {
    "midterm": (
        "OpenFlex · 中期衔接与自研贡献审阅（非最终学位论文）",
        "2026年11月12日 · 基于作者中期材料、源码和现有证据的研究重构",
        "本文不是新增机器人实验报告。中期原件、旧论文和当前机器人运行现场均不替换。",
    ),
    "retail": (
        "OpenFlex · 商超轮臂机器人研究定位与实验方案（待验证）",
        "2026年11月12日 · 按最新研究决议重构：自研系统、算法、对照实验与局限",
        "本文为研究方案和论证稿，不是已完成实验的学位论文。底盘观测表征不再作为必做研究主线；旧稿与运行现场均保留。",
    ),
}


def fit_images(document, maximum_height_cm=17.0):
    limit = Cm(maximum_height_cm)
    for image in document.inline_shapes:
        if image.height > limit:
            image.width = int(image.width * limit / image.height)
            image.height = limit


def provenance_sources(profile="midterm"):
    original = Path("/home/wgb/openflex_0")
    return [ROOT / relative for relative in (
        "openflex_ws/src/openflex_chassis/motion_control_layer/swerve_controller/src/swerve_drive_kinematics.cpp",
        "openflex_ws/src/openflex_chassis/motion_control_layer/swerve_controller/src/swerve_drive_controller.cpp",
        "openflex_ws/src/openflex_integrated/openarmx_integrated_moveit_config/src/openflex_ik_pipeline.cpp",
        "openflex_ws/src/openflex_integrated/openarmx_integrated_moveit_config/src/openflex_kinematics_plugin.cpp",
        "openflex_ws/src/openflex_thesis/openflex_tc_wam/openflex_tc_wam/scoring.py",
        "openflex_ws/src/openflex_thesis/openflex_tc_wam/openflex_tc_wam/candidate_generation.py",
        "openflex_ws/src/openflex_thesis/openflex_thesis_mujoco/openflex_thesis_mujoco/model_builder.py",
        "openflex_ws/src/openflex_thesis/openflex_roboos_bridge/openflex_roboos_bridge/scene_registry.py",
        "openflex_ws/src/openflex_thesis/openflex_thesis_mujoco/config/mujoco.yaml",
        "supermarket_scene/mujoco/generate_scene_from_database.py",
    )] + ([ROOT / relative for relative in (
        "openflex_ws/src/openflex_thesis/openflex_tc_wam/openflex_tc_wam/moveit_evaluator_node.py",
        "openflex_ws/src/openflex_thesis/openflex_roboos_bridge/openflex_roboos_bridge/action_journal.py",
        "openflex_ws/src/openflex_thesis/openflex_roboos_bridge/openflex_roboos_bridge/manipulation_skill.py",
        "openflex_ws/src/openflex_thesis/openflex_roboos_bridge/openflex_roboos_bridge/place_skill.py",
        "scripts/openflex_delivery_protocol.json",
    )] if profile == "retail" else [original / relative for relative in (
        "src/openarmx/openarmx_vla/lerobot_robot_openflex_follower_ros2/lerobot_robot_openflex_follower_ros2/robot_bridge.py",
        "src/openarmx/openarmx_vla/lerobot_robot_openflex_follower_ros2/lerobot_robot_openflex_follower_ros2/config.py",
        "src/openarmx/openarmx_vla/lerobot_teleoperator_openflex_leader_ros2/lerobot_teleoperator_openflex_leader_ros2/teleop_bridge.py",
        "scripts/lerobot_record_openflex.py", "scripts/lerobot-infer-openflex.py",
        "scripts/convert_dataset_action_to_future_odom.py", "VLA_BASE_EXPERIMENT_GUIDE_CN.md",
    )])


def build(audit, output, midterm, sources=None, stem=STEM, profile="midterm"):
    if Path(stem).name != stem or not stem:
        raise ValueError("Output stem must be a nonempty filename")
    if profile not in PROFILES:
        raise ValueError("Unknown document profile")
    if not sources:
        raise ValueError("Export requires explicit source documents")
    texts = [source.read_text() for source in sources]
    hashes = [digest(source) for source in sources]
    text = texts[0]
    for appendix in texts[1:]:
        heading = appendix.splitlines()[0].removeprefix("# ")
        text += "\n\n## 附录：" + heading + "\n\n" + "\n".join(appendix.splitlines()[1:])
    output.mkdir(parents=True, exist_ok=False)
    assets = output / "assets"
    assets.mkdir()
    for filename in ("overview_angle.png", "shelf_close.png", "top_view.png"):
        shutil.copy2(audit / filename, assets / filename)
    platform = midterm / "裸机.png"
    with Image.open(platform) as image:
        image.thumbnail((1200, 1600))
        image.save(assets / "openflex_platform.png")
    for filename in ("asset_audit.json", "mathematical_checks.json"):
        shutil.copy2(audit / filename, output / filename)
    for source, content in zip(sources, texts):
        (output / source.name).write_text(content)
    document = Document()
    setup(document)
    header = document.sections[0].header.paragraphs[0]
    header.clear()
    profile_header, profile_subtitle, profile_notice = PROFILES[profile]
    header.add_run(profile_header)
    title = texts[0].splitlines()[0].removeprefix("# ")
    document.add_paragraph(title, style="Title")
    document.add_paragraph(profile_subtitle, style="Subtitle")
    document.add_paragraph(profile_notice)
    contents = document.add_paragraph()
    field = OxmlElement("w:fldSimple")
    field.set(qn("w:instr"), 'TOC \\o "1-3" \\h \\z \\u')
    contents._p.append(field)
    render(text, document, assets)
    fit_images(document)
    document.core_properties.title = title
    document.core_properties.author = ""
    document.core_properties.subject = "Source-audited research review; not a completed thesis or new robot evaluation"
    document.core_properties.created = datetime.now()
    document.core_properties.modified = datetime.now()
    docx = output / (stem + ".docx")
    document.save(docx)
    legacy = convert(docx, output, "doc", "MS Word 97")
    if legacy.read_bytes()[:8] != bytes.fromhex("D0CF11E0A1B11AE1"):
        raise RuntimeError("Not a genuine Word 97-2003 OLE document")
    validation = output / "validation"
    validation.mkdir()
    restored = Document(convert(legacy, validation, "docx", "Office Open XML Text"))
    equation_validation = validate_math_roundtrip(document, restored)
    if len(document.tables) != len(restored.tables) or len(document.inline_shapes) != len(restored.inline_shapes):
        raise RuntimeError("Table or image count changed during DOC conversion")
    restored_text = "\n".join(paragraph.text for paragraph in restored.paragraphs)
    required_text = (title,)
    for required in required_text:
        if required not in restored_text:
            raise RuntimeError("DOC content missing: " + required)
    pdf = convert(docx, output, "pdf", "writer_pdf_Export")
    if [digest(source) for source in sources] != hashes:
        raise RuntimeError("Source changed while exporting")
    provenance = [{"path": str(path), "exists": path.is_file(),
                   "sha256": digest(path) if path.is_file() else None}
                  for path in provenance_sources(profile)]
    report = {
        "kind": "retail_research_plan_not_completed_experiments" if profile == "retail" else "source_audited_chapter_draft",
        "profile": profile, "title": title,
        "sources": [{"path": str(source), "sha256": signature} for source, signature in zip(sources, hashes)],
        "code_sources": provenance,
        "platform_photo": {"source": str(platform), "sha256": digest(platform), "operation": "thumbnail only"},
        "chinese_characters_including_tables_references_and_appendix": chinese_count(text),
        "numbered_equations": len(re.findall(r"\\tag\{[^}]+\}", text)),
        "tables": len(document.tables), "images": len(document.inline_shapes),
        "doc_format": "Word 97-2003 OLE, not renamed DOCX/HTML",
        "equation_validation": equation_validation,
        "validation_limit": "DOC formula token multisets and counts checked; PDF layout spot-checked separately. Word/WPS UI not directly tested.",
        "files": {path.name: {"bytes": path.stat().st_size, "sha256": digest(path)} for path in (docx, legacy, pdf)},
    }
    (output / "document_manifest.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({"output": str(output), "equations": report["numbered_equations"],
                      "tables": report["tables"], "images": report["images"], "validated": True}, ensure_ascii=False))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--audit", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--midterm", type=Path, default=Path("/home/wgb/中期"))
    parser.add_argument("--source", type=Path, action="append", required=True)
    parser.add_argument("--stem", default=STEM)
    parser.add_argument("--profile", choices=PROFILES, default="retail")
    arguments = parser.parse_args()
    sources = [source.resolve() for source in arguments.source] if arguments.source else None
    build(arguments.audit.resolve(), arguments.output.resolve(), arguments.midterm.resolve(), sources, arguments.stem,
          arguments.profile)


if __name__ == "__main__":
    main()
