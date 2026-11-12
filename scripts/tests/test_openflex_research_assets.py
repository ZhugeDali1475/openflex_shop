import importlib.util
import json
from pathlib import Path
import sqlite3
from zipfile import ZipFile

import pytest


SPEC = importlib.util.spec_from_file_location(
    "research_assets", Path(__file__).resolve().parents[1] / "audit_openflex_research_assets.py"
)
audit = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(audit)


def test_generator_model_scan_does_not_execute_code(tmp_path):
    source = tmp_path / "generator.py"
    source.write_text('raise RuntimeError("must not run")\nYCB_MODELS={"box": {}}\nSCANNED_MODELS={"cup": {}}\n')
    assert audit.declared_models(source) == {"box", "cup"}


def test_database_audit_preserves_source_and_does_not_guess_sku(tmp_path):
    database = tmp_path / "inventory.db"
    with sqlite3.connect(database) as connection:
        connection.executescript("""
            CREATE TABLE shelf_groups (id INTEGER);
            CREATE TABLE delivery_tables (id INTEGER);
            CREATE TABLE sku_catalog (sku TEXT, mesh_file TEXT, tex_file TEXT);
            CREATE TABLE shelf_inventory (actual_sku TEXT, expected_sku TEXT);
            INSERT INTO sku_catalog VALUES ('中文饼干', '', '');
            INSERT INTO shelf_inventory VALUES ('中文饼干', '中文饼干');
            INSERT INTO shelf_inventory VALUES ('box', 'box');
            INSERT INTO shelf_inventory VALUES (NULL, 'box');
        """)
    before = audit.sha256(database)
    result = audit.database_audit(database, {"box"})
    assert result["populated_slots"] == 2
    assert result["generator_exact_name_matched_slots"] == 1
    assert result["generator_unmapped_skus"] == ["中文饼干"]
    assert result["catalog_nonempty_mesh_fields"] == 0
    assert result["expected_actual_differences"] == 1
    assert audit.sha256(database) == before


def test_missing_database_is_not_created(tmp_path):
    database = tmp_path / "missing.db"
    with pytest.raises(sqlite3.OperationalError):
        audit.database_audit(database, set())
    assert not database.exists()


def test_scene_count_distinguishes_products_and_other_free_bodies(tmp_path):
    scene = tmp_path / "scene.xml"
    scene.write_text('<mujoco><asset><mesh name="box" file="missing.obj"/></asset>'
                     '<worldbody><body name="product_box_uid0"><freejoint/>'
                     '<geom mesh="box"/></body><body name="basket"><freejoint/>'
                     '</body></worldbody></mujoco>')
    result = audit.xml_audit(scene)
    assert result["product_bodies"] == 1
    assert result["freejoints"] == 2
    assert result["product_mesh_types"] == ["box"]
    assert result["assets"][0]["exists"] is False


def test_docx_extraction_includes_math_and_stable_paragraph_index(tmp_path):
    document = tmp_path / "source.docx"
    with ZipFile(document, "w") as archive:
        archive.writestr("word/document.xml", '<w:document xmlns:w="' + audit.WORD_NS["word"] + '" '
                         'xmlns:m="http://schemas.openxmlformats.org/officeDocument/2006/math">'
                         '<w:body><w:p/><w:p><w:r><w:t>公式</w:t></w:r>'
                         '<m:oMath><m:r><m:t>x</m:t></m:r></m:oMath></w:p></w:body></w:document>')
    result = audit.document_extract(document)
    assert result["paragraphs"] == [{"paragraph": 2, "text": "公式x"}]
    json.dumps(result, ensure_ascii=False)


def test_slides_sort_numerically(tmp_path):
    presentation = tmp_path / "source.pptx"
    with ZipFile(presentation, "w") as archive:
        for index in (10, 2):
            archive.writestr(f"ppt/slides/slide{index}.xml", '<slide xmlns:a="' + audit.DRAW_NS["draw"] + '">'
                             f'<a:t>slide{index}</a:t></slide>')
    result = audit.document_extract(presentation)
    assert [slide["text"] for slide in result["slides"]] == ["slide2", "slide10"]
