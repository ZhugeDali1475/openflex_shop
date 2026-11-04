import math
from pathlib import Path

import pytest
from PyQt5.QtWidgets import QApplication

from openflex_navigation_gui.scene_registration import SceneRegistrationPage


def _write_map(root: Path) -> Path:
    pgm = root / "arena.pgm"
    pgm.write_bytes(b"P5\n100 80\n255\n" + bytes([254] * 100 * 80))
    yaml = root / "arena.yaml"
    yaml.write_text(
        "image: arena.pgm\nresolution: 0.05\norigin: [-2.5, -2.0, 0.0]\n",
        encoding="utf-8",
    )
    return yaml


def _write_named_map(root: Path, name: str) -> Path:
    map_dir = root / "2d" / name
    map_dir.mkdir(parents=True)
    pgm = map_dir / "map.pgm"
    pgm.write_bytes(b"P5\n100 80\n255\n" + bytes([254] * 100 * 80))
    yaml = map_dir / "map.yaml"
    yaml.write_text(
        "image: map.pgm\nresolution: 0.05\norigin: [-2.5, -2.0, 0.0]\n",
        encoding="utf-8",
    )
    return yaml


def _snapshot():
    return {
        "scene_revision": 7,
        "map_binding": None,
        "shelf_types": [],
        "shelves": [{
            "id": 1,
            "name": "Shelf A",
            "world_x": 0.0,
            "world_y": 0.0,
            "yaw": 0.0,
            "shelf_width": 0.8,
            "shelf_length": 1.8,
            "face_0_representatives": ["normal: tea"],
            "face_1_representatives": ["missing: cola"],
        }],
        "delivery_tables": [{
            "id": 2,
            "name": "Table B",
            "world_x": 2.0,
            "world_y": 1.0,
            "yaw": 0.2,
            "length": 1.2,
            "width": 0.8,
        }],
        "skus": [],
        "slots": [],
    }


def test_registration_page_loads_map_entities_and_numeric_pose_edit(tmp_path: Path):
    app = QApplication.instance() or QApplication([])
    page = SceneRegistrationPage(None)
    page.resize(1100, 700)
    page.load_map(_write_map(tmp_path))
    page._accept_snapshot(_snapshot())
    page.show()
    app.processEvents()

    assert page.yaml_path.name == "arena.yaml"
    assert page.map_name == "arena"
    assert page.map_fingerprint.startswith("sha256:")
    assert len(page.canvas._entity_items) == 2
    assert page.entity_combo.count() == 2
    assert page.revision_label.text() == "修订 7"
    assert page.face_0_products.text() == "normal: tea"
    assert page.store.map_name == "arena"
    assert page.map_scale_label.text() == "0.050 m/px · 100×80 px"

    shelf_item = page.canvas._entity_items[("shelf", 1)]
    assert shelf_item.width_px == pytest.approx(0.8 / 0.05)
    assert shelf_item.height_px == pytest.approx(1.8 / 0.05)
    origin_item = page.canvas._map_origin_item
    assert origin_item is not None
    assert (origin_item.pos().x(), origin_item.pos().y()) == pytest.approx((50.0, 40.0))
    assert math.hypot(origin_item.x_axis.x(), origin_item.x_axis.y()) == pytest.approx(20.0)
    assert math.hypot(origin_item.y_axis.x(), origin_item.y_axis.y()) == pytest.approx(20.0)

    page.canvas.zoom_in()
    assert page.canvas._zoom_factor == pytest.approx(1.25)
    page.canvas.fit_map()
    assert page.canvas._zoom_factor == pytest.approx(1.0)

    page.world_x.setValue(1.25)
    page.world_y.setValue(-0.5)
    page.yaw_degrees.setValue(30.0)
    page._pose_controls_changed()

    entity = page.store.entity("shelf", 1)
    assert entity.world_x == pytest.approx(1.25)
    assert entity.world_y == pytest.approx(-0.5)
    assert entity.yaw == pytest.approx(0.5235987756)
    assert page.store.can_undo
    page.close()


def test_registration_page_selects_named_map_from_dropdown(tmp_path: Path):
    app = QApplication.instance() or QApplication([])
    _write_named_map(tmp_path, "supermarket_test_08")
    _write_named_map(tmp_path, "supermarket_test_05")

    page = SceneRegistrationPage(
        None,
        map_root=tmp_path,
        map_name="supermarket_test_08",
    )
    app.processEvents()

    assert page.map_root_combo.currentText() == str(tmp_path)
    assert [
        page.map_name_combo.itemText(index)
        for index in range(page.map_name_combo.count())
    ] == ["supermarket_test_05", "supermarket_test_08"]
    assert page.map_name_combo.currentText() == "supermarket_test_08"
    assert page.map_name == "supermarket_test_08"
    assert page.yaml_path == (
        tmp_path / "2d" / "supermarket_test_08" / "map.yaml"
    ).resolve()
    page.close()
