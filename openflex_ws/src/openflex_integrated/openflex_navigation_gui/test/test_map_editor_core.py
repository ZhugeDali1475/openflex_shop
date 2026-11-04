from pathlib import Path

from openflex_navigation_gui.map_editor_core import (
    FREE,
    OCCUPIED,
    UNKNOWN,
    GridMap,
    edit_summary,
    fill_component,
    paint_circle,
    paint_line,
    read_pgm,
    save_with_backup,
    write_pgm,
)


def test_pgm_round_trip(tmp_path: Path):
    source = GridMap(3, 2, bytearray([FREE, OCCUPIED, UNKNOWN, 1, 2, 3]))
    path = tmp_path / "map.pgm"
    write_pgm(path, source)
    loaded = read_pgm(path)
    assert loaded.width == 3
    assert loaded.height == 2
    assert loaded.pixels == source.pixels


def test_brush_line_and_component_tools():
    grid = GridMap(7, 7, bytearray([FREE] * 49))
    assert paint_circle(grid, 3, 3, 1, OCCUPIED) == 5
    assert fill_component(grid, 3, 3, FREE) == 5
    assert paint_line(grid, (1, 1), (5, 1), 0, UNKNOWN) == 5
    assert grid.pixels.count(UNKNOWN) == 5


def test_save_creates_backup_and_reports_safety_transition(tmp_path: Path):
    path = tmp_path / "map.pgm"
    original = GridMap(2, 2, bytearray([OCCUPIED, UNKNOWN, FREE, FREE]))
    write_pgm(path, original)
    edited = GridMap(2, 2, bytearray([FREE, FREE, FREE, OCCUPIED]))
    backup = save_with_backup(path, edited)
    assert backup is not None and backup.is_file()
    assert read_pgm(backup).pixels == original.pixels
    assert read_pgm(path).pixels == edited.pixels
    assert edit_summary(bytes(original.pixels), bytes(edited.pixels)) == {
        "changed": 3,
        "occupied_to_free": 1,
        "unknown_to_free": 1,
        "to_occupied": 1,
        "to_unknown": 0,
    }
