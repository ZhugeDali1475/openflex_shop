import importlib.util
from pathlib import Path

import pytest


MODULE_PATH = Path(__file__).resolve().parents[2] / 'openflex_ws/src/openflex_thesis/openflex_roboos_bridge/openflex_roboos_bridge/support_contacts.py'
SPEC = importlib.util.spec_from_file_location('support_contacts', MODULE_PATH)
SUPPORT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SUPPORT)


def test_only_wheel_floor_pairs_are_allowed():
    names, rows = SUPPORT.allow_wheel_floor_support(['hand', 'arm'], [[False, True], [True, False]], 'floor')
    floor = names.index('floor')
    assert {name for name, allowed in zip(names, rows[floor]) if allowed} == {
        'fl_wheel_link', 'fr_wheel_link', 'bl_wheel_link', 'br_wheel_link'}
    assert rows[0][1] and rows[1][0]
    assert not rows[0][floor] and not rows[floor][1]
    assert all(rows[index][floor] == rows[floor][index] for index in range(len(names)))


def test_existing_broad_floor_exclusion_is_removed():
    names, rows = SUPPORT.allow_wheel_floor_support(['hand', 'floor'], [[False, True], [True, True]], 'floor')
    assert not rows[0][1] and not rows[1][0] and not rows[1][1]


@pytest.mark.parametrize('names,rows', [(['hand', 'hand'], [[False] * 2] * 2),
    (['hand'], []), (['hand'], [[False, False]])])
def test_malformed_matrices_are_rejected(names, rows):
    with pytest.raises(ValueError):
        SUPPORT.allow_wheel_floor_support(names, rows, 'floor')
