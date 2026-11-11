from pathlib import Path
import runpy
from unittest.mock import patch

import pytest


@pytest.fixture
def package_metadata(monkeypatch):
    package_root = Path(__file__).resolve().parents[1]
    monkeypatch.chdir(package_root)
    with patch("setuptools.setup") as setup:
        runpy.run_path(str(package_root / "setup.py"))
    return package_root, setup.call_args.kwargs


def test_retired_recovery_commands_are_not_registered(package_metadata):
    metadata = package_metadata[1]
    commands = {
        entry.split("=", 1)[0].strip()
        for entry in metadata["entry_points"]["console_scripts"]
    }
    assert not commands & {
        "train_recovery", "run_recovery_benchmark",
        "generate_recovery_matrix", "report_recovery",
    }
    required = {"audit_moveit_path", "probe_plan_path", "thesis_preflight"}
    assert required <= commands


def test_registered_command_modules_exist(package_metadata):
    package_root, metadata = package_metadata
    for entry in metadata["entry_points"]["console_scripts"]:
        module = entry.split("=", 1)[1].split(":", 1)[0].strip()
        module_path = package_root.joinpath(*module.split("."))
        assert module_path.with_suffix(".py").is_file()


def test_packaged_assets_exist_without_learning_extra(package_metadata):
    package_root, metadata = package_metadata
    assert "learning" not in metadata.get("extras_require", {})
    for destination, sources in metadata["data_files"]:
        for source in sources:
            assert (package_root / source).is_file()
