from pathlib import Path
import sys

from docx import Document
from docx.shared import Cm
from PIL import Image
import pytest


sys.path.insert(0, str(Path(__file__).resolve().parent))
from build_research_review import PROFILES, build, fit_images, provenance_sources


def test_portrait_platform_photo_fits_page(tmp_path):
    picture = tmp_path / "portrait.png"
    Image.new("RGB", (100, 300), "white").save(picture)
    document = Document()
    image = document.add_picture(str(picture), width=Cm(15.5))
    ratio = image.width / image.height
    fit_images(document)
    assert image.height == Cm(17)
    assert image.width / image.height == pytest.approx(ratio, abs=1.0e-6)


def test_landscape_image_is_not_upscaled(tmp_path):
    picture = tmp_path / "landscape.png"
    Image.new("RGB", (300, 100), "white").save(picture)
    document = Document()
    image = document.add_picture(str(picture), width=Cm(15.5))
    before = (image.width, image.height)
    fit_images(document)
    assert (image.width, image.height) == before


def test_provenance_tracks_current_and_original_workspaces():
    sources = [str(path) for path in provenance_sources()]
    assert any("openflex_shop/" in path for path in sources)
    assert any("openflex_0/" in path for path in sources)
    assert len(sources) == len(set(sources))


def test_export_stem_cannot_escape_output(tmp_path):
    output = tmp_path / "output"
    with pytest.raises(ValueError, match="filename"):
        build(tmp_path, output, tmp_path, stem="../outside")
    assert not output.exists()


def test_retail_profile_excludes_retired_vla_sources():
    sources = [str(path) for path in provenance_sources("retail")]
    assert not any("openflex_0/" in path for path in sources)
    assert any("action_journal.py" in path for path in sources)
    assert "观测表征不再作为必做研究主线" in PROFILES["retail"][2]


def test_unknown_profile_fails_before_export(tmp_path):
    output = tmp_path / "output"
    with pytest.raises(ValueError, match="profile"):
        build(tmp_path, output, tmp_path, profile="invalid")
    assert not output.exists()


def test_retail_profile_cannot_relabel_default_midterm_sources(tmp_path):
    output = tmp_path / "output"
    with pytest.raises(ValueError, match="explicit source"):
        build(tmp_path, output, tmp_path, profile="retail")
    assert not output.exists()


def test_default_profile_requires_explicit_source_after_old_drafts_removed(tmp_path):
    output = tmp_path / "output"
    with pytest.raises(ValueError, match="explicit source"):
        build(tmp_path, output, tmp_path)
    assert not output.exists()
