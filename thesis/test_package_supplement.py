import hashlib
from pathlib import Path
import sys

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parent))
from package_supplement import checksum, copy_tree, package


def test_checksum_is_exact_for_files_larger_than_one_chunk(tmp_path):
    path = tmp_path / 'evidence.bin'
    content = b'OpenFlex' * 200000
    path.write_bytes(content)
    assert checksum(path) == hashlib.sha256(content).hexdigest()


def test_copy_preserves_evidence_and_does_not_follow_symlinks_or_caches(tmp_path):
    source = tmp_path / 'source'
    source.mkdir()
    (source / 'result.json').write_text('{"status":"unknown"}')
    cache = source / '__pycache__'
    cache.mkdir()
    (cache / 'skip.pyc').write_bytes(b'cache')
    outside = tmp_path / 'outside'
    outside.write_text('do not package')
    (source / 'link').symlink_to(outside)
    destination = tmp_path / 'copied'
    copy_tree(source, destination)
    assert (destination / 'result.json').read_text() == '{"status":"unknown"}'
    assert sorted(path.name for path in destination.rglob('*')) == ['result.json']


def test_missing_evidence_cannot_be_silently_omitted(tmp_path):
    with pytest.raises(FileNotFoundError):
        copy_tree(tmp_path / 'missing', tmp_path / 'output')


def test_output_outside_handoff_is_rejected_before_any_write(tmp_path):
    output = tmp_path / 'output'
    with pytest.raises(ValueError, match='handoff'):
        package(output)
    assert not output.exists()
