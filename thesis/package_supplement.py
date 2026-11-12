import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile


ROOT = Path(__file__).resolve().parents[1]
RUNS = ROOT / 'local_dependencies/openflex_roboos/runs'
HANDOFF = ROOT / 'local_dependencies/openflex_roboos/handoff'
BATCHES = ('speed_paired_v1_20261111', 'speed_paired_v1_1_20261111', 'speed_paired_v1_2_20261111')


def checksum(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def save(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def copy_tree(source, destination):
    if not source.is_dir():
        raise FileNotFoundError(source)
    for path in sorted(source.rglob('*')):
        if path.is_file() and not path.is_symlink() and not {'__pycache__', '.pytest_cache'}.intersection(path.parts):
            target = destination / path.relative_to(source)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, target)


def package(output):
    if output.parent != HANDOFF:
        raise ValueError('Supplement output must be a new directory under the existing handoff directory')
    output.mkdir(exist_ok=False)
    original = HANDOFF / 'openflex_handoff_20261111.tar.gz'
    original_digest = 'c49157b276af926f6246f62083996fd116130b1d806e0258303ee4e0b54f82c2'
    if checksum(original) != original_digest:
        raise RuntimeError('Original evidence archive changed')
    baseline = output / 'baseline_handoff'
    baseline.mkdir()
    shutil.copy2(original, baseline / original.name)
    shutil.copy2(original.with_suffix(original.suffix + '.sha256'), baseline)
    referenced = set()
    batch_reports = []
    for name in BATCHES:
        batch = RUNS / name
        protocol = json.loads((batch / 'frozen_protocol.json').read_text())
        seal = json.loads((batch / 'freeze_seal.json').read_text())
        if checksum(batch / 'frozen_protocol.json') != seal['frozen_protocol_sha256']:
            raise RuntimeError(f'Protocol seal changed: {name}')
        verified = None
        frozen_sources = batch / 'frozen_sources.tar.gz'
        if frozen_sources.exists():
            with tarfile.open(frozen_sources) as archive:
                for filename, digest in protocol['source_sha256'].items():
                    if hashlib.sha256(archive.extractfile(filename).read()).hexdigest() != digest:
                        raise RuntimeError(f'Frozen source hash mismatch: {name}/{filename}')
            verified = len(protocol['source_sha256'])
        copy_tree(batch, output / 'runs' / name)
        for definition in protocol['protocol']['trials']:
            trial_path = batch / definition['id'] / 'trial_result.json'
            if not trial_path.exists():
                continue
            trial = json.loads(trial_path.read_text())
            for operation in trial.get('operations', []):
                candidate = RUNS / operation['request_id']
                if candidate.is_dir():
                    referenced.add(candidate)
                for event in operation.get('events', []):
                    report = event.get('data', {}).get('report')
                    if report:
                        candidate = Path(report).resolve().parent
                        if candidate.parent != RUNS:
                            raise ValueError('Unexpected planning report outside the reviewed runs directory')
                        referenced.add(candidate)
        batch_reports.append({'batch': name, 'frozen_source_files_verified': verified,
                              'source_archive_missing_not_reconstructed': verified is None,
                              'summary': json.loads((batch / 'summary.json').read_text())})
    for path in RUNS.glob('safe_stop_20261111_*'):
        if 'safe_stop_20261111_150000' <= path.name <= 'safe_stop_20261111_163000':
            referenced.add(path)
    for name in ('speed_regression_20261111', 'speed_ack_regression_20261111', 'final_offline_regression_20261111'):
        referenced.add(RUNS / name)
    for path in sorted(referenced):
        copy_tree(path, output / 'runs' / path.name)
    copy_tree(ROOT / 'local_dependencies/openflex_thesis', output / 'documents_and_analysis')
    suffixes = {'.py', '.yaml', '.yml', '.json', '.sh', '.patch', '.xml', '.msg', '.srv', '.action', '.cfg', '.md', '.txt'}
    sources = set(ROOT.glob('*.md')) | {ROOT / '.env.example'}
    for directory in ('scripts', 'thesis', 'openflex_ws/src/openflex_thesis'):
        sources.update(path for path in (ROOT / directory).rglob('*') if path.is_file() and path.suffix in suffixes
                       and not {'__pycache__', '.pytest_cache', 'build', 'install', 'log'}.intersection(path.parts))
    for path in sorted(sources):
        if path.is_symlink():
            raise ValueError(f'Unexpected source symlink: {path}')
        target = output / 'offline_working_tree_source' / path.relative_to(ROOT)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, target)
    save(output / 'offline_source_hashes.json', {str(path.relative_to(ROOT)): checksum(path) for path in sorted(sources)})
    state = json.loads(subprocess.check_output([sys.executable, str(ROOT / 'scripts/openflex_runtime.py'), 'status'], text=True))
    save(output / 'runtime_status_read_only.json', state)
    shutil.copy2(ROOT / 'local_dependencies/openflex_roboos/navigation_operations.jsonl', output / 'operation_journal_snapshot.jsonl')
    save(output / 'provenance.json', {
        'created_at': datetime.now().astimezone().isoformat(), 'physical_verification': False,
        'git_head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'git_status': subprocess.check_output(['git', 'status', '--short'], cwd=ROOT, text=True),
        'baseline_archive_sha256': original_digest, 'batches': batch_reports,
        'includes_models_virtualenvs_or_secrets': False, 'motion_or_unlock_sent': False,
        'working_tree_source_is_not_the_source_used_by_frozen_trials': True,
        'requires': 'Base repository, model weights and existing dependencies; not an offline installation'})
    shutil.copy2(ROOT / 'thesis/README_CN.md', output / 'HANDOFF_README_CN.md')
    paths = sorted(path for path in output.rglob('*') if path.is_file())
    (output / 'SHA256SUMS').write_text(''.join(f'{checksum(path)}  {path.relative_to(output)}\n' for path in paths))
    subprocess.run(['sha256sum', '-c', 'SHA256SUMS', '--quiet'], cwd=output, check=True)
    archive_path = output.with_suffix('.tar.gz')
    if archive_path.exists():
        raise FileExistsError(archive_path)
    with tarfile.open(archive_path, 'w:gz') as archive:
        archive.add(output, arcname=output.name)
    with tarfile.open(archive_path) as archive:
        for path in paths + [output / 'SHA256SUMS']:
            if hashlib.sha256(archive.extractfile(f'{output.name}/{path.relative_to(output)}').read()).hexdigest() != checksum(path):
                raise RuntimeError(f'Archive content mismatch: {path}')
    digest = checksum(archive_path)
    archive_path.with_suffix(archive_path.suffix + '.sha256').write_text(f'{digest}  {archive_path.name}\n')
    print(json.dumps({'archive': str(archive_path), 'files': len(paths) + 1,
                      'archive_bytes': archive_path.stat().st_size, 'archive_sha256': digest,
                      'checksums_and_archive_members_verified': True}, ensure_ascii=False, indent=2))


def main():
    parser = argparse.ArgumentParser(description='Read-only packaging of documents, negative evidence and offline fixes')
    parser.add_argument('--output', type=Path, required=True)
    arguments = parser.parse_args()
    package(arguments.output.resolve())


if __name__ == '__main__':
    main()
