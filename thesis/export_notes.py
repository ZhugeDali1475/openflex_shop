import argparse
from datetime import datetime
import json
from pathlib import Path

from docx import Document

from build_thesis import convert, digest, render, setup


def main():
    parser = argparse.ArgumentParser(description='Export editable project notes as genuine legacy Word documents')
    parser.add_argument('source', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    arguments = parser.parse_args()
    output = arguments.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    source = arguments.source.resolve()
    text = source.read_text()
    title = text.splitlines()[0].removeprefix('# ')
    document = Document()
    setup(document)
    document.sections[0].header.paragraphs[0].text = 'OpenFlex 项目材料 · 工程证据版'
    document.add_heading(title, level=0)
    render(text, document, source.parent / 'assets')
    document.core_properties.title = title
    document.core_properties.author = ''
    document.core_properties.created = datetime.now()
    document.core_properties.modified = datetime.now()
    path = output / f'{source.stem}.docx'
    document.save(path)
    legacy = convert(path, output, 'doc', 'MS Word 97')
    if legacy.read_bytes()[:8] != bytes.fromhex('D0CF11E0A1B11AE1'):
        raise RuntimeError('Legacy Word signature missing')
    validation = output / 'validation'
    validation.mkdir()
    roundtrip = convert(legacy, validation, 'docx', 'Office Open XML Text')
    checked = Document(roundtrip)
    if len(checked.tables) != len(document.tables) or not any(title in paragraph.text for paragraph in checked.paragraphs):
        raise RuntimeError('Round-trip lost title or editable tables')
    report = {'source': str(source), 'source_sha256': digest(source),
              'doc_sha256': digest(legacy), 'roundtrip_validated': True,
              'editable_tables': len(checked.tables)}
    (output / 'manifest.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(legacy)


if __name__ == '__main__':
    main()
