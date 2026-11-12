import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

from docx import Document
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm, Pt

from academic_support import add_equation, validate_math_roundtrip, write_counts


ROOT = Path(__file__).resolve().parents[1]
TITLE = '基于大模型的移动机器人分层任务规划与技能调度系统设计与验证'
STEM = '移动机器人分层任务规划与技能调度_论文初稿_v1'
BATCH = ROOT / 'local_dependencies/openflex_roboos/runs/evaluation_small_v1_1_20261111'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def experiment_data(batch):
    frozen = json.loads((batch / 'frozen_protocol.json').read_text())
    seal = json.loads((batch / 'freeze_seal.json').read_text())
    if digest(batch / 'frozen_protocol.json') != seal['frozen_protocol_sha256']:
        raise ValueError('Frozen experiment seal mismatch')
    summary = json.loads((batch / 'summary.json').read_text())
    if (summary['scheduled'], summary['collected'], summary['successes']) != (3, 3, 3):
        raise ValueError('This draft requires the declared three-trial evidence, not substituted results')
    trials = []
    labels = ['同义指令', '首次观察拒绝', '固定顺序对照']
    for definition, label in zip(frozen['protocol']['trials'], labels):
        folder = batch / definition['id']
        report = json.loads((folder / 'trial_result.json').read_text())
        if not (report['success'] and report['independent_completion'] and report['measured_hold']):
            raise ValueError('Independent measured completion missing')
        if len(report['operations']) != 4 or not all(
                operation['status'] == 'succeeded' and operation['stopped_confirmed'] is True
                for operation in report['operations']):
            raise ValueError('Four stopped motion terminals required')
        samples = [json.loads(line) for line in (folder / 'resources.jsonl').read_text().splitlines()]
        trials.append({'id': definition['id'], 'label': label,
                       'execution_seconds': report['execution_seconds'],
                       'worker_inferences': report['worker_inferences'],
                       'tool_calls': report['tool_calls'], 'motion_calls': report['motion_calls'],
                       'frames': report['video']['frames'], 'samples': len(samples),
                       'ram_min_mib': min(item['memory_kib']['MemAvailable'] / 1024 for item in samples),
                       'swap_max_mib': max((item['memory_kib']['SwapTotal'] - item['memory_kib']['SwapFree']) / 1024 for item in samples),
                       'gpu_max_mib': max(int(item['gpu_memory_mib'].split(',')[0]) for item in samples if item.get('gpu_memory_mib'))})
    return {'batch': str(batch.relative_to(ROOT)), 'summary_sha256': digest(batch / 'summary.json'),
            'each_condition_n': 1, 'physical_verification': False, 'trials': trials}


def expand_tables(text, evidence):
    results = ['| 条件 | 独立完成 | Worker推理/工具 | 运动 | 耗时/s | 视频帧 |',
               '|---|---|---|---|---|---|']
    resources = ['| 条件 | 样本数 | 最低可用RAM/MiB | 最高swap/MiB | 整卡显存/MiB |',
                 '|---|---|---|---|---|']
    for trial in evidence['trials']:
        results.append(f"| {trial['label']} | 通过 | {trial['worker_inferences']}/{trial['tool_calls']} | {trial['motion_calls']} | {trial['execution_seconds']:.1f} | {trial['frames']} |")
        resources.append(f"| {trial['label']} | {trial['samples']} | {trial['ram_min_mib']:.1f} | {trial['swap_max_mib']:.1f} | {trial['gpu_max_mib']} |")
    return text.replace('{{RESULTS_TABLE}}', '\n'.join(results)).replace('{{RESOURCES_TABLE}}', '\n'.join(resources))


def font(style, east_asian='宋体', size=12):
    style.font.name = 'Times New Roman'
    style.font.size = Pt(size)
    style.element.get_or_add_rPr().rFonts.set(qn('w:eastAsia'), east_asian)


def inline(paragraph, text):
    for part in re.split(r'(\*\*.*?\*\*|`[^`]+`)', text):
        if not part:
            continue
        if part.startswith('**') and part.endswith('**'):
            paragraph.add_run(part[2:-2]).bold = True
        elif part.startswith('`') and part.endswith('`'):
            run = paragraph.add_run(part[1:-1])
            run.font.name = 'DejaVu Sans Mono'
            run.font.size = Pt(9)
        else:
            paragraph.add_run(part)


def table(document, lines):
    rows = [[cell.strip() for cell in line.strip().strip('|').split('|')] for line in lines]
    rows = [row for row in rows if not all(re.fullmatch(r':?-+:?', cell) for cell in row)]
    count = len(rows[0])
    if any(len(row) != count for row in rows):
        raise ValueError('Inconsistent table columns')
    result = document.add_table(rows=0, cols=count)
    result.style = 'Table Grid'
    result.alignment = WD_TABLE_ALIGNMENT.CENTER
    for row_index, cells in enumerate(rows):
        row = result.add_row()
        for cell, text in zip(row.cells, cells):
            paragraph = cell.paragraphs[0]
            paragraph.paragraph_format.first_line_indent = Cm(0)
            paragraph.paragraph_format.space_after = Pt(4)
            paragraph.paragraph_format.line_spacing = 1.1
            inline(paragraph, text)
            for run in paragraph.runs:
                run.font.size = Pt(9)
                if row_index == 0:
                    run.bold = True
        if row_index == 0:
            repeat = OxmlElement('w:tblHeader')
            row._tr.get_or_add_trPr().append(repeat)
    document.add_paragraph().paragraph_format.space_after = Pt(2)
    return result


def setup(document):
    section = document.sections[0]
    section.page_height, section.page_width = Cm(29.7), Cm(21)
    section.top_margin = section.bottom_margin = Cm(2.5)
    section.left_margin, section.right_margin = Cm(2.7), Cm(2.3)
    normal = document.styles['Normal']
    font(normal)
    normal.paragraph_format.line_spacing = 1.5
    normal.paragraph_format.first_line_indent = Cm(0.84)
    normal.paragraph_format.space_after = Pt(5)
    for name, size in [('Title', 22), ('Heading 1', 18), ('Heading 2', 14), ('Heading 3', 12)]:
        style = document.styles[name]
        font(style, '黑体', size)
        style.paragraph_format.first_line_indent = Cm(0)
        style.paragraph_format.keep_with_next = True
    font(document.styles['Caption'], size=10)
    document.styles['Caption'].paragraph_format.first_line_indent = Cm(0)
    document.styles['Caption'].paragraph_format.alignment = WD_ALIGN_PARAGRAPH.CENTER
    for paragraph in (section.header.paragraphs[0], section.footer.paragraphs[0]):
        paragraph.paragraph_format.first_line_indent = Cm(0)
        paragraph.alignment = WD_ALIGN_PARAGRAPH.CENTER
    section.header.paragraphs[0].add_run('OpenFlex 移动取送研究 · 学术修改稿（待审核）').font.size = Pt(9)
    page = OxmlElement('w:fldSimple')
    page.set(qn('w:instr'), 'PAGE')
    section.footer.paragraphs[0]._p.append(page)


def cover(document, title):
    document.add_paragraph('毕业设计论文 · 学术修改稿（待审核）', style='Subtitle')
    heading = document.add_paragraph(title, style='Title')
    heading.alignment = WD_ALIGN_PARAGRAPH.CENTER
    for text in ['作者：________________', '学号：________________', '学院／专业：________________',
                 '指导教师：________________', '编制日期：2026年11月11日']:
        paragraph = document.add_paragraph(text)
        paragraph.paragraph_format.first_line_indent = Cm(0)
        paragraph.paragraph_format.space_before = Pt(12)
    document.add_paragraph('用户授权暂定题目；非导师审批版本。学校封面、声明及格式需另行核对。')
    document.add_page_break()


def render(text, document, assets):
    lines = text.splitlines()
    index = 0
    while index < len(lines):
        line = lines[index].strip()
        if not line or line.startswith('# '):
            index += 1
            continue
        if line.startswith('```'):
            index += 1
            while index < len(lines) and not lines[index].startswith('```'):
                paragraph = document.add_paragraph()
                paragraph.paragraph_format.first_line_indent = Cm(0)
                paragraph.paragraph_format.line_spacing = 1.1
                run = paragraph.add_run(lines[index])
                run.font.name, run.font.size = 'DejaVu Sans Mono', Pt(8)
                index += 1
        elif line.startswith('|'):
            group = []
            while index < len(lines) and lines[index].strip().startswith('|'):
                group.append(lines[index])
                index += 1
            table(document, group)
            continue
        elif line.startswith('## '):
            paragraph = document.add_heading(line[3:], level=1)
            paragraph.paragraph_format.page_break_before = True
        elif line.startswith('### '):
            document.add_heading(line[4:], level=2)
        elif line.startswith('#### '):
            document.add_heading(line[5:], level=3)
        elif line.startswith('$$'):
            add_equation(document, line)
        elif line.startswith('!['):
            match = re.fullmatch(r'!\[(.+)\]\((.+)\)', line)
            if match is None:
                raise ValueError('Malformed image')
            image = assets / Path(match.group(2)).name
            if not image.is_file():
                raise FileNotFoundError(image)
            paragraph = document.add_paragraph()
            paragraph.paragraph_format.first_line_indent = Cm(0)
            paragraph.alignment = WD_ALIGN_PARAGRAPH.CENTER
            paragraph.add_run().add_picture(str(image), width=Cm(15.5))
            document.add_paragraph(match.group(1), style='Caption')
        elif line.startswith('- '):
            paragraph = document.add_paragraph(style='List Bullet')
            paragraph.paragraph_format.first_line_indent = Cm(0)
            inline(paragraph, line[2:])
        else:
            paragraph = document.add_paragraph()
            inline(paragraph, line)
        index += 1


def figures(assets, evidence):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plot
    from matplotlib.font_manager import FontProperties

    font_path = subprocess.check_output(['fc-match', 'Noto Sans CJK SC', '-f', '%{file}'], text=True)
    chinese = FontProperties(fname=font_path)
    values = [trial['execution_seconds'] for trial in evidence['trials']]
    figure, axis = plot.subplots(figsize=(9, 4.7))
    bars = axis.bar(range(3), values, color=['#356e99', '#b87932', '#698876'], width=0.6)
    axis.set_xticks(range(3), [trial['label'] for trial in evidence['trials']], fontproperties=chinese)
    axis.set_ylabel('执行耗时 / s', fontproperties=chinese)
    axis.set_title('冻结修正版：每条件仅一次（非统计比较）', fontproperties=chinese)
    axis.set_ylim(0, 630)
    for bar, value in zip(bars, values):
        axis.text(bar.get_x() + bar.get_width() / 2, value + 8, f'{value:.1f}', ha='center')
    figure.tight_layout()
    figure.savefig(assets / 'execution_seconds.png', dpi=180)
    plot.close(figure)
    shutil.copy2(BATCH / 'model_synonym/video/first_frame.png', assets / 'measured_first_frame.png')


def convert(path, output, extension, filter_name):
    with tempfile.TemporaryDirectory(prefix='openflex-document-') as profile:
        subprocess.run(['libreoffice', '-env:UserInstallation=' + Path(profile).as_uri(),
                        '--headless', '--convert-to', f'{extension}:{filter_name}',
                        '--outdir', str(output), str(path)], check=True, timeout=150)
    result = output / f'{path.stem}.{extension}'
    if not result.is_file() or result.stat().st_size < 1024:
        raise RuntimeError(f'Document conversion missing: {result}')
    return result


def build(output, source, stem=STEM):
    output.mkdir(parents=True, exist_ok=False)
    assets = output / 'assets'
    assets.mkdir()
    evidence = experiment_data(BATCH)
    figures(assets, evidence)
    manuscript = source.read_text()
    (output / '论文源稿_构建快照.md').write_text(manuscript)
    source_sha256 = hashlib.sha256(manuscript.encode()).hexdigest()
    text = expand_tables(manuscript, evidence)
    if 'assets/method_architecture.png' in text or 'assets/ready_counterexample.png' in text:
        from validate_method_counterexamples import evaluate, make_figures
        make_figures(assets)
        (output / 'counterexample.json').write_text(json.dumps(evaluate(), ensure_ascii=False, indent=2) + '\n')
    title_match = re.search(r'^# (.+)$', text, re.MULTILINE)
    if title_match is None:
        raise ValueError('Manuscript title missing')
    title = title_match.group(1)
    counts = write_counts(output, text)
    document = Document()
    setup(document)
    cover(document, title)
    document.add_heading('目录（可在 Word/WPS 中更新）', level=1)
    contents = document.add_paragraph()
    contents.paragraph_format.first_line_indent = Cm(0)
    field = OxmlElement('w:fldSimple')
    field.set(qn('w:instr'), 'TOC \\o "1-3" \\h \\z \\u')
    contents._p.append(field)
    document.add_paragraph('若目录为空，请在 Word/WPS 中更新整个目录；正文已设置标题层级。')
    render(text, document, assets)
    document.core_properties.title = title
    document.core_properties.subject = '隔离仿真工程证据版论文初稿；非学校审批版本'
    document.core_properties.author = ''
    document.core_properties.created = datetime.now()
    document.core_properties.modified = datetime.now()
    document_path = output / f'{stem}.docx'
    document.save(document_path)
    legacy = convert(document_path, output, 'doc', 'MS Word 97')
    if legacy.read_bytes()[:8] != bytes.fromhex('D0CF11E0A1B11AE1'):
        raise RuntimeError('Not a real OLE Word 97-2003 document')
    legacy_preview = output / 'legacy_preview'
    legacy_preview.mkdir()
    legacy_pdf = convert(legacy, legacy_preview, 'pdf', 'writer_pdf_Export')
    pdf = convert(document_path, output, 'pdf', 'writer_pdf_Export')
    validation = output / 'validation'
    validation.mkdir()
    roundtrip = convert(legacy, validation, 'docx', 'Office Open XML Text')
    restored = Document(roundtrip)
    math_validation = validate_math_roundtrip(document, restored)
    restored_text = '\n'.join(paragraph.text for paragraph in restored.paragraphs)
    restored_text += '\n'.join(cell.text for item in restored.tables for row in item.rows for cell in row.cells)
    required = [title, '第1章 绪论', '第6章 总结与展望', '附录B', '494.8', '532.4', '484.9', '337', '14676.0']
    if not all(value in restored_text for value in required):
        raise RuntimeError('DOC round-trip content validation failed')
    if len(document.tables) != len(restored.tables) or len(document.inline_shapes) != len(restored.inline_shapes):
        raise RuntimeError('DOC round-trip lost tables or images')
    pdf_text = subprocess.check_output(['pdftotext', str(pdf), '-'], text=True)
    compact_pdf_text = re.sub(r'\s+', '', pdf_text)
    if not all(value in compact_pdf_text for value in ['第1章', '第6章', '494.8', '532.4', '484.9']):
        raise RuntimeError('DOC PDF text validation failed')
    legacy_text = re.sub(r'\s+', '', subprocess.check_output(['pdftotext', str(legacy_pdf), '-'], text=True))
    if not all(value in legacy_text for value in ['第1章', '第6章', '494.8', '532.4', '484.9']):
        raise RuntimeError('Legacy DOC PDF text validation failed')
    report = {'created_at': datetime.now().astimezone().isoformat(), 'title': title,
              'source': str(source.relative_to(ROOT)), 'source_sha256': source_sha256,
              'source_unchanged_after_build': digest(source) == source_sha256,
              'evidence': evidence, 'format': 'Word 97-2003 OLE, not renamed DOCX or HTML',
              'roundtrip_validated': True, 'tables': len(restored.tables),
              'word_counts': counts,
              'docx_math_objects': len(document._element.xpath('//m:oMath')),
              'roundtrip_math_objects': len(restored._element.xpath('//m:oMath')),
              'math_validation': math_validation,
              'pdf_source': 'DOCX direct export for vector equations; legacy DOC PDF retained separately',
              'legacy_formula_note': 'Legacy DOC may display low-resolution equation previews in some editors; DOCX contains native Office Math. Token round-trip is checked but visual behavior in Word/WPS is not certified.',
              'images': len(restored.inline_shapes), 'paragraphs': len(restored.paragraphs),
              'pdf_info': subprocess.check_output(['pdfinfo', str(pdf)], text=True),
              'files': {path.name: {'bytes': path.stat().st_size, 'sha256': digest(path)}
                        for path in (document_path, legacy, pdf)}}
    (output / 'document_manifest.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    (output / '论文正文_已展开.md').write_text(text)
    print(json.dumps({'output': str(output), 'doc': str(legacy), 'validated': True,
                      'tables': report['tables'], 'images': report['images']}, ensure_ascii=False, indent=2))


def main():
    parser = argparse.ArgumentParser(description='Build editable thesis documents without calling ROS or a model')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--stem', default=STEM)
    arguments = parser.parse_args()
    build(arguments.output.resolve(), arguments.source.resolve(), arguments.stem)


if __name__ == '__main__':
    main()
