import argparse
import hashlib
import json
from pathlib import Path
import re

from pptx import Presentation
from pptx.dml.color import RGBColor
from pptx.util import Cm, Pt

from build_thesis import BATCH, convert


def main():
    parser = argparse.ArgumentParser(description='Editable slides from the evidence-based defense outline')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--source', type=Path, required=True)
    arguments = parser.parse_args()
    output = arguments.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    source = arguments.source.resolve()
    text = source.read_text()
    pages = re.findall(r'^## 第\d+页 ([^\n]+)\n(.*?)(?=^## |\Z)', text, re.M | re.S)
    if len(pages) != 12:
        raise RuntimeError('Twelve declared defense slides required')
    presentation = Presentation()
    presentation.slide_width, presentation.slide_height = Cm(33.87), Cm(19.05)
    for index, (title, body) in enumerate(pages, 1):
        slide = presentation.slides.add_slide(presentation.slide_layouts[6])
        slide.background.fill.solid()
        slide.background.fill.fore_color.rgb = RGBColor.from_string('F4F7FA')
        heading = slide.shapes.add_textbox(Cm(1.4), Cm(0.8), Cm(30), Cm(2))
        paragraph = heading.text_frame.paragraphs[0]
        paragraph.text = f'{index:02d}  {title}'
        paragraph.font.name, paragraph.font.size = 'Noto Sans CJK SC', Pt(29)
        paragraph.font.bold = True
        paragraph.font.color.rgb = RGBColor.from_string('183E60')
        width = Cm(18.8) if index == 6 else Cm(30)
        box = slide.shapes.add_textbox(Cm(1.6), Cm(3.0), width, Cm(12.8))
        box.text_frame.word_wrap = True
        paragraphs = [part.strip() for part in body.split('\n\n') if part.strip()]
        if index == 1:
            paragraphs.insert(0, '基于大模型的移动机器人分层任务规划与技能调度系统设计与验证')
        for paragraph_index, content in enumerate(paragraphs):
            paragraph = box.text_frame.paragraphs[0] if paragraph_index == 0 else box.text_frame.add_paragraph()
            paragraph.text = content
            paragraph.font.name, paragraph.font.size = 'Noto Sans CJK SC', Pt(22)
            paragraph.font.color.rgb = RGBColor.from_string('233443')
            paragraph.space_after = Pt(24)
        if index == 6:
            image = BATCH / 'model_synonym/video/first_frame.png'
            slide.shapes.add_picture(str(image), Cm(21), Cm(4), width=Cm(11.2))
            label = slide.shapes.add_textbox(Cm(21), Cm(12.7), Cm(11.2), Cm(2))
            label.text_frame.word_wrap = True
            label.text_frame.paragraphs[0].text = '实测镜像首帧；非交付完成证明'
            label.text_frame.paragraphs[0].font.size = Pt(14)
        footer = slide.shapes.add_textbox(Cm(1.5), Cm(17.5), Cm(30), Cm(0.8))
        footer.text_frame.paragraphs[0].text = f'OpenFlex · 答辩初稿 · 仅隔离仿真证据，非学校审核版    {index}/12'
        footer.text_frame.paragraphs[0].font.size = Pt(12)
        slide.notes_slide.notes_text_frame.text = body.strip() + f'\n详细问答见 {source}；补充提速结果另见冻结实验报告。'
    path = output / 'OpenFlex_论文答辩初稿_v1.pptx'
    presentation.save(path)
    restored = Presentation(path)
    if len(restored.slides) != 12:
        raise RuntimeError('Presentation slide count changed')
    pdf = convert(path, output, 'pdf', 'impress_pdf_Export')
    report = {'slides': 12, 'native_editable_text': True, 'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
              'pptx_sha256': hashlib.sha256(path.read_bytes()).hexdigest(), 'pdf_bytes': pdf.stat().st_size,
              'scope': 'Original three-trial feasibility results; speed supplement is separate'}
    (output / 'manifest.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(path)


if __name__ == '__main__':
    main()
