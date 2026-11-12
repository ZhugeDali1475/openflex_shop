from copy import deepcopy
from pathlib import Path
import sys

from docx import Document
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from build_thesis import expand_tables, render, setup as configure_document, table


def test_experiment_values_are_inserted_without_changing_evidence():
    evidence = {'trials': [{'label': '同义指令', 'worker_inferences': 7, 'tool_calls': 7,
                            'motion_calls': 4, 'execution_seconds': 494.808, 'frames': 988,
                            'samples': 244, 'ram_min_mib': 942.21, 'swap_max_mib': 14676.0,
                            'gpu_max_mib': 5971}]}
    original = deepcopy(evidence)
    text = expand_tables('{{RESULTS_TABLE}}\n{{RESOURCES_TABLE}}', evidence)
    assert '494.8' in text and '14676.0' in text and '{{' not in text
    assert evidence == original


def test_markdown_table_remains_editable_and_separator_is_not_content():
    document = Document()
    result = table(document, ['| 条件 | 结果 |', '|---|---|', '| 观察 | 通过 |'])
    assert len(result.rows) == 2 and len(result.columns) == 2
    assert result.cell(1, 1).text == '通过'


def test_bad_table_is_not_silently_truncated():
    with pytest.raises(ValueError, match='columns'):
        table(Document(), ['| A | B |', '| C |'])


def test_headings_lists_and_literals_survive_save(tmp_path):
    document = Document()
    configure_document(document)
    render('## 第1章 绪论\n\n### 1.1 范围\n\n正文 **粗体** `接口`\n\n- 注意事项', document, tmp_path)
    path = tmp_path / 'test.docx'
    document.save(path)
    restored = Document(path)
    assert restored.paragraphs[0].style.name == 'Heading 1'
    assert '正文 粗体 接口' in [paragraph.text for paragraph in restored.paragraphs]


def test_missing_image_fails_before_export(tmp_path):
    with pytest.raises(FileNotFoundError):
        render('![图1](assets/missing.png)', Document(), tmp_path)
