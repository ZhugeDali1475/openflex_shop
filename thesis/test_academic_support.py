from pathlib import Path
import sys

from docx import Document
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from academic_support import add_equation, manuscript_counts, validate_math_roundtrip
from build_thesis import render


def test_counts_exclude_abstract_references_appendices():
    text = '# 题目\n## 摘要\n摘要\n## 第1章 绪论\n研究方法\n| 参数 | 数值 |\n## 第6章 总结与展望\n结论\n## 参考文献\n文献\n## 附录B\n附录'
    report = manuscript_counts(text)
    assert report['body_chinese'] == 10
    assert report['prose_chinese'] == 6
    assert len(report['chapters']) == 2


@pytest.mark.parametrize('expression', [
    r'\mathcal{R}_{g,h}=\operatorname{CC}_{b^{*}}\{b:S(b)\geq\tau\}',
    r'J=\sum_{i=1}^{n}\frac{q_i-q_i^{-}}{q_i^{+}-q_i^{-}}',
    r'\delta=\max(\sqrt{\Sigma_{xx}},\epsilon)',
    r'\left(\begin{matrix}x&y\\z&w\end{matrix}\right)',
])
def test_equations_are_structured_office_math(expression, tmp_path):
    document = Document()
    add_equation(document, '$$ ' + expression + r' \tag{3-1} $$')
    path = tmp_path / 'equation.docx'
    document.save(path)
    restored = Document(path)
    assert len(restored._element.xpath('//m:oMath')) == 1
    assert '(3-1)' in restored.paragraphs[0].text
    assert expression not in restored.paragraphs[0].text


def test_unnumbered_equation_is_not_silently_lost():
    with pytest.raises(ValueError, match='Equation requires'):
        add_equation(Document(), '$$ x=1 $$')


def test_third_level_headings_render(tmp_path):
    document = Document()
    render('#### 3.1.1 条件绑定', document, tmp_path)
    assert document.paragraphs[0].style.name == 'Heading 3'


def test_accents_sums_and_delimiters_are_not_plain_text():
    document = Document()
    add_equation(document, r'$$ \hat s=\sum_{j=1}^{6}\bar w_j\left(1+|q_j|\right) \tag{1-1} $$')
    assert len(document._element.xpath('//m:acc')) == 2
    assert len(document._element.xpath('//m:nary')) == 1
    assert len(document._element.xpath('//m:d')) == 2


def test_math_validation_detects_content_loss_even_with_same_equation_count():
    first, second = Document(), Document()
    add_equation(first, r'$$ x+y=1 \tag{1-1} $$')
    add_equation(second, r'$$ x=1 \tag{1-1} $$')
    with pytest.raises(RuntimeError, match='tokens changed'):
        validate_math_roundtrip(first, second)


def test_math_validation_accepts_unchanged_formulas():
    document = Document()
    add_equation(document, r'$$ c^{\star}=\frac{x}{y} \tag{1-1} $$')
    assert validate_math_roundtrip(document, document)['equations'] == 1


def test_current_research_plan_keeps_method_attribution_and_evidence_boundaries():
    source = Path(__file__).with_name('商超轮臂机器人_研究主线与算法实验方案_CN.md').read_text()
    assert source.count(r'\tag{') == 14
    assert '作者方法一' in source and '作者方法二' in source
    assert '同预算' in source and '待实验' in source
    assert '不是已经完成全部实验的学位论文' in source
