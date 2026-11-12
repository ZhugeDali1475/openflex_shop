import json
from collections import Counter
from pathlib import Path
import re
import unicodedata

from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm
from latex2mathml.converter import convert
from lxml import etree


def chinese_count(text):
    return len(re.findall(r'[\u3400-\u4dbf\u4e00-\u9fff]', text))


def manuscript_counts(text):
    chapters = []
    matches = list(re.finditer(r'^## (第\d+章[^\n]*)\n', text, re.MULTILINE))
    for index, match in enumerate(matches):
        end = matches[index + 1].start() if index + 1 < len(matches) else len(text)
        body = text[match.end():end]
        if index == len(matches) - 1:
            body = re.split(r'^## (?:参考文献|附录)', body, maxsplit=1, flags=re.MULTILINE)[0]
        body = re.sub(r'```.*?```', '', body, flags=re.DOTALL)
        prose = '\n'.join(line for line in body.splitlines()
                          if not line.startswith(('#', '|', '![', '$$')))
        chapters.append({'chapter': match.group(1), 'body_chinese': chinese_count(body),
                         'prose_chinese': chinese_count(prose),
                         'numbered_equations': len(re.findall(r'\\tag\{[^}]+\}', body))})
    return {'definition': '正文仅第1至末章，不含摘要、参考文献和附录；纯叙述另排除标题、表格、图片说明、代码及独立公式。汉字不等同Word字符数或学校字数口径。',
            'body_chinese': sum(item['body_chinese'] for item in chapters),
            'prose_chinese': sum(item['prose_chinese'] for item in chapters),
            'numbered_equations': sum(item['numbered_equations'] for item in chapters),
            'chapters': chapters}


def math_element(name, *children):
    element = OxmlElement('m:' + name)
    for child in children:
        element.append(child)
    return element


def math_container(name, node):
    result = math_element(name)
    for child in math_nodes(node):
        result.append(child)
    return result


def math_delimiter(opening, closing, content):
    properties = math_element('dPr')
    for name, value in [('begChr', opening), ('endChr', closing)]:
        symbol = math_element(name)
        symbol.set(qn('m:val'), value)
        properties.append(symbol)
    return math_element('d', properties, math_element('e', *content))


def math_sequence(children):
    result = []
    index = 0
    pairs = {'(': ')', '[': ']', '{': '}', '⟨': '⟩', '|': '|', '‖': '‖'}
    while index < len(children):
        node = children[index]
        name = etree.QName(node).localname
        token = ''.join(node.itertext()) if name == 'mo' else ''
        if token in pairs:
            depth = 1
            closing_index = None
            for position in range(index + 1, len(children)):
                other = children[position]
                if etree.QName(other).localname != 'mo':
                    continue
                symbol = ''.join(other.itertext())
                if symbol == pairs[token]:
                    depth -= 1
                    if depth == 0:
                        closing_index = position
                        break
                elif symbol == token:
                    depth += 1
            if closing_index is not None:
                result.append(math_delimiter(token, pairs[token], math_sequence(children[index + 1:closing_index])))
                index = closing_index + 1
                continue
            if node.get('fence') == 'true':
                result.append(math_delimiter(token, '', math_sequence(children[index + 1:])))
                break
        if name in {'msub', 'msup', 'msubsup', 'munder', 'mover', 'munderover'}:
            base = list(node)[0]
            if etree.QName(base).localname == 'mo' and ''.join(base.itertext()) in {'∑', '∏', '⋀', '⋁', '∫'}:
                parts = list(node)
                properties = math_element('naryPr')
                symbol = math_element('chr')
                symbol.set(qn('m:val'), ''.join(base.itertext()))
                properties.append(symbol)
                limits = math_element('limLoc')
                limits.set(qn('m:val'), 'undOvr')
                properties.append(limits)
                lower = math_container('sub', parts[1]) if name in {'msub', 'msubsup', 'munder', 'munderover'} else math_element('sub')
                upper = math_container('sup', parts[-1]) if name in {'msup', 'msubsup', 'mover', 'munderover'} else math_element('sup')
                remaining = math_sequence(children[index + 1:])
                boundary = next((position for position, item in enumerate(remaining)
                                 if etree.QName(item).localname == 'r'
                                 and ''.join(item.itertext()).strip() in {',', ';', '=', '≥', '≤', '⇒'}), len(remaining))
                result.append(math_element('nary', properties, lower, upper, math_element('e', *remaining[:boundary])))
                result.extend(remaining[boundary:])
                break
        result.extend(math_nodes(node))
        index += 1
    return result


def math_nodes(node):
    name = etree.QName(node).localname
    children = list(node)
    if name in {'math', 'mrow', 'mstyle', 'mpadded', 'semantics'}:
        return math_sequence(children)
    if name in {'mi', 'mn', 'mo', 'mtext'}:
        text = ''.join(node.itertext())
        if not text:
            return []
        run = math_element('r')
        properties = math_element('rPr')
        if name == 'mtext' or len(text) > 1:
            properties.append(math_element('nor'))
            properties.append(math_element('lit'))
        if node.get('mathvariant') in {'script', 'double-struck'}:
            script = math_element('scr')
            script.set(qn('m:val'), node.get('mathvariant'))
            properties.append(script)
        run.append(properties)
        token = math_element('t')
        token.text = text
        token.set(qn('xml:space'), 'preserve')
        run.append(token)
        return [run]
    if name == 'mspace':
        return []
    if name == 'mfrac':
        return [math_element('f', math_container('num', children[0]),
                             math_container('den', children[1]))]
    if name in {'msub', 'msup', 'msubsup'}:
        structure = {'msub': 'sSub', 'msup': 'sSup', 'msubsup': 'sSubSup'}[name]
        result = math_element(structure, math_container('e', children[0]))
        if name in {'msub', 'msubsup'}:
            result.append(math_container('sub', children[1]))
        if name in {'msup', 'msubsup'}:
            result.append(math_container('sup', children[-1]))
        return [result]
    if name in {'msqrt', 'mroot'}:
        result = math_element('rad')
        if name == 'msqrt':
            properties = math_element('radPr')
            hidden = math_element('degHide')
            hidden.set(qn('m:val'), '1')
            properties.append(hidden)
            result.extend([properties, math_element('deg')])
            result.append(math_element('e', *[item for child in children for item in math_nodes(child)]))
        else:
            result.extend([math_container('deg', children[1]), math_container('e', children[0])])
        return [result]
    if name in {'munder', 'mover', 'munderover'}:
        accent = ''.join(children[-1].itertext())
        if name == 'mover' and accent in {'¯', '^', '˙', '→', '˜'}:
            properties = math_element('accPr')
            symbol = math_element('chr')
            symbol.set(qn('m:val'), {'¯': '\u0305', '^': '\u0302', '˙': '\u0307', '→': '\u20d7', '˜': '\u0303'}[accent])
            properties.append(symbol)
            return [math_element('acc', properties, math_container('e', children[0]))]
        if name == 'munderover':
            lower = math_element('limLow', math_container('e', children[0]),
                                 math_container('lim', children[1]))
            return [math_element('limUpp', math_element('e', lower), math_container('lim', children[2]))]
        structure = 'limLow' if name == 'munder' else 'limUpp'
        return [math_element(structure, math_container('e', children[0]),
                             math_container('lim', children[1]))]
    if name == 'mfenced':
        properties = math_element('dPr')
        for key, value in [('begChr', node.get('open', '(')), ('endChr', node.get('close', ')'))]:
            symbol = math_element(key)
            symbol.set(qn('m:val'), value)
            properties.append(symbol)
        content = math_element('e', *[item for child in children for item in math_nodes(child)])
        return [math_element('d', properties, content)]
    if name in {'mtable', 'mtr', 'mtd'}:
        if name != 'mtable':
            return [item for child in children for item in math_nodes(child)]
        matrix = math_element('m')
        properties = math_element('mPr')
        columns = math_element('mcs')
        column = math_element('mc')
        column_properties = math_element('mcPr')
        count = math_element('count')
        count.set(qn('m:val'), str(len(children[0]) if children else 1))
        column_properties.append(count)
        column.append(column_properties)
        columns.append(column)
        properties.append(columns)
        matrix.append(properties)
        for row in children:
            matrix.append(math_element('mr', *[math_container('e', cell) for cell in row]))
        return [matrix]
    raise ValueError(f'Unsupported MathML element: {name}')


def add_equation(document, line):
    match = re.fullmatch(r'\$\$\s*(.*?)\s*\\tag\{([^}]+)\}\s*\$\$', line)
    if match is None:
        raise ValueError('Equation requires one-line $$ expression \\tag{number} $$')
    expression, number = match.groups()
    root = etree.fromstring(convert(expression).encode())
    paragraph = document.add_paragraph()
    paragraph.paragraph_format.first_line_indent = Cm(0)
    paragraph.paragraph_format.keep_together = True
    paragraph.paragraph_format.tab_stops.add_tab_stop(Cm(14.4))
    paragraph._p.append(math_element('oMath', *math_nodes(root)))
    paragraph.add_run('\t(' + number + ')')
    return paragraph


def write_counts(output, text):
    report = manuscript_counts(text)
    (output / '正文字数核算.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    lines = ['# 正文字数核算', '', report['definition'], '',
             '| 章节 | 正文汉字 | 纯叙述汉字 | 编号公式 |', '|---|---:|---:|---:|']
    for item in report['chapters']:
        lines.append(f"| {item['chapter']} | {item['body_chinese']} | {item['prose_chinese']} | {item['numbered_equations']} |")
    lines.extend(['', f"正文合计：{report['body_chinese']} 汉字；纯叙述：{report['prose_chinese']} 汉字；编号公式：{report['numbered_equations']}。",
                  '', '正文不少于2万汉字为本轮工作目标，不代表已取得学校最低字数规定。页数不代替字数。'])
    (output / '正文字数核算.md').write_text('\n'.join(lines) + '\n')
    return report


def math_signature(equation):
    tokens = equation.xpath('.//m:t/text()', namespaces=equation.nsmap)
    for node in equation.iter():
        name = etree.QName(node).localname
        if name in {'begChr', 'endChr', 'chr'}:
            tokens.append(node.get(qn('m:val'), ''))
    text = unicodedata.normalize('NFKC', ''.join(tokens))
    return Counter(character for character in text if not character.isspace())


def validate_math_roundtrip(original, restored):
    before = original._element.xpath('//m:oMath')
    after = restored._element.xpath('//m:oMath')
    if len(before) != len(after):
        raise RuntimeError('Equation count changed in DOC round-trip')
    differences = []
    for index, (first, second) in enumerate(zip(before, after), start=1):
        expected, actual = math_signature(first), math_signature(second)
        if expected != actual:
            differences.append({'equation_index': index, 'lost': dict(expected - actual),
                                'added': dict(actual - expected)})
    if differences:
        raise RuntimeError('Equation tokens changed in DOC round-trip: ' + json.dumps(differences, ensure_ascii=False))
    return {'equations': len(before), 'token_and_symbol_roundtrip': True,
            'scope': 'Per-equation token multisets including delimiters, accents and n-ary operators; visual layout inspected separately.'}
