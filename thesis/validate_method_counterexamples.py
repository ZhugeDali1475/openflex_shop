import argparse
from dataclasses import asdict, replace
from datetime import datetime
import hashlib
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'openflex_ws/src/openflex_thesis/openflex_tc_wam'
sys.path.insert(0, str(PACKAGE))
from openflex_tc_wam.scoring import (
    CandidateMetrics, ScoreWeights, conditioned_ready_candidate_ids,
    ready_candidate_ids, score_candidate, select_best,
)


def counterexample():
    first = CandidateMetrics(1, 0, 0, 0, True, True, True, True, True,
                             1.0, 0.8, 1.0, 0.9, grasp_id=7, lift_seed=0.15,
                             lift_position=0.16, planning_group='right_arm_with_lift',
                             grasp_quality=0.9, visibility_score=0.8)
    return [first,
            replace(first, candidate_id=2, x=0.1, robustness=0.9),
            replace(first, candidate_id=3, x=0.2, grasp_id=8, robustness=0.9),
            replace(first, candidate_id=4, y=0.1, lift_seed=-0.15, robustness=0.9),
            replace(first, candidate_id=5, y=-0.1, lift_position=0.25, robustness=0.9),
            replace(first, candidate_id=6, x=0.8, robustness=0.9),
            replace(first, candidate_id=7, x=0.2, y=0.1,
                    planning_group='left_arm_with_lift', robustness=0.9),
            replace(first, candidate_id=8, x=0.3, robustness=0.0, joint_margin=0.0),
            replace(first, candidate_id=9, x=-0.1, collision_free=False)]


def evaluate():
    candidates = counterexample()
    scores = [score_candidate(item, ScoreWeights(), 0.34) for item in candidates]
    best = select_best(scores)
    selected = next(item for item in candidates if item.candidate_id == best.candidate_id)
    pooled = ready_candidate_ids(scores, 0.65, 0.90)
    conditioned = [item.candidate_id for item in candidates
                   if item.candidate_id in pooled and item.grasp_id == selected.grasp_id
                   and item.planning_group == selected.planning_group
                   and abs(item.lift_seed - selected.lift_seed) <= 1e-4
                   and abs(item.lift_position - selected.lift_position) <= 0.02]
    connected = conditioned_ready_candidate_ids(candidates, scores, best.candidate_id,
                                                0.65, 0.90, 0.11, 0.20)
    variants = {'pooled_high_score': pooled, 'condition_only': conditioned,
                'condition_and_component': connected}
    expected = {'pooled_high_score': [1, 2, 3, 4, 5, 6, 7],
                'condition_only': [1, 2, 6], 'condition_and_component': [1, 2]}
    if variants != expected:
        raise AssertionError(f'Counterexample changed: {variants}')
    code = PACKAGE / 'openflex_tc_wam/scoring.py'
    return {'created_at': datetime.now().astimezone().isoformat(),
            'evidence_type': 'constructed_offline_counterexample_NOT_robot_experiment',
            'source': str(code.relative_to(ROOT)), 'source_sha256': hashlib.sha256(code.read_bytes()).hexdigest(),
            'best_id': best.candidate_id, 'candidates': [asdict(item) for item in candidates],
            'scores': [asdict(item) for item in scores], 'variants': variants,
            'condition_mismatch_ids': [3, 4, 5, 7], 'disconnected_same_condition_ids': [6],
            'low_score_rejected_ids': [8], 'collision_rejected_ids': [9],
            'limitation': 'One designed counterexample; no estimated error rate, success rate, or robot speed-up.'}


def make_figures(assets):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plot
    from matplotlib.font_manager import FontProperties
    from matplotlib.patches import FancyBboxPatch
    import subprocess

    font = FontProperties(fname=subprocess.check_output(
        ['fc-match', 'Noto Sans CJK SC', '-f', '%{file}'], text=True))
    figure, axis = plot.subplots(figsize=(10, 6))
    axis.set(xlim=(0, 10), ylim=(0, 7))
    axis.axis('off')
    blocks = [((0.5, 5.3), '语义提议层\n自然语言／图像 → 模型选技能'),
              ((5.5, 5.3), '证据状态层\n时间、对象、模式与操作关联'),
              ((0.5, 3.1), '契约审查层\n参数、阶段、时效、互斥'),
              ((5.5, 3.1), '条件化能力图\n联合候选 → 一致条件连通域'),
              ((0.5, 0.9), '技能执行层\n导航 → 抓取 → 放置'),
              ((5.5, 0.9), '独立完成检查\n支撑 ∧ 释放 ∧ 姿态 ∧ 停稳')]
    for (left, bottom), label in blocks:
        axis.add_patch(FancyBboxPatch((left, bottom), 3.8, 1.15,
                                     boxstyle='round,pad=0.12', facecolor='#eef3fa', edgecolor='#315780'))
        axis.text(left + 1.9, bottom + 0.57, label, ha='center', va='center',
                  fontproperties=font, fontsize=11)
    for start, finish in [((2.4, 5.15), (2.4, 4.45)), ((2.4, 2.95), (2.4, 2.2)),
                           ((5.35, 3.65), (4.45, 3.65)), ((4.45, 1.45), (5.35, 1.45)),
                           ((7.4, 2.2), (7.4, 2.95)), ((7.4, 4.45), (7.4, 5.15)),
                           ((5.35, 5.85), (4.45, 5.85))]:
        axis.annotate('', xy=finish, xytext=start, arrowprops={'arrowstyle': '->', 'lw': 1.5})
    figure.tight_layout()
    figure.savefig(assets / 'method_architecture.png', dpi=180)
    plot.close(figure)
    report = evaluate()
    figure, axes = plot.subplots(1, 3, figsize=(12, 4.3), sharey=True)
    titles = ['高分并集（无条件绑定）', '仅条件绑定（无连通过滤）', '条件绑定＋最优点连通分量']
    for axis, (variant, kept), title in zip(axes, report['variants'].items(), titles):
        for item in report['candidates']:
            identifier = item['candidate_id']
            axis.scatter(item['x'], item['y'], c='#1c7d55' if identifier in kept else '#c8ccd1', s=90)
            axis.annotate(str(identifier), (item['x'], item['y']), xytext=(4, 6), textcoords='offset points')
        axis.set_title(title, fontproperties=font, fontsize=10)
        axis.set(xlim=(-0.2, 1), ylim=(-0.2, 0.25), xlabel='x (m)')
        axis.grid(alpha=0.2)
    axes[0].set_ylabel('y (m)')
    figure.suptitle('构造性离线反例，非机器人性能实验；绿色为保留点', fontproperties=font, fontsize=12)
    figure.tight_layout()
    figure.savefig(assets / 'ready_counterexample.png', dpi=180)
    plot.close(figure)


def main():
    parser = argparse.ArgumentParser(description='Read-only deterministic scoring counterexample; no ROS commands')
    parser.add_argument('--output', type=Path, required=True)
    arguments = parser.parse_args()
    arguments.output.mkdir(parents=True, exist_ok=False)
    report = evaluate()
    (arguments.output / 'counterexample.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    make_figures(arguments.output)
    print(json.dumps(report['variants'], ensure_ascii=False))


if __name__ == '__main__':
    main()
