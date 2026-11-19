#!/usr/bin/env python3
"""Draw the OpenFlex P6 paper figures from the aggregated evidence matrix.

The figures are schematic (system layering, two-level skill call graph, object
lifecycle, PlanningScene synchronisation, failure-feedback timing) plus one data
figure drawn from ``experiment_matrix.json``.  Every schematic is annotated with
the boundary it must not over-claim (isolated MuJoCo, ``physical_verification =
false``) and the data figure is drawn straight from the recorded matrix, so each
number stays traceable to a run log.

Usage::

    python3 scripts/make_openflex_p6_figures.py --matrix <experiment_matrix.json> \
        --output-root <dir>
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

SELF = "#1f6feb"        # self-developed OpenFlex
ADOPTED = "#9a6700"     # adopted third-party component (RoboOS/MCP/RoboBrain)
BOUNDARY = "#8250df"    # verification boundary
OK = "#1a7f37"
FAIL = "#cf222e"

plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 9,
                     "figure.dpi": 140, "savefig.bbox": "tight"})


def _box(ax, x, y, w, h, text, color, fontsize=9, text_color="white"):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.01,rounding_size=0.02",
                                linewidth=1.2, edgecolor=color, facecolor=color, alpha=0.14))
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=fontsize,
            color="#111111", wrap=True)
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.01,rounding_size=0.02",
                                linewidth=1.2, edgecolor=color, facecolor="none"))


def _arrow(ax, xy_a, xy_b, color="#444444", style="-|>", lw=1.2, ls="-"):
    ax.add_patch(FancyArrowPatch(xy_a, xy_b, arrowstyle=style, mutation_scale=12,
                                 color=color, linewidth=lw, linestyle=ls,
                                 shrinkA=2, shrinkB=2))


def _canvas(w=9.0, h=6.0):
    fig, ax = plt.subplots(figsize=(w, h))
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")
    return fig, ax


def figure_system_layers(out: Path):
    fig, ax = _canvas(8.6, 7.0)
    ax.text(0.5, 0.97, "OpenFlex two-level skill system: layers and verification boundary",
            ha="center", va="top", fontsize=12, weight="bold")
    layers = [
        (0.80, "RoboBrain2.5 planner (local Qwen3-4B GGUF)  ·  adopted component", ADOPTED),
        (0.655, "MCP tool surface  ·  6 model-facing + 3 host-managed tools  ·  self-developed", SELF),
        (0.51, "Skill layer  ·  composite (fetch_and_deliver, search_and_pick)\n+  atomic (navigate / grasp / place)  ·  self-developed", SELF),
        (0.365, "ROS 2 control  ·  control_mode arbiter  ·  MoveIt trajectory exec  ·  fixed_place_supervisor", SELF),
        (0.22, "MuJoCo v9 retail fixture  ·  ground-truth observation + oracle grasp", BOUNDARY),
    ]
    for y, text, color in layers:
        _box(ax, 0.05, y, 0.90, 0.105, text, color, fontsize=9)
    for a, b in [(0.80, 0.76), (0.655, 0.615), (0.51, 0.47), (0.365, 0.325)]:
        _arrow(ax, (0.5, a), (0.5, b))
    ax.text(0.5, 0.145,
            "Verification boundary: isolated MuJoCo only, physical_verification = false.\n"
            "No sim-passed result is a real-hardware safety claim.",
            ha="center", va="center", fontsize=8.5, color=BOUNDARY)
    ax.add_patch(FancyBboxPatch((0.05, 0.09), 0.90, 0.11,
                                boxstyle="round,pad=0.01,rounding_size=0.02",
                                linewidth=1.0, edgecolor=BOUNDARY, facecolor=BOUNDARY, alpha=0.06))
    fig.savefig(out / "fig_01_system_layers.png")
    plt.close(fig)


def figure_skill_callgraph(out: Path):
    fig, ax = _canvas(9.0, 6.4)
    ax.text(0.5, 0.97, "Two-level skill call graph (composite -> atomic)",
            ha="center", va="top", fontsize=12, weight="bold")
    _box(ax, 0.30, 0.80, 0.40, 0.10, "composite: fetch_and_deliver\n(RunPickDeliver action)", SELF, 9)
    _box(ax, 0.02, 0.55, 0.26, 0.10, "composite:\nsearch_and_pick", SELF, 9)
    _box(ax, 0.36, 0.55, 0.28, 0.10, "atomic:\nnavigate_to_target", SELF, 9)
    _box(ax, 0.68, 0.55, 0.30, 0.10, "atomic:\ngrasp_object", SELF, 9)
    _box(ax, 0.36, 0.33, 0.28, 0.10, "atomic:\nplace_to_affordance", SELF, 9)
    _box(ax, 0.68, 0.33, 0.30, 0.10, "observe_scene /\nget_robot_state", ADOPTED, 9)
    _arrow(ax, (0.42, 0.80), (0.15, 0.65))
    _arrow(ax, (0.48, 0.80), (0.50, 0.65))
    _arrow(ax, (0.55, 0.80), (0.80, 0.65))
    _arrow(ax, (0.83, 0.55), (0.60, 0.43))
    _arrow(ax, (0.50, 0.55), (0.50, 0.43))
    _arrow(ax, (0.83, 0.33), (0.66, 0.43), color=ADOPTED)
    _arrow(ax, (0.15, 0.55), (0.72, 0.43), color=ADOPTED, ls="--")
    ax.text(0.5, 0.22,
            "search_and_pick: image-derived candidate selection (unique red at the registered slot)\n"
            "then the same grasp/place atomic skills; oracle object id is not used for selection.",
            ha="center", va="center", fontsize=8.5)
    ax.text(0.5, 0.12,
            "host-managed / internal tools (sim reset, drop, randomize, viewer) are NOT on the model surface.",
            ha="center", va="center", fontsize=8.5, color=BOUNDARY)
    fig.savefig(out / "fig_02_skill_callgraph.png")
    plt.close(fig)


def figure_object_lifecycle(out: Path):
    fig, ax = _canvas(9.6, 6.2)
    ax.text(0.5, 0.97, "Single-red object lifecycle and attachment synchronisation",
            ha="center", va="top", fontsize=12, weight="bold")
    row1 = [
        (0.03, "nominal\n(on shelf, free joint)", ADOPTED),
        (0.35, "observed\n(fresh RGB-D + pose)", ADOPTED),
        (0.67, "approach\n(pre-grasp clearance)", SELF),
    ]
    row2 = [
        (0.03, "attached\n(holding=thesis_item_01)", SELF),
        (0.35, "transport\n(payload clearance)", SELF),
        (0.67, "placed\n(support contact, holding=null)", SELF),
    ]
    for x, text, color in row1:
        _box(ax, x, 0.60, 0.30, 0.14, text, color, 8.6)
    for x, text, color in row2:
        _box(ax, x, 0.38, 0.30, 0.14, text, color, 8.6)
    for i in range(len(row1) - 1):
        _arrow(ax, (row1[i][0] + 0.30, 0.67), (row1[i + 1][0], 0.67))
    for i in range(len(row2) - 1):
        _arrow(ax, (row2[i][0] + 0.30, 0.45), (row2[i + 1][0], 0.45))
    _arrow(ax, (0.82, 0.60), (0.82, 0.545))
    ax.add_patch(FancyArrowPatch((0.82, 0.545), (0.18, 0.545), arrowstyle="-",
                                 color="#444444", linewidth=1.2))
    _arrow(ax, (0.18, 0.545), (0.18, 0.52))
    ax.text(0.5, 0.30,
            "attach: exclusive MoveIt control + measured jaw/steps -> _try_attach()\n"
            "detach: release opens jaws, free joint re-parented to world, 0.5 s settle",
            ha="center", va="center", fontsize=9)
    ax.text(0.5, 0.195,
            "guards on every transition: fresh observation (<= max_age), measured joint target,\n"
            "stationary velocities, forbidden-contact audit; a failed/uncertain motion freezes the loop.",
            ha="center", va="center", fontsize=9, color=BOUNDARY)
    ax.add_patch(FancyBboxPatch((0.13, 0.055), 0.74, 0.085,
                                boxstyle="round,pad=0.01,rounding_size=0.02",
                                linewidth=1.0, edgecolor=BOUNDARY, facecolor=BOUNDARY, alpha=0.06))
    ax.text(0.5, 0.097,
            "failed / UNKNOWN -> must_stop = true, stopped_confirmed = true (never a false success)",
            ha="center", va="center", fontsize=8.6, color=BOUNDARY)
    fig.savefig(out / "fig_03_object_lifecycle.png")
    plt.close(fig)


def figure_planning_scene_sync(out: Path):
    fig, ax = _canvas(9.0, 5.8)
    ax.text(0.5, 0.97, "PlanningScene synchronisation and registry binding",
            ha="center", va="top", fontsize=12, weight="bold")
    boxes = [
        (0.02, 0.66, 0.20, 0.16, "MJCF scene (v9)\nscene.xml", SELF),
        (0.26, 0.66, 0.20, 0.16, "collision-set\nprojection", SELF),
        (0.50, 0.66, 0.20, 0.16, "MoveIt\nPlanningScene", SELF),
        (0.74, 0.66, 0.24, 0.16, "registry-bound\nskill targets", SELF),
        (0.02, 0.40, 0.20, 0.14, "scene_sha256\n6db3a9bf...", ADOPTED),
        (0.26, 0.40, 0.44, 0.14, "left delivery table  sim_delivery_top_left  (only reported destination)", OK),
        (0.74, 0.40, 0.24, 0.14, "right table excluded\nfrom P6 results", FAIL),
    ]
    for x, y, w, h, text, color in boxes:
        _box(ax, x, y, w, h, text, color, 8.6)
    _arrow(ax, (0.22, 0.74), (0.26, 0.74))
    _arrow(ax, (0.46, 0.74), (0.50, 0.74))
    _arrow(ax, (0.70, 0.74), (0.74, 0.74))
    ax.text(0.5, 0.26,
            "Only registered ids reach the model surface; free coordinates / arbitrary paths are rejected by the\n"
            "closed advertised surface and additionalProperties=false schemas.",
            ha="center", va="center", fontsize=9)
    ax.text(0.5, 0.12,
            "boundary: the same v9 MJCF hashes the bridge, the viewer and the audits (single source of truth).",
            ha="center", va="center", fontsize=8.5, color=BOUNDARY)
    fig.savefig(out / "fig_04_planning_scene_sync.png")
    plt.close(fig)


def figure_failure_feedback_timing(out: Path):
    fig, ax = plt.subplots(figsize=(9.4, 5.2))
    ax.set_xlim(0, 1.6)
    ax.set_ylim(0, 1)
    ax.axis("off")
    ax.text(0.8, 0.97, "Failure-feedback timing at the 0.5 s exclusivity gate",
            ha="center", va="top", fontsize=12, weight="bold")
    ax.plot([0.05, 1.55], [0.72, 0.72], color="#444444", lw=1.0)
    ax.text(0.05, 0.76, "publisher: /thesis/control_mode, 50 Hz (stamp gap <= 0.14 s measured)",
            fontsize=8.5, va="bottom")
    for t in [0.10, 0.16, 0.22, 0.28, 0.34, 0.40, 0.46, 0.52, 0.58, 0.64]:
        ax.plot([t, t], [0.70, 0.74], color=ADOPTED, lw=1.0)
    ax.plot([0.70, 1.05], [0.70, 0.70], color=FAIL, lw=2.0)
    for t in [0.70, 1.05]:
        ax.plot([t, t], [0.69, 0.73], color=FAIL, lw=2.0)
    ax.annotate("", xy=(1.05, 0.62), xytext=(0.70, 0.62),
                arrowprops=dict(arrowstyle="<->", color=FAIL))
    ax.text(0.875, 0.57, "bridge-side dispatch gap  entry_gap 0.42-0.84 s", ha="center",
            fontsize=8.5, color=FAIL)
    ax.text(0.875, 0.50, "lock_wait = 0.000 s   ->   not a lock problem", ha="center", fontsize=8.5)
    ax.plot([0.70, 1.05], [0.40, 0.40], color=SELF, lw=1.4, ls="--")
    ax.text(0.875, 0.345, "0.5 s freshness window exhausted -> exclusive MoveIt control revoked",
            ha="center", fontsize=8.5, color=SELF)
    ax.text(0.5, 0.265,
            "on revocation: goal aborted, arm held at measured pose, stopped_confirmed = true;\n"
            "the feedback carried back to the planner is must_stop = true, success = false, reason = \"revoked\".",
            ha="center", va="center", fontsize=9, color=BOUNDARY)
    ax.add_patch(FancyBboxPatch((0.18, 0.07), 1.24, 0.12,
                                boxstyle="round,pad=0.01,rounding_size=0.02",
                                linewidth=1.0, edgecolor=BOUNDARY, facecolor=BOUNDARY, alpha=0.06))
    ax.text(0.8, 0.13,
            "residual root cause not yet isolated (in-process interpreter stall); no control-side constant was relaxed.",
            ha="center", va="center", fontsize=8.5, color=BOUNDARY)
    fig.savefig(out / "fig_05_failure_feedback_timing.png")
    plt.close(fig)


def figure_success_rates(out: Path, matrix: dict):
    summary = matrix["group_summary"]
    order = ["fixed_policy_3_atomic_tools", "composite_driver", "search_and_pick",
             "search_and_pick_negative_control", "model_policy_online",
             "model_policy_before_vram_fix"]
    labels = {
        "fixed_policy_3_atomic_tools": "fixed policy\n(3 atomic tools)",
        "composite_driver": "composite driver\n(nav+grasp+place)",
        "search_and_pick": "search_and_pick\n(image candidates)",
        "search_and_pick_negative_control": "negative control\n(fail-closed)",
        "model_policy_online": "model policy\n(online, 4B)",
        "model_policy_before_vram_fix": "model policy\n(pre-fix, OOM)",
    }
    fig, ax = plt.subplots(figsize=(9.6, 4.8))
    xs, ys, texts = [], [], []
    for i, key in enumerate(order):
        group = summary.get(key)
        if not group:
            continue
        xs.append(i)
        ys.append(group["rate"] * 100)
        texts.append(f"{group['passed']}/{group['total']}")
    bars = ax.bar(xs, ys, color=[FAIL if key == "model_policy_before_vram_fix" else SELF
                                 for key in order if key in summary],
                  width=0.6, alpha=0.85)
    for bar, text in zip(bars, texts):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 1.5, text,
                ha="center", fontsize=9)
    ax.set_xticks(xs)
    ax.set_xticklabels([labels[key] for key in order if key in summary], fontsize=8)
    ax.set_ylabel("end-to-end success rate (%)")
    ax.set_ylim(0, 105)
    ax.set_title("P6.1 recorded end-to-end success rates (all attempts in the denominator)",
                 fontsize=11)
    ax.grid(axis="y", alpha=0.25)
    ax.text(0.5, -0.34,
            "isolated MuJoCo v9, scene_sha256 6db3a9bf..., physical_verification = false; "
            "model policy measured online after the VRAM fix (q8_0 KV, no mmproj offload);\n"
            "the pre-fix batch is kept separately as 0/3 (local 4B decode CUDA OOM).",
            transform=ax.transAxes, ha="center", fontsize=8, color=BOUNDARY)
    fig.savefig(out / "fig_06_success_rates.png")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--matrix", required=True)
    parser.add_argument("--output-root", required=True)
    args = parser.parse_args()
    out = Path(args.output_root)
    out.mkdir(parents=True, exist_ok=True)
    matrix = json.loads(Path(args.matrix).read_text(encoding="utf-8"))

    figure_system_layers(out)
    figure_skill_callgraph(out)
    figure_object_lifecycle(out)
    figure_planning_scene_sync(out)
    figure_failure_feedback_timing(out)
    figure_success_rates(out, matrix)
    print(json.dumps({"output_root": str(out),
                      "figures": sorted(p.name for p in out.glob("fig_*.png"))},
                     ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
