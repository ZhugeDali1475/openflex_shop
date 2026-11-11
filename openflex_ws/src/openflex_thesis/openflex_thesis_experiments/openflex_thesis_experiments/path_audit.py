"""Offline classification of endpoint and intermediate path failures."""

import argparse
import ast
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import re
from typing import Callable, Iterable, Mapping, Sequence, Tuple


PathChecker = Callable[[Mapping[str, float]], Tuple[bool, Iterable[str]]]


def classify_invalid_indices(invalid_indices: Sequence[int], total_states: int) -> str:
    """Classify invalid samples without treating a valid endpoint as a valid path."""
    if type(total_states) is not int or total_states < 0:
        raise ValueError("total_states must be a non-negative integer")
    invalid = set(invalid_indices)
    if any(type(index) is not int or not 0 <= index < total_states for index in invalid):
        raise ValueError("invalid indices must be integers inside the path")
    if not total_states:
        return "unknown"
    if not invalid:
        return "valid"
    categories = []
    if 0 in invalid:
        categories.append("start")
    if any(0 < index < total_states - 1 for index in invalid):
        categories.append("intermediate")
    if total_states - 1 in invalid:
        categories.append("goal")
    return "_and_".join(categories)


def joint_positions(positions):
    if not isinstance(positions, Mapping) or not positions:
        raise ValueError("positions must contain named joints")
    if any(not isinstance(name, str) or not name for name in positions):
        raise ValueError("joint names must be non-empty strings")
    if any(isinstance(value, bool) or not isinstance(value, (int, float))
           or not math.isfinite(value) for value in positions.values()):
        raise ValueError("joint positions must be finite numbers")
    return {name: float(value) for name, value in positions.items()}


def audit_discrete_path(states: Sequence[Mapping[str, object]]) -> dict:
    """Audit already sampled states from a planner or a collision checker.

    Complete ordered joint positions are required. Missing validity remains
    unknown; caller-supplied results are not independent collision checking.
    """
    normalized = []
    if not isinstance(states, (list, tuple)):
        raise ValueError("states must be a list of ordered sample records")
    for fallback_index, state in enumerate(states):
        if not isinstance(state, Mapping):
            raise ValueError("each state must be a sample record")
        index = state.get("index", fallback_index)
        if type(index) is not int or index != fallback_index:
            raise ValueError("states must be contiguous and ordered from index zero")
        valid = state.get("valid")
        if valid is not None and type(valid) is not bool:
            raise ValueError("valid must be a boolean or null, never a truthy string")
        contacts = state.get("contacts", [])
        if not isinstance(contacts, (tuple, list)) or any(
            not isinstance(contact, str) or not contact for contact in contacts
        ):
            raise ValueError("contacts must be a list of non-empty pair descriptions")
        positions = joint_positions(state.get("positions"))
        if normalized and set(positions) != set(normalized[0]["positions"]):
            raise ValueError("all states must contain the same joint names")
        normalized.append({
            "index": index,
            "valid": valid,
            "contacts": list(contacts),
            "positions": positions,
        })
    invalid = [state["index"] for state in normalized if state["valid"] is False]
    unknown = [state["index"] for state in normalized if state["valid"] is None]
    total_states = len(normalized)
    category = classify_invalid_indices(invalid, total_states)
    if unknown and not invalid:
        category = "unknown"
    return {
        "total_states": total_states,
        "invalid_indices": invalid,
        "unknown_indices": unknown,
        "category": category,
        "invalid_states": [state for state in normalized if state["valid"] is False],
        "endpoint_states_checked": bool(normalized) and all(
            normalized[index]["valid"] is not None for index in (0, -1)),
        "intermediate_states_checked": total_states > 2 and all(
            state["valid"] is not None for state in normalized[1:-1]),
        "complete_sample_evidence": bool(normalized) and not unknown,
        "continuous_safety_proven": False,
        "evidence_source": "caller_supplied_checks_NOT_independent_validation",
        "states": normalized,
    }


def interpolate_and_audit_path(
    start: Mapping[str, float],
    goal: Mapping[str, float],
    checker: PathChecker,
    samples: int = 101,
) -> dict:
    """Sample a straight joint-space path and classify checker failures.

    This is a diagnostic sampler, not a replacement for MoveIt's planner or
    continuous collision checking. The caller must provide the actual checker.
    """
    if type(samples) is not int or samples < 2:
        raise ValueError("path audit requires at least two samples")
    start, goal = joint_positions(start), joint_positions(goal)
    names = tuple(start)
    if set(names) != set(goal):
        raise ValueError("start and goal must contain the same joint names")
    states = []
    for index in range(samples):
        fraction = index / (samples - 1)
        positions = {
            name: float(start[name]) + fraction * (float(goal[name]) - float(start[name]))
            for name in names
        }
        checked = checker(deepcopy(positions))
        if isinstance(checked, tuple):
            valid, contacts = checked
        else:
            valid, contacts = checked, ()
        states.append({
            "index": index,
            "valid": valid,
            "contacts": contacts,
            "positions": positions,
        })
    result = audit_discrete_path(states)
    result["sampler"] = "linear_joint_space_diagnostic_only"
    result["sample_count"] = samples
    return result


def parse_moveit_path_failure(log_text: str) -> dict:
    """Keep each invalid-path explanation scoped to its own log block."""
    matches = list(re.finditer(
        r"Invalid states at index locations:\s*\[([^]]*)\]\s*out of\s*(\d+)",
        log_text,
    ))
    if log_text.count("Invalid states at index locations:") != len(matches):
        raise ValueError("malformed invalid-path marker; refusing partial audit")
    failures = []
    targets = list(re.finditer(r"(\w+) target joints: (\{[^\n]*\})", log_text))
    for match_index, match in enumerate(matches):
        indices = [int(value) for value in match.group(1).split()]
        total_states = int(match.group(2))
        category = classify_invalid_indices(indices, total_states)
        if not indices:
            raise ValueError("invalid-path marker has no invalid indices")
        next_offset = matches[match_index + 1].start() if match_index + 1 < len(matches) else (
            len(log_text))
        future_targets = [entry.start() for entry in targets if entry.start() > match.end()]
        response_end = min([next_offset] + future_targets)
        following = log_text[match.end():response_end]
        explanations = following.split("Completed listing of explanations", 1)[0]
        contacts = []
        for first, second in re.findall(
            r"Found a contact between '([^']+)' \(type '[^']+'\)\s+and\s+"
            r"'([^']+)' \(type '[^']+'\)", explanations,
        ):
            pair = f"{first}<->{second}"
            if pair not in contacts:
                contacts.append(pair)
        previous_targets = [target for target in targets if target.start() < match.start()]
        target = previous_targets[-1] if previous_targets else None
        response_block = log_text[match.end():response_end]
        failures.append({
            "line": log_text.count("\n", 0, match.start()) + 1,
            "invalid_indices": indices,
            "total_states": total_states,
            "category": category,
            "contacts": contacts,
            "contact_association": "explanation_block_not_individual_point",
            "request_label": target.group(1) if target else None,
            "request_target_positions": (
                joint_positions(ast.literal_eval(target.group(2))) if target else None),
            "request_start_positions": None,
            "trajectory_positions": None,
            "execution_rejected_marker": "Not executing." in response_block,
            "endpoint_validity_proven": False,
        })
    return {
        "parse_status": "parsed" if failures else "not_found",
        "failure_count": len(failures),
        "failures": failures,
        "continuous_safety_proven": False,
        "endpoint_validity_proven": False,
        "limitations": [
            "Only logged rejected paths; absent failures do not establish validity.",
            "Request labels are nearest preceding target logs, not correlated request IDs.",
            "No recorded trajectory positions or exact planning start state in these logs.",
            "Not executing markers refer to reported paths, not all robot activity.",
        ],
    }


def main(args=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    inputs = parser.add_mutually_exclusive_group(required=True)
    inputs.add_argument("--log", type=Path)
    inputs.add_argument("--states", type=Path, help="JSON list of sampled states and checks")
    parser.add_argument("--output", type=Path)
    options = parser.parse_args(args)
    source = options.log or options.states
    raw = source.read_bytes()
    report = (parse_moveit_path_failure(raw.decode()) if options.log else
              audit_discrete_path(json.loads(raw)))
    report["schema_version"] = 1
    report["analysis_type"] = (
        "offline_moveit_log_path_audit" if options.log else "offline_sample_evidence_audit")
    report["source_path"] = str(source)
    report["source_sha256"] = hashlib.sha256(raw).hexdigest()
    report["analyzer_sha256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    encoded = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    if options.output:
        with options.output.open("x") as stream:
            stream.write(encoded)
    else:
        print(encoded, end="")


if __name__ == "__main__":
    main()
