"""Static, fail-closed readiness checks before collecting thesis evidence."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import yaml


@dataclass(frozen=True)
class CheckResult:
    name: str
    passed: bool
    detail: str


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a YAML mapping")
    return value


def nested(value: dict[str, Any], *keys: str, default=None):
    current: Any = value
    for key in keys:
        if not isinstance(current, dict) or key not in current:
            return default
        current = current[key]
    return current


def boolean_check(name: str, value: Any, source: str) -> CheckResult:
    return CheckResult(name, value is True, f"{source}={value!r}")


def physical_readiness_checks(
    shelf_scene: dict[str, Any],
    perception: dict[str, Any],
    control: dict[str, Any],
    act_schema: dict[str, Any],
    experiment_protocol: dict[str, Any] | None = None,
    scope: str = "end_to_end",
) -> list[CheckResult]:
    perception_parameters = nested(perception, "perception", default={})
    executor_parameters = nested(
        control, "moveit_grasp_executor", "ros__parameters", default={}
    )
    arbiter_parameters = nested(
        control, "velocity_arbiter", "ros__parameters", default={}
    )
    mission_parameters = nested(
        control, "mission_orchestrator", "ros__parameters", default={}
    )
    action = act_schema.get("action", {})
    observation = act_schema.get("numeric_observation", {})
    act_scope = act_schema.get("scope", {})
    if scope not in {"perception", "tcwam", "act", "end_to_end"}:
        raise ValueError(f"unknown preflight scope {scope!r}")
    tagged = [
        ("tcwam", boolean_check("shelf_scene_calibrated", shelf_scene.get("calibrated"), "shelf_scene.calibrated")),
        ("perception", boolean_check(
            "physical_perception_candidates",
            perception_parameters.get("physical_candidates_available"),
            "perception.physical_candidates_available",
        )),
        ("end_to_end", boolean_check(
            "moveit_object_geometry_calibrated",
            executor_parameters.get("object_geometry_calibrated"),
            "moveit_grasp_executor.object_geometry_calibrated",
        )),
        ("act", boolean_check("act_schema_frozen", act_schema.get("frozen"), "act_schema.frozen")),
        ("end_to_end", boolean_check(
            "experiment_protocol_frozen",
            None if experiment_protocol is None else experiment_protocol.get("frozen"),
            "experiment_protocol.frozen",
        )),
        ("act", CheckResult(
            "act_numeric_observation_24d",
            observation.get("dimension") == 24,
            f"numeric_observation.dimension={observation.get('dimension')!r}",
        )),
        ("act", CheckResult(
            "act_base_twist_3d",
            action.get("dimension") == 3
            and action.get("semantics") == "body_twist_only"
            and action.get("order") == ["base_vx", "base_vy", "base_wz"],
            f"action.dimension={action.get('dimension')!r}, semantics={action.get('semantics')!r}",
        )),
        ("act", CheckResult(
            "act_has_no_manipulation_authority",
            act_scope.get("act_may_command_lift_arm_or_gripper") is False,
            "scope.act_may_command_lift_arm_or_gripper="
            f"{act_scope.get('act_may_command_lift_arm_or_gripper')!r}",
        )),
        ("act", boolean_check(
            "arbiter_requires_physical_affordance",
            arbiter_parameters.get("require_physical_affordance"),
            "velocity_arbiter.require_physical_affordance",
        )),
        ("end_to_end", boolean_check(
            "mission_requires_physical_affordance",
            mission_parameters.get("require_physical_affordance"),
            "mission_orchestrator.require_physical_affordance",
        )),
        ("end_to_end", boolean_check(
            "mission_requires_physical_perception",
            mission_parameters.get("require_physical_perception"),
            "mission_orchestrator.require_physical_perception",
        )),
    ]
    dependencies = {
        "perception": {"perception"},
        "tcwam": {"perception", "tcwam"},
        "act": {"perception", "tcwam", "act"},
        "end_to_end": {"perception", "tcwam", "act", "end_to_end"},
    }
    return [result for tag, result in tagged if tag in dependencies[scope]]


def gate_checks(hardware_gates: dict[str, Any], through_gate: str) -> list[CheckResult]:
    gates = hardware_gates.get("gates", {})
    ordering = hardware_gates.get("ordering", [])
    if through_gate not in ordering:
        raise ValueError(f"unknown gate {through_gate!r}; expected one of {ordering}")
    output = []
    for gate_name in ordering[: ordering.index(through_gate) + 1]:
        gate = gates.get(gate_name, {})
        output.append(
            CheckResult(
                f"hardware_gate_{gate_name}",
                gate.get("pass") is True,
                f"pass={gate.get('pass')!r}; {gate.get('requirement', 'requirement missing')}",
            )
        )
    return output


def resolve_default_paths() -> dict[str, Path]:
    from ament_index_python.packages import get_package_share_directory

    tc = Path(get_package_share_directory("openflex_tc_wam")) / "config"
    control = Path(get_package_share_directory("openflex_thesis_control")) / "config"
    bringup = Path(get_package_share_directory("openflex_thesis_bringup")) / "config"
    experiments = Path(
        get_package_share_directory("openflex_thesis_experiments")
    ) / "config"
    return {
        "shelf_scene": tc / "shelf_scene.yaml",
        "perception": experiments / "legacy_perception.yaml",
        "control": control / "control.yaml",
        "act_schema": control / "act_schema.yaml",
        "hardware_gates": bringup / "hardware_gates.yaml",
        "experiment_protocol": experiments / "experiment_protocol.yaml",
    }


def parser(defaults: dict[str, Path]) -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(
        description="Fail closed unless physical thesis evidence prerequisites are explicit."
    )
    for name, path in defaults.items():
        result.add_argument(f"--{name.replace('_', '-')}", type=Path, default=path)
    result.add_argument(
        "--scope",
        choices=["perception", "tcwam", "act", "end_to_end"],
        default="end_to_end",
        help="Only require configuration needed by this evidence stage.",
    )
    result.add_argument(
        "--through-gate",
        default="G0_static_model",
        help="Require hardware gates from G0 through this gate (default: G0_static_model).",
    )
    result.add_argument(
        "--skip-hardware-gates",
        action="store_true",
        help="Check configuration consistency only; never use this for physical evidence.",
    )
    return result


def print_results(results: Iterable[CheckResult]) -> bool:
    passed = True
    for item in results:
        label = "PASS" if item.passed else "FAIL"
        print(f"[{label}] {item.name}: {item.detail}")
        passed = passed and item.passed
    return passed


def main(argv=None) -> int:
    defaults = resolve_default_paths()
    args = parser(defaults).parse_args(argv)
    try:
        shelf = load_yaml(args.shelf_scene)
        perception = load_yaml(args.perception)
        control = load_yaml(args.control)
        schema = load_yaml(args.act_schema)
        protocol = load_yaml(args.experiment_protocol)
        results = physical_readiness_checks(
            shelf,
            perception,
            control,
            schema,
            experiment_protocol=protocol,
            scope=args.scope,
        )
        if not args.skip_hardware_gates:
            results.extend(gate_checks(load_yaml(args.hardware_gates), args.through_gate))
    except (OSError, ValueError, yaml.YAMLError) as error:
        print(f"[FAIL] preflight_configuration: {error}")
        return 2
    passed = print_results(results)
    print(
        "PHYSICAL_EVIDENCE_PREFLIGHT_PASSED"
        if passed
        else "PHYSICAL_EVIDENCE_PREFLIGHT_FAILED"
    )
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
