"""Grouped offline fitting of short-horizon recovery probabilities."""

import argparse
import hashlib
import json
from pathlib import Path

from openflex_care_agent.experience import FEATURES, feature_row


def read_transitions(paths):
    records = []
    identities = set()
    for path in paths:
        for line in Path(path).read_text(encoding="utf-8").splitlines():
            record = json.loads(line)
            if record.get("event") != "transition" or record.get("label") not in (0, 1):
                continue
            if record.get("evidence_class") != "mujoco_oracle_geometry_proxy":
                raise ValueError("training requires classified simulator evidence, not mock/synthetic records")
            if not record.get("scenario_id") or not record.get("task_id"):
                raise ValueError("scenario and task identifiers are required")
            if record.get("partition") == "held_out":
                raise ValueError("formal held-out evaluation records cannot be used for fitting")
            identity = (record["task_id"], record.get("monotonic_sec"))
            if identity in identities:
                raise ValueError("duplicate transitions cannot be counted as independent samples")
            identities.add(identity)
            records.append(record)
    return records


def grouped_split(records, seed=7):
    task_scenarios = {}
    for record in records:
        previous = task_scenarios.setdefault(record["task_id"], record["scenario_id"])
        if previous != record["scenario_id"]:
            raise ValueError("one task cannot span different scenarios")
    groups = sorted(set(record["scenario_id"] for record in records),
                    key=lambda group: hashlib.sha256(f"{seed}:{group}".encode()).hexdigest())
    if len(groups) < 5:
        raise ValueError("at least five independent scenarios are required")
    held_out = max(1, len(groups) // 5)
    splits = {group: "test" if index < held_out else "validation" if index < 2 * held_out else "train"
              for index, group in enumerate(groups)}
    return {name: [record for record in records if splits[record["scenario_id"]] == name]
            for name in ("train", "validation", "test")}


def rows(records, use_memory, drop_capability=False):
    output = []
    for record in records:
        features = dict(record["features"])
        if not use_memory:
            features["repeat_count"] = 0.0
        if drop_capability:
            for name in ("score", "joint_margin", "robustness", "grasp_quality", "candidate_count"):
                features[name] = 0.0
        output.append(feature_row(features, record["action"], record["reason"], record["stage"]))
    return output


def fit_models(records, output_dir, use_memory=True, seed=7, drop_capability=False):
    import joblib
    from sklearn.ensemble import GradientBoostingClassifier
    from sklearn.feature_extraction import DictVectorizer
    from sklearn.linear_model import LogisticRegression
    from sklearn.metrics import brier_score_loss, log_loss
    from sklearn.pipeline import Pipeline
    from sklearn.preprocessing import StandardScaler
    import sklearn

    splits = grouped_split(records, seed)
    for name, values in splits.items():
        if len(set(record["label"] for record in values)) != 2:
            raise ValueError(f"{name} split needs both outcomes; collect more independent scenarios")
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    results = {}
    fitted = {}
    for name, classifier in (
        ("logistic", LogisticRegression(max_iter=2000, random_state=seed)),
        ("gradient_boosting", GradientBoostingClassifier(n_estimators=50, max_depth=2,
                                                       min_samples_leaf=5, random_state=seed)),
    ):
        pipeline = Pipeline([("vectorizer", DictVectorizer(sparse=False)),
                             ("scale", StandardScaler()), ("classifier", classifier)])
        pipeline.fit(rows(splits["train"], use_memory, drop_capability),
                     [record["label"] for record in splits["train"]])
        probabilities = pipeline.predict_proba(rows(splits["validation"], use_memory, drop_capability))[:, 1]
        results[name] = {"validation_brier": float(brier_score_loss(
            [record["label"] for record in splits["validation"]], probabilities))}
        fitted[name] = pipeline
    selected = min(results, key=lambda name: results[name]["validation_brier"])
    pipeline = fitted[selected]
    probabilities = pipeline.predict_proba(rows(splits["test"], use_memory, drop_capability))[:, 1]
    labels = [record["label"] for record in splits["test"]]
    report = {"seed": seed, "selected_on": "validation_brier", "selected": selected,
              "dataset_sha256": hashlib.sha256(json.dumps(records, sort_keys=True).encode()).hexdigest(),
              "sklearn_version": sklearn.__version__,
              "use_memory": use_memory, "drop_capability": drop_capability,
              "models": results, "test_brier": float(brier_score_loss(labels, probabilities)),
              "test_log_loss": float(log_loss(labels, probabilities, labels=[0, 1])),
              "split_scenarios": {name: sorted(set(record["scenario_id"] for record in values))
                                  for name, values in splits.items()},
              "split_counts": {name: len(values) for name, values in splits.items()},
              "reliability": []}
    for lower in (0.0, 0.2, 0.4, 0.6, 0.8):
        indices = [index for index, probability in enumerate(probabilities)
                   if lower <= probability < lower + 0.2 or lower == 0.8 and probability == 1.0]
        if indices:
            report["reliability"].append({"lower": lower, "count": len(indices),
                "predicted": float(sum(probabilities[index] for index in indices) / len(indices)),
                "observed": sum(labels[index] for index in indices) / len(indices)})
    joblib.dump({"features": list(FEATURES), "pipeline": pipeline, "use_memory": use_memory,
                 "drop_capability": drop_capability, "training_report": report}, output / "model.joblib")
    (output / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("inputs", nargs="+")
    parser.add_argument("--output", required=True)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--no-memory", action="store_true")
    parser.add_argument("--drop-capability", action="store_true")
    parser.add_argument("--gate-report", required=True)
    args = parser.parse_args()
    gate = json.loads(Path(args.gate_report).read_text())["research_gate"]
    if not gate["ready_for_expansion"]:
        parser.error("research go/no-go gate has not passed; do not train on insufficient evidence")
    report = fit_models(read_transitions(args.inputs), args.output, not args.no_memory,
                        args.seed, args.drop_capability)
    print(json.dumps(report, ensure_ascii=False))


if __name__ == "__main__":
    main()
