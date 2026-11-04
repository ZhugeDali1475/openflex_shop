from pathlib import Path


REQUIRED_MODEL_FILES = (
    "Qwen3.5-4B/config.json",
    "sam2.1-hiera-small/config.json",
    "graspnet-baseline/checkpoint-rs.tar",
    "SHA256SUMS",
)


def validate_model_root(model_root: str) -> list[str]:
    root = Path(model_root).expanduser()
    return [str(root / relative) for relative in REQUIRED_MODEL_FILES if not (root / relative).is_file()]


def duplicate_backend_owners(node_names) -> list[str]:
    monitored = (
        "/openflex_detect_object",
        "/openflex_graspnet_worker",
        "/openflex_pick_object",
        "/move_group",
    )
    return [name for name in monitored if list(node_names).count(name) > 1]
