"""Resolve simulation-only planning configuration without confusing MJCF and YAML."""

from pathlib import Path
from typing import Mapping


def resolve_shelf_scene(default_scene: Path, environment: Mapping[str, str]) -> str:
    """Return a YAML collision-scene path, preferring the active runtime's manifest.

    OPENFLEX_SIM_SHELF_SCENE_CONFIG is the explicit setting. The older
    OPENFLEX_SIM_SHELF_SCENE variable is tolerated, but an MJCF path resolves to
    the adjacent generated shelf_scene.yaml rather than being passed to PyYAML.
    """
    explicit = environment.get("OPENFLEX_SIM_SHELF_SCENE_CONFIG")
    legacy = environment.get("OPENFLEX_SIM_SHELF_SCENE")
    model = environment.get("OPENFLEX_SIM_SCENE_MODEL")

    if explicit:
        candidate = Path(explicit).expanduser()
    elif legacy:
        candidate = Path(legacy).expanduser()
        if candidate.suffix.lower() == ".xml":
            candidate = candidate.with_name("shelf_scene.yaml")
    elif model:
        candidate = Path(model).expanduser().with_name("shelf_scene.yaml")
        if not candidate.is_file():
            candidate = Path(default_scene)
    else:
        candidate = Path(default_scene)

    if candidate.suffix.lower() not in {".yaml", ".yml"}:
        raise ValueError(
            "MoveIt shelf_scene_file must be YAML; pass "
            "OPENFLEX_SIM_SHELF_SCENE_CONFIG, not a scene.xml path"
        )
    return str(candidate.resolve(strict=True))
