from pathlib import Path

import numpy as np


def write_ply(path: Path, points: np.ndarray) -> None:
    header = (
        "ply\nformat ascii 1.0\n"
        f"element vertex {points.shape[0]}\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n"
    )
    with path.open("w", encoding="ascii") as output:
        output.write(header)
        for x, y, z, r, g, b in points:
            output.write(
                f"{x:.6f} {y:.6f} {z:.6f} {int(r)} {int(g)} {int(b)}\n"
            )
