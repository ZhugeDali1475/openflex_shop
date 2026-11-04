import math

import numpy as np


def require_pose_above_minimum(pose, minimum_tcp_z, label):
    minimum = float(minimum_tcp_z)
    height = float(pose.pose.position.z)
    if not math.isfinite(minimum) or not math.isfinite(height):
        raise ValueError(f"{label} TCP height must be finite")
    if height + 1e-6 < minimum:
        raise ValueError(
            f"{label} TCP height {height:.3f} m is below the product-cloud "
            f"minimum path height {minimum:.3f} m"
        )


def pointcloud_xyz(message) -> np.ndarray:
    fields = {field.name: field for field in message.fields}
    missing = {"x", "y", "z"} - fields.keys()
    if missing:
        raise ValueError(f"PointCloud2 is missing fields: {sorted(missing)}")
    byte_order = ">" if bool(message.is_bigendian) else "<"
    dtype = np.dtype({
        "names": ["x", "y", "z"],
        "formats": [f"{byte_order}f4"] * 3,
        "offsets": [fields[axis].offset for axis in ("x", "y", "z")],
        "itemsize": message.point_step,
    })
    count = int(message.width) * max(1, int(message.height))
    records = np.frombuffer(message.data, dtype=dtype, count=count)
    points = np.column_stack((records["x"], records["y"], records["z"]))
    points = points[np.isfinite(points).all(axis=1)]
    if points.size == 0:
        raise ValueError("PointCloud2 contains no finite XYZ points")
    return points.astype(np.float64, copy=False)


def transformed_minimum_z(points, translation, quaternion) -> float:
    values = np.asarray(points, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 3 or values.shape[0] == 0:
        raise ValueError("points must have shape (N, 3)")
    values = values[np.isfinite(values).all(axis=1)]
    if values.shape[0] == 0:
        raise ValueError("points contain no finite XYZ rows")

    qx, qy, qz, qw = (float(value) for value in quaternion)
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("transform quaternion must be finite and nonzero")
    qx, qy, qz, qw = (value / norm for value in (qx, qy, qz, qw))
    rotation_z = np.array([
        2.0 * (qx * qz - qw * qy),
        2.0 * (qy * qz + qw * qx),
        1.0 - 2.0 * (qx * qx + qy * qy),
    ])
    translated_z = values @ rotation_z + float(tuple(translation)[2])
    minimum = float(np.min(translated_z))
    if not math.isfinite(minimum):
        raise ValueError("transformed point-cloud minimum Z is not finite")
    return minimum
