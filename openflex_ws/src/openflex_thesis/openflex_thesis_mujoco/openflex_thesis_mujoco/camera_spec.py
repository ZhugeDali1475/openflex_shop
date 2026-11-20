from dataclasses import dataclass


@dataclass(frozen=True)
class HeadCameraSpec:
    parent_body: str = "head_yaw_link"
    optical_frame: str = "thesis_head_color_optical_frame"
    position: tuple = (0.0, 0.087, 0.055)
    optical_quaternion_wxyz: tuple = (0.7071067811865476, -0.7071067811865476, 0.0, 0.0)
    fovy_degrees: float = 70.0
    provenance: str = "simulation_mesh_estimated_not_physically_calibrated"


HEAD_CAMERA = HeadCameraSpec()


@dataclass(frozen=True)
class Mid360LidarSpec:
    """Simulated Livox MID-360 range sensor (ray-cast, not a physical unit).

    The real MID-360 is non-repetitive; the simulation approximates it with a
    fixed grid of elevation rings whose span matches the MID-360 vertical field
    of view (-7 deg to +52 deg).  Points are emitted in the sensor frame.
    """

    parent_body: str = "mid360_link"
    frame: str = "mid360_link"
    elevation_min_deg: float = -7.0
    elevation_max_deg: float = 52.0
    channels: int = 8
    azimuth_bins: int = 240
    min_range_m: float = 0.5
    max_range_m: float = 20.0
    provenance: str = "simulation_raycast_not_physical"


MID360_LIDAR = Mid360LidarSpec()
