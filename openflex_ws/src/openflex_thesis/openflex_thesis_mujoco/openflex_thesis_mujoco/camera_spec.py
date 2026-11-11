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
