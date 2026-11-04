from dataclasses import dataclass


@dataclass(frozen=True)
class ArmContext:
    arm_name: str
    planning_group: str
    tcp_frame: str
    camera_frame: str
    gripper_action: str
    approach_axis: tuple[float, float, float]
    closing_axis: tuple[float, float, float]
    joint_names: tuple[str, ...]
    payload_mass_kg: float
    payload_com_xyz_m: tuple[float, float, float]

    @property
    def planning_joint_names(self) -> tuple[str, ...]:
        return ("lift_joint",) + self.joint_names


def arm_context(arm_name: str) -> ArmContext:
    arm = str(arm_name).strip().lower()
    if arm == "right":
        return ArmContext(
            arm, "right_arm_with_lift", "openarmx_right_hand_tcp", "right_camera_color_optical_frame",
            "/right_gripper_controller/gripper_cmd", (1.0, 0.0, 0.0), (0.0, -1.0, 0.0),
            tuple(f"openarmx_right_joint{index}" for index in range(1, 8)),
            0.55, (0.0, 0.0, 0.05),
        )
    if arm == "left":
        return ArmContext(
            arm, "left_arm_with_lift", "openarmx_left_hand_tcp", "left_camera_color_optical_frame",
            "/left_gripper_controller/gripper_cmd", (1.0, 0.0, 0.0), (0.0, 1.0, 0.0),
            tuple(f"openarmx_left_joint{index}" for index in range(1, 8)),
            0.55, (0.0, 0.0, 0.05),
        )
    raise ValueError("arm_name must be left or right")
