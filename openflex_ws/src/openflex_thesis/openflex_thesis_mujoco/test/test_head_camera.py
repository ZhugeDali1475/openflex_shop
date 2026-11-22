import math
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

from builtin_interfaces.msg import Time
import mujoco
import numpy as np
import pytest
from sensor_msgs.msg import CameraInfo

from openflex_thesis_mujoco.bridge_node import CAMERAS, MujocoBridgeNode
from openflex_thesis_mujoco.camera_spec import HEAD_CAMERA, MID360_LIDAR
from openflex_thesis_mujoco.model_builder import build_thesis_model


@pytest.fixture
def camera_scene(tmp_path):
    source = Path(__file__).resolve().parents[3] / "openflex_mujoco/openflex_mujoco.xml"
    path = build_thesis_model(str(source), str(tmp_path / "scene.xml"))
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return path, model, data


def test_head_camera_is_on_head_not_chassis(camera_scene):
    path, model, data = camera_scene
    tree = ET.parse(path)
    assert tree.find(f".//body[@name='head_yaw_link']/body[@name='{HEAD_CAMERA.optical_frame}']/camera[@name='thesis_head_camera']") is not None
    assert tree.find(".//body[@name='d435_color_optical_frame']/camera") is None
    assert CAMERAS["head"][2] == HEAD_CAMERA.optical_frame
    camera = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "thesis_head_camera")
    assert data.cam_xpos[camera, 2] > 1.2
    assert (-data.cam_xmat[camera].reshape(3, 3)[:, 2])[0] > 0.99


def test_optical_frame_matches_renderer_and_camera_info(camera_scene):
    _, model, data = camera_scene
    camera = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "thesis_head_camera")
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, HEAD_CAMERA.optical_frame)
    renderer_rotation = data.cam_xmat[camera].reshape(3, 3)
    optical_rotation = data.xmat[body].reshape(3, 3)
    np.testing.assert_allclose(renderer_rotation @ np.diag([1, -1, -1]), optical_rotation, atol=1e-8)
    np.testing.assert_allclose(data.cam_xpos[camera], data.xpos[body])
    bridge = SimpleNamespace(model=model, get_parameter=lambda name: SimpleNamespace(value={
        "image_width": 320, "image_height": 240,
    }[name]))
    info = MujocoBridgeNode._camera_info(bridge, Time(sec=2), HEAD_CAMERA.optical_frame, "thesis_head_camera")
    assert info.header.frame_id == HEAD_CAMERA.optical_frame
    assert info.k[4] == pytest.approx(120 / math.tan(math.radians(HEAD_CAMERA.fovy_degrees) / 2))


def test_camera_outside_front_shell_and_target_ray_not_robot(camera_scene):
    _, model, data = camera_scene
    camera = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "thesis_head_camera")
    target = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "thesis_object_geom")
    vector = data.geom_xpos[target] - data.cam_xpos[camera]
    geom_id = np.array([-1], dtype=np.int32)
    distance = mujoco.mj_ray(model, data, data.cam_xpos[camera], vector / np.linalg.norm(vector), None, 1, -1, geom_id)
    assert distance > 0.5
    assert geom_id[0] == target


def test_camera_publication_uses_render_snapshot_tf_and_identical_stamps(camera_scene):
    _, model, data = camera_scene
    published, transforms = {}, []

    class Renderer:
        depth = False

        def disable_depth_rendering(self):
            self.depth = False

        def enable_depth_rendering(self):
            self.depth = True

        def update_scene(self, sample, camera):
            assert sample is data

        def render(self):
            return np.ones((2, 3), dtype=np.float32) if self.depth else np.zeros((2, 3, 3), dtype=np.uint8)

    bridge = SimpleNamespace(
        model=model, _id=lambda kind, name: mujoco.mj_name2id(model, kind, name),
        _mujoco_body_pose=MujocoBridgeNode._mujoco_body_pose,
        tf_broadcaster=SimpleNamespace(sendTransform=transforms.append),
        renderers={key: Renderer() for key in CAMERAS},
        _camera_info=lambda stamp, frame, name: CameraInfo(),
        camera_publishers={key: {kind: SimpleNamespace(publish=lambda message, key=key, kind=kind:
                            published.__setitem__((key, kind), message))
                            for kind in ("color", "depth", "info")} for key in CAMERAS},
    )
    import rclpy
    from unittest.mock import patch

    with patch.object(rclpy, "ok", return_value=True):
        MujocoBridgeNode._publish_cameras(bridge, data, Time(sec=7))
    assert len(transforms) == 1
    transform = transforms[0]
    assert transform.child_frame_id == HEAD_CAMERA.optical_frame
    assert transform.header.frame_id == "map"
    assert transform.header.stamp == published[("head", "color")].header.stamp == Time(sec=7)
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, HEAD_CAMERA.optical_frame)
    assert transform.transform.translation.z == pytest.approx(data.xpos[body, 2])


def test_camera_publication_can_disable_ground_truth_tf(camera_scene):
    _, model, data = camera_scene
    transforms = []

    class Renderer:
        depth = False
        def disable_depth_rendering(self): self.depth = False
        def enable_depth_rendering(self): self.depth = True
        def update_scene(self, sample, camera): pass
        def render(self):
            return np.ones((2, 3), dtype=np.float32) if self.depth else np.zeros((2, 3, 3), dtype=np.uint8)

    bridge = SimpleNamespace(
        model=model, _id=lambda kind, name: mujoco.mj_name2id(model, kind, name),
        _mujoco_body_pose=MujocoBridgeNode._mujoco_body_pose,
        get_parameter=lambda name: SimpleNamespace(value=False)
        if name == "publish_ground_truth_tf" else SimpleNamespace(value={"image_width": 320, "image_height": 240}[name]),
        tf_broadcaster=SimpleNamespace(sendTransform=transforms.append),
        renderers={key: Renderer() for key in CAMERAS},
        _camera_info=lambda stamp, frame, name: CameraInfo(),
        camera_publishers={key: {kind: SimpleNamespace(publish=lambda message: None)
                            for kind in ("color", "depth", "info")} for key in CAMERAS},
    )
    import rclpy
    from unittest.mock import patch
    with patch.object(rclpy, "ok", return_value=True):
        MujocoBridgeNode._publish_cameras(bridge, data, Time(sec=8))
    assert transforms == []


def test_simulated_lidar_does_not_publish_ground_truth_sensor_pose(camera_scene):
    _, model, data = camera_scene
    lidar_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "mid360_link")
    directions = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float64)
    published, transforms = [], []

    def raycast(_model, _data, _point, _vectors, _groups, _mode, _exclude,
                _geom_ids, distances, _rays, _max_range):
        distances[:] = 1.0

    bridge = SimpleNamespace(
        lidar_pub=SimpleNamespace(publish=published.append),
        lidar_body_id=lidar_body,
        model=model,
        _lidar_dirs=directions,
        _lidar_geomgroup=np.ones(6, dtype=np.int32),
        _quaternion_to_matrix=MujocoBridgeNode._quaternion_to_matrix,
        get_parameter=lambda name: SimpleNamespace(value={
            "lidar_max_range_m": 20.0,
            "lidar_min_range_m": 0.1,
            "lidar_noise_std_m": 0.0,
        }[name]),
        tf_broadcaster=SimpleNamespace(sendTransform=transforms.append),
    )

    from unittest.mock import patch

    with patch.object(mujoco, "mj_multiRay", side_effect=raycast):
        MujocoBridgeNode._publish_lidar(bridge, data, Time(sec=9))

    assert len(published) == 1
    assert published[0].header.frame_id == MID360_LIDAR.frame
    assert published[0].header.stamp == Time(sec=9)
    assert transforms == []
