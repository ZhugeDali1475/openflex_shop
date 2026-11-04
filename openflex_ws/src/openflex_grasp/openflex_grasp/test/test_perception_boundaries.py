import numpy as np

from openflex_grasp.perception.cache import TtlCache
from openflex_grasp.perception.rgbd import CameraIntrinsics, FrozenRgbdFrame


def test_frozen_rgbd_frame_owns_immutable_aligned_arrays():
    rgb = np.zeros((2, 3, 3), dtype=np.uint8)
    depth = np.ones((2, 3), dtype=np.uint16)
    frame = FrozenRgbdFrame(
        rgb=rgb,
        depth=depth,
        intrinsics=CameraIntrinsics(100.0, 100.0, 1.0, 1.0, 3, 2),
        arm_name="left",
        camera_serial="left-test",
        frame_id="left_color_optical_frame",
        stamp_ns=123,
        depth_scale=0.001,
    )
    rgb[0, 0] = 255
    depth[0, 0] = 999
    assert frame.rgb[0, 0].tolist() == [0, 0, 0]
    assert int(frame.depth[0, 0]) == 1
    assert not frame.rgb.flags.writeable
    assert not frame.depth.flags.writeable


def test_ttl_cache_expires_and_supports_single_pop():
    now = [10.0]
    cache = TtlCache(clock=lambda: now[0])
    cache.put("one", {"value": 1}, 2.0)
    assert cache.get("one") == {"value": 1}
    assert cache.pop("one") == {"value": 1}
    assert cache.pop("one") is None
    cache.put("two", 2, 1.0)
    now[0] = 11.0
    assert cache.get("two") is None
