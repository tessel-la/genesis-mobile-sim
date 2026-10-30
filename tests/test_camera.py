from types import SimpleNamespace

import numpy as np

from genesis_mobile_sim.world import CAMERA_FAR, CAMERA_NEAR, World


def test_camera_moves_with_mount_and_masks_invalid_axial_metres():
    calls = []
    values = np.array([[0, -1, np.nan, np.inf, CAMERA_NEAR, CAMERA_FAR, 2.5]])
    camera = SimpleNamespace(
        move_to_attach=lambda: calls.append("move"),
        render=lambda **channels: (calls.append(channels), values, None, None),
    )
    rgb, depth = World.read_camera(SimpleNamespace(camera=camera, np=np), rgb=False, depth=True)
    assert calls == ["move", {"rgb": True, "depth": True}]
    assert depth.dtype == np.dtype("<f4")
    assert depth.tolist() == [[0, 0, 0, 0, 0, 0, 2.5]]


def test_rgb_only_requests_no_depth_work():
    calls = []
    pixels = np.ones((2, 2, 3), dtype="uint8")
    camera = SimpleNamespace(
        move_to_attach=lambda: None,
        render=lambda **channels: (calls.append(channels) or pixels, None, None, None),
    )
    rgb, depth = World.read_camera(SimpleNamespace(camera=camera, np=np))
    assert rgb is pixels and depth is None
    assert calls == [{"rgb": True, "depth": False}]
