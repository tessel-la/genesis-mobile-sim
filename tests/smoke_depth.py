"""Validate real Go2 RGB/depth consistency without ROS or motion commands."""

import json
import os
import time

import numpy as np

from genesis_mobile_sim.world import World

world = World(os.environ.get("GENESIS_DEVICE", "cpu"))
try:
    for _ in range(30):
        world.step(time.monotonic())
    rgb, depth = world.read_camera(rgb=True, depth=True)
    assert rgb.shape == (240, 320, 3) and rgb.std() > 1
    assert depth.shape == (240, 320) and depth.dtype == np.dtype("<f4")
    valid = depth > 0
    assert valid.sum() > depth.size // 4 and np.isfinite(depth).all()
    color, depth_only = world.read_camera(rgb=False, depth=True)
    difference = np.abs(depth - depth_only)
    print(
        {
            "color_none": color is None,
            "max_diff": float(difference.max()),
            "changed_pixels": int((difference > 0.01).sum()),
            "center_combined": float(depth[120, 160]),
            "center_depth_only": float(depth_only[120, 160]),
        },
        flush=True,
    )
    assert color is None and np.allclose(depth, depth_only, atol=1e-4)
    print(
        json.dumps(
            {
                "device": str(world.gs.device),
                "rgb_shape": rgb.shape,
                "valid_depth_pixels": int(valid.sum()),
                "depth_metres": [float(depth[valid].min()), float(depth[valid].max())],
            }
        ),
        flush=True,
    )
finally:
    world.close()
