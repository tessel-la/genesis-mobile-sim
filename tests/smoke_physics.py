"""Exercise the existing Go2 and pretrained policy on CPU, including actual travel."""

import math
import os
import time

from genesis_mobile_sim.world import World

world = World(os.environ.get("GENESIS_DEVICE", "cpu"))


def run(steps, command=None):
    for _ in range(steps):
        now = time.monotonic()
        if command is not None:
            world.command(*command, now)
        world.step(now)


def yaw():
    w, x, y, z = world.array(world.base.get_quat())
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


try:
    run(100)
    before = world.array(world.lidar.read().distances).copy()
    assert before.shape == (360, 1)
    assert before.min() > 1.1, float(before.min())
    assert 6.5 < before[180, 0] < 7.1, before[180, 0]
    rgb = world.array(world.camera.read().rgb)
    assert rgb.shape == (240, 320, 3) and rgb.std() > 1, rgb.shape
    start = world.array(world.base.get_pos()).copy()
    run(200, (0.2, 0, 0))
    end = world.array(world.base.get_pos()).copy()
    assert end[0] - start[0] > 0.3, (start, end)
    after = world.array(world.lidar.read().distances)
    assert before[180, 0] - after[180, 0] > 0.2, (before[180], after[180])
    heading = yaw()
    run(150, (0, 0, 0.4))
    turn = math.atan2(math.sin(yaw() - heading), math.cos(yaw() - heading))
    assert turn > 0.4, turn
    # Expire the command by wall time, then allow the published policy to settle.
    world.commands.received_at = time.monotonic() - 1
    run(100)
    stopped = world.array(world.base.get_pos()).copy()
    run(100)
    drift = float(world.np.linalg.norm(world.array(world.base.get_pos())[:2] - stopped[:2]))
    assert drift < 0.15, drift
    print(
        {
            "robot": "existing Unitree Go2",
            "physical_forward_travel_m": float(end[0] - start[0]),
            "turn_rad": turn,
            "standing_drift_m": drift,
            "lidar_rays": before.size,
            "camera_shape": rgb.shape,
            "resolved_device": str(world.gs.device),
        },
        flush=True,
    )
finally:
    world.close()
