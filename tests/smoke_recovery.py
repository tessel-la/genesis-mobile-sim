"""Physically recover the existing Go2; test resets are confined to this script."""

import json
import os
import time

from genesis_mobile_sim.world import World

world = World(os.environ.get("GENESIS_DEVICE", "cpu"))


def run(count, command=None):
    for _ in range(count):
        now = time.monotonic()
        if command is not None:
            world.command(*command, now)
        world.step(now)


try:
    run(100)
    for _ in range(2):
        run(50, (0.2, 0, 0))
        world.commands.received_at = time.monotonic() - 1
        run(50)
        assert world.motion.mode == "stand"
    results = {}
    for name, quaternion in (
        ("back", (0.0, 1.0, 0.0, 0.0)),
        ("side", (0.70710678, 0.70710678, 0.0, 0.0)),
    ):
        world.robot.set_pos((0.0, 0.0, 0.35))
        world.robot.set_quat(quaternion)
        world.robot.set_dofs_velocity(
            world.policy.torch.zeros(world.robot.n_dofs, device=world.gs.device)
        )
        world.robot.set_dofs_position(world.recovery.default[world.recovery.inverse], world.motors)
        world.request_motion("recover")
        assert not world.scan_valid
        started = world.steps
        for _ in range(600):
            run(1)
            if world.motion.mode != "recover":
                break
        assert world.motion.mode == "stand", world.motion.reason
        assert world.upright > 0.92 and float(world.base.get_pos()[2]) > 0.26
        results[name] = {
            "simulated_seconds": (world.steps - started) * 0.02,
            "height_m": float(world.base.get_pos()[2]),
            "upright": world.upright,
        }
        run(100)
        assert world.scan_valid
        run(20, (0.2, 0, 0))
        assert world.motion.mode == "walk"
        world.commands.received_at = time.monotonic() - 1
        run(50)
    print(json.dumps({"backend": str(world.gs.device), "recovered": results}), flush=True)
finally:
    world.close()
