"""Profile real physics and sensor reads without ROS or motion commands.

Run inside the simulator image with the desired GENESIS_DEVICE. Kernel compilation
and the first camera frame are excluded. This does not connect to a ROS domain.
"""

import json
import os
import statistics
import time

from genesis_mobile_sim.world import DT, World

world = World(os.environ.get("GENESIS_DEVICE", "cpu"))
samples = {"step": [], "lidar": [], "camera": []}


def measure(name, action):
    start = time.perf_counter()
    action()
    # Sensor reads and world.step already synchronize CPU-visible state.
    samples[name].append(time.perf_counter() - start)


try:
    for _ in range(30):
        world.step(time.monotonic())
    world.array(world.camera.read().rgb)
    started = time.perf_counter()
    for _ in range(200):
        measure("step", lambda: world.step(time.monotonic()))
        if world.steps % 10 == 0:
            measure("lidar", lambda: world.array(world.lidar.read().distances))
            measure("camera", lambda: world.array(world.camera.read().rgb))
    elapsed = time.perf_counter() - started
    print(
        json.dumps(
            {
                "device": str(world.gs.device),
                "unpaced_real_time_factor": 200 * DT / elapsed,
                "mean_ms": {key: statistics.mean(value) * 1000 for key, value in samples.items()},
                "max_ms": {key: max(value) * 1000 for key, value in samples.items()},
            }
        ),
        flush=True,
    )
finally:
    world.close()
