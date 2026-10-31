"""Read-only ROS timing probe; never publishes commands or requests motion."""

import json
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, LaserScan, PointCloud2

rclpy.init()
node = Node("mobile_timing_probe")
latest = {}
counts = {}
arrivals = {}
gaps = {}


def record(name):
    def receive(message):
        now = time.monotonic()
        if name in arrivals:
            gaps.setdefault(name, []).append(now - arrivals[name])
        arrivals[name] = now
        latest[name] = message
        counts[name] = counts.get(name, 0) + 1
    return receive


subscriptions = [
    node.create_subscription(Clock, "/clock", record("clock"), 1),
    node.create_subscription(Odometry, "/odom", record("odom"), 1),
    node.create_subscription(LaserScan, "/scan", record("scan"), qos_profile_sensor_data),
]


def spin_for(seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)


def sim_time():
    clock = latest["clock"].clock
    return clock.sec + clock.nanosec * 1e-9


try:
    deadline = time.monotonic() + 120
    while "clock" not in latest:
        assert time.monotonic() < deadline, "No simulation clock"
        rclpy.spin_once(node, timeout_sec=0.1)
    for viewers in (False, True):
        if viewers:
            subscriptions.extend([
                node.create_subscription(Image, "/camera/image_raw", record("camera"), 1),
                node.create_subscription(
                    PointCloud2, "/points", record("points"), qos_profile_sensor_data
                ),
            ])
            # The first EGL frame can take much longer than steady-state rendering.
            deadline = time.monotonic() + 120
            while "camera" not in latest:
                assert time.monotonic() < deadline, "No camera frame after warmup"
                rclpy.spin_once(node, timeout_sec=0.1)
        spin_for(2)
        counts.clear()
        arrivals.clear()
        gaps.clear()
        start, stamp = time.monotonic(), sim_time()
        spin_for(8)
        elapsed, simulated = time.monotonic() - start, sim_time() - stamp
        print(json.dumps({
            "camera_and_cloud_subscribed": viewers,
            "real_time_factor": simulated / elapsed,
            "wall_hz": {key: count / elapsed for key, count in counts.items()},
            "max_gap_ms": {key: max(values) * 1000 for key, values in gaps.items()},
            "cloud_hits": latest["points"].width if "points" in latest else None,
        }), flush=True)
finally:
    node.destroy_node()
    rclpy.shutdown()
