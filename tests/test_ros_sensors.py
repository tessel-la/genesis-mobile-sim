"""Use real Jazzy message classes when run in the simulator image."""

from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip("rclpy")

from genesis_mobile_sim.lidar import AZIMUTHS, ELEVATIONS, PLANAR_RING  # noqa: E402
from genesis_mobile_sim.ros_bridge import Simulator  # noqa: E402


class Publisher:
    def __init__(self, subscribers=1):
        self.subscribers = subscribers
        self.messages = []

    def get_subscription_count(self):
        return self.subscribers

    def publish(self, message):
        self.messages.append(message)


@pytest.fixture
def simulator():
    distances = np.full((len(AZIMUTHS), len(ELEVATIONS)), 10.0, dtype="float32")
    distances[180, PLANAR_RING] = 2.0
    distances[180, PLANAR_RING + 1] = 3.0
    directions = np.zeros((*distances.shape, 3), dtype="float32")
    directions[..., 0] = 1
    directions[180, PLANAR_RING + 1] = [0, 0.6, 0.8]
    lidar_reads, camera_reads = [], []

    def lidar():
        lidar_reads.append(True)
        return SimpleNamespace(distances=distances)

    def camera():
        camera_reads.append(True)
        return SimpleNamespace(rgb=np.zeros((240, 320, 3), dtype="uint8"))

    world = SimpleNamespace(
        steps=10,
        scan_valid=True,
        array=lambda value: value,
        lidar=SimpleNamespace(read=lidar),
        camera=SimpleNamespace(read=camera),
        lidar_directions=directions,
    )
    node = SimpleNamespace(
        world=world,
        scan_pub=Publisher(),
        cloud_pub=Publisher(),
        image_pub=Publisher(),
        info_pub=Publisher(),
        lidar_reads=lidar_reads,
        camera_reads=camera_reads,
    )
    node.stamp = lambda: Simulator.stamp(node)
    return node


def test_scan_and_cloud_share_one_measurement_and_timestamp(simulator):
    Simulator.publish_lidar(simulator)
    scan, = simulator.scan_pub.messages
    cloud, = simulator.cloud_pub.messages
    assert len(simulator.lidar_reads) == 1
    assert len(scan.ranges) == 360 and scan.ranges[180] == 2.0
    assert scan.scan_time == pytest.approx(0.1)
    assert cloud.header == scan.header
    assert cloud.header.frame_id == "laser" and cloud.header.stamp.nanosec == 200_000_000
    assert cloud.height == 1 and cloud.width == 2
    assert cloud.point_step == 12 and cloud.row_step == len(cloud.data) == 24
    assert [(f.name, f.offset, f.datatype, f.count) for f in cloud.fields] == [
        ("x", 0, 7, 1), ("y", 4, 7, 1), ("z", 8, 7, 1)
    ]
    assert not cloud.is_bigendian and cloud.is_dense
    assert np.frombuffer(bytes(cloud.data), dtype="<f4").reshape(-1, 3) == pytest.approx(
        np.array([[2, 0, 0], [0, 1.8, 2.4]])
    )


def test_recovery_suppresses_only_the_planar_scan(simulator):
    simulator.world.scan_valid = False
    Simulator.publish_lidar(simulator)
    assert not simulator.scan_pub.messages
    assert len(simulator.cloud_pub.messages) == 1
    simulator.cloud_pub.subscribers = 0
    Simulator.publish_lidar(simulator)
    assert len(simulator.lidar_reads) == 1


def test_cloud_runs_at_half_the_planar_rate_and_only_with_subscribers(simulator):
    simulator.world.steps = 5
    Simulator.publish_lidar(simulator)
    assert not simulator.cloud_pub.messages
    simulator.world.steps = 10
    simulator.cloud_pub.subscribers = 0
    Simulator.publish_lidar(simulator)
    assert len(simulator.scan_pub.messages) == 2
    assert not simulator.cloud_pub.messages


def test_camera_renders_only_with_subscribers_and_resumes_on_reconnect(simulator):
    simulator.image_pub.subscribers = 0
    Simulator.publish_camera(simulator)
    assert not simulator.camera_reads and not simulator.image_pub.messages
    assert len(simulator.info_pub.messages) == 1
    simulator.image_pub.subscribers = 1
    Simulator.publish_camera(simulator)
    assert len(simulator.camera_reads) == 1
    image, = simulator.image_pub.messages
    assert image.header == simulator.info_pub.messages[-1].header
    assert image.width == 320 and image.height == 240
    assert image.step == 960 and len(image.data) == 320 * 240 * 3
