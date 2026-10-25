import math
import struct

import numpy as np
import pytest

from genesis_mobile_sim.lidar import (
    AZIMUTHS,
    ELEVATIONS,
    PLANAR_RING,
    cloud_bytes,
    scan_ranges,
)


def test_planar_scan_keeps_angular_order_and_does_not_project_other_rings():
    distances = np.full((len(AZIMUTHS), len(ELEVATIONS)), 0.5)
    distances[:, PLANAR_RING] = np.linspace(1, 9, len(AZIMUTHS))
    distances[:6, PLANAR_RING] = [0.08, 0.079, 10, math.inf, math.nan, -1]
    ranges = scan_ranges(distances)
    assert len(ranges) == 360
    assert ranges[0] == 0.08
    assert all(math.isinf(value) for value in ranges[1:6])
    assert ranges[6:] == distances[6:, PLANAR_RING].tolist()


def test_cloud_uses_sensor_directions_and_excludes_misses():
    distances = np.array([[2, 3, 10, math.inf, math.nan, 0.07, -1]])
    directions = np.array([[[1, 0, 0], [0, 0.6, 0.8], *([[0, 0, 1]] * 5)]])
    packed = cloud_bytes(distances, directions)
    # Independent ROS wire decoding: three little-endian float32s per hit.
    assert len(packed) == 2 * 12
    assert np.array(list(struct.iter_unpack("<fff", packed))) == pytest.approx(
        np.array([(2, 0, 0), (0, 1.8, 2.4)])
    )
    assert cloud_bytes(np.full_like(distances, 10), directions) == b""
