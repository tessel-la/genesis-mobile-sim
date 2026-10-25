"""One spherical measurement supplies planar navigation and a compact 3D cloud."""

import numpy as np

AZIMUTHS = tuple(range(-180, 180))
ELEVATIONS = tuple(range(-16, 17, 2))
PLANAR_RING = ELEVATIONS.index(0)
MIN_RANGE, MAX_RANGE = 0.08, 10.0


def scan_ranges(distances):
    """Preserve misses as infinity for Nav2's clearing rays."""
    planar = np.asarray(distances)[:, PLANAR_RING]
    valid = np.isfinite(planar) & (planar >= MIN_RANGE) & (planar < MAX_RANGE)
    return np.where(valid, planar, np.inf).tolist()


def cloud_bytes(distances, directions):
    """Pack finite hits as little-endian FLOAT32 XYZ in the laser frame.

    Genesis's distances-only sensor avoids allocating XYZ history every physics
    step. Project only when a cloud subscriber needs the current measurement.
    Directions come from the same Genesis pattern that casts the rays.
    """
    distances = np.asarray(distances)
    valid = np.isfinite(distances) & (distances >= MIN_RANGE) & (distances < MAX_RANGE)
    points = distances[valid, None] * directions[valid]
    return np.asarray(points, dtype="<f4").tobytes()
