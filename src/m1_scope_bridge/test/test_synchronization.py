import math
import sys
from pathlib import Path

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from m1_scope_bridge.synchronization import (  # noqa: E402
    common_coverage_mask, interpolate_odometry)


def test_interpolate_odometry_uses_shortest_yaw_arc():
    stamps = np.array([0, 1_000_000_000], dtype=np.int64)
    poses = np.array([[0.0, 0.0, math.radians(179.0)],
                      [2.0, 4.0, math.radians(-179.0)]])
    twists = np.array([[0.0, 1.0, 2.0], [2.0, 3.0, 4.0]])

    pose, twist = interpolate_odometry(
        stamps, poses, twists, np.array([500_000_000], dtype=np.int64))

    assert np.allclose(pose[0, :2], [1.0, 2.0])
    assert abs(abs(pose[0, 2]) - math.pi) < 1e-9
    assert np.allclose(twist[0], [1.0, 2.0, 3.0])


def test_interpolate_odometry_rejects_extrapolation():
    stamps = np.array([10, 20], dtype=np.int64)
    poses = np.zeros((2, 3))
    twists = np.zeros((2, 3))

    try:
        interpolate_odometry(stamps, poses, twists, np.array([9]))
    except ValueError as error:
        assert "outside odometry coverage" in str(error)
    else:
        raise AssertionError("expected extrapolation to be rejected")


def test_common_coverage_mask_intersects_all_series_boundaries():
    queries = np.array([0, 5, 10, 15, 20, 25, 30], dtype=np.int64)

    mask = common_coverage_mask(
        queries, np.array([0, 30]), np.array([5, 25]), np.array([10, 20]))

    assert mask.tolist() == [False, False, True, True, True, False, False]
