import math
import sys
from pathlib import Path

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from m1_scope_bridge.scope_ogm import scan_to_points  # noqa: E402


def test_scan_to_points_uses_laserscan_metadata():
    points = scan_to_points(
        np.array([1.0, 2.0, 1.0]), -math.pi / 2.0, math.pi / 2.0,
        0.05, 12.0)
    assert np.allclose(points, [[0.0, -1.0], [2.0, 0.0], [0.0, 1.0]],
                       atol=1e-7)


def test_scan_to_points_filters_nonfinite_and_out_of_range():
    points = scan_to_points(
        np.array([np.nan, np.inf, 0.04, 12.01, 3.0]),
        0.0, 0.1, 0.05, 12.0)
    assert points.shape == (1, 2)
    assert np.allclose(points[0], [3.0 * math.cos(0.4), 3.0 * math.sin(0.4)])
