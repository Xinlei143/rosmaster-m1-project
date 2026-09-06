import math
import sys
from pathlib import Path

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from m1_scope_bridge.se2 import (  # noqa: E402
    integrate_body_twist, invert_transform, pose_to_matrix, transform_points)


def test_integrate_body_twist_supports_forward_and_lateral_motion():
    assert np.allclose(
        integrate_body_twist([1.0, 2.0, 0.0], [2.0, 0.0, 0.0], 0.5),
        [2.0, 2.0, 0.0])
    assert np.allclose(
        integrate_body_twist([1.0, 2.0, 0.0], [0.0, 2.0, 0.0], 0.5),
        [1.0, 3.0, 0.0])


def test_integrate_body_twist_uses_se2_exponential_for_combined_motion():
    result = integrate_body_twist([0.0, 0.0, 0.0], [1.0, 0.5, 1.0], 1.0)
    expected_x = math.sin(1.0) - 0.5 * (1.0 - math.cos(1.0))
    expected_y = 1.0 - math.cos(1.0) + 0.5 * math.sin(1.0)
    assert np.allclose(result, [expected_x, expected_y, 1.0])


def test_pure_rotation_and_inverse_transform_are_consistent():
    pose = integrate_body_twist([2.0, -1.0, 0.25], [0.0, 0.0, 2.0], 0.5)
    assert np.allclose(pose, [2.0, -1.0, 1.25])
    transform = pose_to_matrix(pose)
    points = np.array([[1.0, 2.0], [-3.0, 4.0]])
    assert np.allclose(
        transform_points(invert_transform(transform),
                         transform_points(transform, points)), points)
