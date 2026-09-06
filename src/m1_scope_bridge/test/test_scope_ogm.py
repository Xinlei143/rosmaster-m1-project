import math
import sys
from pathlib import Path

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from m1_scope_bridge.scope_ogm import (  # noqa: E402
    find_nearest_indices, points_to_grid, resolve_static_transform,
    validate_no_slip_roi_semantics)


def test_points_to_grid_has_scope_shape_and_bounds():
    points = np.array([
        [0.0, -3.2], [0.09, -3.11], [6.39, 3.19],
        [6.4, 0.0], [-0.01, 0.0], [1.0, 3.2],
    ])
    grid = points_to_grid(points)
    assert grid.shape == (1, 64, 64)
    assert set(np.unique(grid)) <= {0, 1}
    assert np.isfinite(grid).all()
    assert grid[0, 0, 0] == 1
    assert grid[0, 63, 63] == 1
    assert int(grid.sum()) == 2


def test_find_nearest_indices_prefers_earlier_timestamp_on_tie():
    indices, errors = find_nearest_indices(
        np.array([0, 100, 200]), np.array([50, 151]), tolerance_ns=51)
    assert indices.tolist() == [0, 2]
    assert errors.tolist() == [50, 49]


def test_find_nearest_indices_rejects_out_of_tolerance():
    indices, _ = find_nearest_indices(
        np.array([0, 100]), np.array([60]), tolerance_ns=39)
    assert indices.tolist() == [-1]


def test_resolve_static_transform_follows_multihop_chain():
    transform = resolve_static_transform(
        np.array(["base_footprint", "base_link", "unused"]),
        np.array(["base_link", "laser_scan_link", "elsewhere"]),
        np.array([[1.0, 0.0, 0.0], [0.0, 2.0, 0.0], [9.0, 9.0, 0.0]]),
        np.array([[0.0, 0.0, 0.0, 1.0],
                  [0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)],
                  [0.0, 0.0, 0.0, 1.0]]),
        "base_footprint", "laser_scan_link")
    assert np.allclose(transform[:2, 2], [1.0, 2.0])
    assert np.allclose(transform[:2, :2], [[0.0, -1.0], [1.0, 0.0]], atol=1e-7)


def test_resolve_static_transform_fails_when_chain_is_missing():
    try:
        resolve_static_transform(
            np.array(["a"]), np.array(["b"]), np.zeros((1, 3)),
            np.array([[0.0, 0.0, 0.0, 1.0]]), "base", "laser")
    except ValueError as error:
        assert "no static transform chain" in str(error)
    else:
        raise AssertionError("missing TF chain must not imply zero offset")


def test_dynamic_roi_rejects_slipped_odom_frame_semantics():
    odom = np.array([[0.0, 0.0, 0.0], [0.10, 0.0, 0.0]])
    ground_truth = np.zeros((2, 3))
    try:
        validate_no_slip_roi_semantics(odom, ground_truth)
    except ValueError as error:
        assert "dynamic-obstacle ROI requires no-slip" in str(error)
    else:
        raise AssertionError("slipped odometry must disable mislabeled ROI coordinates")
