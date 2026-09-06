import math
import sys
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "m1_scope_bridge"))
sys.path.insert(0, str(ROOT / "m1_scope_predictor"))

from m1_scope_bridge.scope_ogm import build_scope_input_window  # noqa: E402
from m1_scope_predictor.latest_mailbox import LatestMailbox  # noqa: E402
from m1_scope_predictor.occupancy_grid import (  # noqa: E402
    grid_origin_pose,
    probability_data,
    uncertainty_data,
)


def test_shared_window_builder_uses_lateral_twist_and_future_frame():
    wall_world = np.array([[4.0, -0.2], [4.0, 0.0], [4.0, 0.2]])
    past_poses = np.array([[0.0, y, 0.0] for y in np.linspace(0.0, 0.9, 10)])
    scans = [wall_world - pose[:2] for pose in past_poses]
    input_ogm, future_pose, _ = build_scope_input_window(
        scans,
        past_poses,
        current_pose=past_poses[-1],
        current_twist=np.array([0.0, 1.0, 0.0]),
        base_to_laser=np.eye(3),
        horizon_seconds=0.5,
    )
    assert input_ogm.shape == (10, 1, 64, 64)
    assert np.allclose(future_pose, [0.0, 1.4, 0.0])
    assert all(input_ogm[index, 0, 40, 18] for index in range(10))


def test_latest_mailbox_overwrites_unstarted_work():
    mailbox = LatestMailbox()
    assert mailbox.put("job-1") is False
    assert mailbox.put("job-2") is True
    assert mailbox.get() == "job-2"
    assert mailbox.dropped == 1


def test_latest_mailbox_take_is_nonblocking():
    mailbox = LatestMailbox()
    assert mailbox.take() is None
    mailbox.put("result")
    assert mailbox.take() == "result"


def test_grid_origin_applies_scope_lower_left_in_rotated_frame():
    x, y, yaw = grid_origin_pose([2.0, 3.0, math.pi / 2.0])
    assert np.allclose([x, y, yaw], [5.2, 3.0, math.pi / 2.0])


def test_probability_grid_transposes_xy_storage_for_ros():
    grid = np.zeros((1, 64, 64), dtype=np.float32)
    grid[0, 2, 3] = 0.51
    data = probability_data(grid)
    assert data[3 * 64 + 2] == 51
    assert sum(data) == 51


def test_uncertainty_uses_fixed_half_probability_scale():
    grid = np.zeros((1, 64, 64), dtype=np.float32)
    grid[0, 1, 2] = 0.25
    grid[0, 3, 4] = 0.75
    data = uncertainty_data(grid)
    assert data[2 * 64 + 1] == 50
    assert data[4 * 64 + 3] == 100
