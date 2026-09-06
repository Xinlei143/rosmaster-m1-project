import sys
from pathlib import Path

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from m1_scope_bridge.scope_ogm import compensate_points  # noqa: E402


def test_static_wall_aligns_after_ego_motion_compensation():
    wall_world = np.array([[4.0, -0.2], [4.0, 0.0], [4.0, 0.2]])
    past_laser_poses = [np.array([x, 0.0, 0.0]) for x in (0.0, 0.5, 1.0)]
    future_laser_pose = np.array([1.5, 0.0, 0.0])
    past_observations = [wall_world - np.array([pose[0], pose[1]])
                         for pose in past_laser_poses]

    compensated = [
        compensate_points(points, pose, future_laser_pose)
        for points, pose in zip(past_observations, past_laser_poses)
    ]

    assert all(np.allclose(points, [[2.5, -0.2], [2.5, 0.0], [2.5, 0.2]])
               for points in compensated)
