"""Verify real-node geometry parameters update the loaded planner itself."""

import importlib.util
import sys
from pathlib import Path

import torch


PACKAGE_ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from imperative_navigation.m1_controller_node import configure_planner_robot_clearance


ALGORITHM_PATH = PACKAGE_ROOT / "algorithm" / "Imperative_learning_2D_moving.py"


def load_algorithm():
    spec = importlib.util.spec_from_file_location("planner_under_test", ALGORITHM_PATH)
    algorithm = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(algorithm)
    return algorithm


def test_default_real_robot_clearance_matches_existing_behavior():
    algorithm = load_algorithm()
    configure_planner_robot_clearance(algorithm, robot_radius=0.18, safety_margin=0.18)

    assert algorithm.ROBOT_RADIUS == 0.18
    assert algorithm.SAFETY_MARGIN == 0.18
    assert algorithm.LIDAR_SAFE_DISTANCE == 0.36
    # collision_penalty is called by select_acceleration, proving the actual
    # running planner receives the recomputed derived clearance.
    assert float(algorithm.collision_penalty(torch.tensor(0.35))) > 0.0


def test_explicit_real_robot_clearance_recomputes_running_planner_radius():
    algorithm = load_algorithm()
    configure_planner_robot_clearance(algorithm, robot_radius=0.18, safety_margin=0.10)

    assert algorithm.ROBOT_RADIUS == 0.18
    assert algorithm.SAFETY_MARGIN == 0.10
    assert algorithm.LIDAR_SAFE_DISTANCE == 0.28
    assert float(algorithm.collision_penalty(torch.tensor(0.30))) == 0.0


def test_directional_profile_keeps_more_clearance_ahead_than_at_the_sides():
    algorithm = load_algorithm()
    configure_planner_robot_clearance(
        algorithm, robot_radius=0.18, safety_margin=0.02,
        front_safety_margin=0.12, lateral_planning_margin=0.03,
        front_planning_margin=0.10, forward_half_angle_degrees=45.0)

    distance = torch.tensor([0.30, 0.30])
    forward_cosines = torch.tensor([1.0, 0.0])
    penalties = algorithm.directional_collision_penalty(distance, forward_cosines)

    # Front clearance = .18 + .12 + .10 = .40 m; lateral = .18 + .02 + .03 = .23 m.
    assert float(penalties[0]) > 0.0
    assert float(penalties[1]) == 0.0
