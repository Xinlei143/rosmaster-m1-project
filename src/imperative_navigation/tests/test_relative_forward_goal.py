"""Unit tests for the M1 relative forward-goal geometry."""

import math

import torch

from imperative_navigation.m1_controller_node import forward_goal_from_pose


def test_forward_goal_at_zero_yaw_moves_along_world_x():
    goal = forward_goal_from_pose(torch.tensor([1.5, -0.5]), 0.0, 2.0)

    assert torch.allclose(goal, torch.tensor([3.5, -0.5]))


def test_forward_goal_at_right_angle_moves_along_world_y():
    goal = forward_goal_from_pose(torch.tensor([-1.0, 2.0]), math.pi / 2.0, 2.0)

    assert torch.allclose(goal, torch.tensor([-1.0, 4.0]), atol=1e-6)
