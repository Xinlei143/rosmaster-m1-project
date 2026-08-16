import torch

from imperative_navigation.holonomic_local_planner import choose_velocity, goal_is_dynamically_blocked


def test_rollout_curves_away_from_a_front_box_instead_of_using_waypoints():
    position, velocity, goal = torch.zeros(2), torch.zeros(2), torch.tensor([2.0, 0.0])
    box_y = torch.linspace(-0.12, 0.12, 13)
    box_face = torch.stack((torch.full_like(box_y, 0.45), box_y), dim=1)

    command, _, clearance = choose_velocity(
        position, velocity, goal, box_face, [], max_speed=0.08,
        max_acceleration=0.10, robot_clearance=0.12, horizon=40)

    assert abs(float(command[1])) > 1e-4
    assert clearance >= 0.12


def test_only_confirmed_moving_track_can_block_goal_arrival_region():
    goal = torch.tensor([1.0, 0.0])
    static_track = {"position": torch.tensor([1.02, 0.0]), "velocity": torch.zeros(2)}
    moving_track = {"position": torch.tensor([1.02, 0.0]), "velocity": torch.tensor([0.3, 0.0])}
    assert not goal_is_dynamically_blocked(goal, [static_track], 0.12, 0.20)
    assert goal_is_dynamically_blocked(goal, [moving_track], 0.12, 0.20)


def test_goal_blocking_uses_each_tracks_measured_radius():
    goal = torch.tensor([0.0, 0.0])
    small_track = {
        "position": torch.tensor([0.35, 0.0]),
        "velocity": torch.tensor([0.3, 0.0]),
        "radius": 0.10,
    }
    large_track = dict(small_track, radius=0.30)
    assert not goal_is_dynamically_blocked(goal, [small_track], 0.12, 0.20)
    assert goal_is_dynamically_blocked(goal, [large_track], 0.12, 0.20)
