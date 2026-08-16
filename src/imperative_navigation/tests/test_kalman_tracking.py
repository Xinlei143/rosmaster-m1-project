"""Tests for constant-velocity Kalman multi-object tracking."""

import importlib.util
from pathlib import Path

import pytest
import torch


ALGORITHM_PATH = Path(__file__).parents[1] / "algorithm" / "Imperative_learning_2D_moving.py"
SPEC = importlib.util.spec_from_file_location("imperative_planner_kalman", ALGORITHM_PATH)
PLANNER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PLANNER)


def detection(x, y=0.0, radius=0.25):
    return {
        "position": torch.tensor([x, y], dtype=torch.float32),
        "radius": radius,
        "point_count": 8,
        "nearest_range": 1.0,
    }


def test_new_track_contains_kalman_state_covariance_and_radius():
    tracks, next_id = PLANNER.update_tracks([], [detection(1.0, 2.0, 0.3)], 7, dt=0.1)
    assert next_id == 8
    assert tracks[0]["id"] == 7
    assert torch.equal(tracks[0]["state"], torch.tensor([1.0, 2.0, 0.0, 0.0]))
    assert tracks[0]["covariance"].shape == (4, 4)
    assert tracks[0]["radius"] == pytest.approx(0.3)


def test_constant_velocity_measurements_converge_to_velocity():
    tracks, next_id = [], 0
    dt, velocity = 0.1, 0.4
    for step in range(30):
        tracks, next_id = PLANNER.update_tracks(
            tracks, [detection(step * dt * velocity)], next_id, dt=dt)
    assert len(tracks) == 1
    assert tracks[0]["velocity"][0] == pytest.approx(velocity, abs=0.03)
    assert abs(float(tracks[0]["velocity"][1])) < 1e-5
    assert torch.allclose(tracks[0]["covariance"], tracks[0]["covariance"].T, atol=1e-6)


def test_missed_detection_uses_kalman_prediction():
    tracks, next_id = PLANNER.update_tracks([], [detection(0.0)], 0, dt=0.1)
    tracks[0]["state"][2] = 0.5
    tracks[0]["velocity"][0] = 0.5
    tracks, _ = PLANNER.update_tracks(tracks, [], next_id, dt=0.2)
    assert tracks[0]["position"][0] == pytest.approx(0.1)
    assert tracks[0]["missed"] == 1


def test_global_nearest_neighbour_is_one_to_one_and_order_independent():
    tracks, next_id = PLANNER.update_tracks(
        [], [detection(0.0), detection(0.5)], 0, dt=0.1)
    tracks, _ = PLANNER.update_tracks(
        tracks, [detection(0.4), detection(0.75)], next_id, dt=0.1)
    by_id = {track["id"]: track for track in tracks}
    assert set(by_id) == {0, 1}
    assert by_id[0]["velocity"][0] > 0.0
    assert by_id[1]["velocity"][0] < 0.0
    assert by_id[0]["missed"] == by_id[1]["missed"] == 0


def test_radius_is_smoothed_after_association():
    tracks, next_id = PLANNER.update_tracks([], [detection(0.0, radius=0.2)], 0, dt=0.1)
    tracks, _ = PLANNER.update_tracks(
        tracks, [detection(0.0, radius=0.4)], next_id, dt=0.1)
    expected = (PLANNER.TRACK_RADIUS_SMOOTHING * 0.4 +
                (1.0 - PLANNER.TRACK_RADIUS_SMOOTHING) * 0.2)
    assert tracks[0]["radius"] == pytest.approx(expected)


def test_track_is_removed_after_maximum_missed_scans():
    tracks, next_id = PLANNER.update_tracks([], [detection(0.0)], 0, dt=0.1)
    for _ in range(PLANNER.TRACK_MAX_MISSED):
        tracks, next_id = PLANNER.update_tracks(tracks, [], next_id, dt=0.1)
        assert len(tracks) == 1
    tracks, _ = PLANNER.update_tracks(tracks, [], next_id, dt=0.1)
    assert tracks == []
