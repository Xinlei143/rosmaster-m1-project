"""Tests for adapter-side track confirmation and smoothing."""

import sys
from pathlib import Path

import torch


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from imperative_navigation.track_stability import ConfirmedTrackFilter


def track(track_id=1, age=3, missed=0, position=(0.0, 0.0), velocity=(0.0, 0.0)):
    return {
        "id": track_id,
        "age": age,
        "missed": missed,
        "position": torch.tensor(position, dtype=torch.float32),
        "velocity": torch.tensor(velocity, dtype=torch.float32),
    }


def test_transient_or_missed_tracks_are_not_predicted():
    filter_ = ConfirmedTrackFilter(confirmation_age=3)
    assert filter_.update([track(age=2)]) == []
    assert filter_.update([track(age=3, missed=1)]) == []


def test_confirmed_track_position_is_smoothed_and_static_velocity_zeroed():
    filter_ = ConfirmedTrackFilter(position_alpha=0.25, static_speed_threshold=0.25)
    first = filter_.update([track(position=(0.0, 0.0), velocity=(0.10, 0.0))])[0]
    second = filter_.update([track(position=(1.0, 0.0), velocity=(0.10, 0.0))])[0]
    assert torch.allclose(first["position"], torch.tensor([0.0, 0.0]))
    assert torch.allclose(second["position"], torch.tensor([0.25, 0.0]))
    assert torch.equal(second["velocity"], torch.zeros(2))


def test_fast_confirmed_track_keeps_motion_prediction():
    filter_ = ConfirmedTrackFilter(static_speed_threshold=0.25)
    output = filter_.update([track(velocity=(0.40, 0.0))])
    assert len(output) == 1
    assert torch.allclose(output[0]["velocity"], torch.tensor([0.40, 0.0]))
