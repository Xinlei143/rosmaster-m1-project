"""Regression tests for measured planner timing used by Gazebo and hardware."""

import sys
from pathlib import Path

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from imperative_navigation.planner_timing import MeasuredPlannerPeriod


def test_first_period_uses_configured_control_period():
    timer = MeasuredPlannerPeriod(0.10)
    assert timer.next_period(100.0) == 0.10


def test_elapsed_period_tracks_real_callback_time():
    timer = MeasuredPlannerPeriod(0.10)
    timer.next_period(10.0)
    assert timer.next_period(10.46) == pytest.approx(0.46)


def test_elapsed_period_is_bounded_after_long_pause():
    timer = MeasuredPlannerPeriod(0.10)
    timer.next_period(10.0)
    assert timer.next_period(20.0) == 0.50
