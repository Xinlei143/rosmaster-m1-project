"""Pure-function tests for costmap freshness diagnostics."""

import importlib.util
from pathlib import Path

import pytest


NODE_PATH = (Path(__file__).parents[1] / "m1_nav2_support"
             / "costmap_freshness_diagnostic.py")
SPEC = importlib.util.spec_from_file_location(
    "costmap_freshness_diagnostic", NODE_PATH)
DIAGNOSTIC = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(DIAGNOSTIC)


def test_percentile_and_interval_summary_include_p50_p95_and_max():
    summary = DIAGNOSTIC.summarize_intervals([0.1, 0.2, 0.3, 0.4, 0.5])

    assert summary["p50"] == pytest.approx(0.3)
    assert summary["p95"] == pytest.approx(0.48)
    assert summary["max"] == pytest.approx(0.5)
    assert DIAGNOSTIC.summarize_intervals([]) == {
        "p50": None,
        "p95": None,
        "max": None,
    }


def test_scan_range_stats_separate_finite_positive_inf_and_nan():
    stats = DIAGNOSTIC.scan_range_stats(
        [0.4, float("inf"), float("nan"), 2.0], 12.0)

    assert stats == {
        "finite_count": 2,
        "positive_inf_count": 1,
        "nan_count": 1,
        "finite_ratio": pytest.approx(0.5),
        "positive_inf_ratio": pytest.approx(0.25),
        "nan_ratio": pytest.approx(0.25),
        "min_finite_range": pytest.approx(0.4),
        "range_max": pytest.approx(12.0),
    }


def test_header_age_uses_exact_stamp_and_preserves_negative_clock_skew():
    assert DIAGNOSTIC.header_age_seconds(
        2_000_000_000, 1_750_000_000) == pytest.approx(0.25)
    assert DIAGNOSTIC.header_age_seconds(
        1_000_000_000, 1_250_000_000) == pytest.approx(-0.25)


def test_clock_jump_stats_expose_backward_jumps_and_forward_gaps():
    stats = DIAGNOSTIC.clock_jump_stats([
        1_000_000_000, 2_000_000_000, 500_000_000, 1_500_000_000,
    ])

    assert stats["sample_count"] == 4
    assert stats["backward_jump_count"] == 1
    assert stats["max_backward_jump_seconds"] == pytest.approx(1.5)
    assert stats["forward_gap_p95_seconds"] == pytest.approx(1.0)
    assert DIAGNOSTIC.clock_jump_stats([])["backward_jump_count"] == 0


def test_dynamic_obstacle_match_uses_surface_search_radius_not_center_cell():
    lethal_cells = [(1.30, 2.0), (4.0, 4.0)]

    assert DIAGNOSTIC.match_dynamic_obstacle((1.0, 2.0), lethal_cells, 0.45)
    assert not DIAGNOSTIC.match_dynamic_obstacle(
        (1.0, 2.0), lethal_cells, 0.20)


def test_dynamic_obstacle_is_eligible_only_when_search_disk_overlaps_costmap():
    bounds = (-1.0, -1.0, 2.0, 2.0)

    assert DIAGNOSTIC.point_overlaps_costmap(
        (1.8, 0.0), bounds, 0.45)
    assert not DIAGNOSTIC.point_overlaps_costmap((2.01, 0.0), bounds, 0.0)
    assert not DIAGNOSTIC.point_overlaps_costmap(
        (3.0, 0.0), bounds, 0.45)


def test_ghost_retention_waits_for_relocation_and_grace_then_reports_ratio():
    old_cells = [(0.0, 0.0), (0.05, 0.0), (0.10, 0.0)]

    assert DIAGNOSTIC.ghost_retention(
        old_cells, old_cells[:2], (0.0, 0.0), (0.2, 0.0), 0.2, 0.4
    ) is None
    assert DIAGNOSTIC.ghost_retention(
        old_cells, old_cells[:2], (0.0, 0.0), (0.6, 0.0), 0.5, 0.4
    ) == pytest.approx(2.0 / 3.0)
    assert DIAGNOSTIC.ghost_retention(
        old_cells, old_cells[:2], (0.0, 0.0), (0.45, 0.0), 0.5, 0.4,
        move_distance=0.4
    ) == pytest.approx(2.0 / 3.0)


def test_ghost_retention_skips_old_position_reoccupied_by_current_obstacle():
    old_cells = [(0.0, 0.0), (0.05, 0.0)]

    assert DIAGNOSTIC.ghost_retention(
        old_cells,
        old_cells,
        (0.0, 0.0),
        (0.6, 0.0),
        0.5,
        0.4,
        current_obstacle_center=(0.0, 0.0),
    ) is None
