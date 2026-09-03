"""Tests for cross-case GPU-LiDAR matrix aggregation."""

import importlib.util
from pathlib import Path

import pytest


SCRIPT = (Path(__file__).parents[3] / "scripts"
          / "summarize_gpu_lidar_matrix.py")
SPEC = importlib.util.spec_from_file_location("gpu_lidar_matrix_summary", SCRIPT)
SUMMARY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SUMMARY)


def test_aggregate_rows_keeps_raw_and_ros_boundaries_separate():
    result = SUMMARY.aggregate_rows([
        {
            "case": "gui_off",
            "repeat": "repeat_01",
            "raw": {
                "frame_count": 10,
                "stamp_rate_hz": 12.0,
                "beam_count_min": 667,
                "beam_count_max": 667,
                "negative_inf_ratio_mean": 0.1,
                "negative_inf_ratio_max": 0.2,
            },
            "ros": {
                "scan_count": 11,
                "scan_rate_hz": 11.5,
                "negative_inf_ratio_mean": 0.0,
            },
            "status_ok": True,
        },
        {
            "case": "gui_off",
            "repeat": "repeat_02",
            "raw": {
                "frame_count": 12,
                "stamp_rate_hz": 12.2,
                "beam_count_min": 667,
                "beam_count_max": 667,
                "negative_inf_ratio_mean": 0.0,
                "negative_inf_ratio_max": 0.0,
            },
            "ros": {
                "scan_count": 13,
                "scan_rate_hz": 11.8,
                "negative_inf_ratio_mean": 0.05,
            },
            "status_ok": True,
        },
    ])

    case = result["cases"]["gui_off"]
    assert result["repeat_count"] == 2
    assert case["raw_negative_inf_ratio_max"] == pytest.approx(0.2)
    assert case["ros_negative_inf_ratio_max"] == pytest.approx(0.05)
    assert case["raw_frame_count_total"] == 22
    assert case["ros_scan_count_total"] == 24
    assert case["beam_count_min"] == 667
    assert case["beam_count_max"] == 667
    assert result["all_status_ok"] is True


def test_markdown_report_exposes_first_boundary_columns():
    markdown = SUMMARY.render_markdown({
        "repeat_count": 1,
        "all_status_ok": True,
        "cases": {
            "gui_off": {
                "repeat_count": 1,
                "raw_negative_inf_ratio_max": 0.0,
                "ros_negative_inf_ratio_max": 0.0,
                "raw_stamp_rate_hz_mean": 12.0,
                "ros_scan_rate_hz_mean": 12.0,
                "beam_count_min": 667,
                "beam_count_max": 667,
            },
        },
    })

    assert "raw -Inf max" in markdown
    assert "ROS -Inf max" in markdown
    assert "gui_off" in markdown
