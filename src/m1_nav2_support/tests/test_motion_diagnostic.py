"""Pure-function tests for the M1 motion diagnostic output."""

import importlib.util
from pathlib import Path

import pytest
from nav_msgs.msg import OccupancyGrid

NODE_PATH = Path(__file__).parents[1] / "m1_nav2_support" / "motion_diagnostic.py"
SPEC = importlib.util.spec_from_file_location("motion_diagnostic", NODE_PATH)
DIAGNOSTIC = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(DIAGNOSTIC)


def test_wrap_angle_stays_in_planar_range():
    assert -3.141592653589793 <= DIAGNOSTIC.wrap_angle(8.0) <= 3.141592653589793
    assert abs(DIAGNOSTIC.wrap_angle(3.2) + 3.083185307179586) < 1e-9


def test_relative_displacement_uses_initial_heading_frame():
    forward, lateral = DIAGNOSTIC.relative_displacement((1.0, 2.0, 1.5707963267948966), (1.0, 3.0, 1.5707963267948966))
    assert abs(forward - 1.0) < 1e-9
    assert abs(lateral) < 1e-9


def test_trajectory_summary_reports_lateral_and_yaw_drift():
    samples = [
        (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        (1.0, 0.2, 0.03, -0.04, 0.2, 0.03),
    ]
    result = DIAGNOSTIC.trajectory_summary(samples)
    assert result["samples"] == 2
    assert result["forward_distance_m"] == 0.2
    assert result["lateral_max_abs_m"] == 0.03
    assert result["yaw_max_abs_rad"] == 0.04


def test_command_summary_exposes_nonzero_command_stage():
    result = DIAGNOSTIC.command_summary({"smooth": [(0.1, 0.02, 0.03)], "raw": [(0.0, 0.0, 0.0)]})
    assert result["smooth"]["nonzero_samples"] == 1
    assert result["raw"]["nonzero_samples"] == 0


def test_command_veto_summary_detects_smoothed_to_raw_suppression():
    events = [
        (1.0, "cmd_vel_smoothed", (0.10, 0.00, 0.00)),
        (1.01, "m1_cmd_vel_raw", (0.00, 0.00, 0.00)),
        (1.10, "cmd_vel_smoothed", (0.10, 0.00, 0.00)),
        (1.11, "m1_cmd_vel_raw", (0.10, 0.00, 0.00)),
    ]
    result = DIAGNOSTIC.command_veto_summary(events)
    assert result["veto_periods"] == 1
    assert result["max_veto_duration_s"] == pytest.approx(0.10)


def test_goal_preflight_distinguishes_free_terminal_pose_from_lethal_cell():
    costmap = OccupancyGrid()
    costmap.info.width = 20
    costmap.info.height = 20
    costmap.info.resolution = 0.05
    costmap.info.origin.position.x = -0.5
    costmap.info.origin.position.y = -0.5
    costmap.data = [0] * (costmap.info.width * costmap.info.height)

    free = DIAGNOSTIC.goal_preflight(costmap, (0.0, 0.0, 0.0))
    assert free["final_footprint_free"] is True
    assert free["rotation_sweep_free"] is True

    costmap.data[10 * costmap.info.width + 10] = 100
    blocked = DIAGNOSTIC.goal_preflight(costmap, (0.0, 0.0, 0.0))
    assert blocked["final_footprint_free"] is False
