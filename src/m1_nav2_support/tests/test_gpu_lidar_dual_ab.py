"""Unit tests for the controlled single-360 versus dual-180 A/B runner."""

from pathlib import Path
import sys


MODULE_PATH = Path(__file__).parents[1] / "m1_nav2_support" / "gpu_lidar_dual_ab.py"
sys.path.insert(0, str(Path(__file__).parents[1]))
from m1_nav2_support import gpu_lidar_dual_ab as AB  # noqa: E402


def test_campaign_order_is_interleaved_without_early_stop():
    cases = AB.campaign_cases(repeats=3, duration=300.0)

    assert [(case.label, case.repeat) for case in cases] == [
        ("A_ogre2_single360", 1), ("B_ogre1_dual180", 1),
        ("A_ogre2_single360", 2), ("B_ogre1_dual180", 2),
        ("A_ogre2_single360", 3), ("B_ogre1_dual180", 3),
    ]
    assert all(case.duration == 300.0 for case in cases)


def test_launch_commands_change_only_lidar_backend_and_layout():
    a_case, b_case = AB.campaign_cases(repeats=1, duration=300.0)
    a = AB.build_launch_command(a_case)
    b = AB.build_launch_command(b_case)

    common = {
        "gui:=true", "rviz:=true", "software_lidar:=false",
        "dynamic_obstacles:=true", "slip_enabled:=false",
    }
    assert common <= set(a)
    assert common <= set(b)
    assert "render_engine:=ogre2" in a
    assert "dual_gpu_lidar:=false" in a
    assert "render_engine:=ogre" in b
    assert "dual_gpu_lidar:=true" in b


def test_capture_topics_cover_raw_ros_and_merged_paths():
    a_case, b_case = AB.campaign_cases(repeats=1, duration=300.0)

    assert AB.capture_topics(a_case) == {
        "gazebo": ["/scan"], "ros": ["/scan"], "merged": None,
    }
    assert AB.capture_topics(b_case) == {
        "gazebo": ["/scan_front", "/scan_rear"],
        "ros": ["/scan_front", "/scan_rear"],
        "merged": "/scan",
    }


def test_treatment_requires_clean_raw_scans_and_single_merged_publisher():
    raw = {
        name: {
            "frame_count": 3600, "beam_count_min": 334,
            "beam_count_max": 334, "all_negative_frame_count": 0,
            "nan_frame_count": 0, "longest_continuous_bad_frames": 0,
            "stamp_gap_p50": 1.0 / 12.0, "stamp_gap_p95": 0.09,
        }
        for name in ("gazebo_front", "gazebo_rear", "ros_front", "ros_rear")
    }
    matches = {
        "front": {"common_stamp_count": 3500, "mismatch_count": 0},
        "rear": {"common_stamp_count": 3500, "mismatch_count": 0},
    }
    merged = {
        "frame_count": 3590, "beam_count_min": 667, "beam_count_max": 667,
        "frame_ids": ["laser_scan_link"], "stamp_gap_p50": 1.0 / 12.0,
        "stamp_gap_p95": 0.09, "publisher_count": 1,
    }

    assessment = AB.assess_treatment(raw, matches, merged)

    assert assessment == {"passed": True, "failures": []}


def test_positive_control_requires_at_least_two_latched_repeats():
    assert AB.assess_positive_control([True, True, False])["passed"] is True
    result = AB.assess_positive_control([False, False, False])
    assert result["passed"] is False
    assert "fewer than 2/3" in result["failures"][0]


def test_navigation_assessment_requires_motion_goals_costmap_and_raw_stability():
    treatment = {"passed": True, "failures": []}
    waypoint = {
        "succeeded_goal_count": 2,
        "distance_m": 3.0,
        "nonzero_command_counts": {
            "/cmd_vel_nav": 10,
            "/cmd_vel_smoothed": 10,
            "/m1/cmd_vel_raw": 10,
        },
    }
    costmap = {
        "costmap_publish_rate_hz": 2.0,
        "dynamic_obstacle_samples": 100,
        "dynamic_obstacle_detected_ratio": 0.98,
        "dynamic_obstacle_detection_miss_duration_max": 0.2,
    }

    result = AB.assess_navigation(treatment, waypoint, costmap)

    assert result == {"passed": True, "failures": []}


def test_navigation_cases_are_ogre1_dual_only():
    cases = AB.navigation_cases(repeats=3, duration=300.0)

    assert len(cases) == 3
    assert all(case.dual_gpu_lidar for case in cases)
    assert all(case.render_engine == "ogre" for case in cases)
    assert [case.repeat for case in cases] == [1, 2, 3]
