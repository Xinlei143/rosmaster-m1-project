"""Pure tests for raw Gazebo GPU-LiDAR JSON capture analysis."""

import importlib.util
from pathlib import Path

import pytest


MODULE_PATH = (Path(__file__).parents[1] / "m1_nav2_support"
               / "gpu_lidar_probe.py")
SPEC = importlib.util.spec_from_file_location("gpu_lidar_probe", MODULE_PATH)
PROBE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PROBE)


def test_json_scan_stats_match_ros_classification_including_negative_inf():
    stats = PROBE.scan_range_stats(
        [0.4, "Infinity", "-Infinity", "NaN", 2.0], 12.0)

    assert stats["beam_count"] == 5
    assert stats["finite_count"] == 2
    assert stats["positive_inf_count"] == 1
    assert stats["negative_inf_count"] == 1
    assert stats["nan_count"] == 1
    assert stats["finite_ratio"] == pytest.approx(0.4)


def test_json_documents_can_be_extracted_from_multiline_transport_output():
    documents = list(PROBE.iter_json_documents(
        "debug line\n{\n  \"ranges\": [1.0, \"-Infinity\"]\n}\n"
        "{\"ranges\":[2.0]}\n"))

    assert documents == [
        {"ranges": [1.0, "-Infinity"]},
        {"ranges": [2.0]},
    ]


def test_summarize_frames_reports_mean_and_worst_negative_inf_ratio():
    summary = PROBE.summarize_frames([
        PROBE.scan_range_stats([1.0, "-Infinity"], 12.0),
        PROBE.scan_range_stats([1.0, 2.0], 12.0),
    ])

    assert summary["frame_count"] == 2
    assert summary["beam_count_min"] == 2
    assert summary["beam_count_max"] == 2
    assert summary["negative_inf_ratio_mean"] == pytest.approx(0.25)
    assert summary["negative_inf_ratio_max"] == pytest.approx(0.5)


def test_analyze_text_reports_raw_simulation_stamp_rate():
    summary = PROBE.analyze_text(
        '{"header":{"stamp":{"sec":1,"nsec":0}},'
        '"ranges":[1.0]}\n'
        '{"header":{"stamp":{"sec":1,"nsec":100000000}},'
        '"ranges":[1.0]}\n'
    )

    assert summary["stamp_count"] == 2
    assert summary["stamp_gap_p50"] == pytest.approx(0.1)
    assert summary["stamp_rate_hz"] == pytest.approx(10.0)


def test_transition_summary_identifies_one_permanent_all_negative_latch():
    frames = [
        PROBE.scan_range_stats([1.0, 2.0], 12.0),
        PROBE.scan_range_stats([1.0, 2.0], 12.0),
        PROBE.scan_range_stats(["-Infinity", "-Infinity"], 12.0),
        PROBE.scan_range_stats(["-Infinity", "-Infinity"], 12.0),
        PROBE.scan_range_stats(["-Infinity", "-Infinity"], 12.0),
    ]

    summary = PROBE.summarize_transitions(
        frames, [10_000_000_000, 10_100_000_000, 10_200_000_000,
                 10_300_000_000, 10_400_000_000], expected_beam_count=2)

    assert summary["all_negative_frame_count"] == 3
    assert summary["time_to_first_all_negative_s"] == pytest.approx(0.2)
    assert summary["good_to_bad_transition_count"] == 1
    assert summary["bad_to_good_recovery_count"] == 0
    assert summary["longest_continuous_bad_frames"] == 3
    assert summary["longest_continuous_bad_seconds"] == pytest.approx(0.2)
    assert summary["terminal_frame_state"] == "all_negative"


def test_transition_summary_does_not_misclassify_partial_or_wrong_beam_frames():
    frames = [
        PROBE.scan_range_stats(["-Infinity", 0.05], 12.0),
        PROBE.scan_range_stats(["-Infinity", "-Infinity"], 12.0),
        PROBE.scan_range_stats(["-Infinity"], 12.0),
        PROBE.scan_range_stats([1.0, 2.0], 12.0),
    ]

    summary = PROBE.summarize_transitions(
        frames, [0, 100_000_000, 200_000_000, 300_000_000],
        expected_beam_count=2)

    assert summary["all_negative_frame_count"] == 1
    assert summary["good_to_bad_transition_count"] == 1
    assert summary["bad_to_good_recovery_count"] == 1
    assert summary["longest_continuous_bad_frames"] == 1
    assert summary["longest_continuous_bad_seconds"] == pytest.approx(0.0)
    assert summary["terminal_frame_state"] == "other"


def test_all_negative_frame_requires_the_expected_beam_count():
    all_negative = PROBE.scan_range_stats(["-Infinity", "-Infinity"], 12.0)
    partial = PROBE.scan_range_stats(["-Infinity", 0.05], 12.0)

    assert PROBE.is_all_negative_frame(all_negative, expected_beam_count=2)
    assert not PROBE.is_all_negative_frame(partial, expected_beam_count=2)
    assert not PROBE.is_all_negative_frame(all_negative, expected_beam_count=667)


def test_frame_stats_from_document_preserves_the_scan_stamp():
    frame, stamp = PROBE.frame_stats_from_document({
        "header": {"stamp": {"sec": 94, "nsec": 500_000_000}},
        "range_max": 12.0,
        "ranges": [1.0, "-Infinity"],
    })

    assert frame["beam_count"] == 2
    assert frame["negative_inf_count"] == 1
    assert stamp == 94_500_000_000


def test_frame_stats_from_non_scan_document_is_none():
    assert PROBE.frame_stats_from_document({"status": "not a scan"}) is None
