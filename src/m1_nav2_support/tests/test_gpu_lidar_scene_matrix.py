"""Contracts for the raw-only GPU-LiDAR scene-isolation matrix."""

import importlib.util
import json
from pathlib import Path


PACKAGE = Path(__file__).parents[1]
MODULE = PACKAGE / "m1_nav2_support" / "gpu_lidar_scene_matrix.py"
WORLDS = PACKAGE / "worlds"
SETUP = PACKAGE / "setup.py"


def _load_matrix():
    spec = importlib.util.spec_from_file_location("gpu_lidar_scene_matrix", MODULE)
    matrix = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(matrix)
    return matrix


def test_raw_only_scene_matrix_entrypoint_exists():
    assert MODULE.is_file()


def test_static_m1_world_for_test_a_exists():
    assert (WORLDS / "m1_static_raw.sdf").is_file()


def test_minimal_world_without_preloaded_sensor_exists():
    assert (WORLDS / "gpu_lidar_minimal_base.sdf").is_file()


def test_static_m1_world_removes_dynamic_models_but_keeps_static_geometry():
    text = (WORLDS / "m1_static_raw.sdf").read_text(encoding="utf-8")

    for retained in ("ground", "wall_north", "wall_south", "wall_east",
                     "wall_west", "static_obstacle_1", "static_obstacle_2",
                     "goal_marker"):
        assert f'name="{retained}"' in text
    for removed in ("moving_obstacle_1", "moving_obstacle_2",
                    "moving_obstacle_3", "VelocityControl"):
        assert removed not in text


def test_scene_matrix_defines_test_a_and_all_r0_to_r5_cases():
    matrix = _load_matrix()

    cases = {case.label: case for case in matrix.scene_matrix_cases()}

    assert set(cases) == {
        "A_static_raw_m1",
        "R0_minimal_world_load_primitive",
        "R1_minimal_dynamic_primitive",
        "R2_m1_dynamic_primitive",
        "R3_minimal_dynamic_m1",
        "R4_m1_dynamic_m1",
        "R5_m1_world_load_m1",
    }
    assert cases["A_static_raw_m1"].world_file == "m1_static_raw.sdf"
    assert cases["A_static_raw_m1"].creation == "dynamic_urdf"
    assert cases["R0_minimal_world_load_primitive"].topic == "/minimal_scan"
    assert cases["R5_m1_world_load_m1"].creation == "world_loaded_sdf"


def test_dynamic_spawn_command_contains_no_bridge_or_navigation_processes():
    matrix = _load_matrix()
    case = next(
        item for item in matrix.scene_matrix_cases()
        if item.label == "R4_m1_dynamic_m1")

    command = matrix.dynamic_spawn_command(
        case, "/tmp/m1.urdf", world_name="m1")

    assert command[:4] == ["ros2", "run", "ros_gz_sim", "create"]
    assert "-string" in command
    assert "-world" in command and command[command.index("-world") + 1] == "m1"
    assert not set(command) & {"ros_gz_bridge", "nav2_bringup", "rviz2"}


def test_primitive_model_preserves_the_m1_gpu_lidar_contract():
    matrix = _load_matrix()

    model = matrix.primitive_lidar_model_sdf()

    assert '<sensor name="minimal_gpu_lidar" type="gpu_lidar">' in model
    assert "<topic>/scan</topic>" in model
    assert "<update_rate>12</update_rate>" in model
    assert "<samples>667</samples>" in model
    assert "<min>-3.14159265359</min>" not in model
    assert "<min_angle>-3.14159265359</min_angle>" in model
    assert "<max_angle>3.14159265359</max_angle>" in model
    assert "<min>0.05</min>" in model
    assert "<max>12.0</max>" in model


def test_bad_streak_tracker_stops_only_after_consecutive_full_negative_frames():
    matrix = _load_matrix()
    tracker = matrix.BadStreakTracker(expected_beam_count=2, threshold=3)
    good = {"beam_count": 2, "negative_inf_count": 0}
    bad = {"beam_count": 2, "negative_inf_count": 2}
    partial = {"beam_count": 2, "negative_inf_count": 1}

    assert not tracker.observe(good)
    assert not tracker.observe(bad)
    assert not tracker.observe(bad)
    assert not tracker.observe(partial)
    assert not tracker.observe(bad)
    assert not tracker.observe(bad)
    assert tracker.observe(bad)
    assert tracker.current_streak == 3


def test_world_load_combiner_embeds_preconverted_m1_with_spawn_pose(tmp_path):
    matrix = _load_matrix()
    base = tmp_path / "base.sdf"
    converted = tmp_path / "m1.sdf"
    output = tmp_path / "combined.sdf"
    base.write_text(
        '<sdf version="1.9"><world name="m1"><model name="ground"/>'
        '</world></sdf>', encoding="utf-8")
    converted.write_text(
        '<sdf version="1.9"><model name="yahboomcar_M1"><link name="base"/>'
        '</model></sdf>', encoding="utf-8")

    matrix.write_world_loaded_model(base, converted, output)

    text = output.read_text(encoding="utf-8")
    assert '<model name="ground"' in text
    assert '<model name="m1">' in text
    assert '<pose>-2.5 -1.5 0.01 0 0 0</pose>' in text


def test_world_load_combiner_accepts_ign_sdf_output_with_an_unbound_ignition_prefix(
        tmp_path):
    matrix = _load_matrix()
    base = tmp_path / "base.sdf"
    converted = tmp_path / "m1.sdf"
    output = tmp_path / "combined.sdf"
    base.write_text(
        '<sdf version="1.9"><world name="m1"/></sdf>', encoding="utf-8")
    converted.write_text(
        "<sdf version='1.9'><model name='yahboomcar_M1'><link name='base'>"
        "<collision name='collision'><surface><friction><ode>"
        "<fdir1 ignition:expressed_in='base'>1 0 0</fdir1>"
        "</ode></friction></surface></collision></link></model></sdf>",
        encoding="utf-8")

    matrix.write_world_loaded_model(base, converted, output)

    text = output.read_text(encoding="utf-8")
    assert 'xmlns:ignition="http://ignitionrobotics.org/schema"' in text
    assert 'ignition:expressed_in="base"' in text


def test_json_stream_waits_for_a_complete_document_and_keeps_adjacent_frames():
    matrix = _load_matrix()
    stream = matrix.JsonDocumentStream()

    assert stream.feed('noise\n{"ranges": [1.0, ') == []
    assert stream.feed('"-Infinity"]}\n{"ranges": [2.0]}\n') == [
        {"ranges": [1.0, "-Infinity"]},
        {"ranges": [2.0]},
    ]


def test_gazebo_command_is_server_only_and_uses_ogre2(tmp_path):
    matrix = _load_matrix()

    command = matrix.gazebo_command(tmp_path / "world.sdf")

    assert command[:3] == ["ign", "gazebo", "-s"]
    assert "-r" in command
    assert "--render-engine" in command
    assert command[command.index("--render-engine") + 1] == "ogre2"
    assert not set(command) & {"ros_gz_bridge", "nav2_bringup", "rviz2"}


def test_capture_validation_requires_complete_expected_raw_sensor_data():
    matrix = _load_matrix()
    good = {
        "frame_count": 2160,
        "beam_count_min": 667,
        "beam_count_max": 667,
        "malformed_scan_count": 0,
        "stamp_rate_hz": 12.0,
        "stamp_count": 2160,
        "stamp_gap_p50": 0.083,
        "stamp_gap_p95": 0.083,
        "stamp_gap_max": 0.1,
        "time_to_first_all_negative_s": None,
    }

    valid, reasons = matrix.validate_capture_summary(
        good, duration_seconds=180.0, stop_reason="duration_reached")

    assert valid
    assert reasons == []

    bad = dict(good, beam_count_max=666, malformed_scan_count=1)
    valid, reasons = matrix.validate_capture_summary(
        bad, duration_seconds=180.0, stop_reason="duration_reached")

    assert not valid
    assert "unexpected_beam_count" in reasons
    assert "malformed_frames" in reasons


def test_bad_streak_early_stop_requires_a_recorded_all_negative_transition():
    matrix = _load_matrix()
    summary = {
        "frame_count": 1200,
        "beam_count_min": 667,
        "beam_count_max": 667,
        "malformed_scan_count": 0,
        "stamp_rate_hz": 12.0,
        "stamp_count": 1200,
        "stamp_gap_p50": 0.083,
        "stamp_gap_p95": 0.083,
        "stamp_gap_max": 0.1,
        "time_to_first_all_negative_s": 94.0,
        "longest_continuous_bad_frames": 60,
    }

    valid, reasons = matrix.validate_capture_summary(
        summary, duration_seconds=180.0, stop_reason="bad_streak_60")

    assert valid
    assert reasons == []


def test_capture_validation_accepts_a_startup_burst_when_steady_gap_is_12hz():
    matrix = _load_matrix()
    bursty = {
        "frame_count": 107,
        "beam_count_min": 667,
        "beam_count_max": 667,
        "malformed_scan_count": 0,
        "stamp_rate_hz": 21.0,
        "stamp_count": 107,
        "stamp_gap_p50": 0.083,
        "stamp_gap_p95": 0.083,
        "stamp_gap_max": 0.083,
        "time_to_first_all_negative_s": None,
    }

    valid, reasons = matrix.validate_capture_summary(
        bursty, duration_seconds=5.0, stop_reason="duration_reached")

    assert valid
    assert reasons == []


def test_capture_validation_rejects_a_bad_steady_scan_cadence():
    matrix = _load_matrix()
    bad_cadence = {
        "frame_count": 1800,
        "beam_count_min": 667,
        "beam_count_max": 667,
        "malformed_scan_count": 0,
        "stamp_rate_hz": 12.0,
        "stamp_count": 1800,
        "stamp_gap_p50": 0.083,
        "stamp_gap_p95": 0.25,
        "stamp_gap_max": 0.25,
        "time_to_first_all_negative_s": None,
    }

    valid, reasons = matrix.validate_capture_summary(
        bad_cadence, duration_seconds=180.0, stop_reason="duration_reached")

    assert not valid
    assert "unexpected_scan_cadence" in reasons


def test_cli_defaults_match_the_approved_first_round_budget(tmp_path):
    matrix = _load_matrix()

    args = matrix.parse_args(["--output-root", str(tmp_path)])

    assert args.duration == 180.0
    assert args.repeats == 3
    assert args.bad_streak == 60
    assert args.max_wall_seconds == 420.0


def test_cli_can_select_one_named_case_for_a_short_pilot(tmp_path):
    matrix = _load_matrix()

    args = matrix.parse_args([
        "--output-root", str(tmp_path), "--duration", "15", "--repeats", "1",
        "--case", "A_static_raw_m1",
    ])

    assert args.duration == 15.0
    assert args.repeats == 1
    assert args.labels == ["A_static_raw_m1"]


def test_module_entrypoint_runs_only_after_all_runtime_helpers_are_defined():
    source = MODULE.read_text(encoding="utf-8")

    assert source.index('if __name__ == "__main__":') > source.index(
        "def primitive_lidar_model_sdf")
    assert source.index('if __name__ == "__main__":') > source.index(
        "def write_world_loaded_model")


def test_scene_matrix_is_installed_as_a_support_package_entrypoint():
    source = SETUP.read_text(encoding="utf-8")

    assert (
        "gpu_lidar_scene_matrix = "
        "m1_nav2_support.gpu_lidar_scene_matrix:main" in source)


def test_topic_info_counts_exactly_the_publishers_between_transport_headings():
    matrix = _load_matrix()
    info = """Publishers [Address, Message Type]:
  tcp://127.0.0.1:4567, gz.msgs.LaserScan
Subscribers [Address, Message Type]:
  tcp://127.0.0.1:9876, gz.msgs.LaserScan
"""

    assert matrix.publisher_count_from_topic_info(info) == 1
    assert matrix.publisher_count_from_topic_info(
        info.replace("Subscribers", "  tcp://127.0.0.1:4568, gz.msgs.LaserScan\nSubscribers")) == 2


def test_existing_statuses_merge_separate_case_invocations(tmp_path):
    matrix = _load_matrix()
    expected = []
    for case, repeat in (("R4_m1_dynamic_m1", 2), ("A_static_raw_m1", 1)):
        status = {"case": case, "repeat": repeat, "valid": True}
        path = tmp_path / case / f"repeat_{repeat:02d}" / "status.json"
        path.parent.mkdir(parents=True)
        path.write_text(json.dumps(status), encoding="utf-8")
        expected.append(status)

    rows = matrix.load_existing_statuses(tmp_path)

    assert [(row["case"], row["repeat"]) for row in rows] == [
        ("A_static_raw_m1", 1), ("R4_m1_dynamic_m1", 2)]


def test_matrix_summary_keeps_partial_negative_ranges_visible():
    matrix = _load_matrix()
    rows = [
        {
            "case": "R3_minimal_dynamic_m1",
            "repeat": 1,
            "valid": True,
            "stop_reason": "duration_reached",
            "summary": {
                "finite_ratio_mean": 0.936,
                "negative_inf_ratio_mean": 0.064,
                "negative_inf_ratio_max": 0.112,
            },
        },
        {
            "case": "R3_minimal_dynamic_m1",
            "repeat": 2,
            "valid": True,
            "stop_reason": "duration_reached",
            "summary": {
                "finite_ratio_mean": 0.952,
                "negative_inf_ratio_mean": 0.048,
                "negative_inf_ratio_max": 0.091,
            },
        },
    ]

    summary = matrix.summarize_matrix(rows)
    case = summary["cases"]["R3_minimal_dynamic_m1"]

    assert case["finite_ratio_mean_min"] == 0.936
    assert case["negative_inf_ratio_mean_max"] == 0.064
    assert case["negative_inf_ratio_max"] == 0.112
    assert "-Inf max" in matrix.render_markdown(summary)
