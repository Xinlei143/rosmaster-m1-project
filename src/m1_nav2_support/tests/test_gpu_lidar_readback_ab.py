"""Contract tests for the standalone GPU-LiDAR readback campaign."""

import importlib.util
from pathlib import Path
import sys

import pytest


MODULE_PATH = (Path(__file__).parents[1] / "m1_nav2_support"
               / "gpu_lidar_readback_ab.py")
SPEC = importlib.util.spec_from_file_location("gpu_lidar_readback_ab", MODULE_PATH)
AB = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = AB
SPEC.loader.exec_module(AB)


def test_minimal_cases_keep_667_beams_and_only_change_horizontal_fov():
    cases = AB.minimal_world_cases()

    assert [case.label for case in cases] == ["360deg", "180deg"]
    assert all(case.samples == 667 for case in cases)
    assert cases[0].min_angle == pytest.approx(-3.14159265359)
    assert cases[0].max_angle == pytest.approx(3.14159265359)
    assert cases[1].min_angle == pytest.approx(-1.570796326795)
    assert cases[1].max_angle == pytest.approx(1.570796326795)


def test_gazebo_and_raw_commands_are_standalone_server_only():
    case = AB.minimal_world_cases()[0]

    gazebo = AB.build_gazebo_command(Path("/tmp/minimal.sdf"))
    raw = AB.build_raw_capture_command(case.topic, 300.0)

    assert gazebo[:5] == ["ign", "gazebo", "-s", "-r", "-v"]
    assert "--render-engine" in gazebo
    assert "ogre2" in gazebo
    assert "--json-output" in raw
    assert raw[raw.index("-t") + 1] == "/minimal_scan"
    assert raw[raw.index("-d") + 1] == "300.0"
    assert "ros2" not in gazebo + raw


def test_local_prefix_environment_precedes_system_libraries():
    env = AB.build_local_environment(
        {"LD_LIBRARY_PATH": "/existing/lib", "PATH": "/usr/bin"},
        Path("/opt/gz-rendering6-pr1303"),
    )

    assert env["IGN_RENDERING_PLUGIN_PATH"].endswith(
        "lib/ign-rendering-6/engine-plugins")
    assert env["LD_LIBRARY_PATH"].split(":")[0].endswith("/lib")
    assert "/existing/lib" in env["LD_LIBRARY_PATH"].split(":")
    assert env["IGN_RENDERING_RESOURCE_PATH"].split(":")[0].endswith(
        "share/ignition/ignition-rendering6")
    assert "share/ignition-rendering-6" not in env[
        "IGN_RENDERING_RESOURCE_PATH"]


def test_loaded_library_validation_requires_both_rendering_libraries_in_prefix():
    prefix = Path("/opt/gz-rendering6-pr1303")
    valid = (
        "7f /opt/gz-rendering6-pr1303/lib/libignition-rendering6.so.6",
        "8f /opt/gz-rendering6-pr1303/lib/ign-rendering-6/engine-plugins/"
        "libignition-rendering6-ogre2.so.6",
    )
    invalid = valid + (
        "9f /usr/lib/x86_64-linux-gnu/libignition-rendering6.so.6",
    )

    assert AB.validate_loaded_libraries("\n".join(valid), prefix) == {
        "ok": True,
        "rendering_library_paths": [
            "/opt/gz-rendering6-pr1303/lib/libignition-rendering6.so.6"],
        "ogre2_plugin_paths": [
            "/opt/gz-rendering6-pr1303/lib/ign-rendering-6/engine-plugins/"
            "libignition-rendering6-ogre2.so.6"],
        "system_library_paths": [],
    }
    assert AB.validate_loaded_libraries("\n".join(invalid), prefix)["ok"] is False


def test_resource_sample_contains_gpu_and_process_fields():
    sample = AB.parse_resource_sample(
        "timestamp, name, driver_version, utilization.gpu [%], memory.used "
        "[MiB], memory.total [MiB]\n"
        "2026/09/03 12:00:00, RTX 4060, 580.0, 4 %, 123 MiB, 8192 MiB\n",
        "  42  7  1.2  20480 gazebo  ",
    )

    assert sample["gpu"]["name"] == "RTX 4060"
    assert sample["gpu"]["utilization_percent"] == pytest.approx(4.0)
    assert sample["gpu"]["memory_used_mib"] == pytest.approx(123.0)
    assert sample["process"]["pid"] == 42
    assert sample["process"]["rss_kib"] == 20480


def test_soak_summary_marks_any_nonfinite_frame_as_failure():
    summary = AB.summarize_soak_frames([
        {"beam_count": 667, "finite_count": 667, "negative_inf_count": 0,
         "positive_inf_count": 0, "nan_count": 0},
        {"beam_count": 667, "finite_count": 0, "negative_inf_count": 667,
         "positive_inf_count": 0, "nan_count": 0},
    ])

    assert summary["frame_count"] == 2
    assert summary["bad_frame_count"] == 1
    assert summary["whole_frame_negative_inf_count"] == 1
    assert summary["negative_inf_ratio_max"] == pytest.approx(1.0)
