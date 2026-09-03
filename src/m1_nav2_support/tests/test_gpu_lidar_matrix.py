"""Tests for the controlled GPU-LiDAR rendering matrix definition."""

import importlib.util
from pathlib import Path


SCRIPT = (Path(__file__).parents[3] / "scripts" / "run_gpu_lidar_matrix.py")
SPEC = importlib.util.spec_from_file_location("gpu_lidar_matrix", SCRIPT)
MATRIX = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MATRIX)


def test_matrix_has_six_unique_gui_rviz_and_scan_display_conditions():
    cases = MATRIX.matrix_cases()

    assert len(cases) == 6
    assert len({case.label for case in cases}) == 6
    assert {(case.gui, case.rviz, case.scan_display) for case in cases} == {
        (False, False, False),
        (True, False, False),
        (False, True, False),
        (False, True, True),
        (True, True, False),
        (True, True, True),
    }


def test_matrix_commands_keep_native_lidar_and_static_scene_fixed():
    case = MATRIX.matrix_cases()[0]
    command = MATRIX.build_launch_command(case, "ogre2")

    assert "gui:=false" in command
    assert "software_lidar:=false" in command
    assert "dynamic_obstacles:=false" in command
    assert "slip_enabled:=false" in command
    assert "render_engine:=ogre2" in command
    assert "rviz:=false" in command


def test_ros_diagnostic_command_uses_sim_time_for_scan_header_age():
    command = MATRIX.build_ros_diagnostic_command(15.0, "/tmp/summary.json")

    assert "use_sim_time:=true" in command
    assert "duration_seconds:=15.0" in command
