from pathlib import Path
import importlib.util


ROOT = Path(__file__).parents[3]
RUNNER = ROOT / "scripts" / "run_gpu_lidar_single_camera_acceptance.py"
BACKPORT = (
    ROOT / "patches" /
    "ignition-rendering6-6.6.4-single-cubemap-camera.patch")


def test_single_camera_acceptance_runner_exists():
    assert RUNNER.is_file()


def test_backport_preserves_six_face_resources_but_uses_one_camera():
    patch = BACKPORT.read_text()
    additions = "\n".join(
        line[1:] for line in patch.splitlines()
        if line.startswith("+") and not line.startswith("+++"))
    assert "Ogre::Camera *cubeCam{nullptr}" in additions
    assert additions.count("createCamera(") == 1
    assert "cubeCam[i]" not in additions
    assert "setOrientation(Ogre::Quaternion::IDENTITY)" in additions
    assert "firstPassTextures[i]" in patch
    assert "ogreCompositorWorkspace1st[i]" in patch


SPEC = importlib.util.spec_from_file_location("single_camera_acceptance", RUNNER)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _scan_summary():
    return {
        "frame_count": 100,
        "beam_count_min": 667,
        "beam_count_max": 667,
        "all_negative_frame_count": 0,
        "longest_continuous_bad_frames": 0,
        "stamp_gap_p50": 0.083,
        "stamp_gap_p95": 0.084,
    }


def test_launch_command_fixes_the_historical_full_stack_condition():
    command = MODULE.build_launch_command()
    assert "nav2_m1_gazebo.launch.py" in command
    for argument in (
        "gui:=true", "rviz:=true", "software_lidar:=false",
        "dynamic_obstacles:=true", "render_engine:=ogre2",
    ):
        assert argument in command


def test_environment_prefers_only_the_requested_rendering_prefix(tmp_path):
    prefix = tmp_path / "prefix"
    env = MODULE.build_local_environment(
        {"LD_LIBRARY_PATH": "/system-extra"}, prefix, tmp_path, 2)
    assert env["LD_LIBRARY_PATH"].split(":")[0] == str(prefix / "lib")
    assert str(prefix / "lib" / "ign-rendering-6" / "engine-plugins") in env[
        "IGN_RENDERING_PLUGIN_PATH"]
    assert env["IGN_RENDERING_RESOURCE_PATH"] == str(
        prefix / "share" / "ignition" / "ignition-rendering6")
    assert env["ROS_DOMAIN_ID"] == "142"
    assert env["IGN_PARTITION"] == env["GZ_PARTITION"]


def test_common_stamp_comparison_detects_bridge_classification_changes():
    raw = [{"stamp_ns": 10, "beam_count": 667, "finite_count": 667,
            "positive_inf_count": 0, "negative_inf_count": 0, "nan_count": 0}]
    ros = [{**raw[0], "negative_inf_count": 667, "finite_count": 0}]
    assert MODULE.compare_common_stamps(raw, ros) == {
        "common_stamp_count": 1, "mismatch_count": 1}


def test_ros_count_frames_are_normalized_before_summary():
    summary = MODULE.summarize_classified_frames([{
        "stamp_ns": 10,
        "beam_count": 667,
        "finite_count": 667,
        "positive_inf_count": 0,
        "negative_inf_count": 0,
        "nan_count": 0,
    }])
    assert summary["frame_count"] == 1
    assert summary["finite_ratio_mean"] == 1.0


def test_formal_acceptance_rejects_even_one_whole_negative_frame():
    raw = _scan_summary()
    raw["all_negative_frame_count"] = 1
    result = MODULE.assess_repeat(
        raw, _scan_summary(),
        {"common_stamp_count": 90, "mismatch_count": 0},
        {"costmap_publish_rate_hz": 10.0, "dynamic_obstacle_samples": 100,
         "dynamic_obstacle_detected_ratio": 0.99,
         "dynamic_obstacle_detection_miss_duration_max": 0.2},
        {"succeeded_goal_count": 2, "distance_m": 3.0,
         "nonzero_command_counts": {
             "/cmd_vel_nav": 1, "/cmd_vel_smoothed": 1,
             "/m1/cmd_vel_raw": 1}},
        {"ok": True})
    assert not result["passed"]
    assert "raw: whole-frame -Inf observed" in result["failures"]


def test_formal_acceptance_requires_navigation_and_costmap_evidence():
    result = MODULE.assess_repeat(
        _scan_summary(), _scan_summary(),
        {"common_stamp_count": 90, "mismatch_count": 0},
        {"costmap_publish_rate_hz": 10.0, "dynamic_obstacle_samples": 100,
         "dynamic_obstacle_detected_ratio": 0.99,
         "dynamic_obstacle_detection_miss_duration_max": 0.2},
        {"succeeded_goal_count": 2, "distance_m": 3.0,
         "nonzero_command_counts": {
             "/cmd_vel_nav": 1, "/cmd_vel_smoothed": 1,
             "/m1/cmd_vel_raw": 1}},
        {"ok": True})
    assert result == {"passed": True, "failures": []}
