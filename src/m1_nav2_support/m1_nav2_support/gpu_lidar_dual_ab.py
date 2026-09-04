"""Run the controlled OGRE2 single-360 versus Ogre1 dual-180 campaign."""

import argparse
from dataclasses import dataclass
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

from . import gpu_lidar_probe as probe


@dataclass(frozen=True)
class CampaignCase:
    label: str
    repeat: int
    duration: float
    render_engine: str
    dual_gpu_lidar: bool


def campaign_cases(repeats=3, duration=300.0):
    cases = []
    for repeat in range(1, int(repeats) + 1):
        cases.extend([
            CampaignCase(
                "A_ogre2_single360", repeat, float(duration), "ogre2", False),
            CampaignCase(
                "B_ogre1_dual180", repeat, float(duration), "ogre", True),
        ])
    return cases


def navigation_cases(repeats=3, duration=300.0):
    return [
        CampaignCase(
            "navigation_ogre1_dual180", repeat, float(duration), "ogre", True)
        for repeat in range(1, int(repeats) + 1)
    ]


def build_launch_command(case):
    return [
        "ros2", "launch", "m1_nav2_bringup", "nav2_m1_gazebo.launch.py",
        "gui:=true", "rviz:=true", "software_lidar:=false",
        "dynamic_obstacles:=true", "slip_enabled:=false",
        f"render_engine:={case.render_engine}",
        f"dual_gpu_lidar:={'true' if case.dual_gpu_lidar else 'false'}",
    ]


def capture_topics(case):
    if case.dual_gpu_lidar:
        return {
            "gazebo": ["/scan_front", "/scan_rear"],
            "ros": ["/scan_front", "/scan_rear"],
            "merged": "/scan",
        }
    return {"gazebo": ["/scan"], "ros": ["/scan"], "merged": None}


def compare_common_stamps(raw_frames, ros_frames):
    fields = (
        "beam_count", "finite_count", "positive_inf_count",
        "negative_inf_count", "nan_count")
    raw = {frame.get("stamp_ns"): frame for frame in raw_frames}
    ros = {frame.get("stamp_ns"): frame for frame in ros_frames}
    common = sorted((set(raw) & set(ros)) - {None})
    mismatches = sum(
        any(int(raw[stamp].get(field, 0)) != int(ros[stamp].get(field, 0))
            for field in fields)
        for stamp in common)
    return {"common_stamp_count": len(common), "mismatch_count": mismatches}


def _clean_scan_failures(label, summary, expected_beams):
    failures = []
    if summary.get("frame_count", 0) <= 0:
        failures.append(f"{label}: no frames")
    if (summary.get("beam_count_min") != expected_beams
            or summary.get("beam_count_max") != expected_beams):
        failures.append(f"{label}: beam count differs from {expected_beams}")
    if summary.get("all_negative_frame_count", 0) != 0:
        failures.append(f"{label}: whole-frame -Inf observed")
    if summary.get("longest_continuous_bad_frames", 0) != 0:
        failures.append(f"{label}: all-negative streak observed")
    if summary.get("nan_frame_count", 0) != 0 or summary.get("nan_ratio_mean", 0.0) != 0.0:
        failures.append(f"{label}: NaN observed")
    p50 = summary.get("stamp_gap_p50")
    p95 = summary.get("stamp_gap_p95")
    if p50 is None or not 0.075 <= p50 <= 0.092:
        failures.append(f"{label}: p50 interval outside 0.075-0.092 s")
    if p95 is None or p95 > 0.100:
        failures.append(f"{label}: p95 interval exceeds 0.100 s")
    return failures


def assess_treatment(raw, matches, merged):
    failures = []
    for label, summary in raw.items():
        failures.extend(_clean_scan_failures(label, summary, 334))
    for label, match in matches.items():
        if match.get("common_stamp_count", 0) <= 0:
            failures.append(f"{label}: no common Gazebo/ROS timestamps")
        if match.get("mismatch_count", 0) != 0:
            failures.append(f"{label}: Gazebo/ROS classification mismatch")
    if merged.get("frame_count", 0) <= 0:
        failures.append("merged: no frames")
    if (merged.get("beam_count_min") != 667
            or merged.get("beam_count_max") != 667):
        failures.append("merged: beam count differs from 667")
    if merged.get("frame_ids") != ["laser_scan_link"]:
        failures.append("merged: unexpected frame_id")
    if merged.get("publisher_count") != 1:
        failures.append("merged: publisher count differs from 1")
    p50 = merged.get("stamp_gap_p50")
    p95 = merged.get("stamp_gap_p95")
    if p50 is None or not 0.075 <= p50 <= 0.092:
        failures.append("merged: p50 interval outside 0.075-0.092 s")
    if p95 is None or p95 > 0.100:
        failures.append("merged: p95 interval exceeds 0.100 s")
    return {"passed": not failures, "failures": failures}


def assess_positive_control(latched_repeats):
    count = sum(bool(value) for value in latched_repeats)
    failures = [] if count >= 2 else [
        f"positive control reproduced in {count}/3 repeats; fewer than 2/3"]
    return {"passed": not failures, "latched_repeat_count": count, "failures": failures}


def assess_navigation(treatment, waypoint, costmap):
    failures = list(treatment.get("failures", []))
    if not treatment.get("passed") and not failures:
        failures.append("dual-LiDAR treatment assessment failed")
    if waypoint.get("succeeded_goal_count", 0) < 2:
        failures.append("fewer than two navigation goals succeeded")
    if waypoint.get("distance_m", 0.0) < 2.0:
        failures.append("robot traveled less than 2 m")
    for topic in ("/cmd_vel_nav", "/cmd_vel_smoothed", "/m1/cmd_vel_raw"):
        if waypoint.get("nonzero_command_counts", {}).get(topic, 0) <= 0:
            failures.append(f"no nonzero command observed on {topic}")
    if costmap.get("costmap_publish_rate_hz", 0.0) <= 0.0:
        failures.append("local costmap did not publish")
    if costmap.get("dynamic_obstacle_samples", 0) <= 0:
        failures.append("no eligible dynamic-obstacle samples")
    ratio = costmap.get("dynamic_obstacle_detected_ratio")
    if ratio is None or ratio < 0.95:
        failures.append("dynamic-obstacle detection ratio below 0.95")
    miss = costmap.get("dynamic_obstacle_detection_miss_duration_max")
    if miss is not None and miss > 0.5:
        failures.append("dynamic-obstacle miss exceeded 0.5 s")
    return {"passed": not failures, "failures": failures}


def _start(command, output, env):
    stream = open(output, "w", encoding="utf-8")
    try:
        process = subprocess.Popen(
            command, stdout=stream, stderr=subprocess.STDOUT,
            env=env, start_new_session=True, text=True)
    except Exception:
        stream.close()
        raise
    process._output_stream = stream
    return process


def _stop(process, timeout=15.0):
    if process is None:
        return None
    if process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait(timeout=timeout)
    stream = getattr(process, "_output_stream", None)
    if stream is not None:
        stream.close()
    return process.returncode


def _raw_frames(path):
    frames = []
    text = path.read_text(encoding="utf-8", errors="replace") if path.exists() else ""
    for document in probe.iter_json_documents(text):
        found = probe.frame_stats_from_document(document)
        if found is None:
            continue
        frame, stamp = found
        frame["stamp_ns"] = stamp
        frames.append(frame)
    return frames


def _ros_frames(path):
    frames = []
    if not path.exists():
        return frames
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        try:
            frames.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return frames


def _summarize(frames):
    summary = probe.summarize_frames(
        frames, [frame.get("stamp_ns") for frame in frames])
    summary["frame_ids"] = sorted({
        frame.get("frame_id") for frame in frames if frame.get("frame_id")})
    return summary


def _topic_label(topic):
    return topic.strip("/").replace("/", "_") or "scan"


def _publisher_count(text):
    match = re.search(r"Publisher count:\s*(\d+)", text)
    return int(match.group(1)) if match else None


def _environment(case_dir, repeat):
    env = dict(os.environ)
    env["ROS_LOG_DIR"] = str(case_dir / "ros_log")
    env["IGN_LOG_PATH"] = str(case_dir / "ignition_log")
    env["ROS_DOMAIN_ID"] = str(180 + int(repeat))
    partition = f"gpu_lidar_dual_ab_{os.getpid()}_{repeat}"
    env["IGN_PARTITION"] = partition
    env["GZ_PARTITION"] = partition
    return env


def run_case(output_root, case, startup_grace=45.0, navigation=False):
    case_dir = Path(output_root) / f"pair_{case.repeat:02d}" / case.label
    if case_dir.exists():
        raise FileExistsError(f"refusing to overwrite {case_dir}")
    (case_dir / "ros_log").mkdir(parents=True)
    (case_dir / "ignition_log").mkdir()
    env = _environment(case_dir, case.repeat)
    topics = capture_topics(case)
    ros_topics = list(topics["ros"])
    if topics["merged"]:
        ros_topics.append(topics["merged"])
    metadata = {
        "case": case.__dict__, "startup_grace_seconds": startup_grace,
        "commands": {"launch": build_launch_command(case)},
        "topics": topics,
        "environment": {key: env.get(key) for key in (
            "ROS_DOMAIN_ID", "IGN_PARTITION", "GZ_PARTITION")},
    }
    (case_dir / "metadata.json").write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    launch = capture = waypoint_process = costmap_process = None
    raw_processes = []
    status = {}
    try:
        launch = _start(build_launch_command(case), case_dir / "launch.log", env)
        deadline = time.monotonic() + startup_grace
        while time.monotonic() < deadline:
            if launch.poll() is not None:
                raise RuntimeError(f"launch exited early with {launch.returncode}")
            time.sleep(0.5)
        capture_command = [
            "ros2", "run", "m1_nav2_support", "gpu_lidar_ros_capture",
            "--duration", str(case.duration), "--output-root", str(case_dir),
            "--topics", *ros_topics,
        ]
        metadata["commands"]["ros_capture"] = capture_command
        capture = _start(capture_command, case_dir / "ros_capture.log", env)
        if navigation:
            waypoint_command = [
                sys.executable,
                str(Path(__file__).with_name("gpu_lidar_waypoint_soak.py")),
                "--ros-args", "-p", "use_sim_time:=true",
                "-p", f"duration_seconds:={case.duration}",
                "-p", "goal_timeout_seconds:=90.0",
                "-p", f"output:={case_dir / 'waypoint.json'}",
            ]
            costmap_command = [
                "ros2", "run", "m1_nav2_support",
                "m1_costmap_freshness_diagnostic", "--ros-args",
                "-p", "use_sim_time:=true",
                "-p", f"duration_seconds:={case.duration}",
                "-p", f"output:={case_dir / 'costmap.json'}",
            ]
            metadata["commands"]["waypoint"] = waypoint_command
            metadata["commands"]["costmap"] = costmap_command
            waypoint_process = _start(
                waypoint_command, case_dir / "waypoint.log", env)
            costmap_process = _start(
                costmap_command, case_dir / "costmap.log", env)
        for topic in topics["gazebo"]:
            command = [
                "ign", "topic", "-e", "--json-output", "-t", topic,
                "-d", str(case.duration),
            ]
            metadata["commands"][f"gazebo_{_topic_label(topic)}"] = command
            raw_processes.append((topic, _start(
                command, case_dir / f"gazebo_{_topic_label(topic)}.jsonlog", env)))
        (case_dir / "metadata.json").write_text(
            json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        deadline = time.monotonic() + case.duration + 45.0
        while time.monotonic() < deadline:
            if capture.poll() is not None and all(
                    process.poll() is not None for _, process in raw_processes):
                break
            if launch.poll() is not None:
                break
            time.sleep(0.5)
        info = subprocess.run(
            ["ros2", "topic", "info", "/scan", "--verbose"], env=env,
            capture_output=True, text=True, timeout=10.0, check=False)
        (case_dir / "scan_topic_info.txt").write_text(
            info.stdout + info.stderr, encoding="utf-8")
        status["topic_info"] = info.returncode
    finally:
        status["ros_capture"] = _stop(capture)
        status["waypoint"] = _stop(waypoint_process)
        status["costmap"] = _stop(costmap_process)
        for topic, process in raw_processes:
            status[f"gazebo_{_topic_label(topic)}"] = _stop(process)
        status["launch"] = _stop(launch)

    gazebo_frames = {}
    ros_frames = {}
    for topic in topics["gazebo"]:
        label = _topic_label(topic)
        gazebo_frames[label] = _raw_frames(case_dir / f"gazebo_{label}.jsonlog")
    for topic in ros_topics:
        label = _topic_label(topic)
        ros_frames[label] = _ros_frames(case_dir / f"ros_{label}.jsonl")

    summaries = {
        "gazebo": {name: _summarize(frames) for name, frames in gazebo_frames.items()},
        "ros": {name: _summarize(frames) for name, frames in ros_frames.items()},
    }
    matches = {
        name: compare_common_stamps(gazebo_frames[name], ros_frames.get(name, []))
        for name in gazebo_frames
    }
    if case.dual_gpu_lidar:
        raw = {
            "gazebo_front": summaries["gazebo"].get("scan_front", {}),
            "gazebo_rear": summaries["gazebo"].get("scan_rear", {}),
            "ros_front": summaries["ros"].get("scan_front", {}),
            "ros_rear": summaries["ros"].get("scan_rear", {}),
        }
        merged = dict(summaries["ros"].get("scan", {}))
        merged["publisher_count"] = _publisher_count(
            (case_dir / "scan_topic_info.txt").read_text(encoding="utf-8"))
        treatment = assess_treatment(raw, matches, merged)
        if navigation:
            waypoint_path = case_dir / "waypoint.json"
            costmap_path = case_dir / "costmap.json"
            waypoint = json.loads(waypoint_path.read_text()) if waypoint_path.exists() else {}
            costmap = json.loads(costmap_path.read_text()) if costmap_path.exists() else {}
            assessment = assess_navigation(treatment, waypoint, costmap)
        else:
            waypoint = costmap = None
            assessment = treatment
    else:
        latch = summaries["gazebo"].get("scan", {}).get(
            "all_negative_frame_count", 0) > 0
        assessment = {"latched": latch, "passed": latch, "failures": [] if latch else [
            "positive-control run did not reproduce whole-frame -Inf"]}
    result = {
        "case": case.__dict__, "statuses": status, "summaries": summaries,
        "raw_ros_matches": matches, "assessment": assessment,
    }
    if navigation:
        result["waypoint"] = waypoint
        result["costmap"] = costmap
    (case_dir / "summary.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=300.0)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--startup-grace", type=float, default=45.0)
    parser.add_argument(
        "--navigation-only", action="store_true",
        help="Run Ogre1 dual-180 navigation repeats without positive controls.")
    args = parser.parse_args(argv)
    if args.duration <= 0 or args.repeats <= 0 or args.startup_grace < 0:
        parser.error("duration/repeats must be positive and startup grace nonnegative")
    if args.output_root.exists():
        parser.error(f"refusing to overwrite existing output root: {args.output_root}")
    args.output_root.mkdir(parents=True)

    results = []
    cases = (
        navigation_cases(args.repeats, args.duration)
        if args.navigation_only else campaign_cases(args.repeats, args.duration))
    for case in cases:
        results.append(run_case(
            args.output_root, case, args.startup_grace,
            navigation=args.navigation_only))
    if args.navigation_only:
        campaign = {
            "navigation_passed": all(
                result["assessment"]["passed"] for result in results),
            "repeat_count": len(results),
            "run_order": [result["case"] for result in results],
        }
        (args.output_root / "campaign_summary.json").write_text(
            json.dumps(campaign, indent=2, sort_keys=True) + "\n",
            encoding="utf-8")
        print(json.dumps(campaign, indent=2, sort_keys=True))
        return 0 if campaign["navigation_passed"] else 1
    a_results = [result for result in results if not result["case"]["dual_gpu_lidar"]]
    b_results = [result for result in results if result["case"]["dual_gpu_lidar"]]
    positive = assess_positive_control([
        result["assessment"].get("latched", False) for result in a_results])
    treatment_passed = all(result["assessment"]["passed"] for result in b_results)
    campaign = {
        "positive_control": positive,
        "treatment": {
            "passed": treatment_passed,
            "repeat_count": len(b_results),
        },
        "conclusion_supported": positive["passed"] and treatment_passed,
        "run_order": [result["case"] for result in results],
    }
    (args.output_root / "campaign_summary.json").write_text(
        json.dumps(campaign, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(campaign, indent=2, sort_keys=True))
    return 0 if campaign["conclusion_supported"] else 1


if __name__ == "__main__":
    sys.exit(main())
