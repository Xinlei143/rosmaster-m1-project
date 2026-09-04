#!/usr/bin/env python3
"""Run the one-condition full-stack acceptance for the Fortress LiDAR patch."""

import argparse
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
SUPPORT = ROOT / "src" / "m1_nav2_support"
sys.path.insert(0, str(SUPPORT))

from m1_nav2_support import gpu_lidar_probe as probe  # noqa: E402


WORKER = SUPPORT / "m1_nav2_support" / "gpu_lidar_waypoint_soak.py"
EXPECTED_BEAMS = 667
CORE_RE = re.compile(
    r"(?P<path>/[^\s]+libignition-rendering6\.so(?:\.[^\s/]+)*)")
PLUGIN_RE = re.compile(
    r"(?P<path>/[^\s]+libignition-rendering6-ogre2\.so(?:\.[^\s/]+)*)")


def build_launch_command():
    return [
        "ros2", "launch", "m1_nav2_bringup", "nav2_m1_gazebo.launch.py",
        "gui:=true", "rviz:=true", "software_lidar:=false",
        "dynamic_obstacles:=true", "render_engine:=ogre2",
    ]


def build_raw_command(duration):
    return [
        "ign", "topic", "-e", "--json-output", "-t", "/scan",
        "-d", f"{float(duration):.1f}",
    ]


def build_worker_command(duration, output):
    return [
        sys.executable, str(WORKER), "--ros-args",
        "-p", "use_sim_time:=true",
        "-p", f"duration_seconds:={float(duration):.1f}",
        "-p", "goal_timeout_seconds:=90.0",
        "-p", f"output:={output}",
    ]


def build_costmap_command(duration, output):
    return [
        "ros2", "run", "m1_nav2_support",
        "m1_costmap_freshness_diagnostic", "--ros-args",
        "-p", "use_sim_time:=true",
        "-p", f"duration_seconds:={float(duration):.1f}",
        "-p", f"output:={output}",
    ]


def build_local_environment(base, prefix, case_dir, repeat):
    env = dict(base)
    prefix = Path(prefix).resolve()
    library_dirs = [prefix / "lib", prefix / "lib" / "x86_64-linux-gnu"]
    existing = env.get("LD_LIBRARY_PATH", "")
    env["LD_LIBRARY_PATH"] = os.pathsep.join(
        [str(path) for path in library_dirs]
        + ([existing] if existing else []))
    env["IGN_RENDERING_PLUGIN_PATH"] = os.pathsep.join([
        str(prefix / "lib" / "x86_64-linux-gnu" / "ign-rendering-6"
            / "engine-plugins"),
        str(prefix / "lib" / "ign-rendering-6" / "engine-plugins"),
    ])
    env["IGN_RENDERING_RESOURCE_PATH"] = str(
        prefix / "share" / "ignition" / "ignition-rendering6")
    env["ROS_LOG_DIR"] = str(case_dir / "ros_log")
    env["IGN_LOG_PATH"] = str(case_dir / "ignition_log")
    env["ROS_DOMAIN_ID"] = str(140 + int(repeat))
    partition = f"gpu_lidar_singlecam_{os.getpid()}_{repeat}"
    env["IGN_PARTITION"] = partition
    env["GZ_PARTITION"] = partition
    return env


def _unique(values):
    return list(dict.fromkeys(values))


def validate_loaded_libraries(maps_text, prefix):
    prefix_text = str(Path(prefix).resolve())
    core = _unique(match.group("path") for match in CORE_RE.finditer(maps_text))
    plugin = _unique(
        match.group("path") for match in PLUGIN_RE.finditer(maps_text))
    foreign = [
        path for path in core + plugin
        if not path.startswith(prefix_text + os.sep)]
    return {
        "ok": bool(core) and bool(plugin) and not foreign,
        "rendering_library_paths": core,
        "ogre2_plugin_paths": plugin,
        "system_library_paths": _unique(foreign),
    }


def _descendant_pids(root_pid):
    pending = [int(root_pid)]
    found = []
    while pending:
        pid = pending.pop()
        if pid in found:
            continue
        found.append(pid)
        child_file = Path("/proc") / str(pid) / "task" / str(pid) / "children"
        try:
            pending.extend(int(value) for value in child_file.read_text().split())
        except (FileNotFoundError, PermissionError, ValueError):
            pass
    return found


def capture_maps(root_pid):
    chunks = []
    for pid in _descendant_pids(root_pid):
        try:
            chunks.append(
                f"# pid {pid}\n" + (Path("/proc") / str(pid) / "maps").read_text())
        except (FileNotFoundError, PermissionError):
            pass
    return "\n".join(chunks)


def _start(command, output, env):
    stream = open(output, "w", encoding="utf-8")
    try:
        process = subprocess.Popen(
            command, stdout=stream, stderr=subprocess.STDOUT, env=env,
            start_new_session=True, text=True)
    except Exception:
        stream.close()
        raise
    process._output_stream = stream
    return process


def _stop(process, timeout=10.0):
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
    text = Path(path).read_text(encoding="utf-8", errors="replace")
    for document in probe.iter_json_documents(text):
        found = probe.frame_stats_from_document(document)
        if found is not None:
            stats, stamp = found
            stats["stamp_ns"] = stamp
            frames.append(stats)
    return frames


def summarize_classified_frames(frames):
    normalized = []
    for original in frames:
        frame = dict(original)
        beams = int(frame.get("beam_count", 0) or 0)
        denominator = float(beams or 1)
        for name in ("finite", "positive_inf", "negative_inf", "nan"):
            frame.setdefault(
                f"{name}_ratio",
                int(frame.get(f"{name}_count", 0) or 0) / denominator)
        normalized.append(frame)
    frames = normalized
    stamps = [frame.get("stamp_ns") for frame in frames]
    return probe.summarize_frames(frames, stamps)


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


def assess_repeat(raw, ros, match, costmap, waypoint, libraries, pilot=False):
    failures = []
    for label, summary in (("raw", raw), ("ros", ros)):
        if summary.get("frame_count", 0) <= 0:
            failures.append(f"{label}: no frames")
        if (summary.get("beam_count_min") != EXPECTED_BEAMS
                or summary.get("beam_count_max") != EXPECTED_BEAMS):
            failures.append(f"{label}: beam count differs from {EXPECTED_BEAMS}")
        if summary.get("all_negative_frame_count") != 0:
            failures.append(f"{label}: whole-frame -Inf observed")
        if summary.get("longest_continuous_bad_frames") != 0:
            failures.append(f"{label}: continuous whole-frame -Inf run observed")
        p50 = summary.get("stamp_gap_p50")
        p95 = summary.get("stamp_gap_p95")
        if p50 is None or not 0.075 <= p50 <= 0.092:
            failures.append(f"{label}: stamp p50 outside 0.075-0.092 s")
        if p95 is None or p95 > 0.100:
            failures.append(f"{label}: stamp p95 exceeds 0.100 s")
    if match.get("common_stamp_count", 0) <= 0:
        failures.append("raw/ROS: no common timestamps")
    if match.get("mismatch_count") != 0:
        failures.append("raw/ROS: common timestamp classification mismatch")
    if not libraries.get("ok"):
        failures.append("runtime libraries were not isolated to patched prefix")
    if not pilot:
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
        if waypoint.get("succeeded_goal_count", 0) < 2:
            failures.append("fewer than two navigation goals succeeded")
        if waypoint.get("distance_m", 0.0) < 2.0:
            failures.append("robot traveled less than 2 m")
        for topic in ("/cmd_vel_nav", "/cmd_vel_smoothed", "/m1/cmd_vel_raw"):
            if waypoint.get("nonzero_command_counts", {}).get(topic, 0) <= 0:
                failures.append(f"no nonzero command observed on {topic}")
    return {"passed": not failures, "failures": failures}


def _resource_sample(launch_pid):
    result = subprocess.run(
        ["nvidia-smi", "--query-gpu=timestamp,name,driver_version,"
         "utilization.gpu,memory.used,memory.total", "--format=csv,noheader"],
        capture_output=True, text=True, check=False, timeout=5.0)
    return {
        "wall_time": time.time(), "launch_pid": launch_pid,
        "nvidia_smi": result.stdout.strip(), "nvidia_returncode": result.returncode,
    }


def run_repeat(output_root, prefix, duration, repeat, startup_grace, pilot):
    case_dir = Path(output_root) / f"repeat_{repeat:02d}"
    if case_dir.exists():
        raise FileExistsError(f"refusing to overwrite {case_dir}")
    case_dir.mkdir(parents=True)
    (case_dir / "ros_log").mkdir()
    (case_dir / "ignition_log").mkdir()
    env = build_local_environment(os.environ, prefix, case_dir, repeat)
    metadata = {
        "duration_seconds": duration, "startup_grace_seconds": startup_grace,
        "repeat": repeat, "pilot": pilot, "prefix": str(Path(prefix).resolve()),
        "commands": {
            "launch": build_launch_command(),
            "raw": build_raw_command(duration),
            "worker": build_worker_command(duration, case_dir / "waypoint.json"),
            "costmap": build_costmap_command(duration, case_dir / "costmap.json"),
        },
        "environment": {key: env.get(key) for key in (
            "ROS_DOMAIN_ID", "IGN_PARTITION", "GZ_PARTITION", "LD_LIBRARY_PATH",
            "IGN_RENDERING_PLUGIN_PATH", "IGN_RENDERING_RESOURCE_PATH")},
    }
    (case_dir / "metadata.json").write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    launch = raw = worker = costmap_process = None
    statuses = {}
    maps_text = ""
    try:
        launch = _start(build_launch_command(), case_dir / "launch.log", env)
        deadline = time.monotonic() + startup_grace
        while time.monotonic() < deadline:
            if launch.poll() is not None:
                raise RuntimeError("launch exited during startup grace")
            time.sleep(min(1.0, deadline - time.monotonic()))
        maps_text = capture_maps(launch.pid)
        (case_dir / "process_maps.txt").write_text(maps_text, encoding="utf-8")
        raw = _start(build_raw_command(duration), case_dir / "gazebo_scan.jsonlog", env)
        worker = _start(
            build_worker_command(duration, case_dir / "waypoint.json"),
            case_dir / "waypoint.log", env)
        costmap_process = _start(
            build_costmap_command(duration, case_dir / "costmap.json"),
            case_dir / "costmap.log", env)
        with open(case_dir / "resources.jsonl", "w", encoding="utf-8") as stream:
            deadline = time.monotonic() + duration + 60.0
            while time.monotonic() < deadline:
                if all(process.poll() is not None
                       for process in (raw, worker, costmap_process)):
                    break
                stream.write(json.dumps(_resource_sample(launch.pid)) + "\n")
                stream.flush()
                time.sleep(1.0)
        statuses["raw"] = _stop(raw)
        statuses["waypoint"] = _stop(worker)
        statuses["costmap"] = _stop(costmap_process)
    except Exception as error:
        statuses["runner_error"] = str(error)
    finally:
        statuses["raw"] = _stop(raw) if raw is not None else statuses.get("raw")
        statuses["waypoint"] = (
            _stop(worker) if worker is not None else statuses.get("waypoint"))
        statuses["costmap"] = (
            _stop(costmap_process) if costmap_process is not None
            else statuses.get("costmap"))
        statuses["launch"] = _stop(launch)

    raw_frames = _raw_frames(case_dir / "gazebo_scan.jsonlog") \
        if (case_dir / "gazebo_scan.jsonlog").exists() else []
    waypoint = json.loads((case_dir / "waypoint.json").read_text()) \
        if (case_dir / "waypoint.json").exists() else {}
    ros_frames = waypoint.get("frames", [])
    costmap = json.loads((case_dir / "costmap.json").read_text()) \
        if (case_dir / "costmap.json").exists() else {}
    raw_summary = summarize_classified_frames(raw_frames)
    ros_summary = summarize_classified_frames(ros_frames)
    match = compare_common_stamps(raw_frames, ros_frames)
    libraries = validate_loaded_libraries(maps_text, prefix)
    acceptance = assess_repeat(
        raw_summary, ros_summary, match, costmap, waypoint, libraries,
        pilot=pilot)
    if statuses.get("runner_error"):
        acceptance["passed"] = False
        acceptance["failures"].append(statuses["runner_error"])
    for name in ("raw", "waypoint", "costmap"):
        if statuses.get(name) != 0:
            acceptance["passed"] = False
            acceptance["failures"].append(
                f"{name} process exited {statuses.get(name)}")
    summary = {
        "raw": raw_summary, "ros": ros_summary, "raw_ros_match": match,
        "costmap": costmap, "waypoint": {
            key: value for key, value in waypoint.items() if key != "frames"},
        "libraries": libraries, "statuses": statuses,
        "acceptance": acceptance,
    }
    (case_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prefix", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=600.0)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--startup-grace", type=float, default=45.0)
    parser.add_argument("--pilot", action="store_true")
    args = parser.parse_args(argv)
    if args.duration <= 0 or args.repeats <= 0 or args.startup_grace < 0:
        parser.error("duration/repeats must be positive and startup grace nonnegative")
    args.output_root.mkdir(parents=True, exist_ok=True)
    rows = []
    for repeat in range(1, args.repeats + 1):
        rows.append(run_repeat(
            args.output_root, args.prefix, args.duration, repeat,
            args.startup_grace, args.pilot))
        if not rows[-1]["acceptance"]["passed"] and not args.pilot:
            break
    campaign = {
        "passed": len(rows) == args.repeats and all(
            row["acceptance"]["passed"] for row in rows),
        "requested_repeats": args.repeats, "completed_repeats": len(rows),
        "repeats": rows,
    }
    (args.output_root / "campaign_summary.json").write_text(
        json.dumps(campaign, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0 if campaign["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
