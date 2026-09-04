"""Run raw-only GPU-LiDAR scene-isolation experiments."""

import argparse
import copy
from dataclasses import dataclass
import json
import os
from pathlib import Path
import select
import signal
import statistics
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[3]
SUPPORT_SOURCE = ROOT / "src" / "m1_nav2_support"
WORLDS = SUPPORT_SOURCE / "worlds"
M1_XACRO = (ROOT / "src" / "yahboomcar_description" / "urdf"
            / "yahboomcar_M1_gazebo.urdf.xacro")
EXPECTED_BEAM_COUNT = 667
IGNITION_XML_NAMESPACE = "http://ignitionrobotics.org/schema"
ET.register_namespace("ignition", IGNITION_XML_NAMESPACE)


@dataclass(frozen=True)
class SceneCase:
    """One world/carrier/creation-factor experiment."""

    label: str
    world_file: str
    carrier: str
    creation: str
    topic: str


@dataclass
class BadStreakTracker:
    """Track the current sequence of expected-width all-negative frames."""

    expected_beam_count: int
    threshold: int
    current_streak: int = 0

    def observe(self, frame):
        is_all_negative = (
            frame.get("beam_count") == self.expected_beam_count
            and frame.get("negative_inf_count") == self.expected_beam_count
        )
        self.current_streak = self.current_streak + 1 if is_all_negative else 0
        return self.current_streak >= self.threshold


class JsonDocumentStream:
    """Incrementally decode JSON documents emitted by ``ign topic -e``."""

    def __init__(self):
        self._decoder = json.JSONDecoder()
        self._buffer = ""

    def feed(self, chunk):
        self._buffer += chunk
        documents = []
        while self._buffer:
            starts = [index for index in (
                self._buffer.find("{"), self._buffer.find("[")) if index >= 0]
            if not starts:
                self._buffer = ""
                break
            start = min(starts)
            if start:
                self._buffer = self._buffer[start:]
            try:
                document, end = self._decoder.raw_decode(self._buffer)
            except json.JSONDecodeError:
                break
            documents.append(document)
            self._buffer = self._buffer[end:]
        return documents


def scene_matrix_cases():
    """Return the fixed Test A and R0--R5 isolation matrix."""
    return (
        SceneCase("A_static_raw_m1", "m1_static_raw.sdf", "m1",
                  "dynamic_urdf", "/scan"),
        SceneCase("R0_minimal_world_load_primitive",
                  "gpu_lidar_minimal_360.sdf", "primitive",
                  "world_loaded", "/minimal_scan"),
        SceneCase("R1_minimal_dynamic_primitive",
                  "gpu_lidar_minimal_base.sdf", "primitive",
                  "dynamic_sdf", "/scan"),
        SceneCase("R2_m1_dynamic_primitive", "m1.sdf", "primitive",
                  "dynamic_sdf", "/scan"),
        SceneCase("R3_minimal_dynamic_m1",
                  "gpu_lidar_minimal_base.sdf", "m1",
                  "dynamic_urdf", "/scan"),
        SceneCase("R4_m1_dynamic_m1", "m1.sdf", "m1", "dynamic_urdf",
                  "/scan"),
        SceneCase("R5_m1_world_load_m1", "m1.sdf", "m1",
                  "world_loaded_sdf", "/scan"),
    )


def dynamic_spawn_command(case, payload, world_name):
    """Build the sole ROS process used by a dynamic-spawn case."""
    if case.creation not in {"dynamic_urdf", "dynamic_sdf"}:
        raise ValueError(f"{case.label} is not a dynamic-spawn case")
    command = [
        "ros2", "run", "ros_gz_sim", "create",
        "-world", str(world_name),
        "-name", "m1" if case.carrier == "m1" else "gpu_lidar_probe",
    ]
    if case.creation == "dynamic_urdf":
        command.extend(["-string", str(payload), "-x", "-2.5", "-y", "-1.5",
                        "-z", "0.01"])
    else:
        command.extend(["-file", str(payload), "-x", "0", "-y", "0", "-z", "1"])
    return command


def gazebo_command(world_file):
    """Start the Gazebo server only; GUI, bridge, Nav2, and RViz stay absent."""
    return [
        "ign", "gazebo", "-s", "-r", "-v", "4", "--render-engine", "ogre2",
        str(world_file),
    ]


def validate_capture_summary(summary, duration_seconds, stop_reason,
                             expected_beam_count=667):
    """Reject incomplete or malformed raw captures before interpreting outcomes."""
    reasons = []
    if int(summary.get("frame_count") or 0) <= 0:
        reasons.append("zero_frames")
    if (summary.get("beam_count_min") != expected_beam_count
            or summary.get("beam_count_max") != expected_beam_count):
        reasons.append("unexpected_beam_count")
    if int(summary.get("malformed_scan_count") or 0) != 0:
        reasons.append("malformed_frames")
    if int(summary.get("stamp_count") or 0) < 2:
        reasons.append("insufficient_timestamps")
    # Do not use the end-to-end rate as an acceptance gate: Gazebo may emit a
    # short startup burst with closely spaced simulation stamps.  The median
    # and p95 gaps describe the steady sensor cadence without hiding a later
    # scheduling stall in a simple average.
    gap_p50 = summary.get("stamp_gap_p50")
    gap_p95 = summary.get("stamp_gap_p95")
    if (gap_p50 is None or gap_p95 is None
            or not 0.05 <= float(gap_p50) <= 0.12
            or not 0.05 <= float(gap_p95) <= 0.12):
        reasons.append("unexpected_scan_cadence")

    if stop_reason == "duration_reached":
        minimum_frames = int(float(duration_seconds) * 10.0)
        if int(summary.get("frame_count") or 0) < minimum_frames:
            reasons.append("short_capture")
    elif stop_reason.startswith("bad_streak_"):
        required_streak = int(stop_reason.rsplit("_", 1)[1])
        if summary.get("time_to_first_all_negative_s") is None:
            reasons.append("missing_bad_transition")
        if int(summary.get("longest_continuous_bad_frames") or 0) < required_streak:
            reasons.append("insufficient_bad_streak")
    else:
        reasons.append("unknown_stop_reason")
    return not reasons, reasons


def parse_args(argv=None):
    """Parse the deliberately bounded first-round experiment interface."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=180.0,
                        help="Target simulation-time capture duration in seconds.")
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--bad-streak", type=int, default=60,
                        help="Stop after this many consecutive all--Inf frames.")
    parser.add_argument("--max-wall-seconds", type=float, default=420.0,
                        help="Safety limit for a stalled simulation.")
    parser.add_argument("--case", action="append", dest="labels",
                        help="Run only this matrix case; may be repeated.")
    return parser.parse_args(argv)


def _write_json(path, value):
    Path(path).write_text(
        json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _start_logged(command, log_path, env):
    stream = open(log_path, "w", encoding="utf-8")
    try:
        process = subprocess.Popen(
            command, stdout=stream, stderr=subprocess.STDOUT, env=env,
            start_new_session=True, text=True)
    except Exception:
        stream.close()
        raise
    process._matrix_log_stream = stream
    return process


def _stop_process(process, timeout=10.0):
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
    stream = getattr(process, "_matrix_log_stream", None)
    if stream is not None:
        stream.close()
    return process.returncode


def _run_listing(command, env):
    try:
        result = subprocess.run(
            command, env=env, text=True, capture_output=True, timeout=5.0,
            check=False)
    except (FileNotFoundError, subprocess.TimeoutExpired) as error:
        return "", str(error)
    return result.stdout, result.stderr


def _wait_for_listing(command, expected, env, timeout=30.0):
    deadline = time.monotonic() + timeout
    last_error = ""
    while time.monotonic() < deadline:
        output, error = _run_listing(command, env)
        if expected in output.splitlines():
            return True, ""
        last_error = error
        time.sleep(0.5)
    return False, last_error


def publisher_count_from_topic_info(text):
    """Count transport publishers in the section emitted by ``ign topic -i``."""
    in_publishers = False
    count = 0
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith("Publishers"):
            in_publishers = True
            continue
        if stripped.startswith("Subscribers"):
            break
        if in_publishers and stripped:
            count += 1
    return count


def _world_name(world_file):
    root = ET.parse(world_file).getroot()
    world = root.find("world")
    if world is None or not world.get("name"):
        raise ValueError(f"{world_file} does not contain a named SDF world")
    return world.get("name")


def _isolated_environment(case_dir, run_number):
    env = os.environ.copy()
    token = f"m1_gpu_scene_{os.getpid()}_{run_number}_{uuid.uuid4().hex[:8]}"
    resource_root = str(ROOT / "src")
    for variable in ("IGN_GAZEBO_RESOURCE_PATH", "GZ_SIM_RESOURCE_PATH"):
        existing = env.get(variable)
        env[variable] = os.pathsep.join(filter(None, [resource_root, existing]))
    env["IGN_PARTITION"] = token
    env["GZ_PARTITION"] = token
    env["ROS_DOMAIN_ID"] = str(80 + (run_number % 100))
    env["ROS_LOG_DIR"] = str(Path(case_dir) / "ros_log")
    Path(env["ROS_LOG_DIR"]).mkdir(parents=True, exist_ok=True)
    return env


def _expand_m1_xacro(case_dir, env):
    command = [
        "xacro", str(M1_XACRO), "enable_gpu_lidar:=true",
        "gpu_lidar_min_angle:=-3.14159265359",
        "gpu_lidar_max_angle:=3.14159265359",
    ]
    result = subprocess.run(
        command, env=env, text=True, capture_output=True, check=False)
    log_path = Path(case_dir) / "xacro.log"
    log_path.write_text(result.stdout + result.stderr, encoding="utf-8")
    if result.returncode != 0:
        raise RuntimeError(f"xacro failed with status {result.returncode}")
    output = Path(case_dir) / "m1_expanded.urdf"
    output.write_text(result.stdout, encoding="utf-8")
    return result.stdout, output


def _prepare_case_world(case, case_dir, env):
    """Return the world and optional dynamic-spawn payload for a matrix case."""
    base_world = WORLDS / case.world_file
    if case.creation == "world_loaded":
        return base_world, None
    if case.creation == "world_loaded_sdf":
        _, expanded = _expand_m1_xacro(case_dir, env)
        converted = Path(case_dir) / "m1_preconverted.sdf"
        result = subprocess.run(
            ["ign", "sdf", "-p", str(expanded)], env=env, text=True,
            capture_output=True, check=False)
        (Path(case_dir) / "sdf_conversion.log").write_text(
            result.stdout + result.stderr, encoding="utf-8")
        if result.returncode != 0:
            raise RuntimeError(f"ign sdf conversion failed with status {result.returncode}")
        converted.write_text(result.stdout, encoding="utf-8")
        combined = Path(case_dir) / "m1_world_loaded.sdf"
        write_world_loaded_model(base_world, converted, combined)
        return combined, None
    if case.creation == "dynamic_urdf":
        xml, _ = _expand_m1_xacro(case_dir, env)
        return base_world, xml
    if case.creation == "dynamic_sdf":
        model = Path(case_dir) / "gpu_lidar_probe.sdf"
        model.write_text(primitive_lidar_model_sdf(), encoding="utf-8")
        return base_world, model
    raise ValueError(f"unknown creation mode: {case.creation}")


def _capture_raw_scan(case_dir, topic, env, duration_seconds, bad_streak,
                      max_wall_seconds):
    """Capture only native Gazebo Transport scan output until a defined stop."""
    from m1_nav2_support.gpu_lidar_probe import (
        analyze_text, frame_stats_from_document)

    raw_path = Path(case_dir) / "gazebo_scan.jsonlog"
    stderr_path = Path(case_dir) / "raw_capture.stderr.log"
    raw_stream = open(raw_path, "w", encoding="utf-8")
    stderr_stream = open(stderr_path, "w", encoding="utf-8")
    process = None
    tracker = BadStreakTracker(EXPECTED_BEAM_COUNT, bad_streak)
    decoder = JsonDocumentStream()
    first_stamp = None
    last_stamp = None
    stop_reason = "wall_timeout"
    started = time.monotonic()
    try:
        process = subprocess.Popen(
            ["ign", "topic", "-e", "--json-output", "-t", topic],
            stdout=subprocess.PIPE, stderr=stderr_stream, env=env,
            start_new_session=True, text=True, bufsize=1)
        while time.monotonic() - started < max_wall_seconds:
            if process.poll() is not None:
                stop_reason = "raw_capture_exited"
                break
            ready, _, _ = select.select([process.stdout], [], [], 0.5)
            if not ready:
                continue
            chunk = process.stdout.readline()
            if not chunk:
                stop_reason = "raw_capture_exited"
                break
            raw_stream.write(chunk)
            raw_stream.flush()
            for document in decoder.feed(chunk):
                try:
                    found = frame_stats_from_document(document)
                except (TypeError, ValueError):
                    found = None
                if found is None:
                    continue
                frame, stamp = found
                if stamp is not None:
                    first_stamp = stamp if first_stamp is None else first_stamp
                    last_stamp = stamp
                if tracker.observe(frame):
                    stop_reason = f"bad_streak_{bad_streak}"
                    break
                if (first_stamp is not None and last_stamp is not None
                        and (last_stamp - first_stamp) / 1e9 >= duration_seconds):
                    stop_reason = "duration_reached"
                    break
            if stop_reason != "wall_timeout":
                break
    finally:
        raw_stream.close()
        stderr_stream.close()
        returncode = _stop_process(process)

    summary = analyze_text(raw_path.read_text(encoding="utf-8"))
    if first_stamp is not None and last_stamp is not None:
        summary["captured_sim_duration_s"] = (last_stamp - first_stamp) / 1e9
    else:
        summary["captured_sim_duration_s"] = None
    summary["early_stop_streak"] = tracker.current_streak
    _write_json(Path(case_dir) / "gazebo_scan_summary.json", summary)
    return summary, stop_reason, returncode


def _case_metadata(case, duration, repeat, env, world, spawn_command):
    return {
        "case": {
            "label": case.label,
            "world_file": case.world_file,
            "carrier": case.carrier,
            "creation": case.creation,
            "topic": case.topic,
        },
        "duration_seconds": duration,
        "repeat": repeat,
        "world": str(world),
        "spawn_command": spawn_command,
        "environment": {
            name: env.get(name) for name in (
                "IGN_PARTITION", "GZ_PARTITION", "ROS_DOMAIN_ID", "ROS_LOG_DIR",
                "IGN_GAZEBO_RESOURCE_PATH", "GZ_SIM_RESOURCE_PATH",
            )
        },
        "raw_only": True,
        "excluded_processes": [
            "ros_gz_bridge", "Nav2", "RViz", "robot_state_publisher",
            "dynamic_obstacle_mover", "odom_slip_simulator",
        ],
    }


def run_case(case, output_root, duration_seconds, repeat, bad_streak,
             max_wall_seconds, run_number):
    """Run one isolated scene condition and preserve enough evidence to audit it."""
    case_dir = Path(output_root) / case.label / f"repeat_{repeat:02d}"
    case_dir.mkdir(parents=True, exist_ok=True)
    env = _isolated_environment(case_dir, run_number)
    server = None
    status = {"case": case.label, "repeat": repeat, "valid": False}
    try:
        world, payload = _prepare_case_world(case, case_dir, env)
        world_name = _world_name(world)
        server_command = gazebo_command(world)
        server = _start_logged(server_command, case_dir / "gazebo_server.log", env)
        ready, detail = _wait_for_listing(
            ["ign", "service", "-l"], f"/world/{world_name}/create", env)
        if not ready:
            raise RuntimeError(f"Gazebo world did not expose create service: {detail}")

        spawn_command = None
        if payload is not None:
            spawn_command = dynamic_spawn_command(case, payload, world_name)
            spawn = _start_logged(spawn_command, case_dir / "spawn.log", env)
            try:
                spawn_status = spawn.wait(timeout=30.0)
            except subprocess.TimeoutExpired:
                spawn_status = _stop_process(spawn)
                raise RuntimeError("dynamic spawn timed out")
            finally:
                if spawn.poll() is not None:
                    stream = getattr(spawn, "_matrix_log_stream", None)
                    if stream is not None:
                        stream.close()
            if spawn_status != 0:
                raise RuntimeError(f"dynamic spawn failed with status {spawn_status}")

        _write_json(
            case_dir / "metadata.json",
            _case_metadata(case, duration_seconds, repeat, env, world, spawn_command))
        ready, detail = _wait_for_listing(
            ["ign", "topic", "-l"], case.topic, env)
        if not ready:
            raise RuntimeError(f"raw scan topic did not appear: {detail}")
        topic_info, detail = _run_listing(
            ["ign", "topic", "-i", "-t", case.topic], env)
        publisher_count = publisher_count_from_topic_info(topic_info)
        if publisher_count != 1:
            raise RuntimeError(
                f"expected exactly one raw scan publisher, found {publisher_count}: "
                f"{detail}")
        summary, stop_reason, raw_status = _capture_raw_scan(
            case_dir, case.topic, env, duration_seconds, bad_streak,
            max_wall_seconds)
        valid, reasons = validate_capture_summary(
            summary, duration_seconds, stop_reason)
        status.update({
            "stop_reason": stop_reason,
            "raw_capture_status": raw_status,
            "raw_scan_publisher_count": publisher_count,
            "summary": summary,
            "valid": valid,
            "invalid_reasons": reasons,
        })
    except Exception as error:
        status["error"] = str(error)
    finally:
        status["gazebo_server_status"] = _stop_process(server)
        _write_json(case_dir / "status.json", status)
    return status


def _mean_or_none(values):
    values = [value for value in values if value is not None]
    return statistics.fmean(values) if values else None


def _min_or_none(values):
    values = [value for value in values if value is not None]
    return min(values) if values else None


def _max_or_none(values):
    values = [value for value in values if value is not None]
    return max(values) if values else None


def load_existing_statuses(output_root):
    """Load every durable repeat status beneath a matrix output directory."""
    rows = []
    for status_path in sorted(Path(output_root).glob("*/repeat_*/status.json")):
        try:
            row = json.loads(status_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            raise RuntimeError(f"cannot read repeat status {status_path}: {error}") from error
        if not isinstance(row, dict) or "case" not in row or "repeat" not in row:
            raise RuntimeError(f"malformed repeat status: {status_path}")
        rows.append(row)
    return rows


def summarize_matrix(rows):
    """Build a compact, auditable table from individual repeat statuses."""
    grouped = {}
    for row in rows:
        grouped.setdefault(row["case"], []).append(row)
    cases = {}
    for label, values in sorted(grouped.items()):
        summaries = [value.get("summary", {}) for value in values]
        cases[label] = {
            "repeat_count": len(values),
            "valid_repeat_count": sum(bool(value.get("valid")) for value in values),
            "stop_reasons": [value.get("stop_reason") for value in values],
            "first_all_negative_s_mean": _mean_or_none([
                summary.get("time_to_first_all_negative_s")
                for summary in summaries]),
            "all_negative_frame_count_max": max(
                (summary.get("all_negative_frame_count", 0) for summary in summaries),
                default=0),
            "good_to_bad_transition_count_max": max(
                (summary.get("good_to_bad_transition_count", 0) for summary in summaries),
                default=0),
            "bad_to_good_recovery_count_max": max(
                (summary.get("bad_to_good_recovery_count", 0) for summary in summaries),
                default=0),
            "longest_continuous_bad_frames_max": max(
                (summary.get("longest_continuous_bad_frames", 0) for summary in summaries),
                default=0),
            "finite_ratio_mean_min": _min_or_none([
                summary.get("finite_ratio_mean") for summary in summaries]),
            "negative_inf_ratio_mean_max": _max_or_none([
                summary.get("negative_inf_ratio_mean") for summary in summaries]),
            "negative_inf_ratio_max": _max_or_none([
                summary.get("negative_inf_ratio_max") for summary in summaries]),
        }
    return {
        "case_count": len(cases),
        "repeat_count": len(rows),
        "all_repeats_valid": all(row.get("valid") for row in rows),
        "cases": cases,
    }


def render_markdown(summary):
    lines = [
        "# Raw-only GPU-LiDAR scene matrix",
        "",
        "| case | valid | stops | first all-negative (s) | good→bad | bad→good | longest bad frames | finite mean min | -Inf mean max | -Inf max |",
        "|---|---:|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for label, value in summary["cases"].items():
        first_bad = value["first_all_negative_s_mean"]
        lines.append(
            f"| {label} | {value['valid_repeat_count']}/{value['repeat_count']} | "
            f"{', '.join(str(reason) for reason in value['stop_reasons'])} | "
            f"{first_bad if first_bad is not None else '-'} | "
            f"{value['good_to_bad_transition_count_max']} | "
            f"{value['bad_to_good_recovery_count_max']} | "
            f"{value['longest_continuous_bad_frames_max']} | "
            f"{value['finite_ratio_mean_min'] if value['finite_ratio_mean_min'] is not None else '-'} | "
            f"{value['negative_inf_ratio_mean_max'] if value['negative_inf_ratio_mean_max'] is not None else '-'} | "
            f"{value['negative_inf_ratio_max'] if value['negative_inf_ratio_max'] is not None else '-'} |")
    return "\n".join(lines) + "\n"


def main(argv=None):
    args = parse_args(argv)
    if args.duration <= 0.0 or args.repeats <= 0 or args.bad_streak <= 0:
        raise SystemExit("--duration, --repeats, and --bad-streak must be positive")
    cases = scene_matrix_cases()
    if args.labels:
        labels = set(args.labels)
        unknown = labels - {case.label for case in cases}
        if unknown:
            raise SystemExit("unknown case(s): " + ", ".join(sorted(unknown)))
        cases = tuple(case for case in cases if case.label in labels)
    output_root = args.output_root.resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    rows = []
    run_number = 0
    for case in cases:
        for repeat in range(1, args.repeats + 1):
            run_number += 1
            rows.append(run_case(
                case, output_root, args.duration, repeat, args.bad_streak,
                args.max_wall_seconds, run_number))
    all_rows = load_existing_statuses(output_root)
    _write_json(output_root / "matrix_status.json", all_rows)
    summary = summarize_matrix(all_rows)
    _write_json(output_root / "matrix_summary.json", summary)
    (output_root / "matrix_summary.md").write_text(
        render_markdown(summary), encoding="utf-8")
    return 0 if summary["all_repeats_valid"] else 1


def primitive_lidar_model_sdf():
    """Return the sensor-only dynamic-spawn control with the M1 ray contract."""
    return """<?xml version=\"1.0\"?>
<sdf version=\"1.9\">
  <model name=\"gpu_lidar_probe\">
    <static>true</static>
    <link name=\"link\">
      <sensor name=\"minimal_gpu_lidar\" type=\"gpu_lidar\">
        <topic>/scan</topic>
        <update_rate>12</update_rate>
        <always_on>true</always_on>
        <visualize>false</visualize>
        <ray>
          <scan>
            <horizontal>
              <samples>667</samples><resolution>1</resolution>
              <min_angle>-3.14159265359</min_angle>
              <max_angle>3.14159265359</max_angle>
            </horizontal>
            <vertical>
              <samples>1</samples><resolution>1</resolution>
              <min_angle>0</min_angle><max_angle>0</max_angle>
            </vertical>
          </scan>
          <range><min>0.05</min><max>12.0</max><resolution>0.01</resolution></range>
        </ray>
        <gz_frame_id>gpu_lidar_probe</gz_frame_id>
      </sensor>
    </link>
  </model>
</sdf>
"""


def _parse_sdf_tree(path):
    """Parse SDF output, repairing the prefix dropped by ``ign sdf -p``."""
    text = Path(path).read_text(encoding="utf-8")
    if "ignition:" in text and "xmlns:ignition=" not in text:
        start = text.find("<sdf")
        end = text.find(">", start)
        if start < 0 or end < 0:
            raise ValueError(f"{path} does not contain an SDF root element")
        text = (
            text[:end]
            + f' xmlns:ignition="{IGNITION_XML_NAMESPACE}"'
            + text[end:])
    return ET.ElementTree(ET.fromstring(text))


def write_world_loaded_model(base_world, converted_model, output):
    """Embed one preconverted M1 model in a copy of a base world."""
    base_tree = _parse_sdf_tree(base_world)
    model_tree = _parse_sdf_tree(converted_model)
    world = base_tree.getroot().find("world")
    model = model_tree.getroot().find("model")
    if world is None or model is None:
        raise ValueError("expected an SDF world and an SDF model")
    embedded = copy.deepcopy(model)
    embedded.set("name", "m1")
    pose = embedded.find("pose")
    if pose is None:
        pose = ET.Element("pose")
        embedded.insert(0, pose)
    pose.text = "-2.5 -1.5 0.01 0 0 0"
    world.append(embedded)
    ET.indent(base_tree, space="  ")
    base_tree.write(Path(output), encoding="utf-8", xml_declaration=True)


if __name__ == "__main__":
    raise SystemExit(main())
