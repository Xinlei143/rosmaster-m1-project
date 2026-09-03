"""Standalone soak runner for the Fortress OGRE2 GPU-LiDAR readback path.

The runner deliberately has no ROS dependency in its execution path.  It
starts an Ignition Gazebo server, captures the raw Gazebo Transport topic, and
records lightweight GPU/process samples while the server runs.  A local
``ignition-rendering6`` prefix can be selected without changing the system
installation; the legacy readback path is selected with the same environment
switch used by the upstream backport.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
import io
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import time
from typing import Iterable, Mapping, Optional, Sequence


try:
    from . import gpu_lidar_probe as _probe
except ImportError:  # Source-file imports used by the package's pure tests.
    import importlib.util

    _probe_spec = importlib.util.spec_from_file_location(
        "m1_nav2_support_gpu_lidar_probe",
        Path(__file__).with_name("gpu_lidar_probe.py"),
    )
    _probe = importlib.util.module_from_spec(_probe_spec)
    _probe_spec.loader.exec_module(_probe)


DEFAULT_SAMPLES = 667
DEFAULT_UPDATE_RATE_HZ = 12.0
DEFAULT_TOPIC = "/minimal_scan"
READBACK_ENV = "GZ_RENDERING_OGRE2_LEGACY_READBACK"
RENDERING_LIBRARY_RE = re.compile(
    r"(?P<path>/[^\s]+libignition-rendering6\.so(?:\.[^\s/]+)*)")
OGRE2_PLUGIN_RE = re.compile(
    r"(?P<path>/[^\s]+libignition-rendering6-ogre2\.so(?:\.[^\s/]+)*)")


@dataclass(frozen=True)
class MinimalWorldCase:
    """One fixed SDF variant in the minimal 360°/180° experiment."""

    label: str
    world_filename: str
    min_angle: float
    max_angle: float
    samples: int = DEFAULT_SAMPLES
    update_rate_hz: float = DEFAULT_UPDATE_RATE_HZ
    topic: str = DEFAULT_TOPIC


def minimal_world_cases() -> tuple[MinimalWorldCase, ...]:
    """Return the ordered 360° then 180° cases used by the campaign."""
    return (
        MinimalWorldCase(
            "360deg",
            "gpu_lidar_minimal_360.sdf",
            -3.14159265359,
            3.14159265359,
        ),
        MinimalWorldCase(
            "180deg",
            "gpu_lidar_minimal_180.sdf",
            -1.570796326795,
            1.570796326795,
        ),
    )


def resolve_world_path(
    case: MinimalWorldCase,
    world_dir: Optional[Path] = None,
) -> Path:
    """Resolve a world from an explicit directory or the installed package."""
    candidates = []
    if world_dir is not None:
        candidates.append(Path(world_dir) / case.world_filename)
    env_dir = os.environ.get("M1_GPU_LIDAR_WORLD_DIR")
    if env_dir:
        candidates.append(Path(env_dir) / case.world_filename)
    candidates.append(Path(__file__).resolve().parents[1] / "worlds" /
                      case.world_filename)
    try:
        from ament_index_python.packages import get_package_share_directory

        candidates.append(
            Path(get_package_share_directory("m1_nav2_support")) /
            "worlds" / case.world_filename)
    except (ImportError, LookupError):
        pass
    for candidate in candidates:
        if candidate.exists():
            return candidate.resolve()
    attempted = "\n".join(str(candidate) for candidate in candidates)
    raise FileNotFoundError(
        f"cannot find {case.world_filename}; attempted:\n{attempted}")


def build_gazebo_command(
    world_path: Path,
    render_engine: str = "ogre2",
) -> list[str]:
    """Build a server-only, real-time Gazebo command."""
    return [
        "ign", "gazebo", "-s", "-r", "-v", "4",
        "--render-engine", render_engine, str(world_path),
    ]


def build_raw_capture_command(topic: str, duration: float) -> list[str]:
    """Capture one raw Gazebo Transport topic as JSON for ``duration``."""
    return [
        "ign", "topic", "-e", "--json-output", "-t", str(topic),
        "-d", f"{float(duration):.1f}",
    ]


def _prefix_library_dirs(prefix: Path) -> list[Path]:
    # CMake may select either GNUInstallDirs' multiarch directory or lib/;
    # keeping both in the loader path also makes the command portable across
    # a local CMake configuration without weakening the /proc verification.
    return [prefix / "lib", prefix / "lib" / "x86_64-linux-gnu"]


def _prefix_plugin_dirs(prefix: Path) -> list[Path]:
    return [
        prefix / "lib" / "x86_64-linux-gnu" / "ign-rendering-6" /
        "engine-plugins",
        prefix / "lib" / "ign-rendering-6" / "engine-plugins",
    ]


def build_local_environment(
    base_environment: Mapping[str, str],
    prefix: Path,
) -> dict[str, str]:
    """Return an environment that prefers a local rendering prefix."""
    env = dict(base_environment)
    prefix = Path(prefix).resolve()
    library_dirs = [str(path) for path in _prefix_library_dirs(prefix)]
    existing_library_path = env.get("LD_LIBRARY_PATH", "")
    if existing_library_path:
        library_dirs.extend(
            entry for entry in existing_library_path.split(os.pathsep)
            if entry)
    env["LD_LIBRARY_PATH"] = os.pathsep.join(library_dirs)
    env["IGN_RENDERING_PLUGIN_PATH"] = os.pathsep.join(
        str(path) for path in _prefix_plugin_dirs(prefix))
    # ``ign-rendering6`` installs the OGRE2 media tree below this directory
    # (``.../ignition-rendering6/ogre2/media``).  Do not synthesize the newer
    # ``gz-rendering-*`` locations here: pointing OGRE2 at those directories
    # makes it silently fall back to a path without the HLMS templates and
    # the renderer then aborts before the LiDAR sensor can publish a frame.
    resource_dirs = [
        prefix / "share" / "ignition" / "ignition-rendering6",
    ]
    existing_resource_path = env.get("IGN_RENDERING_RESOURCE_PATH", "")
    if existing_resource_path:
        resource_dirs.extend(
            Path(entry) for entry in existing_resource_path.split(os.pathsep)
            if entry)
    env["IGN_RENDERING_RESOURCE_PATH"] = os.pathsep.join(
        str(path) for path in resource_dirs)
    return env


def _unique(items: Iterable[str]) -> list[str]:
    return list(dict.fromkeys(items))


def validate_loaded_libraries(maps_text: str, prefix: Path) -> dict[str, object]:
    """Verify that both rendering libraries came from the requested prefix."""
    prefix_text = str(Path(prefix).resolve())
    rendering_paths = _unique(
        match.group("path") for match in RENDERING_LIBRARY_RE.finditer(maps_text))
    plugin_paths = _unique(
        match.group("path") for match in OGRE2_PLUGIN_RE.finditer(maps_text))
    all_paths = rendering_paths + plugin_paths
    system_paths = [
        path for path in all_paths
        if not (path == prefix_text or path.startswith(prefix_text + os.sep))
    ]
    return {
        "ok": bool(rendering_paths) and bool(plugin_paths) and not system_paths,
        "rendering_library_paths": rendering_paths,
        "ogre2_plugin_paths": plugin_paths,
        "system_library_paths": _unique(system_paths),
    }


def _strip_unit(value: str) -> float:
    cleaned = value.strip().replace(",", "")
    cleaned = re.sub(r"[^0-9+\-.eE]", "", cleaned)
    return float(cleaned)


def parse_resource_sample(
    nvidia_output: str,
    process_output: str,
) -> dict[str, object]:
    """Parse one nvidia-smi CSV row and one ``ps`` row for JSONL storage."""
    rows = list(csv.reader(io.StringIO(nvidia_output.strip())))
    rows = [[cell.strip() for cell in row] for row in rows if row]
    header = rows[0] if rows and any("name" in cell.lower() for cell in rows[0]) else None
    values = rows[1] if header is not None and len(rows) > 1 else (rows[0] if rows else [])
    if header is None:
        header = [
            "timestamp", "name", "driver_version", "utilization.gpu",
            "memory.used", "memory.total",
        ]
    by_name = {
        key.lower().replace(" [%]", "").replace(" [mib]", ""): values[index]
        for index, key in enumerate(header)
        if index < len(values)
    }
    utilization = by_name.get("utilization.gpu")
    memory_used = by_name.get("memory.used")
    memory_total = by_name.get("memory.total")
    process_fields = process_output.split()
    process: dict[str, object] = {}
    if len(process_fields) >= 4:
        try:
            process = {
                "pid": int(process_fields[0]),
                "ppid": int(process_fields[1]),
                "cpu_percent": float(process_fields[2]),
                "rss_kib": int(process_fields[3]),
                "command": " ".join(process_fields[4:]),
            }
        except ValueError:
            process = {"raw": process_output.strip()}
    gpu: dict[str, object] = {}
    if values:
        gpu = {
            "timestamp": by_name.get("timestamp"),
            "name": by_name.get("name"),
            "driver_version": by_name.get("driver_version"),
            "utilization_percent": _strip_unit(utilization)
            if utilization else None,
            "memory_used_mib": _strip_unit(memory_used) if memory_used else None,
            "memory_total_mib": _strip_unit(memory_total) if memory_total else None,
        }
    return {"timestamp": time.time(), "gpu": gpu, "process": process}


def sample_resource_once(gazebo_pid: Optional[int]) -> dict[str, object]:
    """Collect a single low-overhead GPU and Gazebo process sample."""
    nvidia = subprocess.run(
        [
            "nvidia-smi", "--query-gpu=timestamp,name,driver_version,"
            "utilization.gpu,memory.used,memory.total",
            "--format=csv,noheader,nounits",
        ], capture_output=True, text=True, check=False, timeout=5.0)
    process_output = ""
    if gazebo_pid is not None:
        process = subprocess.run(
            ["ps", "-p", str(gazebo_pid), "-o", "pid=,ppid=,pcpu=,rss=,comm="],
            capture_output=True, text=True, check=False, timeout=5.0)
        process_output = process.stdout.strip()
    sample = parse_resource_sample(nvidia.stdout, process_output)
    if nvidia.returncode != 0:
        sample["gpu_error"] = nvidia.stderr.strip() or f"exit {nvidia.returncode}"
    return sample


def summarize_soak_frames(frame_stats: Iterable[Mapping[str, object]]) -> dict[str, object]:
    """Summarize per-frame finite/non-finite counts without filtering values."""
    frames = list(frame_stats)
    if not frames:
        return {
            "frame_count": 0,
            "bad_frame_count": 0,
            "whole_frame_negative_inf_count": 0,
            "negative_inf_ratio_max": None,
            "positive_inf_ratio_max": None,
            "nan_ratio_max": None,
        }

    def count(frame: Mapping[str, object], name: str) -> int:
        return int(frame.get(name, 0) or 0)

    def ratio(frame: Mapping[str, object], name: str, beam_count: int) -> float:
        value = frame.get(name)
        if value is not None:
            return float(value)
        return count(frame, name.replace("_ratio", "_count")) / float(beam_count or 1)

    bad = 0
    whole_negative_inf = 0
    negative_ratios = []
    positive_ratios = []
    nan_ratios = []
    for frame in frames:
        beams = count(frame, "beam_count")
        negative_count = count(frame, "negative_inf_count")
        positive_count = count(frame, "positive_inf_count")
        nan_count = count(frame, "nan_count")
        if negative_count or positive_count or nan_count:
            bad += 1
        if beams and negative_count == beams:
            whole_negative_inf += 1
        negative_ratios.append(ratio(frame, "negative_inf_ratio", beams))
        positive_ratios.append(ratio(frame, "positive_inf_ratio", beams))
        nan_ratios.append(ratio(frame, "nan_ratio", beams))
    return {
        "frame_count": len(frames),
        "bad_frame_count": bad,
        "whole_frame_negative_inf_count": whole_negative_inf,
        "negative_inf_ratio_max": max(negative_ratios),
        "positive_inf_ratio_max": max(positive_ratios),
        "nan_ratio_max": max(nan_ratios),
    }


def _raw_frame_stats(text: str) -> list[dict[str, object]]:
    frame_stats = []
    for document in _probe.iter_json_documents(text):
        found = _probe._ranges_from_document(document)
        if found is None:
            continue
        ranges, range_max = found
        try:
            frame_stats.append(_probe.scan_range_stats(ranges, range_max))
        except (TypeError, ValueError):
            continue
    return frame_stats


def analyze_capture(path: Path) -> dict[str, object]:
    """Analyze raw JSONL and add whole-frame failure counts."""
    text = Path(path).read_text(encoding="utf-8", errors="replace")
    summary = _probe.analyze_text(text)
    summary["soak"] = summarize_soak_frames(_raw_frame_stats(text))
    return summary


def _run_capture(command: Sequence[str], output_path: Path, env: Mapping[str, str]):
    stream = open(output_path, "w", encoding="utf-8")
    try:
        process = subprocess.Popen(
            list(command), stdout=stream, stderr=subprocess.STDOUT,
            env=dict(env), start_new_session=True, text=True)
    except Exception:
        stream.close()
        raise
    process._gpu_lidar_log_stream = stream
    return process


def _stop_process(process, timeout: float = 8.0) -> Optional[int]:
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
    stream = getattr(process, "_gpu_lidar_log_stream", None)
    if stream is not None:
        stream.close()
    return process.returncode


def _command_output(command: Sequence[str]) -> dict[str, object]:
    try:
        result = subprocess.run(
            list(command), capture_output=True, text=True, check=False,
            timeout=15.0)
    except (FileNotFoundError, subprocess.TimeoutExpired) as error:
        return {"command": list(command), "error": str(error)}
    return {
        "command": list(command), "returncode": result.returncode,
        "stdout": result.stdout, "stderr": result.stderr,
    }


def _descendant_pids(root_pid: int) -> list[int]:
    result = {int(root_pid)}
    try:
        ps = subprocess.run(
            ["ps", "-eo", "pid=,ppid="], capture_output=True, text=True,
            check=False, timeout=5.0)
        pairs = []
        for line in ps.stdout.splitlines():
            fields = line.split()
            if len(fields) == 2:
                try:
                    pairs.append((int(fields[0]), int(fields[1])))
                except ValueError:
                    continue
        changed = True
        while changed:
            changed = False
            for pid, ppid in pairs:
                if ppid in result and pid not in result:
                    result.add(pid)
                    changed = True
    except (FileNotFoundError, subprocess.TimeoutExpired):
        pass
    return sorted(result)


def _capture_maps(root_pid: Optional[int]) -> str:
    if root_pid is None:
        return ""
    chunks = []
    for pid in _descendant_pids(root_pid):
        path = Path("/proc") / str(pid) / "maps"
        try:
            chunks.append(f"# pid {pid}\n" + path.read_text(encoding="utf-8"))
        except (FileNotFoundError, PermissionError):
            continue
    return "\n".join(chunks)


def _copy_ogre_logs(
    case_dir: Path,
    started_at: float,
    env: Optional[Mapping[str, str]] = None,
) -> list[str]:
    candidates = []
    if env and env.get("IGN_LOG_PATH"):
        log_root = Path(env["IGN_LOG_PATH"])
        candidates.extend((log_root / "rendering" / "ogre2.log",
                           log_root / "rendering" / "ogre.log"))
    candidates.extend((
        Path.home() / ".ignition" / "rendering" / "ogre2.log",
        Path.home() / ".ignition" / "rendering" / "ogre.log",
    ))
    copied = []
    for source in _unique(str(path) for path in candidates):
        source = Path(source)
        if source.exists() and source.stat().st_mtime >= started_at:
            destination = case_dir / source.name
            shutil.copy2(source, destination)
            copied.append(str(destination))
    return copied


def _write_metadata(
    case_dir: Path,
    case: MinimalWorldCase,
    world_path: Path,
    duration: float,
    repeat: int,
    mode: str,
    prefix: Optional[Path],
    env: Mapping[str, str],
) -> None:
    metadata = {
        "case": asdict(case),
        "world_path": str(world_path),
        "duration_seconds": duration,
        "repeat": repeat,
        "mode": mode,
        "readback_environment": (
            "1" if mode == "legacy" else "0"),
        "local_prefix": str(prefix) if prefix else None,
        "commands": {
            "gazebo": build_gazebo_command(world_path),
            "raw_capture": build_raw_capture_command(case.topic, duration),
        },
        "environment": {
            "variables": {
                name: env.get(name)
                for name in (
                    "DISPLAY", "WAYLAND_DISPLAY", "LD_LIBRARY_PATH",
                    "IGN_RENDERING_PLUGIN_PATH", "IGN_RENDERING_RESOURCE_PATH",
                    "IGN_LOG_PATH", READBACK_ENV,
                )
            },
            "uname": _command_output(["uname", "-a"]),
            "systemd_detect_virt": _command_output(["systemd-detect-virt"]),
            "nvidia_smi": _command_output(["nvidia-smi"]),
            "glxinfo": _command_output(["glxinfo", "-B"]),
        },
    }
    (case_dir / "metadata.json").write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def run_case(
    case: MinimalWorldCase,
    output_root: Path,
    duration: float,
    repeat: int,
    mode: str,
    prefix: Optional[Path] = None,
    world_dir: Optional[Path] = None,
    startup_grace: float = 2.0,
) -> dict[str, object]:
    """Run one isolated soak and preserve artifacts even on failure."""
    if mode not in {"legacy", "persistent"}:
        raise ValueError(f"unknown readback mode: {mode}")
    case_dir = Path(output_root) / mode / case.label / f"repeat_{repeat:02d}"
    case_dir.mkdir(parents=True, exist_ok=True)
    (case_dir / "ignition_log").mkdir(exist_ok=True)
    world_path = resolve_world_path(case, world_dir)
    env = build_local_environment(os.environ, prefix) if prefix else os.environ.copy()
    env[READBACK_ENV] = "1" if mode == "legacy" else "0"
    env["IGN_LOG_PATH"] = str(case_dir / "ignition_log")
    _write_metadata(case_dir, case, world_path, duration, repeat, mode, prefix, env)

    launch = raw = None
    statuses: dict[str, object] = {"mode": mode, "repeat": repeat}
    started_at = time.time()
    maps_text = ""
    try:
        launch = _run_capture(
            build_gazebo_command(world_path), case_dir / "gazebo.log", env)
        statuses["gazebo_pid"] = launch.pid
        time.sleep(max(0.0, float(startup_grace)))
        raw = _run_capture(
            build_raw_capture_command(case.topic, duration),
            case_dir / "gazebo_scan.jsonlog", env)
        next_sample = time.monotonic()
        deadline = time.monotonic() + float(duration) + 30.0
        with open(case_dir / "resources.jsonl", "w", encoding="utf-8") as resources:
            while time.monotonic() < deadline:
                now = time.monotonic()
                if now >= next_sample:
                    resources.write(json.dumps(
                        sample_resource_once(launch.pid), sort_keys=True) + "\n")
                    resources.flush()
                    next_sample = now + 1.0
                if raw.poll() is not None:
                    break
                time.sleep(min(0.25, max(0.01, next_sample - time.monotonic())))
        maps_text = _capture_maps(launch.pid)
        if maps_text:
            (case_dir / "process_maps.txt").write_text(maps_text, encoding="utf-8")
        statuses["raw_capture"] = _stop_process(raw)
        statuses["gazebo"] = _stop_process(launch)
    except (FileNotFoundError, OSError, subprocess.TimeoutExpired) as error:
        statuses["runner_error"] = str(error)
    finally:
        statuses["raw_capture"] = _stop_process(raw) if raw is not None else statuses.get("raw_capture")
        statuses["gazebo"] = _stop_process(launch) if launch is not None else statuses.get("gazebo")
        statuses["ogre_logs"] = _copy_ogre_logs(case_dir, started_at, env)

    raw_path = case_dir / "gazebo_scan.jsonlog"
    if raw_path.exists():
        try:
            summary = analyze_capture(raw_path)
            (case_dir / "gazebo_scan_summary.json").write_text(
                json.dumps(summary, indent=2, sort_keys=True) + "\n",
                encoding="utf-8")
            # A syntactically valid but empty capture is not a successful
            # sensor run.  This matters when Gazebo crashes during renderer
            # initialization: the analyzer can still produce a JSON summary
            # with frame_count=0, which must fail the campaign.
            if int(summary.get("frame_count", 0) or 0) > 0:
                statuses["raw_analyzer"] = 0
            else:
                statuses["raw_analyzer"] = 1
                statuses["raw_analyzer_error"] = (
                    "capture contained no valid scan frames")
        except (OSError, ValueError, json.JSONDecodeError) as error:
            statuses["raw_analyzer"] = 1
            statuses["raw_analyzer_error"] = str(error)
    else:
        statuses["raw_analyzer"] = 1

    if prefix is not None:
        if not maps_text:
            maps_text = (case_dir / "process_maps.txt").read_text(
                encoding="utf-8") if (case_dir / "process_maps.txt").exists() else ""
        statuses["library_validation"] = validate_loaded_libraries(
            maps_text, prefix)
    else:
        statuses["library_validation"] = {"required": False, "ok": None}
    (case_dir / "status.json").write_text(
        json.dumps(statuses, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return statuses


def _selected_cases(labels: Optional[Sequence[str]]) -> tuple[MinimalWorldCase, ...]:
    cases = minimal_world_cases()
    if not labels:
        return cases
    known = {case.label for case in cases}
    unknown = set(labels) - known
    if unknown:
        raise ValueError("unknown case(s): " + ", ".join(sorted(unknown)))
    return tuple(case for case in cases if case.label in labels)


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path,
                        default=Path("results/gpu_lidar_readback"))
    parser.add_argument("--duration", type=float, default=300.0)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--mode", choices=("legacy", "persistent"),
                        default="persistent")
    parser.add_argument("--prefix", type=Path,
                        help="local ignition-rendering6 install prefix")
    parser.add_argument("--world-dir", type=Path)
    parser.add_argument("--case", action="append", dest="labels")
    parser.add_argument("--startup-grace", type=float, default=2.0)
    args = parser.parse_args(argv)
    if args.duration <= 0.0 or args.repeats <= 0:
        parser.error("--duration and --repeats must be positive")
    try:
        cases = _selected_cases(args.labels)
    except ValueError as error:
        parser.error(str(error))
    output_root = args.output_root.resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    rows = []
    for case in cases:
        for repeat in range(1, args.repeats + 1):
            started = time.time()
            statuses = run_case(
                case, output_root, args.duration, repeat, args.mode,
                prefix=args.prefix, world_dir=args.world_dir,
                startup_grace=args.startup_grace)
            rows.append({
                "case": case.label, "repeat": repeat,
                "mode": args.mode, "elapsed_seconds": time.time() - started,
                "statuses": statuses,
            })
    (output_root / f"{args.mode}_status.json").write_text(
        json.dumps(rows, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0 if all(
        row["statuses"].get("raw_analyzer") == 0
        and row["statuses"].get("runner_error") is None
        and (args.prefix is None or row["statuses"].get(
            "library_validation", {}).get("ok") is True)
        for row in rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
