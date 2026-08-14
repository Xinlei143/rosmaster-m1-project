#!/usr/bin/env python3
"""Plot and summarize a directory produced by ``experiment_logger``."""

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def read_rows(path):
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def values(rows, field):
    return np.asarray([float(row[field]) for row in rows], dtype=float)


def finite(values_array):
    return np.isfinite(values_array)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("log_dir", type=Path)
    args = parser.parse_args()
    log_dir = args.log_dir
    plot_dir = log_dir / "plots"
    plot_dir.mkdir(exist_ok=True)

    with (log_dir / "summary.json").open(encoding="utf-8") as handle:
        summary = json.load(handle)
    odom = read_rows(log_dir / "odom.csv")
    planner = read_rows(log_dir / "planner.csv")
    scans = read_rows(log_dir / "scan.csv")
    paths = read_rows(log_dir / "planned_path.csv")
    obstacles = read_rows(log_dir / "obstacles.csv")

    metrics = dict(summary)
    if odom:
        odom_t = values(odom, "stamp")
        # Gazebo can emit one large transient twist sample while the spawned
        # entity is inserted into the physics world. Keep that raw sample in
        # odom.csv, but exclude the first 0.5 s from performance metrics.
        analysis_mask = odom_t >= odom_t[0] + 0.5
        actual_speed = values(odom, "speed")[analysis_mask]
        actual_accel_x = values(odom, "accel_world_x")[analysis_mask]
        actual_accel_y = values(odom, "accel_world_y")[analysis_mask]
        metrics["metrics_exclude_initial_s"] = 0.5
        metrics["max_actual_speed"] = float(np.nanmax(np.abs(actual_speed)))
        metrics["max_actual_acceleration"] = float(
            np.nanmax(np.hypot(actual_accel_x, actual_accel_y)))
        if len(odom) > 1:
            dx = np.diff(values(odom, "position_x"))
            dy = np.diff(values(odom, "position_y"))
            metrics["odometry_path_length"] = float(np.sum(np.hypot(dx, dy)))
    if planner:
        planner_accel = np.hypot(values(planner, "planner_accel_x"), values(planner, "planner_accel_y"))
        command_speed = np.hypot(values(planner, "command_world_vx"), values(planner, "command_world_vy"))
        actual_speed = values(planner, "actual_speed")
        planner_t = values(planner, "stamp")
        # The first controller sample can race Gazebo entity insertion.  Keep
        # that raw sample in planner.csv, but use the same warm-up exclusion
        # as the odometry metrics when comparing command and actual speed.
        valid = (
            finite(command_speed)
            & finite(actual_speed)
            & (planner_t >= planner_t[0] + 0.5)
        )
        metrics["max_planner_acceleration"] = float(np.nanmax(planner_accel))
        metrics["max_planner_command_speed"] = float(np.nanmax(command_speed))
        if np.any(valid):
            metrics["planner_actual_speed_rmse"] = float(
                np.sqrt(np.mean((command_speed[valid] - actual_speed[valid]) ** 2)))
    if scans:
        scan_ids = sorted({int(row["scan_index"]) for row in scans})
        min_ranges = []
        valid_counts = []
        scan_times = []
        for scan_id in scan_ids:
            rows = [row for row in scans if int(row["scan_index"]) == scan_id]
            ranges = np.asarray([float(row["range"]) for row in rows], dtype=float)
            valid_values = np.asarray([int(row["valid"]) for row in rows], dtype=int)
            valid_ranges = ranges[valid_values.astype(bool)]
            min_ranges.append(float(np.min(valid_ranges)) if len(valid_ranges) else math.nan)
            valid_counts.append(int(np.sum(valid_values)))
            scan_times.append(float(rows[0]["stamp"]))
        metrics["scan_frame_count"] = len(scan_ids)
        metrics["scan_min_range"] = float(np.nanmin(min_ranges))
        metrics["scan_max_valid_beams"] = int(max(valid_counts))

    with (log_dir / "metrics.json").open("w", encoding="utf-8") as handle:
        json.dump(metrics, handle, ensure_ascii=False, indent=2, allow_nan=False)

    if odom:
        fig, ax = plt.subplots(figsize=(8, 6))
        odom_x, odom_y = values(odom, "position_x"), values(odom, "position_y")
        ax.plot(odom_x, odom_y, label="Actual trajectory", linewidth=2)
        if paths:
            path_groups = {}
            for row in paths:
                path_groups.setdefault(int(row["path_index"]), []).append(row)
            first = True
            for group in path_groups.values():
                group.sort(key=lambda row: int(row["step_index"]))
                ax.plot(
                    values(group, "position_x"), values(group, "position_y"),
                    color="limegreen", alpha=0.12, linewidth=0.8,
                    label="Planned rollout path" if first else None,
                )
                first = False
        goal = summary["goal"]
        ax.scatter([goal["x"]], [goal["y"]], marker="*", s=160, label="Goal")
        if obstacles:
            obstacle_x = values(obstacles, "position_x")
            obstacle_y = values(obstacles, "position_y")
            ax.scatter(obstacle_x, obstacle_y, s=8, alpha=0.25, label="Obstacle centers")
        ax.set_title("Actual trajectory and planned rollout")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.axis("equal")
        ax.grid(True)
        ax.legend()
        fig.tight_layout()
        fig.savefig(plot_dir / "trajectory.png", dpi=150)
        plt.close(fig)

    if planner:
        planner_t = values(planner, "stamp")
        # Match the performance metrics and hide the Gazebo entity-spawn
        # transient from the plot.  The raw transient remains in planner.csv
        # and odom.csv for debugging.
        warmup_mask = planner_t >= planner_t[0] + 0.5
        plot_t = planner_t[warmup_mask]
        fig, axes = plt.subplots(2, 2, figsize=(12, 8), sharex=True)
        panels = [
            (axes[0, 0], "command_world_vx", "actual_world_vx", "Velocity X [m/s]"),
            (axes[0, 1], "command_world_vy", "actual_world_vy", "Velocity Y [m/s]"),
            (axes[1, 0], "planner_accel_x", "actual_accel_world_x", "Acceleration X [m/s²]"),
            (axes[1, 1], "planner_accel_y", "actual_accel_world_y", "Acceleration Y [m/s²]"),
        ]
        for axis, planner_field, actual_field, ylabel in panels:
            axis.plot(plot_t, values(planner, planner_field)[warmup_mask], label="Planner")
            axis.plot(
                plot_t, values(planner, actual_field)[warmup_mask],
                label="Actual", alpha=0.45,
            )
            axis.set_ylabel(ylabel)
            axis.grid(True)
            axis.legend()
        axes[1, 0].set_xlabel("Time [s]")
        axes[1, 1].set_xlabel("Time [s]")
        fig.suptitle("Planner vs actual motion (after 0.5 s warm-up)")
        fig.tight_layout(rect=[0, 0, 1, 0.96])
        fig.savefig(plot_dir / "velocity_acceleration.png", dpi=150)
        plt.close(fig)

    if scans:
        fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
        axes[0].plot(scan_times, min_ranges)
        axes[0].set_ylabel("Minimum valid range [m]")
        axes[0].grid(True)
        axes[1].plot(scan_times, valid_counts)
        axes[1].set_ylabel("Valid beam count")
        axes[1].set_xlabel("Time [s]")
        axes[1].grid(True)
        fig.suptitle("LiDAR input quality")
        fig.tight_layout()
        fig.savefig(plot_dir / "scan_quality.png", dpi=150)
        plt.close(fig)

    print(f"Metrics written to {log_dir / 'metrics.json'}")
    print(f"Plots written to {plot_dir}")


if __name__ == "__main__":
    main()
