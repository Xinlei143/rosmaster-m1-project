"""Measure sensor, TF, and local costmap freshness during M1 navigation.

The dynamic-obstacle topic consumed here is Gazebo ground truth for measuring
the sensor pipeline only.  It is deliberately never republished to, or read
by, Nav2.  The production navigation chain remains ``/scan`` -> ObstacleLayer.
"""

from collections import deque
from dataclasses import dataclass
import json
import math
import statistics
import time

import rclpy
from geometry_msgs.msg import PolygonStamped, PoseArray
from nav_msgs.msg import OccupancyGrid, Odometry
from rosgraph_msgs.msg import Clock as ClockMessage
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
import tf2_ros


def _percentile(values, percentile):
    """Return a linearly interpolated percentile, or None for no samples."""
    if not values:
        return None
    ordered = sorted(float(value) for value in values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * float(percentile) / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    fraction = position - lower
    return ordered[lower] + fraction * (ordered[upper] - ordered[lower])


def summarize_intervals(values):
    """Summarize durations in seconds with stable JSON-friendly values."""
    return {
        "p50": _percentile(values, 50.0),
        "p95": _percentile(values, 95.0),
        "max": max(values) if values else None,
    }


def scan_range_stats(ranges, range_max):
    """Classify LaserScan ranges without treating NaN as clear space."""
    values = list(ranges)
    finite = [float(value) for value in values if math.isfinite(value)]
    positive_inf = sum(
        1 for value in values if math.isinf(value) and value > 0.0)
    nan = sum(1 for value in values if math.isnan(value))
    total = len(values)
    denominator = float(total) if total else 1.0
    return {
        "finite_count": len(finite),
        "positive_inf_count": positive_inf,
        "nan_count": nan,
        "finite_ratio": len(finite) / denominator,
        "positive_inf_ratio": positive_inf / denominator,
        "nan_ratio": nan / denominator,
        "min_finite_range": min(finite) if finite else None,
        "range_max": float(range_max),
    }


def header_age_seconds(now_ns, stamp_ns):
    """Return receive ROS time minus the exact message header stamp."""
    return (int(now_ns) - int(stamp_ns)) / 1e9


def clock_jump_stats(stamps_ns):
    """Summarize /clock resets separately from ordinary forward gaps."""
    stamps = [int(stamp) for stamp in stamps_ns]
    backward = [
        (earlier - later) / 1e9
        for earlier, later in zip(stamps, stamps[1:])
        if later < earlier
    ]
    forward = [
        (later - earlier) / 1e9
        for earlier, later in zip(stamps, stamps[1:])
        if later >= earlier
    ]
    return {
        "sample_count": len(stamps),
        "backward_jump_count": len(backward),
        "max_backward_jump_seconds": max(backward) if backward else None,
        "forward_gap_p95_seconds": _percentile(forward, 95.0),
    }


def _distance(first, second):
    return math.hypot(first[0] - second[0], first[1] - second[1])


def match_dynamic_obstacle(center, lethal_cells, search_radius):
    """Match a cylinder by its surface cells, not only its center cell."""
    return any(
        _distance(center, cell) <= float(search_radius)
        for cell in lethal_cells)


def point_overlaps_costmap(point, bounds, margin):
    """Return whether a point's search disk overlaps an axis-aligned map.

    ``bounds`` is ``(min_x, min_y, max_x, max_y)`` in the map frame.
    """
    min_x, min_y, max_x, max_y = bounds
    margin = float(margin)
    return (min_x - margin <= point[0] <= max_x + margin and
            min_y - margin <= point[1] <= max_y + margin)


def _cell_present(cell, lethal_cells, tolerance=0.026):
    return any(
        _distance(cell, candidate) <= tolerance for candidate in lethal_cells)


def ghost_retention(
    old_cells,
    current_lethal_cells,
    old_center,
    new_center,
    elapsed_since_relocation,
    grace_seconds,
    current_obstacle_center=None,
    move_distance=0.5,
):
    """Return the fraction of old lethal cells retained after a move.

    ``None`` means the sample is not eligible: the obstacle has not moved far
    enough, the grace period has not elapsed, no old cells were observed, or a
    current obstacle occupies the old position.
    """
    if _distance(old_center, new_center) <= float(move_distance):
        return None
    if float(elapsed_since_relocation) < float(grace_seconds):
        return None
    if not old_cells:
        return None
    if (current_obstacle_center is not None and
            _distance(old_center, current_obstacle_center) <= 0.45):
        return None
    retained = sum(
        1 for cell in old_cells
        if _cell_present(cell, current_lethal_cells))
    return retained / float(len(old_cells))


@dataclass
class PendingScan:
    stamp_ns: int
    frame_id: str
    receive_mono_ns: int


class CostmapFreshnessDiagnostic(Node):
    """Collect freshness evidence without becoming part of navigation."""

    def __init__(self):
        super().__init__("m1_costmap_freshness_diagnostic")
        self.declare_parameter("duration_seconds", 60.0)
        self.declare_parameter("output", "")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("target_frame", "odom")
        self.declare_parameter(
            "dynamic_obstacles_topic", "/m1/dynamic_obstacles")
        self.declare_parameter("tf_timeout_seconds", 0.5)
        self.declare_parameter("ghost_grace_seconds", 0.4)
        self.declare_parameter("ghost_move_distance", 0.5)
        self.declare_parameter("obstacle_search_radius", 0.45)

        self.duration_seconds = float(
            self.get_parameter("duration_seconds").value)
        self.output_path = str(self.get_parameter("output").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.tf_timeout_seconds = float(
            self.get_parameter("tf_timeout_seconds").value)
        self.ghost_grace_seconds = float(
            self.get_parameter("ghost_grace_seconds").value)
        self.ghost_move_distance = float(
            self.get_parameter("ghost_move_distance").value)
        self.search_radius = float(
            self.get_parameter("obstacle_search_radius").value)
        self.started_mono_ns = time.monotonic_ns()
        self.finished = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        grid_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            LaserScan, self.get_parameter("scan_topic").value,
            self._scan_callback, sensor_qos)
        self.create_subscription(Odometry, "/odom", self._odom_callback, 10)
        self.create_subscription(
            ClockMessage, "/clock", self._clock_callback, 10)
        self.create_subscription(
            PolygonStamped, "/local_costmap/published_footprint",
            self._footprint_callback, 10)
        self.create_subscription(
            OccupancyGrid, "/local_costmap/costmap",
            self._costmap_callback, grid_qos)
        self.create_subscription(
            PoseArray, self.get_parameter("dynamic_obstacles_topic").value,
            self._dynamic_obstacles_callback, 10)

        self.pending_scans = deque()
        self.scan_receive_mono_ns = []
        self.scan_stamp_ns = []
        self.scan_header_ages = []
        self.scan_stats = []
        self.odom_receive_count = 0
        self.clock_stamp_ns = []
        self.footprint_receive_mono_ns = []
        self.costmap_receive_mono_ns = []
        self.tf_ready_latencies = []
        self.tf_success_count = 0
        self.tf_timeout_count = 0
        self.latest_costmap = None
        self.latest_costmap_bounds = None
        self.latest_lethal_cells = []
        self.dynamic_obstacle_samples = 0
        self.dynamic_obstacle_detected = 0
        self.dynamic_miss_started_ns = {}
        self.dynamic_miss_durations = []
        self.last_dynamic_centers = {}
        self.ghost_tracks = {}
        self.ghost_retention_samples = []
        self.ghost_clear_latencies = []

        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._timer = self.create_timer(
            0.04, self._timer_callback, clock=self._steady_clock)
        self.get_logger().info(
            "Freshness diagnostic started for %.1f s; ground truth is "
            "diagnostic-only." % self.duration_seconds)

    def _scan_callback(self, message):
        receive_mono_ns = time.monotonic_ns()
        now_ns = self.get_clock().now().nanoseconds
        stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec))
        self.scan_receive_mono_ns.append(receive_mono_ns)
        self.scan_stamp_ns.append(stamp_ns)
        self.scan_header_ages.append(header_age_seconds(now_ns, stamp_ns))
        self.scan_stats.append(
            scan_range_stats(message.ranges, message.range_max))
        self.pending_scans.append(PendingScan(
            stamp_ns=stamp_ns,
            frame_id=message.header.frame_id,
            receive_mono_ns=receive_mono_ns,
        ))

    def _odom_callback(self, _message):
        self.odom_receive_count += 1

    def _clock_callback(self, message):
        self.clock_stamp_ns.append(
            int(message.clock.sec) * 1_000_000_000
            + int(message.clock.nanosec))

    def _footprint_callback(self, _message):
        self.footprint_receive_mono_ns.append(time.monotonic_ns())

    @staticmethod
    def _grid_lethal_cells(message):
        width = int(message.info.width)
        resolution = float(message.info.resolution)
        origin_x = float(message.info.origin.position.x)
        origin_y = float(message.info.origin.position.y)
        cells = []
        for index, value in enumerate(message.data):
            if value < 100:
                continue
            x = index % width
            y = index // width
            cells.append((origin_x + (x + 0.5) * resolution,
                          origin_y + (y + 0.5) * resolution))
        return cells

    def _costmap_callback(self, message):
        self.costmap_receive_mono_ns.append(time.monotonic_ns())
        self.latest_costmap = message
        self.latest_costmap_bounds = (
            float(message.info.origin.position.x),
            float(message.info.origin.position.y),
            float(message.info.origin.position.x)
            + message.info.width * message.info.resolution,
            float(message.info.origin.position.y)
            + message.info.height * message.info.resolution,
        )
        self.latest_lethal_cells = self._grid_lethal_cells(message)

    def _dynamic_obstacles_callback(self, message):
        if not self.latest_costmap:
            return
        centers = [
            (pose.position.x, pose.position.y) for pose in message.poses]
        now_ns = time.monotonic_ns()
        current_cells = self.latest_lethal_cells
        for index, center in enumerate(centers):
            if not point_overlaps_costmap(
                    center, self.latest_costmap_bounds, self.search_radius):
                self.last_dynamic_centers[index] = center
                self.dynamic_miss_started_ns.pop(index, None)
                continue
            detected = match_dynamic_obstacle(
                center, current_cells, self.search_radius)
            self.dynamic_obstacle_samples += 1
            if detected:
                self.dynamic_obstacle_detected += 1
                started = self.dynamic_miss_started_ns.pop(index, None)
                if started is not None:
                    self.dynamic_miss_durations.append(
                        (now_ns - started) / 1e9)
            elif index not in self.dynamic_miss_started_ns:
                self.dynamic_miss_started_ns[index] = now_ns

            previous = self.last_dynamic_centers.get(index)
            track = self.ghost_tracks.get(index)
            if (previous is not None and
                    _distance(previous, center) > self.ghost_move_distance):
                old_cells = [
                    cell for cell in current_cells
                    if _distance(previous, cell) <= self.search_radius
                ]
                if old_cells:
                    self.ghost_tracks[index] = {
                        "old_cells": old_cells,
                        "old_center": previous,
                        "new_center": center,
                        "moved_ns": now_ns,
                        "reported": False,
                    }
                    track = self.ghost_tracks[index]
            if track is not None and not track["reported"]:
                elapsed = (now_ns - track["moved_ns"]) / 1e9
                retention = ghost_retention(
                    track["old_cells"], current_cells,
                    track["old_center"], track["new_center"],
                    elapsed, self.ghost_grace_seconds,
                    current_obstacle_center=center,
                    move_distance=self.ghost_move_distance,
                )
                if retention is not None:
                    self.ghost_retention_samples.append(retention)
                    if retention == 0.0:
                        self.ghost_clear_latencies.append(elapsed)
                        track["reported"] = True
            self.last_dynamic_centers[index] = center

    def _timer_callback(self):
        now_mono_ns = time.monotonic_ns()
        while self.pending_scans:
            pending = self.pending_scans[0]
            try:
                stamp = Time(
                    nanoseconds=pending.stamp_ns,
                    clock_type=ClockType.ROS_TIME)
                ready = self.tf_buffer.can_transform(
                    self.target_frame, pending.frame_id, stamp,
                    timeout=Duration(seconds=0.0))
            except (tf2_ros.TransformException, ValueError, TypeError):
                ready = False
            age = (now_mono_ns - pending.receive_mono_ns) / 1e9
            if ready:
                self.tf_ready_latencies.append(age)
                self.tf_success_count += 1
                self.pending_scans.popleft()
            elif age > self.tf_timeout_seconds:
                self.tf_timeout_count += 1
                self.pending_scans.popleft()
            else:
                break
        if ((now_mono_ns - self.started_mono_ns) / 1e9
                >= self.duration_seconds):
            self.finish()

    @staticmethod
    def _gap_values(times_ns):
        return [
            (later - earlier) / 1e9
            for earlier, later in zip(times_ns, times_ns[1:])
        ]

    @staticmethod
    def _rate(times_ns):
        if len(times_ns) < 2 or times_ns[-1] <= times_ns[0]:
            return 0.0
        return (len(times_ns) - 1) / ((times_ns[-1] - times_ns[0]) / 1e9)

    def _finish_pending_tf(self):
        self.tf_timeout_count += len(self.pending_scans)
        self.pending_scans.clear()

    def summary(self):
        self._finish_pending_tf()
        receive_gaps = summarize_intervals(
            self._gap_values(self.scan_receive_mono_ns))
        stamp_gaps = summarize_intervals([
            (later - earlier) / 1e9
            for earlier, later in zip(
                    self.scan_stamp_ns, self.scan_stamp_ns[1:])
        ])
        costmap_gaps = summarize_intervals(
            self._gap_values(self.costmap_receive_mono_ns))
        footprint_gaps = summarize_intervals(
            self._gap_values(self.footprint_receive_mono_ns))
        tf_total = self.tf_success_count + self.tf_timeout_count
        finite_ratio = statistics.fmean(
            sample["finite_ratio"] for sample in self.scan_stats
        ) if self.scan_stats else None
        positive_inf_ratio = statistics.fmean(
            sample["positive_inf_ratio"] for sample in self.scan_stats
        ) if self.scan_stats else None
        nan_ratio = statistics.fmean(
            sample["nan_ratio"] for sample in self.scan_stats
        ) if self.scan_stats else None
        min_ranges = [
            sample["min_finite_range"] for sample in self.scan_stats
            if sample["min_finite_range"] is not None
        ]
        miss_summary = summarize_intervals(self.dynamic_miss_durations)
        ghost_summary = summarize_intervals(self.ghost_clear_latencies)
        clock_summary = clock_jump_stats(self.clock_stamp_ns)
        return {
            "duration_seconds": (
                time.monotonic_ns() - self.started_mono_ns) / 1e9,
            "scan_count": len(self.scan_receive_mono_ns),
            "scan_rate_hz": self._rate(self.scan_receive_mono_ns),
            "scan_receive_gap_p50": receive_gaps["p50"],
            "scan_receive_gap_p95": receive_gaps["p95"],
            "scan_receive_gap_max": receive_gaps["max"],
            "scan_stamp_gap_p50": stamp_gaps["p50"],
            "scan_stamp_gap_p95": stamp_gaps["p95"],
            "scan_stamp_gap_max": stamp_gaps["max"],
            "scan_header_age_p50": _percentile(self.scan_header_ages, 50.0),
            "scan_header_age_p95": _percentile(self.scan_header_ages, 95.0),
            "scan_header_age_max": (
                max(self.scan_header_ages)
                if self.scan_header_ages else None),
            "finite_ratio_mean": finite_ratio,
            "positive_inf_ratio_mean": positive_inf_ratio,
            "nan_ratio_mean": nan_ratio,
            "min_finite_range_mean": (
                statistics.fmean(min_ranges) if min_ranges else None),
            "costmap_update_rate_hz": self._rate(
                self.footprint_receive_mono_ns),
            "costmap_publish_rate_hz": self._rate(
                self.costmap_receive_mono_ns),
            "costmap_publish_gap_p50": costmap_gaps["p50"],
            "costmap_publish_gap_p95": costmap_gaps["p95"],
            "costmap_publish_gap_max": costmap_gaps["max"],
            "footprint_update_rate_hz": self._rate(
                self.footprint_receive_mono_ns),
            "footprint_update_gap_p50": footprint_gaps["p50"],
            "footprint_update_gap_p95": footprint_gaps["p95"],
            "footprint_update_gap_max": footprint_gaps["max"],
            "odom_count": self.odom_receive_count,
            "clock_sample_count": clock_summary["sample_count"],
            "clock_backward_jump_count": clock_summary["backward_jump_count"],
            "clock_max_backward_jump_seconds": (
                clock_summary["max_backward_jump_seconds"]),
            "clock_forward_gap_p95_seconds": (
                clock_summary["forward_gap_p95_seconds"]),
            "tf_exact_stamp_success_count": self.tf_success_count,
            "tf_exact_stamp_success_ratio": (
                self.tf_success_count / float(tf_total) if tf_total else None),
            "tf_ready_latency_p50": _percentile(
                self.tf_ready_latencies, 50.0),
            "tf_ready_latency_p95": _percentile(
                self.tf_ready_latencies, 95.0),
            "tf_ready_latency_max": (
                max(self.tf_ready_latencies)
                if self.tf_ready_latencies else None),
            "tf_timeout_count": self.tf_timeout_count,
            "dynamic_obstacle_samples": self.dynamic_obstacle_samples,
            "dynamic_obstacle_detected_ratio": (
                self.dynamic_obstacle_detected
                / float(self.dynamic_obstacle_samples)
                if self.dynamic_obstacle_samples else None),
            "dynamic_obstacle_detection_miss_duration_p50": (
                miss_summary["p50"]),
            "dynamic_obstacle_detection_miss_duration_p95": (
                miss_summary["p95"]),
            "dynamic_obstacle_detection_miss_duration_max": (
                miss_summary["max"]),
            "ghost_cell_retention_ratio": (
                statistics.fmean(self.ghost_retention_samples)
                if self.ghost_retention_samples else None),
            "ghost_clear_latency_p50": ghost_summary["p50"],
            "ghost_clear_latency_p95": ghost_summary["p95"],
            "ghost_clear_latency_max": ghost_summary["max"],
        }

    def finish(self):
        if self.finished:
            return
        self.finished = True
        self._timer.cancel()
        result = self.summary()
        encoded = json.dumps(result, indent=2, sort_keys=True)
        if self.output_path:
            with open(self.output_path, "w", encoding="utf-8") as stream:
                stream.write(encoded)
                stream.write("\n")
        self.get_logger().info("Freshness diagnostic summary:\n%s" % encoded)


def main(args=None):
    rclpy.init(args=args)
    node = CostmapFreshnessDiagnostic()
    try:
        # spin_once makes the monotonic-duration completion deterministic;
        # rclpy.spin() can remain blocked while shutdown is requested from
        # the timer callback on some Humble executor versions.
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
