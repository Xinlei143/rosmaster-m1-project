"""Capture unmodified ROS LaserScan classifications as JSON lines."""

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

from .gpu_lidar_probe import scan_range_stats


class ScanCapture(Node):
    """Write one classification record for every received scan."""

    def __init__(self, topics, output_root):
        super().__init__("gpu_lidar_ros_capture")
        self._streams = {}
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        for topic in topics:
            label = topic.strip("/").replace("/", "_") or "scan"
            stream = open(output_root / f"ros_{label}.jsonl", "w", encoding="utf-8")
            self._streams[topic] = stream
            self.create_subscription(
                LaserScan, topic,
                lambda message, topic=topic: self._scan(topic, message), qos)

    def _scan(self, topic, message):
        record = scan_range_stats(message.ranges, message.range_max)
        record.update({
            "stamp_ns": (
                int(message.header.stamp.sec) * 1_000_000_000
                + int(message.header.stamp.nanosec)),
            "frame_id": message.header.frame_id,
            "angle_min": float(message.angle_min),
            "angle_max": float(message.angle_max),
            "angle_increment": float(message.angle_increment),
            "range_min": float(message.range_min),
            "range_max": float(message.range_max),
        })
        stream = self._streams[topic]
        stream.write(json.dumps(record, sort_keys=True) + "\n")
        stream.flush()

    def close(self):
        for stream in self._streams.values():
            stream.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--output-root", type=str, required=True)
    parser.add_argument("--topics", nargs="+", required=True)
    args, ros_args = parser.parse_known_args(argv)
    if args.duration <= 0:
        parser.error("duration must be positive")

    from pathlib import Path
    output_root = Path(args.output_root)
    output_root.mkdir(parents=True, exist_ok=True)
    rclpy.init(args=ros_args)
    node = ScanCapture(args.topics, output_root)
    deadline = time.monotonic() + args.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
