"""Relay the software fallback LaserScan onto Nav2's stable /scan topic."""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


def dropout_active(start_mono, now_mono, start_seconds, duration_seconds):
    """Return whether the diagnostic scan dropout interval is active.

    The interval is relative to the relay's monotonic start time.  Negative
    starts and non-positive durations deliberately disable the gate, keeping
    the production/default path a transparent relay.
    """
    if start_seconds < 0.0 or duration_seconds <= 0.0:
        return False
    elapsed = now_mono - start_mono
    return start_seconds <= elapsed < start_seconds + duration_seconds


class ScanRelay(Node):
    def __init__(self):
        super().__init__("m1_scan_relay")
        self.declare_parameter("dropout_start_seconds", -1.0)
        self.declare_parameter("dropout_duration_seconds", 0.0)
        self.dropout_start_seconds = float(
            self.get_parameter("dropout_start_seconds").value)
        self.dropout_duration_seconds = float(
            self.get_parameter("dropout_duration_seconds").value)
        self.start_mono = time.monotonic()
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher = self.create_publisher(LaserScan, "/scan", qos)
        self.subscription = self.create_subscription(
            LaserScan, "/sim_scan", self.callback, qos)

    def callback(self, message):
        if dropout_active(
                self.start_mono, time.monotonic(),
                self.dropout_start_seconds,
                self.dropout_duration_seconds):
            return
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = ScanRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
