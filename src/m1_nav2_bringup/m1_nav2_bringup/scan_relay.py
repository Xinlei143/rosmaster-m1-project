"""Relay the software fallback LaserScan onto Nav2's stable /scan topic."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class ScanRelay(Node):
    def __init__(self):
        super().__init__("m1_scan_relay")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher = self.create_publisher(LaserScan, "/scan", qos)
        self.subscription = self.create_subscription(
            LaserScan, "/sim_scan", self.callback, qos)

    def callback(self, message):
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
