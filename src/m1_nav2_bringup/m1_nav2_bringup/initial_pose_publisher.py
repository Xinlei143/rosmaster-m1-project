"""Publish a deterministic initial pose while AMCL finishes starting."""

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.node import Node


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__("m1_initial_pose_publisher")
        self.declare_parameter("x", -2.5)
        self.declare_parameter("y", -1.5)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("publish_rate", 1.0)
        self.declare_parameter("publish_count", 15)
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10)
        self.remaining = int(self.get_parameter("publish_count").value)
        self.timer = self.create_timer(
            1.0 / float(self.get_parameter("publish_rate").value),
            self.publish_pose,
        )

    def publish_pose(self):
        if self.remaining <= 0:
            self.destroy_timer(self.timer)
            self.destroy_node()
            return
        yaw = float(self.get_parameter("yaw").value)
        message = PoseWithCovarianceStamped()
        # Use a slightly old simulated timestamp so the odom->base transform
        # is already in the TF buffer when AMCL handles the message.
        message.header.stamp = (
            self.get_clock().now() - Duration(seconds=0.50)).to_msg()
        message.header.frame_id = "map"
        message.pose.pose.position.x = float(self.get_parameter("x").value)
        message.pose.pose.position.y = float(self.get_parameter("y").value)
        message.pose.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(yaw / 2.0)
        message.pose.covariance[0] = 0.04
        message.pose.covariance[7] = 0.04
        message.pose.covariance[35] = 0.10
        self.publisher.publish(message)
        self.remaining -= 1


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
