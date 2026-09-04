#!/usr/bin/env python3
"""Drive deterministic Nav2 waypoints while classifying every ROS scan."""

import json
import math
from pathlib import Path
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


WAYPOINTS = ((-0.8, -1.5, 0.0), (-2.5, -1.5, math.pi))
COMMAND_TOPICS = (
    "/cmd_vel_nav", "/cmd_vel_smoothed", "/m1/cmd_vel_raw", "/cmd_vel")


def scan_stats(message):
    values = [float(value) for value in message.ranges]
    return {
        "stamp_ns": (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)),
        "beam_count": len(values),
        "finite_count": sum(math.isfinite(value) for value in values),
        "positive_inf_count": sum(
            math.isinf(value) and value > 0.0 for value in values),
        "negative_inf_count": sum(
            math.isinf(value) and value < 0.0 for value in values),
        "nan_count": sum(math.isnan(value) for value in values),
    }


class WaypointSoak(Node):
    """Alternate between two known-free map goals for a fixed wall duration."""

    def __init__(self):
        super().__init__("gpu_lidar_waypoint_soak")
        self.declare_parameter("duration_seconds", 600.0)
        self.declare_parameter("goal_timeout_seconds", 90.0)
        self.declare_parameter("output", "")
        self.duration = float(self.get_parameter("duration_seconds").value)
        self.goal_timeout = float(
            self.get_parameter("goal_timeout_seconds").value)
        self.output = Path(str(self.get_parameter("output").value))

        self.action_client = ActionClient(
            self, NavigateToPose, "/navigate_to_pose")
        self.create_subscription(
            LaserScan, "/scan", self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 30)
        for topic in COMMAND_TOPICS:
            self.create_subscription(
                Twist, topic,
                lambda message, name=topic: self.command_callback(name, message),
                10)

        self.started = time.monotonic()
        self.finished = False
        self.frames = []
        self.goal_events = []
        self.goal_handle = None
        self.goal_started = None
        self.goal_index = 0
        self.next_goal_after = self.started
        self.previous_odom = None
        self.distance = 0.0
        self.odom_count = 0
        self.command_counts = {topic: 0 for topic in COMMAND_TOPICS}
        self.nonzero_command_counts = {topic: 0 for topic in COMMAND_TOPICS}
        self.timer = self.create_timer(0.1, self.tick)

    def scan_callback(self, message):
        self.frames.append(scan_stats(message))

    def odom_callback(self, message):
        point = (message.pose.pose.position.x, message.pose.pose.position.y)
        if self.previous_odom is not None:
            self.distance += math.hypot(
                point[0] - self.previous_odom[0],
                point[1] - self.previous_odom[1])
        self.previous_odom = point
        self.odom_count += 1

    def command_callback(self, topic, message):
        self.command_counts[topic] += 1
        if (abs(message.linear.x) > 1e-4 or abs(message.linear.y) > 1e-4
                or abs(message.angular.z) > 1e-4):
            self.nonzero_command_counts[topic] += 1

    def send_goal(self):
        if not self.action_client.server_is_ready():
            return
        waypoint = WAYPOINTS[self.goal_index]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = waypoint[0]
        goal.pose.pose.position.y = waypoint[1]
        goal.pose.pose.orientation.z = math.sin(waypoint[2] / 2.0)
        goal.pose.pose.orientation.w = math.cos(waypoint[2] / 2.0)
        event = {
            "waypoint_index": self.goal_index,
            "waypoint": list(waypoint),
            "sent_wall_seconds": time.monotonic() - self.started,
            "accepted": None,
            "status": "pending",
        }
        self.goal_events.append(event)
        self.goal_started = time.monotonic()
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(
            lambda result, entry=event: self.goal_response(result, entry))
        self.next_goal_after = float("inf")

    def goal_response(self, future, event):
        if self.finished:
            return
        try:
            handle = future.result()
        except Exception as error:  # pragma: no cover - middleware failure
            event.update(accepted=False, status=f"send_error:{error}")
            self.complete_goal(event)
            return
        event["accepted"] = bool(handle.accepted)
        if not handle.accepted:
            event["status"] = "rejected"
            self.complete_goal(event)
            return
        self.goal_handle = handle
        result = handle.get_result_async()
        result.add_done_callback(
            lambda finished, entry=event: self.goal_result(finished, entry))

    def goal_result(self, future, event):
        if self.finished:
            return
        try:
            status = int(future.result().status)
            event["status_code"] = status
            event["status"] = (
                "succeeded" if status == GoalStatus.STATUS_SUCCEEDED
                else f"status_{status}")
        except Exception as error:  # pragma: no cover - middleware failure
            event["status"] = f"result_error:{error}"
        self.complete_goal(event)

    def complete_goal(self, event=None):
        event = event or (self.goal_events[-1] if self.goal_events else None)
        if event is None or event.get("closed"):
            return
        event["closed"] = True
        event["finished_wall_seconds"] = time.monotonic() - self.started
        self.goal_handle = None
        self.goal_started = None
        self.goal_index = (self.goal_index + 1) % len(WAYPOINTS)
        self.next_goal_after = time.monotonic() + 1.0

    def tick(self):
        now = time.monotonic()
        if now - self.started >= self.duration:
            self.finish()
            return
        if (self.goal_handle is not None and self.goal_started is not None
                and now - self.goal_started >= self.goal_timeout):
            self.goal_events[-1]["status"] = "timeout_cancel_requested"
            self.goal_handle.cancel_goal_async()
            self.complete_goal(self.goal_events[-1])
            return
        if self.goal_started is None and now >= self.next_goal_after:
            self.send_goal()

    def finish(self):
        if self.finished:
            return
        self.finished = True
        self.timer.cancel()
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        summary = {
            "duration_seconds": time.monotonic() - self.started,
            "waypoints": [list(waypoint) for waypoint in WAYPOINTS],
            "goal_events": self.goal_events,
            "accepted_goal_count": sum(
                event.get("accepted") is True for event in self.goal_events),
            "succeeded_goal_count": sum(
                event.get("status") == "succeeded" for event in self.goal_events),
            "distance_m": self.distance,
            "odom_count": self.odom_count,
            "command_counts": self.command_counts,
            "nonzero_command_counts": self.nonzero_command_counts,
            "frames": self.frames,
        }
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.output.write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n",
            encoding="utf-8")


def main():
    rclpy.init()
    node = WaypointSoak()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        if not node.finished:
            node.finish()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
