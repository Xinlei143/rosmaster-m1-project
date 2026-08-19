"""Repeatable diagnostics for M1 straight-line and Nav2 goal behavior.

The physics mode deliberately publishes directly to ``/cmd_vel`` and does
not depend on Nav2.  The navigation mode sends one NavigateToPose goal while
recording every command stage, odometry, AMCL pose and the planner path.  The
result is printed as JSON and can optionally be written to a file for CI or
regression comparisons.
"""

import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from action_msgs.msg import GoalStatus


def wrap_angle(angle):
    """Return an angle in [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_yaw(orientation):
    """Extract planar yaw from a ROS quaternion."""
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )


def relative_displacement(start, current):
    """Express current planar displacement in the initial heading frame."""
    x0, y0, yaw0 = start
    x, y, _ = current
    dx, dy = x - x0, y - y0
    cosine, sine = math.cos(yaw0), math.sin(yaw0)
    return cosine * dx + sine * dy, -sine * dx + cosine * dy


def percentile(values, fraction):
    """Compute a small dependency-free percentile for diagnostic values."""
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(round(fraction * (len(ordered) - 1)))))
    return ordered[index]


def trajectory_summary(samples):
    """Summarize (time, forward, lateral, yaw, x, y) samples."""
    if not samples:
        return {
            "samples": 0,
            "forward_distance_m": 0.0,
            "lateral_final_m": 0.0,
            "lateral_max_abs_m": 0.0,
            "yaw_final_rad": 0.0,
            "yaw_max_abs_rad": 0.0,
        }
    return {
        "samples": len(samples),
        "forward_distance_m": samples[-1][1],
        "lateral_final_m": samples[-1][2],
        "lateral_max_abs_m": max(abs(sample[2]) for sample in samples),
        "yaw_final_rad": samples[-1][3],
        "yaw_max_abs_rad": max(abs(sample[3]) for sample in samples),
        "final_pose": {"x": samples[-1][4], "y": samples[-1][5]},
    }


def command_summary(commands):
    """Summarize absolute velocity components captured by each topic."""
    result = {}
    for topic, values in commands.items():
        if not values:
            result[topic] = {"samples": 0}
            continue
        result[topic] = {
            "samples": len(values),
            "abs_x_p95": percentile([abs(value[0]) for value in values], 0.95),
            "abs_y_p95": percentile([abs(value[1]) for value in values], 0.95),
            "abs_wz_p95": percentile([abs(value[2]) for value in values], 0.95),
            "nonzero_samples": sum(
                1 for x, y, wz in values if max(abs(x), abs(y), abs(wz)) > 0.005
            ),
        }
    return result


def command_veto_summary(events):
    """Estimate periods where Collision Monitor suppresses a live command."""
    ordered_events = sorted(events)
    latest_smoothed = None
    veto_started = None
    veto_periods = []
    for timestamp, topic, values in ordered_events:
        if topic == "cmd_vel_smoothed":
            latest_smoothed = values
        elif topic == "m1_cmd_vel_raw":
            smooth_nonzero = latest_smoothed is not None and max(
                abs(value) for value in latest_smoothed
            ) > 0.005
            raw_zero = max(abs(value) for value in values) <= 0.005
            if smooth_nonzero and raw_zero and veto_started is None:
                veto_started = timestamp
            elif not (smooth_nonzero and raw_zero) and veto_started is not None:
                veto_periods.append(timestamp - veto_started)
                veto_started = None
    if veto_started is not None and ordered_events:
        veto_periods.append(ordered_events[-1][0] - veto_started)
    return {
        "veto_periods": len(veto_periods),
        "veto_duration_s": sum(veto_periods),
        "max_veto_duration_s": max(veto_periods, default=0.0),
    }


def footprint_collision_free(costmap, x, y, yaw, half_length=0.17, half_width=0.14):
    """Check a padded rectangular footprint against lethal/unknown cells."""
    if costmap is None or not costmap.data:
        return None
    resolution = costmap.info.resolution
    origin_x = costmap.info.origin.position.x
    origin_y = costmap.info.origin.position.y
    width, height = costmap.info.width, costmap.info.height
    cosine, sine = math.cos(yaw), math.sin(yaw)
    step = max(resolution / 2.0, 0.01)
    longitudinal = [
        -half_length + index * step
        for index in range(int(2.0 * half_length / step) + 1)
    ]
    lateral = [
        -half_width + index * step
        for index in range(int(2.0 * half_width / step) + 1)
    ]
    for local_x in longitudinal:
        for local_y in lateral:
            world_x = x + cosine * local_x - sine * local_y
            world_y = y + sine * local_x + cosine * local_y
            cell_x = int(math.floor((world_x - origin_x) / resolution))
            cell_y = int(math.floor((world_y - origin_y) / resolution))
            if cell_x < 0 or cell_y < 0 or cell_x >= width or cell_y >= height:
                return False
            value = int(costmap.data[cell_y * width + cell_x])
            if value < 0 or value >= 100:
                return False
    return True


def goal_preflight(costmap, goal, approach_yaw=0.0, sweep_steps=12):
    """Check terminal footprint and the final rotation swept footprint."""
    if costmap is None:
        return {"available": False}
    x, y, yaw = goal
    final_free = footprint_collision_free(costmap, x, y, yaw)
    sweep_free = final_free
    for index in range(1, sweep_steps + 1):
        fraction = index / float(sweep_steps)
        intermediate_yaw = approach_yaw + fraction * wrap_angle(yaw - approach_yaw)
        if footprint_collision_free(costmap, x, y, intermediate_yaw) is not True:
            sweep_free = False
            break
    return {
        "available": True,
        "final_footprint_free": final_free,
        "rotation_sweep_free": sweep_free,
    }


class MotionDiagnostic(Node):
    """Run one deterministic physics or Nav2 diagnostic experiment."""

    COMMAND_TOPICS = {
        "cmd_vel_nav": "/cmd_vel_nav",
        "cmd_vel_smoothed": "/cmd_vel_smoothed",
        "m1_cmd_vel_raw": "/m1/cmd_vel_raw",
        "cmd_vel": "/cmd_vel",
    }

    def __init__(self):
        super().__init__("m1_motion_diagnostic")
        self.declare_parameter("mode", "physics_straight")
        self.declare_parameter("warmup_seconds", 2.0)
        self.declare_parameter("duration_seconds", 10.0)
        self.declare_parameter("command_x", 0.20)
        self.declare_parameter("command_y", 0.0)
        self.declare_parameter("command_wz", 0.0)
        self.declare_parameter("goal_x", 0.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_yaw", 0.0)
        self.declare_parameter("approach_yaw", 0.0)
        self.declare_parameter("rotation_sweep_steps", 12)
        self.declare_parameter("goal_timeout_seconds", 60.0)
        self.declare_parameter("output", "")

        self.mode = str(self.get_parameter("mode").value)
        if self.mode not in {"physics_straight", "nav_goal"}:
            raise ValueError("mode must be physics_straight or nav_goal")
        self.warmup = float(self.get_parameter("warmup_seconds").value)
        self.duration = float(self.get_parameter("duration_seconds").value)
        self.command = (
            float(self.get_parameter("command_x").value),
            float(self.get_parameter("command_y").value),
            float(self.get_parameter("command_wz").value),
        )
        self.goal = (
            float(self.get_parameter("goal_x").value),
            float(self.get_parameter("goal_y").value),
            float(self.get_parameter("goal_yaw").value),
        )
        self.approach_yaw = float(self.get_parameter("approach_yaw").value)
        self.rotation_sweep_steps = int(self.get_parameter("rotation_sweep_steps").value)
        self.goal_timeout = float(self.get_parameter("goal_timeout_seconds").value)
        self.output = str(self.get_parameter("output").value)

        self.cmd_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 30)
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.amcl_callback, 10
        )
        self.create_subscription(NavPath, "/plan", self.plan_callback, 10)
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap", self.costmap_callback, 10)
        self.create_subscription(
            LaserScan, "/scan", self.scan_callback, qos_profile_sensor_data
        )
        self.commands = {name: [] for name in self.COMMAND_TOPICS}
        self.command_events = []
        for name, topic in self.COMMAND_TOPICS.items():
            self.create_subscription(
                Twist, topic, lambda message, key=name: self.command_callback(key, message), 10
            )

        self.action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.started_at = time.monotonic()
        self.experiment_started_at = None
        self.goal_sent = False
        self.goal_result = None
        self.start_pose = None
        self.samples = []
        self.amcl_samples = []
        self.plan_end = None
        self.scan_min = None
        self.global_costmap = None
        self.goal_preflight = {"available": False}
        self.finished = False
        self.timer = self.create_timer(0.05, self.tick)

    def now_elapsed(self):
        return time.monotonic() - self.started_at

    def odom_callback(self, message):
        pose = message.pose.pose
        current = (pose.position.x, pose.position.y, quaternion_yaw(pose.orientation))
        if self.start_pose is None:
            self.start_pose = current
        if self.experiment_started_at is not None:
            forward, lateral = relative_displacement(self.start_pose, current)
            self.samples.append(
                (
                    time.monotonic() - self.experiment_started_at,
                    forward,
                    lateral,
                    wrap_angle(current[2] - self.start_pose[2]),
                    current[0],
                    current[1],
                )
            )

    def amcl_callback(self, message):
        pose = message.pose.pose
        self.amcl_samples.append(
            (pose.position.x, pose.position.y, quaternion_yaw(pose.orientation))
        )

    def command_callback(self, topic, message):
        if self.experiment_started_at is None:
            return
        values = (message.linear.x, message.linear.y, message.angular.z)
        self.commands[topic].append(values)
        self.command_events.append((time.monotonic(), topic, values))

    def plan_callback(self, message):
        if message.poses:
            pose = message.poses[-1].pose
            self.plan_end = {
                "x": pose.position.x,
                "y": pose.position.y,
                "yaw": quaternion_yaw(pose.orientation),
                "frame_id": message.header.frame_id,
            }

    def scan_callback(self, message):
        finite = [value for value in message.ranges if math.isfinite(value)]
        if finite:
            self.scan_min = min(finite)

    def costmap_callback(self, message):
        self.global_costmap = message

    def make_twist(self, values):
        message = Twist()
        message.linear.x, message.linear.y, message.angular.z = values
        return message

    def send_nav_goal(self):
        if self.goal_sent:
            return
        self.goal_sent = True
        self.goal_preflight = goal_preflight(
            self.global_costmap,
            self.goal,
            approach_yaw=self.approach_yaw,
            sweep_steps=self.rotation_sweep_steps,
        )
        if not self.action_client.wait_for_server(timeout_sec=0.5):
            self.goal_result = "action_server_unavailable"
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.goal[0]
        goal.pose.pose.position.y = self.goal[1]
        goal.pose.pose.orientation.z = math.sin(self.goal[2] / 2.0)
        goal.pose.pose.orientation.w = math.cos(self.goal[2] / 2.0)
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            handle = future.result()
        except Exception as error:  # pragma: no cover - middleware failure
            self.goal_result = "send_error:%s" % error
            return
        if not handle.accepted:
            self.goal_result = "rejected"
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        try:
            status = future.result().status
            self.goal_result = "succeeded" if status == GoalStatus.STATUS_SUCCEEDED else "status_%d" % status
        except Exception as error:  # pragma: no cover - middleware failure
            self.goal_result = "result_error:%s" % error

    def tick(self):
        if self.finished or self.start_pose is None:
            return
        elapsed = self.now_elapsed()
        if self.experiment_started_at is None and elapsed >= self.warmup:
            self.experiment_started_at = time.monotonic()

        if self.mode == "physics_straight":
            if self.experiment_started_at is None:
                self.cmd_publisher.publish(self.make_twist((0.0, 0.0, 0.0)))
            elif elapsed < self.warmup + self.duration:
                self.cmd_publisher.publish(self.make_twist(self.command))
            else:
                self.cmd_publisher.publish(self.make_twist((0.0, 0.0, 0.0)))
                self.finish()
            return

        if self.experiment_started_at is not None and not self.goal_sent:
            self.send_nav_goal()
        if self.goal_result is not None:
            self.finish()
        elif self.experiment_started_at is not None and elapsed >= self.warmup + self.goal_timeout:
            self.goal_result = "timeout"
            self.finish()

    def finish(self):
        if self.finished:
            return
        self.finished = True
        self.timer.cancel()
        summary = trajectory_summary(self.samples)
        summary.update(
            {
                "mode": self.mode,
                "command": {
                    "x": self.command[0],
                    "y": self.command[1],
                    "wz": self.command[2],
                },
                "goal": {
                    "x": self.goal[0],
                    "y": self.goal[1],
                    "yaw": self.goal[2],
                },
                "goal_result": self.goal_result,
                "goal_preflight": self.goal_preflight,
                "command_topics": command_summary(self.commands),
                "command_veto": command_veto_summary(self.command_events),
                "plan_end": self.plan_end,
                "scan_min_m": self.scan_min,
                "amcl_samples": len(self.amcl_samples),
            }
        )
        text = json.dumps(summary, sort_keys=True)
        print(text, flush=True)
        if self.output:
            Path(self.output).parent.mkdir(parents=True, exist_ok=True)
            Path(self.output).write_text(text + "\n")
        self.create_timer(0.1, self.shutdown_timer)

    def shutdown_timer(self):
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MotionDiagnostic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
