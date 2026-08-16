"""Record commanded and measured motion for repeatable avoidance benchmarks."""

import csv
import json
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseArray, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

from imperative_navigation.scene_profiles import get_scene_profile


def yaw_from_orientation(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def body_to_world(x, y, yaw):
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return cosine * x - sine * y, sine * x + cosine * y


def vector_delta(current, previous, dt):
    if previous is None or dt <= 1e-6:
        return math.nan, math.nan
    return ((current[0] - previous[0]) / dt,
            (current[1] - previous[1]) / dt)


def finite_values(values):
    return [value for value in values if math.isfinite(value)]


class AvoidancePerformanceRecorder(Node):
    """Write raw samples plus goal, tracking, acceleration, and clearance metrics."""

    COMMAND_FIELDS = (
        "stamp", "body_vx", "body_vy", "world_vx", "world_vy", "speed",
        "body_ax", "body_ay", "world_ax", "world_ay", "acceleration",
    )
    ACTUAL_FIELDS = (
        "stamp", "position_x", "position_y", "yaw", "body_vx", "body_vy",
        "world_vx", "world_vy", "speed", "body_ax", "body_ay", "world_ax",
        "world_ay", "acceleration", "command_world_vx", "command_world_vy",
        "velocity_error", "static_clearance", "dynamic_clearance", "wall_clearance",
        "lidar_clearance", "minimum_clearance", "goal_distance",
    )
    OBSTACLE_FIELDS = ("stamp", "obstacle_index", "position_x", "position_y", "radius")

    def __init__(self):
        super().__init__("avoidance_performance_recorder")
        self.declare_parameter("scenario", "static")
        self.declare_parameter("scene", "imperative_m1")
        self.declare_parameter("output_dir", "/tmp/imperative_m1_performance")
        self.declare_parameter("timeout", 60.0)
        self.declare_parameter("goal_x", 2.5)
        self.declare_parameter("goal_y", 1.5)
        self.declare_parameter("goal_tolerance", 0.20)
        self.declare_parameter("stop_speed", 0.10)
        self.declare_parameter("robot_radius", 0.15)
        self.declare_parameter("safety_margin", 0.15)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("dynamic_obstacle_radii", Parameter.Type.DOUBLE_ARRAY)

        self.scenario = str(self.get_parameter("scenario").value)
        self.scene = str(self.get_parameter("scene").value).strip().lower()
        self.profile = get_scene_profile(self.scene)
        self.timeout = float(self.get_parameter("timeout").value)
        self.goal = (float(self.get_parameter("goal_x").value),
                     float(self.get_parameter("goal_y").value))
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.stop_speed = float(self.get_parameter("stop_speed").value)
        self.robot_radius = float(self.get_parameter("robot_radius").value)
        self.safety_margin = float(self.get_parameter("safety_margin").value)
        self.dynamic_radii = [float(value) for value in
                              self.get_parameter("dynamic_obstacle_radii").value]
        self.output_dir = Path(str(self.get_parameter("output_dir").value))
        self.output_dir.mkdir(parents=True, exist_ok=True)
        if any(self.output_dir.iterdir()):
            raise RuntimeError(f"Benchmark output directory is not empty: {self.output_dir}")

        self.command_handle, self.command_writer = self.open_csv(
            "command_motion.csv", self.COMMAND_FIELDS)
        self.actual_handle, self.actual_writer = self.open_csv(
            "actual_motion.csv", self.ACTUAL_FIELDS)
        self.obstacle_handle, self.obstacle_writer = self.open_csv(
            "obstacle_motion.csv", self.OBSTACLE_FIELDS)
        self.command_rows = []
        self.actual_rows = []
        self.obstacle_rows = []
        self.dynamic_centers = []
        self.latest_scan_min_range = math.inf
        self.latest_command = (0.0, 0.0)
        self.previous_command = None
        self.previous_command_stamp = None
        self.previous_actual_body = None
        self.previous_actual_world = None
        self.previous_actual_stamp = None
        self.measurement_start = None
        self.start_position = None
        self.previous_position = None
        self.path_length = 0.0
        self.finished = False

        self.create_subscription(Twist, "/cmd_vel", self.command_callback, 50)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 100)
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, self.get_parameter("scan_topic").value,
                                 self.scan_callback, sensor_qos)
        self.create_subscription(PoseArray, "/imperative/dynamic_obstacles",
                                 self.obstacle_callback, 20)
        self.create_timer(0.1, self.check_timeout)
        self.get_logger().info(
            f"Recording {self.scenario} avoidance performance in {self.output_dir}")

    def open_csv(self, filename, fields):
        handle = (self.output_dir / filename).open("w", newline="", encoding="utf-8")
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        return handle, writer

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def scan_callback(self, message):
        valid = [float(value) for value in message.ranges
                 if math.isfinite(value) and message.range_min <= value <= message.range_max]
        self.latest_scan_min_range = min(valid) if valid else math.inf

    def obstacle_callback(self, message):
        stamp = (float(message.header.stamp.sec) +
                 1e-9 * float(message.header.stamp.nanosec))
        self.dynamic_centers = [
            (float(pose.position.x), float(pose.position.y)) for pose in message.poses]
        for index, (x, y) in enumerate(self.dynamic_centers):
            radius = (self.dynamic_radii[index]
                      if index < len(self.dynamic_radii) else 0.20)
            row = {"stamp": stamp, "obstacle_index": index,
                   "position_x": x, "position_y": y, "radius": radius}
            self.obstacle_writer.writerow(row)
            self.obstacle_rows.append(row)
        self.obstacle_handle.flush()

    def command_callback(self, message):
        stamp = self.now_seconds()
        body = (float(message.linear.x), float(message.linear.y))
        self.latest_command = body
        if self.measurement_start is None:
            self.previous_command = body
            self.previous_command_stamp = stamp
            return
        dt = stamp - self.previous_command_stamp if self.previous_command_stamp is not None else 0.0
        body_acceleration = vector_delta(body, self.previous_command, dt)
        yaw = self.actual_rows[-1]["yaw"] if self.actual_rows else 0.0
        world = body_to_world(*body, yaw)
        previous_world = (self.command_rows[-1]["world_vx"],
                          self.command_rows[-1]["world_vy"]) if self.command_rows else None
        world_acceleration = vector_delta(world, previous_world, dt)
        row = {
            "stamp": stamp, "body_vx": body[0], "body_vy": body[1],
            "world_vx": world[0], "world_vy": world[1], "speed": math.hypot(*body),
            "body_ax": body_acceleration[0], "body_ay": body_acceleration[1],
            "world_ax": world_acceleration[0], "world_ay": world_acceleration[1],
            "acceleration": math.hypot(*body_acceleration),
        }
        self.command_writer.writerow(row)
        self.command_handle.flush()
        self.command_rows.append(row)
        self.previous_command = body
        self.previous_command_stamp = stamp

    def clearances(self, position):
        static_clearance = math.inf
        for primitive in self.profile.get("static_primitives", []):
            if primitive["shape"] == "circle":
                clearance = (math.hypot(position[0] - primitive["center"][0],
                                         position[1] - primitive["center"][1])
                             - primitive["radius"] - self.robot_radius)
            else:
                half_x, half_y = (size / 2.0 for size in primitive["size"])
                clearance = (max(abs(position[0] - primitive["center"][0]) - half_x,
                                 abs(position[1] - primitive["center"][1]) - half_y,
                                 0.0) - self.robot_radius)
            static_clearance = min(static_clearance, clearance)
        dynamic_clearance = (min(
            math.hypot(position[0] - x, position[1] - y) - self.robot_radius -
            (self.dynamic_radii[index] if index < len(self.dynamic_radii) else 0.20)
            for index, (x, y) in enumerate(self.dynamic_centers)
        ) if self.dynamic_centers else math.inf)
        xmin, xmax, ymin, ymax = self.profile["bounds"]
        wall_clearance = min(position[0] - xmin, xmax - position[0],
                             position[1] - ymin, ymax - position[1]) - self.robot_radius
        lidar_clearance = self.latest_scan_min_range - self.robot_radius
        return static_clearance, dynamic_clearance, wall_clearance, lidar_clearance

    def odom_callback(self, message):
        stamp = float(message.header.stamp.sec) + 1e-9 * float(message.header.stamp.nanosec)
        pose = message.pose.pose
        position = (float(pose.position.x), float(pose.position.y))
        yaw = yaw_from_orientation(pose.orientation)
        body = (float(message.twist.twist.linear.x),
                float(message.twist.twist.linear.y))
        if math.hypot(*body) > 5.0:
            return
        if self.measurement_start is None:
            self.measurement_start = stamp
        world = body_to_world(*body, yaw)
        dt = stamp - self.previous_actual_stamp if self.previous_actual_stamp is not None else 0.0
        body_acceleration = vector_delta(body, self.previous_actual_body, dt)
        world_acceleration = vector_delta(world, self.previous_actual_world, dt)
        command_world = body_to_world(*self.latest_command, yaw)
        static_clearance, dynamic_clearance, wall_clearance, lidar_clearance = self.clearances(position)
        minimum_clearance = min(static_clearance, dynamic_clearance, wall_clearance, lidar_clearance)
        goal_distance = math.hypot(position[0] - self.goal[0], position[1] - self.goal[1])
        row = {
            "stamp": stamp, "position_x": position[0], "position_y": position[1],
            "yaw": yaw, "body_vx": body[0], "body_vy": body[1],
            "world_vx": world[0], "world_vy": world[1], "speed": math.hypot(*body),
            "body_ax": body_acceleration[0], "body_ay": body_acceleration[1],
            "world_ax": world_acceleration[0], "world_ay": world_acceleration[1],
            "acceleration": math.hypot(*body_acceleration),
            "command_world_vx": command_world[0], "command_world_vy": command_world[1],
            "velocity_error": math.hypot(world[0] - command_world[0],
                                         world[1] - command_world[1]),
            "static_clearance": static_clearance,
            "dynamic_clearance": dynamic_clearance,
            "wall_clearance": wall_clearance,
            "lidar_clearance": lidar_clearance,
            "minimum_clearance": minimum_clearance,
            "goal_distance": goal_distance,
        }
        self.actual_writer.writerow(row)
        self.actual_handle.flush()
        self.actual_rows.append(row)
        if self.start_position is None:
            self.start_position = position
        if self.previous_position is not None:
            self.path_length += math.hypot(position[0] - self.previous_position[0],
                                           position[1] - self.previous_position[1])
        self.previous_position = position
        self.previous_actual_body = body
        self.previous_actual_world = world
        self.previous_actual_stamp = stamp
        if goal_distance <= self.goal_tolerance and row["speed"] <= self.stop_speed:
            self.finish("goal_reached")
            raise SystemExit(0)

    def check_timeout(self):
        if (not self.finished and self.measurement_start is not None and
                self.now_seconds() - self.measurement_start >= self.timeout):
            self.finish("timeout")
            raise SystemExit(0)

    @staticmethod
    def rms(values):
        values = finite_values(values)
        return math.sqrt(sum(value * value for value in values) / len(values)) if values else None

    @staticmethod
    def maximum(rows, key):
        values = finite_values([row[key] for row in rows])
        return max(values) if values else None

    @staticmethod
    def percentile(rows, key, fraction):
        values = sorted(finite_values([row[key] for row in rows]))
        if not values:
            return None
        index = min(len(values) - 1, max(0, round(fraction * (len(values) - 1))))
        return values[index]

    def finish(self, reason):
        if self.finished:
            return
        self.finished = True
        last = self.actual_rows[-1] if self.actual_rows else None
        elapsed = ((last["stamp"] - self.actual_rows[0]["stamp"])
                   if len(self.actual_rows) >= 2 else 0.0)
        clearance_values = finite_values([row["minimum_clearance"] for row in self.actual_rows])
        static_values = finite_values([row["static_clearance"] for row in self.actual_rows])
        dynamic_values = finite_values([row["dynamic_clearance"] for row in self.actual_rows])
        lidar_values = finite_values([row["lidar_clearance"] for row in self.actual_rows])
        summary = {
            "scenario": self.scenario,
            "termination_reason": reason,
            "goal_reached": reason == "goal_reached",
            "duration_s": elapsed,
            "samples": {"command": len(self.command_rows), "actual": len(self.actual_rows)},
            "start_position": self.start_position,
            "final_position": ([last["position_x"], last["position_y"]] if last else None),
            "final_goal_distance_m": (last["goal_distance"] if last else None),
            "path_length_m": self.path_length,
            "minimum_clearance_m": min(clearance_values) if clearance_values else None,
            "minimum_static_clearance_m": min(static_values) if static_values else None,
            "minimum_dynamic_clearance_m": min(dynamic_values) if dynamic_values else None,
            "minimum_lidar_clearance_m": min(lidar_values) if lidar_values else None,
            "geometric_collision": any(value <= 0.0 for value in clearance_values),
            "safety_margin_m": self.safety_margin,
            "safety_margin_violation": any(
                value < self.safety_margin for value in clearance_values),
            "command_peak_speed_mps": self.maximum(self.command_rows, "speed"),
            "command_peak_acceleration_mps2": self.maximum(self.command_rows, "acceleration"),
            "command_p95_acceleration_mps2": self.percentile(
                self.command_rows, "acceleration", 0.95),
            "actual_peak_speed_mps": self.maximum(self.actual_rows, "speed"),
            "actual_peak_acceleration_mps2": self.maximum(self.actual_rows, "acceleration"),
            "actual_p95_acceleration_mps2": self.percentile(
                self.actual_rows, "acceleration", 0.95),
            "velocity_tracking_rmse_mps": self.rms(
                [row["velocity_error"] for row in self.actual_rows]),
        }
        with (self.output_dir / "summary.json").open("w", encoding="utf-8") as handle:
            json.dump(summary, handle, indent=2, ensure_ascii=False)
        self.get_logger().info(
            "Benchmark finished: scenario=%s reason=%s goal_distance=%s min_clearance=%s" % (
                self.scenario, reason,
                "n/a" if last is None else f"{last['goal_distance']:.3f} m",
                "n/a" if not clearance_values else f"{min(clearance_values):.3f} m"))
        self.command_handle.close()
        self.actual_handle.close()
        self.obstacle_handle.close()


def main(args=None):
    rclpy.init(args=args)
    node = AvoidancePerformanceRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if not node.finished:
            node.finish("interrupted")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
