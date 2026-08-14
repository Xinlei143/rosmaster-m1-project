"""Record a synchronized Gazebo planner experiment to CSV files.

The logger intentionally records the raw streams separately.  Planner rows
contain the exact acceleration selected by the controller, while odometry is
recorded at its native rate so actual acceleration can be estimated without
pretending that it is directly measured by ``nav_msgs/Odometry``.
"""

import csv
import json
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseArray, Twist
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray

from imperative_navigation.debug_protocol import PLANNER_DEBUG_FIELDS


def stamp_to_seconds(stamp):
    value = float(stamp.sec) + 1e-9 * float(stamp.nanosec)
    return value if value > 0.0 else None


def quaternion_to_yaw(orientation):
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )


def finite_or_none(value):
    value = float(value)
    return value if math.isfinite(value) else None


class ExperimentLogger(Node):
    """Persist planner, command, odometry, path, obstacle, and scan streams."""

    def __init__(self):
        super().__init__("experiment_logger")
        self.declare_parameter("output_dir", "/tmp/imperative_m1_experiment")
        self.declare_parameter("run_name", "gazebo")
        self.declare_parameter("goal_x", 2.5)
        self.declare_parameter("goal_y", 1.5)
        self.declare_parameter("goal_tolerance", 0.2)
        self.declare_parameter("scan_topic", "/sim_scan")
        self.declare_parameter("command_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("planner_debug_topic", "/imperative/planner_debug")
        self.declare_parameter("planned_path_topic", "/imperative/planned_path")
        self.declare_parameter("obstacle_topic", "/imperative/dynamic_obstacles")

        self.output_dir = Path(str(self.get_parameter("output_dir").value)).expanduser()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        existing_files = list(self.output_dir.iterdir())
        if existing_files:
            raise RuntimeError(
                f"Experiment output directory is not empty: {self.output_dir}. "
                "Use a new log_dir for each run.")

        self.goal = (
            float(self.get_parameter("goal_x").value),
            float(self.get_parameter("goal_y").value),
        )
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.start_odom = None
        self.latest_odom = None
        self.final_odom = None
        self.last_odom_time = None
        self.start_time = None
        self.end_time = None
        self.scan_index = 0
        self.command_index = 0
        self.path_index = 0
        self.obstacle_index = 0
        self.closed = False
        self.row_counts = {"planner": 0, "odom": 0, "cmd_vel": 0,
                           "scan": 0, "planned_path": 0, "obstacles": 0}

        self.files = {}
        self.writers = {}
        self._open_csv(
            "planner",
            list(PLANNER_DEBUG_FIELDS) + [
                "odom_stamp", "odom_age", "actual_position_x", "actual_position_y",
                "actual_yaw", "actual_body_vx", "actual_body_vy", "actual_world_vx",
                "actual_world_vy", "actual_angular_z", "actual_accel_body_x",
                "actual_accel_body_y", "actual_accel_world_x", "actual_accel_world_y",
                "actual_speed",
            ],
        )
        self._open_csv(
            "odom",
            [
                "stamp", "position_x", "position_y", "yaw", "body_vx", "body_vy",
                "world_vx", "world_vy", "angular_z", "accel_body_x", "accel_body_y",
                "accel_world_x", "accel_world_y", "speed",
            ],
        )
        self._open_csv(
            "cmd_vel",
            ["sample_index", "stamp", "body_vx", "body_vy", "body_vz", "angular_z"],
        )
        self._open_csv(
            "scan",
            [
                "scan_index", "stamp", "frame_id", "angle_min", "angle_increment",
                "range_min", "range_max", "beam_index", "angle", "range", "valid",
            ],
        )
        self._open_csv(
            "planned_path",
            ["path_index", "stamp", "frame_id", "step_index", "position_x", "position_y"],
        )
        self._open_csv(
            "obstacles",
            ["message_index", "stamp", "frame_id", "obstacle_index", "position_x", "position_y", "position_z"],
        )

        sensor_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self.odom_callback, 20)
        self.create_subscription(
            LaserScan, self.get_parameter("scan_topic").value, self.scan_callback, sensor_qos)
        self.create_subscription(
            Twist, self.get_parameter("command_topic").value, self.command_callback, 20)
        self.create_subscription(
            Float64MultiArray, self.get_parameter("planner_debug_topic").value,
            self.planner_debug_callback, 20)
        self.create_subscription(
            NavPath, self.get_parameter("planned_path_topic").value,
            self.planned_path_callback, 20)
        self.create_subscription(
            PoseArray, self.get_parameter("obstacle_topic").value,
            self.obstacle_callback, 20)
        self.get_logger().info(f"Recording experiment data in {self.output_dir}")

    def _open_csv(self, name, fields):
        path = self.output_dir / f"{name}.csv"
        handle = path.open("w", newline="", encoding="utf-8")
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        self.files[name] = handle
        self.writers[name] = writer

    def _event_time(self, message=None):
        stamp = getattr(getattr(message, "header", None), "stamp", None)
        value = stamp_to_seconds(stamp) if stamp is not None else None
        if value is None:
            now = self.get_clock().now().nanoseconds
            value = float(now) * 1e-9
        if self.start_time is None:
            self.start_time = value
        self.end_time = value
        return value

    def _write(self, name, row):
        self.writers[name].writerow(row)
        self.files[name].flush()
        self.row_counts[name] += 1

    def odom_callback(self, message):
        stamp = self._event_time(message)
        pose = message.pose.pose
        yaw = quaternion_to_yaw(pose.orientation)
        body_vx = float(message.twist.twist.linear.x)
        body_vy = float(message.twist.twist.linear.y)
        angular_z = float(message.twist.twist.angular.z)
        cosine, sine = math.cos(yaw), math.sin(yaw)
        world_vx = cosine * body_vx - sine * body_vy
        world_vy = sine * body_vx + cosine * body_vy

        accel_body_x = accel_body_y = accel_world_x = accel_world_y = math.nan
        if self.latest_odom is not None:
            dt = stamp - self.latest_odom["stamp"]
            if dt > 1e-6:
                accel_body_x = (body_vx - self.latest_odom["body_vx"]) / dt
                accel_body_y = (body_vy - self.latest_odom["body_vy"]) / dt
                accel_world_x = (world_vx - self.latest_odom["world_vx"]) / dt
                accel_world_y = (world_vy - self.latest_odom["world_vy"]) / dt

        current = {
            "stamp": stamp,
            "position_x": float(pose.position.x),
            "position_y": float(pose.position.y),
            "yaw": yaw,
            "body_vx": body_vx,
            "body_vy": body_vy,
            "world_vx": world_vx,
            "world_vy": world_vy,
            "angular_z": angular_z,
            "accel_body_x": accel_body_x,
            "accel_body_y": accel_body_y,
            "accel_world_x": accel_world_x,
            "accel_world_y": accel_world_y,
            "speed": math.hypot(world_vx, world_vy),
        }
        if self.start_odom is None:
            self.start_odom = current.copy()
        self.final_odom = current.copy()
        self.latest_odom = current
        self.last_odom_time = stamp
        self._write("odom", current)

    def scan_callback(self, message):
        stamp = self._event_time(message)
        scan_index = self.scan_index
        self.scan_index += 1
        frame_id = message.header.frame_id
        for index, value in enumerate(message.ranges):
            value = float(value)
            valid = math.isfinite(value) and message.range_min <= value <= message.range_max
            self._write("scan", {
                "scan_index": scan_index,
                "stamp": stamp,
                "frame_id": frame_id,
                "angle_min": float(message.angle_min),
                "angle_increment": float(message.angle_increment),
                "range_min": float(message.range_min),
                "range_max": float(message.range_max),
                "beam_index": index,
                "angle": float(message.angle_min + index * message.angle_increment),
                "range": value,
                "valid": int(valid),
            })

    def command_callback(self, message):
        stamp = self._event_time()
        self._write("cmd_vel", {
            "sample_index": self.command_index,
            "stamp": stamp,
            "body_vx": float(message.linear.x),
            "body_vy": float(message.linear.y),
            "body_vz": float(message.linear.z),
            "angular_z": float(message.angular.z),
        })
        self.command_index += 1

    def planner_debug_callback(self, message):
        if len(message.data) != len(PLANNER_DEBUG_FIELDS):
            self.get_logger().error(
                f"Planner debug field count {len(message.data)} != {len(PLANNER_DEBUG_FIELDS)}")
            return
        row = dict(zip(PLANNER_DEBUG_FIELDS, (float(value) for value in message.data)))
        if self.latest_odom is None:
            odom_fields = {
                "odom_stamp": math.nan, "odom_age": math.nan,
                "actual_position_x": math.nan, "actual_position_y": math.nan,
                "actual_yaw": math.nan, "actual_body_vx": math.nan,
                "actual_body_vy": math.nan, "actual_world_vx": math.nan,
                "actual_world_vy": math.nan, "actual_angular_z": math.nan,
                "actual_accel_body_x": math.nan, "actual_accel_body_y": math.nan,
                "actual_accel_world_x": math.nan, "actual_accel_world_y": math.nan,
                "actual_speed": math.nan,
            }
        else:
            odom = self.latest_odom
            odom_fields = {
                "odom_stamp": odom["stamp"],
                "odom_age": float(row["stamp"] - odom["stamp"]),
                "actual_position_x": odom["position_x"],
                "actual_position_y": odom["position_y"],
                "actual_yaw": odom["yaw"],
                "actual_body_vx": odom["body_vx"],
                "actual_body_vy": odom["body_vy"],
                "actual_world_vx": odom["world_vx"],
                "actual_world_vy": odom["world_vy"],
                "actual_angular_z": odom["angular_z"],
                "actual_accel_body_x": odom["accel_body_x"],
                "actual_accel_body_y": odom["accel_body_y"],
                "actual_accel_world_x": odom["accel_world_x"],
                "actual_accel_world_y": odom["accel_world_y"],
                "actual_speed": odom["speed"],
            }
        row.update(odom_fields)
        self._write("planner", row)

    def planned_path_callback(self, message):
        stamp = stamp_to_seconds(message.header.stamp)
        if stamp is None:
            stamp = self._event_time()
        for step_index, pose in enumerate(message.poses):
            self._write("planned_path", {
                "path_index": self.path_index,
                "stamp": stamp,
                "frame_id": message.header.frame_id,
                "step_index": step_index,
                "position_x": float(pose.pose.position.x),
                "position_y": float(pose.pose.position.y),
            })
        self.path_index += 1

    def obstacle_callback(self, message):
        stamp = stamp_to_seconds(message.header.stamp)
        if stamp is None:
            stamp = self._event_time()
        message_index = self.obstacle_index
        self.obstacle_index += 1
        for index, pose in enumerate(message.poses):
            self._write("obstacles", {
                "message_index": message_index,
                "stamp": stamp,
                "frame_id": message.header.frame_id,
                "obstacle_index": index,
                "position_x": float(pose.position.x),
                "position_y": float(pose.position.y),
                "position_z": float(pose.position.z),
            })

    def _write_summary(self):
        if self.closed:
            return
        self.closed = True
        final = self.final_odom
        start = self.start_odom
        final_distance = None
        if final is not None:
            final_distance = math.hypot(
                final["position_x"] - self.goal[0],
                final["position_y"] - self.goal[1],
            )
        summary = {
            "run_name": str(self.get_parameter("run_name").value),
            "goal": {"x": self.goal[0], "y": self.goal[1]},
            "start": None if start is None else {
                "x": start["position_x"], "y": start["position_y"], "yaw": start["yaw"]},
            "termination": None if final is None else {
                "x": final["position_x"], "y": final["position_y"], "yaw": final["yaw"]},
            "final_distance_to_goal": final_distance,
            "goal_tolerance": self.goal_tolerance,
            "goal_reached": final_distance is not None and final_distance <= self.goal_tolerance,
            "termination_reason": (
                "goal_reached" if final_distance is not None and final_distance <= self.goal_tolerance
                else "shutdown_or_timeout"),
            "start_time": self.start_time,
            "end_time": self.end_time,
            "duration_s": None if self.start_time is None or self.end_time is None else self.end_time - self.start_time,
            "row_counts": {
                "planner": self.row_counts["planner"],
                "odom": self.row_counts["odom"],
                "cmd_vel": self.row_counts["cmd_vel"],
                "scan_beams": self.row_counts["scan"],
                "planned_path_points": self.row_counts["planned_path"],
                "obstacle_points": self.row_counts["obstacles"],
                "scan_frames": self.scan_index,
                "planned_path_messages": self.path_index,
                "obstacle_messages": self.obstacle_index,
            },
        }
        (self.output_dir / "summary.json").write_text(
            json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False),
            encoding="utf-8",
        )
        for handle in self.files.values():
            handle.flush()
            handle.close()

    def destroy_node(self):
        self._write_summary()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
