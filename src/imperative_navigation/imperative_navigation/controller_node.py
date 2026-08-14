"""Run the original planner against real ROS 2 LaserScan and Odometry messages."""

import math
import time

import numpy as np
import rclpy
import torch
from geometry_msgs.msg import Point, PoseArray, PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray

from imperative_navigation.algorithm_loader import load_algorithm
from imperative_navigation.debug_protocol import PLANNER_DEBUG_FIELDS
from imperative_navigation.planner_timing import MeasuredPlannerPeriod
from imperative_navigation.track_stability import ConfirmedTrackFilter


class ImperativeController(Node):
    """Planner adapter for a holonomic Rosmaster M1 base."""

    def __init__(self):
        super().__init__("imperative_controller")
        self.declare_parameter("goal_x", 2.5)
        self.declare_parameter("goal_y", 1.5)
        self.declare_parameter("control_period", 0.1)
        # Keep the planner's numerical model aligned with the original demo.
        # A lower speed can still be supplied as a launch parameter for a real car.
        self.declare_parameter("max_speed", 1.0)
        self.declare_parameter("max_acceleration", 1.0)
        # Preserve the former Gazebo geometry by default. Pass the physical
        # values explicitly when performing a sim-to-real comparison.
        self.declare_parameter("robot_radius", 0.15)
        self.declare_parameter("safety_margin", 0.15)
        self.declare_parameter("track_confirmation_age", 3)
        self.declare_parameter("track_position_alpha", 0.35)
        self.declare_parameter("static_track_speed_threshold", 0.25)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("command_topic", "/imperative/cmd_vel")
        # Optional [x, y, radius, ...] obstacle list used only by the Gazebo
        # WSLg fallback when GPU LiDAR returns a saturated frame.  Leave empty
        # on a physical robot: a failed LiDAR then causes a safe stop.
        self.declare_parameter("static_obstacles", Parameter.Type.DOUBLE_ARRAY)
        # Gazebo can publish the exact centers of its moving obstacles.  This
        # is intentionally a simulation-only perception adapter for WSLg,
        # where the GPU LiDAR currently reports a saturated scan.
        self.declare_parameter("dynamic_obstacles_topic", "/imperative/dynamic_obstacles")
        self.declare_parameter("require_dynamic_obstacles", False)
        self.declare_parameter("dynamic_obstacle_timeout", 0.5)

        self.algorithm = load_algorithm()
        self.algorithm.GOAL = torch.tensor([
            self.get_parameter("goal_x").value,
            self.get_parameter("goal_y").value,
        ], dtype=torch.float32)
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.period = float(self.get_parameter("control_period").value)
        self.planner_period = MeasuredPlannerPeriod(self.period)
        self.algorithm.DT = self.period
        self.algorithm.MAX_SPEED = self.max_speed
        self.algorithm.MAX_ACCELERATION = float(self.get_parameter("max_acceleration").value)
        self.algorithm.PLAN_ACCELERATIONS = torch.tensor([
            0.25 * self.algorithm.MAX_ACCELERATION,
            0.50 * self.algorithm.MAX_ACCELERATION,
            self.algorithm.MAX_ACCELERATION,
        ], dtype=torch.float32)
        self.algorithm.ROBOT_RADIUS = float(self.get_parameter("robot_radius").value)
        self.algorithm.SAFETY_MARGIN = float(self.get_parameter("safety_margin").value)
        self.algorithm.LIDAR_SAFE_DISTANCE = (
            self.algorithm.ROBOT_RADIUS + self.algorithm.SAFETY_MARGIN)

        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan_subscriber = self.create_subscription(
            LaserScan, self.get_parameter("scan_topic").value, self.scan_callback, sensor_qos)
        self.odom_subscriber = self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self.odom_callback, 10)
        self.dynamic_obstacle_subscriber = self.create_subscription(
            PoseArray, self.get_parameter("dynamic_obstacles_topic").value,
            self.dynamic_obstacles_callback, 10)
        self.command_publisher = self.create_publisher(
            Twist, self.get_parameter("command_topic").value, 10)
        self.path_publisher = self.create_publisher(Path, "/imperative/planned_path", 10)
        self.track_publisher = self.create_publisher(MarkerArray, "/imperative/tracks", 10)
        self.planner_debug_publisher = self.create_publisher(
            Float64MultiArray, "/imperative/planner_debug", 10)

        self.latest_scan = None
        self.position = None
        self.yaw = 0.0
        self.velocity_world = torch.zeros(2)
        self.tracks = []
        self.planning_tracks = []
        self.track_filter = ConfirmedTrackFilter(
            self.get_parameter("track_confirmation_age").value,
            self.get_parameter("track_position_alpha").value,
            self.get_parameter("static_track_speed_threshold").value)
        self.published_track_ids = set()
        self.next_track_id = 0
        self.track_histories = {}
        self.dynamic_track_ids = set()
        static_obstacles = list(self.get_parameter("static_obstacles").value)
        if len(static_obstacles) % 3:
            raise ValueError("static_obstacles must contain x, y, radius triples")
        static_boundary_points = []
        for index in range(0, len(static_obstacles), 3):
            x, y, radius = (float(value) for value in static_obstacles[index:index + 3])
            # The planner represents static geometry as measured surface
            # points, so sample each circle's collision boundary.
            angles = torch.arange(36, dtype=torch.float32) * (2.0 * math.pi / 36.0)
            center = torch.tensor([x, y], dtype=torch.float32)
            static_boundary_points.append(center + radius * torch.stack((torch.cos(angles), torch.sin(angles)), dim=1))
        self.static_fallback_map = (torch.cat(static_boundary_points)
                                    if static_boundary_points else torch.empty(0, 2))
        self.map_points = self.static_fallback_map.clone()
        self.previous_plan = None
        self.scan_is_saturated = False
        self.scan_hit_count = 0
        self.scan_min_range = float("nan")
        self.sim_dynamic_detections = torch.empty(0, 2)
        self.last_dynamic_obstacles_time = None
        self.last_debug_log_time = -1
        self.last_saturation_warning_time = -1
        self.create_timer(self.period, self.control_callback)
        self.get_logger().info("Imperative controller ready; waiting for /scan and /odom.")

    def scan_callback(self, message):
        self.latest_scan = message

    def odom_callback(self, message):
        pose = message.pose.pose
        self.position = torch.tensor([pose.position.x, pose.position.y], dtype=torch.float32)
        orientation = pose.orientation
        self.yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
        )
        velocity = message.twist.twist.linear
        cosine, sine = math.cos(self.yaw), math.sin(self.yaw)
        self.velocity_world = torch.tensor([
            cosine * velocity.x - sine * velocity.y,
            sine * velocity.x + cosine * velocity.y,
        ], dtype=torch.float32)

    def dynamic_obstacles_callback(self, message):
        """Accept Gazebo's simulation-truth obstacle centers in the WSLg fallback."""
        self.sim_dynamic_detections = torch.tensor(
            [[pose.position.x, pose.position.y] for pose in message.poses], dtype=torch.float32)
        self.last_dynamic_obstacles_time = self.get_clock().now().nanoseconds

    def have_fresh_dynamic_obstacles(self, now):
        if self.last_dynamic_obstacles_time is None:
            return False
        timeout = float(self.get_parameter("dynamic_obstacle_timeout").value) * 1_000_000_000
        return now - self.last_dynamic_obstacles_time <= timeout

    def scan_as_world_relative_points(self):
        scan = self.latest_scan
        ranges = np.asarray(scan.ranges, dtype=np.float32)
        angles = scan.angle_min + np.arange(len(ranges), dtype=np.float32) * scan.angle_increment
        valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)

        # A GPU-LiDAR frame in which almost every beam equals the minimum range
        # is a sensor/model failure (typically self-returns), not a ring of
        # obstacles around the robot. Never add such a frame to the original
        # algorithm's map or tracker.
        near_minimum = valid & (ranges <= scan.range_min + 0.01)
        self.scan_is_saturated = len(ranges) > 0 and np.count_nonzero(near_minimum) >= 0.95 * len(ranges)
        if self.scan_is_saturated:
            valid = np.zeros_like(valid, dtype=bool)

        self.scan_hit_count = int(np.count_nonzero(valid))
        self.scan_min_range = float(np.min(ranges[valid])) if self.scan_hit_count else float("nan")
        ranges = np.where(valid, ranges, float(self.algorithm.LIDAR_RANGE))
        local_points = np.column_stack((ranges * np.cos(angles), ranges * np.sin(angles)))
        cosine, sine = math.cos(self.yaw), math.sin(self.yaw)
        rotation = np.array([[cosine, -sine], [sine, cosine]], dtype=np.float32)
        return torch.from_numpy(local_points @ rotation.T), torch.from_numpy(valid)

    def control_callback(self):
        self.algorithm.DT = self.planner_period.next_period(time.monotonic())
        if self.position is None or self.latest_scan is None:
            stop_command = self.publish_stop()
            if self.position is not None:
                goal_distance = torch.linalg.norm(self.algorithm.GOAL - self.position)
                self.publish_planner_debug(
                    torch.zeros(2), torch.zeros(2), stop_command, goal_distance, stopped=True)
            return

        lidar_points, lidar_hits = self.scan_as_world_relative_points()
        now = self.get_clock().now().nanoseconds
        if (self.scan_is_saturated and
                (self.last_saturation_warning_time < 0 or
                 now - self.last_saturation_warning_time >= 1_000_000_000)):
            self.last_saturation_warning_time = now
            self.get_logger().warn(
                "Ignoring a saturated /scan frame: almost all beams equal the sensor minimum range.")
        # Do not continue toward the goal without obstacle observations.  The
        # WSLg / OGRE Gazebo 6 stack returns an all-minimum GPU-LiDAR frame;
        # its explicitly declared static scene map is sufficient for the
        # static-obstacle test, but never substitutes for a failed sensor on a
        # real robot.
        needs_dynamic_feed = bool(self.get_parameter("require_dynamic_obstacles").value)
        dynamic_feed_is_fresh = self.have_fresh_dynamic_obstacles(now)
        if self.scan_is_saturated and (not len(self.static_fallback_map) or
                                      (needs_dynamic_feed and not dynamic_feed_is_fresh)):
            stop_command = self.publish_stop()
            goal_distance = torch.linalg.norm(self.algorithm.GOAL - self.position)
            self.publish_planner_debug(
                torch.zeros(2), torch.zeros(2), stop_command, goal_distance, stopped=True)
            self.publish_visualization()
            return
        if self.scan_is_saturated:
            detections = self.sim_dynamic_detections if dynamic_feed_is_fresh else torch.empty(0, 2)
            world_points = torch.empty(0, 2)
            if self.last_saturation_warning_time == now:
                self.get_logger().warn(
                    "GPU LiDAR is saturated; using configured static_obstacles fallback. "
                    "Dynamic obstacles are supplied by the Gazebo simulation feed.")
        else:
            detections, world_points, _ = self.algorithm.scan_to_detections(
                self.position, lidar_points, lidar_hits)
        self.tracks, self.next_track_id = self.algorithm.update_tracks(
            self.tracks, detections, self.next_track_id)

        # This is the same dynamic-point removal policy used in
        # Imperative_learning_2D_moving.run_navigation: retain a trajectory
        # history for confirmed moving tracks so their old laser returns do not
        # become false static obstacles in the SLAM map.
        for track in self.tracks:
            self.track_histories.setdefault(track["id"], []).append(track["position"].clone())
        self.planning_tracks = self.track_filter.update(self.tracks)
        self.dynamic_track_ids = {
            track["id"] for track in self.planning_tracks
            if torch.linalg.norm(track["velocity"]) > 0.0}

        moving_histories = [torch.stack(self.track_histories[track_id])
                            for track_id in self.dynamic_track_ids
                            if track_id in self.track_histories]
        active_dynamic_tracks = [track["position"] for track in self.planning_tracks
                                 if track["id"] in self.dynamic_track_ids]
        if active_dynamic_tracks:
            moving_histories.append(torch.stack(active_dynamic_tracks))
        dynamic_centers = (torch.cat(moving_histories)
                           if moving_histories else torch.empty(0, 2))
        self.map_points = self.algorithm.update_slam_map(self.map_points, world_points, dynamic_centers)

        goal_distance = torch.linalg.norm(self.algorithm.GOAL - self.position)
        if goal_distance <= self.algorithm.GOAL_TOLERANCE:
            stop_command = self.publish_stop()
            self.publish_planner_debug(
                torch.zeros(2), torch.zeros(2), stop_command, goal_distance, stopped=True)
            self.publish_visualization()
            return

        acceleration, self.previous_plan = self.algorithm.select_acceleration(
            self.position, self.velocity_world, self.planning_tracks, self.map_points, world_points,
            self.previous_plan, return_plan=True)
        # Use the original demo's dynamics helper verbatim for the command
        # calculation. Gazebo then provides the measured next state via /odom.
        _, commanded_world_velocity = self.algorithm.step_robot(
            self.position, self.velocity_world, acceleration)
        command = self.publish_body_velocity(commanded_world_velocity)
        self.publish_planner_debug(
            acceleration, commanded_world_velocity, command, goal_distance)
        if self.last_debug_log_time < 0 or now - self.last_debug_log_time >= 1_000_000_000:
            self.last_debug_log_time = now
            self.get_logger().info(
                "planner state: pos=(%.2f, %.2f), scan_hits=%d, dynamic_tracks=%d, scan_min=%s, "
                "acc=(%.2f, %.2f), cmd_world=(%.2f, %.2f)" % (
                    self.position[0], self.position[1], self.scan_hit_count,
                    len(self.planning_tracks),
                    "n/a" if math.isnan(self.scan_min_range) else f"{self.scan_min_range:.2f}",
                    acceleration[0], acceleration[1],
                    commanded_world_velocity[0], commanded_world_velocity[1]))
        self.publish_visualization()

    def publish_body_velocity(self, velocity_world):
        cosine, sine = math.cos(self.yaw), math.sin(self.yaw)
        command = Twist()
        command.linear.x = float(cosine * velocity_world[0] + sine * velocity_world[1])
        command.linear.y = float(-sine * velocity_world[0] + cosine * velocity_world[1])
        self.command_publisher.publish(command)
        return command

    def publish_stop(self):
        command = Twist()
        self.command_publisher.publish(command)
        return command

    def publish_planner_debug(self, acceleration, commanded_world_velocity, command,
                              goal_distance, stopped=False):
        """Publish exact planner outputs for synchronized experiment logging."""
        if self.position is None:
            return
        values = {
            "stamp": self.get_clock().now().nanoseconds * 1e-9,
            "goal_x": float(self.algorithm.GOAL[0]),
            "goal_y": float(self.algorithm.GOAL[1]),
            "position_x": float(self.position[0]),
            "position_y": float(self.position[1]),
            "yaw": float(self.yaw),
            "planner_accel_x": float(acceleration[0]),
            "planner_accel_y": float(acceleration[1]),
            "command_world_vx": float(commanded_world_velocity[0]),
            "command_world_vy": float(commanded_world_velocity[1]),
            "command_body_vx": float(command.linear.x),
            "command_body_vy": float(command.linear.y),
            "planner_dt": float(self.algorithm.DT),
            "scan_hits": float(self.scan_hit_count),
            "scan_min_range": float(self.scan_min_range),
            "dynamic_tracks": float(len(self.planning_tracks)),
            "goal_distance": float(goal_distance),
            "stopped": float(stopped),
            "scan_saturated": float(self.scan_is_saturated),
        }
        message = Float64MultiArray()
        message.data = [values[field] for field in PLANNER_DEBUG_FIELDS]
        self.planner_debug_publisher.publish(message)

    def publish_visualization(self):
        stamp = self.get_clock().now().to_msg()
        path = Path()
        path.header.frame_id = "odom"
        path.header.stamp = stamp
        if self.previous_plan is not None:
            predicted_position = self.position.clone()
            predicted_velocity = self.velocity_world.clone()
            for acceleration in self.previous_plan:
                # ``DT`` is the measured elapsed planner interval, not merely
                # the requested timer period.  Draw the same rollout horizon
                # that the planner used to choose this plan.
                predicted_velocity += self.algorithm.DT * acceleration
                predicted_position += self.algorithm.DT * predicted_velocity
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = float(predicted_position[0])
                pose.pose.position.y = float(predicted_position[1])
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
        self.path_publisher.publish(path)

        markers = MarkerArray()
        visible_track_ids = {track["id"] for track in self.planning_tracks}
        for track_id in sorted(self.published_track_ids - visible_track_ids):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = stamp
            marker.ns = "imperative_tracks"
            marker.id = track_id
            marker.action = Marker.DELETE
            markers.markers.append(marker)
        self.published_track_ids = visible_track_ids
        for track in self.planning_tracks:
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = stamp
            marker.ns = "imperative_tracks"
            marker.id = track["id"]
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = float(track["position"][0])
            marker.pose.position.y = float(track["position"][1])
            marker.pose.orientation.w = 1.0
            marker.scale.x = 2.0 * self.algorithm.OBSTACLE_RADIUS
            marker.scale.y = 2.0 * self.algorithm.OBSTACLE_RADIUS
            marker.scale.z = 0.08
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.3, 0.0, 0.8
            markers.markers.append(marker)
        self.track_publisher.publish(markers)

def main(args=None):
    rclpy.init(args=args)
    node = ImperativeController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # ROS may already have invalidated the context after Ctrl+C, so don't
        # publish from destroy_node. Gazebo's velocity controller receives
        # commands only while this node is alive.
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
