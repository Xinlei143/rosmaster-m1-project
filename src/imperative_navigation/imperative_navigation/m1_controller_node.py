"""Physical-robot adapter for the imperative navigation planner.

This node deliberately consumes only the Rosmaster M1's ROS interfaces:
LaserScan, EKF odometry and TF.  It has no Gazebo fallback or simulator-truth
obstacle input.  A missing/stale sensor, unavailable TF, disabled controller,
or close laser return always results in a zero raw command for the independent
hardware command watchdog.
"""

import math
import threading
import time
import traceback

import numpy as np
import rclpy
import tf2_ros
import torch
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Point, PoseArray, PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

from imperative_navigation.algorithm_loader import load_algorithm
from imperative_navigation.planner_timing import MeasuredPlannerPeriod
from imperative_navigation.track_stability import ConfirmedTrackFilter


def configure_planner_robot_clearance(algorithm, robot_radius, safety_margin):
    """Apply physical geometry and recompute its existing planner derivative.

    ``LIDAR_SAFE_DISTANCE`` is the planner's clearance radius used by
    ``collision_penalty``.  It is calculated at module import in the original
    algorithm, so it must be refreshed after ROS parameters override the two
    source values.  Emergency stopping is intentionally separate and remains
    in ``control_callback``.
    """
    algorithm.ROBOT_RADIUS = float(robot_radius)
    algorithm.SAFETY_MARGIN = float(safety_margin)
    algorithm.LIDAR_SAFE_DISTANCE = algorithm.ROBOT_RADIUS + algorithm.SAFETY_MARGIN


class ImperativeM1Controller(Node):
    """Run the imperative planner on a holonomic Rosmaster M1."""

    def __init__(self):
        super().__init__("imperative_m1_controller")
        for name, value in (
            ("goal_x", 1.0), ("goal_y", 0.0), ("control_period", 0.1),
            # Conservative indoor first-test limits.  Increase only after the
            # stop distance and obstacle detection have been validated.
            ("max_speed", 0.18), ("max_acceleration", 0.25),
            ("robot_radius", 0.18), ("safety_margin", 0.18),
            ("scan_topic", "/scan"), ("odom_topic", "/odom"),
            ("command_topic", "/imperative/cmd_vel_raw"), ("odom_frame", "odom"),
            ("scan_timeout", 0.50), ("odom_timeout", 0.50),
            ("tf_max_age", 0.30),
            # Raw laser points always remain collision obstacles.  These
            # adapter-only settings only decide when a noisy fitted circle is
            # trusted for moving-obstacle prediction and RViz output.
            ("track_confirmation_age", 3),
            ("track_position_alpha", 0.35),
            ("static_track_speed_threshold", 0.25),
            # Zero preserves PyTorch's platform default.  The value can be
            # overridden for on-robot benchmarking without changing planning.
            ("torch_num_threads", 0),
            ("emergency_stop_distance", 0.45), ("enabled", False),
        ):
            self.declare_parameter(name, value)

        self.period = float(self.get_parameter("control_period").value)
        self.planner_period = MeasuredPlannerPeriod(self.period)
        requested_torch_threads = int(self.get_parameter("torch_num_threads").value)
        if requested_torch_threads > 0:
            torch.set_num_threads(requested_torch_threads)
        self.algorithm = load_algorithm()
        # The original source has a simulated 8 x 6.4 m room.  A real robot's
        # walls come from its laser map, so disable those artificial bounds.
        self.algorithm.ROOM_X_MIN = self.algorithm.ROOM_Y_MIN = -1000.0
        self.algorithm.ROOM_X_MAX = self.algorithm.ROOM_Y_MAX = 1000.0
        self.algorithm.GOAL = torch.tensor([
            self.get_parameter("goal_x").value, self.get_parameter("goal_y").value],
            dtype=torch.float32)
        self.algorithm.DT = self.period
        self.algorithm.MAX_SPEED = float(self.get_parameter("max_speed").value)
        self.algorithm.MAX_ACCELERATION = float(self.get_parameter("max_acceleration").value)
        # Keep rollout candidates within the same acceleration envelope as the
        # command sent to hardware.  The demo's 0.25/0.5/1.0 values were for a
        # 1 m/s² simulated AGV and otherwise make its prediction too optimistic.
        self.algorithm.PLAN_ACCELERATIONS = torch.tensor([
            0.25 * self.algorithm.MAX_ACCELERATION,
            0.50 * self.algorithm.MAX_ACCELERATION,
            self.algorithm.MAX_ACCELERATION,
        ], dtype=torch.float32)
        configure_planner_robot_clearance(
            self.algorithm,
            self.get_parameter("robot_radius").value,
            self.get_parameter("safety_margin").value)

        # Sensor callbacks may run while planning; the timer callback remains
        # mutually exclusive so two planner cycles can never overlap.
        self.sensor_callback_group = ReentrantCallbackGroup()
        self.planner_callback_group = MutuallyExclusiveCallbackGroup()
        self.sensor_lock = threading.Lock()
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, self.get_parameter("scan_topic").value,
                                 self.scan_callback, sensor_qos,
                                 callback_group=self.sensor_callback_group)
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value,
                                 self.odom_callback, 10,
                                 callback_group=self.sensor_callback_group)
        self.command_publisher = self.create_publisher(
            Twist, self.get_parameter("command_topic").value, 10)
        self.path_publisher = self.create_publisher(Path, "/imperative/planned_path", 10)
        self.track_publisher = self.create_publisher(MarkerArray, "/imperative/tracks", 10)
        self.status_publisher = self.create_publisher(PoseArray, "/imperative/obstacle_centers", 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=False)
        self.latest_scan = None
        self.last_scan_time = None
        self.last_odom_time = None
        self.scan_geometry = None
        self.scan_cos = None
        self.scan_sin = None
        self.position = None
        self.yaw = 0.0
        self.velocity_world = torch.zeros(2)
        self.map_points = torch.empty(0, 2)
        self.tracks, self.track_histories, self.dynamic_track_ids = [], {}, set()
        self.planning_tracks = []
        self.track_filter = ConfirmedTrackFilter(
            self.get_parameter("track_confirmation_age").value,
            self.get_parameter("track_position_alpha").value,
            self.get_parameter("static_track_speed_threshold").value)
        self.published_track_ids = set()
        self.next_track_id = 0
        self.previous_plan = None
        self.last_warning = ""
        self.last_warning_time = -1
        self.last_debug_time = -1
        self.last_dry_run_log_time = -1
        self.scan_hit_count = 0
        self.nearest_range = float("nan")
        self.create_timer(self.period, self.control_callback,
                          callback_group=self.planner_callback_group)
        if bool(self.get_parameter("enabled").value):
            self.get_logger().warn(
                "M1 controller starts with physical motion enabled. Keep the area clear and the "
                "independent command watchdog running.")
        else:
            self.get_logger().warn(
                "M1 controller starts in dry-run mode: planning is active, physical motion is disabled. "
                "Set enabled:=true only in a clear test area.")
        self.get_logger().info(
            "PyTorch CPU threads: intra-op=%d inter-op=%d (torch_num_threads=%d)." %
            (torch.get_num_threads(), torch.get_num_interop_threads(), requested_torch_threads))

    def scan_callback(self, message):
        received_time = self.get_clock().now().nanoseconds
        with self.sensor_lock:
            self.latest_scan = message
            self.last_scan_time = received_time

    def odom_callback(self, message):
        pose = message.pose.pose
        position = torch.tensor([pose.position.x, pose.position.y], dtype=torch.float32)
        q = pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        velocity = message.twist.twist.linear
        cosine, sine = math.cos(yaw), math.sin(yaw)
        velocity_world = torch.tensor([
            cosine * velocity.x - sine * velocity.y,
            sine * velocity.x + cosine * velocity.y,
        ], dtype=torch.float32)
        received_time = self.get_clock().now().nanoseconds
        with self.sensor_lock:
            self.position = position
            self.yaw = yaw
            self.velocity_world = velocity_world
            self.last_odom_time = received_time

    def warn_throttled(self, text):
        now = self.get_clock().now().nanoseconds
        if text != self.last_warning or now - self.last_warning_time > 1_000_000_000:
            self.get_logger().warn(text)
            self.last_warning, self.last_warning_time = text, now

    def sensor_snapshot(self):
        """Return one consistent sensor state while callbacks update in parallel."""
        with self.sensor_lock:
            return (
                self.latest_scan,
                self.last_scan_time,
                self.position.clone() if self.position is not None else None,
                self.yaw,
                self.velocity_world.clone(),
                self.last_odom_time,
            )

    def input_is_fresh(self, now, scan_time, position, odom_time):
        if scan_time is None or position is None or odom_time is None:
            self.warn_throttled("Waiting for both /scan and /odom; publishing stop.")
            return False, None, None
        scan_limit = int(float(self.get_parameter("scan_timeout").value) * 1e9)
        odom_limit = int(float(self.get_parameter("odom_timeout").value) * 1e9)
        scan_age = (now - scan_time) / 1e9
        odom_age = (now - odom_time) / 1e9
        if now - scan_time > scan_limit or now - odom_time > odom_limit:
            self.warn_throttled("Stale /scan or /odom; publishing stop.")
            return False, scan_age, odom_age
        return True, scan_age, odom_age

    @staticmethod
    def planar_rotation(quaternion):
        """Return the first two rows of a quaternion rotation matrix.

        A full matrix is necessary here: the M1 launch's laser TF includes a
        3-D flip, which cannot safely be represented by yaw alone.
        """
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm < 1e-9:
            raise ValueError("zero quaternion in laser transform")
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z)],
        ], dtype=np.float32)

    def scan_as_robot_relative_points(self, scan, position):
        ranges = np.asarray(scan.ranges, dtype=np.float32)
        geometry = (len(ranges), float(scan.angle_min), float(scan.angle_increment))
        if geometry != self.scan_geometry:
            angles = (scan.angle_min +
                      np.arange(len(ranges), dtype=np.float32) * scan.angle_increment)
            self.scan_cos = np.cos(angles)
            self.scan_sin = np.sin(angles)
            self.scan_geometry = geometry
        valid = (np.isfinite(ranges) & (ranges >= scan.range_min) &
                 (ranges <= scan.range_max) & (ranges > 0.0))
        self.scan_hit_count = int(np.count_nonzero(valid))
        self.nearest_range = float(np.min(ranges[valid])) if self.scan_hit_count else float("nan")
        if not self.scan_hit_count:
            raise RuntimeError("Laser scan contains no valid returns")

        # The laser and TF publishers are not perfectly synchronized on the
        # physical robot.  Request the latest available transform instead of
        # extrapolating to the scan timestamp, then reject it if it is too old.
        transform = self.tf_buffer.lookup_transform(
            self.get_parameter("odom_frame").value, scan.header.frame_id, Time())
        transform_time = Time.from_msg(transform.header.stamp).nanoseconds
        tf_age = (self.get_clock().now().nanoseconds - transform_time) / 1e9
        tf_max_age = float(self.get_parameter("tf_max_age").value)
        if transform_time <= 0:
            raise RuntimeError("latest laser transform has no usable timestamp")
        if tf_age < 0.0 or tf_age > tf_max_age:
            raise RuntimeError(
                f"latest laser transform age {tf_age:.3f} s is outside [0, {tf_max_age:.3f}] s")
        # Laser angular geometry is invariant across normal scans, so cache
        # the trigonometry while preserving the exact range-to-point equation.
        local_points = np.empty((len(ranges), 2), dtype=np.float32)
        np.multiply(ranges, self.scan_cos, out=local_points[:, 0])
        np.multiply(ranges, self.scan_sin, out=local_points[:, 1])
        rotation = self.planar_rotation(transform.transform.rotation)
        translation = np.array([transform.transform.translation.x, transform.transform.translation.y],
                               dtype=np.float32)
        # The planner adds self.position internally.  Give it vectors relative
        # to that origin after applying the physical laser->odom TF.
        relative_points = local_points @ rotation.T + translation - position.numpy()
        return torch.from_numpy(relative_points), torch.from_numpy(valid), tf_age

    def control_callback(self):
        now = self.get_clock().now().nanoseconds
        # The RDK X5 takes about 0.4--0.5 s for one planner callback.  Use the
        # elapsed callback interval in the unchanged planner equations rather
        # than predicting at 0.1 s while a Twist is held much longer in reality.
        planner_dt = self.planner_period.next_period(time.monotonic())
        self.algorithm.DT = planner_dt
        (scan, scan_time, position, yaw, current_velocity_world,
         odom_time) = self.sensor_snapshot()
        inputs_fresh, scan_age, odom_age = self.input_is_fresh(
            now, scan_time, position, odom_time)
        if not inputs_fresh:
            self.publish_stop()
            return
        profile = {}
        detection_profile = {}
        callback_start = time.perf_counter()
        try:
            # Autograd is never used by the online planner. Inference mode
            # removes its bookkeeping without altering the planner equations.
            with torch.inference_mode():
                stage_start = time.perf_counter()
                lidar_points, lidar_hits, tf_age = self.scan_as_robot_relative_points(scan, position)
                profile["laser_scan_to_points"] = time.perf_counter() - stage_start
        except (tf2_ros.TransformException, RuntimeError, ValueError) as error:
            self.warn_throttled(f"Cannot transform current laser scan ({error}); publishing stop.")
            self.publish_stop()
            return

        emergency_distance = float(self.get_parameter("emergency_stop_distance").value)
        if self.nearest_range <= emergency_distance:
            self.warn_throttled(
                f"Laser return at {self.nearest_range:.2f} m <= emergency limit "
                f"{emergency_distance:.2f} m; publishing stop.")
            self.publish_stop()
            self.publish_visualization(position, current_velocity_world)
            return

        try:
            with torch.inference_mode():
                stage_start = time.perf_counter()
                detections, world_points, _ = self.algorithm.scan_to_detections(
                    position, lidar_points, lidar_hits, profile=detection_profile)
                profile["scan_to_detections"] = time.perf_counter() - stage_start

                stage_start = time.perf_counter()
                self.tracks, self.next_track_id = self.algorithm.update_tracks(
                    self.tracks, detections, self.next_track_id)
                for track in self.tracks:
                    self.track_histories.setdefault(track["id"], []).append(track["position"].clone())
                self.planning_tracks = self.track_filter.update(self.tracks)
                self.dynamic_track_ids = {
                    track["id"] for track in self.planning_tracks
                    if torch.linalg.norm(track["velocity"]) > 0.0}
                dynamic_history = [torch.stack(self.track_histories[track_id])
                                   for track_id in self.dynamic_track_ids
                                   if track_id in self.track_histories]
                active_dynamic_tracks = [track["position"] for track in self.planning_tracks
                                         if track["id"] in self.dynamic_track_ids]
                if active_dynamic_tracks:
                    dynamic_history.append(torch.stack(active_dynamic_tracks))
                dynamic_centers = torch.cat(dynamic_history) if dynamic_history else torch.empty(0, 2)
                profile["update_tracks"] = time.perf_counter() - stage_start

                stage_start = time.perf_counter()
                self.map_points = self.algorithm.update_slam_map(
                    self.map_points, world_points, dynamic_centers)
                profile["update_slam_map"] = time.perf_counter() - stage_start

                if torch.linalg.norm(self.algorithm.GOAL - position) <= self.algorithm.GOAL_TOLERANCE:
                    self.get_logger().info("Goal reached; publishing stop.")
                    self.publish_stop()
                    self.publish_visualization(position, current_velocity_world)
                    return

                stage_start = time.perf_counter()
                acceleration, self.previous_plan = self.algorithm.select_acceleration(
                    position, current_velocity_world, self.planning_tracks, self.map_points, world_points,
                    self.previous_plan, return_plan=True)
                profile["select_acceleration"] = time.perf_counter() - stage_start

                stage_start = time.perf_counter()
                _, velocity_world = self.algorithm.step_robot(
                    position, current_velocity_world, acceleration)
                profile["step_robot"] = time.perf_counter() - stage_start
        except Exception as error:
            # Planning is never allowed to leave a previous velocity command
            # active, irrespective of whether this is dry-run or live mode.
            # Keep the complete traceback in the throttled warning while this
            # new split-search path is validated on the physical robot.  The
            # traceback exposes the exact file and line for any unexpected
            # planner exception without allowing a prior command to persist.
            self.warn_throttled(
                f"Planner failure ({error}); publishing stop.\n{traceback.format_exc()}")
            self.publish_stop()
            self.publish_visualization(position, current_velocity_world)
            return

        self.publish_body_velocity(velocity_world, yaw)
        stage_start = time.perf_counter()
        with torch.inference_mode():
            self.publish_visualization(position, current_velocity_world)
        profile["visualization"] = time.perf_counter() - stage_start
        planner_compute_time = time.perf_counter() - callback_start
        if self.last_debug_time < 0 or now - self.last_debug_time >= 1_000_000_000:
            self.last_debug_time = now
            self.get_logger().info(
                "scan_age=%.3f s odom_age=%.3f s tf_age=%.3f s planner_dt=%.3f s planner_compute_time=%.3f s; "
                "LaserScan->points=%.3f s scan_to_detections=%.3f s update_tracks=%.3f s "
                "update_slam_map=%.3f s select_acceleration=%.3f s step_robot=%.3f s visualization=%.3f s; "
                "detections[prepare=%.3f s cluster_scan=%.3f s cluster_wrap=%.3f s pending_total=%.3f s "
                "circle_primary=%.3f s circle_split=%.3f s split_search=%.3f s finalize=%.3f s; "
                "hits=%d clusters=%d primary_fits=%d split_fits=%d split_candidates=%d split_lstsq_fallbacks=%d]; "
                "pos=(%.2f, %.2f) goal=(%.2f, %.2f) hits=%d raw_tracks=%d confirmed_tracks=%d "
                "map_points=%d nearest=%.2f cmd=(%.2f, %.2f)" % (
                    scan_age, odom_age, tf_age, planner_dt, planner_compute_time,
                    profile["laser_scan_to_points"], profile["scan_to_detections"],
                    profile["update_tracks"], profile["update_slam_map"],
                    profile["select_acceleration"], profile["step_robot"], profile["visualization"],
                    detection_profile["prepare"], detection_profile["cluster_scan"],
                    detection_profile["cluster_wrap"], detection_profile["pending_clusters_total"],
                    detection_profile["circle_fit_primary"], detection_profile["circle_fit_split"],
                    detection_profile["split_search"], detection_profile["finalize"],
                    detection_profile["hit_count"], detection_profile["initial_cluster_count"],
                    detection_profile["primary_fit_calls"], detection_profile["split_fit_calls"],
                    detection_profile["split_candidates"], detection_profile["split_lstsq_fallbacks"],
                    position[0], position[1], self.algorithm.GOAL[0], self.algorithm.GOAL[1],
                    self.scan_hit_count, len(self.tracks), len(self.planning_tracks),
                    len(self.map_points), self.nearest_range,
                    velocity_world[0], velocity_world[1]))

    def publish_body_velocity(self, velocity_world, yaw):
        # Keep sensing, tracking, planning, and visualization active in
        # dry-run mode.  The enabled gate belongs only at the physical output.
        if not bool(self.get_parameter("enabled").value):
            now = self.get_clock().now().nanoseconds
            if (self.last_dry_run_log_time < 0 or
                    now - self.last_dry_run_log_time >= 1_000_000_000):
                self.last_dry_run_log_time = now
                self.get_logger().info(
                    "Dry-run planning active; physical motion disabled. "
                    "Desired world velocity=(%.2f, %.2f)" %
                    (velocity_world[0], velocity_world[1]))
            self.publish_stop()
            return

        cosine, sine = math.cos(yaw), math.sin(yaw)
        # The planner itself is configured with this cap, but enforce it again
        # at the sole hardware-output boundary as a final safety guard.
        speed_limit = float(self.get_parameter("max_speed").value)
        speed = torch.linalg.norm(velocity_world)
        limited_velocity = velocity_world * min(1.0, speed_limit / float(speed.clamp_min(1e-6)))
        command = Twist()
        command.linear.x = float(cosine * limited_velocity[0] + sine * limited_velocity[1])
        command.linear.y = float(-sine * limited_velocity[0] + cosine * limited_velocity[1])
        # The planner is translational.  M1's mecanum base can execute x/y
        # body velocity without injecting an unplanned yaw rotation.
        self.command_publisher.publish(command)

    def publish_stop(self):
        self.command_publisher.publish(Twist())

    def publish_stop_burst(self, count=3, interval_seconds=0.05):
        """Best-effort raw zero burst before process teardown.

        The separately launched watchdog remains the safety guarantee if this
        cleanup cannot run, but a few immediate raw zeros reduce stop latency
        while the ROS context is still valid.
        """
        for index in range(count):
            if not rclpy.ok():
                break
            self.publish_stop()
            if index + 1 < count:
                time.sleep(interval_seconds)

    def publish_visualization(self, position, velocity_world):
        stamp = self.get_clock().now().to_msg()
        frame = self.get_parameter("odom_frame").value
        path = Path()
        path.header.frame_id, path.header.stamp = frame, stamp
        if position is not None and self.previous_plan is not None:
            position, velocity = position.clone(), velocity_world.clone()
            for acceleration in self.previous_plan:
                # Match the visualized rollout to the actual elapsed planner
                # interval used for this command.
                velocity += self.algorithm.DT * acceleration
                position += self.algorithm.DT * velocity
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x, pose.pose.position.y, pose.pose.orientation.w = (
                    float(position[0]), float(position[1]), 1.0)
                path.poses.append(pose)
        self.path_publisher.publish(path)
        markers, centers = MarkerArray(), PoseArray()
        centers.header.frame_id, centers.header.stamp = frame, stamp
        visible_track_ids = {track["id"] for track in self.planning_tracks}
        for track_id in sorted(self.published_track_ids - visible_track_ids):
            marker = Marker()
            marker.header.frame_id, marker.header.stamp = frame, stamp
            marker.ns, marker.id, marker.action = "imperative_tracks", track_id, Marker.DELETE
            markers.markers.append(marker)
        self.published_track_ids = visible_track_ids
        for track in self.planning_tracks:
            marker = Marker()
            marker.header.frame_id, marker.header.stamp = frame, stamp
            marker.ns, marker.id, marker.type, marker.action = (
                "imperative_tracks", track["id"], Marker.CYLINDER, Marker.ADD)
            marker.pose.position.x, marker.pose.position.y, marker.pose.orientation.w = (
                float(track["position"][0]), float(track["position"][1]), 1.0)
            marker.scale.x = marker.scale.y = 2.0 * self.algorithm.OBSTACLE_RADIUS
            marker.scale.z, marker.color.r, marker.color.g, marker.color.a = 0.08, 1.0, 0.3, 0.8
            markers.markers.append(marker)
            centers.poses.append(marker.pose)
        self.track_publisher.publish(markers)
        self.status_publisher.publish(centers)


def main(args=None):
    rclpy.init(args=args)
    node = ImperativeM1Controller()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # The separate watchdog guarantees the eventual stop. Send an extra
        # best-effort raw-zero burst while this process still has a ROS context.
        if rclpy.ok():
            node.publish_stop_burst()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
