"""Integrate a configurable, deterministic wheel-odometry error model.

The Gazebo odometry publisher is deliberately treated as ground truth.  This
node consumes that pose, integrates frame-to-frame body increments with scale,
bias, noise, and optional burst slip, and is the sole publisher of the normal
``/odom`` topic and ``odom -> base_footprint`` transform in the localized
simulation.
"""

from dataclasses import dataclass
import math
import random

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


@dataclass
class SlipState:
    """The simulated odometry pose in the odom/world coordinate system."""

    x: float
    y: float
    yaw: float


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def world_to_body(dx_world, dy_world, yaw):
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return cosine * dx_world + sine * dy_world, -sine * dx_world + cosine * dy_world


def body_to_world(dx_body, dy_body, yaw):
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return cosine * dx_body - sine * dy_body, sine * dx_body + cosine * dy_body


def integrate_increment(
    state,
    dx_world,
    dy_world,
    dyaw,
    dt,
    x_scale,
    y_scale,
    yaw_scale,
    x_bias_per_second,
    y_bias_per_second,
    yaw_bias_per_second,
    noise=(0.0, 0.0, 0.0),
):
    """Apply one noisy, scaled local odometry increment to ``state``.

    Scale and bias are applied in the robot-local frame.  The resulting local
    translation is then integrated using the current simulated heading, which
    makes lateral mecanum slip accumulate in the expected direction.
    """

    if dt <= 0.0:
        raise ValueError("odometry increment dt must be positive")
    local_x, local_y = world_to_body(dx_world, dy_world, state.yaw)
    local_x = x_scale * local_x + x_bias_per_second * dt + noise[0]
    local_y = y_scale * local_y + y_bias_per_second * dt + noise[1]
    delta_yaw = yaw_scale * dyaw + yaw_bias_per_second * dt + noise[2]
    world_x, world_y = body_to_world(local_x, local_y, state.yaw)
    state.x += world_x
    state.y += world_y
    state.yaw += delta_yaw
    return local_x / dt, local_y / dt, delta_yaw / dt


def profile_parameters(profile):
    """Return the small, documented set of built-in slip profiles."""

    profiles = {
        "none": {"x_scale": 1.0, "y_scale": 1.0, "yaw_scale": 1.0},
        "mild": {"x_scale": 1.05, "y_scale": 1.10, "yaw_scale": 1.0},
        "lateral": {"x_scale": 1.0, "y_scale": 1.20, "yaw_scale": 1.0},
        "severe": {"x_scale": 1.10, "y_scale": 1.30, "yaw_scale": 1.0},
        "burst": {"x_scale": 1.0, "y_scale": 1.0, "yaw_scale": 1.0,
                   "burst_enabled": True, "burst_start": 2.0,
                   "burst_duration": 2.0, "burst_x_scale": 1.0,
                   "burst_y_scale": 2.0, "burst_yaw_scale": 1.0},
    }
    if profile not in profiles:
        raise ValueError("unknown slip profile %r" % profile)
    return dict(profiles[profile])


class OdomSlipSimulator(Node):
    """Convert Gazebo ground-truth odometry into simulated wheel odometry."""

    def __init__(self):
        super().__init__("odom_slip_simulator")
        defaults = (
            ("enabled", True),
            ("input_topic", "/ground_truth/odom"),
            ("output_topic", "/odom"),
            ("odom_frame", "odom"),
            ("base_frame", "base_footprint"),
            ("profile", "none"),
            ("x_scale", 1.0), ("y_scale", 1.0), ("yaw_scale", 1.0),
            ("x_noise_std", 0.0), ("y_noise_std", 0.0), ("yaw_noise_std", 0.0),
            ("x_bias_per_second", 0.0), ("y_bias_per_second", 0.0),
            ("yaw_bias_per_second", 0.0),
            ("random_walk_xy_std", 0.0), ("random_walk_yaw_std", 0.0),
            ("random_seed", 20260902),
            ("burst_enabled", False), ("burst_start", 0.0),
            ("burst_duration", 0.0), ("burst_x_scale", 1.0),
            ("burst_y_scale", 1.0), ("burst_yaw_scale", 1.0),
        )
        for name, value in defaults:
            self.declare_parameter(name, value)
        self.rng = random.Random(int(self.get_parameter("random_seed").value))
        self.true_previous = None
        self.true_previous_yaw = None
        self.last_stamp_ns = None
        self.sim_state = None
        self.sim_time_origin_ns = None
        self.publisher = self.create_publisher(
            Odometry, self.get_parameter("output_topic").value, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Odometry, self.get_parameter("input_topic").value,
            self.odom_callback, 10)

    def _parameter(self, name):
        return self.get_parameter(name).value

    def _stamp_ns(self, message):
        stamp = message.header.stamp
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        return stamp_ns if stamp_ns > 0 else self.get_clock().now().nanoseconds

    def _active_scales(self, elapsed):
        if not bool(self._parameter("enabled")):
            return 1.0, 1.0, 1.0
        scales = profile_parameters(str(self._parameter("profile")))
        x_scale = float(scales.get("x_scale", 1.0))
        y_scale = float(scales.get("y_scale", 1.0))
        yaw_scale = float(scales.get("yaw_scale", 1.0))
        # Explicit parameters are the public override for a built-in profile.
        for name in ("x_scale", "y_scale", "yaw_scale"):
            value = float(self._parameter(name))
            if value != 1.0:
                if name == "x_scale":
                    x_scale = value
                elif name == "y_scale":
                    y_scale = value
                else:
                    yaw_scale = value
        burst = bool(self._parameter("burst_enabled")) or bool(scales.get("burst_enabled", False))
        start = float(self._parameter("burst_start"))
        duration = float(self._parameter("burst_duration"))
        if "burst_start" in scales and start == 0.0:
            start = float(scales["burst_start"])
        if "burst_duration" in scales and duration == 0.0:
            duration = float(scales["burst_duration"])
        if burst and start <= elapsed < start + max(0.0, duration):
            x_scale *= float(self._parameter("burst_x_scale")) if float(self._parameter("burst_x_scale")) != 1.0 else float(scales.get("burst_x_scale", 1.0))
            y_scale *= float(self._parameter("burst_y_scale")) if float(self._parameter("burst_y_scale")) != 1.0 else float(scales.get("burst_y_scale", 1.0))
            yaw_scale *= float(self._parameter("burst_yaw_scale")) if float(self._parameter("burst_yaw_scale")) != 1.0 else float(scales.get("burst_yaw_scale", 1.0))
        return x_scale, y_scale, yaw_scale

    def _noise(self, dt):
        if not bool(self._parameter("enabled")):
            return 0.0, 0.0, 0.0
        return (
            self.rng.gauss(0.0, float(self._parameter("x_noise_std")))
            + self.rng.gauss(0.0, float(self._parameter("random_walk_xy_std")) * math.sqrt(dt)),
            self.rng.gauss(0.0, float(self._parameter("y_noise_std")))
            + self.rng.gauss(0.0, float(self._parameter("random_walk_xy_std")) * math.sqrt(dt)),
            self.rng.gauss(0.0, float(self._parameter("yaw_noise_std")))
            + self.rng.gauss(0.0, float(self._parameter("random_walk_yaw_std")) * math.sqrt(dt)),
        )

    def odom_callback(self, message):
        pose = message.pose.pose
        current = (float(pose.position.x), float(pose.position.y))
        current_yaw = quaternion_to_yaw(pose.orientation)
        stamp_ns = self._stamp_ns(message)
        if self.true_previous is None:
            self.true_previous = current
            self.true_previous_yaw = current_yaw
            self.last_stamp_ns = stamp_ns
            self.sim_time_origin_ns = stamp_ns
            self.sim_state = SlipState(current[0], current[1], current_yaw)
            self._publish(message, (0.0, 0.0, 0.0))
            return
        dt = (stamp_ns - self.last_stamp_ns) / 1e9
        if dt <= 0.0 or dt > 2.0:
            self.get_logger().warn("Ignoring non-monotonic or excessive ground-truth odometry interval.")
            self.true_previous = current
            self.true_previous_yaw = current_yaw
            self.last_stamp_ns = stamp_ns
            return
        elapsed = (stamp_ns - self.sim_time_origin_ns) / 1e9
        x_scale, y_scale, yaw_scale = self._active_scales(elapsed)
        velocity = integrate_increment(
            self.sim_state,
            current[0] - self.true_previous[0],
            current[1] - self.true_previous[1],
            normalize_angle(current_yaw - self.true_previous_yaw),
            dt,
            x_scale, y_scale, yaw_scale,
            float(self._parameter("x_bias_per_second")) if bool(self._parameter("enabled")) else 0.0,
            float(self._parameter("y_bias_per_second")) if bool(self._parameter("enabled")) else 0.0,
            float(self._parameter("yaw_bias_per_second")) if bool(self._parameter("enabled")) else 0.0,
            self._noise(dt),
        )
        self.true_previous = current
        self.true_previous_yaw = current_yaw
        self.last_stamp_ns = stamp_ns
        self._publish(message, velocity)

    def _publish(self, source, velocity):
        output = Odometry()
        output.header = source.header
        output.header.frame_id = str(self._parameter("odom_frame"))
        output.child_frame_id = str(self._parameter("base_frame"))
        output.pose.pose.position.x = self.sim_state.x
        output.pose.pose.position.y = self.sim_state.y
        output.pose.pose.orientation.x, output.pose.pose.orientation.y, \
            output.pose.pose.orientation.z, output.pose.pose.orientation.w = yaw_to_quaternion(self.sim_state.yaw)
        output.twist.twist.linear.x = velocity[0]
        output.twist.twist.linear.y = velocity[1]
        output.twist.twist.angular.z = velocity[2]
        self.publisher.publish(output)
        transform = TransformStamped()
        transform.header = output.header
        transform.child_frame_id = output.child_frame_id
        transform.transform.translation.x = output.pose.pose.position.x
        transform.transform.translation.y = output.pose.pose.position.y
        transform.transform.rotation = output.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = OdomSlipSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
