"""WSL-safe approximate 2D LaserScan publisher.

The formal house/cafe benchmark uses Gazebo's GPU LiDAR.  This node is a
development fallback: it models scene bounds, configured primitive obstacles,
and the configured dynamic obstacle shapes without requiring a renderer.
"""

import math

import rclpy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

from imperative_navigation.scene_profiles import get_scene_profile


class SoftwareLidar(Node):
    """Raycast configured circles and oriented rectangles in the XY plane."""

    RANGE_MIN = 0.05
    RANGE_MAX = 12.0
    SAMPLE_COUNT = 667

    def __init__(self):
        super().__init__("software_lidar")
        self.declare_parameter("scene", "imperative_m1")
        self.declare_parameter("dynamic_obstacles_topic", "/imperative/dynamic_obstacles")
        self.declare_parameter("scan_topic", "/sim_scan")
        self.scene = str(self.get_parameter("scene").value).strip().lower()
        self.profile = get_scene_profile(self.scene)
        self.position = None
        self.yaw = 0.0
        self.dynamic_obstacles = []
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(
            PoseArray, self.get_parameter("dynamic_obstacles_topic").value,
            self.dynamic_callback, 10)
        self.publisher = self.create_publisher(
            LaserScan, self.get_parameter("scan_topic").value, sensor_qos)
        self.angle_min = -math.pi
        self.angle_increment = 2.0 * math.pi / self.SAMPLE_COUNT
        self.create_timer(1.0 / 12.0, self.publish_scan)
        if self.scene in {"house", "cafe"}:
            self.get_logger().warn(
                f"Software LiDAR for scene={self.scene} is an approximate fallback; "
                "formal tests must use GPU /scan.")
        self.get_logger().info(
            f"Software lidar ready for scene={self.scene}; "
            f"publishing {self.SAMPLE_COUNT} beams at 12 Hz on /sim_scan.")

    def odom_callback(self, message):
        pose = message.pose.pose
        self.position = (float(pose.position.x), float(pose.position.y))
        orientation = pose.orientation
        self.yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
        )

    def dynamic_callback(self, message):
        configured = self.profile["obstacles"]
        self.dynamic_obstacles = []
        for index, pose in enumerate(message.poses):
            obstacle = configured[index] if index < len(configured) else configured[-1]
            orientation = pose.orientation
            obstacle_yaw = math.atan2(
                2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
            )
            self.dynamic_obstacles.append({
                "shape": obstacle["shape"],
                "center": (float(pose.position.x), float(pose.position.y)),
                "size": obstacle["dimensions"][:2],
                "radius": float(obstacle["radius"]),
                "yaw": obstacle_yaw,
            })

    def ray_circle_distance(self, origin, direction, center, radius):
        offset_x = center[0] - origin[0]
        offset_y = center[1] - origin[1]
        projection = offset_x * direction[0] + offset_y * direction[1]
        perpendicular_squared = max(
            0.0, offset_x * offset_x + offset_y * offset_y - projection * projection)
        discriminant = radius * radius - perpendicular_squared
        if discriminant < 0.0:
            return self.RANGE_MAX
        root = math.sqrt(discriminant)
        near, far = projection - root, projection + root
        if near >= self.RANGE_MIN:
            return near
        if far >= self.RANGE_MIN:
            return far
        return self.RANGE_MAX

    def ray_box_distance(self, origin, direction, center, size, yaw=0.0):
        cosine, sine = math.cos(yaw), math.sin(yaw)
        relative_x, relative_y = origin[0] - center[0], origin[1] - center[1]
        local_origin = (cosine * relative_x + sine * relative_y,
                        -sine * relative_x + cosine * relative_y)
        local_direction = (cosine * direction[0] + sine * direction[1],
                           -sine * direction[0] + cosine * direction[1])
        half_x, half_y = float(size[0]) / 2.0, float(size[1]) / 2.0
        t_min, t_max = -math.inf, math.inf
        for coordinate, ray, half_extent in (
                (local_origin[0], local_direction[0], half_x),
                (local_origin[1], local_direction[1], half_y)):
            if abs(ray) < 1e-9:
                if abs(coordinate) > half_extent:
                    return self.RANGE_MAX
                continue
            first = (-half_extent - coordinate) / ray
            second = (half_extent - coordinate) / ray
            if first > second:
                first, second = second, first
            t_min, t_max = max(t_min, first), min(t_max, second)
            if t_min > t_max:
                return self.RANGE_MAX
        if t_min >= self.RANGE_MIN:
            return t_min
        if t_max >= self.RANGE_MIN:
            return t_max
        return self.RANGE_MAX

    def ray_wall_distance(self, origin, direction):
        min_x, max_x, min_y, max_y = self.profile["bounds"]
        candidates = []
        if direction[0] > 1e-9:
            candidates.append((max_x - origin[0]) / direction[0])
        elif direction[0] < -1e-9:
            candidates.append((min_x - origin[0]) / direction[0])
        if direction[1] > 1e-9:
            candidates.append((max_y - origin[1]) / direction[1])
        elif direction[1] < -1e-9:
            candidates.append((min_y - origin[1]) / direction[1])
        valid = [distance for distance in candidates if distance >= self.RANGE_MIN]
        return min(valid) if valid else self.RANGE_MAX

    def primitive_distance(self, origin, direction, primitive):
        if primitive["shape"] == "circle":
            return self.ray_circle_distance(
                origin, direction, primitive["center"], primitive["radius"])
        return self.ray_box_distance(
            origin, direction, primitive["center"], primitive["size"], primitive.get("yaw", 0.0))

    def dynamic_distance(self, origin, direction, obstacle):
        if obstacle["shape"] == "cylinder":
            return self.ray_circle_distance(
                origin, direction, obstacle["center"], obstacle["radius"])
        return self.ray_box_distance(
            origin, direction, obstacle["center"], obstacle["size"], obstacle["yaw"])

    def publish_scan(self):
        if self.position is None:
            return
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = "laser_Link"
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_min + (self.SAMPLE_COUNT - 1) * self.angle_increment
        scan.angle_increment = self.angle_increment
        scan.scan_time = 1.0 / 12.0
        scan.range_min = self.RANGE_MIN
        scan.range_max = self.RANGE_MAX
        ranges = []
        for index in range(self.SAMPLE_COUNT):
            heading = self.yaw + self.angle_min + index * self.angle_increment
            direction = (math.cos(heading), math.sin(heading))
            distance = self.ray_wall_distance(self.position, direction)
            for primitive in self.profile.get("static_primitives", []):
                distance = min(distance, self.primitive_distance(
                    self.position, direction, primitive))
            for obstacle in self.dynamic_obstacles:
                distance = min(distance, self.dynamic_distance(
                    self.position, direction, obstacle))
            ranges.append(max(self.RANGE_MIN, min(self.RANGE_MAX, distance)))
        scan.ranges = ranges
        self.publisher.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = SoftwareLidar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
