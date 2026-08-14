"""WSLg-safe 2D laser scan publisher for the Gazebo test world.

Gazebo Sim 6's GPU lidar needs a rendering backend. On this WSLg host OGRE
returns its near clip distance for every ray, while OGRE2 fails during texture
copy. This node raycasts the same Gazebo scene geometry in software and
publishes a normal ``sensor_msgs/LaserScan`` so the controller's ordinary scan
clustering, tracking, and planning path can be exercised.
"""

import math

import rclpy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class SoftwareLidar(Node):
    """Raycast the room walls and circular obstacles from measured odometry."""

    # T-MINI PLUS simulation profile: 0.54 deg, 12 Hz, 0.05--12 m.
    RANGE_MIN = 0.05
    RANGE_MAX = 12.0
    OBSTACLE_RADIUS = 0.30
    X_MIN, X_MAX = -4.0, 4.0
    Y_MIN, Y_MAX = -3.2, 3.2
    STATIC_CENTERS = [(-2.05, -1.18), (-3.0, 2.2)]

    def __init__(self):
        super().__init__("software_lidar")
        self.declare_parameter("dynamic_obstacles_topic", "/imperative/dynamic_obstacles")
        self.declare_parameter("scan_topic", "/sim_scan")
        self.position = None
        self.yaw = 0.0
        self.dynamic_centers = []
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(
            PoseArray, self.get_parameter("dynamic_obstacles_topic").value,
            self.dynamic_callback, 10)
        self.publisher = self.create_publisher(
            LaserScan, self.get_parameter("scan_topic").value, sensor_qos)
        self.sample_count = 667
        self.angle_min = -math.pi
        self.angle_increment = 2.0 * math.pi / self.sample_count
        self.create_timer(1.0 / 12.0, self.publish_scan)
        self.get_logger().info(
            "Software lidar ready; publishing T-MINI PLUS profile "
            "(667 beams, 12 Hz, 0.05-12 m) on /sim_scan.")

    def odom_callback(self, message):
        pose = message.pose.pose
        self.position = (pose.position.x, pose.position.y)
        orientation = pose.orientation
        self.yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
        )

    def dynamic_callback(self, message):
        self.dynamic_centers = [(pose.position.x, pose.position.y) for pose in message.poses]

    def ray_circle_distance(self, direction, center):
        offset_x = center[0] - self.position[0]
        offset_y = center[1] - self.position[1]
        projection = offset_x * direction[0] + offset_y * direction[1]
        perpendicular_squared = offset_x * offset_x + offset_y * offset_y - projection * projection
        discriminant = self.OBSTACLE_RADIUS ** 2 - perpendicular_squared
        if discriminant < 0.0:
            return self.RANGE_MAX
        near = projection - math.sqrt(discriminant)
        far = projection + math.sqrt(discriminant)
        if near >= self.RANGE_MIN:
            return near
        if far >= self.RANGE_MIN:
            return far
        return self.RANGE_MAX

    def ray_wall_distance(self, direction):
        candidates = []
        if direction[0] > 1e-9:
            candidates.append((self.X_MAX - self.position[0]) / direction[0])
        elif direction[0] < -1e-9:
            candidates.append((self.X_MIN - self.position[0]) / direction[0])
        if direction[1] > 1e-9:
            candidates.append((self.Y_MAX - self.position[1]) / direction[1])
        elif direction[1] < -1e-9:
            candidates.append((self.Y_MIN - self.position[1]) / direction[1])
        return min(distance for distance in candidates if distance >= self.RANGE_MIN)

    def publish_scan(self):
        if self.position is None:
            return
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = "laser_Link"
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_min + (self.sample_count - 1) * self.angle_increment
        scan.angle_increment = self.angle_increment
        scan.scan_time = 1.0 / 12.0
        scan.range_min = self.RANGE_MIN
        scan.range_max = self.RANGE_MAX
        centers = self.STATIC_CENTERS + self.dynamic_centers
        ranges = []
        for index in range(self.sample_count):
            heading = self.yaw + self.angle_min + index * self.angle_increment
            direction = (math.cos(heading), math.sin(heading))
            distance = self.ray_wall_distance(direction)
            for center in centers:
                distance = min(distance, self.ray_circle_distance(direction, center))
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
