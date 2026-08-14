"""Randomly move Gazebo obstacles and publish their simulation-truth centers.

This node exists solely for the Gazebo WSLg test.  A real robot must obtain
obstacle centers from its sensor pipeline, never from a simulator pose service.
"""

import math
import random
import time

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose


class DynamicObstacleMover(Node):
    """Move three collision cylinders between independently sampled waypoints."""

    def __init__(self):
        super().__init__("dynamic_obstacle_mover")
        self.declare_parameter("control_period", 0.1)
        self.declare_parameter("random_seed", -1)
        self.declare_parameter("move_obstacles", True)
        self.declare_parameter("pose_service", "/world/imperative_m1/set_pose")
        self.declare_parameter("obstacle_topic", "/imperative/dynamic_obstacles")

        self.period = float(self.get_parameter("control_period").value)
        configured_seed = int(self.get_parameter("random_seed").value)
        self.move_obstacles = bool(self.get_parameter("move_obstacles").value)
        self.seed = configured_seed if configured_seed >= 0 else time.time_ns() & 0xFFFFFFFF
        self.random = random.Random(self.seed)
        # (name, starting x/y, m/s).  The cylinders begin around the goal, then
        # make one randomized approach into the robot's route before switching
        # to unrestricted random patrols.
        self.obstacles = [
            {"name": "moving_obstacle_1", "position": [2.35, 2.35], "speed": 0.34},
            {"name": "moving_obstacle_2", "position": [3.10, 1.35], "speed": 0.42},
            {"name": "moving_obstacle_3", "position": [2.40, 0.55], "speed": 0.50},
        ]
        self.static_centers = [(-2.05, -1.18), (-3.0, 2.2)]
        for index, obstacle in enumerate(self.obstacles):
            obstacle["target"] = self.sample_approach_target(obstacle["position"], index)
            obstacle["approaching"] = True

        self.client = self.create_client(
            SetEntityPose, self.get_parameter("pose_service").value)
        self.publisher = self.create_publisher(
            PoseArray, self.get_parameter("obstacle_topic").value, 10)
        self.last_service_warning = -1
        self.create_timer(self.period, self.step)
        self.get_logger().info(
            f"Obstacle controller ready (moving={self.move_obstacles}, seed={self.seed}).")

    @staticmethod
    def distance(first, second):
        return math.hypot(first[0] - second[0], first[1] - second[1])

    @staticmethod
    def point_to_segment_distance(point, start, end):
        segment_x, segment_y = end[0] - start[0], end[1] - start[1]
        length_squared = segment_x * segment_x + segment_y * segment_y
        if length_squared <= 1e-9:
            return DynamicObstacleMover.distance(point, start)
        fraction = ((point[0] - start[0]) * segment_x +
                    (point[1] - start[1]) * segment_y) / length_squared
        fraction = min(1.0, max(0.0, fraction))
        closest = [start[0] + fraction * segment_x, start[1] + fraction * segment_y]
        return DynamicObstacleMover.distance(point, closest)

    def is_safe_target(self, current, candidate):
        # 0.72 m is the two cylinder radii plus a small path clearance.
        if any(self.distance(candidate, center) < 0.72 for center in self.static_centers):
            return False
        if any(self.point_to_segment_distance(center, current, candidate) < 0.72
               for center in self.static_centers):
            return False
        return not any(self.distance(candidate, other["position"]) < 0.80
                       for other in self.obstacles)

    def sample_target(self, current):
        """Sample a reachable target with collision clearance in the 8 x 6.4 m room."""
        for _ in range(100):
            candidate = [self.random.uniform(-3.45, 3.45), self.random.uniform(-2.65, 2.65)]
            if self.distance(candidate, current) < 0.9:
                continue
            if self.is_safe_target(current, candidate):
                return candidate
        # A sampling failure is extremely unlikely; retain the old point so no
        # unsafe, arbitrary jump is introduced.
        return list(current)

    def sample_approach_target(self, current, index):
        """Randomize the first encounter in the robot's usual start-to-goal corridor."""
        centers = [(-0.65, -1.45), (0.05, -0.85), (0.75, -0.25)]
        center = centers[index]
        for _ in range(100):
            candidate = [self.random.uniform(center[0] - 0.35, center[0] + 0.35),
                         self.random.uniform(center[1] - 0.30, center[1] + 0.30)]
            if self.is_safe_target(current, candidate):
                return candidate
        return self.sample_target(current)

    def send_pose(self, obstacle):
        request = SetEntityPose.Request()
        request.entity.name = obstacle["name"]
        request.entity.type = Entity.MODEL
        request.pose.position.x = obstacle["position"][0]
        request.pose.position.y = obstacle["position"][1]
        request.pose.position.z = 0.3
        request.pose.orientation.w = 1.0
        self.client.call_async(request)

    def publish_positions(self):
        message = PoseArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "odom"
        for obstacle in self.obstacles:
            pose = Pose()
            pose.position.x = obstacle["position"][0]
            pose.position.y = obstacle["position"][1]
            pose.position.z = 0.3
            pose.orientation.w = 1.0
            message.poses.append(pose)
        self.publisher.publish(message)

    def step(self):
        if not self.client.service_is_ready():
            now = self.get_clock().now().nanoseconds
            if self.last_service_warning < 0 or now - self.last_service_warning >= 1_000_000_000:
                self.last_service_warning = now
                self.get_logger().warn("Waiting for Gazebo /set_pose service bridge.")
            return

        for obstacle in self.obstacles:
            if self.move_obstacles:
                offset_x = obstacle["target"][0] - obstacle["position"][0]
                offset_y = obstacle["target"][1] - obstacle["position"][1]
                distance = math.hypot(offset_x, offset_y)
                travel = obstacle["speed"] * self.period
                if distance <= travel:
                    obstacle["position"] = list(obstacle["target"])
                    obstacle["target"] = self.sample_target(obstacle["position"])
                    obstacle["approaching"] = False
                else:
                    obstacle["position"][0] += travel * offset_x / distance
                    obstacle["position"][1] += travel * offset_y / distance
            self.send_pose(obstacle)
        self.publish_positions()


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
