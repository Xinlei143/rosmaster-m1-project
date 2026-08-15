"""Continuously move Gazebo obstacles and publish Gazebo-measured centers.

This node exists solely for the Gazebo WSLg test.  A real robot must obtain
obstacle centers from its sensor pipeline, never from a simulator pose service.
"""

import math
import random
import time

import rclpy
from geometry_msgs.msg import Pose, PoseArray, Twist
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from tf2_msgs.msg import TFMessage


class DynamicObstacleMover(Node):
    """Command three Gazebo velocity-controlled cylinders between waypoints."""

    def __init__(self):
        super().__init__("dynamic_obstacle_mover")
        self.declare_parameter("control_period", 0.1)
        self.declare_parameter("random_seed", -1)
        self.declare_parameter("enabled", True)
        self.declare_parameter("pose_service", "/world/imperative_m1/set_pose")
        self.declare_parameter("obstacle_topic", "/imperative/dynamic_obstacles")
        self.declare_parameter("pose_publish_period", 1.0 / 30.0)
        self.declare_parameter("gazebo_pose_topic", "/imperative/gazebo_dynamic_tf")

        self.period = float(self.get_parameter("control_period").value)
        self.enabled = bool(self.get_parameter("enabled").value)
        configured_seed = int(self.get_parameter("random_seed").value)
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

        # Gazebo's VelocityControl system owns the continuous motion.  The
        # controller sends a direction at a modest ROS rate, while Gazebo
        # integrates that velocity at every physics step.  This avoids the
        # old 10 Hz set_pose teleports, which were especially visible on WSLg.
        self.velocity_publishers = {
            obstacle["name"]: self.create_publisher(
                Twist, f"/model/{obstacle['name']}/cmd_vel", 10)
            for obstacle in self.obstacles
        }
        # This service is retained only to park the models in static-only
        # scenarios. It is never used while the obstacles are moving.
        self.client = self.create_client(
            SetEntityPose, self.get_parameter("pose_service").value)
        self.publisher = self.create_publisher(
            PoseArray, self.get_parameter("obstacle_topic").value, 10)
        self.actual_poses = {}
        self.obstacle_names = {obstacle["name"] for obstacle in self.obstacles}
        self.create_subscription(
            TFMessage, self.get_parameter("gazebo_pose_topic").value,
            self.gazebo_pose_callback, 10)
        self.last_service_warning = -1
        self.parked = False
        self.create_timer(self.period, self.step)
        self.create_timer(
            float(self.get_parameter("pose_publish_period").value), self.publish_actual_positions)
        self.get_logger().info(
            f"Dynamic-obstacle controller ready (enabled={self.enabled}, seed={self.seed}).")

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
        request.pose.position.z = 0.35
        request.pose.orientation.w = 1.0
        self.client.call_async(request)

    def send_velocity(self, obstacle, velocity_x, velocity_y):
        command = Twist()
        command.linear.x = velocity_x
        command.linear.y = velocity_y
        self.velocity_publishers[obstacle["name"]].publish(command)

    def gazebo_pose_callback(self, message):
        """Store world-frame poses from Gazebo's dynamic-pose publisher."""
        for transform in message.transforms:
            name = transform.child_frame_id
            if name not in self.obstacle_names:
                continue
            pose = Pose()
            pose.position.x = transform.transform.translation.x
            pose.position.y = transform.transform.translation.y
            pose.position.z = transform.transform.translation.z
            pose.orientation = transform.transform.rotation
            self.actual_poses[name] = pose

    def publish_actual_positions(self):
        """Publish Gazebo's current model poses, not the controller estimate."""
        if len(self.actual_poses) != len(self.obstacles):
            return
        message = PoseArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "odom"
        for obstacle in self.obstacles:
            message.poses.append(self.actual_poses[obstacle["name"]])
        self.publisher.publish(message)

    def step(self):
        if not self.enabled:
            # Keep the dynamic models from contaminating a static-only test.
            # They remain in the shared SDF but are parked outside the room and
            # an empty truth message is published for the software lidar/logger.
            if not self.parked:
                if not self.client.service_is_ready():
                    now = self.get_clock().now().nanoseconds
                    if self.last_service_warning < 0 or now - self.last_service_warning >= 1_000_000_000:
                        self.last_service_warning = now
                        self.get_logger().warn("Waiting for Gazebo /set_pose service bridge to park obstacles.")
                    return
                for index, obstacle in enumerate(self.obstacles):
                    self.send_velocity(obstacle, 0.0, 0.0)
                    obstacle["position"] = [10.0 + index, 10.0]
                    self.send_pose(obstacle)
                self.parked = True
                self.get_logger().info("Dynamic obstacles parked for the static-only scenario.")
            message = PoseArray()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = "odom"
            self.publisher.publish(message)
            return

        for obstacle in self.obstacles:
            offset_x = obstacle["target"][0] - obstacle["position"][0]
            offset_y = obstacle["target"][1] - obstacle["position"][1]
            distance = math.hypot(offset_x, offset_y)
            travel = obstacle["speed"] * self.period
            if distance <= travel:
                obstacle["position"] = list(obstacle["target"])
                self.send_velocity(obstacle, 0.0, 0.0)
                obstacle["target"] = self.sample_target(obstacle["position"])
                obstacle["approaching"] = False
            else:
                velocity_x = obstacle["speed"] * offset_x / distance
                velocity_y = obstacle["speed"] * offset_y / distance
                self.send_velocity(obstacle, velocity_x, velocity_y)
                obstacle["position"][0] += velocity_x * self.period
                obstacle["position"][1] += velocity_y * self.period


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
