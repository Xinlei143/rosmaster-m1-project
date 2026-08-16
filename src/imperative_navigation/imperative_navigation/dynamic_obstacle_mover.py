"""Drive the repeatable, scene-specific Gazebo dynamic obstacles."""

import math
import random
import time

import rclpy
from geometry_msgs.msg import Pose, PoseArray, Twist
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from tf2_msgs.msg import TFMessage

from imperative_navigation.scene_profiles import get_scene_profile


class DynamicObstacleMover(Node):
    """Command primitive obstacle models and publish their measured centers."""

    def __init__(self):
        super().__init__("dynamic_obstacle_mover")
        self.declare_parameter("scene", "imperative_m1")
        self.declare_parameter("control_period", 0.1)
        self.declare_parameter("random_seed", -1)
        self.declare_parameter("enabled", True)
        self.declare_parameter("motion_mode", "auto")
        self.declare_parameter("pose_service", "")
        self.declare_parameter("obstacle_topic", "/imperative/dynamic_obstacles")
        self.declare_parameter("pose_publish_period", 1.0 / 30.0)
        self.declare_parameter("gazebo_pose_topic", "/imperative/gazebo_dynamic_tf")

        self.scene = str(self.get_parameter("scene").value).strip().lower()
        self.profile = get_scene_profile(self.scene)
        self.period = max(0.01, float(self.get_parameter("control_period").value))
        self.enabled = bool(self.get_parameter("enabled").value)
        configured_mode = str(self.get_parameter("motion_mode").value).strip().lower()
        self.motion_mode = (self.profile["motion_mode"]
                            if configured_mode in {"", "auto"} else configured_mode)
        if self.motion_mode not in {"continuous", "route", "random_waypoint"}:
            self.get_logger().warn(
                f"Unknown motion_mode={self.motion_mode!r}; using {self.profile['motion_mode']}.")
            self.motion_mode = self.profile["motion_mode"]

        configured_seed = int(self.get_parameter("random_seed").value)
        self.seed = configured_seed if configured_seed >= 0 else time.time_ns() & 0xFFFFFFFF
        self.random = random.Random(self.seed)
        self.obstacles = self._prepare_obstacles(self.profile["obstacles"])
        self.obstacle_names = {obstacle["name"] for obstacle in self.obstacles}
        self.motion_time = 0.0
        self.parked = False
        self.last_service_warning = -1

        pose_service = str(self.get_parameter("pose_service").value).strip()
        if not pose_service:
            pose_service = f"/world/{self.profile['world_name']}/set_pose"
        gazebo_pose_topic = str(self.get_parameter("gazebo_pose_topic").value).strip()
        if not gazebo_pose_topic:
            gazebo_pose_topic = "/imperative/gazebo_dynamic_tf"

        self.velocity_publishers = {
            obstacle["name"]: self.create_publisher(
                Twist, f"/model/{obstacle['name']}/cmd_vel", 10)
            for obstacle in self.obstacles
        }
        self.client = self.create_client(SetEntityPose, pose_service)
        self.publisher = self.create_publisher(
            PoseArray, self.get_parameter("obstacle_topic").value, 10)
        self.actual_poses = {}
        self.create_subscription(TFMessage, gazebo_pose_topic,
                                 self.gazebo_pose_callback, 10)
        self.create_timer(self.period, self.step)
        self.create_timer(
            float(self.get_parameter("pose_publish_period").value),
            self.publish_actual_positions)
        self.get_logger().info(
            f"Dynamic obstacles ready: scene={self.scene}, enabled={self.enabled}, "
            f"mode={self.motion_mode}, seed={self.seed}, "
            f"shapes={[obstacle['shape'] for obstacle in self.obstacles]}")

    @staticmethod
    def _prepare_obstacles(configured):
        obstacles = []
        for configured_obstacle in configured:
            obstacle = dict(configured_obstacle)
            obstacle["position"] = list(obstacle["position"])
            obstacle["initial_position"] = list(obstacle["position"])
            obstacle["park"] = list(obstacle["park"])
            obstacle["route"] = [list(point) for point in obstacle.get("route", [])]
            obstacle["route_index"] = 1 if len(obstacle["route"]) > 1 else 0
            obstacle["velocity"] = [0.0, 0.0]
            obstacle["target_velocity"] = [0.0, 0.0]
            obstacle["yaw"] = 0.0
            obstacles.append(obstacle)
        return obstacles

    @staticmethod
    def distance(first, second):
        return math.hypot(first[0] - second[0], first[1] - second[1])

    def static_clearance(self, candidate):
        clearance = math.inf
        for primitive in self.profile.get("static_primitives", []):
            if primitive["shape"] == "circle":
                distance = self.distance(candidate, primitive["center"]) - primitive["radius"]
            else:
                half_x, half_y = (size / 2.0 for size in primitive["size"])
                distance = max(
                    abs(candidate[0] - primitive["center"][0]) - half_x,
                    abs(candidate[1] - primitive["center"][1]) - half_y,
                    0.0)
            clearance = min(clearance, distance)
        return clearance

    def is_safe_position(self, candidate, obstacle):
        min_x, max_x, min_y, max_y = self.profile["bounds"]
        radius = float(obstacle["radius"])
        if not (min_x + radius <= candidate[0] <= max_x - radius and
                min_y + radius <= candidate[1] <= max_y - radius):
            return False
        if self.static_clearance(candidate) < radius + 0.10:
            return False
        for other in self.obstacles:
            if other is obstacle:
                continue
            required = radius + float(other["radius"]) + 0.10
            if self.distance(candidate, other["position"]) < required:
                return False
        return True

    def sample_random_velocity(self, obstacle):
        speed = float(obstacle["speed"])
        heading = self.random.uniform(-math.pi, math.pi)
        return [speed * math.cos(heading), speed * math.sin(heading)]

    def send_pose(self, obstacle, position):
        if not self.client.service_is_ready():
            return
        request = SetEntityPose.Request()
        request.entity.name = obstacle["name"]
        request.entity.type = Entity.MODEL
        request.pose.position.x = float(position[0])
        request.pose.position.y = float(position[1])
        request.pose.position.z = float(position[2])
        request.pose.orientation.w = 1.0
        self.client.call_async(request)

    def send_velocity(self, obstacle, velocity_x, velocity_y):
        command = Twist()
        command.linear.x = float(velocity_x)
        command.linear.y = float(velocity_y)
        self.velocity_publishers[obstacle["name"]].publish(command)

    def gazebo_pose_callback(self, message):
        """Store world-frame poses from Gazebo's dynamic-pose publisher."""
        for transform in message.transforms:
            frame_names = {transform.child_frame_id,
                           transform.child_frame_id.strip("/").split("/")[-1]}
            name = next((candidate for candidate in frame_names
                         if candidate in self.obstacle_names), None)
            if name is None:
                continue
            pose = Pose()
            pose.position.x = transform.transform.translation.x
            pose.position.y = transform.transform.translation.y
            pose.position.z = transform.transform.translation.z
            pose.orientation = transform.transform.rotation
            self.actual_poses[name] = pose

    def publish_actual_positions(self):
        """Publish measured poses, with the configured pose as startup fallback."""
        message = PoseArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "odom"
        for obstacle in self.obstacles:
            pose = self.actual_poses.get(obstacle["name"])
            if pose is None:
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = obstacle["position"]
                pose.orientation.w = 1.0
            message.poses.append(pose)
        if self.enabled:
            self.publisher.publish(message)

    def update_route_motion(self, obstacle):
        route = obstacle.get("route", [])
        if len(route) < 2:
            self.update_continuous_motion(obstacle)
            return
        target = route[obstacle["route_index"]]
        offset_x = target[0] - obstacle["position"][0]
        offset_y = target[1] - obstacle["position"][1]
        distance = math.hypot(offset_x, offset_y)
        travel = float(obstacle["speed"]) * self.period
        if distance <= max(travel, 1e-6):
            obstacle["position"][0] = target[0]
            obstacle["position"][1] = target[1]
            obstacle["route_index"] = (obstacle["route_index"] + 1) % len(route)
            self.send_velocity(obstacle, 0.0, 0.0)
            return
        velocity = [float(obstacle["speed"]) * offset_x / distance,
                    float(obstacle["speed"]) * offset_y / distance]
        candidate = [obstacle["position"][0] + velocity[0] * self.period,
                     obstacle["position"][1] + velocity[1] * self.period]
        if not self.is_safe_position(candidate, obstacle):
            self.send_velocity(obstacle, 0.0, 0.0)
            return
        obstacle["position"][0], obstacle["position"][1] = candidate
        self.send_velocity(obstacle, velocity[0], velocity[1])

    def update_continuous_motion(self, obstacle):
        if self.motion_time >= obstacle.get("next_velocity_change", 0.0):
            obstacle["target_velocity"] = self.sample_random_velocity(obstacle)
            obstacle["next_velocity_change"] = self.motion_time + self.random.uniform(0.8, 2.8)
        velocity_x, velocity_y = obstacle["velocity"]
        target_x, target_y = obstacle["target_velocity"]
        delta_x, delta_y = target_x - velocity_x, target_y - velocity_y
        delta_norm = math.hypot(delta_x, delta_y)
        max_delta = float(obstacle["acceleration"]) * self.period
        if delta_norm <= max_delta:
            velocity_x, velocity_y = target_x, target_y
        elif delta_norm > 1e-9:
            scale = max_delta / delta_norm
            velocity_x += scale * delta_x
            velocity_y += scale * delta_y
        candidate = [obstacle["position"][0] + velocity_x * self.period,
                     obstacle["position"][1] + velocity_y * self.period]
        min_x, max_x, min_y, max_y = self.profile["bounds"]
        if candidate[0] < min_x or candidate[0] > max_x:
            velocity_x = -velocity_x
            candidate[0] = min(max(candidate[0], min_x), max_x)
        if candidate[1] < min_y or candidate[1] > max_y:
            velocity_y = -velocity_y
            candidate[1] = min(max(candidate[1], min_y), max_y)
        if not self.is_safe_position(candidate, obstacle):
            velocity_x, velocity_y = 0.0, 0.0
            candidate = list(obstacle["position"])
            obstacle["target_velocity"] = self.sample_random_velocity(obstacle)
        obstacle["velocity"] = [velocity_x, velocity_y]
        obstacle["position"] = candidate
        self.send_velocity(obstacle, velocity_x, velocity_y)

    def park_obstacles(self):
        if self.parked:
            return
        if not self.client.service_is_ready():
            now = self.get_clock().now().nanoseconds
            if self.last_service_warning < 0 or now - self.last_service_warning >= 1_000_000_000:
                self.last_service_warning = now
                self.get_logger().warn("Waiting for Gazebo set_pose service to park obstacles.")
            return
        for obstacle in self.obstacles:
            self.send_velocity(obstacle, 0.0, 0.0)
            obstacle["position"] = list(obstacle["park"])
            self.send_pose(obstacle, obstacle["park"])
        self.parked = True
        self.get_logger().info("Dynamic obstacles parked for the static-only scenario.")

    def step(self):
        if not self.enabled:
            self.park_obstacles()
            return
        for obstacle in self.obstacles:
            if self.motion_mode == "route":
                self.update_route_motion(obstacle)
            else:
                self.update_continuous_motion(obstacle)
        self.motion_time += self.period


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
