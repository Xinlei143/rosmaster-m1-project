"""Record per-wheel Gazebo contact points and wheel-joint force-torque data.

Gazebo Fortress contact messages provide contact locations, while the
force-torque sensor on each wheel joint provides the wrench used here.  In the
parent wheel-joint frame, the horizontal force magnitude is the simulated
friction force and the z component is the normal force.
"""

import csv
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import Wrench
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy._rclpy_pybind11 import RCLError
from ros_gz_interfaces.msg import Contacts


WHEELS = ("le_fr", "le_be", "ri_fr", "ri_be")
FIELDS = [
    "stamp", "wheel", "contact_index", "collision1", "collision2",
    "position_x", "position_y", "position_z", "normal_x", "normal_y",
    "normal_z", "depth", "force_x", "force_y", "force_z", "force_norm",
    "normal_force", "friction_force", "torque_x", "torque_y", "torque_z",
]
FT_FIELDS = [
    "stamp", "wheel", "force_x", "force_y", "force_z", "force_norm",
    "normal_force", "friction_force", "torque_x", "torque_y", "torque_z",
]


def stamp_to_seconds(stamp):
    return float(stamp.sec) + 1e-9 * float(stamp.nanosec)


def vector_norm(vector):
    return math.sqrt(float(vector.x) ** 2 + float(vector.y) ** 2 + float(vector.z) ** 2)


def vector_dot(first, second):
    return (float(first.x) * float(second.x) +
            float(first.y) * float(second.y) +
            float(first.z) * float(second.z))


def vector_subtract(first, second, scale):
    return (
        float(first.x) - scale * float(second.x),
        float(first.y) - scale * float(second.y),
        float(first.z) - scale * float(second.z),
    )


def safe_name(value):
    if hasattr(value, "name"):
        return str(value.name)
    if hasattr(value, "data"):
        return str(value.data)
    return str(value)


class ContactForceLogger(Node):
    """Write one row per wheel contact at every received Gazebo message."""

    def __init__(self):
        super().__init__("contact_force_logger")
        self.declare_parameter("output_dir", "/tmp/imperative_m1_contact_force")
        self.declare_parameter("topic_prefix", "/m1/contact")
        self.declare_parameter(
            "force_topic_prefix",
            "/world/imperative_m1_empty/model/m1/joint",
        )
        self.output_dir = Path(str(self.get_parameter("output_dir").value)).expanduser()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        existing_files = list(self.output_dir.iterdir())
        if existing_files:
            raise RuntimeError(
                f"Contact-force output directory is not empty: {self.output_dir}. "
                "Use a new output_dir for each run.")

        self.path = self.output_dir / "contact_forces.csv"
        self.csv_handle = self.path.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.csv_handle, fieldnames=FIELDS)
        self.writer.writeheader()
        self.ft_path = self.output_dir / "wheel_force_torque.csv"
        self.ft_handle = self.ft_path.open("w", newline="", encoding="utf-8")
        self.ft_writer = csv.DictWriter(self.ft_handle, fieldnames=FT_FIELDS)
        self.ft_writer.writeheader()
        self.rows = 0

        prefix = str(self.get_parameter("topic_prefix").value).rstrip("/")
        force_prefix = str(self.get_parameter("force_topic_prefix").value).rstrip("/")
        for wheel in WHEELS:
            self.create_subscription(
                Contacts, f"{prefix}/{wheel}",
                lambda message, wheel_name=wheel: self.contacts_callback(wheel_name, message),
                100,
            )
            joint = f"{wheel}_joint"
            force_topic = (
                f"{force_prefix}/{joint}/sensor/{wheel}_force_torque/forcetorque")
            self.create_subscription(
                Wrench, force_topic,
                lambda message, wheel_name=wheel: self.force_torque_callback(
                    wheel_name, message),
                100,
            )
        self.get_logger().info(
            f"Recording wheel contact forces in {self.path} and joint wrenches in {self.ft_path}")

    @staticmethod
    def _wheel_force(contact, wheel):
        """Select the wrench associated with this wheel collision."""
        if not contact.wrenches:
            return None
        wheel_token = f"{wheel}_Link".lower()
        for body_name, wrench in (
            (safe_name(contact.wrenches[0].body_1_name), contact.wrenches[0].body_1_wrench),
            (safe_name(contact.wrenches[0].body_2_name), contact.wrenches[0].body_2_wrench),
        ):
            if wheel_token in body_name.lower():
                return wrench
        # Gazebo normally puts the sensor collision in body_1. Keep a
        # deterministic fallback for versions that omit body names.
        return contact.wrenches[0].body_1_wrench

    def contacts_callback(self, wheel, message):
        stamp = stamp_to_seconds(message.header.stamp)
        for contact_index, contact in enumerate(message.contacts):
            wrench = self._wheel_force(contact, wheel)
            if wrench is None:
                continue
            force = wrench.force
            force_norm = vector_norm(force)
            if contact.normals:
                normal = contact.normals[0]
                normal_norm = vector_norm(normal)
                if normal_norm > 1e-12:
                    nx = float(normal.x) / normal_norm
                    ny = float(normal.y) / normal_norm
                    nz = float(normal.z) / normal_norm
                    normal_unit = type("Normal", (), {"x": nx, "y": ny, "z": nz})()
                    normal_component = vector_dot(force, normal_unit)
                    tangent = vector_subtract(force, normal_unit, normal_component)
                    normal_force = abs(normal_component)
                    friction_force = math.sqrt(sum(value * value for value in tangent))
                else:
                    nx = ny = nz = math.nan
                    normal_force = math.nan
                    friction_force = math.nan
            else:
                nx = ny = nz = math.nan
                normal_force = math.nan
                friction_force = math.nan

            position = contact.positions[0] if contact.positions else None
            depth = contact.depths[0] if contact.depths else math.nan
            self.writer.writerow({
                "stamp": stamp,
                "wheel": wheel,
                "contact_index": contact_index,
                "collision1": safe_name(contact.collision1),
                "collision2": safe_name(contact.collision2),
                "position_x": float(position.x) if position else math.nan,
                "position_y": float(position.y) if position else math.nan,
                "position_z": float(position.z) if position else math.nan,
                "normal_x": nx,
                "normal_y": ny,
                "normal_z": nz,
                "depth": float(depth),
                "force_x": float(force.x),
                "force_y": float(force.y),
                "force_z": float(force.z),
                "force_norm": force_norm,
                "normal_force": normal_force,
                "friction_force": friction_force,
                "torque_x": float(wrench.torque.x),
                "torque_y": float(wrench.torque.y),
                "torque_z": float(wrench.torque.z),
            })
            self.csv_handle.flush()
            self.rows += 1

    def force_torque_callback(self, wheel, message):
        """Record the horizontal (tangential) and vertical (normal) force.

        The force-torque sensors are configured in the parent wheel-joint
        frame.  That frame is the base frame before wheel rotation, so z is
        vertical and sqrt(x^2 + y^2) is the ground-plane friction magnitude.
        """
        force = message.force
        torque = message.torque
        force_norm = vector_norm(force)
        normal_force = abs(float(force.z))
        friction_force = math.hypot(float(force.x), float(force.y))
        stamp = self.get_clock().now().nanoseconds * 1e-9
        self.ft_writer.writerow({
            "stamp": stamp,
            "wheel": wheel,
            "force_x": float(force.x),
            "force_y": float(force.y),
            "force_z": float(force.z),
            "force_norm": force_norm,
            "normal_force": normal_force,
            "friction_force": friction_force,
            "torque_x": float(torque.x),
            "torque_y": float(torque.y),
            "torque_z": float(torque.z),
        })
        self.ft_handle.flush()

    def close(self):
        if not self.csv_handle.closed:
            self.csv_handle.flush()
            self.csv_handle.close()
        if not self.ft_handle.closed:
            self.ft_handle.flush()
            self.ft_handle.close()


def main(args=None):
    rclpy.init(args=args)
    node = ContactForceLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
