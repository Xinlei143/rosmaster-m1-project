"""Numerically stable planar rigid-body transformations."""

import math

import numpy as np


def normalize_angle(angle):
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


def pose_to_matrix(pose):
    x, y, yaw = [float(value) for value in pose]
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cosine, -sine, x],
        [sine, cosine, y],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)


def matrix_to_pose(transform):
    transform = np.asarray(transform, dtype=np.float64)
    return np.array([
        transform[0, 2], transform[1, 2],
        math.atan2(transform[1, 0], transform[0, 0]),
    ], dtype=np.float64)


def invert_transform(transform):
    transform = np.asarray(transform, dtype=np.float64)
    rotation = transform[:2, :2]
    translation = transform[:2, 2]
    inverse = np.eye(3, dtype=np.float64)
    inverse[:2, :2] = rotation.T
    inverse[:2, 2] = -rotation.T.dot(translation)
    return inverse


def transform_points(transform, points):
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 2:
        raise ValueError("points must have shape [N,2]")
    if len(points) == 0:
        return np.empty((0, 2), dtype=np.float64)
    return points.dot(np.asarray(transform)[:2, :2].T) + transform[:2, 2]


def integrate_body_twist(pose, twist, dt, angular_epsilon=1e-8):
    """Integrate a constant body-frame [vx, vy, wz] with the SE(2) exponential."""
    pose = np.asarray(pose, dtype=np.float64)
    vx, vy, omega = [float(value) for value in twist]
    dt = float(dt)
    if dt < 0.0:
        raise ValueError("integration interval must be non-negative")
    angle = omega * dt
    if abs(omega) < angular_epsilon:
        dx_body = vx * dt
        dy_body = vy * dt
    else:
        sine = math.sin(angle)
        one_minus_cosine = 1.0 - math.cos(angle)
        dx_body = sine / omega * vx - one_minus_cosine / omega * vy
        dy_body = one_minus_cosine / omega * vx + sine / omega * vy
    cosine, sine = math.cos(pose[2]), math.sin(pose[2])
    return np.array([
        pose[0] + cosine * dx_body - sine * dy_body,
        pose[1] + sine * dx_body + cosine * dy_body,
        normalize_angle(pose[2] + angle),
    ], dtype=np.float64)
