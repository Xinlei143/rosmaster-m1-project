"""Small, dependency-free 2-D frame helpers for localized Imperative goals."""

import math


def _transform_parts(transform):
    value = transform.transform if hasattr(transform, "transform") else transform
    translation = value.translation
    rotation = value.rotation
    yaw = math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
    )
    return float(translation.x), float(translation.y), yaw


def transform_point_2d(point, transform):
    """Apply a ``target <- source`` planar transform to ``(x, y)``."""

    tx, ty, yaw = _transform_parts(transform)
    cosine, sine = math.cos(yaw), math.sin(yaw)
    x, y = float(point[0]), float(point[1])
    return (cosine * x - sine * y + tx, sine * x + cosine * y + ty)


def inverse_transform_point_2d(point, transform):
    """Apply the inverse of a ``target <- source`` planar transform."""

    tx, ty, yaw = _transform_parts(transform)
    cosine, sine = math.cos(yaw), math.sin(yaw)
    x, y = float(point[0]) - tx, float(point[1]) - ty
    return (cosine * x + sine * y, -sine * x + cosine * y)


def _stamp_ns(transform):
    stamp = transform.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def resolve_goal_odom(goal_map, transform, now_ns, max_age):
    """Return a current odom-frame goal or ``None`` for unusable TF."""

    if transform is None:
        return None
    stamp_ns = _stamp_ns(transform)
    age = (int(now_ns) - stamp_ns) / 1e9
    if stamp_ns <= 0 or age < 0.0 or age > float(max_age):
        return None
    return transform_point_2d(goal_map, transform)

