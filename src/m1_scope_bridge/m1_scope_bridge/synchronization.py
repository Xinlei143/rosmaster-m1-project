"""Timestamp synchronization utilities independent of ROS message classes."""

import math

import numpy as np


def normalize_angle(angle):
    """Wrap an angle or NumPy array to [-pi, pi]."""
    return np.arctan2(np.sin(angle), np.cos(angle))


def _validate_series(stamps_ns, poses, twists):
    stamps_ns = np.asarray(stamps_ns, dtype=np.int64)
    poses = np.asarray(poses, dtype=np.float64)
    twists = np.asarray(twists, dtype=np.float64)
    if stamps_ns.ndim != 1 or len(stamps_ns) < 2:
        raise ValueError("odometry requires at least two timestamped samples")
    if poses.shape != (len(stamps_ns), 3) or twists.shape != (len(stamps_ns), 3):
        raise ValueError("odometry pose and twist must have shape [N,3]")
    if np.any(np.diff(stamps_ns) <= 0):
        raise ValueError("odometry timestamps must be strictly increasing")
    return stamps_ns, poses, twists


def interpolate_odometry(stamps_ns, poses, twists, query_ns):
    """Linearly interpolate planar odometry without extrapolation.

    Yaw interpolation follows the shortest wrapped angular displacement.
    """
    stamps_ns, poses, twists = _validate_series(stamps_ns, poses, twists)
    query_ns = np.asarray(query_ns, dtype=np.int64)
    if query_ns.ndim != 1:
        raise ValueError("query timestamps must be one-dimensional")
    if len(query_ns) == 0:
        return np.empty((0, 3)), np.empty((0, 3))
    if query_ns.min() < stamps_ns[0] or query_ns.max() > stamps_ns[-1]:
        raise ValueError("query timestamp is outside odometry coverage")

    right = np.searchsorted(stamps_ns, query_ns, side="left")
    right = np.clip(right, 1, len(stamps_ns) - 1)
    exact_first = query_ns == stamps_ns[0]
    left = right - 1
    left[exact_first] = 0
    right[exact_first] = 1
    span = (stamps_ns[right] - stamps_ns[left]).astype(np.float64)
    alpha = (query_ns - stamps_ns[left]).astype(np.float64) / span
    alpha[exact_first] = 0.0

    output_pose = poses[left] + alpha[:, None] * (poses[right] - poses[left])
    yaw_delta = normalize_angle(poses[right, 2] - poses[left, 2])
    output_pose[:, 2] = normalize_angle(poses[left, 2] + alpha * yaw_delta)
    output_twist = twists[left] + alpha[:, None] * (twists[right] - twists[left])
    return output_pose, output_twist


def nearest_time_offsets(reference_ns, query_ns):
    """Return absolute offsets to the nearest reference timestamp."""
    reference_ns = np.asarray(reference_ns, dtype=np.int64)
    query_ns = np.asarray(query_ns, dtype=np.int64)
    if reference_ns.ndim != 1 or len(reference_ns) == 0:
        raise ValueError("reference timestamps must be a non-empty vector")
    if np.any(np.diff(reference_ns) < 0):
        raise ValueError("reference timestamps must be sorted")
    indices = np.searchsorted(reference_ns, query_ns, side="left")
    later = np.clip(indices, 0, len(reference_ns) - 1)
    earlier = np.clip(indices - 1, 0, len(reference_ns) - 1)
    earlier_error = np.abs(query_ns - reference_ns[earlier])
    later_error = np.abs(reference_ns[later] - query_ns)
    return np.minimum(earlier_error, later_error)


def common_coverage_mask(query_ns, *timestamp_series):
    """Select queries bracketed by every required timestamp series."""
    query_ns = np.asarray(query_ns, dtype=np.int64)
    if not timestamp_series:
        raise ValueError("at least one timestamp series is required")
    lower = None
    upper = None
    for series in timestamp_series:
        series = np.asarray(series, dtype=np.int64)
        if series.ndim != 1 or len(series) == 0:
            raise ValueError("timestamp series must be non-empty vectors")
        current_lower = int(series[0])
        current_upper = int(series[-1])
        lower = current_lower if lower is None else max(lower, current_lower)
        upper = current_upper if upper is None else min(upper, current_upper)
    if lower > upper:
        return np.zeros(query_ns.shape, dtype=np.bool_)
    return (query_ns >= lower) & (query_ns <= upper)


def quaternion_to_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))
