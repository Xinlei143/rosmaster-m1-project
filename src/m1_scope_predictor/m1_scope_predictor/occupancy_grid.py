"""Conversions between SCOPE's x/y array convention and OccupancyGrid."""

import math

import numpy as np


def grid_origin_pose(laser_pose):
    x, y, yaw = [float(value) for value in laser_pose]
    lateral_offset = -3.2
    return (
        x - math.sin(yaw) * lateral_offset,
        y + math.cos(yaw) * lateral_offset,
        yaw,
    )


def _validated_grid(grid):
    value = np.asarray(grid)
    if value.shape != (1, 64, 64) or not np.isfinite(value).all():
        raise ValueError("grid must be finite with shape [1,64,64]")
    return value


def probability_data(grid):
    value = np.clip(_validated_grid(grid), 0.0, 1.0)
    return np.rint(value[0].T * 100.0).astype(np.int8).ravel().tolist()


def uncertainty_data(grid):
    value = np.clip(_validated_grid(grid) / 0.5, 0.0, 1.0)
    return np.rint(value[0].T * 100.0).astype(np.int8).ravel().tolist()


def endpoint_data(grid):
    value = _validated_grid(grid)
    return np.where(value[0].T > 0, 100, -1).astype(np.int8).ravel().tolist()
