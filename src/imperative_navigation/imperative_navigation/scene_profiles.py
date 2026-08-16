"""Shared Gazebo scene profiles for launch, sensing, and repeatable tests."""

import math
from copy import deepcopy


_COMMON_OBSTACLE_MODELS = {
    "cylinder": {
        "model": "imperative_dynamic_cylinder",
        "radius": 0.30,
        "dimensions": [0.60, 0.60, 0.60],
    },
    "box": {
        "model": "imperative_dynamic_box",
        "radius": 0.39,
        "dimensions": [0.55, 0.55, 0.60],
    },
    "long_box": {
        "model": "imperative_dynamic_long_box",
        "radius": 0.55,
        "dimensions": [1.00, 0.45, 0.60],
    },
}


SCENE_PROFILES = {
    "imperative_m1": {
        "world_file": "imperative_m1.sdf",
        "world_name": "imperative_m1",
        "start": [-2.5, -1.5, 0.01, 0.0],
        "goal": [2.5, 1.5],
        "motion_mode": "continuous",
        "bounds": [-3.45, 3.45, -2.65, 2.65],
        "static_primitives": [
            {"shape": "circle", "center": [-2.05, -1.18], "radius": 0.30},
            {"shape": "circle", "center": [-3.0, 2.2], "radius": 0.30},
        ],
        "static_fallback": [-2.05, -1.18, 0.30, -3.0, 2.2, 0.30],
        "obstacles": [
            {
                "name": "moving_obstacle_1", "shape": "cylinder",
                "position": [2.35, 2.35, 0.35], "park": [10.0, 10.0, 0.35],
                "speed": 0.34, "acceleration": 0.45,
            },
            {
                "name": "moving_obstacle_2", "shape": "cylinder",
                "position": [3.10, 1.35, 0.35], "park": [11.0, 10.0, 0.35],
                "speed": 0.42, "acceleration": 0.50,
            },
            {
                "name": "moving_obstacle_3", "shape": "cylinder",
                "position": [2.40, 0.55, 0.35], "park": [12.0, 10.0, 0.35],
                "speed": 0.50, "acceleration": 0.55,
            },
        ],
    },
    "cafe": {
        "world_file": "cafe.world",
        "world_name": "cafe",
        "start": [-3.50, 6.40, 0.01, -math.pi / 2],
        "goal": [-3.50, -10.30],
        "motion_mode": "route",
        "bounds": [-4.90, 4.10, -10.70, 7.20],
        "static_primitives": [
            {"shape": "box", "center": [0.5, -1.6], "size": [0.913, 0.913], "yaw": 0.0},
            {"shape": "box", "center": [2.4, -5.5], "size": [0.913, 0.913], "yaw": 0.0},
            {"shape": "box", "center": [-1.5, -5.5], "size": [0.913, 0.913], "yaw": 0.0},
            {"shape": "box", "center": [2.4, -9.0], "size": [0.913, 0.913], "yaw": 0.0},
            {"shape": "box", "center": [-1.5, -9.0], "size": [0.913, 0.913], "yaw": 0.0},
        ],
        "static_fallback": [],
        "obstacles": [
            {
                "name": "moving_obstacle_1", "shape": "cylinder",
                "position": [-2.80, 5.80, 0.35], "park": [20.0, 20.0, 0.35],
                "speed": 0.30, "acceleration": 0.45,
                "route": [[-2.80, 5.80], [-2.80, -9.50], [-3.50, -9.50], [-3.50, 5.80]],
            },
            {
                "name": "moving_obstacle_2", "shape": "box",
                "position": [-1.80, -2.60, 0.30], "park": [21.0, 20.0, 0.30],
                "speed": 0.26, "acceleration": 0.40,
                "route": [[-1.80, -2.60], [1.80, -2.60], [1.80, -3.80], [-1.80, -3.80]],
            },
            {
                "name": "moving_obstacle_3", "shape": "long_box",
                "position": [3.40, 5.80, 0.30], "park": [22.0, 20.0, 0.30],
                "speed": 0.28, "acceleration": 0.40,
                "route": [[3.40, 5.80], [3.40, -9.50], [3.70, -9.50], [3.70, 5.80]],
            },
        ],
    },
    "house": {
        "world_file": "house.world",
        "world_name": "house",
        "start": [-3.70, 4.40, 0.01, -math.pi / 2],
        "goal": [-3.70, -4.40],
        "motion_mode": "route",
        "bounds": [-9.00, 9.00, -5.20, 5.20],
        "static_primitives": [],
        "static_fallback": [],
        "obstacles": [
            {
                "name": "moving_obstacle_1", "shape": "cylinder",
                "position": [-2.80, 3.80, 0.35], "park": [20.0, 20.0, 0.35],
                "speed": 0.24, "acceleration": 0.40,
                "route": [[-2.80, 3.80], [-2.80, -3.80], [-3.50, -3.80], [-3.50, 3.80]],
            },
            {
                "name": "moving_obstacle_2", "shape": "box",
                "position": [-1.70, 0.60, 0.30], "park": [21.0, 20.0, 0.30],
                "speed": 0.22, "acceleration": 0.35,
                "route": [[-1.70, 0.60], [1.70, 0.60], [1.70, -0.60], [-1.70, -0.60]],
            },
            {
                "name": "moving_obstacle_3", "shape": "long_box",
                "position": [3.20, 3.80, 0.30], "park": [22.0, 20.0, 0.30],
                "speed": 0.25, "acceleration": 0.38,
                "route": [[3.20, 3.80], [3.20, -3.80], [3.70, -3.80], [3.70, 3.80]],
            },
        ],
    },
}


def get_scene_profile(scene):
    """Return a mutable scene profile with derived obstacle geometry."""
    key = str(scene).strip().lower()
    if key not in SCENE_PROFILES:
        raise ValueError(f"Unknown scene {scene!r}; expected imperative_m1, cafe, or house")
    profile = deepcopy(SCENE_PROFILES[key])
    for obstacle in profile["obstacles"]:
        obstacle.update(deepcopy(_COMMON_OBSTACLE_MODELS[obstacle["shape"]]))
    profile["dynamic_radii"] = [float(obstacle["radius"]) for obstacle in profile["obstacles"]]
    return profile
