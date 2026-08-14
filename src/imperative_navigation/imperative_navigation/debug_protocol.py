"""Stable field layout for the planner experiment debug topic."""


PLANNER_DEBUG_FIELDS = (
    "stamp",
    "goal_x",
    "goal_y",
    "position_x",
    "position_y",
    "yaw",
    "planner_accel_x",
    "planner_accel_y",
    "command_world_vx",
    "command_world_vy",
    "command_body_vx",
    "command_body_vy",
    "planner_dt",
    "scan_hits",
    "scan_min_range",
    "dynamic_tracks",
    "goal_distance",
    "stopped",
    "scan_saturated",
)
