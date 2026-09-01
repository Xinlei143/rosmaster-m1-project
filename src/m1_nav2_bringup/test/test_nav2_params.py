"""Regression tests for the GPU-LiDAR Nav2 costmap observation settings."""

from pathlib import Path

import yaml


PARAMS = Path(__file__).parents[1] / "config" / "nav2_params.yaml"
GAZEBO_LAUNCH = Path(__file__).parents[1] / "launch" / "nav2_m1_gazebo.launch.py"


def costmap_scan_source(costmap_name):
    parameters = yaml.safe_load(PARAMS.read_text())[costmap_name][costmap_name][
        "ros__parameters"
    ]
    return parameters["obstacle_layer"]["scan"]


def controller_follow_path():
    parameters = yaml.safe_load(PARAMS.read_text())["controller_server"][
        "ros__parameters"
    ]
    return parameters["FollowPath"]


def velocity_smoother_parameters():
    return yaml.safe_load(PARAMS.read_text())["velocity_smoother"][
        "ros__parameters"
    ]


def controller_parameters():
    return yaml.safe_load(PARAMS.read_text())["controller_server"][
        "ros__parameters"
    ]


def planner_parameters():
    return yaml.safe_load(PARAMS.read_text())["planner_server"][
        "ros__parameters"
    ]


def collision_monitor_parameters():
    return yaml.safe_load(PARAMS.read_text())["collision_monitor"][
        "ros__parameters"
    ]


def costmap_parameters(costmap_name):
    return yaml.safe_load(PARAMS.read_text())[costmap_name][costmap_name][
        "ros__parameters"
    ]


def test_gpu_lidar_scan_has_an_explicit_observation_height_window():
    """Laser returns transformed above z=0 must not be filtered by defaults."""
    for costmap_name in ("local_costmap", "global_costmap"):
        scan = costmap_scan_source(costmap_name)
        assert scan["min_obstacle_height"] == 0.0
        assert scan["max_obstacle_height"] >= 0.175


def test_nav2_gazebo_defaults_to_native_gpu_lidar():
    launch_source = GAZEBO_LAUNCH.read_text()
    assert '"software_lidar", default_value="false"' in launch_source


def test_gpu_lidar_scan_marks_and_clears_both_costmaps():
    for costmap_name in ("local_costmap", "global_costmap"):
        scan = costmap_scan_source(costmap_name)
        assert scan["data_type"] == "LaserScan"
        assert scan["marking"] is True
        assert scan["clearing"] is True


def test_local_costmap_has_a_dynamic_obstacle_lookahead_window():
    parameters = costmap_parameters("local_costmap")
    scan = parameters["obstacle_layer"]["scan"]

    assert parameters["rolling_window"] is True
    assert parameters["width"] == 3
    assert parameters["height"] == 3
    assert scan["obstacle_max_range"] == 4.0
    assert scan["raytrace_max_range"] == 4.5
    assert parameters["inflation_layer"]["inflation_radius"] == 0.4
    assert parameters["always_send_full_costmap"] is False


def test_mppi_uses_supported_omni_velocity_constraints():
    follow_path = controller_follow_path()
    controller = controller_parameters()

    assert follow_path["plugin"] == "nav2_mppi_controller::MPPIController"
    assert follow_path["motion_model"] == "Omni"
    assert controller["controller_frequency"] == 20.0
    assert follow_path["time_steps"] == 40
    assert follow_path["model_dt"] == 0.05
    assert follow_path["batch_size"] == 500
    assert follow_path["iteration_count"] == 1
    assert follow_path["vx_std"] == 0.3
    assert follow_path["vy_std"] == 0.3
    assert follow_path["wz_std"] == 0.5
    assert follow_path["vx_max"] == 1.0
    assert follow_path["vx_min"] == -1.0
    assert follow_path["vy_max"] == 1.0
    assert follow_path["wz_max"] == 0.8

    for unsupported_parameter in (
        "vy_min",
        "ax_max",
        "ax_min",
        "ay_max",
        "ay_min",
        "az_max",
    ):
        assert unsupported_parameter not in follow_path


def test_velocity_smoother_matches_mppi_limits_and_acceleration_profile():
    smoother = velocity_smoother_parameters()

    assert smoother["smoothing_frequency"] == 20.0
    assert smoother["scale_velocities"] is True
    assert smoother["feedback"] == "CLOSED_LOOP"
    assert smoother["max_velocity"] == [1.0, 1.0, 0.8]
    assert smoother["min_velocity"] == [-1.0, -1.0, -0.8]
    assert smoother["max_accel"] == [0.8, 0.8, 1.0]
    assert smoother["max_decel"] == [-0.8, -0.8, -1.0]


def test_mppi_path_alignment_and_forward_preference_weights():
    follow_path = controller_follow_path()

    assert follow_path["PathAlignCritic"]["cost_weight"] == 4.0
    assert follow_path["PreferForwardCritic"]["enabled"] is True
    assert follow_path["PreferForwardCritic"]["cost_weight"] == 1.0
    assert follow_path["GoalCritic"]["cost_weight"] == 8.0


def test_collision_monitor_uses_requested_slowdown_and_stop_polygons():
    parameters = collision_monitor_parameters()
    slow = parameters["PolygonSlow"]
    stop = parameters["PolygonStop"]

    assert slow["points"] == [0.50, 0.40, 0.50, -0.40, -0.50, -0.40, -0.50, 0.40]
    assert slow["action_type"] == "slowdown"
    assert slow["slowdown_ratio"] == 0.40
    assert slow["max_points"] == 4
    assert stop["points"] == [0.30, 0.25, 0.30, -0.25, -0.30, -0.25, -0.30, 0.25]
    assert stop["action_type"] == "stop"
    assert stop["max_points"] == 4


def test_terminal_goal_and_planner_tolerances_are_consistent_for_near_obstacles():
    controller = controller_parameters()
    planner = planner_parameters()
    goal_checker = controller["general_goal_checker"]

    assert goal_checker["xy_goal_tolerance"] == 0.12
    assert goal_checker["yaw_goal_tolerance"] == 0.40
    assert planner["GridBased"]["tolerance"] <= goal_checker["xy_goal_tolerance"]
