"""Regression tests for the GPU-LiDAR Nav2 costmap observation settings."""

from pathlib import Path

import yaml


PARAMS = Path(__file__).parents[1] / "config" / "nav2_params.yaml"


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


def test_gpu_lidar_scan_has_an_explicit_observation_height_window():
    """Laser returns transformed above z=0 must not be filtered by defaults."""
    for costmap_name in ("local_costmap", "global_costmap"):
        scan = costmap_scan_source(costmap_name)
        assert scan["min_obstacle_height"] == 0.0
        assert scan["max_obstacle_height"] >= 0.175


def test_gpu_lidar_scan_marks_and_clears_both_costmaps():
    for costmap_name in ("local_costmap", "global_costmap"):
        scan = costmap_scan_source(costmap_name)
        assert scan["data_type"] == "LaserScan"
        assert scan["marking"] is True
        assert scan["clearing"] is True


def test_mppi_uses_supported_omni_velocity_constraints():
    follow_path = controller_follow_path()

    assert follow_path["plugin"] == "nav2_mppi_controller::MPPIController"
    assert follow_path["motion_model"] == "Omni"
    assert follow_path["vx_max"] == 0.6
    assert follow_path["vx_min"] == -0.4
    assert follow_path["vy_max"] == 0.4
    assert follow_path["wz_max"] == 1.0

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
    assert smoother["max_velocity"] == [0.6, 0.4, 1.0]
    assert smoother["min_velocity"] == [-0.4, -0.4, -1.0]
    assert smoother["max_accel"] == [0.5, 0.5, 1.0]
    assert smoother["max_decel"] == [-0.5, -0.5, -1.0]
