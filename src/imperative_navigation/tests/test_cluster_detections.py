"""Regression tests for direct LiDAR cluster detections."""

import importlib.util
from pathlib import Path

import pytest
import torch


ALGORITHM_PATH = Path(__file__).parents[1] / "algorithm" / "Imperative_learning_2D_moving.py"
SPEC = importlib.util.spec_from_file_location("imperative_planner_clusters", ALGORITHM_PATH)
PLANNER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PLANNER)


def make_scan(indexed_points, sample_count=32):
    lidar_points = torch.zeros(sample_count, 2)
    lidar_hits = torch.zeros(sample_count, dtype=torch.bool)
    for index, point in indexed_points:
        lidar_points[index] = torch.tensor(point, dtype=torch.float32)
        lidar_hits[index] = True
    return lidar_points, lidar_hits


def test_cluster_centroid_p90_radius_and_metadata():
    points = [(10 + index, (1.0, -0.15 + 0.05 * index)) for index in range(7)]
    lidar_points, lidar_hits = make_scan(points)
    robot_position = torch.tensor([2.0, -1.0])
    profile = {}

    detections, world_points, clusters = PLANNER.scan_to_cluster_detections(
        robot_position, lidar_points, lidar_hits, profile=profile)

    assert len(detections) == len(clusters) == 1
    expected_cluster = lidar_points[lidar_hits] + robot_position
    expected_center = torch.mean(expected_cluster, dim=0)
    expected_radius = torch.quantile(
        torch.linalg.norm(expected_cluster - expected_center, dim=1),
        PLANNER.CLUSTER_RADIUS_QUANTILE).item() + PLANNER.CLUSTER_RADIUS_MARGIN
    expected_radius = min(PLANNER.CLUSTER_MAX_RADIUS,
                          max(PLANNER.CLUSTER_MIN_RADIUS, expected_radius))
    assert torch.allclose(detections[0]["position"], expected_center)
    assert detections[0]["radius"] == pytest.approx(expected_radius)
    assert detections[0]["point_count"] == 7
    assert detections[0]["nearest_range"] == pytest.approx(
        torch.linalg.norm(lidar_points[lidar_hits], dim=1).min().item())
    assert torch.equal(world_points, expected_cluster)
    assert profile["detection_count"] == 1
    assert not any("circle" in key or "split" in key for key in profile)


def test_large_wall_cluster_is_not_tracked_but_raw_points_are_preserved():
    points = [(index, (-1.0 + 0.1 * index, 2.0)) for index in range(21)]
    lidar_points, lidar_hits = make_scan(points)
    detections, world_points, clusters = PLANNER.scan_to_cluster_detections(
        torch.zeros(2), lidar_points, lidar_hits)

    assert detections == []
    assert len(world_points) == 21
    assert len(clusters) == 1
    assert torch.equal(clusters[0], world_points)


def test_small_cluster_is_not_promoted_to_detection():
    lidar_points, lidar_hits = make_scan([(4, (1.0, 0.0)), (5, (1.0, 0.1))])
    detections, world_points, clusters = PLANNER.scan_to_cluster_detections(
        torch.zeros(2), lidar_points, lidar_hits)
    assert detections == []
    assert len(world_points) == 2
    assert len(clusters) == 2


def test_scan_boundary_clusters_are_merged():
    lidar_points, lidar_hits = make_scan([
        (0, (1.00, 0.02)), (1, (1.00, 0.04)), (31, (1.00, 0.00)),
    ])
    detections, _, clusters = PLANNER.scan_to_cluster_detections(
        torch.zeros(2), lidar_points, lidar_hits)
    assert len(clusters) == 1
    assert len(detections) == 1
    assert detections[0]["point_count"] == 3


def test_missing_beam_starts_a_new_cluster():
    lidar_points, lidar_hits = make_scan([
        (2, (1.0, 0.0)), (3, (1.0, 0.02)), (4, (1.0, 0.04)),
        (8, (1.1, 0.0)), (9, (1.1, 0.02)), (10, (1.1, 0.04)),
    ])
    detections, _, clusters = PLANNER.scan_to_cluster_detections(
        torch.zeros(2), lidar_points, lidar_hits)
    assert len(clusters) == len(detections) == 2
