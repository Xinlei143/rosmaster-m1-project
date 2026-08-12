"""Equivalence checks for the prefix-statistics split-fit implementation.

Run directly from ``proj_ws``:

    python3 src/imperative_navigation/tests/test_circle_fit_equivalence.py

To include a recorded RDK X5 cluster, set ``IMPERATIVE_REAL_SCAN_FIXTURE`` to
an ``.npz`` file containing ``robot_position``, ``lidar_points``, and
``lidar_hits`` arrays.  The fixture is deliberately external so real sensor
data is not fabricated or silently replaced by synthetic data.
"""

import importlib.util
import os
from pathlib import Path

import numpy as np
import torch


ALGORITHM_PATH = Path(__file__).parents[1] / "algorithm" / "Imperative_learning_2D_moving.py"
SPEC = importlib.util.spec_from_file_location("imperative_planner", ALGORITHM_PATH)
PLANNER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PLANNER)


def assert_close(actual, expected, label, atol=1e-6, rtol=1e-5):
    if not torch.allclose(actual, expected, atol=atol, rtol=rtol, equal_nan=True):
        difference = torch.max(torch.abs(actual - expected)).item()
        raise AssertionError(f"{label} differs; max error={difference}")


def legacy_split_indices(cluster_size):
    """Reference candidate sequence from the pre-optimization Python loop."""
    return torch.tensor(list(range(3, cluster_size - 2)), dtype=torch.long)


def split_decision(points, split_indices, split_errors):
    """Apply the unchanged old split selection rule to a candidate error set."""
    if len(points) < 3:
        return None
    _, fit_error = PLANNER._fit_circle_lstsq_points(points)
    best_split, best_error = None, fit_error
    for candidate_index, split_index in enumerate(split_indices.tolist()):
        split_error = float(split_errors[candidate_index])
        if split_error < best_error:
            best_error = split_error
            best_split = split_index
    return best_split if (best_split is not None and
                          best_error < PLANNER.CIRCLE_SPLIT_IMPROVEMENT * fit_error) else None


def assert_split_equivalence(points, label):
    split_indices = PLANNER._split_candidate_indices(len(points), points.device)
    expected_indices = legacy_split_indices(len(points)).to(points.device)
    if not torch.equal(split_indices, expected_indices):
        raise AssertionError(f"{label}: split candidate indices differ from legacy range")
    try:
        reference = PLANNER._split_circle_errors_reference(points, split_indices)
    except RuntimeError:
        # Exact rank-zero input can make the original LAPACK driver fail. The
        # optimized path deliberately falls back to that implementation, so
        # preserving the same failure is the equivalence requirement here.
        try:
            PLANNER._split_circle_errors_prefix(points, split_indices)
        except RuntimeError:
            return
        raise AssertionError(f"{label}: optimized path did not preserve reference failure")
    optimized, fallback_count = PLANNER._split_circle_errors_prefix(
        points, split_indices, return_fallback_count=True)
    assert_close(optimized, reference, f"{label}: split residual", atol=2e-6, rtol=2e-5)
    # For ill-conditioned candidates, both paths use the same LAPACK fallback.
    # Its near-tied least-squares solutions are not bitwise deterministic across
    # separate calls, so only compare selected splits for analytic candidates.
    if not fallback_count and len(reference) and torch.argmin(optimized) != torch.argmin(reference):
        raise AssertionError(f"{label}: selected split differs")
    if split_decision(points, split_indices, optimized) != split_decision(
            points, expected_indices, reference):
        raise AssertionError(f"{label}: final split decision differs")
    # The detector's circle radius is intentionally fixed at OBSTACLE_RADIUS;
    # neither the reference nor optimized path estimates a separate radius.
    assert PLANNER.OBSTACLE_RADIUS == 0.3


def assert_detection_equivalence(robot_position, lidar_points, lidar_hits, label):
    try:
        reference = PLANNER.scan_to_detections(
            robot_position, lidar_points, lidar_hits, split_fit_strategy="reference")
    except RuntimeError:
        try:
            PLANNER.scan_to_detections(
                robot_position, lidar_points, lidar_hits, split_fit_strategy="optimized")
        except RuntimeError:
            return
        raise AssertionError(f"{label}: optimized detections did not preserve reference failure")
    optimized = PLANNER.scan_to_detections(
        robot_position, lidar_points, lidar_hits, split_fit_strategy="optimized")
    assert_close(optimized[0], reference[0], f"{label}: detection centers")
    assert_close(optimized[1], reference[1], f"{label}: world points")
    if len(optimized[2]) != len(reference[2]):
        raise AssertionError(f"{label}: entity-cluster count differs")
    for index, (actual, expected) in enumerate(zip(optimized[2], reference[2])):
        assert_close(actual, expected, f"{label}: entity cluster {index}")


def circle_arc(center, radius, start, end, count, noise=0.0):
    angles = torch.linspace(start, end, count, dtype=torch.float64)
    points = torch.stack((center[0] + radius * torch.cos(angles),
                          center[1] + radius * torch.sin(angles)), dim=1)
    if noise:
        torch.manual_seed(17)
        points += noise * torch.randn_like(points)
    return points.float()


def realistic_laser_cluster():
    """Polar samples from a circular obstacle, matching a real LaserScan arc."""
    center = torch.tensor([1.45, 0.25], dtype=torch.float64)
    radius = PLANNER.OBSTACLE_RADIUS
    bearing = torch.atan2(center[1], center[0])
    visible_half_angle = torch.asin(radius / torch.linalg.norm(center))
    angles = torch.linspace(bearing - 0.9 * visible_half_angle,
                            bearing + 0.9 * visible_half_angle, 33, dtype=torch.float64)
    # Intersect each ray from the origin with the obstacle circle.
    directions = torch.stack((torch.cos(angles), torch.sin(angles)), dim=1)
    projection = directions @ center
    discriminant = projection ** 2 - (center @ center - radius ** 2)
    ranges = projection - torch.sqrt(discriminant)
    return (ranges[:, None] * directions).float()


def make_scan(points, offset=20, sample_count=181):
    lidar_points = torch.zeros(sample_count, 2)
    lidar_hits = torch.zeros(sample_count, dtype=torch.bool)
    lidar_points[offset:offset + len(points)] = points
    lidar_hits[offset:offset + len(points)] = True
    return lidar_points, lidar_hits


def run_fixture(path):
    fixture = np.load(path)
    assert_detection_equivalence(
        torch.from_numpy(fixture["robot_position"]),
        torch.from_numpy(fixture["lidar_points"]),
        torch.from_numpy(fixture["lidar_hits"]),
        "recorded RDK X5 fixture",
    )


def test_split_candidate_indices_sizes_zero_through_twenty():
    """Every small-N candidate set must match the old range item-for-item."""
    for cluster_size in range(21):
        expected = legacy_split_indices(cluster_size)
        actual = PLANNER._split_candidate_indices(cluster_size)
        assert torch.equal(actual, expected), f"N={cluster_size}: candidate sequence differs"
        assert len(actual) == max(0, cluster_size - 5)
        for split_index in actual.tolist():
            assert len(range(split_index)) == split_index
            assert split_index >= 3
            assert cluster_size - split_index >= 3

        if cluster_size < 3:
            # The legacy detector never attempts a circle fit or split here.
            assert len(actual) == 0
            continue

        # Use a non-degenerate arc wherever a circle fit is defined, then
        # compare the exact old/new split decision as well as candidate bounds.
        if cluster_size >= 3:
            points = circle_arc(torch.tensor([0.7, -0.2]), 0.3, -0.7, 0.7,
                                cluster_size)
            assert_split_equivalence(points, f"small cluster N={cluster_size}")


def test_small_unsplittable_clusters_use_the_legacy_empty_split_branch():
    """N=3..5 must reach split search without constructing an invalid arange."""
    for cluster_size in (3, 4, 5):
        coordinate = torch.linspace(-0.2, 0.2, cluster_size)
        # A short almost-straight return has a poor fixed-radius fit, so this
        # deliberately exercises the post-primary-fit split-search branch.
        points = torch.stack((coordinate, 1e-5 * coordinate ** 2), dim=1)
        lidar_points, lidar_hits = make_scan(points)
        profile = {}
        PLANNER.scan_to_detections(torch.zeros(2), lidar_points, lidar_hits,
                                   profile=profile, split_fit_strategy="optimized")
        assert profile["primary_fit_calls"] == 1
        assert profile["split_candidates"] == 0
        assert profile["split_fit_calls"] == 0
        assert_detection_equivalence(torch.zeros(2), lidar_points, lidar_hits,
                                     f"unsplittable N={cluster_size}")


def test_random_cluster_sizes_split_and_detection_equivalence():
    """Exercise random sizes and geometry without changing candidate rules."""
    generator = torch.Generator().manual_seed(20260811)
    for sample in range(40):
        cluster_size = int(torch.randint(0, 80, (1,), generator=generator))
        expected = legacy_split_indices(cluster_size)
        actual = PLANNER._split_candidate_indices(cluster_size)
        assert torch.equal(actual, expected), f"random sample {sample}: candidate mismatch"
        if cluster_size < 3:
            continue
        center = torch.tensor([1.2, -0.4])
        points = circle_arc(center, 0.3, -0.9, 0.9, cluster_size)
        points += 0.004 * torch.randn(points.shape, generator=generator)
        assert_split_equivalence(points, f"random cluster {sample}, N={cluster_size}")
        lidar_points, lidar_hits = make_scan(points[:min(cluster_size, 80)])
        assert_detection_equivalence(torch.zeros(2), lidar_points, lidar_hits,
                                     f"random scan {sample}, N={cluster_size}")


def test_normal_noisy_near_degenerate_and_laser_geometry_equivalence():
    """Cover the fixed test cases independently of the script entry point."""
    normal_arc = circle_arc(torch.tensor([1.2, -0.4]), 0.3, -0.8, 0.8, 35)
    noisy_arc = circle_arc(torch.tensor([1.2, -0.4]), 0.3, -0.8, 0.8, 35, noise=0.003)
    near_line = torch.stack((torch.linspace(-1.0, 1.0, 35),
                             1e-6 * torch.linspace(-1.0, 1.0, 35) ** 2), dim=1)
    degenerate = torch.tensor([[0.2, -0.1]] * 12, dtype=torch.float32)
    for label, points in (("normal arc", normal_arc), ("noisy arc", noisy_arc),
                          ("near line", near_line), ("degenerate", degenerate),
                          ("laser geometry", realistic_laser_cluster())):
        assert_split_equivalence(points, label)
        if label == "degenerate":
            continue
        lidar_points, lidar_hits = make_scan(points)
        assert_detection_equivalence(torch.zeros(2), lidar_points, lidar_hits, label)


def test_recorded_rdk_x5_fixture_if_supplied():
    """Run the exact same equivalence check on a captured physical scan."""
    fixture_path = os.environ.get("IMPERATIVE_REAL_SCAN_FIXTURE")
    if not fixture_path:
        import pytest
        pytest.skip("IMPERATIVE_REAL_SCAN_FIXTURE was not supplied")
    run_fixture(fixture_path)


def main():
    normal_arc = circle_arc(torch.tensor([1.2, -0.4]), 0.3, -0.8, 0.8, 35)
    noisy_arc = circle_arc(torch.tensor([1.2, -0.4]), 0.3, -0.8, 0.8, 35, noise=0.003)
    near_line = torch.stack((torch.linspace(-1.0, 1.0, 35),
                             1e-6 * torch.linspace(-1.0, 1.0, 35) ** 2), dim=1)
    tiny_cluster = circle_arc(torch.tensor([0.5, 0.2]), 0.3, -0.1, 0.1, 5)
    degenerate = torch.tensor([[0.2, -0.1]] * 12, dtype=torch.float32)

    for label, points in (
            ("normal arc", normal_arc), ("noisy arc", noisy_arc),
            ("near line", near_line), ("tiny cluster", tiny_cluster),
            ("degenerate cluster", degenerate), ("laser geometry", realistic_laser_cluster())):
        assert_split_equivalence(points, label)
        if label == "degenerate cluster":
            continue
        lidar_points, lidar_hits = make_scan(points)
        assert_detection_equivalence(torch.zeros(2), lidar_points, lidar_hits, label)

    fixture_path = os.environ.get("IMPERATIVE_REAL_SCAN_FIXTURE")
    if fixture_path:
        run_fixture(fixture_path)
    else:
        print("No recorded RDK X5 fixture supplied; synthetic and laser-geometry cases passed.")
    print("Circle-fit and split-search equivalence tests passed.")


if __name__ == "__main__":
    main()
