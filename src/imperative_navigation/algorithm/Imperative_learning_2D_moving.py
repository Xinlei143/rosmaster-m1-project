import os
import time
from pathlib import Path

# Matplotlib must be configured before importing pyplot.  The previous hard-coded
# ``MacOSX`` backend prevents this program from starting on Linux and Windows.
# Keep the user's explicit backend choice, otherwise use Agg automatically when
# no graphical display is available (for example, VS Code Remote/SSH).
if "MPLCONFIGDIR" not in os.environ:
    matplotlib_cache = Path(__file__).with_name(".matplotlib")
    matplotlib_cache.mkdir(exist_ok=True)
    os.environ["MPLCONFIGDIR"] = str(matplotlib_cache)

import matplotlib

if "MPLBACKEND" not in os.environ and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
    matplotlib.use("Agg")

import numpy as np
import torch
from matplotlib import pyplot as plt
from matplotlib.patches import Circle
from tqdm import tqdm
# ============================================================
# Configuration
# ============================================================

DT = 0.1  # s, control step
NUM_STEPS = 180  # steps, run length
MAX_SPEED = 1.0  # m/s, AGV speed
MAX_ACCELERATION = 1.0  # m/s^2, AGV acceleration
GOAL_SPEED_TOLERANCE = 0.05  # m/s, stopped goal
START = torch.tensor([-2.5, -1.5])  # m, AGV start
GOAL = torch.tensor([2.5, 1.5])  # m, AGV goal
ROOM_X_MIN = -4.0  # m, left wall
ROOM_X_MAX = 4.0  # m, right wall
ROOM_Y_MIN = -3.2  # m, bottom wall
ROOM_Y_MAX = 3.2  # m, top wall

NN = 3  # moving obstacles
OBSTACLE_RADIUS = 0.3  # m
OBSTACLE_SAFETY_MARGIN = 0.10  # m, pair clearance
OBSTACLE_REPULSION_DISTANCE = 2.2  # m, steering range
STATIC_REPULSION_DISTANCE = 2.2  # m, static steering range
OBSTACLE_REPULSION_STRENGTH = 4.0  # 1/s, steering strength
OBSTACLE_MAX_TURN_RATE = 1.0  # rad/s, direction rate
OBSTACLE_MIN_DISTANCE = 2.0 * OBSTACLE_RADIUS + OBSTACLE_SAFETY_MARGIN  # m, no contact
OBSTACLE_WALL_DISTANCE = OBSTACLE_RADIUS + OBSTACLE_SAFETY_MARGIN  # m, center clearance
# OBSTACLE_STARTS = torch.tensor([[-0.05, 1.99], [1.99, 1.50]])  # m, initial centers
OBSTACLE_STARTS = torch.randn(NN, 2) * 1.0 + 2.0
OBSTACLE_SPEEDS = 0.35 + 0.05 * torch.arange(NN)  # m/s, individual speeds
OBSTACLE_DIRECTIONS = -OBSTACLE_STARTS / torch.linalg.norm(OBSTACLE_STARTS, dim=1, keepdim=True)  # direction, inward
OBSTACLE_VELOCITIES = OBSTACLE_SPEEDS[:, None] * OBSTACLE_DIRECTIONS  # m/s, initial velocities
STATIC_OBSTACLE_CENTERS = torch.tensor([[-2.5, 1.8], [2.5, -1.8]])  # m, static centers

ROBOT_RADIUS = 0.15  # m
SAFETY_MARGIN = 0.15  # m
LIDAR_RANGE = 12.0  # m, T-MINI PLUS maximum range
LIDAR_POINTS = 667  # beams, approximately 0.54 deg over a full scan
LIDAR_SAFE_DISTANCE = ROBOT_RADIUS + SAFETY_MARGIN  # m, surface clearance
PLANNING_CLEARANCE_MARGIN = 0.15  # m, acceleration response
# The physical M1 adapter can make these directional: a larger clearance is
# retained for obstacles in the goal direction, while tight corridors use the
# actual robot footprint plus a small lateral margin.  Keep the defaults equal
# to the original isotropic model so the standalone simulator is unchanged.
FRONT_LIDAR_SAFE_DISTANCE = LIDAR_SAFE_DISTANCE
FRONT_PLANNING_CLEARANCE_MARGIN = PLANNING_CLEARANCE_MARGIN
LATERAL_PLANNING_CLEARANCE_MARGIN = PLANNING_CLEARANCE_MARGIN
FORWARD_CLEARANCE_COSINE = float(torch.cos(torch.tensor(torch.pi / 4.0)))
LIDAR_ANGLES = torch.arange(LIDAR_POINTS) * 2.0 * torch.pi / LIDAR_POINTS - torch.pi  # rad, beam angles

CLUSTER_GAP = 0.30  # m, cluster separation
CIRCLE_FIT_MAX_ERROR = 0.02  # m, radius residual
CIRCLE_SPLIT_IMPROVEMENT = 0.50  # ratio, split acceptance
MAP_RESOLUTION = 0.10  # m, voxel size
MAP_DYNAMIC_MARGIN = 0.15  # m, moving-point removal
MAX_MAP_POINTS = 2000  # points
TRACK_ASSOCIATION_DISTANCE = 0.80  # m
TRACK_MAX_MISSED = 5  # scans
STATIC_SPEED_THRESHOLD = 0.15  # m/s
VELOCITY_SMOOTHING = 0.25  # estimate weight
MAX_ESTIMATED_SPEED = 1.0  # m/s
MAX_ESTIMATED_ACCELERATION = 0.8  # m/s^2, velocity rate

PLAN_HORIZON = 15  # steps
PLAN_HEADINGS = 36  # directions
PLAN_ACCELERATIONS = torch.tensor([0.25, 0.50, 1.00])  # m/s^2
COLLISION_COST = 100000.0  # collision weight
GOAL_POSITION_COST = 100.0  # position weight
GOAL_VELOCITY_COST = 20.0  # stop weight
ACCELERATION_COST = 0.20  # effort weight
GOAL_TOLERANCE = 0.20  # m

ESTIMATE_COLORS = ["tomato", "darkorange", "mediumseagreen", "orchid", "sienna", "slateblue"]
INTERACTIVE = matplotlib.get_backend().lower() != "agg"


# ============================================================
# Simulator truth, used only inside the onboard LiDAR sensor
# ============================================================

def simulate_obstacles(num_steps):
    positions = OBSTACLE_STARTS.clone()
    velocities = OBSTACLE_VELOCITIES.clone()
    eye = torch.eye(NN, dtype=torch.bool)
    lower_wall_limits = torch.tensor([ROOM_X_MIN, ROOM_Y_MIN]) + OBSTACLE_WALL_DISTANCE
    upper_wall_limits = torch.tensor([ROOM_X_MAX, ROOM_Y_MAX]) - OBSTACLE_WALL_DISTANCE
    positions = torch.maximum(torch.minimum(positions, upper_wall_limits), lower_wall_limits)

    for _ in range(20):
        for first in range(NN):
            for second in range(first + 1, NN):
                offset = positions[first] - positions[second]
                distance = torch.linalg.norm(offset)

                if distance < OBSTACLE_MIN_DISTANCE:
                    correction = 0.5 * (OBSTACLE_MIN_DISTANCE - distance) * offset / distance.clamp_min(1e-6)
                    positions[first] += correction
                    positions[second] -= correction

    paths = [positions.clone()]

    for _ in range(num_steps):
        offsets = positions[:, None, :] - positions[None, :, :]
        distances = torch.linalg.norm(offsets, dim=2)
        nearby = (distances < OBSTACLE_REPULSION_DISTANCE) & ~eye
        directions = offsets / distances.clamp_min(1e-6)[:, :, None]
        strengths = torch.clamp((OBSTACLE_REPULSION_DISTANCE - distances) / OBSTACLE_REPULSION_DISTANCE, min=0.0)
        repulsion = torch.sum(torch.where(nearby[:, :, None], strengths[:, :, None] * directions, torch.zeros_like(directions)), dim=1)
        static_offsets = positions[:, None, :] - STATIC_OBSTACLE_CENTERS[None, :, :]
        static_distances = torch.linalg.norm(static_offsets, dim=2)
        static_directions = static_offsets / static_distances.clamp_min(1e-6)[:, :, None]
        static_strengths = torch.clamp((STATIC_REPULSION_DISTANCE - static_distances) / STATIC_REPULSION_DISTANCE, min=0.0)
        repulsion += torch.sum(static_strengths[:, :, None] * static_directions, dim=1)
        lower_wall_distances = positions - lower_wall_limits
        upper_wall_distances = upper_wall_limits - positions
        repulsion += torch.clamp((OBSTACLE_REPULSION_DISTANCE - lower_wall_distances) / OBSTACLE_REPULSION_DISTANCE, min=0.0)
        repulsion -= torch.clamp((OBSTACLE_REPULSION_DISTANCE - upper_wall_distances) / OBSTACLE_REPULSION_DISTANCE, min=0.0)
        desired_velocities = velocities + DT * OBSTACLE_REPULSION_STRENGTH * repulsion
        current_angles = torch.atan2(velocities[:, 1], velocities[:, 0])
        desired_angles = torch.atan2(desired_velocities[:, 1], desired_velocities[:, 0])
        angle_changes = torch.atan2(torch.sin(desired_angles - current_angles), torch.cos(desired_angles - current_angles))
        new_angles = current_angles + torch.clamp(angle_changes, -OBSTACLE_MAX_TURN_RATE * DT, OBSTACLE_MAX_TURN_RATE * DT)
        velocities = OBSTACLE_SPEEDS[:, None] * torch.stack([torch.cos(new_angles), torch.sin(new_angles)], dim=1)
        next_positions = positions + DT * velocities

        for _ in range(3):
            for first in range(NN):
                for second in range(first + 1, NN):
                    offset = next_positions[first] - next_positions[second]
                    distance = torch.linalg.norm(offset)

                    if distance < OBSTACLE_MIN_DISTANCE:
                        direction = offset / distance.clamp_min(1e-6)
                        correction = 0.5 * (OBSTACLE_MIN_DISTANCE - distance) * direction
                        next_positions[first] += correction
                        next_positions[second] -= correction
                        relative_speed = torch.dot(velocities[first] - velocities[second], direction)

                        if relative_speed < 0.0:
                            velocities[first] -= relative_speed * direction
                            velocities[second] += relative_speed * direction

            for first in range(NN):
                for static_position in STATIC_OBSTACLE_CENTERS:
                    offset = next_positions[first] - static_position
                    distance = torch.linalg.norm(offset)

                    if distance < OBSTACLE_MIN_DISTANCE:
                        direction = offset / distance.clamp_min(1e-6)
                        next_positions[first] += (OBSTACLE_MIN_DISTANCE - distance) * direction
                        inward_speed = torch.dot(velocities[first], direction)

                        if inward_speed < 0.0:
                            velocities[first] -= inward_speed * direction

            below_lower_wall = next_positions < lower_wall_limits
            above_upper_wall = next_positions > upper_wall_limits
            next_positions = torch.maximum(torch.minimum(next_positions, upper_wall_limits), lower_wall_limits)
            velocities = torch.where(below_lower_wall & (velocities < 0.0), -velocities, velocities)
            velocities = torch.where(above_upper_wall & (velocities > 0.0), -velocities, velocities)

        velocities = OBSTACLE_SPEEDS[:, None] * velocities / torch.linalg.norm(velocities, dim=1,
                                                                               keepdim=True).clamp_min(1e-6)
        positions = next_positions
        paths.append(positions.clone())

    return torch.stack(paths)


def simulate_lidar(robot_position, time_index):
    moving_positions = simulate_obstacles(time_index)[-1].to(robot_position)
    obstacle_positions = torch.cat([moving_positions, STATIC_OBSTACLE_CENTERS.to(robot_position)])
    relative_centers = obstacle_positions - robot_position
    lidar_angles = LIDAR_ANGLES.to(robot_position)
    ray_directions = torch.stack([torch.cos(lidar_angles), torch.sin(lidar_angles)], dim=1)
    projections = relative_centers @ ray_directions.T
    discriminants = projections ** 2 - torch.sum(relative_centers ** 2, dim=1, keepdim=True) + OBSTACLE_RADIUS ** 2
    roots = torch.sqrt(torch.clamp(discriminants, min=0.0) + 1e-12)
    near_ranges = projections - roots
    far_ranges = projections + roots
    ray_ranges = torch.where(near_ranges >= 0.0, near_ranges, far_ranges)
    obstacle_hits = (discriminants >= 0.0) & (ray_ranges >= 0.0) & (ray_ranges <= LIDAR_RANGE)
    obstacle_ranges = torch.where(obstacle_hits, ray_ranges, torch.full_like(ray_ranges, LIDAR_RANGE))
    infinite_ranges = torch.full((LIDAR_POINTS,), torch.inf, dtype=robot_position.dtype,
                                 device=robot_position.device)
    x_wall_ranges = torch.where(ray_directions[:, 0] > 1e-8,
                                (ROOM_X_MAX - robot_position[0]) / ray_directions[:, 0], infinite_ranges)
    x_wall_ranges = torch.where(ray_directions[:, 0] < -1e-8,
                                (ROOM_X_MIN - robot_position[0]) / ray_directions[:, 0], x_wall_ranges)
    y_wall_ranges = torch.where(ray_directions[:, 1] > 1e-8,
                                (ROOM_Y_MAX - robot_position[1]) / ray_directions[:, 1], infinite_ranges)
    y_wall_ranges = torch.where(ray_directions[:, 1] < -1e-8,
                                (ROOM_Y_MIN - robot_position[1]) / ray_directions[:, 1], y_wall_ranges)
    wall_ranges = torch.minimum(x_wall_ranges, y_wall_ranges)
    wall_hits = wall_ranges <= LIDAR_RANGE
    measured_ranges = torch.minimum(obstacle_ranges.min(dim=0).values,
                                    torch.where(wall_hits, wall_ranges, infinite_ranges))
    lidar_hits = obstacle_hits.any(dim=0) | wall_hits

    if torch.any(torch.linalg.norm(relative_centers, dim=1) < OBSTACLE_RADIUS):
        measured_ranges[0] = 0.0
        lidar_hits[0] = True

    lidar_points = measured_ranges[:, None] * ray_directions
    return lidar_points, lidar_hits


# ============================================================
# Pose-aided point-cloud mapping and consecutive-scan motion estimation
# ============================================================

def _fit_circle_lstsq_points(points):
    """Original algebraic circle fit, retained as the numerical fallback."""
    points = points.double()
    origin = torch.mean(points, dim=0)
    centered_points = points - origin
    circle_matrix = torch.cat(
        [2.0 * centered_points, torch.ones(len(points), 1, dtype=torch.double)], dim=1)
    circle_vector = torch.sum(centered_points ** 2, dim=1)
    center = torch.linalg.lstsq(circle_matrix, circle_vector).solution[:2] + origin
    fit_error = torch.mean(
        torch.abs(torch.linalg.norm(points - center, dim=1) - OBSTACLE_RADIUS)).item()
    return center.float(), fit_error


def _split_candidate_indices(cluster_size, device=None):
    """Return the exact candidate sequence from ``range(3, N - 2)``.

    A split at index ``k`` creates ``points[:k]`` and ``points[k:]``.  The
    original Python loop allowed only k = 3 .. N - 3, so both child clusters
    contain at least three points.  Python's ``range`` returns an empty
    sequence when this interval does not exist; unlike it, ``torch.arange``
    raises for a positive step with end < start.  Construct from the known
    candidate count instead, so N <= 5 is an explicit, safe empty set.
    """
    candidate_count = max(0, int(cluster_size) - 5)
    if candidate_count == 0:
        return torch.empty(0, dtype=torch.long, device=device)
    return torch.arange(candidate_count, dtype=torch.long, device=device) + 3


def _split_circle_errors_reference(points, split_indices):
    """Reference split errors, byte-for-byte equivalent to the former loop."""
    errors = []
    for split_index in split_indices.tolist():
        _, first_error = _fit_circle_lstsq_points(points[:split_index])
        _, second_error = _fit_circle_lstsq_points(points[split_index:])
        errors.append((split_index * first_error +
                       (len(points) - split_index) * second_error) / len(points))
    return torch.tensor(errors, dtype=torch.double)


def _split_circle_errors_prefix(points, split_indices, return_fallback_count=False):
    """Compute all split errors from prefix moments, with lstsq fallback.

    For centered coordinates q, the original least-squares system is
    [2*qx, 2*qy, 1] * theta = qx² + qy².  Its normal equations give
    center_offset = 0.5 * inv(C) * [sum(qx*r²), sum(qy*r²)], where C is the
    2x2 centered second-moment matrix. Prefix sums of raw moments through
    degree three provide C and the right-hand side for every prefix/suffix in
    O(1). The residual remains the original mean absolute radial residual.
    """
    if not len(split_indices):
        result = torch.empty(0, dtype=torch.double)
        return (result, 0) if return_fallback_count else result

    points = points.double()
    x, y = points[:, 0], points[:, 1]
    raw_stats = torch.stack((
        torch.ones_like(x), x, y, x * x, y * y, x * y,
        x * x * x, y * y * y, x * x * y, x * y * y,
    ), dim=1)
    prefix = torch.cat((torch.zeros(1, raw_stats.shape[1], dtype=torch.double),
                        torch.cumsum(raw_stats, dim=0)), dim=0)
    left_stats = prefix[split_indices]
    right_stats = prefix[-1] - left_stats

    def centers_from_stats(stats):
        count, sx, sy, sxx, syy, sxy, sx3, sy3, sx2y, sxy2 = stats.unbind(dim=1)
        mean_x, mean_y = sx / count, sy / count
        cxx = sxx - sx * mean_x
        cyy = syy - sy * mean_y
        cxy = sxy - sx * mean_y

        cxxx = sx3 - 3.0 * mean_x * sxx + 3.0 * mean_x * mean_x * sx - count * mean_x ** 3
        cyyy = sy3 - 3.0 * mean_y * syy + 3.0 * mean_y * mean_y * sy - count * mean_y ** 3
        cxxy = (sx2y - 2.0 * mean_x * sxy + mean_x * mean_x * sy - mean_y * sxx +
                2.0 * mean_x * mean_y * sx - count * mean_x * mean_x * mean_y)
        cxyy = (sxy2 - 2.0 * mean_y * sxy + mean_y * mean_y * sx - mean_x * syy +
                2.0 * mean_x * mean_y * sy - count * mean_x * mean_y * mean_y)

        rhs_x = 0.5 * (cxxx + cxyy)
        rhs_y = 0.5 * (cxxy + cyyy)
        determinant = cxx * cyy - cxy * cxy
        scale = (cxx.abs() + cyy.abs()).clamp_min(torch.finfo(torch.double).eps)
        well_conditioned = (torch.isfinite(determinant) & torch.isfinite(rhs_x) &
                            torch.isfinite(rhs_y) &
                            (determinant.abs() > 1e-5 * scale * scale))
        offset_x = (cyy * rhs_x - cxy * rhs_y) / determinant
        offset_y = (cxx * rhs_y - cxy * rhs_x) / determinant
        centers = torch.stack((mean_x + offset_x, mean_y + offset_y), dim=1)
        return centers, well_conditioned

    left_centers, left_valid = centers_from_stats(left_stats)
    right_centers, right_valid = centers_from_stats(right_stats)
    sample_indices = torch.arange(len(points))
    residual_left = torch.abs(
        torch.linalg.norm(points[None, :, :] - left_centers[:, None, :], dim=2) - OBSTACLE_RADIUS)
    residual_right = torch.abs(
        torch.linalg.norm(points[None, :, :] - right_centers[:, None, :], dim=2) - OBSTACLE_RADIUS)
    left_mask = sample_indices[None, :] < split_indices[:, None]
    right_mask = ~left_mask
    left_errors = torch.sum(residual_left * left_mask, dim=1) / split_indices
    right_counts = len(points) - split_indices
    right_errors = torch.sum(residual_right * right_mask, dim=1) / right_counts
    split_errors = (split_indices * left_errors + right_counts * right_errors) / len(points)

    # The normal-equation solve is deliberately rejected for poorly
    # conditioned prefixes/suffixes. Those candidates use the original QR/SVD
    # backed torch.linalg.lstsq path and therefore preserve degenerate cases.
    fallback_indices = torch.where(~(left_valid & right_valid))[0].tolist()
    for candidate_index in fallback_indices:
        split_index = int(split_indices[candidate_index])
        _, left_error = _fit_circle_lstsq_points(points[:split_index])
        _, right_error = _fit_circle_lstsq_points(points[split_index:])
        split_errors[candidate_index] = (
            split_index * left_error + (len(points) - split_index) * right_error) / len(points)
    return (split_errors, len(fallback_indices)) if return_fallback_count else split_errors


def scan_to_detections(robot_position, lidar_points, lidar_hits, profile=None,
                       split_fit_strategy="optimized"):
    """Cluster laser returns and fit circular obstacle detections.

    When ``profile`` is a dict, timing and count diagnostics are populated
    without changing the clustering or fitting calculations.
    """
    profiling = profile is not None
    if profiling:
        profile.clear()
        profile.update({
            "prepare": 0.0,
            "cluster_scan": 0.0,
            "cluster_wrap": 0.0,
            "pending_clusters_total": 0.0,
            "circle_fit_primary": 0.0,
            "circle_fit_split": 0.0,
            "split_search": 0.0,
            "finalize": 0.0,
            "hit_count": 0,
            "initial_cluster_count": 0,
            "primary_fit_calls": 0,
            "split_fit_calls": 0,
            "split_candidates": 0,
            "split_lstsq_fallbacks": 0,
        })

    stage_start = time.perf_counter() if profiling else None
    hit_indices = torch.where(lidar_hits)[0]
    local_points = lidar_points[lidar_hits]
    world_points = local_points + robot_position
    if profiling:
        profile["prepare"] = time.perf_counter() - stage_start
        profile["hit_count"] = len(hit_indices)

    if len(hit_indices) < 3:
        stage_start = time.perf_counter() if profiling else None
        world_clusters = [point[None] for point in world_points]
        if profiling:
            profile["finalize"] = time.perf_counter() - stage_start
        return torch.empty(0, 2), world_points, world_clusters

    clusters = []
    cluster_start = 0

    stage_start = time.perf_counter() if profiling else None
    for index in range(1, len(hit_indices)):
        new_cluster = hit_indices[index] - hit_indices[index - 1] > 1
        new_cluster |= torch.linalg.norm(local_points[index] - local_points[index - 1]) > CLUSTER_GAP

        if new_cluster:
            clusters.append(local_points[cluster_start:index])
            cluster_start = index

    clusters.append(local_points[cluster_start:])
    if profiling:
        profile["cluster_scan"] = time.perf_counter() - stage_start
        profile["initial_cluster_count"] = len(clusters)

    stage_start = time.perf_counter() if profiling else None
    if len(clusters) > 1:
        # Real and simulated LaserScan messages may use a different beam
        # count from the built-in demo. Infer the count from the message so
        # wrap-around clustering remains correct for the Yahboom model.
        wrapped_gap = hit_indices[0] + len(lidar_points) - hit_indices[-1]
        wrapped_distance = torch.linalg.norm(local_points[0] - local_points[-1])

        if wrapped_gap == 1 and wrapped_distance <= CLUSTER_GAP:
            clusters = [torch.cat([clusters[-1], clusters[0]])] + clusters[1:-1]
    if profiling:
        profile["cluster_wrap"] = time.perf_counter() - stage_start

    def fit_circle(cluster, fit_stage):
        fit_start = time.perf_counter() if profiling else None
        center, fit_error = _fit_circle_lstsq_points(cluster + robot_position)
        if profiling:
            profile[fit_stage] += time.perf_counter() - fit_start
            if fit_stage == "circle_fit_primary":
                profile["primary_fit_calls"] += 1
            else:
                profile["split_fit_calls"] += 1
        return center.float(), fit_error

    detections = []
    entity_clusters = []
    pending_clusters = clusters.copy()

    pending_start = time.perf_counter() if profiling else None
    while pending_clusters:
        cluster = pending_clusters.pop(0)

        if len(cluster) < 3:
            entity_clusters.extend([point[None] + robot_position for point in cluster])
            continue

        center, fit_error = fit_circle(cluster, "circle_fit_primary")

        if fit_error <= CIRCLE_FIT_MAX_ERROR:
            detections.append(center)
            entity_clusters.append(cluster + robot_position)
            continue

        best_split = None
        best_error = fit_error

        split_start = time.perf_counter() if profiling else None
        cluster_points = cluster + robot_position
        # This is exactly ``list(range(3, len(cluster) - 2))`` from the
        # former implementation, including its empty result for N <= 5.
        split_indices = _split_candidate_indices(len(cluster), cluster_points.device)
        split_fit_start = time.perf_counter() if profiling else None
        if split_fit_strategy == "optimized":
            split_errors, fallback_count = _split_circle_errors_prefix(
                cluster_points, split_indices, return_fallback_count=True)
        elif split_fit_strategy == "reference":
            split_errors = _split_circle_errors_reference(cluster_points, split_indices)
            fallback_count = len(split_indices)
        else:
            raise ValueError(f"Unknown split_fit_strategy: {split_fit_strategy}")
        if profiling:
            profile["circle_fit_split"] += time.perf_counter() - split_fit_start
            profile["split_fit_calls"] += 2 * len(split_indices)
            profile["split_lstsq_fallbacks"] += fallback_count

        for candidate_index, split_index in enumerate(split_indices.tolist()):
            split_error = float(split_errors[candidate_index])

            if split_error < best_error:
                best_error = split_error
                best_split = split_index
            if profiling:
                profile["split_candidates"] += 1
        if profiling:
            profile["split_search"] += time.perf_counter() - split_start

        if best_split is not None and best_error < CIRCLE_SPLIT_IMPROVEMENT * fit_error:
            pending_clusters = [cluster[:best_split], cluster[best_split:]] + pending_clusters
        else:
            entity_clusters.extend([point[None] + robot_position for point in cluster])
    if profiling:
        profile["pending_clusters_total"] = time.perf_counter() - pending_start

    stage_start = time.perf_counter() if profiling else None
    detection_tensor = torch.stack(detections) if detections else torch.empty(0, 2)
    if profiling:
        profile["finalize"] = time.perf_counter() - stage_start
    return detection_tensor, world_points, entity_clusters


def update_slam_map(map_points, world_points, dynamic_centers):
    points = torch.cat([map_points, world_points]) if len(map_points) else world_points.clone()

    if len(points) == 0:
        return torch.empty(0, 2)

    if len(dynamic_centers):
        points = points[torch.all(torch.cdist(points, dynamic_centers) > OBSTACLE_RADIUS + MAP_DYNAMIC_MARGIN, dim=1)]

    voxel_points = torch.unique(torch.round(points / MAP_RESOLUTION) * MAP_RESOLUTION, dim=0)
    return voxel_points[-MAX_MAP_POINTS:]


def update_tracks(tracks, detections, next_track_id):
    updated_tracks = []
    used_detections = set()

    for track in tracks:
        predicted_position = track["position"] + DT * track["velocity"]
        matched_index = None

        if len(detections):
            association_distances = torch.linalg.norm(detections - predicted_position, dim=1)

            for used_index in used_detections:
                association_distances[used_index] = torch.inf

            nearest_index = int(torch.argmin(association_distances))

            if association_distances[nearest_index] <= TRACK_ASSOCIATION_DISTANCE:
                matched_index = nearest_index

        if matched_index is not None:
            detection = detections[matched_index]
            measured_velocity = (detection - track["last_detection"]) / (DT * (track["missed"] + 1))
            target_velocity = measured_velocity if track["age"] == 1 else VELOCITY_SMOOTHING * measured_velocity + (1.0 - VELOCITY_SMOOTHING) * track["velocity"]
            velocity_change = target_velocity - track["velocity"]
            maximum_change = MAX_ESTIMATED_ACCELERATION * DT * (track["missed"] + 1)
            velocity_change *= torch.clamp(maximum_change / torch.linalg.norm(velocity_change).clamp_min(1e-6), max=1.0)
            velocity = track["velocity"] + velocity_change
            velocity *= torch.clamp(MAX_ESTIMATED_SPEED / torch.linalg.norm(velocity).clamp_min(1e-6), max=1.0)
            updated_tracks.append(
                {"id": track["id"], "position": detection, "velocity": velocity, "age": track["age"] + 1, "missed": 0,
                 "last_detection": detection, "history": track["history"] + [detection.clone()]})
            used_detections.add(matched_index)
        elif track["missed"] < TRACK_MAX_MISSED:
            updated_tracks.append({"id": track["id"], "position": predicted_position, "velocity": track["velocity"],
                                   "age": track["age"] + 1, "missed": track["missed"] + 1,
                                   "last_detection": track["last_detection"],
                                   "history": track["history"] + [predicted_position.clone()]})

    for detection_index, detection in enumerate(detections):
        if detection_index not in used_detections:
            updated_tracks.append(
                {"id": next_track_id, "position": detection, "velocity": torch.zeros(2), "age": 1, "missed": 0,
                 "last_detection": detection, "history": [detection.clone()]})
            next_track_id += 1

    return updated_tracks, next_track_id


def predict_tracks(tracks, horizon):
    if not tracks:
        return torch.empty(horizon, 0, 2)

    positions = torch.stack([track["position"] for track in tracks])
    velocities = torch.stack([track["velocity"] for track in tracks])
    times = torch.arange(1, horizon + 1, dtype=positions.dtype) * DT
    return positions[None, :, :] + times[:, None, None] * velocities[None, :, :]


# ============================================================
# Lightweight receding-horizon planning from estimates only
# ============================================================

def step_robot(position, velocity, acceleration):
    acceleration = acceleration * torch.clamp(MAX_ACCELERATION / torch.linalg.norm(acceleration).clamp_min(1e-6), max=1.0)
    next_velocity = velocity + DT * acceleration
    next_velocity *= torch.clamp(MAX_SPEED / torch.linalg.norm(next_velocity).clamp_min(1e-6), max=1.0)
    next_position = position + DT * next_velocity
    return next_position, next_velocity


def collision_penalty(distances):
    return torch.relu(LIDAR_SAFE_DISTANCE - distances) ** 2


def directional_collision_penalty(distances, forward_cosines):
    """Penalize front obstacles more than lateral corridor boundaries.

    ``forward_cosines`` is the cosine between a robot-to-obstacle vector and
    the current robot-to-goal vector.  The configured front clearance applies
    only inside that forward cone; all other points use the lateral clearance.
    With the default constants both branches equal the original isotropic
    collision model.
    """
    front_clearance = FRONT_LIDAR_SAFE_DISTANCE + FRONT_PLANNING_CLEARANCE_MARGIN
    lateral_clearance = LIDAR_SAFE_DISTANCE + LATERAL_PLANNING_CLEARANCE_MARGIN
    clearance = torch.where(
        forward_cosines >= FORWARD_CLEARANCE_COSINE,
        torch.as_tensor(front_clearance, dtype=distances.dtype, device=distances.device),
        torch.as_tensor(lateral_clearance, dtype=distances.dtype, device=distances.device),
    )
    return torch.relu(clearance - distances) ** 2


def select_acceleration(robot_position, robot_velocity, tracks, map_points, current_world_points,
                        previous_plan=None, return_plan=False):
    goal_offset = GOAL - robot_position
    goal_distance = torch.linalg.norm(goal_offset)

    if goal_distance <= GOAL_TOLERANCE and torch.linalg.norm(robot_velocity) <= GOAL_SPEED_TOLERANCE:
        stopped_plan = torch.zeros(PLAN_HORIZON, 2)
        return (stopped_plan[0], stopped_plan) if return_plan else stopped_plan[0]

    angles = torch.arange(PLAN_HEADINGS) * 2.0 * torch.pi / PLAN_HEADINGS - torch.pi
    directions = torch.stack([torch.cos(angles), torch.sin(angles)], dim=1)
    accelerations = (PLAN_ACCELERATIONS[:, None, None] * directions[None, :, :]).reshape(-1, 2)
    goal_direction = goal_offset / goal_distance.clamp_min(1e-6)
    accelerations = torch.cat([torch.zeros(1, 2), accelerations,
                               MAX_ACCELERATION * goal_direction[None]])
    plans = accelerations[:, None, :].repeat(1, PLAN_HORIZON, 1)
    half_horizon = PLAN_HORIZON // 2
    move_and_stop_plans = torch.zeros(len(accelerations) - 1, PLAN_HORIZON, 2)
    move_and_stop_plans[:, :half_horizon] = accelerations[1:, None]
    move_and_stop_plans[:, half_horizon:2 * half_horizon] = -accelerations[1:, None]

    if previous_plan is None:
        warm_start = (MAX_ACCELERATION * goal_direction).repeat(PLAN_HORIZON, 1)
    else:
        warm_start = torch.cat([previous_plan[1:], torch.zeros(1, 2)])

    braking_plan = torch.zeros(PLAN_HORIZON, 2)
    braking_velocity = robot_velocity.clone()

    for step in range(PLAN_HORIZON):
        braking_acceleration = -braking_velocity / DT
        braking_acceleration *= torch.clamp(MAX_ACCELERATION /
                                            torch.linalg.norm(braking_acceleration).clamp_min(1e-6), max=1.0)
        braking_plan[step] = braking_acceleration
        braking_velocity += DT * braking_acceleration

    plans = torch.cat([warm_start[None], braking_plan[None], move_and_stop_plans, plans])
    predicted_position = robot_position.repeat(len(plans), 1)
    predicted_velocity = robot_velocity.repeat(len(plans), 1)
    position_predictions = []
    velocity_predictions = []

    for step in range(PLAN_HORIZON):
        predicted_velocity += DT * plans[:, step]
        predicted_speeds = torch.linalg.norm(predicted_velocity, dim=1)
        predicted_velocity *= torch.clamp(MAX_SPEED / predicted_speeds.clamp_min(1e-6), max=1.0)[:, None]
        predicted_position += DT * predicted_velocity
        position_predictions.append(predicted_position.clone())
        velocity_predictions.append(predicted_velocity.clone())

    robot_predictions = torch.stack(position_predictions, dim=1)
    robot_velocity_predictions = torch.stack(velocity_predictions, dim=1)
    goal_cost = GOAL_POSITION_COST * torch.sum((robot_predictions[:, -1] - GOAL) ** 2, dim=1)
    goal_cost += GOAL_VELOCITY_COST * torch.sum(robot_velocity_predictions[:, -1] ** 2, dim=1)
    effort_cost = ACCELERATION_COST * torch.sum(plans ** 2, dim=(1, 2))
    obstacle_cost = torch.zeros(len(plans))

    goal_directions = GOAL - robot_predictions
    goal_directions /= torch.linalg.norm(goal_directions, dim=2, keepdim=True).clamp_min(1e-6)

    if tracks:
        track_predictions = predict_tracks(tracks, PLAN_HORIZON)
        track_offsets = track_predictions[None, :, :, :] - robot_predictions[:, :, None, :]
        track_distances = torch.linalg.norm(track_offsets, dim=3) - OBSTACLE_RADIUS
        track_directions = track_offsets / torch.linalg.norm(track_offsets, dim=3, keepdim=True).clamp_min(1e-6)
        track_forward_cosines = torch.sum(track_directions * goal_directions[:, :, None, :], dim=3)
        obstacle_cost += COLLISION_COST * torch.sum(directional_collision_penalty(track_distances,
                                                                                    track_forward_cosines),
                                                    dim=(1, 2))

    planning_points = torch.cat([map_points, current_world_points]) if len(map_points) else current_world_points

    if len(planning_points):
        point_offsets = planning_points[None, None, :, :] - robot_predictions[:, :, None, :]
        point_distances = torch.linalg.norm(point_offsets, dim=3)
        point_directions = point_offsets / point_distances[:, :, :, None].clamp_min(1e-6)
        point_forward_cosines = torch.sum(point_directions * goal_directions[:, :, None, :], dim=3)
        obstacle_cost += COLLISION_COST * torch.sum(directional_collision_penalty(point_distances,
                                                                                    point_forward_cosines),
                                                    dim=(1, 2))

    wall_distances = torch.stack([robot_predictions[:, :, 0] - ROOM_X_MIN,
                                  ROOM_X_MAX - robot_predictions[:, :, 0],
                                  robot_predictions[:, :, 1] - ROOM_Y_MIN,
                                  ROOM_Y_MAX - robot_predictions[:, :, 1]], dim=2)
    obstacle_cost += COLLISION_COST * torch.sum(collision_penalty(wall_distances - PLANNING_CLEARANCE_MARGIN),
                                                dim=(1, 2))
    best_plan = plans[torch.argmin(goal_cost + effort_cost + obstacle_cost)]
    return (best_plan[0], best_plan) if return_plan else best_plan[0]


def run_navigation():
    robot_positions = [START.clone()]
    robot_velocities = [torch.zeros(2)]
    accelerations = []
    tracks = []
    track_histories = {}
    track_time_histories = {}
    track_frame_history = []
    velocity_histories = {}
    dynamic_track_ids = set()
    lidar_history = []
    world_point_history = []
    world_cluster_history = []
    nearest_distance_history = []
    map_points = torch.empty(0, 2)
    next_track_id = 0
    previous_plan = None
    progress = tqdm(range(NUM_STEPS + 1), disable=not INTERACTIVE)

    for time_index in progress:
        robot_position = robot_positions[-1]
        robot_velocity = robot_velocities[-1]
        lidar_points, lidar_hits = simulate_lidar(robot_position, time_index)
        detections, world_points, world_clusters = scan_to_detections(robot_position, lidar_points, lidar_hits)
        tracks, next_track_id = update_tracks(tracks, detections, next_track_id)

        for track in tracks:
            track_histories.setdefault(track["id"], []).append(track["position"].clone())
            track_time_histories.setdefault(track["id"], []).append(time_index)
            velocity_histories.setdefault(track["id"], []).append(track["velocity"].clone())

            if track["age"] >= 2 and torch.linalg.norm(track["velocity"]) > STATIC_SPEED_THRESHOLD:
                dynamic_track_ids.add(track["id"])

        moving_histories = [torch.stack(track_histories[track_id]) for track_id in dynamic_track_ids]
        active_centers = torch.stack([track["position"] for track in tracks]) if tracks else torch.empty(0, 2)
        moving_histories.append(active_centers)
        dynamic_centers = torch.cat(moving_histories) if moving_histories else torch.empty(0, 2)
        map_points = update_slam_map(map_points, world_points, dynamic_centers)

        track_frame_history.append([{"id": track["id"], "position": track["position"].clone(), "velocity": track["velocity"].clone(), "age": track["age"], "missed": track["missed"]} for track in tracks])
        lidar_history.append((lidar_points.clone(), lidar_hits.clone()))
        world_point_history.append(world_points.clone())
        world_cluster_history.append([cluster.clone() for cluster in world_clusters])
        measured_ranges = torch.linalg.norm(lidar_points[lidar_hits], dim=1)
        nearest_distance_history.append(measured_ranges.min() if len(measured_ranges) else torch.tensor(LIDAR_RANGE))

        reached_goal = torch.linalg.norm(GOAL - robot_position) <= GOAL_TOLERANCE
        stopped = torch.linalg.norm(robot_velocity) <= GOAL_SPEED_TOLERANCE

        if time_index == NUM_STEPS or reached_goal and stopped:
            break

        acceleration, previous_plan = select_acceleration(robot_position, robot_velocity, tracks, map_points,
                                                          world_points, previous_plan, return_plan=True)
        next_position, next_velocity = step_robot(robot_position, robot_velocity, acceleration)
        robot_positions.append(next_position)
        robot_velocities.append(next_velocity)
        accelerations.append(acceleration)

    return {"robot_positions": torch.stack(robot_positions),
            "robot_velocities": torch.stack(robot_velocities),
            "accelerations": torch.stack(accelerations) if accelerations else torch.empty(0, 2),
            "tracks": tracks, "track_histories": track_histories,
            "track_time_histories": track_time_histories, "track_frame_history": track_frame_history,
            "velocity_histories": velocity_histories, "lidar_history": lidar_history,
            "world_point_history": world_point_history, "world_cluster_history": world_cluster_history,
            "nearest_distance_history": torch.stack(nearest_distance_history), "map_points": map_points}


def plot_room():
    plt.plot([ROOM_X_MIN, ROOM_X_MAX, ROOM_X_MAX, ROOM_X_MIN, ROOM_X_MIN],
             [ROOM_Y_MIN, ROOM_Y_MIN, ROOM_Y_MAX, ROOM_Y_MAX, ROOM_Y_MIN], color="black", linewidth=2)


def plot_fading_trajectory(path, color, label=None):
    alphas = np.linspace(0.1, 1.0, len(path) - 1)

    for index, alpha in enumerate(alphas):
        line = plt.plot(path[index:index + 2, 0], path[index:index + 2, 1], color=color, linestyle="-", linewidth=2, alpha=alpha, solid_capstyle="round", label=label if index == len(alphas) - 1 else None)[0]
        line.set_gid("trajectory")


def plot_summary(result):
    final_positions = result["robot_positions"].numpy()
    final_accelerations = result["accelerations"].numpy()
    pose_indices = np.linspace(0, len(final_positions) - 1, 10, dtype=int)
    pose_alphas = np.linspace(0.1, 1.0, len(pose_indices))
    summary_figure = plt.figure(figsize=(8, 6))
    plt.subplot(221)
    plot_room()
    map_points = result["map_points"].numpy()
    plt.scatter(map_points[:, 0], map_points[:, 1], s=8, color="gray", alpha=0.35, label="Pose-aided point map")
    plot_fading_trajectory(final_positions, "royalblue", label="AGV")

    for track_id, history in result["track_histories"].items():
        color = ESTIMATE_COLORS[track_id % len(ESTIMATE_COLORS)]
        plot_fading_trajectory(torch.stack(history).numpy(), color, label=f"Estimate {track_id}")

    for index, alpha in zip(pose_indices, pose_alphas):
        lidar_cloud = result["world_point_history"][index].numpy()
        plt.scatter(lidar_cloud[:, 0], lidar_cloud[:, 1], s=8, color="tomato", alpha=alpha,
                    label="LiDAR points" if index == pose_indices[-1] else None)
        plt.gca().add_patch(Circle(final_positions[index], ROBOT_RADIUS, color="deepskyblue", alpha=alpha))

    plt.scatter(START[0], START[1], s=120, marker="o", color="royalblue", edgecolor="black", zorder=10)
    plt.scatter(GOAL[0], GOAL[1], s=180, marker="*", color="gold", edgecolor="black", zorder=10)
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.gca().set_aspect("equal", adjustable="box")
    plt.xlim(ROOM_X_MIN - 0.3, ROOM_X_MAX + 0.3)
    plt.ylim(ROOM_Y_MIN - 0.3, ROOM_Y_MAX + 0.3)
    plt.legend(fontsize=7, ncol=3)

    plt.subplot(222)

    for track_id, velocity_history in result["velocity_histories"].items():
        speeds = torch.linalg.norm(torch.stack(velocity_history), dim=1).numpy()
        plt.plot(np.arange(len(speeds)) * DT, speeds,
                 color=ESTIMATE_COLORS[track_id % len(ESTIMATE_COLORS)], label=f"Estimate {track_id}")

    plt.xlabel("Time [s]")
    plt.ylabel("Estimated speed [m/s]")
    plt.legend(fontsize=7, ncol=2)

    plt.subplot(223)
    time_positions = np.arange(len(final_positions)) * DT
    plt.plot(time_positions, result["nearest_distance_history"].numpy(), color="purple", linewidth=2,
             label="Nearest LiDAR")
    plt.axhline(ROBOT_RADIUS, color="red", linewidth=2, label="Contact distance")
    plt.axhline(LIDAR_SAFE_DISTANCE, color="darkorange", linestyle="--", linewidth=2, label="Safe distance")
    plt.fill_between(time_positions, 0, ROBOT_RADIUS, color="red", alpha=0.15)
    plt.xlabel("Time [s]")
    plt.ylabel("LiDAR distance [m]")
    plt.ylim(0, LIDAR_RANGE + 0.2)
    plt.legend(ncol=2)

    plt.subplot(224)
    time_accelerations = np.arange(len(final_accelerations)) * DT
    plt.plot(time_accelerations, final_accelerations[:, 0], color="royalblue", linewidth=2, label=r"$a_x$")
    plt.plot(time_accelerations, final_accelerations[:, 1], color="darkorange", linewidth=2, label=r"$a_y$")
    plt.plot(time_accelerations, np.linalg.norm(final_accelerations, axis=1), color="green", linestyle="--",
             linewidth=2, label="Acceleration")
    plt.axhline(MAX_ACCELERATION, color="black", linestyle=":", label="Acceleration limit")
    plt.xlabel("Time [s]")
    plt.ylabel("Acceleration [m/s²]")
    plt.legend(ncol=2)
    plt.tight_layout()
    return summary_figure


def draw_navigation_frame(result, time_index):
    final_positions = result["robot_positions"].numpy()
    robot_position = final_positions[time_index]
    map_points = result["map_points"].numpy()
    plt.clf()
    plt.xlim(ROOM_X_MIN - 0.3, ROOM_X_MAX + 0.3)
    plt.ylim(ROOM_Y_MIN - 0.3, ROOM_Y_MAX + 0.3)
    plt.gca().set_aspect("equal", adjustable="box")
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plot_room()
    plt.scatter(map_points[:, 0], map_points[:, 1], s=8, color="gray", alpha=0.25)
    plot_fading_trajectory(final_positions[:time_index + 1], "royalblue")

    for track_id, history in result["track_histories"].items():
        visible_indices = [index for index, sample_time in enumerate(result["track_time_histories"][track_id]) if sample_time <= time_index]

        if visible_indices:
            visible_history = torch.stack([history[index] for index in visible_indices]).numpy()
            plot_fading_trajectory(visible_history, ESTIMATE_COLORS[track_id % len(ESTIMATE_COLORS)])

    frame_tracks = result["track_frame_history"][time_index]
    track_predictions = predict_tracks(frame_tracks, PLAN_HORIZON)

    for track_index, track in enumerate(frame_tracks):
        color = ESTIMATE_COLORS[track["id"] % len(ESTIMATE_COLORS)]
        predicted_path = torch.cat([track["position"][None], track_predictions[:, track_index]]).numpy()
        prediction_line = plt.plot(predicted_path[:, 0], predicted_path[:, 1], color=color, linestyle="-", marker=".", markersize=3, linewidth=1, alpha=0.65)[0]
        prediction_line.set_gid("prediction")
        plt.gca().add_patch(Circle(track["position"].numpy(), OBSTACLE_RADIUS, color=color, alpha=0.12 if track["missed"] else 0.20))

    for cluster in result["world_cluster_history"][time_index]:
        cluster_points = cluster.numpy()
        entity_line = plt.plot(cluster_points[:, 0], cluster_points[:, 1], color="tomato", linestyle="-", marker=".", markersize=5, linewidth=1, alpha=0.85)[0]
        entity_line.set_gid("lidar_entity")

    goal_handle = plt.scatter(GOAL[0], GOAL[1], s=180, marker="*", color="gold", edgecolor="black", zorder=10, label="Goal")
    agv_handle = Circle(robot_position, ROBOT_RADIUS, color="deepskyblue", zorder=10, label="AGV")
    plt.gca().add_patch(agv_handle)
    plt.legend([goal_handle, agv_handle], ["Goal", "AGV"], loc="upper right", fontsize=8)
    plt.tight_layout()


def main():
    result = run_navigation()
    final_positions = result["robot_positions"].numpy()
    final_goal_error = np.linalg.norm(final_positions[-1] - GOAL.numpy())
    minimum_clearance = result["nearest_distance_history"].min().item() - LIDAR_SAFE_DISTANCE
    plot_summary(result)

    if not INTERACTIVE:
        print("No graphical Matplotlib backend is available; start this program from a graphical WSL/VS Code session.")
        return result

    from matplotlib.animation import FuncAnimation

    animation_figure = plt.figure(figsize=(6, 6))
    animation = FuncAnimation(animation_figure, lambda time_index: draw_navigation_frame(result, time_index),
                              frames=len(final_positions), interval=DT * 1000, repeat=True)
    # Retain the animation while its window is open; no file is written.
    animation_figure.navigation_animation = animation

    print(f"Final goal error: {final_goal_error:.3f} m")
    print(f"Minimum LiDAR clearance above safety distance: {minimum_clearance:.3f} m")
    plt.show()

    return result


if __name__ == "__main__":
    main()
