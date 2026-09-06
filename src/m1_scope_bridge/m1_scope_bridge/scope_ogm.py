"""Build endpoint-only, SCOPE-compatible OGM sequences from exported M1 data."""

import argparse
import json
from pathlib import Path

import numpy as np

from .bag_export import strict_json_dump
from .se2 import (
    integrate_body_twist, invert_transform, matrix_to_pose, pose_to_matrix,
    transform_points)
from .synchronization import interpolate_odometry, quaternion_to_yaw


SEQ_LEN = 10
GRID_SIZE = 64
RESOLUTION = 0.1
X_LIMIT = (0.0, 6.4)
Y_LIMIT = (-3.2, 3.2)
HISTORY_PERIOD_NS = 100_000_000
SCAN_TOLERANCE_NS = 50_000_000


def scan_to_points(ranges, angle_min, angle_increment, range_min, range_max):
    ranges = np.asarray(ranges, dtype=np.float64)
    angles = float(angle_min) + np.arange(len(ranges)) * float(angle_increment)
    valid = (np.isfinite(ranges) & (ranges >= float(range_min)) &
             (ranges <= float(range_max)))
    return np.column_stack(
        (ranges[valid] * np.cos(angles[valid]),
         ranges[valid] * np.sin(angles[valid])))


def points_to_grid(points):
    points = np.asarray(points, dtype=np.float64)
    grid = np.zeros((1, GRID_SIZE, GRID_SIZE), dtype=np.uint8)
    if len(points) == 0:
        return grid
    rows = np.floor((points[:, 0] - X_LIMIT[0]) / RESOLUTION).astype(np.int64)
    columns = np.floor((points[:, 1] - Y_LIMIT[0]) / RESOLUTION).astype(np.int64)
    valid = ((rows >= 0) & (rows < GRID_SIZE) &
             (columns >= 0) & (columns < GRID_SIZE))
    grid[0, rows[valid], columns[valid]] = 1
    return grid


def find_nearest_indices(reference_ns, query_ns, tolerance_ns=SCAN_TOLERANCE_NS):
    """Find deterministic nearest samples; exact ties select the earlier sample."""
    reference_ns = np.asarray(reference_ns, dtype=np.int64)
    query_ns = np.asarray(query_ns, dtype=np.int64)
    if len(reference_ns) == 0 or np.any(np.diff(reference_ns) <= 0):
        raise ValueError("reference timestamps must be non-empty and increasing")
    insertion = np.searchsorted(reference_ns, query_ns, side="left")
    later = np.clip(insertion, 0, len(reference_ns) - 1)
    earlier = np.clip(insertion - 1, 0, len(reference_ns) - 1)
    earlier_error = np.abs(query_ns - reference_ns[earlier])
    later_error = np.abs(reference_ns[later] - query_ns)
    use_later = later_error < earlier_error
    indices = np.where(use_later, later, earlier).astype(np.int64)
    errors = np.where(use_later, later_error, earlier_error).astype(np.int64)
    indices[errors > int(tolerance_ns)] = -1
    return indices, errors


def _quaternion_planar_pose(translation, quaternion):
    yaw = quaternion_to_yaw(*[float(value) for value in quaternion])
    return np.array([float(translation[0]), float(translation[1]), yaw])


def resolve_static_transform(parents, children, translations, quaternions,
                             source_frame, target_frame):
    """Resolve source->target across a bidirectional static TF graph."""
    graph = {}
    for parent, child, translation, quaternion in zip(
            parents, children, translations, quaternions):
        forward = pose_to_matrix(_quaternion_planar_pose(translation, quaternion))
        graph.setdefault(str(parent), []).append((str(child), forward))
        graph.setdefault(str(child), []).append((str(parent), invert_transform(forward)))
    queue = [(str(source_frame), np.eye(3, dtype=np.float64))]
    visited = set()
    while queue:
        frame, transform = queue.pop(0)
        if frame == str(target_frame):
            return transform
        if frame in visited:
            continue
        visited.add(frame)
        for neighbor, edge in graph.get(frame, []):
            if neighbor not in visited:
                queue.append((neighbor, transform.dot(edge)))
    raise ValueError("no static transform chain from %s to %s" %
                     (source_frame, target_frame))


def compensate_points(points, past_laser_pose, future_laser_pose):
    past = (pose_to_matrix(past_laser_pose)
            if np.asarray(past_laser_pose).shape == (3,) else past_laser_pose)
    future = (pose_to_matrix(future_laser_pose)
              if np.asarray(future_laser_pose).shape == (3,) else future_laser_pose)
    return transform_points(invert_transform(future).dot(past), points)


def _scan_points(data, index):
    return scan_to_points(
        data["scan_ranges"][index], data["scan_angle_min"][index],
        data["scan_angle_increment"][index], data["scan_range_min"][index],
        data["scan_range_max"][index])


def _interpolate_obstacles(stamps_ns, poses, query_ns):
    stamps_ns = np.asarray(stamps_ns, dtype=np.int64)
    if query_ns < stamps_ns[0] or query_ns > stamps_ns[-1]:
        raise ValueError("future scan is outside dynamic-obstacle coverage")
    right = int(np.searchsorted(stamps_ns, query_ns, side="left"))
    if right == 0:
        return poses[0, :, :2]
    if right == len(stamps_ns):
        return poses[-1, :, :2]
    left = right - 1
    alpha = float(query_ns - stamps_ns[left]) / float(stamps_ns[right] - stamps_ns[left])
    return poses[left, :, :2] + alpha * (poses[right, :, :2] - poses[left, :, :2])


def _roi_grid(points, radius):
    rows = X_LIMIT[0] + (np.arange(GRID_SIZE) + 0.5) * RESOLUTION
    columns = Y_LIMIT[0] + (np.arange(GRID_SIZE) + 0.5) * RESOLUTION
    x_grid, y_grid = np.meshgrid(rows, columns, indexing="ij")
    roi = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.bool_)
    for point in points:
        roi |= ((x_grid - point[0]) ** 2 + (y_grid - point[1]) ** 2
                <= float(radius) ** 2)
    return roi[np.newaxis].astype(np.uint8)


def validate_no_slip_roi_semantics(odom_pose, ground_truth_pose,
                                   translation_limit=0.02, yaw_limit=0.02):
    """Guard Gazebo-world PoseArray coordinates from slipped odom semantics."""
    odom_pose = np.asarray(odom_pose, dtype=np.float64)
    ground_truth_pose = np.asarray(ground_truth_pose, dtype=np.float64)
    if odom_pose.shape != ground_truth_pose.shape or odom_pose.shape[1] != 3:
        raise ValueError("odom and ground-truth poses must have matching [N,3] shape")
    delta = odom_pose - ground_truth_pose
    delta[:, 2] = np.arctan2(np.sin(delta[:, 2]), np.cos(delta[:, 2]))
    translation_rmse = float(np.sqrt(np.mean(np.sum(delta[:, :2] ** 2, axis=1))))
    yaw_rmse = float(np.sqrt(np.mean(delta[:, 2] ** 2)))
    if translation_rmse > float(translation_limit) or yaw_rmse > float(yaw_limit):
        raise ValueError(
            "dynamic-obstacle ROI requires no-slip odometry semantics; "
            "translation_rmse=%.6f yaw_rmse=%.6f" %
            (translation_rmse, yaw_rmse))
    return translation_rmse, yaw_rmse


def _base_laser_transform(data, fallback_transform):
    frame_ids = sorted(set(data["scan_frame_id"].tolist()))
    if len(frame_ids) != 1:
        raise ValueError("preprocessing requires one scan frame")
    static = data["tf_is_static"].astype(bool)
    try:
        return resolve_static_transform(
            data["tf_parent"][static], data["tf_child"][static],
            data["tf_translation"][static], data["tf_quaternion"][static],
            "base_footprint", frame_ids[0]), "tf_static"
    except ValueError:
        if fallback_transform is None:
            raise
        return pose_to_matrix(fallback_transform), "cli_fallback"


def preprocess_dataset(input_path, output_path, horizon_steps=5,
                       tolerance_ns=SCAN_TOLERANCE_NS, roi_radius=0.45,
                       fallback_transform=None):
    if not 1 <= int(horizon_steps) <= 10:
        raise ValueError("horizon must be between 1 and 10 steps")
    with np.load(str(input_path), allow_pickle=False) as loaded:
        data = {name: loaded[name] for name in loaded.files}
    if str(data["schema_version"].item()) != "m1_scope_raw_v1":
        raise ValueError("unsupported raw dataset schema")
    scan_stamps = data["scan_stamp_ns"]
    base_laser, transform_source = _base_laser_transform(data, fallback_transform)
    horizon_ns = int(horizon_steps) * HISTORY_PERIOD_NS
    candidate_count = max(
        0, int((scan_stamps[-1] - scan_stamps[0] - horizon_ns) //
               HISTORY_PERIOD_NS) - (SEQ_LEN - 1) + 1)
    anchor_targets = (scan_stamps[0] + (SEQ_LEN - 1) * HISTORY_PERIOD_NS +
                      np.arange(candidate_count, dtype=np.int64) * HISTORY_PERIOD_NS)

    all_inputs = []
    all_ground_truth = []
    all_roi = []
    accepted_anchor_target = []
    accepted_anchor_stamp = []
    accepted_target_target = []
    accepted_target_stamp = []
    accepted_history_stamps = []
    accepted_history_errors = []
    predicted_future_poses = []
    true_future_poses = []
    invalid = {"scan_tolerance": 0, "duplicate_history": 0, "coverage": 0}

    odom_stamps = data["odom_stamp_ns"]
    gt_stamps = data["ground_truth_odom_stamp_ns"]
    dynamic_stamps = data["dynamic_obstacle_stamp_ns"]
    audit_mask = ((scan_stamps >= max(odom_stamps[0], gt_stamps[0])) &
                  (scan_stamps <= min(odom_stamps[-1], gt_stamps[-1])))
    audit_stamps = scan_stamps[audit_mask]
    audit_odom, _ = interpolate_odometry(
        odom_stamps, data["odom_pose"], data["odom_twist"], audit_stamps)
    audit_gt, _ = interpolate_odometry(
        gt_stamps, data["ground_truth_odom_pose"],
        data["ground_truth_odom_twist"], audit_stamps)
    roi_translation_rmse, roi_yaw_rmse = validate_no_slip_roi_semantics(
        audit_odom, audit_gt)
    for anchor_target in anchor_targets:
        history_targets = anchor_target - np.arange(
            SEQ_LEN - 1, -1, -1, dtype=np.int64) * HISTORY_PERIOD_NS
        target_target = anchor_target + horizon_ns
        history_indices, history_errors = find_nearest_indices(
            scan_stamps, history_targets, tolerance_ns)
        target_indices, _ = find_nearest_indices(
            scan_stamps, np.array([target_target]), tolerance_ns)
        if np.any(history_indices < 0) or target_indices[0] < 0:
            invalid["scan_tolerance"] += 1
            continue
        if np.any(np.diff(history_indices) <= 0):
            invalid["duplicate_history"] += 1
            continue
        target_index = int(target_indices[0])
        history_actual = scan_stamps[history_indices]
        target_actual = int(scan_stamps[target_index])
        required = np.concatenate((history_actual, [target_actual]))
        coverage_start = max(odom_stamps[0], gt_stamps[0], dynamic_stamps[0])
        coverage_end = min(odom_stamps[-1], gt_stamps[-1], dynamic_stamps[-1])
        if required.min() < coverage_start or required.max() > coverage_end:
            invalid["coverage"] += 1
            continue

        past_pose, _ = interpolate_odometry(
            odom_stamps, data["odom_pose"], data["odom_twist"], history_actual)
        current_pose, current_twist = interpolate_odometry(
            odom_stamps, data["odom_pose"], data["odom_twist"],
            np.array([history_actual[-1]], dtype=np.int64))
        dt = float(target_actual - history_actual[-1]) / 1e9
        future_pose = integrate_body_twist(current_pose[0], current_twist[0], dt)
        future_laser = pose_to_matrix(future_pose).dot(base_laser)
        input_sequence = []
        for scan_index, pose in zip(history_indices, past_pose):
            past_laser = pose_to_matrix(pose).dot(base_laser)
            points = compensate_points(_scan_points(data, scan_index),
                                       past_laser, future_laser)
            input_sequence.append(points_to_grid(points))

        gt_pose, _ = interpolate_odometry(
            gt_stamps, data["ground_truth_odom_pose"],
            data["ground_truth_odom_twist"], np.array([target_actual]))
        gt_laser = pose_to_matrix(gt_pose[0]).dot(base_laser)
        obstacle_world = _interpolate_obstacles(
            dynamic_stamps, data["dynamic_obstacle_pose"], target_actual)
        obstacle_laser = transform_points(invert_transform(gt_laser), obstacle_world)

        all_inputs.append(np.stack(input_sequence, axis=0))
        all_ground_truth.append(points_to_grid(_scan_points(data, target_index)))
        all_roi.append(_roi_grid(obstacle_laser, roi_radius))
        accepted_anchor_target.append(anchor_target)
        accepted_anchor_stamp.append(history_actual[-1])
        accepted_target_target.append(target_target)
        accepted_target_stamp.append(target_actual)
        accepted_history_stamps.append(history_actual)
        accepted_history_errors.append(history_errors)
        predicted_future_poses.append(future_pose)
        true_future_poses.append(gt_pose[0])

    if not all_inputs:
        raise ValueError("no valid SCOPE windows were produced")
    input_ogm = np.stack(all_inputs).astype(np.uint8)
    ground_truth = np.stack(all_ground_truth).astype(np.uint8)
    dynamic_roi = np.stack(all_roi).astype(np.uint8)
    output = {
        "schema_version": np.asarray("m1_scope_ogm_v1"),
        "horizon_steps": np.asarray(int(horizon_steps), dtype=np.int64),
        "input_ogm": input_ogm,
        "ground_truth_ogm": ground_truth,
        "copy_last_ogm": input_ogm[:, -1],
        "dynamic_roi": dynamic_roi,
        "anchor_target_stamp_ns": np.asarray(accepted_anchor_target, dtype=np.int64),
        "anchor_scan_stamp_ns": np.asarray(accepted_anchor_stamp, dtype=np.int64),
        "target_target_stamp_ns": np.asarray(accepted_target_target, dtype=np.int64),
        "target_scan_stamp_ns": np.asarray(accepted_target_stamp, dtype=np.int64),
        "history_scan_stamp_ns": np.asarray(accepted_history_stamps, dtype=np.int64),
        "history_mismatch_ns": np.asarray(accepted_history_errors, dtype=np.int64),
        "predicted_future_pose": np.asarray(predicted_future_poses, dtype=np.float64),
        "ground_truth_future_pose": np.asarray(true_future_poses, dtype=np.float64),
        "base_to_laser_pose": matrix_to_pose(base_laser),
        "transform_source": np.asarray(transform_source),
        "grid_resolution": np.asarray(RESOLUTION, dtype=np.float64),
        "x_limit": np.asarray(X_LIMIT, dtype=np.float64),
        "y_limit": np.asarray(Y_LIMIT, dtype=np.float64),
        "roi_radius": np.asarray(roi_radius, dtype=np.float64),
    }
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(str(output_path), **output)
    pose_delta = output["predicted_future_pose"] - output["ground_truth_future_pose"]
    pose_delta[:, 2] = np.arctan2(np.sin(pose_delta[:, 2]), np.cos(pose_delta[:, 2]))
    summary = {
        "schema_version": "m1_scope_preprocess_summary_v1",
        "input": str(Path(input_path).resolve()),
        "output": str(output_path.resolve()),
        "horizon_steps": int(horizon_steps),
        "horizon_seconds": float(horizon_steps) / 10.0,
        "candidate_windows": len(anchor_targets),
        "valid_windows": len(input_ogm),
        "invalid_windows": invalid,
        "input_shape": list(input_ogm.shape),
        "ground_truth_shape": list(ground_truth.shape),
        "binary_input": bool(np.all((input_ogm == 0) | (input_ogm == 1))),
        "transform_source": transform_source,
        "base_to_laser_pose": output["base_to_laser_pose"].tolist(),
        "history_mismatch_max_s": float(output["history_mismatch_ns"].max()) / 1e9,
        "future_pose_translation_rmse_m": float(np.sqrt(np.mean(
            np.sum(pose_delta[:, :2] ** 2, axis=1)))),
        "future_pose_yaw_rmse_rad": float(np.sqrt(np.mean(pose_delta[:, 2] ** 2))),
        "dynamic_roi_nonempty_windows": int(np.count_nonzero(
            dynamic_roi.reshape(len(dynamic_roi), -1).any(axis=1))),
        "roi_frame_odom_ground_truth_translation_rmse_m": roi_translation_rmse,
        "roi_frame_odom_ground_truth_yaw_rmse_rad": roi_yaw_rmse,
    }
    strict_json_dump(output_path.with_name("preprocess_summary.json"), summary)
    return summary


def _parse_fallback(value):
    if value is None:
        return None
    parts = [float(item) for item in value.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("fallback transform must be x,y,yaw")
    return np.asarray(parts, dtype=np.float64)


def _parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--horizon", type=int, default=5)
    parser.add_argument("--scan-tolerance", type=float, default=0.05)
    parser.add_argument("--roi-radius", type=float, default=0.45)
    parser.add_argument("--laser-transform", type=_parse_fallback)
    parser.add_argument("--diagnostics-dir")
    return parser.parse_args(argv)


def main(argv=None):
    args = _parse_args(argv)
    summary = preprocess_dataset(
        args.dataset, args.output, horizon_steps=args.horizon,
        tolerance_ns=int(args.scan_tolerance * 1e9), roi_radius=args.roi_radius,
        fallback_transform=args.laser_transform)
    if args.diagnostics_dir:
        from .visualization import save_preprocess_diagnostics
        save_preprocess_diagnostics(args.output, args.diagnostics_dir)
    print(json.dumps(summary, indent=2, sort_keys=True, allow_nan=False))


if __name__ == "__main__":
    main()
