"""Export M1 ROS 2 bags to a stable NumPy dataset without PyTorch."""

import argparse
import json
from pathlib import Path

import numpy as np

from .synchronization import (
    common_coverage_mask, interpolate_odometry, nearest_time_offsets,
    quaternion_to_yaw)


REQUIRED_TOPICS = (
    "/scan", "/odom", "/ground_truth/odom", "/m1/dynamic_obstacles",
)


def _json_value(value):
    if isinstance(value, dict):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_value(item) for item in value]
    if isinstance(value, np.ndarray):
        return _json_value(value.tolist())
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, np.floating):
        return float(value)
    return value


def strict_json_dump(path, payload):
    """Write standards-compliant JSON, rejecting NaN and Infinity."""
    encoded = json.dumps(
        _json_value(payload), indent=2, sort_keys=True, allow_nan=False) + "\n"
    Path(path).write_text(encoded, encoding="utf-8")


def validate_scan_series(records):
    if not records:
        raise ValueError("bag contains no /scan messages")
    beam_count = len(records[0]["ranges"])
    first_geometry = (
        records[0]["angle_min"], records[0]["angle_increment"],
        records[0]["range_min"], records[0]["range_max"],
        records[0]["frame_id"],
    )
    for index, record in enumerate(records):
        if len(record["ranges"]) != beam_count:
            raise ValueError("scan beam count changed at record %d" % index)
        geometry = (
            record["angle_min"], record["angle_increment"],
            record["range_min"], record["range_max"], record["frame_id"],
        )
        numeric_equal = np.allclose(
            geometry[:4], first_geometry[:4], rtol=0.0, atol=1e-7)
        if not numeric_equal or geometry[4] != first_geometry[4]:
            raise ValueError("scan geometry changed at record %d" % index)
    return {
        "scan_stamp_ns": np.asarray(
            [record["stamp_ns"] for record in records], dtype=np.int64),
        "scan_record_stamp_ns": np.asarray(
            [record["record_stamp_ns"] for record in records], dtype=np.int64)
        if "record_stamp_ns" in records[0] else np.asarray(
            [record["stamp_ns"] for record in records], dtype=np.int64),
        "scan_ranges": np.asarray(
            [record["ranges"] for record in records], dtype=np.float32),
        "scan_angle_min": np.asarray(
            [record["angle_min"] for record in records], dtype=np.float32),
        "scan_angle_increment": np.asarray(
            [record["angle_increment"] for record in records], dtype=np.float32),
        "scan_range_min": np.asarray(
            [record["range_min"] for record in records], dtype=np.float32),
        "scan_range_max": np.asarray(
            [record["range_max"] for record in records], dtype=np.float32),
        "scan_frame_id": np.asarray(
            [record["frame_id"] for record in records], dtype="U128"),
    }


def _stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _pose_array(pose):
    orientation = pose.orientation
    return [
        float(pose.position.x), float(pose.position.y),
        quaternion_to_yaw(
            orientation.x, orientation.y, orientation.z, orientation.w),
    ]


def _twist_array(twist):
    return [float(twist.linear.x), float(twist.linear.y),
            float(twist.angular.z)]


def _rate_summary(stamps_ns):
    stamps_ns = np.asarray(stamps_ns, dtype=np.int64)
    if len(stamps_ns) < 2:
        return {"count": len(stamps_ns), "duration_s": 0.0, "average_rate_hz": None}
    duration = float(stamps_ns[-1] - stamps_ns[0]) / 1e9
    return {
        "count": len(stamps_ns),
        "duration_s": duration,
        "average_rate_hz": float(len(stamps_ns) - 1) / duration if duration > 0 else None,
    }


def _records_to_odom(records, prefix):
    return {
        prefix + "_stamp_ns": np.asarray([item[0] for item in records], dtype=np.int64),
        prefix + "_pose": np.asarray([item[1] for item in records], dtype=np.float64),
        prefix + "_twist": np.asarray([item[2] for item in records], dtype=np.float64),
    }


def _transform_record(transform, is_static, record_stamp_ns):
    value = transform.transform
    rotation = value.rotation
    return (
        _stamp_ns(transform.header.stamp), int(record_stamp_ns),
        str(transform.header.frame_id), str(transform.child_frame_id),
        float(value.translation.x), float(value.translation.y),
        float(value.translation.z), float(rotation.x), float(rotation.y),
        float(rotation.z), float(rotation.w), bool(is_static),
    )


def _read_bag(uri):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(uri), storage_id=""),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()}
    missing = [topic for topic in REQUIRED_TOPICS if topic not in topic_types]
    if missing:
        raise ValueError("bag is missing required topics: %s" % ", ".join(missing))

    records = {name: [] for name in topic_types}
    record_bounds = []
    while reader.has_next():
        topic, serialized, record_stamp_ns = reader.read_next()
        if topic not in records:
            continue
        message = deserialize_message(serialized, get_message(topic_types[topic]))
        records[topic].append((message, int(record_stamp_ns)))
        record_bounds.append(int(record_stamp_ns))
    return records, topic_types, record_bounds


def export_bag(bag_uri, output_dir):
    records, topic_types, record_bounds = _read_bag(bag_uri)
    scans = []
    for message, record_stamp in records["/scan"]:
        scans.append({
            "stamp_ns": _stamp_ns(message.header.stamp),
            "record_stamp_ns": record_stamp,
            "ranges": message.ranges,
            "angle_min": message.angle_min,
            "angle_increment": message.angle_increment,
            "range_min": message.range_min,
            "range_max": message.range_max,
            "frame_id": message.header.frame_id,
        })
    arrays = validate_scan_series(scans)

    for topic, prefix in (("/odom", "odom"),
                          ("/ground_truth/odom", "ground_truth_odom")):
        converted = []
        for message, _ in records[topic]:
            converted.append((
                _stamp_ns(message.header.stamp), _pose_array(message.pose.pose),
                _twist_array(message.twist.twist)))
        if len(converted) < 2:
            raise ValueError("bag requires at least two %s messages" % topic)
        arrays.update(_records_to_odom(converted, prefix))

    dynamic = records["/m1/dynamic_obstacles"]
    obstacle_counts = {len(message.poses) for message, _ in dynamic}
    if not dynamic or len(obstacle_counts) != 1:
        raise ValueError("dynamic obstacle PoseArray count is missing or inconsistent")
    arrays["dynamic_obstacle_stamp_ns"] = np.asarray(
        [_stamp_ns(message.header.stamp) for message, _ in dynamic], dtype=np.int64)
    arrays["dynamic_obstacle_pose"] = np.asarray([
        [[pose.position.x, pose.position.y, pose.position.z,
          pose.orientation.x, pose.orientation.y, pose.orientation.z,
          pose.orientation.w] for pose in message.poses]
        for message, _ in dynamic
    ], dtype=np.float64)
    arrays["dynamic_obstacle_frame_id"] = np.asarray(
        [message.header.frame_id for message, _ in dynamic], dtype="U128")

    cmd_records = records.get("/cmd_vel", [])
    arrays["cmd_vel_record_stamp_ns"] = np.asarray(
        [stamp for _, stamp in cmd_records], dtype=np.int64)
    arrays["cmd_vel"] = np.asarray(
        [_twist_array(message) for message, _ in cmd_records], dtype=np.float64
    ).reshape((-1, 3))

    transforms = []
    for topic, is_static in (("/tf", False), ("/tf_static", True)):
        for message, record_stamp in records.get(topic, []):
            transforms.extend(
                _transform_record(item, is_static, record_stamp)
                for item in message.transforms)
    arrays["tf_stamp_ns"] = np.asarray([item[0] for item in transforms], dtype=np.int64)
    arrays["tf_record_stamp_ns"] = np.asarray([item[1] for item in transforms], dtype=np.int64)
    arrays["tf_parent"] = np.asarray([item[2] for item in transforms], dtype="U128")
    arrays["tf_child"] = np.asarray([item[3] for item in transforms], dtype="U128")
    arrays["tf_translation"] = np.asarray(
        [item[4:7] for item in transforms], dtype=np.float64).reshape((-1, 3))
    arrays["tf_quaternion"] = np.asarray(
        [item[7:11] for item in transforms], dtype=np.float64).reshape((-1, 4))
    arrays["tf_is_static"] = np.asarray([item[11] for item in transforms], dtype=np.bool_)

    clock_records = records.get("/clock", [])
    arrays["clock_stamp_ns"] = np.asarray(
        [_stamp_ns(message.clock) for message, _ in clock_records], dtype=np.int64)
    arrays["clock_record_stamp_ns"] = np.asarray(
        [stamp for _, stamp in clock_records], dtype=np.int64)
    arrays["schema_version"] = np.asarray("m1_scope_raw_v1")

    scan_stamps = arrays["scan_stamp_ns"]
    odom_stamps = arrays["odom_stamp_ns"]
    offsets = nearest_time_offsets(odom_stamps, scan_stamps).astype(np.float64) / 1e9
    covered = common_coverage_mask(
        scan_stamps, odom_stamps, arrays["ground_truth_odom_stamp_ns"])
    if not np.any(covered):
        raise ValueError("scan, odometry, and ground-truth timelines do not overlap")
    synchronized_pose, synchronized_twist = interpolate_odometry(
        odom_stamps, arrays["odom_pose"], arrays["odom_twist"], scan_stamps[covered])
    motion = synchronized_twist
    motion_counts = {
        "forward": int(np.count_nonzero(np.abs(motion[:, 0]) >= 0.05)),
        "lateral": int(np.count_nonzero(np.abs(motion[:, 1]) >= 0.05)),
        "rotation": int(np.count_nonzero(np.abs(motion[:, 2]) >= 0.10)),
        "lateral_and_rotation": int(np.count_nonzero(
            (np.abs(motion[:, 1]) >= 0.05) & (np.abs(motion[:, 2]) >= 0.10))),
    }

    gt_pose, _ = interpolate_odometry(
        arrays["ground_truth_odom_stamp_ns"], arrays["ground_truth_odom_pose"],
        arrays["ground_truth_odom_twist"], scan_stamps[covered])
    pose_error = synchronized_pose - gt_pose
    pose_error[:, 2] = np.arctan2(
        np.sin(pose_error[:, 2]), np.cos(pose_error[:, 2]))

    finite = np.isfinite(arrays["scan_ranges"])
    range_valid = finite & (
        arrays["scan_ranges"] >= arrays["scan_range_min"][:, None]) & (
        arrays["scan_ranges"] <= arrays["scan_range_max"][:, None])
    summary = {
        "schema_version": "m1_scope_export_summary_v1",
        "bag_uri": str(Path(bag_uri).resolve()),
        "bag_duration_s": ((max(record_bounds) - min(record_bounds)) / 1e9
                           if len(record_bounds) >= 2 else 0.0),
        "topic_types": topic_types,
        "topics": {
            "/scan": _rate_summary(scan_stamps),
            "/odom": _rate_summary(odom_stamps),
            "/ground_truth/odom": _rate_summary(
                arrays["ground_truth_odom_stamp_ns"]),
            "/m1/dynamic_obstacles": _rate_summary(
                arrays["dynamic_obstacle_stamp_ns"]),
        },
        "scan": {
            "beam_count": int(arrays["scan_ranges"].shape[1]),
            "frame_ids": sorted(set(arrays["scan_frame_id"].tolist())),
            "finite_value_count": int(np.count_nonzero(finite)),
            "valid_range_count": int(np.count_nonzero(range_valid)),
            "invalid_value_count": int(range_valid.size - np.count_nonzero(range_valid)),
        },
        "synchronization": {
            "valid_scan_frames": int(np.count_nonzero(covered)),
            "scan_odom_mismatch_mean_s": float(offsets.mean()),
            "scan_odom_mismatch_p95_s": float(np.percentile(offsets, 95.0)),
            "scan_odom_mismatch_max_s": float(offsets.max()),
        },
        "motion_coverage_frames": motion_counts,
        "odom_vs_ground_truth_at_scan": {
            "translation_rmse_m": float(np.sqrt(np.mean(
                np.sum(pose_error[:, :2] ** 2, axis=1)))),
            "yaw_rmse_rad": float(np.sqrt(np.mean(pose_error[:, 2] ** 2))),
        },
        "tf_static_count": int(np.count_nonzero(arrays["tf_is_static"])),
        "dynamic_obstacle_count": int(next(iter(obstacle_counts))),
    }

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(str(output_dir / "raw_dataset.npz"), **arrays)
    strict_json_dump(output_dir / "export_summary.json", summary)
    return summary


def _parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, help="rosbag2 directory")
    parser.add_argument("--output-dir", required=True)
    return parser.parse_args(argv)


def main(argv=None):
    args = _parse_args(argv)
    summary = export_bag(args.bag, args.output_dir)
    print(json.dumps(summary, indent=2, sort_keys=True, allow_nan=False))


if __name__ == "__main__":
    main()
