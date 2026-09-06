import json
import sys
from pathlib import Path

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from m1_scope_bridge.bag_export import (  # noqa: E402
    strict_json_dump, validate_scan_series)


def test_validate_scan_series_stacks_consistent_scans():
    records = [
        {
            "stamp_ns": 10,
            "ranges": [1.0, float("inf"), 3.0],
            "angle_min": -1.0,
            "angle_increment": 1.0,
            "range_min": 0.05,
            "range_max": 12.0,
            "frame_id": "laser_scan_link",
        },
        {
            "stamp_ns": 20,
            "ranges": [2.0, 4.0, 6.0],
            "angle_min": -1.0,
            "angle_increment": 1.0,
            "range_min": 0.05,
            "range_max": 12.0,
            "frame_id": "laser_scan_link",
        },
    ]

    arrays = validate_scan_series(records)

    assert arrays["scan_ranges"].shape == (2, 3)
    assert arrays["scan_ranges"].dtype == np.float32
    assert np.isinf(arrays["scan_ranges"][0, 1])
    assert arrays["scan_frame_id"].tolist() == ["laser_scan_link"] * 2


def test_validate_scan_series_rejects_geometry_changes():
    records = [
        {"stamp_ns": 1, "ranges": [1.0], "angle_min": 0.0,
         "angle_increment": 1.0, "range_min": 0.05,
         "range_max": 12.0, "frame_id": "laser"},
        {"stamp_ns": 2, "ranges": [1.0, 2.0], "angle_min": 0.0,
         "angle_increment": 1.0, "range_min": 0.05,
         "range_max": 12.0, "frame_id": "laser"},
    ]

    try:
        validate_scan_series(records)
    except ValueError as error:
        assert "beam count changed" in str(error)
    else:
        raise AssertionError("expected inconsistent scan geometry to fail")


def test_strict_json_dump_rejects_non_finite_values(tmp_path):
    output = tmp_path / "summary.json"
    try:
        strict_json_dump(output, {"bad": float("nan")})
    except ValueError:
        pass
    else:
        raise AssertionError("strict JSON must reject NaN")
    assert not output.exists()


def test_strict_json_dump_writes_portable_json(tmp_path):
    output = tmp_path / "summary.json"
    strict_json_dump(output, {"count": np.int64(3), "rate": np.float64(12.0)})
    assert json.loads(output.read_text()) == {"count": 3, "rate": 12.0}
