import sys
from pathlib import Path

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from m1_scope_predictor.evaluator import (  # noqa: E402
    PendingEvaluationQueue,
    endpoint_metrics,
)


def test_endpoint_metrics_reports_iou_f1_and_mae():
    prediction = np.array([[[0.9, 0.7], [0.2, 0.1]]], dtype=np.float32)
    target = np.array([[[1, 0], [0, 0]]], dtype=np.uint8)
    metrics = endpoint_metrics(prediction, target, threshold=0.5)
    assert metrics["tp"] == 1
    assert metrics["fp"] == 1
    assert metrics["fn"] == 0
    assert metrics["occupied_iou"] == 0.5
    assert np.isclose(metrics["f1"], 2.0 / 3.0)
    assert np.isclose(metrics["mae"], 0.275)


def test_endpoint_metrics_uses_none_when_occupancy_denominator_is_empty():
    metrics = endpoint_metrics(
        np.zeros((1, 2, 2), dtype=np.float32),
        np.zeros((1, 2, 2), dtype=np.uint8))
    assert metrics["occupied_iou"] is None
    assert metrics["f1"] is None


def test_pending_evaluator_retains_newer_predictions_while_matching_oldest():
    class Job:
        def __init__(self, target_stamp_ns):
            self.target_stamp_ns = target_stamp_ns

    class Result:
        def __init__(self, target_stamp_ns):
            self.job = Job(target_stamp_ns)

    queue = PendingEvaluationQueue(maximum=4)
    first = Result(1_000)
    second = Result(2_000)
    queue.add(first)
    queue.add(second)
    matched = queue.pop_match([900, 1_010, 1_900], tolerance_ns=50)
    assert matched == (first, 1)
    assert len(queue) == 1
    assert queue.pop_match([1_900, 2_010], tolerance_ns=50) == (second, 1)
