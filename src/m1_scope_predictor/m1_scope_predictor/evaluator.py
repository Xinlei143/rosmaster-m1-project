"""Delayed endpoint-grid evaluation utilities."""

from collections import deque

import numpy as np


class PendingEvaluationQueue:
    """Bounded forecast queue that matches results after their target time."""

    def __init__(self, maximum=32):
        self._values = deque(maxlen=int(maximum))

    def __len__(self):
        return len(self._values)

    def add(self, result):
        self._values.append(result)

    def restore_front(self, result):
        self._values.appendleft(result)

    def pop_match(self, scan_stamps, tolerance_ns):
        stamps = np.asarray(scan_stamps, dtype=np.int64)
        if len(stamps) == 0:
            return None
        while self._values:
            result = self._values[0]
            target = int(result.job.target_stamp_ns)
            if stamps[-1] < target - int(tolerance_ns):
                return None
            index = int(np.argmin(np.abs(stamps - target)))
            error = abs(int(stamps[index]) - target)
            self._values.popleft()
            if error <= int(tolerance_ns):
                return result, index
        return None


def endpoint_metrics(probability, target, threshold=0.5):
    probability = np.asarray(probability, dtype=np.float64)
    target = np.asarray(target, dtype=bool)
    if probability.shape != target.shape:
        raise ValueError("prediction and target shapes differ")
    predicted = probability >= float(threshold)
    tp = int(np.count_nonzero(predicted & target))
    fp = int(np.count_nonzero(predicted & ~target))
    fn = int(np.count_nonzero(~predicted & target))
    iou_denominator = tp + fp + fn
    f1_denominator = 2 * tp + fp + fn
    return {
        "mae": float(np.mean(np.abs(probability - target))),
        "occupied_iou": (float(tp) / iou_denominator
                         if iou_denominator else None),
        "f1": (float(2 * tp) / f1_denominator if f1_denominator else None),
        "tp": tp,
        "fp": fp,
        "fn": fn,
    }
