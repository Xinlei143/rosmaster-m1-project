import sys
import unittest
from pathlib import Path

import numpy as np


TOOL_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOL_ROOT))

from evaluate_pretrained import (  # noqa: E402
    classify_zero_shot, compute_metrics, latency_measurement_scope,
    select_explicit_indices, select_window_indices)
from visualize_prediction import (  # noqa: E402
    dynamic_obstacle_rank, representative_rank,
    stratified_dynamic_obstacle_ranks)
from visualize_timeline import matching_index  # noqa: E402


class EvaluationUtilityTests(unittest.TestCase):
    def test_matching_index_resolves_a_unique_anchor_timestamp(self):
        self.assertEqual(matching_index(np.array([11, 22, 33]), 22), 1)
        with self.assertRaisesRegex(ValueError, "exactly once"):
            matching_index(np.array([11, 22, 22]), 22)

    def test_select_explicit_indices_preserves_requested_order(self):
        selected = select_explicit_indices([6, 2, 9], total=10)
        self.assertEqual(selected.tolist(), [6, 2, 9])
        with self.assertRaisesRegex(ValueError, "outside"):
            select_explicit_indices([10], total=10)

    def test_latency_measurement_scope_uses_requested_horizon(self):
        self.assertEqual(
            latency_measurement_scope(10),
            "10 autoregressive model forwards per window")

    def test_select_window_indices_is_deterministic_and_spans_dataset(self):
        selected = select_window_indices(total=11, maximum=5, minimum=3)
        self.assertEqual(selected.tolist(), [0, 2, 5, 8, 10])

    def test_select_window_indices_rejects_too_few_windows(self):
        with self.assertRaisesRegex(ValueError, "at least 100"):
            select_window_indices(total=99, maximum=200, minimum=100)

    def test_compute_metrics_reports_micro_and_macro(self):
        probabilities = np.array([
            [[[0.9, 0.6], [0.1, 0.2]]],
            [[[0.8, 0.1], [0.9, 0.2]]],
        ])
        target = np.array([
            [[[1, 0], [0, 0]]],
            [[[1, 0], [1, 0]]],
        ])

        result, per_window = compute_metrics(probabilities, target, threshold=0.5)

        self.assertEqual(result["micro"]["tp"], 3)
        self.assertEqual(result["micro"]["fp"], 1)
        self.assertEqual(result["micro"]["fn"], 0)
        self.assertAlmostEqual(result["micro"]["occupied_iou"], 0.75)
        self.assertAlmostEqual(result["micro"]["f1"], 6.0 / 7.0)
        self.assertAlmostEqual(result["macro"]["occupied_iou"], 0.75)
        self.assertEqual(len(per_window), 2)

    def test_empty_denominators_are_excluded_not_scored_as_one(self):
        result, per_window = compute_metrics(
            np.zeros((1, 1, 2, 2)), np.zeros((1, 1, 2, 2)), threshold=0.5)
        self.assertIsNone(result["micro"]["occupied_iou"])
        self.assertIsNone(result["macro"]["occupied_iou"])
        self.assertIsNone(per_window[0]["occupied_iou"])
        self.assertEqual(result["macro_valid_counts"]["occupied_iou"], 0)

    def test_metrics_respect_roi_mask(self):
        probabilities = np.array([[[[0.9, 0.9], [0.0, 0.0]]]])
        target = np.array([[[[1, 0], [0, 0]]]])
        mask = np.array([[[[1, 0], [0, 0]]]])
        result, _ = compute_metrics(probabilities, target, threshold=0.5, mask=mask)
        self.assertEqual(result["micro"]["fp"], 0)
        self.assertEqual(result["micro"]["occupied_iou"], 1.0)

    def test_classification_requires_all_four_primary_improvements(self):
        copy_last = {
            "overall": {"micro": {"occupied_iou": 0.3, "f1": 0.4}},
            "dynamic_roi": {"micro": {"occupied_iou": 0.2, "f1": 0.3}},
        }
        scope = {
            "overall": {"micro": {"occupied_iou": 0.4, "f1": 0.5}},
            "dynamic_roi": {"micro": {"occupied_iou": 0.3, "f1": 0.4}},
        }
        self.assertEqual(classify_zero_shot(scope, copy_last), "supportive")
        scope["dynamic_roi"]["micro"]["f1"] = 0.25
        self.assertEqual(classify_zero_shot(scope, copy_last), "mixed")
        for region in ("overall", "dynamic_roi"):
            for metric in ("occupied_iou", "f1"):
                scope[region]["micro"][metric] = copy_last[region]["micro"][metric]
        self.assertEqual(classify_zero_shot(scope, copy_last), "unsupported")

    def test_representative_rank_is_deterministic_middle_evaluation(self):
        self.assertEqual(representative_rank(200), 100)
        with self.assertRaisesRegex(ValueError, "at least one"):
            representative_rank(0)

    def test_dynamic_obstacle_rank_selects_most_observed_roi_occupancy(self):
        targets = np.zeros((3, 1, 2, 2), dtype=np.uint8)
        roi = np.ones_like(targets)
        targets[0, 0, 0, 0] = 1
        targets[1, 0, 0, :] = 1
        targets[2, 0, :, :] = 1
        self.assertEqual(dynamic_obstacle_rank(targets, roi), 2)

    def test_stratified_dynamic_ranks_cover_each_time_segment(self):
        targets = np.zeros((8, 1, 2, 2), dtype=np.uint8)
        roi = np.ones_like(targets)
        for index, count in enumerate((1, 4, 2, 3, 4, 1, 0, 2)):
            targets[index, 0].flat[:count] = 1
        self.assertEqual(
            stratified_dynamic_obstacle_ranks(targets, roi, count=4).tolist(),
            [1, 3, 4, 7])


if __name__ == "__main__":
    unittest.main()
