import sys
import unittest
from pathlib import Path

import numpy as np


TOOL_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOL_ROOT))

from evaluate_pretrained import (  # noqa: E402
    classify_zero_shot, compute_metrics, select_window_indices)
from visualize_prediction import representative_rank  # noqa: E402


class EvaluationUtilityTests(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
