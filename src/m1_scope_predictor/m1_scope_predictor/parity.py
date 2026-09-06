"""Compare the modern runtime against frozen strict-environment predictions."""

import argparse
import json
from pathlib import Path

import numpy as np

from .evaluator import endpoint_metrics
from .runtime_backend import ScopeRuntimeBackend


def run(args):
    with np.load(args.dataset, allow_pickle=False) as dataset:
        inputs = dataset["input_ogm"]
        targets = dataset["ground_truth_ogm"]
        roi = dataset["dynamic_roi"].astype(bool)
    with np.load(args.reference, allow_pickle=False) as reference:
        selected = reference["dataset_indices"][:args.windows]
        strict_means = reference["prediction_mean"][:args.windows]
    backend = ScopeRuntimeBackend(args.checkpoint, args.device, args.seed)
    modern = []
    latencies = []
    for progress, index in enumerate(selected, start=1):
        mean, _, latency, _ = backend.infer(
            inputs[int(index)].astype(np.float32), args.horizon, args.samples)
        modern.append(mean)
        latencies.append(latency)
        print("parity %d/%d" % (progress, len(selected)), flush=True)
    modern = np.stack(modern)
    probability_mae = float(np.mean(np.abs(modern - strict_means)))
    comparisons = {}
    passed = probability_mae <= args.probability_mae_limit
    for region, mask in (("overall", None), ("dynamic_roi", roi[selected])):
        region_target = targets[selected]
        if mask is not None:
            modern_values = modern[mask]
            strict_values = strict_means[mask]
            target_values = region_target[mask]
        else:
            modern_values = modern
            strict_values = strict_means
            target_values = region_target
        modern_metrics = endpoint_metrics(modern_values, target_values, args.threshold)
        strict_metrics = endpoint_metrics(strict_values, target_values, args.threshold)
        differences = {}
        for key in ("occupied_iou", "f1"):
            difference = abs(modern_metrics[key] - strict_metrics[key])
            differences[key] = difference
            passed = passed and difference <= args.metric_limit
        comparisons[region] = {
            "modern": modern_metrics, "strict": strict_metrics,
            "absolute_difference": differences,
        }
    summary = {
        "passed": bool(passed), "windows": int(len(selected)),
        "probability_mean_mae": probability_mae,
        "latency_seconds": {
            "mean": float(np.mean(latencies)),
            "p95": float(np.percentile(latencies, 95.0)),
        },
        "comparisons": comparisons,
    }
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    Path(args.output).write_text(
        json.dumps(summary, indent=2, sort_keys=True, allow_nan=False) + "\n")
    if not passed:
        raise RuntimeError("modern SCOPE runtime failed the parity gate")
    return summary


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--reference", required=True)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--device", default="cuda", choices=("cpu", "cuda"))
    parser.add_argument("--windows", type=int, default=100)
    parser.add_argument("--horizon", type=int, default=5)
    parser.add_argument("--samples", type=int, default=32)
    parser.add_argument("--seed", type=int, default=1337)
    parser.add_argument("--threshold", type=float, default=0.5)
    parser.add_argument("--probability-mae-limit", type=float, default=0.01)
    parser.add_argument("--metric-limit", type=float, default=0.01)
    return parser.parse_args(argv)


def main(argv=None):
    print(json.dumps(run(parse_args(argv)), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
