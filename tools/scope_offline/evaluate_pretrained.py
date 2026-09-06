#!/usr/bin/env python
"""Evaluate official pretrained SCOPE on ROS-independent M1 OGM datasets."""

from __future__ import print_function

import argparse
import csv
import hashlib
import importlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np


def select_window_indices(total, maximum, minimum=100):
    total = int(total)
    maximum = int(maximum)
    minimum = int(minimum)
    if total < minimum:
        raise ValueError("evaluation requires at least %d valid windows" % minimum)
    count = min(total, maximum)
    return np.rint(np.linspace(0, total - 1, count)).astype(np.int64)


def _safe_divide(numerator, denominator):
    return float(numerator) / float(denominator) if denominator else None


def _confusion_metrics(probability, target, threshold, mask):
    selected = mask.astype(bool)
    predicted = probability >= float(threshold)
    occupied = target.astype(bool)
    tp = int(np.count_nonzero(predicted & occupied & selected))
    fp = int(np.count_nonzero(predicted & ~occupied & selected))
    fn = int(np.count_nonzero(~predicted & occupied & selected))
    pixel_count = int(np.count_nonzero(selected))
    mae = (float(np.mean(np.abs(probability[selected] - target[selected])))
           if pixel_count else None)
    precision = _safe_divide(tp, tp + fp)
    recall = _safe_divide(tp, tp + fn)
    return {
        "pixel_count": pixel_count,
        "tp": tp,
        "fp": fp,
        "fn": fn,
        "mae": mae,
        "occupied_iou": _safe_divide(tp, tp + fp + fn),
        "precision": precision,
        "recall": recall,
        "f1": _safe_divide(2 * tp, 2 * tp + fp + fn),
    }


def compute_metrics(probability, target, threshold=0.5, mask=None):
    probability = np.asarray(probability, dtype=np.float64)
    target = np.asarray(target, dtype=np.uint8)
    if probability.shape != target.shape or probability.ndim != 4:
        raise ValueError("probability and target must have matching [N,1,H,W] shape")
    if not np.isfinite(probability).all():
        raise ValueError("prediction contains non-finite values")
    if mask is None:
        mask = np.ones(target.shape, dtype=np.bool_)
    else:
        mask = np.broadcast_to(np.asarray(mask, dtype=np.bool_), target.shape)

    per_window = [
        _confusion_metrics(probability[index], target[index], threshold, mask[index])
        for index in range(len(probability))
    ]
    micro = _confusion_metrics(probability, target, threshold, mask)
    macro = {}
    valid_counts = {}
    for name in ("mae", "occupied_iou", "precision", "recall", "f1"):
        values = [item[name] for item in per_window if item[name] is not None]
        macro[name] = float(np.mean(values)) if values else None
        valid_counts[name] = len(values)
    return {
        "threshold": float(threshold),
        "micro": micro,
        "macro": macro,
        "macro_valid_counts": valid_counts,
    }, per_window


def classify_zero_shot(scope_metrics, copy_last_metrics):
    pairs = []
    for region in ("overall", "dynamic_roi"):
        for metric in ("occupied_iou", "f1"):
            scope_value = scope_metrics[region]["micro"][metric]
            baseline_value = copy_last_metrics[region]["micro"][metric]
            if scope_value is None or baseline_value is None:
                return "mixed"
            pairs.append(scope_value > baseline_value)
    if all(pairs):
        return "supportive"
    if not any(pairs):
        return "unsupported"
    return "mixed"


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        while True:
            block = stream.read(1024 * 1024)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def _git_output(scope_root, *arguments):
    command = ["git", "-C", str(scope_root)] + list(arguments)
    return subprocess.check_output(command, universal_newlines=True).strip()


def source_identity(scope_root, checkpoint):
    scope_root = Path(scope_root).resolve()
    scripts = scope_root / "scripts"
    required = [scripts / "model.py", scripts / "convlstm.py", Path(checkpoint)]
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise ValueError("official SCOPE files are missing: %s" % ", ".join(missing))
    status = _git_output(scope_root, "status", "--porcelain")
    if status:
        raise ValueError("official SCOPE reference is not Git clean: %s" % status)
    return {
        "scope_root": str(scope_root),
        "git_commit": _git_output(scope_root, "rev-parse", "HEAD"),
        "git_status": "clean",
        "model_sha256": _sha256(scripts / "model.py"),
        "convlstm_sha256": _sha256(scripts / "convlstm.py"),
        "checkpoint": str(Path(checkpoint).resolve()),
        "checkpoint_sha256": _sha256(checkpoint),
    }


def _load_official_model(scope_root, checkpoint, device):
    scripts = str(Path(scope_root).resolve() / "scripts")
    if scripts not in sys.path:
        sys.path.insert(0, scripts)
    official = importlib.import_module("model")
    model = official.scope(input_channels=1, latent_dim=512, output_channels=1)
    model.to(device)
    model.eval()
    import torch
    state = torch.load(str(checkpoint), map_location=device)
    model.load_state_dict(state["model"])
    return model


def _synchronize(torch, device):
    if device.type == "cuda":
        torch.cuda.synchronize(device)


def _infer_window(torch, model, input_ogm, horizon_steps, num_samples, device):
    inputs = torch.from_numpy(input_ogm.astype(np.float32)).unsqueeze(0).to(device)
    inputs = inputs.repeat(int(num_samples), 1, 1, 1, 1)
    _synchronize(torch, device)
    start = time.perf_counter()
    with torch.no_grad():
        prediction = None
        for _ in range(int(horizon_steps)):
            prediction, _ = model(inputs)
            prediction = prediction.reshape(-1, 1, 64, 64)
            inputs = torch.cat([inputs[:, 1:], prediction.unsqueeze(1)], dim=1)
    _synchronize(torch, device)
    latency = time.perf_counter() - start
    samples = prediction.detach().cpu().numpy()
    return samples.mean(axis=0), samples.std(axis=0), latency


def _portable(value):
    if isinstance(value, dict):
        return {str(key): _portable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_portable(item) for item in value]
    if isinstance(value, np.ndarray):
        return _portable(value.tolist())
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating, float)):
        value = float(value)
        return value if math.isfinite(value) else None
    return value


def _write_json(path, payload):
    Path(path).write_text(
        json.dumps(_portable(payload), indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8")


def _write_per_window(path, selected, scope_rows, copy_rows, scope_roi, copy_roi):
    fields = ["dataset_index"]
    for prefix in ("scope", "copy_last", "scope_roi", "copy_last_roi"):
        for metric in ("mae", "occupied_iou", "precision", "recall", "f1"):
            fields.append(prefix + "_" + metric)
    with Path(path).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for index, dataset_index in enumerate(selected):
            row = {"dataset_index": int(dataset_index)}
            for prefix, values in (("scope", scope_rows), ("copy_last", copy_rows),
                                   ("scope_roi", scope_roi),
                                   ("copy_last_roi", copy_roi)):
                for metric in ("mae", "occupied_iou", "precision", "recall", "f1"):
                    row[prefix + "_" + metric] = values[index][metric]
            writer.writerow(row)


def evaluate(args):
    os.environ["PYTHONDONTWRITEBYTECODE"] = "1"
    identity_before = source_identity(args.scope_root, args.checkpoint)
    with np.load(args.dataset, allow_pickle=False) as dataset:
        if str(dataset["schema_version"].item()) != "m1_scope_ogm_v1":
            raise ValueError("unsupported preprocessed dataset schema")
        dataset_horizon = int(dataset["horizon_steps"].item())
        if dataset_horizon != int(args.horizon):
            raise ValueError("dataset horizon does not match --horizon")
        selected = select_window_indices(
            len(dataset["input_ogm"]), args.max_windows, args.min_windows)
        inputs = dataset["input_ogm"][selected]
        ground_truth = dataset["ground_truth_ogm"][selected]
        copy_last = dataset["copy_last_ogm"][selected].astype(np.float32)
        roi = dataset["dynamic_roi"][selected]
        anchor_stamps = dataset["anchor_scan_stamp_ns"][selected]
        target_stamps = dataset["target_scan_stamp_ns"][selected]

    import torch
    if args.device == "auto":
        device_name = "cuda" if torch.cuda.is_available() else "cpu"
    else:
        device_name = args.device
    if device_name == "cuda" and not torch.cuda.is_available():
        raise ValueError("CUDA was requested but torch.cuda.is_available() is false")
    device = torch.device(device_name)
    torch.manual_seed(args.seed)
    if device.type == "cuda":
        torch.cuda.manual_seed_all(args.seed)
    np.random.seed(args.seed)
    model = _load_official_model(args.scope_root, args.checkpoint, device)

    if args.warmup_windows:
        _infer_window(
            torch, model, inputs[0], args.horizon, args.num_samples, device)
        torch.manual_seed(args.seed)
        if device.type == "cuda":
            torch.cuda.manual_seed_all(args.seed)

    means = []
    standard_deviations = []
    latencies = []
    for progress, input_ogm in enumerate(inputs, start=1):
        mean, standard_deviation, latency = _infer_window(
            torch, model, input_ogm, args.horizon, args.num_samples, device)
        means.append(mean)
        standard_deviations.append(standard_deviation)
        latencies.append(latency)
        if progress == 1 or progress % 10 == 0 or progress == len(inputs):
            print("evaluated %d/%d windows" % (progress, len(inputs)), flush=True)
    means = np.stack(means).astype(np.float32)
    standard_deviations = np.stack(standard_deviations).astype(np.float32)
    latencies = np.asarray(latencies, dtype=np.float64)

    scope_overall, scope_rows = compute_metrics(
        means, ground_truth, args.threshold)
    copy_overall, copy_rows = compute_metrics(
        copy_last, ground_truth, args.threshold)
    scope_roi_summary, scope_roi_rows = compute_metrics(
        means, ground_truth, args.threshold, mask=roi)
    copy_roi_summary, copy_roi_rows = compute_metrics(
        copy_last, ground_truth, args.threshold, mask=roi)
    scope_metrics = {"overall": scope_overall, "dynamic_roi": scope_roi_summary}
    copy_metrics = {"overall": copy_overall, "dynamic_roi": copy_roi_summary}
    classification = classify_zero_shot(scope_metrics, copy_metrics)

    identity_after = source_identity(args.scope_root, args.checkpoint)
    if identity_before != identity_after:
        raise RuntimeError("official SCOPE identity changed during evaluation")
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        str(output_dir / ("predictions_h%02d.npz" % int(args.horizon))),
        schema_version=np.asarray("m1_scope_prediction_v1"),
        dataset_indices=selected,
        anchor_scan_stamp_ns=anchor_stamps,
        target_scan_stamp_ns=target_stamps,
        prediction_mean=means,
        prediction_std=standard_deviations,
        latency_seconds=latencies,
    )
    _write_per_window(
        output_dir / "per_window_metrics.csv", selected, scope_rows, copy_rows,
        scope_roi_rows, copy_roi_rows)
    summary = {
        "schema_version": "m1_scope_evaluation_summary_v1",
        "dataset": str(Path(args.dataset).resolve()),
        "horizon_steps": int(args.horizon),
        "horizon_seconds": float(args.horizon) / 10.0,
        "num_samples": int(args.num_samples),
        "evaluated_windows": len(selected),
        "selected_dataset_indices": selected.tolist(),
        "threshold": float(args.threshold),
        "seed": int(args.seed),
        "device": str(device),
        "python_version": sys.version.split()[0],
        "torch_version": torch.__version__,
        "numpy_version": np.__version__,
        "source_identity": identity_after,
        "latency_seconds": {
            "mean": float(latencies.mean()),
            "median": float(np.median(latencies)),
            "p95": float(np.percentile(latencies, 95.0)),
            "measurement_scope": "five autoregressive model forwards per window",
        },
        "scope": scope_metrics,
        "copy_last": copy_metrics,
        "zero_shot_evidence": classification,
        "uncertainty": {
            "kind": "pixel-wise sample standard deviation",
            "mean": float(standard_deviations.mean()),
            "max": float(standard_deviations.max()),
        },
    }
    _write_json(output_dir / "metrics_summary.json", summary)
    return summary


def _parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--scope-root", required=True)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--horizon", type=int, default=5)
    parser.add_argument("--num-samples", type=int, default=32)
    parser.add_argument("--max-windows", type=int, default=200)
    parser.add_argument("--min-windows", type=int, default=100)
    parser.add_argument("--threshold", type=float, default=0.5)
    parser.add_argument("--seed", type=int, default=1337)
    parser.add_argument("--device", choices=("auto", "cpu", "cuda"), default="auto")
    parser.add_argument("--warmup-windows", type=int, default=1)
    parser.add_argument("--output-dir", required=True)
    return parser.parse_args(argv)


def main(argv=None):
    summary = evaluate(_parse_args(argv))
    print(json.dumps(_portable(summary), indent=2, sort_keys=True, allow_nan=False))


if __name__ == "__main__":
    main()
