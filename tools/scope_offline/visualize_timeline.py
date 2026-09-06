#!/usr/bin/env python
"""Render matched 0.5 s and 1.0 s SCOPE predictions for one M1 scene."""

from __future__ import print_function

import argparse
from pathlib import Path

import numpy as np


def matching_index(stamps, target_stamp):
    """Return the unique row containing an anchor timestamp."""
    matches = np.flatnonzero(np.asarray(stamps, dtype=np.int64) == int(target_stamp))
    if len(matches) != 1:
        raise ValueError("anchor timestamp must occur exactly once")
    return int(matches[0])


def render_timeline(dataset_h05, predictions_h05, dataset_h10, predictions_h10,
                    output_path, h05_dataset_index):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    with np.load(dataset_h05, allow_pickle=False) as data_h05:
        h05_dataset_index = int(h05_dataset_index)
        if h05_dataset_index < 0 or h05_dataset_index >= len(data_h05["input_ogm"]):
            raise ValueError("h05 dataset index is outside the dataset")
        anchor_stamp = data_h05["anchor_scan_stamp_ns"][h05_dataset_index]
        inputs = data_h05["input_ogm"][h05_dataset_index]
        target_h05 = data_h05["ground_truth_ogm"][h05_dataset_index, 0]
        x_limits = data_h05["x_limit"].astype(float)
        y_limits = data_h05["y_limit"].astype(float)
    with np.load(predictions_h05, allow_pickle=False) as prediction_h05:
        h05_prediction_index = matching_index(
            prediction_h05["anchor_scan_stamp_ns"], anchor_stamp)
        mean_h05 = prediction_h05["prediction_mean"][h05_prediction_index, 0]
    with np.load(dataset_h10, allow_pickle=False) as data_h10:
        h10_dataset_index = matching_index(data_h10["anchor_scan_stamp_ns"], anchor_stamp)
        target_h10 = data_h10["ground_truth_ogm"][h10_dataset_index, 0]
    with np.load(predictions_h10, allow_pickle=False) as prediction_h10:
        h10_prediction_index = matching_index(
            prediction_h10["anchor_scan_stamp_ns"], anchor_stamp)
        mean_h10 = prediction_h10["prediction_mean"][h10_prediction_index, 0]

    extent = [x_limits[0], x_limits[1], y_limits[0], y_limits[1]]
    panels = [
        (inputs[0, 0], "History (t-0.9 s)"),
        (inputs[3, 0], "History (t-0.6 s)"),
        (inputs[6, 0], "History (t-0.3 s)"),
        (inputs[9, 0], "Current input (t)"),
        (mean_h05, "SCOPE prediction (t+0.5 s)"),
        (target_h05, "Observed target (t+0.5 s)"),
        (mean_h10, "SCOPE prediction (t+1.0 s)"),
        (target_h10, "Observed target (t+1.0 s)"),
    ]
    figure, axes = plt.subplots(2, 4, figsize=(14.6, 6.7), constrained_layout=True)
    images = []
    for axis, (values, title) in zip(axes.ravel(), panels):
        image = axis.imshow(
            values.T, origin="lower", extent=extent, cmap="gray_r", vmin=0.0, vmax=1.0,
            interpolation="nearest", aspect="equal")
        images.append(image)
        axis.set_title(title, fontsize=9)
        axis.set_xlabel("forward x [m]")
        axis.set_ylabel("left y [m]")
    figure.colorbar(images[-1], ax=axes.ravel(), shrink=0.78, label="occupancy / probability")
    figure.suptitle(
        "M1 SCOPE matched multi-horizon diagnostic: dataset window %d\n"
        "Black = occupied endpoint in binary panels; SCOPE panels show probability"
        % h05_dataset_index, fontsize=10)
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(str(output), dpi=180, bbox_inches="tight")
    plt.close(figure)
    return {
        "h05_dataset_index": h05_dataset_index,
        "h10_dataset_index": h10_dataset_index,
        "anchor_stamp_ns": int(anchor_stamp),
    }


def _parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-h05", required=True)
    parser.add_argument("--predictions-h05", required=True)
    parser.add_argument("--dataset-h10", required=True)
    parser.add_argument("--predictions-h10", required=True)
    parser.add_argument("--h05-dataset-index", type=int, required=True)
    parser.add_argument("--output", required=True)
    return parser.parse_args(argv)


def main(argv=None):
    arguments = _parse_args(argv)
    result = render_timeline(
        arguments.dataset_h05, arguments.predictions_h05,
        arguments.dataset_h10, arguments.predictions_h10,
        arguments.output, arguments.h05_dataset_index)
    print(
        "rendered h05 window {h05_dataset_index}, h10 window {h10_dataset_index}".format(
            **result))


if __name__ == "__main__":
    main()
