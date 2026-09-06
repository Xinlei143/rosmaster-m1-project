#!/usr/bin/env python
"""Render a deterministic five-panel SCOPE prediction diagnostic."""

from __future__ import print_function

import argparse
from pathlib import Path

import numpy as np


def representative_rank(count):
    """Return the middle evaluated rank without metric-based cherry-picking."""
    count = int(count)
    if count < 1:
        raise ValueError("at least one evaluated prediction is required")
    return count // 2


def dynamic_obstacle_rank(targets, dynamic_roi):
    """Select the evaluated window with most observed future ROI occupancy."""
    targets = np.asarray(targets, dtype=np.uint8)
    dynamic_roi = np.asarray(dynamic_roi, dtype=np.uint8)
    if targets.shape != dynamic_roi.shape or targets.ndim != 4:
        raise ValueError("targets and dynamic_roi must share [N,1,H,W] shape")
    if len(targets) < 1:
        raise ValueError("at least one evaluated prediction is required")
    occupancy_counts = np.count_nonzero(
        (targets != 0) & (dynamic_roi != 0), axis=(1, 2, 3))
    return int(np.argmax(occupancy_counts))


def stratified_dynamic_obstacle_ranks(targets, dynamic_roi, count):
    """Select one dynamic-obstacle-rich sample from each temporal stratum."""
    targets = np.asarray(targets, dtype=np.uint8)
    dynamic_roi = np.asarray(dynamic_roi, dtype=np.uint8)
    count = int(count)
    if targets.shape != dynamic_roi.shape or targets.ndim != 4:
        raise ValueError("targets and dynamic_roi must share [N,1,H,W] shape")
    if count < 1 or count > len(targets):
        raise ValueError("count must be 1 through the number of predictions")
    occupancy_counts = np.count_nonzero(
        (targets != 0) & (dynamic_roi != 0), axis=(1, 2, 3))
    ranks = []
    for stratum in np.array_split(np.arange(len(targets)), count):
        ranks.append(int(stratum[np.argmax(occupancy_counts[stratum])]))
    return np.asarray(ranks, dtype=np.int64)


def render(dataset_path, predictions_path, output_path, rank=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    with np.load(dataset_path, allow_pickle=False) as dataset:
        inputs = dataset["input_ogm"]
        targets = dataset["ground_truth_ogm"]
        dynamic_roi = dataset["dynamic_roi"]
        x_limits = dataset["x_limit"].astype(float)
        y_limits = dataset["y_limit"].astype(float)
    with np.load(predictions_path, allow_pickle=False) as predictions:
        indices = predictions["dataset_indices"].astype(int)
        means = predictions["prediction_mean"]
        deviations = predictions["prediction_std"]

    selected_targets = targets[indices]
    selected_roi = dynamic_roi[indices]
    selected_rank = (dynamic_obstacle_rank(selected_targets, selected_roi)
                     if rank is None else int(rank))
    if selected_rank < 0 or selected_rank >= len(indices):
        raise ValueError("rank is outside the evaluated prediction range")
    dataset_index = int(indices[selected_rank])
    extent = [x_limits[0], x_limits[1], y_limits[0], y_limits[1]]
    panels = [
        (inputs[dataset_index, 0, 0], "Oldest input (t-0.9 s)", "gray_r", 0.0, 1.0),
        (inputs[dataset_index, -1, 0], "Current input (t)", "gray_r", 0.0, 1.0),
        (means[selected_rank, 0], "SCOPE mean (t+0.5 s)", "gray_r", 0.0, 1.0),
        (targets[dataset_index, 0], "Observed target (t+0.5 s)", "gray_r", 0.0, 1.0),
        (deviations[selected_rank, 0], "Sample std. deviation", "magma", 0.0, None),
    ]

    figure, axes = plt.subplots(1, 5, figsize=(15.2, 3.4), constrained_layout=True)
    images = []
    for axis, (values, title, colourmap, minimum, maximum) in zip(axes, panels):
        image = axis.imshow(
            values.T, origin="lower", extent=extent, cmap=colourmap,
            vmin=minimum, vmax=maximum, interpolation="nearest", aspect="equal")
        images.append(image)
        axis.set_title(title, fontsize=9)
        axis.set_xlabel("forward x [m]")
        axis.set_ylabel("left y [m]")
    figure.colorbar(images[3], ax=axes[:4], shrink=0.74, label="occupancy / probability")
    figure.colorbar(images[4], ax=axes[4], shrink=0.74, label="probability std.")
    figure.suptitle(
        "M1 SCOPE zero-shot diagnostic: dynamic-ROI rank %d, "
        "dataset window %d\n"
        "Selected by maximum observed future occupancy in the dynamic ROI"
        % (selected_rank, dataset_index), fontsize=10)
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(str(output), dpi=180, bbox_inches="tight")
    plt.close(figure)
    return {"evaluation_rank": selected_rank, "dataset_index": dataset_index}


def _parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--predictions", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--rank", type=int)
    return parser.parse_args(argv)


def main(argv=None):
    arguments = _parse_args(argv)
    selection = render(
        arguments.dataset, arguments.predictions, arguments.output, arguments.rank)
    print("rendered evaluation rank {evaluation_rank}, dataset window {dataset_index}".format(
        **selection))


if __name__ == "__main__":
    main()
