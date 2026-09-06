"""Headless diagnostic plots for SCOPE preprocessing."""

from pathlib import Path

import numpy as np


def save_preprocess_diagnostics(dataset_path, output_dir, index=0):
    import matplotlib.pyplot as plt

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    with np.load(str(dataset_path), allow_pickle=False) as data:
        sequence = data["input_ogm"][index, :, 0]
        ground_truth = data["ground_truth_ogm"][index, 0]
    figure, axes = plt.subplots(1, 3, figsize=(12, 3.6), constrained_layout=True)
    axes[0].imshow(sequence[0].T, origin="lower", extent=[0, 6.4, -3.2, 3.2],
                   cmap="gray_r", vmin=0, vmax=1, aspect="equal")
    axes[0].set_title("Past t-0.9 s in future frame")
    axes[1].imshow(sequence[-1].T, origin="lower", extent=[0, 6.4, -3.2, 3.2],
                   cmap="gray_r", vmin=0, vmax=1, aspect="equal")
    axes[1].set_title("Current t in future frame")
    axes[2].imshow(ground_truth.T, origin="lower", extent=[0, 6.4, -3.2, 3.2],
                   cmap="gray_r", vmin=0, vmax=1, aspect="equal")
    axes[2].set_title("Future LaserScan ground truth")
    for axis in axes:
        axis.set_xlabel("forward x [m]")
        axis.set_ylabel("left y [m]")
    output = output_dir / "ego_compensation_sample.png"
    figure.savefig(str(output), dpi=180)
    plt.close(figure)
    return output
