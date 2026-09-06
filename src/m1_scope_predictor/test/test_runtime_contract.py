import hashlib
import sys
from pathlib import Path

import numpy as np
import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from m1_scope_predictor.runtime_backend import (  # noqa: E402
    aggregate_samples,
    validate_model_input,
    verify_frozen_assets,
)


def test_validate_model_input_accepts_scope_tensor_contract():
    value = np.zeros((10, 1, 64, 64), dtype=np.float32)
    assert validate_model_input(value) is value


@pytest.mark.parametrize(
    "value, message",
    [
        (np.zeros((1, 10, 1, 64, 64), dtype=np.float32), "shape"),
        (np.zeros((10, 1, 64, 64), dtype=np.float64), "float32"),
        (np.full((10, 1, 64, 64), np.nan, dtype=np.float32), "finite"),
    ],
)
def test_validate_model_input_rejects_invalid_arrays(value, message):
    with pytest.raises(ValueError, match=message):
        validate_model_input(value)


def test_aggregate_samples_returns_pixel_mean_and_standard_deviation():
    samples = np.zeros((2, 1, 64, 64), dtype=np.float32)
    samples[:, 0, 0, 0] = [0.0, 1.0]
    samples[:, 0, 0, 1] = [1.0, 1.0]
    samples[:, 0, 1, 0] = [0.5, 0.5]
    samples[:, 0, 1, 1] = [0.5, 0.0]
    mean, standard_deviation = aggregate_samples(samples)
    assert np.allclose(mean[0, :2, :2], [[0.5, 1.0], [0.5, 0.25]])
    assert np.allclose(standard_deviation[0, :2, :2], [[0.5, 0.0], [0.0, 0.25]])


def test_vendored_sources_match_frozen_upstream_hashes():
    vendor = PACKAGE_ROOT / "m1_scope_predictor" / "third_party" / "templerail_scope"
    expected = {
        "model.py": "a7b404a2909bcc8e563e3a70520bf4398957fab98f85d80eab4fdf70e24906f6",
        "convlstm.py": "eb350d8d2297a2a2f7e652500aeabaa5c92ab6b3bbd9d0a77d516f0718b114e2",
        "LICENSE": "6a24bc77980f5232c73b36f392227155415c56c4be6e32f6188be31ce019682d",
    }
    for name, digest in expected.items():
        assert hashlib.sha256((vendor / name).read_bytes()).hexdigest() == digest


def test_runtime_rejects_an_unrecognized_checkpoint(tmp_path):
    checkpoint = tmp_path / "scope.pth"
    checkpoint.write_bytes(b"not the frozen checkpoint")
    with pytest.raises(ValueError, match="checkpoint SHA-256"):
        verify_frozen_assets(checkpoint)
