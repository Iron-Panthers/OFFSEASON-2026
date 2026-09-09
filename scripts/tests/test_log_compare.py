import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from log_compare import GRID_DT, find_first_enable, resample


def test_resample_zero_order_hold():
    # Samples at 0.0 and 1.0; grid every 0.5s from 0.0 to 1.5
    series = [(0.0, 10.0), (1.0, 20.0)]
    grid = [0.0, 0.5, 1.0, 1.5]
    assert resample(series, grid) == [10.0, 10.0, 20.0, 20.0]


def test_resample_before_first_sample_is_none():
    series = [(1.0, 5.0)]
    assert resample(series, [0.0, 1.0]) == [None, 5.0]


def test_resample_empty_series():
    assert resample([], [0.0, 1.0]) == [None, None]


def test_find_first_enable():
    data = {"DriverStation/Enabled": [(1.0, False), (61.3, True), (82.0, False)]}
    assert find_first_enable(data) == 61.3


def test_find_first_enable_missing_key_returns_none():
    assert find_first_enable({}) is None


def test_grid_dt_matches_robot_loop():
    assert GRID_DT == 0.02


from log_compare import DivergenceScore, score_pair


def test_identical_series_scores_zero_divergence():
    a = [1.0, 2.0, 3.0, 4.0]
    s = score_pair(a, list(a))
    assert s.nrmse == 0.0
    assert s.mean_shift == 0.0
    assert s.correlation == 1.0


def test_constant_offset_is_reported_as_mean_shift():
    real = [1.0, 2.0, 3.0, 4.0]
    sim = [2.0, 3.0, 4.0, 5.0]
    s = score_pair(sim, real)
    assert abs(s.mean_shift - 1.0) < 1e-9
    assert s.correlation > 0.999  # shape is identical, only offset differs


def test_nrmse_normalizes_by_real_range():
    # real spans 0..10, sim is off by a constant 1.0 -> nrmse == 0.1
    real = [0.0, 5.0, 10.0]
    sim = [1.0, 6.0, 11.0]
    s = score_pair(sim, real)
    assert abs(s.nrmse - 0.1) < 1e-9


def test_none_entries_are_skipped_pairwise():
    s = score_pair([None, 2.0, 3.0], [1.0, 2.0, 3.0])
    assert s.samples == 2
    assert s.nrmse == 0.0


def test_no_overlapping_samples_returns_none_score():
    assert score_pair([None, None], [1.0, 2.0]) is None


def test_flat_real_series_uses_absolute_error():
    # real has zero range; nrmse would divide by zero, so fall back to abs error
    s = score_pair([2.0, 2.0], [1.0, 1.0])
    assert s.nrmse == 1.0
