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


from log_compare import extract_events, pair_events, worst_windows


def test_worst_windows_finds_the_divergent_stretch():
    grid = [i * 0.5 for i in range(10)]      # 0.0 .. 4.5
    real = [0.0] * 10
    sim = [0.0] * 10
    sim[4] = 10.0                             # spike at t=2.0
    sim[5] = 10.0                             # spike at t=2.5
    windows = worst_windows(sim, real, grid, window_s=1.0, top=1)
    assert len(windows) == 1
    start, end, err = windows[0]
    assert start <= 2.0 <= end
    assert err > 0


def test_worst_windows_returns_at_most_top_n():
    grid = [i * 0.5 for i in range(20)]
    real = [0.0] * 20
    sim = [float(i) for i in range(20)]
    assert len(worst_windows(sim, real, grid, window_s=1.0, top=3)) == 3


def test_worst_windows_are_non_overlapping():
    grid = [i * 0.5 for i in range(20)]
    real = [0.0] * 20
    sim = [float(i) for i in range(20)]
    windows = worst_windows(sim, real, grid, window_s=1.0, top=3)
    starts = [w[0] for w in windows]
    assert len(set(starts)) == len(starts)


def test_extract_events_returns_transitions_only():
    series = [(1.0, "STOW"), (1.5, "STOW"), (2.0, "INTAKE"), (3.0, "STOW")]
    assert extract_events(series) == [(1.0, "STOW"), (2.0, "INTAKE"), (3.0, "STOW")]


def test_extract_events_on_empty_series():
    assert extract_events([]) == []


def test_pair_events_matches_same_sequence_and_reports_delta():
    real = [(1.0, "STOW"), (2.0, "INTAKE")]
    sim = [(1.0, "STOW"), (2.4, "INTAKE")]
    pairs = pair_events(sim, real)
    assert len(pairs) == 2
    assert pairs[1][0] == "INTAKE"
    assert pairs[1][3] == pytest.approx(0.4)


def test_pair_events_stops_at_first_sequence_mismatch():
    real = [(1.0, "STOW"), (2.0, "INTAKE"), (3.0, "EJECT")]
    sim = [(1.0, "STOW"), (2.0, "REVERSE")]
    pairs = pair_events(sim, real)
    # Only the common prefix is comparable; after divergence the runs are
    # doing different things and further pairing would be meaningless.
    assert len(pairs) == 1
    assert pairs[0][0] == "STOW"
