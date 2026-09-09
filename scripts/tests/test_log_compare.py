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
