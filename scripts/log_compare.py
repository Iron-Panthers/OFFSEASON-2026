"""
Compare a simulation .wpilog against a real match .wpilog.

Aligns both logs on first-enable, resamples every shared numeric key onto a
common grid, and ranks keys by how much they diverge.
"""

from bisect import bisect_right

GRID_DT = 0.02  # matches Constants.PERIODIC_LOOP_SEC


def resample(series, grid):
    """
    Zero-order-hold resample [(ts, value), ...] onto the timestamps in `grid`.

    Returns a list the same length as `grid`. Entries before the first sample
    are None, since there is no value to hold.
    """
    if not series:
        return [None] * len(grid)
    timestamps = [ts for ts, _ in series]
    out = []
    for t in grid:
        idx = bisect_right(timestamps, t) - 1
        out.append(series[idx][1] if idx >= 0 else None)
    return out


def find_first_enable(data):
    """Timestamp of the first DriverStation/Enabled -> True, or None."""
    for ts, value in data.get("DriverStation/Enabled", []):
        if value:
            return ts
    return None
