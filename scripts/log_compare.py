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


import math
from dataclasses import dataclass


@dataclass
class DivergenceScore:
    """How far a simulated signal sits from the real one."""

    nrmse: float          # RMSE normalized by the real signal's range
    mean_shift: float     # sim mean minus real mean
    p95_shift: float      # sim p95 minus real p95
    peak_shift: float     # sim peak minus real peak (absolute values)
    correlation: float    # Pearson correlation of shape, 1.0 == identical shape
    samples: int          # how many grid points had values in both


def _percentile(values, pct):
    if not values:
        return 0.0
    ordered = sorted(values)
    idx = min(len(ordered) - 1, int(round((len(ordered) - 1) * pct)))
    return ordered[idx]


def _pearson(xs, ys):
    n = len(xs)
    if n < 2:
        return 0.0
    mx, my = sum(xs) / n, sum(ys) / n
    dx = [x - mx for x in xs]
    dy = [y - my for y in ys]
    denom = math.sqrt(sum(v * v for v in dx) * sum(v * v for v in dy))
    if denom == 0.0:
        # At least one series is constant. Identical constants correlate
        # perfectly; a constant against a varying signal has no shape match.
        return 1.0 if all(v == 0.0 for v in dx) and all(v == 0.0 for v in dy) else 0.0
    return sum(a * b for a, b in zip(dx, dy)) / denom


def score_pair(sim, real):
    """
    Score a resampled sim series against a resampled real series.

    Both lists must already share a grid. Grid points where either side is
    None are skipped. Returns None if nothing overlaps.
    """
    pairs = [(s, r) for s, r in zip(sim, real) if s is not None and r is not None]
    if not pairs:
        return None
    sim_vals = [s for s, _ in pairs]
    real_vals = [r for _, r in pairs]

    n = len(pairs)
    mse = sum((s - r) ** 2 for s, r in pairs) / n
    rmse = math.sqrt(mse)

    real_range = max(real_vals) - min(real_vals)
    # A flat real signal has no range to normalize against, so report raw error.
    nrmse = rmse / real_range if real_range > 0 else rmse

    return DivergenceScore(
        nrmse=nrmse,
        mean_shift=(sum(sim_vals) / n) - (sum(real_vals) / n),
        p95_shift=_percentile(sim_vals, 0.95) - _percentile(real_vals, 0.95),
        peak_shift=max(abs(v) for v in sim_vals) - max(abs(v) for v in real_vals),
        correlation=_pearson(sim_vals, real_vals),
        samples=n,
    )
