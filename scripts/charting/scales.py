"""
Scales, axis ticks and series reshaping.

Pure functions and frozen dataclasses: no SVG, no file I/O, no global state.
That is the point of the split -- every rule about where a pixel goes or how a
number reads is testable without a .wpilog file.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# Step mantissas in preference order. 2.5 earns its place because flywheel
# setpoints in rot/s land on .5 boundaries far more often than on multiples of 2.
_TICK_MANTISSAS = (1.0, 2.0, 2.5, 5.0, 10.0)


def nice_step(span: float, target: int = 6) -> float:
    """Round span/target up to the nearest 1, 2, 2.5 or 5 times a power of ten."""
    if not math.isfinite(span) or span <= 0:
        return 1.0
    raw = span / max(target, 1)
    if raw <= 0:
        return 1.0
    magnitude = 10.0 ** math.floor(math.log10(raw))
    normalized = raw / magnitude
    for mantissa in _TICK_MANTISSAS:
        if normalized <= mantissa * (1 + 1e-9):
            return mantissa * magnitude
    return 10.0 * magnitude


def pad_domain(lo: float, hi: float) -> tuple[float, float]:
    """
    Widen a degenerate domain so it can be drawn.

    Constant signals are everywhere in robot logs -- a setpoint that never moves,
    a boolean false for the whole match -- and a zero-width domain divides by
    zero in every scale. Padding here means no caller special-cases it.
    """
    if not (math.isfinite(lo) and math.isfinite(hi)):
        return (0.0, 1.0)
    if hi < lo:
        lo, hi = hi, lo
    if hi > lo:
        return (lo, hi)
    if lo == 0.0:
        return (-1.0, 1.0)
    pad = abs(lo) * 0.05
    return (lo - pad, hi + pad)


def nice_axis(
    lo: float,
    hi: float,
    target: int = 6,
    include_zero: bool = False,
) -> tuple[float, float, list[float]]:
    """
    Snap a domain outward to tick boundaries.

    Returns (domain_lo, domain_hi, ticks). ``include_zero`` is for magnitude
    signals -- a velocity chart that omits zero overstates how big a dip was --
    and is left off for positions and poses, where the interesting range rarely
    contains the origin.
    """
    lo, hi = pad_domain(lo, hi)
    if include_zero:
        lo, hi = pad_domain(min(lo, 0.0), max(hi, 0.0))
    step = nice_step(hi - lo, target)
    d0 = math.floor(lo / step) * step
    d1 = math.ceil(hi / step) * step
    if d1 <= d0:
        d1 = d0 + step
    # Accumulate by index rather than repeated +=, which drifts visibly by the
    # eighth tick and renders 0.30000000000000004.
    count = int(round((d1 - d0) / step))
    ticks = [d0 + i * step for i in range(count + 1)]
    return (d0, d1, ticks)


@dataclass(frozen=True)
class Scale:
    """
    Maps a value in [d0, d1] onto pixels in [p0, p1].

    An inverted axis is not a special case: for y, pass p0 as the bottom pixel
    and p1 as the top.
    """

    d0: float
    d1: float
    p0: float
    p1: float

    def px(self, value: float) -> float:
        span = self.d1 - self.d0
        if span == 0:
            return self.p0
        return self.p0 + ((value - self.d0) / span) * (self.p1 - self.p0)

    def clamp_px(self, value: float) -> float:
        """Pixel position clamped to the plot, so an outlier cannot draw outside it."""
        p = self.px(value)
        lo, hi = min(self.p0, self.p1), max(self.p0, self.p1)
        return max(lo, min(hi, p))

    @property
    def units_per_px(self) -> float:
        span_px = abs(self.p1 - self.p0)
        return (self.d1 - self.d0) / span_px if span_px else 0.0


def format_number(value, step: float | None = None) -> str:
    """
    Format an axis tick or a table cell.

    Decimal places follow the tick step, so an axis stepping by 0.5 reads
    0.0 / 0.5 / 1.0 and one stepping by 500 reads 0 / 500 / 1,000. Without a
    step (table cells) the magnitude decides.
    """
    if value is None or not isinstance(value, (int, float)) or isinstance(value, bool):
        return "-"
    if not math.isfinite(value):
        return "-"
    if abs(value) < 1e-12:
        value = 0.0
    if step is None or not math.isfinite(step) or step <= 0:
        magnitude = abs(value)
        decimals = 0 if magnitude >= 100 else (1 if magnitude >= 10 else 2)
    else:
        exponent = math.floor(math.log10(step))
        decimals = max(0, -exponent)
        # A 2.5-style step needs one more place than its exponent implies,
        # otherwise the ticks round to 0 / 2 / 5 / 8.
        if abs(step / (10.0**exponent) - 2.5) < 1e-9:
            decimals += 1
    return f"{value:,.{min(decimals, 6)}f}"


def format_duration(seconds: float) -> str:
    """Durations read in seconds; milliseconds below a tenth, so 0.02s stays visible."""
    if seconds < 0.1:
        return f"{seconds * 1000:.0f}ms"
    return f"{seconds:.2f}s"


def is_number(value) -> bool:
    """True for a plottable number. Booleans are excluded: they belong on a timeline."""
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
    )


def clip_series(series, t0, t1) -> list[tuple]:
    """
    Restrict a series to a time window, keeping one sample either side.

    The straddling samples matter: without them a line drawn over a window
    starts at the window edge instead of where the signal actually was, which
    reads as the signal having jumped.
    """
    if not series:
        return []
    if t0 is None and t1 is None:
        return list(series)
    lo = -math.inf if t0 is None else t0
    hi = math.inf if t1 is None else t1
    out: list[tuple] = []
    before = None
    for ts, value in series:
        if ts < lo:
            before = (ts, value)
            continue
        if ts > hi:
            out.append((ts, value))
            break
        if before is not None and not out:
            out.append(before)
        out.append((ts, value))
    if not out and before is not None:
        out.append(before)
    return out


def decimate(series, max_points: int = 1200) -> list[tuple]:
    """
    Reduce a numeric series to roughly ``max_points`` without losing spikes.

    Stride sampling would drop the 20ms dip that explains a missed shot, so
    bucket the samples and keep each bucket first, minimum, maximum and last
    sample in chronological order. Worst case is four points per bucket.
    """
    count = len(series)
    if count <= max_points or max_points < 4:
        return list(series)
    buckets = max(1, max_points // 4)
    width = count / buckets
    out: list[tuple] = []
    for index in range(buckets):
        start = int(index * width)
        end = count if index == buckets - 1 else int((index + 1) * width)
        chunk = series[start:end]
        if not chunk:
            continue
        numeric = [point for point in chunk if is_number(point[1])]
        if not numeric:
            out.append(chunk[0])
            continue
        picks = [
            chunk[0],
            min(numeric, key=lambda p: p[1]),
            max(numeric, key=lambda p: p[1]),
            chunk[-1],
        ]
        seen = set()
        for point in sorted(picks, key=lambda p: p[0]):
            if point[0] not in seen:
                seen.add(point[0])
                out.append(point)
    return out


def sample_series(series, interval: float = 0.5) -> list[tuple]:
    """Thin a series to at most one sample per ``interval`` seconds, for table views."""
    out: list[tuple] = []
    next_ts = -math.inf
    for ts, value in series:
        if ts >= next_ts:
            out.append((ts, value))
            next_ts = ts + interval
    if series and out and out[-1][0] != series[-1][0]:
        out.append(series[-1])
    return out


def _block_key(value) -> str:
    """Discrete values compare as their rendered text, matching the text reports."""
    return str(value)


def coalesce_blocks(series, t_end=None) -> list[tuple[float, float, str]]:
    """
    Collapse a discrete series into [(start, end, value)] blocks.

    Consecutive samples with the same value become one block. The final block
    runs to ``t_end`` (the log end), because a state that never changes again
    has no closing sample of its own -- without this the last state, often the
    one that matters, renders as a hairline at the right edge.
    """
    if not series:
        return []
    blocks: list[tuple[float, float, str]] = []
    start = series[0][0]
    current = _block_key(series[0][1])
    for ts, value in series[1:]:
        key = _block_key(value)
        if key != current:
            blocks.append((start, ts, current))
            start, current = ts, key
    last = t_end if t_end is not None and t_end > start else series[-1][0]
    if last <= start:
        # A single-sample series would otherwise be a zero-width block: invisible.
        last = start + 0.02
    blocks.append((start, last, current))
    return blocks


def series_extent(series_list) -> tuple[float, float]:
    """Min/max across several numeric series. (inf, -inf) when nothing is plottable."""
    lo, hi = math.inf, -math.inf
    for series in series_list:
        for _, value in series:
            if is_number(value):
                lo = min(lo, value)
                hi = max(hi, value)
    return lo, hi


def time_extent(series_list) -> tuple[float, float]:
    """Min/max timestamp across several series."""
    lo, hi = math.inf, -math.inf
    for series in series_list:
        if series:
            lo = min(lo, series[0][0])
            hi = max(hi, series[-1][0])
    return lo, hi
