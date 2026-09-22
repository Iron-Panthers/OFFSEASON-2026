"""
The chart forms: timeseries, state timeline, field path.

Each renderer takes already-resolved series data and returns a ``Rendered`` --
SVG markup, a hover configuration for the page script, and a table view. No file
I/O and no log reading, so a chart can be rendered from a literal series in a
test.

Mark specs here are fixed by the data-viz reference and are not style choices:
2px lines with round caps, markers at r=4 wearing a 2px surface ring, area fills
at 10% opacity, solid hairline gridlines (never dashed), a 2px surface gap
between touching blocks, and labels drawn inside a block only when they measure
as fitting. Colours are emitted as CSS variables rather than hex so that one
render serves both light and dark mode.
"""

from __future__ import annotations

import math
from bisect import bisect_right
from dataclasses import dataclass, field

from .palette import SlotRegistry, on_series_var, series_var
from .spec import Prose
from .scales import (
    Scale,
    coalesce_blocks,
    decimate,
    format_duration,
    format_number,
    is_number,
    nice_axis,
    nice_step,
    sample_series,
    series_extent,
    time_extent,
)

VIEW_W = 880
PAD_L = 68            # room for y tick labels
PAD_R = 98            # room for direct end labels
PAD_T = 22          # clears the unit label that sits above the plot
AXIS_BAND = 36        # x tick labels live here; the viewBox must include it
PLOT_H = 210
LANE_H = 24
LANE_GAP = 10
TIMELINE_PAD_L = 162  # room for lane names
PATH_H = 420

FONT_AXIS = 11
FONT_LANE = 12
FONT_BLOCK = 11
SURFACE_GAP = 2       # the gap that separates touching blocks
MARKER_R = 4
MIN_LABEL_SEPARATION = 14   # px between end labels before they count as colliding
MAX_DIRECT_LABELS = 4       # past this, identity rides the legend alone
MAX_TABLE_ROWS = 160


@dataclass
class Rendered:
    """One finished chart: markup plus everything the page needs around it."""

    title: str
    svg: str
    table_html: str
    subtitle: str = ""
    note: str = ""
    # Carried from the spec so the page can print the reading of the chart
    # directly beneath it, where the reader is still looking at the lines.
    reads_as: Prose = field(default_factory=Prose)
    legend: tuple = field(default_factory=tuple)   # ((label, slot), ...)
    hover: dict | None = None
    empty_reason: str = ""


# --- small helpers ---------------------------------------------------------


def esc(text) -> str:
    """Escape text for XML content and attribute values."""
    return (
        str(text)
        .replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
        .replace("'", "&#39;")
    )


def _n(value: float) -> str:
    """Compact coordinate: two decimals, no trailing zeros, no negative zero."""
    if not math.isfinite(value):
        return "0"
    text = f"{value:.2f}".rstrip("0").rstrip(".")
    return "0" if text in ("", "-", "-0") else text


# Average glyph advance as a fraction of font size for a UI sans. Used only to
# decide whether a label fits inside a block -- deliberately generous, because
# the failure mode of guessing too narrow is clipped text, which is worse than
# no label at all.
_ADVANCE_MIXED = 0.58
_ADVANCE_CAPS = 0.66


def text_width(text: str, size: float) -> float:
    """Estimated rendered width. Conservative: over-estimates rather than clips."""
    if not text:
        return 0.0
    caps = sum(1 for char in text if char.isupper() or char.isdigit())
    advance = _ADVANCE_CAPS if caps > len(text) * 0.6 else _ADVANCE_MIXED
    return len(text) * size * advance


def _hold(series, timestamps, at: float):
    """Value in force at time ``at`` (zero-order hold), or None before the first sample."""
    index = bisect_right(timestamps, at) - 1
    return series[index][1] if index >= 0 else None


def _window_domain(window, fallback_lo, fallback_hi):
    """
    x-domain for a chart.

    A finding window is padded by 20% either side rather than clipped flush to
    it: the shaded window then reads as an interval inside a wider trace, so you
    can see what the signal was doing on the way in and out. Clipping flush
    makes every window look like the whole story.
    """
    if window is None:
        return fallback_lo, fallback_hi
    start, end = window
    pad = max((end - start) * 0.2, 0.25)
    return start - pad, end + pad


def _plot_frame(x: Scale, y: Scale, x_ticks, y_ticks, x_step, y_step, top, bottom, unit):
    """Gridlines, ticks and the two axis rules. Solid hairlines, recessive."""
    out = []
    left, right = x.p0, x.p1
    for tick in y_ticks:
        py = y.px(tick)
        out.append(
            f'<line class="vz-grid" x1="{_n(left)}" y1="{_n(py)}" '
            f'x2="{_n(right)}" y2="{_n(py)}"/>'
        )
        out.append(
            f'<text class="vz-tick vz-tick-y" x="{_n(left - 8)}" y="{_n(py + 3.5)}">'
            f"{esc(format_number(tick, y_step))}</text>"
        )
    for tick in x_ticks:
        px = x.px(tick)
        if px < left - 0.5 or px > right + 0.5:
            continue
        out.append(
            f'<line class="vz-grid-x" x1="{_n(px)}" y1="{_n(top)}" '
            f'x2="{_n(px)}" y2="{_n(bottom)}"/>'
        )
        out.append(
            f'<text class="vz-tick vz-tick-x" x="{_n(px)}" y="{_n(bottom + 16)}">'
            f"{esc(format_number(tick, x_step))}</text>"
        )
    out.append(
        f'<line class="vz-axis" x1="{_n(left)}" y1="{_n(bottom)}" '
        f'x2="{_n(right)}" y2="{_n(bottom)}"/>'
    )
    out.append(
        f'<text class="vz-axis-label" x="{_n(right)}" y="{_n(bottom + 16)}" '
        f'text-anchor="end">seconds</text>'
    )
    if unit:
        # Sits above the plot at its left edge, not in the tick column: at
        # text-anchor="end" beside the axis it lands on top of the highest tick.
        out.append(
            f'<text class="vz-axis-label" x="{_n(left)}" y="{_n(top - 5)}" '
            f'text-anchor="start">{esc(unit)}</text>'
        )
    return out


def _polyline(points, slot: int, extra: str = "") -> str:
    if len(points) == 1:
        px, py = points[0]
        return (
            f'<circle cx="{_n(px)}" cy="{_n(py)}" r="{MARKER_R}" '
            f'fill="{series_var(slot)}"/>'
        )
    coords = " ".join(f"{_n(px)},{_n(py)}" for px, py in points)
    return (
        f'<polyline class="vz-line" points="{coords}" '
        f'stroke="{series_var(slot)}" {extra}/>'
    )


def _end_marker(px: float, py: float, slot: int) -> str:
    """r=4 dot wearing a 2px surface ring, so it stays legible over a crossing line."""
    return (
        f'<circle class="vz-dot" cx="{_n(px)}" cy="{_n(py)}" r="{MARKER_R}" '
        f'fill="{series_var(slot)}"/>'
    )


def _direct_labels(entries):
    """
    End labels, or nothing.

    Labels are dropped wholesale when two of them would sit within
    MIN_LABEL_SEPARATION: nudging converging labels apart detaches them from
    their lines and reads as noise. The legend is always present, so dropping
    them loses no identity.
    """
    if not entries or len(entries) > MAX_DIRECT_LABELS:
        return []
    ordered = sorted(entries, key=lambda item: item[1])
    for (_, y_a, _), (_, y_b, _) in zip(ordered, ordered[1:]):
        if abs(y_b - y_a) < MIN_LABEL_SEPARATION:
            return []
    return [
        f'<text class="vz-end-label" x="{_n(px)}" y="{_n(py + 3.5)}">{esc(label)}</text>'
        for label, py, px in entries
    ]


# --- table views -----------------------------------------------------------


def _numeric_table(resolved, x0: float, x1: float) -> str:
    """
    Sampled value table -- the non-visual twin of a timeseries.

    Every chart gets one because hover must never be the only way to read a
    value, and because three of the light-mode palette slots sit below 3:1
    against the surface, which obliges exactly this relief.
    """
    duration = max(x1 - x0, 1e-6)
    interval = max(nice_step(duration, MAX_TABLE_ROWS), 0.02)
    grid: list[float] = []
    steps = int(duration / interval) + 1
    for index in range(min(steps, MAX_TABLE_ROWS)):
        grid.append(x0 + index * interval)

    prepared = []
    for label, slot, points in resolved:
        stamps = [ts for ts, _ in points]
        prepared.append((label, stamps, points))

    head = "".join(f"<th>{esc(label)}</th>" for label, _, _ in prepared)
    rows = []
    for at in grid:
        cells = "".join(
            f"<td>{esc(format_number(_hold(points, stamps, at)))}</td>"
            for _, stamps, points in prepared
        )
        rows.append(f"<tr><th scope=\"row\">{esc(format_number(at, interval))}</th>{cells}</tr>")
    caption = f"Sampled every {format_duration(interval)}."
    return (
        f'<table class="vz-table"><caption>{esc(caption)}</caption>'
        f"<thead><tr><th scope=\"col\">Time (s)</th>{head}</tr></thead>"
        f"<tbody>{''.join(rows)}</tbody></table>"
    )


def _timeline_table(lanes) -> str:
    rows = []
    truncated = False
    for label, blocks in lanes:
        for start, end, value in blocks:
            if len(rows) >= MAX_TABLE_ROWS:
                truncated = True
                break
            rows.append(
                f"<tr><th scope=\"row\">{esc(label)}</th><td>{esc(value)}</td>"
                f"<td>{esc(format_number(start, 0.01))}</td>"
                f"<td>{esc(format_number(end, 0.01))}</td>"
                f"<td>{esc(format_duration(end - start))}</td></tr>"
            )
    caption = "Every state block." if not truncated else f"First {MAX_TABLE_ROWS} state blocks."
    return (
        f'<table class="vz-table"><caption>{esc(caption)}</caption><thead><tr>'
        "<th scope=\"col\">Lane</th><th scope=\"col\">State</th>"
        "<th scope=\"col\">Start (s)</th><th scope=\"col\">End (s)</th>"
        "<th scope=\"col\">Duration</th></tr></thead>"
        f"<tbody>{''.join(rows)}</tbody></table>"
    )


def _path_table(points) -> str:
    sampled = sample_series(points, max(nice_step(1.0, 1), 0.1))
    rows = []
    for ts, value in sampled[:MAX_TABLE_ROWS]:
        x, y = value[0], value[1]
        rows.append(
            f"<tr><th scope=\"row\">{esc(format_number(ts, 0.01))}</th>"
            f"<td>{esc(format_number(x, 0.01))}</td>"
            f"<td>{esc(format_number(y, 0.01))}</td></tr>"
        )
    return (
        '<table class="vz-table"><caption>Pose samples.</caption><thead><tr>'
        '<th scope="col">Time (s)</th><th scope="col">X (m)</th>'
        '<th scope="col">Y (m)</th></tr></thead>'
        f"<tbody>{''.join(rows)}</tbody></table>"
    )


# --- timeseries ------------------------------------------------------------


def timeseries(spec, data, t_end=None) -> Rendered:
    """
    Actual against setpoint over time. One axis, always.

    Two signals of different units never share this chart -- that is the
    dual-axis mistake, where the arbitrary alignment of two scales invents a
    correlation. Callers wanting velocity beside current emit two charts.
    """
    resolved = []
    for index, series_spec in enumerate(spec.series):
        points = [
            (ts, float(value))
            for ts, value in data.get(series_spec.key, [])
            if is_number(value)
        ]
        if points:
            # Slot by position within the chart: the leading slots are the
            # high-contrast ones, and position is stable here because nothing
            # filters or reorders series after render.
            resolved.append((series_spec.label, index + 1, points))
    if not resolved:
        return Rendered(title=spec.title, svg="", table_html="", empty_reason="no numeric samples")

    raw_lo, raw_hi = time_extent([points for _, _, points in resolved])
    x0, x1 = _window_domain(spec.window, raw_lo, raw_hi)
    if x1 <= x0:
        x1 = x0 + 1.0

    clipped = []
    for label, slot, points in resolved:
        inside = [(ts, value) for ts, value in points if x0 <= ts <= x1]
        if not inside:
            inside = points[-1:]
        clipped.append((label, slot, decimate(inside)))

    values = [points for _, _, points in clipped]
    lo, hi = series_extent(values)
    if spec.tolerance_pct:
        target = clipped[-1][2]
        for _, value in target:
            lo = min(lo, value * (1 - spec.tolerance_pct))
            hi = max(hi, value * (1 + spec.tolerance_pct))
    y0, y1, y_ticks = nice_axis(lo, hi, include_zero=spec.include_zero)
    y_step = y_ticks[1] - y_ticks[0] if len(y_ticks) > 1 else 1.0
    _, _, x_ticks = nice_axis(x0, x1, target=7)
    x_step = x_ticks[1] - x_ticks[0] if len(x_ticks) > 1 else 1.0

    left, right = PAD_L, VIEW_W - PAD_R
    top, bottom = PAD_T, PAD_T + PLOT_H
    height = bottom + AXIS_BAND
    x = Scale(x0, x1, left, right)
    y = Scale(y0, y1, bottom, top)

    body = []
    if spec.window is not None:
        w0, w1 = spec.window
        body.append(
            f'<rect class="vz-window" x="{_n(x.clamp_px(w0))}" y="{_n(top)}" '
            f'width="{_n(max(x.clamp_px(w1) - x.clamp_px(w0), 1))}" '
            f'height="{_n(bottom - top)}"/>'
        )
    body += _plot_frame(x, y, x_ticks, y_ticks, x_step, y_step, top, bottom, spec.unit)

    if spec.tolerance_pct:
        label, slot, target = clipped[-1]
        upper = [(x.px(ts), y.clamp_px(value * (1 + spec.tolerance_pct))) for ts, value in target]
        lower = [(x.px(ts), y.clamp_px(value * (1 - spec.tolerance_pct))) for ts, value in target]
        ring = " ".join(f"{_n(px)},{_n(py)}" for px, py in upper + list(reversed(lower)))
        body.append(
            f'<polygon class="vz-band" points="{ring}" fill="{series_var(slot)}"/>'
        )

    end_labels = []
    for label, slot, points in clipped:
        pixels = [(x.px(ts), y.clamp_px(value)) for ts, value in points]
        body.append(_polyline(pixels, slot))
        last_px, last_py = pixels[-1]
        body.append(_end_marker(last_px, last_py, slot))
        end_labels.append((label, last_py, min(last_px + 10, VIEW_W - 4)))
    body += _direct_labels(end_labels)

    hover = {
        "mode": "x",
        "vw": VIEW_W,
        "px0": left,
        "px1": right,
        "py0": bottom,
        "py1": top,
        "x0": x0,
        "x1": x1,
        "y0": y0,
        "y1": y1,
        "unit": spec.unit,
        "step": y_step,
        "series": [
            {
                "label": label,
                "slot": slot,
                "pts": [[round(ts, 4), round(value, 5)] for ts, value in points],
            }
            for label, slot, points in clipped
        ],
    }

    subtitle = f"{format_number(x0, 0.01)}s to {format_number(x1, 0.01)}s"
    if spec.tolerance_pct:
        subtitle += f" · {spec.tolerance_pct:.0%} band on {clipped[-1][0]}"

    return Rendered(
        title=spec.title,
        svg=_svg(height, body),
        table_html=_numeric_table(clipped, x0, x1),
        subtitle=subtitle,
        note=spec.note,
        reads_as=spec.reads_as,
        legend=tuple((label, slot) for label, slot, _ in clipped),
        hover=hover,
    )


# --- state timeline --------------------------------------------------------


def timeline(spec, data, t_end=None, registry: SlotRegistry | None = None) -> Rendered:
    """
    A lane per state-machine key: what was commanded, and when.

    For the question "what happened, in what order" this is the highest-value
    form in the set, because a state machine has no magnitude to plot -- only
    identity and duration.

    Booleans are treated as a special case: ``False`` draws as an empty track
    rather than a saturated block, so a lane that is mostly false reads as mostly
    nothing instead of shouting. A wall of loud fill is the thing that makes
    these charts unreadable.
    """
    registry = registry if registry is not None else SlotRegistry()

    lanes = []
    for series_spec in spec.series:
        points = data.get(series_spec.key, [])
        if points:
            lanes.append((series_spec.label, coalesce_blocks(points, t_end)))
    if not lanes:
        return Rendered(title=spec.title, svg="", table_html="", empty_reason="no samples")

    raw_lo = min(blocks[0][0] for _, blocks in lanes)
    raw_hi = max(blocks[-1][1] for _, blocks in lanes)
    x0, x1 = _window_domain(spec.window, raw_lo, raw_hi)
    if x1 <= x0:
        x1 = x0 + 1.0
    _, _, x_ticks = nice_axis(x0, x1, target=7)
    x_step = x_ticks[1] - x_ticks[0] if len(x_ticks) > 1 else 1.0

    left, right = TIMELINE_PAD_L, VIEW_W - 24
    top = PAD_T
    plot_h = len(lanes) * LANE_H + (len(lanes) - 1) * LANE_GAP
    bottom = top + plot_h
    height = bottom + AXIS_BAND
    x = Scale(x0, x1, left, right)

    body = []
    if spec.window is not None:
        w0, w1 = spec.window
        body.append(
            f'<rect class="vz-window" x="{_n(x.clamp_px(w0))}" y="{_n(top)}" '
            f'width="{_n(max(x.clamp_px(w1) - x.clamp_px(w0), 1))}" '
            f'height="{_n(plot_h)}"/>'
        )
    for tick in x_ticks:
        px = x.px(tick)
        if px < left - 0.5 or px > right + 0.5:
            continue
        body.append(
            f'<line class="vz-grid-x" x1="{_n(px)}" y1="{_n(top)}" '
            f'x2="{_n(px)}" y2="{_n(bottom)}"/>'
        )
        body.append(
            f'<text class="vz-tick vz-tick-x" x="{_n(px)}" y="{_n(bottom + 16)}">'
            f"{esc(format_number(tick, x_step))}</text>"
        )
    body.append(
        f'<line class="vz-axis" x1="{_n(left)}" y1="{_n(bottom)}" '
        f'x2="{_n(right)}" y2="{_n(bottom)}"/>'
    )
    body.append(
        f'<text class="vz-axis-label" x="{_n(right)}" y="{_n(bottom + 16)}" '
        f'text-anchor="end">seconds</text>'
    )

    legend_values: dict[str, int] = {}
    for lane_index, (label, blocks) in enumerate(lanes):
        lane_y = top + lane_index * (LANE_H + LANE_GAP)
        lane_label = label
        while lane_label and text_width(lane_label, FONT_LANE) > TIMELINE_PAD_L - 16:
            lane_label = lane_label[:-1]
        if lane_label != label:
            # Truncate rather than overflow into the plot; the full name is in
            # the block tooltip and the table view.
            lane_label = lane_label[:-1] + "…" if len(lane_label) > 1 else "…"
        body.append(
            f'<text class="vz-lane" x="{_n(left - 10)}" y="{_n(lane_y + LANE_H / 2 + 4)}">'
            f"<title>{esc(label)}</title>{esc(lane_label)}</text>"
        )
        body.append(
            f'<rect class="vz-track" x="{_n(left)}" y="{_n(lane_y)}" '
            f'width="{_n(right - left)}" height="{LANE_H}" rx="3"/>'
        )
        for start, end, value in blocks:
            if end < x0 or start > x1:
                continue
            bx0 = x.clamp_px(start)
            bx1 = x.clamp_px(end)
            width = bx1 - bx0 - SURFACE_GAP
            if width <= 0.4:
                continue
            falsey = value in ("False", "false", "0")
            tip = (
                f"{label}: {value} · {format_number(start, 0.01)}s to "
                f"{format_number(end, 0.01)}s · {format_duration(end - start)}"
            )
            if falsey:
                body.append(
                    f'<rect class="vz-block vz-block-off" x="{_n(bx0)}" y="{_n(lane_y)}" '
                    f'width="{_n(width)}" height="{LANE_H}" rx="3" tabindex="0" '
                    f'data-tip="{esc(tip)}"/>'
                )
                continue
            slot = registry.slot(value)
            legend_values.setdefault(value, slot)
            body.append(
                f'<rect class="vz-block" x="{_n(bx0)}" y="{_n(lane_y)}" '
                f'width="{_n(width)}" height="{LANE_H}" rx="3" '
                f'fill="{series_var(slot)}" tabindex="0" data-tip="{esc(tip)}"/>'
            )
            # Only label inside the block when it measurably fits with padding.
            # Clipping the first characters of a state name is worse than no
            # label, and the value is in the table view either way.
            if text_width(value, FONT_BLOCK) + 12 <= width:
                body.append(
                    f'<text class="vz-block-label" x="{_n(bx0 + width / 2)}" '
                    f'y="{_n(lane_y + LANE_H / 2 + 3.5)}" '
                    f'fill="{on_series_var(slot)}">{esc(value)}</text>'
                )

    return Rendered(
        title=spec.title,
        svg=_svg(height, body),
        table_html=_timeline_table(lanes),
        subtitle=f"{format_number(x0, 0.01)}s to {format_number(x1, 0.01)}s",
        note=spec.note,
        reads_as=spec.reads_as,
        legend=tuple(sorted(legend_values.items(), key=lambda item: item[1])),
    )


# --- field path ------------------------------------------------------------


def path(spec, data, t_end=None) -> Rendered:
    """
    Driven pose against commanded pose, in field coordinates.

    Equal aspect, because an unequal one turns a straight line into a curve and a
    circle into an ellipse -- on a chart whose whole job is the shape of a path,
    that is a lie rather than a distortion.

    The path error is deliberately *not* encoded into this chart as colour or
    thickness. It goes in its own strip below (see log_charts), which is small
    multiples rather than a second scale smuggled onto one plot.
    """
    resolved = []
    for index, series_spec in enumerate(spec.series):
        points = []
        for ts, value in data.get(series_spec.key, []):
            # Pose2d/Pose3d arrive as tuples from wpilog_to_csv; anything else
            # is not a pose and is skipped rather than guessed at.
            if isinstance(value, (tuple, list)) and len(value) >= 2:
                if is_number(value[0]) and is_number(value[1]):
                    points.append((ts, (float(value[0]), float(value[1]))))
        if points:
            resolved.append((series_spec.label, index + 1, points))
    if not resolved:
        return Rendered(title=spec.title, svg="", table_html="", empty_reason="no pose samples")

    if spec.window is not None:
        w0, w1 = spec.window
        trimmed = []
        for label, slot, points in resolved:
            inside = [point for point in points if w0 <= point[0] <= w1]
            if inside:
                trimmed.append((label, slot, inside))
        if trimmed:
            resolved = trimmed

    xs = [value[0] for _, _, points in resolved for _, value in points]
    ys = [value[1] for _, _, points in resolved for _, value in points]
    x_lo, x_hi, x_ticks = nice_axis(min(xs), max(xs), target=6)
    y_lo, y_hi, y_ticks = nice_axis(min(ys), max(ys), target=5)
    x_step = x_ticks[1] - x_ticks[0] if len(x_ticks) > 1 else 1.0
    y_step = y_ticks[1] - y_ticks[0] if len(y_ticks) > 1 else 1.0

    left, right = PAD_L, VIEW_W - 24
    top = PAD_T
    avail_w = right - left
    avail_h = PATH_H - AXIS_BAND - PAD_T

    # Equal aspect: grow the shorter domain to match the longer one metre-for-pixel.
    span_x = x_hi - x_lo
    span_y = y_hi - y_lo
    scale = max(span_x / avail_w, span_y / avail_h)
    plot_w = span_x / scale
    plot_h = span_y / scale
    bottom = top + plot_h
    height = bottom + AXIS_BAND
    x = Scale(x_lo, x_lo + plot_w * scale, left, left + plot_w)
    y = Scale(y_lo, y_lo + plot_h * scale, bottom, top)

    body = []
    for tick in y_ticks:
        if not y_lo - 1e-9 <= tick <= y.d1 + 1e-9:
            continue
        py = y.px(tick)
        body.append(
            f'<line class="vz-grid" x1="{_n(left)}" y1="{_n(py)}" '
            f'x2="{_n(left + plot_w)}" y2="{_n(py)}"/>'
        )
        body.append(
            f'<text class="vz-tick vz-tick-y" x="{_n(left - 8)}" y="{_n(py + 3.5)}">'
            f"{esc(format_number(tick, y_step))}</text>"
        )
    for tick in x_ticks:
        if not x_lo - 1e-9 <= tick <= x.d1 + 1e-9:
            continue
        px = x.px(tick)
        body.append(
            f'<line class="vz-grid-x" x1="{_n(px)}" y1="{_n(top)}" '
            f'x2="{_n(px)}" y2="{_n(bottom)}"/>'
        )
        body.append(
            f'<text class="vz-tick vz-tick-x" x="{_n(px)}" y="{_n(bottom + 16)}">'
            f"{esc(format_number(tick, x_step))}</text>"
        )
    body.append(
        f'<rect class="vz-frame" x="{_n(left)}" y="{_n(top)}" '
        f'width="{_n(plot_w)}" height="{_n(plot_h)}"/>'
    )
    body.append(
        f'<text class="vz-axis-label" x="{_n(left + plot_w)}" y="{_n(bottom + 16)}" '
        f'text-anchor="end">X (m)</text>'
    )
    body.append(
        f'<text class="vz-axis-label" x="{_n(left)}" y="{_n(top - 5)}" '
        f'text-anchor="start">Y (m)</text>'
    )

    hover_series = []
    for label, slot, points in resolved:
        reduced = points if len(points) <= 1500 else points[:: max(1, len(points) // 1500)]
        pixels = [(x.px(value[0]), y.px(value[1])) for _, value in reduced]
        body.append(_polyline(pixels, slot))
        body.append(_end_marker(pixels[0][0], pixels[0][1], slot))
        body.append(
            f'<rect class="vz-dot" x="{_n(pixels[-1][0] - 4)}" y="{_n(pixels[-1][1] - 4)}" '
            f'width="8" height="8" rx="1.5" fill="{series_var(slot)}"/>'
        )
        hover_series.append(
            {
                "label": label,
                "slot": slot,
                "pts": [
                    [round(ts, 3), round(value[0], 4), round(value[1], 4)]
                    for ts, value in reduced
                ],
            }
        )

    first_label, _, first_points = resolved[0]
    body.append(
        f'<text class="vz-end-label" x="{_n(x.px(first_points[0][1][0]) + 8)}" '
        f'y="{_n(y.px(first_points[0][1][1]) - 7)}">start</text>'
    )
    body.append(
        f'<text class="vz-end-label" x="{_n(x.px(first_points[-1][1][0]) + 8)}" '
        f'y="{_n(y.px(first_points[-1][1][1]) - 7)}">end</text>'
    )

    return Rendered(
        title=spec.title,
        svg=_svg(height, body),
        table_html=_path_table(first_points),
        subtitle=f"{len(first_points):,} poses · equal aspect",
        note=spec.note,
        reads_as=spec.reads_as,
        legend=tuple((label, slot) for label, slot, _ in resolved),
        hover={
            "mode": "xy",
            "vw": VIEW_W,
            "px0": left,
            "px1": left + plot_w,
            "py0": bottom,
            "py1": top,
            "x0": x.d0,
            "x1": x.d1,
            "y0": y.d0,
            "y1": y.d1,
            "unit": "m",
            "series": hover_series,
        },
    )


def _svg(height: float, body) -> str:
    return (
        f'<svg class="vz-svg" viewBox="0 0 {VIEW_W} {_n(height)}" '
        f'preserveAspectRatio="xMidYMid meet" role="img" '
        f'xmlns="http://www.w3.org/2000/svg">{"".join(body)}</svg>'
    )


RENDERERS = {"timeseries": timeseries, "timeline": timeline, "path": path}
