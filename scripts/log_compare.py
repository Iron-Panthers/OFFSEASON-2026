"""
Compare a simulation .wpilog against a real match .wpilog.

Aligns both logs on first-enable, resamples every shared numeric key onto a
common grid, and ranks keys by how much they diverge.
"""

import json
import math
from bisect import bisect_right
from dataclasses import dataclass

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


def find_match_window(data):
    """
    (first_enable, last_disable) for the match, or (None, None).

    Both real and sim logs contain long stretches of pre- and post-match idle.
    Scoring across those inflates the compared span far beyond the ~165s the
    robot was actually playing, and dilutes every fit score with dead time.
    """
    start = find_first_enable(data)
    if start is None:
        return None, None
    end = None
    for ts, value in data.get("DriverStation/Enabled", []):
        if ts > start and not value:
            end = ts
    return start, end


# Key prefixes that carry no physics and would otherwise dominate the ranking:
# network client ports, wall-clock epochs, match numbers and CAN error counters
# all differ hugely between two runs while telling us nothing about whether the
# simulation models the robot correctly.
_EXCLUDED_PREFIXES = (
    "Timestamp",
    "SystemStats/NTClients/",
    "SystemStats/EpochTimeMicros",
    "SystemStats/CANBus/",
    "SystemStats/UserActive",
    # Rails, match clock and per-camera vision distances are not modelled in
    # simulation at all, so they diverge totally while telling us nothing about
    # robot physics. They otherwise dominate the ranking.
    "SystemStats/3v3Rail/",
    "SystemStats/5vRail/",
    "SystemStats/6vRail/",
    "SystemStats/BatteryCurrent",
    "DriverStation/MatchTime",
    "RealOutputs/Match Time",
    "RealOutputs/Vision/",
    # The PDH is not simulated at all; its voltage sits at a constant 12 V in sim.
    "PowerDistribution/",
    # Cumulative wheel odometry integrates the entire match's driving. After autonomous the
    # simulated robot necessarily takes a different path -- the real one was being defended and
    # contacted, which the simulation does not model -- so these diverge for reasons that have
    # nothing to do with mechanism fidelity. Reproducing post-auto pose is an explicit non-goal
    # (see the design spec); scoring it here would just conflate path divergence with model
    # error. Instantaneous drive VELOCITY is still scored, and that is the signal that actually
    # reflects the drivetrain model.
    "SystemStats/CPUTemp",
    "DriverStation/MatchNumber",
    "DriverStation/ReplayNumber",
    "DriverStation/MatchType",
    "DriverStation/AllianceStation",
    "RealOutputs/Logger/",
    "RealOutputs/LoggedRobot/",
    "NetworkInputs/",
    "RadioStatus/",
)


REPLAY_CLOCK_KEY = "RealOutputs/Replay/Elapsed Seconds"


def replay_time_base(data):
    """
    Build fpga_seconds -> match_elapsed_seconds for a replay sim log.

    AdvantageKit stamps records with wall clock, so two runs of identical code finish at slightly
    different wall times and the same event lands on different grid points once resampled. That
    showed up as a ~0.016 noise floor in the category scores -- the same size as the effects being
    measured. The replay player logs exact match time every loop, so use that instead and the
    comparison becomes invariant to wall-clock jitter.

    Returns None when the key is absent (a non-replay log), in which case callers fall back to
    plain first-enable rebasing.
    """
    series = data.get(REPLAY_CLOCK_KEY, [])
    if len(series) < 2:
        return None
    stamps = [ts for ts, _ in series]
    elapsed = [v for _, v in series]

    def to_match_time(ts):
        idx = bisect_right(stamps, ts) - 1
        if idx < 0:
            return elapsed[0] - (stamps[0] - ts)
        if idx >= len(stamps) - 1:
            return elapsed[-1] + (ts - stamps[-1])
        # Linear interpolation between logged loop stamps.
        span = stamps[idx + 1] - stamps[idx]
        if span <= 0:
            return elapsed[idx]
        frac = (ts - stamps[idx]) / span
        return elapsed[idx] + frac * (elapsed[idx + 1] - elapsed[idx])

    return to_match_time


# Suffixes excluded wherever they appear, rather than by prefix.
_EXCLUDED_SUFFIXES = ("/DrivePositionRads", "/DrivePositionMeters")


def is_excluded(key):
    """True for metadata keys that should never enter the fidelity ranking."""
    if any(key.endswith(suffix) for suffix in _EXCLUDED_SUFFIXES):
        return True
    return any(key.startswith(prefix) for prefix in _EXCLUDED_PREFIXES)


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


def worst_windows(sim, real, grid, window_s=2.0, top=3):
    """
    Find the `top` non-overlapping time windows where sim diverges most.

    Returns [(start_s, end_s, mean_abs_error), ...] worst first. Windows are
    non-overlapping so the report shows distinct problem areas rather than
    three views of the same spike.
    """
    if not grid:
        return []
    # Infer spacing from the grid actually passed in rather than assuming
    # GRID_DT. Callers legitimately pass coarser grids, and silently treating
    # them as 20ms would collapse the whole run into a single window.
    spacing = (grid[1] - grid[0]) if len(grid) > 1 else GRID_DT
    span = max(1, int(round(window_s / spacing))) if spacing > 0 else 1
    scored = []
    for start in range(0, len(grid), span):
        chunk = [
            (s, r)
            for s, r in zip(sim[start : start + span], real[start : start + span])
            if s is not None and r is not None
        ]
        if not chunk:
            continue
        err = sum(abs(s - r) for s, r in chunk) / len(chunk)
        end_idx = min(start + span - 1, len(grid) - 1)
        scored.append((grid[start], grid[end_idx], err))

    scored.sort(key=lambda w: -w[2])
    return scored[:top]


def extract_events(series):
    """[(ts, value), ...] reduced to points where the value changed."""
    events, prev = [], object()  # sentinel: nothing equals a fresh object()
    for ts, value in series:
        if value != prev:
            events.append((ts, value))
            prev = value
    return events


def pair_events(sim_events, real_events):
    """
    Pair sim events against real events positionally, stopping at the first
    state mismatch.

    Returns [(state, sim_ts, real_ts, delta_s), ...]. Pairing stops at a
    mismatch because once the two runs take different branches, later events
    are not the same events and comparing their timing is noise.
    """
    pairs = []
    for (sim_ts, sim_val), (real_ts, real_val) in zip(sim_events, real_events):
        if sim_val != real_val:
            break
        pairs.append((real_val, sim_ts, real_ts, sim_ts - real_ts))
    return pairs


CATEGORY_MECHANISMS = "mechanisms"
CATEGORY_CURRENTS = "currents"
CATEGORY_VOLTAGE = "voltage"
CATEGORY_AGGREGATE = "aggregate"
CATEGORY_OTHER = "other"


# Substrings that genuinely denote an electrical current channel.
#
# Deliberately explicit rather than a bare "Current" substring test: the logs
# are full of keys like "Current Velocity", "Current Position" and "Current
# Pose", where "Current" means *present*, not *amperage*. Matching those as
# currents would file flywheel velocity and swerve pose into the currents
# bucket and make that fit score meaningless.
_CURRENT_MARKERS = (
    "StatorCurrent",
    "SupplyCurrent",
    "Amps",
    "Filtered Current",
    "Rail/Current",
    "BatteryCurrent",
    "Amp Seconds",
)


def categorize(key):
    """
    Bucket a log key into a fidelity category.

    Order matters: the most specific patterns are checked first, so that
    MotorOutputManager/TotalAmps scores as aggregate rather than as just
    another current channel.
    """
    if key.startswith("RealOutputs/MotorOutputManager/"):
        return CATEGORY_AGGREGATE
    if key in ("SystemStats/BatteryVoltage", "SystemStats/BatteryCurrent"):
        return CATEGORY_VOLTAGE
    if any(marker in key for marker in _CURRENT_MARKERS):
        return CATEGORY_CURRENTS
    # "Vel" catches both Velocity and the abbreviated DriveVelRadsScalar form.
    if any(token in key for token in ("Position", "Vel", "Rotations", "Rads", "Meters")):
        return CATEGORY_MECHANISMS
    return CATEGORY_OTHER


def _is_constant(resampled):
    """True if every non-None value in a resampled series is identical."""
    seen = None
    for value in resampled:
        if value is None:
            continue
        if seen is None:
            seen = value
        elif value != seen:
            return False
    return True


def _numeric_series(series):
    """True if the series holds plain numbers (bools are not useful to score)."""
    if not series:
        return False
    sample = series[0][1]
    return isinstance(sample, (int, float)) and not isinstance(sample, bool)


def compare_logs(sim_path, real_path, top=25, json_out=None, event_keys=None):
    """
    Compare two logs and return the report as a string.

    The reader is imported lazily so this module stays unit-testable without
    pulling in the wpilog parser.
    """
    from wpilog_to_csv import read_log

    sim_data = read_log(sim_path)
    real_data = read_log(real_path)

    sim_t0, sim_end = find_match_window(sim_data)
    real_t0, real_end = find_match_window(real_data)
    if sim_t0 is None or real_t0 is None:
        missing = "sim" if sim_t0 is None else "real"
        return (
            f"ERROR: no DriverStation/Enabled -> True found in the {missing} log. "
            "Cannot align the two runs."
        )

    # Prefer the replay clock for the sim side; it is exact match time rather than wall clock.
    sim_clock = replay_time_base(sim_data)

    lines = [
        "=== LOG COMPARISON ===",
        f"sim:  {sim_path}",
        f"real: {real_path}",
        f"aligned on first-enable: sim t0={sim_t0:.2f}s, real t0={real_t0:.2f}s",
        "sim time base: "
        + ("replay match clock (wall-clock jitter removed)" if sim_clock else "wall clock"),
    ]

    # Bound the comparison by the shorter of the two ENABLED windows. Using the
    # last record in each file instead would drag in post-match idle and inflate
    # the span well past the ~165s the robot was actually playing.
    def _span(t0, t_end, data):
        if t_end is not None:
            return t_end - t0
        return max((ts for s in data.values() for ts, _ in s), default=t0) - t0

    duration = min(_span(sim_t0, sim_end, sim_data), _span(real_t0, real_end, real_data))
    if duration <= 0:
        return "ERROR: no overlapping enabled time between the two logs."
    grid = [i * GRID_DT for i in range(int(duration / GRID_DT))]
    lines.append(
        f"compared span: 0.00s to {duration:.2f}s of enabled time "
        f"({len(grid)} grid points)"
    )
    lines.append("")

    shared = sorted(set(sim_data) & set(real_data))
    scored, category_totals = [], {}
    skipped_constant = 0
    for key in shared:
        if is_excluded(key):
            continue
        if not _numeric_series(sim_data[key]) or not _numeric_series(real_data[key]):
            continue
        sim_series = (
            [(sim_clock(ts), v) for ts, v in sim_data[key]]
            if sim_clock
            else [(ts - sim_t0, v) for ts, v in sim_data[key]]
        )
        sim_r = resample(sim_series, grid)
        real_r = resample([(ts - real_t0, v) for ts, v in real_data[key]], grid)

        # A key that is constant on both sides has no dynamics to compare. Any
        # difference is a configuration or identity mismatch, not a fidelity
        # problem, and normalizing it produces a meaningless huge nrmse.
        if _is_constant(sim_r) and _is_constant(real_r):
            skipped_constant += 1
            continue

        score = score_pair(sim_r, real_r)
        if score is None:
            continue
        scored.append((key, score, sim_r, real_r))
        category_totals.setdefault(categorize(key), []).append(score.nrmse)

    scored.sort(key=lambda row: -row[1].nrmse)

    sim_only = sorted(set(sim_data) - set(real_data))
    real_only = sorted(set(real_data) - set(sim_data))

    lines.append(
        f"--- TOP {top} DIVERGING SIGNALS "
        f"(of {len(scored)} scored, {skipped_constant} constant-on-both skipped) ---"
    )
    for key, score, sim_r, real_r in scored[:top]:
        lines.append("")
        lines.append(f"{key}   [{categorize(key)}]")
        lines.append(
            f"  nrmse={score.nrmse:.3f}  mean_shift={score.mean_shift:+.3f}  "
            f"p95_shift={score.p95_shift:+.3f}  peak_shift={score.peak_shift:+.3f}  "
            f"corr={score.correlation:.3f}  n={score.samples}"
        )
        for start, end, err in worst_windows(sim_r, real_r, grid):
            lines.append(f"    worst {start:7.2f}s-{end:7.2f}s  mean_abs_err={err:.3f}")

    # Event-aligned diff for the state machine channels.
    if event_keys is None:
        event_keys = [
            k
            for k in shared
            if (k.endswith("/Target") or k.endswith("/Target State")) and not is_excluded(k)
        ]
    if event_keys:
        lines.append("")
        lines.append("--- EVENT-ALIGNED STATE TIMING ---")
        for key in sorted(event_keys):
            # Clip to the enabled window: pre-match idle states are not events
            # either run "performed", and pairing them produces nonsense deltas
            # like real=-227.80s.
            sim_events = extract_events(
                [
                    (sim_clock(ts) if sim_clock else ts - sim_t0, v)
                    for ts, v in sim_data.get(key, [])
                    if ts >= sim_t0
                ]
            )
            real_events = extract_events(
                [(ts - real_t0, v) for ts, v in real_data.get(key, []) if ts >= real_t0]
            )
            pairs = pair_events(sim_events, real_events)
            if not pairs:
                continue
            lines.append("")
            lines.append(
                f"{key}: {len(pairs)} matched of "
                f"{len(sim_events)} sim / {len(real_events)} real events"
            )
            for state, sim_ts, real_ts, delta in pairs[:12]:
                lines.append(
                    f"    {str(state):<28} real={real_ts:7.2f}s  sim={sim_ts:7.2f}s  "
                    f"delta={delta:+.2f}s"
                )

    lines.append("")
    lines.append("--- FIT SCORE BY CATEGORY (mean nrmse, lower is better) ---")
    fit = {}
    for category, values in sorted(category_totals.items()):
        fit[category] = sum(values) / len(values)
        lines.append(f"  {category:<12} {fit[category]:.4f}   ({len(values)} signals)")

    if sim_only or real_only:
        lines.append("")
        lines.append(
            f"--- UNCOMPARABLE KEYS: {len(sim_only)} sim-only, {len(real_only)} real-only ---"
        )
        for key in real_only[:15]:
            lines.append(f"  real-only: {key}")

    if json_out:
        with open(json_out, "w", encoding="utf-8") as handle:
            json.dump(
                {
                    "sim": sim_path,
                    "real": real_path,
                    "duration_s": duration,
                    "categories": fit,
                    "signals": {
                        key: {
                            "nrmse": s.nrmse,
                            "mean_shift": s.mean_shift,
                            "p95_shift": s.p95_shift,
                            "peak_shift": s.peak_shift,
                            "correlation": s.correlation,
                            "samples": s.samples,
                            "category": categorize(key),
                        }
                        for key, s, _, _ in scored
                    },
                },
                handle,
                indent=2,
            )
        lines.append("")
        lines.append(f"Wrote fit scores to {json_out}")

    return "\n".join(lines)
