"""
WPILog analysis tool for AI-assisted FRC debugging.

Two modes:
  --investigate MODE   Structured analysis report (low token cost, use this by default)
  --prefix KEY         Raw CSV for a specific key prefix (deep dive only)

Usage:
  python scripts/wpilog_to_csv.py <file.wpilog> --investigate auto
  python scripts/wpilog_to_csv.py <file.wpilog> --investigate shooter
  python scripts/wpilog_to_csv.py <file.wpilog> --summary
  python scripts/wpilog_to_csv.py <file.wpilog> --prefix ShooterFlywheel/velocityRadsPerSec
"""

import argparse
import math
import struct
import sys
from collections import defaultdict
from pathlib import Path

# Keys to pull for each investigation mode
# All subsystem outputs are under "RealOutputs/" in AdvantageKit logs.
# DriverStation inputs are at the root (no prefix).
INVESTIGATE_KEYS = {
    "auto": [
        "DriverStation/Enabled",
        "DriverStation/Autonomous",
        "RealOutputs/Path Planner/Target Pose",
        "RealOutputs/Path Planner/Current Pose",
        "RealOutputs/Path Planner/Active Path",
        "RealOutputs/Intake/Intake Rack/Target",
        "RealOutputs/Intake/Intake Rollers/Target",
        "RealOutputs/Shooter/Target State",
        "RealOutputs/Field Simulation/Fuel Count",
        "RealOutputs/Shooter/Shooter Flywheels/Current Velocity",
    ],
    "shooter": [
        "DriverStation/Enabled",
        "RealOutputs/Shooter/Target State",
        "RealOutputs/Shooter/Shooter Flywheels/Current Velocity",
        "RealOutputs/Shooter/Shooter Flywheels/Target Velocity",
        "RealOutputs/Shooter/Shooter Hood/Target Position",
        "RealOutputs/Shooter/Shooter Hood/Target Position Manual",
        "RealOutputs/Shooter/Shooter Accelerator/Target Velocity",
        "RealOutputs/RobotState/Target Shooting State/Shooter Angle",
        "RealOutputs/RobotState/Target Shooting State/Shooter Speed",
        "RealOutputs/Shooter/Flywheels Up To Speed",
    ],
    "drive": [
        "DriverStation/Enabled",
        "DriverStation/Autonomous",
        "RealOutputs/Path Planner/Target Pose",
        "RealOutputs/Path Planner/Current Pose",
        "RealOutputs/Path Planner/Active Path",
        "RealOutputs/Swerve/Current Position",
        "RealOutputs/Robot State/Estimated Pose",
        "RealOutputs/Swerve/Distance From Setpoint",
    ],
    "intake": [
        "DriverStation/Enabled",
        "RealOutputs/Intake/Intake Rack/Target",
        "RealOutputs/Intake/Intake Rollers/Target",
        "RealOutputs/Intake/Intake Rack/Position Target Rotations",
        "RealOutputs/Intake/Intake Rack/Reached Target",
        "RealOutputs/Intake/Intake Rollers/Target Velocity",
        "RealOutputs/Field Simulation/Fuel Count",
        "RealOutputs/Shooter/Target State",
    ],
    "vision": [
        "DriverStation/Enabled",
        "RealOutputs/Vision/Camera0/Accepted Poses",
        "RealOutputs/Vision/Camera0/Average Distance",
        "RealOutputs/Robot State/Estimated Pose",
        "RealOutputs/Vision/GetMultiTags",
    ],
}

# Anomaly thresholds
PATH_ERROR_WARN_M = 0.25       # metres
VELOCITY_LAG_WARN_PCT = 0.10   # 10% below setpoint
VELOCITY_LAG_WARN_DUR_S = 0.3  # sustained for 300ms


# ─── WPILog reading ───────────────────────────────────────────────────────────

def _load_reader(path: str):
    try:
        from wpiutil.log import DataLogReader
    except ImportError:
        print("ERROR: wpiutil not found. Install: pip install robotpy-wpilib", file=sys.stderr)
        sys.exit(1)
    r = DataLogReader(path)
    if not r.isValid():
        print(f"ERROR: not a valid .wpilog: {path}", file=sys.stderr)
        sys.exit(1)
    return r


def _parse_value(record, dtype: str):
    """Return a Python value from a DataLog record, or None if unparseable."""
    try:
        if dtype == "boolean":        return record.getBoolean()
        if dtype == "int64":          return record.getInteger()
        if dtype in ("float", "double"): return record.getFloat() if dtype == "float" else record.getDouble()
        if dtype == "string":         return record.getString()
        if dtype == "boolean[]":      return list(record.getBooleanArray())
        if dtype == "int64[]":        return list(record.getIntegerArray())
        if dtype in ("float[]", "double[]"):
            return list(record.getFloatArray() if dtype == "float[]" else record.getDoubleArray())
        if dtype == "string[]":       return list(record.getStringArray())
        if dtype == "struct:Pose2d":  return _unpack_pose2d(record.getRaw())
        if dtype == "struct:Pose2d[]": return _unpack_pose2d_array(record.getRaw())
    except Exception:
        pass
    return None


def _unpack_pose2d(data: bytes):
    """Unpack a WPILib Pose2d struct -> (x_m, y_m, heading_rad)."""
    if len(data) < 24:
        return None
    x, y, rot = struct.unpack_from("<ddd", data)
    return (x, y, rot)


def _unpack_pose2d_array(data: bytes):
    poses = []
    for i in range(0, len(data) - 23, 24):
        x, y, rot = struct.unpack_from("<ddd", data, i)
        poses.append((x, y, rot))
    return poses


def read_log(
    path: str,
    keys: list[str] | None = None,
    prefix: str | None = None,
    time_start: float | None = None,
    time_end: float | None = None,
) -> dict[str, list[tuple[float, object]]]:
    """
    Read a .wpilog file and return a dict: key -> [(timestamp_sec, value), ...]
    keys: exact key names to include (None = all)
    prefix: include any key starting with this string (overrides keys)
    time_start/time_end: optional inclusive time window in seconds
    """
    reader = _load_reader(path)
    entry_map = {}   # entry_id -> (name, dtype)
    data: dict[str, list] = defaultdict(list)

    for record in reader:
        if record.isControl():
            if record.isStart():
                s = record.getStartData()
                entry_map[s.entry] = (s.name, s.type)
            continue

        entry_id = record.getEntry()
        if entry_id not in entry_map:
            continue

        name, dtype = entry_map[entry_id]
        name = name.lstrip("/")

        if prefix:
            if not name.startswith(prefix):
                continue
        elif keys:
            if name not in keys:
                continue

        value = _parse_value(record, dtype)
        if value is None:
            continue

        ts = record.getTimestamp() / 1_000_000.0
        if time_start is not None and ts < time_start:
            continue
        if time_end is not None and ts > time_end:
            continue

        data[name].append((ts, value))

    return dict(data)


# ─── Analysis helpers ─────────────────────────────────────────────────────────

def transitions(series: list[tuple]) -> list[tuple]:
    """[(ts, old, new)] where value changed."""
    result = []
    prev_val = None
    for ts, val in series:
        str_val = str(val)
        if str_val != prev_val:
            result.append((ts, prev_val, str_val))
            prev_val = str_val
    return result


def num_stats(series: list[tuple]) -> dict:
    """min/max/mean/std for a numeric series."""
    vals = [v for _, v in series if isinstance(v, (int, float)) and not math.isnan(v)]
    if not vals:
        return {}
    n = len(vals)
    mean = sum(vals) / n
    variance = sum((v - mean) ** 2 for v in vals) / n if n > 1 else 0
    return {
        "min": min(vals), "max": max(vals),
        "mean": mean, "std": math.sqrt(variance),
        "first": vals[0], "last": vals[-1], "n": n,
    }


def pose_distance(a: tuple, b: tuple) -> float:
    """Euclidean distance between two (x, y, rot) poses (ignores heading)."""
    return math.hypot(a[0] - b[0], a[1] - b[1])


def find_velocity_lags(actual: list[tuple], setpoint: list[tuple], pct: float = VELOCITY_LAG_WARN_PCT, min_dur: float = VELOCITY_LAG_WARN_DUR_S) -> list[tuple]:
    """
    Find sustained periods where actual velocity is >pct% below setpoint.
    Returns list of (start_ts, end_ts, avg_deficit_pct).
    """
    # Build setpoint lookup by timestamp
    sp_map = {ts: v for ts, v in setpoint}
    sp_times = sorted(sp_map.keys())

    def sp_at(t):
        # nearest setpoint
        if not sp_times:
            return None
        idx = min(range(len(sp_times)), key=lambda i: abs(sp_times[i] - t))
        return sp_map[sp_times[idx]]

    in_lag = False
    lag_start = None
    deficits = []
    lags = []

    for ts, v in actual:
        sp = sp_at(ts)
        if sp is None or sp == 0:
            if in_lag and deficits:
                dur = ts - lag_start
                if dur >= min_dur:
                    lags.append((lag_start, ts, sum(deficits) / len(deficits)))
                in_lag = False
                deficits = []
            continue

        deficit = (sp - v) / abs(sp)
        if deficit > pct:
            if not in_lag:
                in_lag = True
                lag_start = ts
                deficits = []
            deficits.append(deficit)
        else:
            if in_lag and deficits:
                dur = ts - lag_start
                if dur >= min_dur:
                    lags.append((lag_start, ts, sum(deficits) / len(deficits)))
            in_lag = False
            deficits = []

    return lags


def enabled_windows(data: dict) -> list[tuple]:
    """
    Return list of (start_ts, end_ts, is_auto) for each enable window.
    """
    enabled = data.get("DriverStation/Enabled", [])
    auto = data.get("DriverStation/Autonomous", [])
    auto_map = {}
    for ts, v in auto:
        auto_map[ts] = v

    def auto_at(t):
        if not auto_map:
            return False
        nearest = min(auto_map.keys(), key=lambda x: abs(x - t))
        return bool(auto_map[nearest])

    windows = []
    start = None
    for ts, v in enabled:
        if v and start is None:
            start = ts
        elif not v and start is not None:
            windows.append((start, ts, auto_at(start + 0.1)))
            start = None
    if start is not None:
        last_ts = enabled[-1][0] if enabled else 0
        windows.append((start, last_ts, auto_at(start + 0.1)))
    return windows


# ─── Investigation report builders ───────────────────────────────────────────

def _fmt(v: float, decimals: int = 2) -> str:
    return f"{v:.{decimals}f}"


def _report_match_state(data: dict, lines: list, log_duration: float = 0.0):
    enabled_series = data.get("DriverStation/Enabled", [])
    auto_series = data.get("DriverStation/Autonomous", [])

    if not enabled_series:
        lines.append("  [no DriverStation/Enabled data found]")
        return

    # Determine initial enabled state and mode
    first_enabled = enabled_series[0][1] if enabled_series else False
    first_auto = auto_series[0][1] if auto_series else False

    if first_enabled and len(enabled_series) == 1:
        # Robot was enabled the entire run (AdvantageKit only logs changes)
        mode = "autonomous" if first_auto else "teleop"
        lines.append(f"  Robot enabled in {mode} mode for entire log ({log_duration:.2f}s)")
        return

    windows = enabled_windows(data)
    if not windows:
        lines.append("  [no enable/disable events found -- robot stayed disabled]")
        return
    for start, end, is_auto in windows:
        mode = "autonomous" if is_auto else "teleop"
        dur = end - start
        if dur < 0.1:
            continue  # skip zero-duration artifacts
        lines.append(f"  T={start:6.2f}  {mode} ENABLED")
        lines.append(f"  T={end:6.2f}  {mode} DISABLED  ({dur:.2f}s)")


def _report_path_tracking(data: dict, lines: list) -> list[str]:
    """Returns anomaly strings for path tracking errors."""
    target = data.get("RealOutputs/Path Planner/Target Pose", [])
    current = data.get("RealOutputs/Path Planner/Current Pose", [])
    anomalies = []

    if not target or not current:
        lines.append("  [no path tracking data -- Path Planner/Target Pose or Current Pose not logged]")
        return anomalies

    # Build current pose lookup
    cur_map = {}
    for ts, pose in current:
        if pose:
            cur_map[ts] = pose
    cur_times = sorted(cur_map.keys())

    def cur_at(t):
        if not cur_times:
            return None
        idx = min(range(len(cur_times)), key=lambda i: abs(cur_times[i] - t))
        return cur_map[cur_times[idx]]

    errors = []
    for ts, tgt in target:
        if not tgt:
            continue
        cur = cur_at(ts)
        if not cur:
            continue
        err = pose_distance(tgt, cur)
        errors.append((ts, err))

    if not errors:
        lines.append("  [could not compute path error -- pose data may be wrong type]")
        return anomalies

    avg_err = sum(e for _, e in errors) / len(errors)
    max_ts, max_err = max(errors, key=lambda x: x[1])
    lines.append(f"  Overall: avg {_fmt(avg_err)}m  max {_fmt(max_err)}m @ T={max_ts:.2f}s")

    if max_err > PATH_ERROR_WARN_M:
        msg = f"Path tracking error {_fmt(max_err)}m exceeded {PATH_ERROR_WARN_M}m threshold at T={max_ts:.2f}s"
        lines.append(f"  <- OVER THRESHOLD")
        anomalies.append(f"WARN: {msg}")

    return anomalies


def _report_state_changes(data: dict, controller_keys: list[str], lines: list):
    events = []
    for key in controller_keys:
        series = data.get(key)
        if not series:
            continue
        parts = key.split("/")
        # Use last two meaningful segments as label (e.g. "Intake Rack/Target")
        label = "/".join(parts[-2:]) if len(parts) >= 2 else key
        for ts, old, new in transitions(series):
            if old is None:
                continue
            events.append((ts, label, old, new))
    events.sort(key=lambda x: x[0])
    if not events:
        lines.append("  [no state changes recorded]")
        return
    for ts, label, old, new in events:
        lines.append(f"  T={ts:6.2f}  {label:<30} {old} -> {new}")


def _report_continuous(data: dict, pairs: list[tuple[str, str | None]], lines: list) -> list[str]:
    """
    pairs: list of (actual_key, setpoint_key_or_None)
    Returns anomaly strings.
    """
    anomalies = []
    for actual_key, sp_key in pairs:
        series = data.get(actual_key)
        if not series:
            continue
        s = num_stats(series)
        if not s:
            continue
        short = actual_key.split("/", 1)[-1]
        sp_str = ""
        if sp_key and sp_key in data:
            sp_s = num_stats(data[sp_key])
            if sp_s:
                sp_str = f"  setpoint mean: {_fmt(sp_s['mean'])}"
        lines.append(
            f"  {short:<48} "
            f"min:{_fmt(s['min']):>8}  max:{_fmt(s['max']):>8}  "
            f"mean:{_fmt(s['mean']):>8}  std:{_fmt(s['std']):>6}"
            + sp_str
        )
        if sp_key and sp_key in data:
            lags = find_velocity_lags(series, data[sp_key])
            for lag_start, lag_end, deficit in lags:
                msg = f"{short} {deficit*100:.0f}% below setpoint at T={lag_start:.2f}s for {lag_end-lag_start:.2f}s"
                anomalies.append(f"WARN: {msg}")

    return anomalies


def _report_game_pieces(data: dict, lines: list):
    fuel = data.get("RealOutputs/Field Simulation/Fuel Count")
    if not fuel:
        lines.append("  [no sim game piece data -- not running in SIM mode, or not logged]")
        return
    first_val = fuel[0][1]
    last_val = fuel[-1][1]
    delta = last_val - first_val
    lines.append(f"  FuelCount: {first_val} -> {last_val}  (net {'+' if delta >= 0 else ''}{delta})")

    # Find intake bursts: group consecutive pickups within 2s windows
    burst_start = None
    burst_total = 0
    bursts = []
    prev_ts, prev_v = None, None
    for ts, v in fuel:
        if prev_v is not None and v > prev_v:
            if burst_start is None or ts - prev_ts > 2.0:
                if burst_start is not None:
                    bursts.append((burst_start, prev_ts, burst_total))
                burst_start = ts
                burst_total = v - prev_v
            else:
                burst_total += v - prev_v
        elif prev_v is not None and v < prev_v and burst_start is not None:
            bursts.append((burst_start, ts, burst_total))
            burst_start = None
            burst_total = 0
        prev_ts, prev_v = ts, v
    if burst_start is not None and burst_total > 0:
        bursts.append((burst_start, prev_ts, burst_total))

    if bursts:
        burst_strs = [f"T~{s:.1f}s (+{tot})" for s, e, tot in bursts]
        lines.append(f"  Intake bursts: {', '.join(burst_strs)}")


# ─── Top-level report dispatch ────────────────────────────────────────────────

def _log_duration(data: dict) -> float:
    """Estimate log duration from any available high-frequency key."""
    best = 0.0
    for series in data.values():
        if len(series) > 50:
            times = [ts for ts, _ in series]
            best = max(best, max(times) - min(times))
    return best


def report_auto(data: dict) -> str:
    out = []
    anomalies = []
    dur = _log_duration(data)

    out.append("MATCH STATE")
    _report_match_state(data, out, dur)
    out.append("")

    out.append("PATH TRACKING")
    anomalies += _report_path_tracking(data, out)
    out.append("")

    out.append("STATE CHANGES")
    _report_state_changes(
        data,
        [
            "RealOutputs/Intake/Intake Rack/Target",
            "RealOutputs/Intake/Intake Rollers/Target",
            "RealOutputs/Shooter/Target State",
        ],
        out,
    )
    out.append("")

    out.append("CONTINUOUS CHANNELS")
    anomalies += _report_continuous(
        data,
        [("RealOutputs/Shooter/Shooter Flywheels/Current Velocity", "RealOutputs/Shooter/Shooter Flywheels/Target Velocity")],
        out,
    )
    out.append("")

    out.append("GAME PIECES (SIM)")
    _report_game_pieces(data, out)
    out.append("")

    if anomalies:
        out.append("ANOMALIES")
        out.extend(f"  {a}" for a in anomalies)
    else:
        out.append("ANOMALIES")
        out.append("  None detected.")

    return "\n".join(out)


def report_shooter(data: dict) -> str:
    out = []
    anomalies = []
    dur = _log_duration(data)

    out.append("MATCH STATE")
    _report_match_state(data, out, dur)
    out.append("")

    out.append("STATE CHANGES")
    _report_state_changes(data, ["RealOutputs/Shooter/Target State"], out)
    out.append("")

    out.append("CONTINUOUS CHANNELS")
    anomalies += _report_continuous(
        data,
        [
            ("RealOutputs/Shooter/Shooter Flywheels/Current Velocity", "RealOutputs/Shooter/Shooter Flywheels/Target Velocity"),
            ("RealOutputs/Shooter/Shooter Hood/Target Position Manual", None),
            ("RealOutputs/Shooter/Shooter Accelerator/Target Velocity", None),
        ],
        out,
    )
    out.append("")

    if anomalies:
        out.append("ANOMALIES")
        out.extend(f"  {a}" for a in anomalies)
    else:
        out.append("ANOMALIES")
        out.append("  None detected.")

    return "\n".join(out)


def report_drive(data: dict) -> str:
    out = []
    anomalies = []
    dur = _log_duration(data)

    out.append("MATCH STATE")
    _report_match_state(data, out, dur)
    out.append("")

    out.append("PATH TRACKING")
    anomalies += _report_path_tracking(data, out)
    out.append("")

    out.append("CONTINUOUS CHANNELS")
    anomalies += _report_continuous(
        data,
        [("RealOutputs/Swerve/Distance From Setpoint", None)],
        out,
    )
    out.append("")

    if anomalies:
        out.append("ANOMALIES")
        out.extend(f"  {a}" for a in anomalies)
    else:
        out.append("ANOMALIES")
        out.append("  None detected.")

    return "\n".join(out)


def report_intake(data: dict) -> str:
    out = []
    anomalies = []
    dur = _log_duration(data)

    out.append("MATCH STATE")
    _report_match_state(data, out, dur)
    out.append("")

    out.append("STATE CHANGES")
    _report_state_changes(
        data,
        [
            "RealOutputs/Intake/Intake Rack/Target",
            "RealOutputs/Intake/Intake Rollers/Target",
            "RealOutputs/Shooter/Target State",
        ],
        out,
    )
    out.append("")

    out.append("CONTINUOUS CHANNELS")
    anomalies += _report_continuous(
        data,
        [
            ("RealOutputs/Intake/Intake Rack/Position Target Rotations", None),
            ("RealOutputs/Intake/Intake Rollers/Target Velocity", None),
        ],
        out,
    )
    out.append("")

    out.append("GAME PIECES (SIM)")
    _report_game_pieces(data, out)
    out.append("")

    if anomalies:
        out.append("ANOMALIES")
        out.extend(f"  {a}" for a in anomalies)
    else:
        out.append("ANOMALIES")
        out.append("  None detected.")

    return "\n".join(out)


def report_vision(data: dict) -> str:
    out = []
    dur = _log_duration(data)

    out.append("MATCH STATE")
    _report_match_state(data, out, dur)
    out.append("")

    out.append("CAMERA PRESENCE")
    key = "RealOutputs/Vision/Camera0/Average Distance"
    series = data.get(key, [])
    if not series:
        out.append("  Camera0: [no accepted poses logged]")
    else:
        s = num_stats(series)
        out.append(f"  Camera0: {s['n']} accepted pose readings, avg distance {_fmt(s['mean'])}m")

    multi = data.get("RealOutputs/Vision/GetMultiTags", [])
    if multi:
        active = sum(1 for _, v in multi if v)
        out.append(f"  Multi-tag: active {active}/{len(multi)} readings")
    out.append("")

    return "\n".join(out)


REPORT_FNS = {
    "auto": report_auto,
    "shooter": report_shooter,
    "drive": report_drive,
    "intake": report_intake,
    "vision": report_vision,
}


# ─── Direct key query ────────────────────────────────────────────────────────

SAMPLE_INTERVAL_S  = 0.5    # one sample per this many seconds for high-frequency keys
SPARSE_THRESHOLD   = 40     # below this many records, show every value instead of sampling
PLATEAU_EPS_FRAC   = 0.005  # consecutive values within 0.5% of range are considered a plateau


def _sample_series(series: list[tuple], interval: float = SAMPLE_INTERVAL_S) -> list[tuple]:
    """Down-sample a high-frequency numeric series to one value per interval."""
    if not series:
        return []
    result = []
    next_bucket = series[0][0]
    for ts, v in series:
        if ts >= next_bucket:
            result.append((ts, v))
            next_bucket = ts + interval
    return result


def _compress_numeric(series: list[tuple]) -> list[str]:
    """
    Render a numeric series compactly:
    - Sample to SAMPLE_INTERVAL_S if long
    - Collapse plateau runs (consecutive near-equal values) into one summary line
    Returns list of formatted strings ready to print.
    """
    if not series:
        return []

    rows = series if len(series) <= SPARSE_THRESHOLD else _sample_series(series)

    # Compute plateau epsilon from value range
    vals = [v for _, v in rows if isinstance(v, (int, float))]
    if not vals:
        return [f"    T={ts:7.2f}s  {_fmt(v, 4)}" for ts, v in rows]

    v_range = max(vals) - min(vals)
    eps = v_range * PLATEAU_EPS_FRAC if v_range > 0 else 0.0001

    out = []
    i = 0
    while i < len(rows):
        ts, v = rows[i]
        if not isinstance(v, (int, float)):
            out.append(f"    T={ts:7.2f}s  {v}")
            i += 1
            continue

        # Count how many consecutive rows are within eps of v
        j = i + 1
        while j < len(rows) and abs(rows[j][1] - v) <= eps:
            j += 1

        run = j - i
        if run >= 3:
            out.append(f"    T={ts:7.2f}s  {_fmt(v, 4)}  (steady through T={rows[j-1][0]:.2f}s, {run} samples)")
            i = j
        else:
            out.append(f"    T={ts:7.2f}s  {_fmt(v, 4)}")
            i += 1

    return out


def query_keys(
    path: str,
    key_names: list[str],
    time_start: float | None = None,
    time_end: float | None = None,
) -> str:
    """
    Structured query for specific keys with optional time window.
    Returns a compact, token-efficient report.

    For numeric keys: stats + sampled values (1 per 0.5s) or all values if sparse.
    For string/boolean keys: all transitions.
    For struct keys: first/last/count.
    """
    data = read_log(path, keys=key_names, time_start=time_start, time_end=time_end)

    window_str = ""
    if time_start is not None or time_end is not None:
        t0 = f"{time_start:.2f}s" if time_start is not None else "start"
        t1 = f"{time_end:.2f}s" if time_end is not None else "end"
        window_str = f"  |  window: {t0} to {t1}"

    out = [f"=== KEY QUERY ===" ]
    out.append(f"File: {Path(path).name}{window_str}\n")

    for key in key_names:
        series = data.get(key)
        if not series:
            out.append(f"{key}")
            out.append(f"  [no data in range]\n")
            continue

        # Detect dtype from first value
        first_val = series[0][1]
        if isinstance(first_val, bool):
            klass = "discrete"
        elif isinstance(first_val, (int, float)):
            klass = "numeric"
        elif isinstance(first_val, str):
            klass = "discrete"
        else:
            klass = "struct"

        ts_first = series[0][0]
        ts_last = series[-1][0]
        n = len(series)

        out.append(f"{key}")
        out.append(f"  {n} records  [{ts_first:.2f}s - {ts_last:.2f}s]")

        if klass == "numeric":
            s = num_stats(series)
            if s:
                out.append(
                    f"  stats:  min={_fmt(s['min'])}  max={_fmt(s['max'])}  "
                    f"mean={_fmt(s['mean'])}  std={_fmt(s['std'])}"
                )
            label = "values" if n <= SPARSE_THRESHOLD else f"sampled (1 per {SAMPLE_INTERVAL_S}s, plateaus collapsed)"
            out.append(f"  {label}:")
            out.extend(_compress_numeric(series))

        elif klass == "discrete":
            trns = transitions(series)
            if not trns:
                out.append(f"  value: {first_val}  (no changes in range)")
            else:
                out.append(f"  transitions:")
                for ts, old, new in trns:
                    if old is None:
                        out.append(f"    T={ts:7.2f}s  (initial) {new}")
                    else:
                        out.append(f"    T={ts:7.2f}s  {old} -> {new}")

        else:  # struct
            out.append(f"  first: {first_val}")
            if n > 1:
                out.append(f"  last:  {series[-1][1]}")

        out.append("")

    return "\n".join(out)


# ─── Summary (unchanged behaviour, slightly tighter output) ──────────────────

def print_summary(path: str, prefix: str | None = None):
    reader = _load_reader(path)
    entry_map = {}
    key_counts: dict[str, int] = defaultdict(int)
    key_types: dict[str, str] = {}
    key_first: dict[str, float] = {}
    key_last: dict[str, float] = {}
    min_ts = float("inf")
    max_ts = float("-inf")

    for record in reader:
        if record.isControl():
            if record.isStart():
                s = record.getStartData()
                entry_map[s.entry] = (s.name, s.type)
            continue
        entry_id = record.getEntry()
        if entry_id not in entry_map:
            continue
        name, dtype = entry_map[entry_id]
        name = name.lstrip("/")
        if prefix and not name.startswith(prefix):
            continue
        ts = record.getTimestamp() / 1_000_000.0
        min_ts = min(min_ts, ts)
        max_ts = max(max_ts, ts)
        key_counts[name] += 1
        key_types[name] = dtype
        if name not in key_first:
            key_first[name] = ts
        key_last[name] = ts

    duration = max_ts - min_ts if min_ts != float("inf") else 0.0
    print(f"Duration: {duration:.3f}s  ({min_ts:.3f}s to {max_ts:.3f}s)  |  {len(key_counts)} keys\n")

    groups: dict[str, list] = defaultdict(list)
    for key in sorted(key_counts):
        groups[key.split("/")[0] + "/"].append(key)

    for group, keys in sorted(groups.items()):
        total = sum(key_counts[k] for k in keys)
        print(f"  {group}  ({total} records, {len(keys)} keys)")
        for key in keys:
            dtype = key_types.get(key, "?")
            print(
                f"    {key:<56} {dtype:<16} "
                f"{key_counts[key]:>6} records  "
                f"[{key_first[key]:.2f}s-{key_last[key]:.2f}s]"
            )
    print()


# ─── Raw CSV (deep-dive escape hatch) ────────────────────────────────────────

def print_csv(path: str, prefix: str, out_path: str | None = None):
    import csv as csv_mod
    data = read_log(path, prefix=prefix)
    records = []
    for key, series in sorted(data.items()):
        for ts, val in series:
            records.append((ts, key, str(val)))
    records.sort(key=lambda r: r[0])

    out = open(out_path, "w", newline="", encoding="utf-8") if out_path else sys.stdout
    try:
        w = csv_mod.writer(out)
        w.writerow(["timestamp_sec", "key", "value"])
        for ts, key, val in records:
            w.writerow([f"{ts:.6f}", key, val])
    finally:
        if out_path:
            out.close()
            print(f"Wrote {len(records)} records to {out_path}", file=sys.stderr)


# ─── CLI ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Analyze WPILib .wpilog files. Use --investigate for AI-friendly reports."
    )
    parser.add_argument("wpilog", help="Path to .wpilog file")
    parser.add_argument(
        "--investigate",
        choices=REPORT_FNS.keys(),
        help="Generate structured analysis report (low token cost -- use this by default)",
    )
    parser.add_argument(
        "--keys",
        help=(
            "Comma-separated exact key names to query (supports --from/--to). "
            "Example: --keys 'RealOutputs/Shooter/Target State,RealOutputs/Shooter/Shooter Flywheels/Current Velocity'"
        ),
    )
    parser.add_argument(
        "--from", dest="time_start", type=float, metavar="T",
        help="Start of time window in seconds (applies to --keys and --investigate)",
    )
    parser.add_argument(
        "--to", dest="time_end", type=float, metavar="T",
        help="End of time window in seconds (applies to --keys and --investigate)",
    )
    parser.add_argument(
        "--summary",
        action="store_true",
        help="List all keys with record counts and time ranges",
    )
    parser.add_argument(
        "--prefix",
        help="Dump raw CSV for keys starting with PREFIX (deep dive only -- high token cost)",
    )
    parser.add_argument("--out", help="Write output to FILE instead of stdout")
    args = parser.parse_args()

    log_path = Path(args.wpilog)
    if not log_path.exists():
        print(f"ERROR: file not found: {log_path}", file=sys.stderr)
        sys.exit(1)

    if args.summary:
        print_summary(str(log_path), prefix=args.prefix)
        return

    if args.keys:
        key_list = [k.strip() for k in args.keys.split(",") if k.strip()]
        report = query_keys(str(log_path), key_list, args.time_start, args.time_end)
        if args.out:
            Path(args.out).write_text(report, encoding="utf-8")
            print(f"Report written to {args.out}", file=sys.stderr)
        else:
            print(report)
        return

    if args.investigate:
        keys = INVESTIGATE_KEYS[args.investigate]
        data = read_log(str(log_path), keys=keys, time_start=args.time_start, time_end=args.time_end)
        report = f"=== {args.investigate.upper()} INVESTIGATION ===\n"
        report += f"File: {log_path.name}\n\n"
        report += REPORT_FNS[args.investigate](data)
        if args.out:
            Path(args.out).write_text(report, encoding="utf-8")
            print(f"Report written to {args.out}", file=sys.stderr)
        else:
            print(report)
        return

    if args.prefix:
        print_csv(str(log_path), prefix=args.prefix, out_path=args.out)
        return

    parser.print_help()
    print(
        "\nTip: start with --summary to see what's in the log, "
        "then --investigate auto, then --keys for targeted follow-up.",
        file=sys.stderr,
    )


if __name__ == "__main__":
    main()
