# Log-Driven Simulation Fidelity Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replay a real match's driver inputs through the simulation and compare the resulting log against the real one, so power draw, brownout timing, and mechanism curves can be tuned to match reality.

**Architecture:** A SIM-only `LogInputPlayer` reads a real `.wpilog` with WPILib's `DataLogReader` and injects the logged joystick/DS state into `DriverStationSim` frame-locked to the robot loop. A new `SimBattery` closes the missing voltage-sag feedback loop so motors lose torque under load. A new `scripts/log_compare.py` scores sim output against the real log and ranks divergences.

**Tech Stack:** Java 17, WPILib 2026.2.1, AdvantageKit, Phoenix 6, maple-sim (IronMaple), Gradle, JUnit 5, Python 3 + pytest.

**Spec:** `docs/superpowers/specs/2026-09-08-log-driven-sim-fidelity-design.md`

---

## Context An Engineer Needs Before Starting

### Build commands

```bash
./gradlew compileJava --no-daemon      # fast compile check
./gradlew test --no-daemon             # JUnit tests
./gradlew spotlessApply --no-daemon    # REQUIRED before every commit
```

**Always pass `--no-daemon`.** Without it the Gradle wrapper hangs after "BUILD SUCCESSFUL" on Windows. Spotless (Google Java Format) runs on `build`; the build fails on a formatting diff, so run `spotlessApply` before committing every time.

### Real logs (read-only, outside the repo)

```
C:\Users\bruce\Downloads\LOGS-2026-main\LOGS-2026-main\Worlds\
```

Do not commit these files — they are 34-80 MB each. Wherever a task writes `<PRIMARY_LOG>` or another placeholder, substitute the full path from this table:

| Placeholder | Role | Filename (in the folder above) |
| --- | --- | --- |
| `<PRIMARY_LOG>` | tune | `akit_26-04-30_14-50-56_johnson_q54.wpilog` |
| `<VALIDATION_LOG_1>` | validate, browning | `akit_26-04-30_08-04-41_johnson_q14.wpilog` |
| `<VALIDATION_LOG_2>` | validate, clean | `akit_26-05-01_11-41-50_johnson_q93.wpilog` |
| `<HELDOUT_LOG>` | held out | `akit_26-05-01_13-08-14_johnson_q103.wpilog` |
| `<STRESS_LOG>` | brownout stress | `akit_26-05-01_06-05-03_johnson_q64.wpilog` |

`<NEWEST>` means the most recent file in `build/ai-logs/`:

```bash
ls -t build/ai-logs/*.wpilog | head -1
```

### Facts established by investigation (do not re-derive)

- `edu.wpi.first.util.datalog.DataLogReader` and `DataLogWriter` are both present in wpiutil 2026.2.1. `DataLogReader` implements `Iterable<DataLogRecord>`.
- In the real logs the match window is bounded by `DriverStation/Enabled` transitions. In `q3` these are at 61.33 s (auto start) and 226.68 s (match end); every log has leading pre-match idle, so **timestamps must be rebased on first-enable**, never assumed to start at zero.
- `DriverStation/Joystick0/ButtonValues` is a **single `int64` bitfield**, not an array. Button N (1-indexed, as WPILib numbers them) is bit `N-1`.
- `DriverStation/Joystick0/AxisValues` is `float[]`. `POVs` is `int64[]`.
- **Every sim IO already calls `talon.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage())`**, and so does `PhoenixUtil.TalonFXMotorControllerSim` for the swerve modules. This means `RoboRioSim.setVInVoltage(v)` propagates to the whole robot automatically — `SimBattery` does not need to touch any motor directly.

### Defects found during investigation that this plan fixes

These are real, confirmed by reading the code. They are the reason power draw cannot currently match:

| File | Line | Defect |
| --- | --- | --- |
| `IntakeRackIOSim.java` | 76 | `inputs.supplyCurrentAmps = 1.0; // Not simulated` — contributes a constant fake 1 A |
| `SerializerSim.java` | 55 | `inputs.supplyCurrentAmps = 1.0; // Not simulated` — same |
| `IntakeRollersIOSim.java` | 69 | `Math.max(-12, Math.min(12, ...))` — hardcoded 12 V clamp, so sag cannot reduce torque |
| `ShooterAcceleratorIOSim.java` | 70 | same hardcoded clamp |
| `ShooterFlywheelIOSim.java` | 69 | same hardcoded clamp |
| `ShooterOmniwheelIOSim.java` | 70 | same hardcoded clamp |
| 4 roller IOSims | — | assign `getCurrentDrawAmps()` (stator current) to `inputs.supplyCurrentAmps`, overstating supply draw |

---

## File Structure

### Created

| Path | Responsibility |
| --- | --- |
| `src/main/java/frc/robot/utility/replay/LogTimeline.java` | Generic zero-order-hold time series with binary-search lookup. Pure, no HAL. |
| `src/main/java/frc/robot/utility/replay/MatchInputs.java` | Immutable record holding every timeline parsed from a match log. |
| `src/main/java/frc/robot/utility/replay/MatchLogReader.java` | Parses a `.wpilog` into `MatchInputs`. Pure, no HAL. |
| `src/main/java/frc/robot/utility/replay/LogInputPlayer.java` | Frame-locked injection of `MatchInputs` into `DriverStationSim` / `GenericHIDSim`. |
| `src/main/java/frc/robot/utility/replay/PoseAnchor.java` | Teleop pose re-anchoring and `Replay/Anchor Error` logging. |
| `src/main/java/frc/robot/utility/SimBattery.java` | Battery voltage sag model and current-source registry. |
| `scripts/log_compare.py` | Comparison engine: alignment, resampling, divergence scoring, event diff, JSON scores. |
| `scripts/tests/test_log_compare.py` | pytest coverage for the comparison engine. |
| `src/test/java/frc/robot/utility/replay/LogTimelineTest.java` | Unit tests for zero-order-hold semantics. |
| `src/test/java/frc/robot/utility/replay/MatchLogReaderTest.java` | Round-trip tests against a synthetic `.wpilog` written by `DataLogWriter`. |
| `src/test/java/frc/robot/utility/SimBatteryTest.java` | Unit tests for sag, recovery, and brownout. |

### Modified

| Path | Change |
| --- | --- |
| `src/main/java/frc/robot/Robot.java` | Construct and tick `LogInputPlayer`; honour `-Preplay.fast`. |
| `src/main/java/frc/robot/RobotContainer.java` | Tick `SimBattery` and `PoseAnchor` in `updateSimulation()`; resolve auto name from the replay log. |
| `build.gradle` | Wire the new `-Preplay.*` properties to system properties. |
| `scripts/wpilog_to_csv.py` | Add `--compare` argument that delegates to `log_compare.py`. |
| 6 × `*IOSim.java` | Fix voltage clamps, fake supply currents, and register with `SimBattery`. |
| `src/main/java/frc/robot/subsystems/swerve/ModuleIOTalonFXSim.java` | Register drive and steer supply current with `SimBattery`. |
| `.claude/commands/simulation-agent.md` | Document Log-Driven Replay mode. |
| `.claude/commands/log-analysis.md` | Document Comparison mode. |

**Why `log_compare.py` is a separate module:** `wpilog_to_csv.py` is already ~1000 lines. The comparison engine is a distinct responsibility with its own tests; bolting it on would push a single file past 1400 lines. `wpilog_to_csv.py` keeps the CLI and imports the engine.

---

## Phase A — Comparison Tooling

This phase is pure Python and needs no simulation runs, so it is fast to iterate on and lands first.

### Task 1: `LogSeries` resampling and alignment

**Files:**
- Create: `scripts/log_compare.py`
- Create: `scripts/tests/test_log_compare.py`

- [ ] **Step 1: Write the failing test**

Create `scripts/tests/test_log_compare.py`:

```python
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from log_compare import resample, find_first_enable, GRID_DT


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
```

- [ ] **Step 2: Run test to verify it fails**

```bash
python -m pytest scripts/tests/test_log_compare.py -v
```

Expected: FAIL with `ModuleNotFoundError: No module named 'log_compare'`

- [ ] **Step 3: Write minimal implementation**

Create `scripts/log_compare.py`:

```python
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
```

- [ ] **Step 4: Run test to verify it passes**

```bash
python -m pytest scripts/tests/test_log_compare.py -v
```

Expected: 6 passed

- [ ] **Step 5: Commit**

```bash
git add scripts/log_compare.py scripts/tests/test_log_compare.py
git commit -m "Add log comparison resampling and alignment primitives"
```

---

### Task 2: Divergence scoring

**Files:**
- Modify: `scripts/log_compare.py`
- Modify: `scripts/tests/test_log_compare.py`

- [ ] **Step 1: Write the failing test**

Append to `scripts/tests/test_log_compare.py`:

```python
from log_compare import score_pair, DivergenceScore


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
```

- [ ] **Step 2: Run test to verify it fails**

```bash
python -m pytest scripts/tests/test_log_compare.py -v
```

Expected: FAIL with `ImportError: cannot import name 'score_pair'`

- [ ] **Step 3: Write minimal implementation**

Append to `scripts/log_compare.py`:

```python
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
```

- [ ] **Step 4: Run test to verify it passes**

```bash
python -m pytest scripts/tests/test_log_compare.py -v
```

Expected: 12 passed

- [ ] **Step 5: Commit**

```bash
git add scripts/log_compare.py scripts/tests/test_log_compare.py
git commit -m "Add divergence scoring for sim-vs-real signal comparison"
```

---

### Task 3: Worst-window location

**Files:**
- Modify: `scripts/log_compare.py`
- Modify: `scripts/tests/test_log_compare.py`

- [ ] **Step 1: Write the failing test**

Append to `scripts/tests/test_log_compare.py`:

```python
from log_compare import worst_windows


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
    for (s1, e1, _), (s2, _, _) in zip(windows, windows[1:]):
        assert e1 <= s2 or s2 >= e1  # no window starts inside the previous one
```

- [ ] **Step 2: Run test to verify it fails**

```bash
python -m pytest scripts/tests/test_log_compare.py -v
```

Expected: FAIL with `ImportError: cannot import name 'worst_windows'`

- [ ] **Step 3: Write minimal implementation**

Append to `scripts/log_compare.py`:

```python
def worst_windows(sim, real, grid, window_s=2.0, top=3):
    """
    Find the `top` non-overlapping time windows where sim diverges most.

    Returns [(start_s, end_s, mean_abs_error), ...] worst first. Windows are
    non-overlapping so the report shows distinct problem areas rather than
    three views of the same spike.
    """
    if not grid:
        return []
    span = max(1, int(round(window_s / GRID_DT)))
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
```

- [ ] **Step 4: Run test to verify it passes**

```bash
python -m pytest scripts/tests/test_log_compare.py -v
```

Expected: 15 passed

- [ ] **Step 5: Commit**

```bash
git add scripts/log_compare.py scripts/tests/test_log_compare.py
git commit -m "Locate worst-diverging time windows per signal"
```

---

### Task 4: Event-aligned diff

**Files:**
- Modify: `scripts/log_compare.py`
- Modify: `scripts/tests/test_log_compare.py`

- [ ] **Step 1: Write the failing test**

Append to `scripts/tests/test_log_compare.py`:

```python
from log_compare import extract_events, pair_events


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
    assert pairs[1] == ("INTAKE", 2.4, 2.0, pytest.approx(0.4))


def test_pair_events_stops_at_first_sequence_mismatch():
    real = [(1.0, "STOW"), (2.0, "INTAKE"), (3.0, "EJECT")]
    sim = [(1.0, "STOW"), (2.0, "REVERSE")]
    pairs = pair_events(sim, real)
    # Only the common prefix is comparable; after divergence the runs are
    # doing different things and further pairing would be meaningless.
    assert len(pairs) == 1
    assert pairs[0][0] == "STOW"
```

Add `import pytest` to the top of the test file if it is not already there.

- [ ] **Step 2: Run test to verify it fails**

```bash
python -m pytest scripts/tests/test_log_compare.py -v
```

Expected: FAIL with `ImportError: cannot import name 'extract_events'`

- [ ] **Step 3: Write minimal implementation**

Append to `scripts/log_compare.py`:

```python
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
```

- [ ] **Step 4: Run test to verify it passes**

```bash
python -m pytest scripts/tests/test_log_compare.py -v
```

Expected: 19 passed

- [ ] **Step 5: Commit**

```bash
git add scripts/log_compare.py scripts/tests/test_log_compare.py
git commit -m "Add event-aligned diff for state machine timing comparison"
```

---

### Task 5: Fit-score categories and JSON output

**Files:**
- Modify: `scripts/log_compare.py`
- Modify: `scripts/tests/test_log_compare.py`

- [ ] **Step 1: Write the failing test**

Append to `scripts/tests/test_log_compare.py`:

```python
from log_compare import categorize, CATEGORY_MECHANISMS, CATEGORY_CURRENTS
from log_compare import CATEGORY_VOLTAGE, CATEGORY_AGGREGATE, CATEGORY_OTHER


def test_categorize_currents():
    assert categorize("Swerve/Module0/DriveStatorCurrent") == CATEGORY_CURRENTS
    assert categorize("Intake/Intake Rollers/SupplyCurrentAmps") == CATEGORY_CURRENTS


def test_categorize_voltage():
    assert categorize("SystemStats/BatteryVoltage") == CATEGORY_VOLTAGE


def test_categorize_mechanisms():
    assert categorize("Intake/Intake Rack/PositionRotations") == CATEGORY_MECHANISMS
    assert categorize(
        "RealOutputs/Shooter/Shooter Flywheels/Current Velocity"
    ) == CATEGORY_MECHANISMS


def test_categorize_aggregate():
    assert categorize(
        "RealOutputs/MotorOutputManager/TotalAmps"
    ) == CATEGORY_AGGREGATE


def test_categorize_falls_back_to_other():
    assert categorize("RealOutputs/Console") == CATEGORY_OTHER


def test_applied_volts_is_not_miscategorized_as_battery_voltage():
    # AppliedVolts is a per-motor output, not the battery rail.
    assert categorize("Intake/Intake Rack/AppliedVolts") != CATEGORY_VOLTAGE
```

- [ ] **Step 2: Run test to verify it fails**

```bash
python -m pytest scripts/tests/test_log_compare.py -v
```

Expected: FAIL with `ImportError: cannot import name 'categorize'`

- [ ] **Step 3: Write minimal implementation**

Append to `scripts/log_compare.py`:

```python
CATEGORY_MECHANISMS = "mechanisms"
CATEGORY_CURRENTS = "currents"
CATEGORY_VOLTAGE = "voltage"
CATEGORY_AGGREGATE = "aggregate"
CATEGORY_OTHER = "other"


def categorize(key):
    """Bucket a log key into a fidelity category. Order matters: the most
    specific patterns are checked first."""
    if key.startswith("RealOutputs/MotorOutputManager/"):
        return CATEGORY_AGGREGATE
    if key in ("SystemStats/BatteryVoltage", "SystemStats/BatteryCurrent"):
        return CATEGORY_VOLTAGE
    if "Current" in key or "Amps" in key:
        return CATEGORY_CURRENTS
    if any(
        token in key
        for token in ("Position", "Velocity", "PositionRads", "PositionRotations")
    ):
        return CATEGORY_MECHANISMS
    return CATEGORY_OTHER
```

- [ ] **Step 4: Run test to verify it passes**

```bash
python -m pytest scripts/tests/test_log_compare.py -v
```

Expected: 25 passed

- [ ] **Step 5: Commit**

```bash
git add scripts/log_compare.py scripts/tests/test_log_compare.py
git commit -m "Bucket log keys into fidelity score categories"
```

---

### Task 6: `--compare` CLI wiring

**Files:**
- Modify: `scripts/log_compare.py`
- Modify: `scripts/wpilog_to_csv.py:913-993` (the `main()` function)

- [ ] **Step 1: Add the report driver to `log_compare.py`**

Append to `scripts/log_compare.py`:

```python
import json


def compare_logs(sim_path, real_path, top=25, json_out=None):
    """
    Compare two logs and return the report as a string.

    Imported lazily so log_compare stays unit-testable without the reader.
    """
    from wpilog_to_csv import read_log

    sim_data = read_log(sim_path)
    real_data = read_log(real_path)

    sim_t0 = find_first_enable(sim_data)
    real_t0 = find_first_enable(real_data)
    if sim_t0 is None or real_t0 is None:
        return "ERROR: could not find DriverStation/Enabled -> True in both logs."

    lines = [
        "=== LOG COMPARISON ===",
        f"sim:  {sim_path}",
        f"real: {real_path}",
        f"aligned on first-enable: sim t0={sim_t0:.2f}s, real t0={real_t0:.2f}s",
        "",
    ]

    shared = sorted(set(sim_data) & set(real_data))
    duration = min(
        max((ts for s in sim_data.values() for ts, _ in s), default=sim_t0) - sim_t0,
        max((ts for s in real_data.values() for ts, _ in s), default=real_t0) - real_t0,
    )
    grid = [i * GRID_DT for i in range(int(duration / GRID_DT))]

    scored, category_totals = [], {}
    for key in shared:
        sim_series = [(ts - sim_t0, v) for ts, v in sim_data[key]]
        real_series = [(ts - real_t0, v) for ts, v in real_data[key]]
        if not sim_series or not isinstance(sim_series[0][1], (int, float)):
            continue
        if isinstance(sim_series[0][1], bool):
            continue

        sim_r = resample(sim_series, grid)
        real_r = resample(real_series, grid)
        score = score_pair(sim_r, real_r)
        if score is None:
            continue
        scored.append((key, score, sim_r, real_r))
        category_totals.setdefault(categorize(key), []).append(score.nrmse)

    scored.sort(key=lambda row: -row[1].nrmse)

    lines.append(f"--- TOP {top} DIVERGING SIGNALS (of {len(scored)} scored) ---")
    for key, score, sim_r, real_r in scored[:top]:
        lines.append(f"\n{key}   [{categorize(key)}]")
        lines.append(
            f"  nrmse={score.nrmse:.3f}  mean_shift={score.mean_shift:+.3f}  "
            f"p95_shift={score.p95_shift:+.3f}  peak_shift={score.peak_shift:+.3f}  "
            f"corr={score.correlation:.3f}  n={score.samples}"
        )
        for start, end, err in worst_windows(sim_r, real_r, grid):
            lines.append(f"    worst {start:7.2f}s-{end:7.2f}s  mean_abs_err={err:.3f}")

    lines.append("\n--- FIT SCORE BY CATEGORY (mean nrmse, lower is better) ---")
    fit = {}
    for category, values in sorted(category_totals.items()):
        fit[category] = sum(values) / len(values)
        lines.append(f"  {category:<12} {fit[category]:.4f}   ({len(values)} signals)")

    if json_out:
        with open(json_out, "w") as handle:
            json.dump(
                {
                    "sim": sim_path,
                    "real": real_path,
                    "categories": fit,
                    "signals": {
                        key: {
                            "nrmse": s.nrmse,
                            "mean_shift": s.mean_shift,
                            "p95_shift": s.p95_shift,
                            "peak_shift": s.peak_shift,
                            "correlation": s.correlation,
                        }
                        for key, s, _, _ in scored
                    },
                },
                handle,
                indent=2,
            )
        lines.append(f"\nWrote fit scores to {json_out}")

    return "\n".join(lines)
```

- [ ] **Step 2: Wire the CLI flag**

In `scripts/wpilog_to_csv.py`, inside `main()`, add these arguments alongside the existing ones:

```python
    parser.add_argument(
        "--compare",
        nargs=2,
        metavar=("SIM_LOG", "REAL_LOG"),
        help="Compare a sim log against a real match log",
    )
    parser.add_argument("--json", help="Write fit scores to this JSON path")
    parser.add_argument(
        "--top", type=int, default=25, help="How many diverging signals to print"
    )
```

Then, as the **first** branch in `main()`'s dispatch (before the existing positional-file handling, because `--compare` supplies its own two files):

```python
    if args.compare:
        from log_compare import compare_logs

        print(compare_logs(args.compare[0], args.compare[1], args.top, args.json))
        return
```

The existing positional `file` argument must become optional for this to work — change it to `nargs="?"` and add a guard that errors if neither `--compare` nor a file is given.

- [ ] **Step 3: Verify the CLI parses**

```bash
python scripts/wpilog_to_csv.py --help
```

Expected: `--compare SIM_LOG REAL_LOG` appears in the output, and the command exits 0.

- [ ] **Step 4: Run the full Python test suite**

```bash
python -m pytest scripts/tests/ -v
```

Expected: 25 passed

- [ ] **Step 5: Commit**

```bash
git add scripts/log_compare.py scripts/wpilog_to_csv.py
git commit -m "Wire --compare CLI into wpilog analysis tool"
```

---

## Phase B — Log Input Replay

### Task 7: `LogTimeline`

**Files:**
- Create: `src/main/java/frc/robot/utility/replay/LogTimeline.java`
- Test: `src/test/java/frc/robot/utility/replay/LogTimelineTest.java`

- [ ] **Step 1: Write the failing test**

Create `src/test/java/frc/robot/utility/replay/LogTimelineTest.java`:

```java
package frc.robot.utility.replay;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class LogTimelineTest {

  @Test
  void holdsValueUntilNextSample() {
    LogTimeline<String> timeline =
        LogTimeline.<String>builder().add(0.0, "a").add(1.0, "b").build();

    assertEquals("a", timeline.valueAt(0.0));
    assertEquals("a", timeline.valueAt(0.99));
    assertEquals("b", timeline.valueAt(1.0));
    assertEquals("b", timeline.valueAt(500.0));
  }

  @Test
  void returnsNullBeforeFirstSample() {
    LogTimeline<String> timeline = LogTimeline.<String>builder().add(5.0, "a").build();
    assertNull(timeline.valueAt(4.99));
  }

  @Test
  void returnsDefaultBeforeFirstSampleWhenGiven() {
    LogTimeline<String> timeline = LogTimeline.<String>builder().add(5.0, "a").build();
    assertEquals("fallback", timeline.valueAt(0.0, "fallback"));
  }

  @Test
  void emptyTimelineIsEmptyAndYieldsNull() {
    LogTimeline<String> timeline = LogTimeline.<String>builder().build();
    assertTrue(timeline.isEmpty());
    assertNull(timeline.valueAt(0.0));
  }

  @Test
  void reportsFirstAndLastTimestamps() {
    LogTimeline<String> timeline =
        LogTimeline.<String>builder().add(2.0, "a").add(7.5, "b").build();
    assertEquals(2.0, timeline.firstTimestamp());
    assertEquals(7.5, timeline.lastTimestamp());
  }

  @Test
  void findsFirstTimestampMatchingPredicate() {
    LogTimeline<Boolean> timeline =
        LogTimeline.<Boolean>builder().add(1.0, false).add(9.0, true).add(12.0, false).build();
    assertEquals(9.0, timeline.firstTimestampWhere(v -> v));
  }

  @Test
  void findsLastTimestampMatchingPredicate() {
    LogTimeline<Boolean> timeline =
        LogTimeline.<Boolean>builder().add(1.0, true).add(9.0, false).add(12.0, true).build();
    assertEquals(12.0, timeline.lastTimestampWhere(v -> v));
  }

  @Test
  void predicateSearchReturnsNaNWhenNothingMatches() {
    LogTimeline<Boolean> timeline = LogTimeline.<Boolean>builder().add(1.0, false).build();
    assertTrue(Double.isNaN(timeline.firstTimestampWhere(v -> v)));
  }

  @Test
  void lookupIsCorrectAcrossManySamples() {
    // Binary search must stay correct at scale; a real log has thousands.
    LogTimeline.Builder<Integer> builder = LogTimeline.builder();
    for (int i = 0; i < 5000; i++) {
      builder.add(i * 0.02, i);
    }
    LogTimeline<Integer> timeline = builder.build();
    assertEquals(2500, timeline.valueAt(2500 * 0.02));
    assertEquals(2500, timeline.valueAt(2500 * 0.02 + 0.019));
  }
}
```

- [ ] **Step 2: Run test to verify it fails**

```bash
./gradlew test --no-daemon --tests "frc.robot.utility.replay.LogTimelineTest"
```

Expected: FAIL — compilation error, `package frc.robot.utility.replay does not exist`

- [ ] **Step 3: Write minimal implementation**

Create `src/main/java/frc/robot/utility/replay/LogTimeline.java`:

```java
package frc.robot.utility.replay;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Predicate;

/**
 * An immutable, time-ordered series of values with zero-order-hold lookup.
 *
 * <p>Zero-order hold means {@link #valueAt(double)} returns the value of the most recent sample at
 * or before the requested time — the same semantics the driver station uses, where a joystick value
 * persists until the next packet arrives.
 *
 * <p>Pure data: no HAL, no WPILib runtime dependency, unit-testable without a robot.
 */
public final class LogTimeline<T> {

  private final double[] timestamps;
  private final List<T> values;

  private LogTimeline(double[] timestamps, List<T> values) {
    this.timestamps = timestamps;
    this.values = values;
  }

  public static <T> Builder<T> builder() {
    return new Builder<>();
  }

  /** Value in effect at {@code seconds}, or null if that is before the first sample. */
  public T valueAt(double seconds) {
    return valueAt(seconds, null);
  }

  /** Value in effect at {@code seconds}, or {@code fallback} if before the first sample. */
  public T valueAt(double seconds, T fallback) {
    int index = indexAt(seconds);
    return index < 0 ? fallback : values.get(index);
  }

  public boolean isEmpty() {
    return timestamps.length == 0;
  }

  public int size() {
    return timestamps.length;
  }

  public double firstTimestamp() {
    return isEmpty() ? Double.NaN : timestamps[0];
  }

  public double lastTimestamp() {
    return isEmpty() ? Double.NaN : timestamps[timestamps.length - 1];
  }

  /** Timestamp of the earliest sample satisfying {@code predicate}, or NaN if none does. */
  public double firstTimestampWhere(Predicate<T> predicate) {
    for (int i = 0; i < timestamps.length; i++) {
      if (predicate.test(values.get(i))) {
        return timestamps[i];
      }
    }
    return Double.NaN;
  }

  /** Timestamp of the latest sample satisfying {@code predicate}, or NaN if none does. */
  public double lastTimestampWhere(Predicate<T> predicate) {
    for (int i = timestamps.length - 1; i >= 0; i--) {
      if (predicate.test(values.get(i))) {
        return timestamps[i];
      }
    }
    return Double.NaN;
  }

  /** Index of the last sample at or before {@code seconds}, or -1. */
  private int indexAt(double seconds) {
    if (timestamps.length == 0 || seconds < timestamps[0]) {
      return -1;
    }
    int found = Arrays.binarySearch(timestamps, seconds);
    // binarySearch returns -(insertionPoint) - 1 on a miss; the sample we want
    // is the one just before the insertion point.
    return found >= 0 ? found : -found - 2;
  }

  /** Accumulates samples in ascending timestamp order. */
  public static final class Builder<T> {
    private final List<Double> timestamps = new ArrayList<>();
    private final List<T> values = new ArrayList<>();

    public Builder<T> add(double seconds, T value) {
      timestamps.add(seconds);
      values.add(value);
      return this;
    }

    public LogTimeline<T> build() {
      double[] array = new double[timestamps.size()];
      for (int i = 0; i < array.length; i++) {
        array[i] = timestamps.get(i);
      }
      return new LogTimeline<>(array, List.copyOf(values));
    }
  }
}
```

- [ ] **Step 4: Run test to verify it passes**

```bash
./gradlew test --no-daemon --tests "frc.robot.utility.replay.LogTimelineTest"
```

Expected: BUILD SUCCESSFUL, 9 tests passed

- [ ] **Step 5: Format and commit**

```bash
./gradlew spotlessApply --no-daemon
git add src/main/java/frc/robot/utility/replay/LogTimeline.java src/test/java/frc/robot/utility/replay/LogTimelineTest.java
git commit -m "Add LogTimeline zero-order-hold series for log replay"
```

---

### Task 8: `MatchInputs` record

**Files:**
- Create: `src/main/java/frc/robot/utility/replay/MatchInputs.java`

This is a plain data holder with no behavior to test on its own; `MatchLogReaderTest` in Task 9 covers it.

- [ ] **Step 1: Write the record**

Create `src/main/java/frc/robot/utility/replay/MatchInputs.java`:

```java
package frc.robot.utility.replay;

/**
 * Every driver-station and joystick timeline parsed out of a real match log, plus the match
 * boundaries needed to rebase time.
 *
 * <p>All timestamps are raw FPGA seconds as they appear in the source log. Consumers rebase against
 * {@link #matchStartSeconds()}.
 *
 * @param joystickAxes per-port axis values; index 0 is port 0 (driverA), index 1 is port 1
 *     (driverB)
 * @param joystickButtons per-port button bitfields — WPILib button N is bit {@code N-1}
 * @param joystickPovs per-port POV values in degrees, or -1 when centered
 * @param enabled DriverStation enable state
 * @param autonomous true during autonomous
 * @param allianceStation raw AllianceStation enum ordinal from the log
 * @param estimatedPose robot pose as {x metres, y metres, theta radians}
 * @param autoName the auto selected on the dashboard, or null if the log did not record one
 * @param matchStartSeconds timestamp of the first enable — the replay time origin
 * @param matchEndSeconds timestamp of the final disable
 */
public record MatchInputs(
    LogTimeline<float[]>[] joystickAxes,
    LogTimeline<Long>[] joystickButtons,
    LogTimeline<long[]>[] joystickPovs,
    LogTimeline<Boolean> enabled,
    LogTimeline<Boolean> autonomous,
    LogTimeline<Long> allianceStation,
    LogTimeline<double[]> estimatedPose,
    String autoName,
    double matchStartSeconds,
    double matchEndSeconds) {

  /** Number of joystick ports this replay carries. Ports 0 and 1 are driverA and driverB. */
  public static final int PORT_COUNT = 2;

  /** Total match length in seconds. */
  public double durationSeconds() {
    return matchEndSeconds - matchStartSeconds;
  }
}
```

- [ ] **Step 2: Verify it compiles**

```bash
./gradlew compileJava --no-daemon
```

Expected: BUILD SUCCESSFUL

- [ ] **Step 3: Format and commit**

```bash
./gradlew spotlessApply --no-daemon
git add src/main/java/frc/robot/utility/replay/MatchInputs.java
git commit -m "Add MatchInputs record for parsed match log timelines"
```

---

### Task 9: `MatchLogReader`

**Files:**
- Create: `src/main/java/frc/robot/utility/replay/MatchLogReader.java`
- Test: `src/test/java/frc/robot/utility/replay/MatchLogReaderTest.java`

The test writes a tiny synthetic `.wpilog` with `DataLogWriter` and reads it back, so it needs no external fixture file.

- [ ] **Step 1: Write the failing test**

Create `src/test/java/frc/robot/utility/replay/MatchLogReaderTest.java`:

```java
package frc.robot.utility.replay;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.util.datalog.DataLogWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class MatchLogReaderTest {

  /** Writes a minimal but realistic match log: idle, enable, auto, teleop, disable. */
  private Path writeSyntheticLog(Path dir) throws IOException {
    Path file = dir.resolve("synthetic.wpilog");
    try (DataLogWriter log = new DataLogWriter(file.toString())) {
      int enabled = log.start("DriverStation/Enabled", "boolean", "");
      int auto = log.start("DriverStation/Autonomous", "boolean", "");
      int station = log.start("DriverStation/AllianceStation", "int64", "");
      int axes = log.start("DriverStation/Joystick0/AxisValues", "float[]", "");
      int buttons = log.start("DriverStation/Joystick0/ButtonValues", "int64", "");
      int povs = log.start("DriverStation/Joystick0/POVs", "int64[]", "");
      int chooser = log.start("NetworkInputs/SmartDashboard/Auto Chooser", "string", "");

      // Timestamps are microseconds in the wpilog format.
      log.appendBoolean(enabled, false, 1_000_000L);
      log.appendInteger(station, 1L, 1_000_000L);
      log.appendString(chooser, "2x4TRight", 1_000_000L);

      log.appendBoolean(enabled, true, 10_000_000L); // match starts at t=10s
      log.appendBoolean(auto, true, 10_000_000L);
      log.appendFloatArray(axes, new float[] {0.0f, -0.5f}, 10_000_000L);
      log.appendInteger(buttons, 0L, 10_000_000L);
      log.appendIntegerArray(povs, new long[] {-1L}, 10_000_000L);

      log.appendBoolean(auto, false, 25_000_000L); // teleop at t=25s
      log.appendFloatArray(axes, new float[] {0.25f, -1.0f}, 26_000_000L);
      log.appendInteger(buttons, 0b100000L, 26_000_000L); // button 6 held
      log.appendIntegerArray(povs, new long[] {180L}, 26_000_000L);

      log.appendBoolean(enabled, false, 40_000_000L); // match ends at t=40s
    }
    return file;
  }

  @Test
  void readsMatchBoundaries(@TempDir Path dir) throws IOException {
    MatchInputs inputs = MatchLogReader.read(writeSyntheticLog(dir).toString());
    assertEquals(10.0, inputs.matchStartSeconds(), 1e-6);
    assertEquals(40.0, inputs.matchEndSeconds(), 1e-6);
    assertEquals(30.0, inputs.durationSeconds(), 1e-6);
  }

  @Test
  void readsAxesWithZeroOrderHold(@TempDir Path dir) throws IOException {
    MatchInputs inputs = MatchLogReader.read(writeSyntheticLog(dir).toString());
    assertArrayEquals(
        new float[] {0.0f, -0.5f}, inputs.joystickAxes()[0].valueAt(20.0), 1e-6f);
    assertArrayEquals(
        new float[] {0.25f, -1.0f}, inputs.joystickAxes()[0].valueAt(30.0), 1e-6f);
  }

  @Test
  void readsButtonBitfield(@TempDir Path dir) throws IOException {
    MatchInputs inputs = MatchLogReader.read(writeSyntheticLog(dir).toString());
    assertEquals(0L, inputs.joystickButtons()[0].valueAt(20.0));
    assertEquals(0b100000L, inputs.joystickButtons()[0].valueAt(30.0));
  }

  @Test
  void readsPovs(@TempDir Path dir) throws IOException {
    MatchInputs inputs = MatchLogReader.read(writeSyntheticLog(dir).toString());
    assertEquals(-1L, inputs.joystickPovs()[0].valueAt(20.0)[0]);
    assertEquals(180L, inputs.joystickPovs()[0].valueAt(30.0)[0]);
  }

  @Test
  void readsAutonomousTransition(@TempDir Path dir) throws IOException {
    MatchInputs inputs = MatchLogReader.read(writeSyntheticLog(dir).toString());
    assertTrue(inputs.autonomous().valueAt(15.0));
    assertTrue(!inputs.autonomous().valueAt(30.0));
  }

  @Test
  void readsAutoChooserName(@TempDir Path dir) throws IOException {
    MatchInputs inputs = MatchLogReader.read(writeSyntheticLog(dir).toString());
    assertEquals("2x4TRight", inputs.autoName());
  }

  @Test
  void readsAllianceStation(@TempDir Path dir) throws IOException {
    MatchInputs inputs = MatchLogReader.read(writeSyntheticLog(dir).toString());
    assertEquals(1L, inputs.allianceStation().valueAt(15.0));
  }

  @Test
  void missingFileThrows() {
    assertThrows(IOException.class, () -> MatchLogReader.read("does-not-exist.wpilog"));
  }

  @Test
  void logWithNoEnableThrows(@TempDir Path dir) throws IOException {
    Path file = dir.resolve("never-enabled.wpilog");
    try (DataLogWriter log = new DataLogWriter(file.toString())) {
      int enabled = log.start("DriverStation/Enabled", "boolean", "");
      log.appendBoolean(enabled, false, 1_000_000L);
    }
    // A log the robot was never enabled in has no match to replay; failing loudly
    // beats silently replaying an empty match.
    assertThrows(IllegalStateException.class, () -> MatchLogReader.read(file.toString()));
  }

  @Test
  void cleansUpTempFile(@TempDir Path dir) throws IOException {
    Path file = writeSyntheticLog(dir);
    assertTrue(Files.exists(file));
  }
}
```

- [ ] **Step 2: Run test to verify it fails**

```bash
./gradlew test --no-daemon --tests "frc.robot.utility.replay.MatchLogReaderTest"
```

Expected: FAIL — `cannot find symbol: class MatchLogReader`

- [ ] **Step 3: Write minimal implementation**

Create `src/main/java/frc/robot/utility/replay/MatchLogReader.java`:

```java
package frc.robot.utility.replay;

import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

/**
 * Parses a real match {@code .wpilog} into the driver-station and joystick timelines needed to
 * replay it in simulation.
 *
 * <p>Pure parsing: no HAL, no WPILib runtime state. Safe to unit test.
 */
public final class MatchLogReader {

  private static final String KEY_ENABLED = "DriverStation/Enabled";
  private static final String KEY_AUTONOMOUS = "DriverStation/Autonomous";
  private static final String KEY_STATION = "DriverStation/AllianceStation";
  private static final String KEY_POSE = "RealOutputs/Robot State/Estimated Pose";
  private static final String KEY_CHOOSER = "NetworkInputs/SmartDashboard/Auto Chooser";

  private MatchLogReader() {}

  /**
   * Read {@code path} and extract everything needed to replay the match.
   *
   * @throws IOException if the file cannot be read
   * @throws IllegalStateException if the log never shows the robot enabled
   */
  @SuppressWarnings("unchecked")
  public static MatchInputs read(String path) throws IOException {
    DataLogReader reader = new DataLogReader(path);
    if (!reader.isValid()) {
      throw new IOException("Not a valid wpilog file: " + path);
    }

    Map<Integer, String> entryNames = new HashMap<>();
    Map<Integer, String> entryTypes = new HashMap<>();

    LogTimeline.Builder<float[]>[] axes = new LogTimeline.Builder[MatchInputs.PORT_COUNT];
    LogTimeline.Builder<Long>[] buttons = new LogTimeline.Builder[MatchInputs.PORT_COUNT];
    LogTimeline.Builder<long[]>[] povs = new LogTimeline.Builder[MatchInputs.PORT_COUNT];
    for (int port = 0; port < MatchInputs.PORT_COUNT; port++) {
      axes[port] = LogTimeline.builder();
      buttons[port] = LogTimeline.builder();
      povs[port] = LogTimeline.builder();
    }

    LogTimeline.Builder<Boolean> enabled = LogTimeline.builder();
    LogTimeline.Builder<Boolean> autonomous = LogTimeline.builder();
    LogTimeline.Builder<Long> station = LogTimeline.builder();
    LogTimeline.Builder<double[]> pose = LogTimeline.builder();
    String autoName = null;

    for (DataLogRecord record : reader) {
      if (record.isControl()) {
        if (record.isStart()) {
          DataLogRecord.StartRecordData start = record.getStartData();
          // Log keys are written with a leading slash in some producers.
          entryNames.put(start.entry, start.name.startsWith("/")
              ? start.name.substring(1)
              : start.name);
          entryTypes.put(start.entry, start.type);
        }
        continue;
      }

      String name = entryNames.get(record.getEntry());
      if (name == null) {
        continue;
      }
      double seconds = record.getTimestamp() / 1_000_000.0;

      switch (name) {
        case KEY_ENABLED -> enabled.add(seconds, record.getBoolean());
        case KEY_AUTONOMOUS -> autonomous.add(seconds, record.getBoolean());
        case KEY_STATION -> station.add(seconds, record.getInteger());
        case KEY_CHOOSER -> autoName = record.getString();
        case KEY_POSE -> {
          double[] parsed = parsePose2d(record.getRaw());
          if (parsed != null) {
            pose.add(seconds, parsed);
          }
        }
        default -> {
          int port = joystickPort(name);
          if (port >= 0 && port < MatchInputs.PORT_COUNT) {
            if (name.endsWith("/AxisValues")) {
              axes[port].add(seconds, record.getFloatArray());
            } else if (name.endsWith("/ButtonValues")) {
              buttons[port].add(seconds, record.getInteger());
            } else if (name.endsWith("/POVs")) {
              povs[port].add(seconds, record.getIntegerArray());
            }
          }
        }
      }
    }

    LogTimeline<Boolean> enabledTimeline = enabled.build();
    double start = enabledTimeline.firstTimestampWhere(value -> value);
    if (Double.isNaN(start)) {
      throw new IllegalStateException(
          "Log never shows DriverStation/Enabled = true, so there is no match to replay: " + path);
    }
    double end = enabledTimeline.lastTimestamp();

    LogTimeline<float[]>[] axesBuilt = new LogTimeline[MatchInputs.PORT_COUNT];
    LogTimeline<Long>[] buttonsBuilt = new LogTimeline[MatchInputs.PORT_COUNT];
    LogTimeline<long[]>[] povsBuilt = new LogTimeline[MatchInputs.PORT_COUNT];
    for (int port = 0; port < MatchInputs.PORT_COUNT; port++) {
      axesBuilt[port] = axes[port].build();
      buttonsBuilt[port] = buttons[port].build();
      povsBuilt[port] = povs[port].build();
    }

    return new MatchInputs(
        axesBuilt,
        buttonsBuilt,
        povsBuilt,
        enabledTimeline,
        autonomous.build(),
        station.build(),
        pose.build(),
        autoName,
        start,
        end);
  }

  /**
   * Extract the port number from a key like {@code DriverStation/Joystick0/AxisValues}, or -1 if the
   * key is not a joystick key.
   */
  private static int joystickPort(String key) {
    final String prefix = "DriverStation/Joystick";
    if (!key.startsWith(prefix) || key.length() <= prefix.length()) {
      return -1;
    }
    char digit = key.charAt(prefix.length());
    return Character.isDigit(digit) ? digit - '0' : -1;
  }

  /**
   * Decode a WPILib struct:Pose2d payload into {x, y, theta}.
   *
   * <p>The layout is three little-endian doubles: Translation2d x, Translation2d y, then the
   * Rotation2d angle in radians.
   */
  private static double[] parsePose2d(byte[] raw) {
    if (raw.length < 24) {
      return null;
    }
    java.nio.ByteBuffer buffer =
        java.nio.ByteBuffer.wrap(raw).order(java.nio.ByteOrder.LITTLE_ENDIAN);
    return new double[] {buffer.getDouble(), buffer.getDouble(), buffer.getDouble()};
  }
}
```

- [ ] **Step 4: Run test to verify it passes**

```bash
./gradlew test --no-daemon --tests "frc.robot.utility.replay.MatchLogReaderTest"
```

Expected: BUILD SUCCESSFUL, 10 tests passed

- [ ] **Step 5: Verify against a real Worlds log**

The synthetic log proves the parser handles well-formed input, but not that it handles a 62 MB file written by a real robot with 481 keys. Add this test to `MatchLogReaderTest`. The `assumeTrue` makes it skip cleanly on a machine without the log folder, so it is safe to keep permanently.

```java
  @Test
  void parsesRealWorldsLog() throws IOException {
    String path =
        "C:\\Users\\bruce\\Downloads\\LOGS-2026-main\\LOGS-2026-main\\Worlds\\"
            + "akit_26-04-30_14-50-56_johnson_q54.wpilog";
    org.junit.jupiter.api.Assumptions.assumeTrue(java.nio.file.Files.exists(Path.of(path)));
    MatchInputs inputs = MatchLogReader.read(path);
    // q54 is a full match: roughly 165 s of enabled time.
    assertTrue(inputs.durationSeconds() > 150.0 && inputs.durationSeconds() < 180.0);
    assertTrue(inputs.joystickAxes()[0].size() > 1000);
  }
```

```bash
./gradlew test --no-daemon --tests "frc.robot.utility.replay.MatchLogReaderTest"
```

Expected: 11 tests passed (or 10 passed + 1 skipped if the log folder is absent)

- [ ] **Step 6: Format and commit**

```bash
./gradlew spotlessApply --no-daemon
git add src/main/java/frc/robot/utility/replay/MatchLogReader.java src/test/java/frc/robot/utility/replay/MatchLogReaderTest.java
git commit -m "Parse match logs into replayable driver input timelines"
```

---

### Task 10: `LogInputPlayer`

**Files:**
- Create: `src/main/java/frc/robot/utility/replay/LogInputPlayer.java`

- [ ] **Step 1: Write the implementation**

Create `src/main/java/frc/robot/utility/replay/LogInputPlayer.java`:

```java
package frc.robot.utility.replay;

import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.GenericHIDSim;
import org.littletonrobotics.junction.Logger;

/**
 * Injects a real match's logged driver inputs into simulation, frame-locked to the robot loop.
 *
 * <p>Advanced by exactly one loop period per call to {@link #step(double)} rather than by wall
 * clock, so injection stays aligned with the 20 ms robot loop even when the simulation runs faster
 * or slower than real time.
 *
 * <p>SIM only. Constructing this on a real robot would fight the real driver station.
 */
public final class LogInputPlayer {

  /** Axis and button counts to advertise for each simulated controller. */
  private static final int AXIS_COUNT = 6;

  private static final int BUTTON_COUNT = 10;

  private final MatchInputs inputs;
  private final GenericHIDSim[] controllers = new GenericHIDSim[MatchInputs.PORT_COUNT];

  private double elapsedSeconds = 0.0;
  private boolean finished = false;

  public LogInputPlayer(MatchInputs inputs) {
    this.inputs = inputs;

    for (int port = 0; port < MatchInputs.PORT_COUNT; port++) {
      GenericHIDSim controller = new GenericHIDSim(port);
      controller.setAxisCount(AXIS_COUNT);
      controller.setButtonCount(BUTTON_COUNT);
      controller.setPOVCount(1);
      controller.setPOV(-1);
      controller.notifyNewData();
      controllers[port] = controller;
    }

    // Alliance station is fixed for the match, so apply it once up front.
    Long station = inputs.allianceStation().valueAt(inputs.matchStartSeconds());
    if (station != null) {
      DriverStationSim.setAllianceStationId(
          edu.wpi.first.hal.AllianceStationID.values()[(int) (long) station]);
    }
    DriverStationSim.setDsAttached(true);
    DriverStationSim.notifyNewData();
  }

  /** The auto selected in the source log, or null if it recorded none. */
  public String autoName() {
    return inputs.autoName();
  }

  /** Seconds of match time replayed so far. */
  public double elapsedSeconds() {
    return elapsedSeconds;
  }

  /**
   * Skip autonomous and start the replay at the moment teleop began, for faster iteration on
   * teleop-only power and mechanism tuning.
   *
   * <p>Returns the logged pose at teleop entry so the caller can place the robot there — without
   * that, the replay would start teleop from wherever the drivetrain was initialised rather than
   * where the real robot finished its auto.
   *
   * @return the {x, y, theta} pose at teleop entry, or null if the log has no pose data
   */
  public double[] seekToTeleop() {
    double teleopStart = inputs.autonomous().firstTimestampWhere(value -> !value);
    if (Double.isNaN(teleopStart) || teleopStart < inputs.matchStartSeconds()) {
      // The log never left autonomous; nothing to seek to.
      return null;
    }
    elapsedSeconds = teleopStart - inputs.matchStartSeconds();
    return inputs.estimatedPose().valueAt(teleopStart);
  }

  /** True once the replay has passed the end of the logged match. */
  public boolean isFinished() {
    return finished;
  }

  /** True while the source log was in autonomous at the current replay position. */
  public boolean isAutonomous() {
    return Boolean.TRUE.equals(inputs.autonomous().valueAt(logTime(), false));
  }

  /** The logged robot pose at the current replay position, as {x, y, theta}, or null. */
  public double[] loggedPose() {
    return inputs.estimatedPose().valueAt(logTime());
  }

  /** Current position in the source log's own time base. */
  private double logTime() {
    return inputs.matchStartSeconds() + elapsedSeconds;
  }

  /**
   * Advance the replay by {@code dtSeconds} and push the resulting driver station and joystick state
   * into simulation.
   */
  public void step(double dtSeconds) {
    if (finished) {
      return;
    }
    elapsedSeconds += dtSeconds;
    double now = logTime();

    if (elapsedSeconds > inputs.durationSeconds()) {
      finished = true;
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
      return;
    }

    for (int port = 0; port < MatchInputs.PORT_COUNT; port++) {
      GenericHIDSim controller = controllers[port];

      float[] axisValues = inputs.joystickAxes()[port].valueAt(now);
      if (axisValues != null) {
        for (int axis = 0; axis < Math.min(axisValues.length, AXIS_COUNT); axis++) {
          controller.setRawAxis(axis, axisValues[axis]);
        }
      }

      Long bitfield = inputs.joystickButtons()[port].valueAt(now);
      if (bitfield != null) {
        for (int button = 1; button <= BUTTON_COUNT; button++) {
          // WPILib numbers buttons from 1; the log packs button N into bit N-1.
          controller.setRawButton(button, (bitfield & (1L << (button - 1))) != 0);
        }
      }

      long[] povValues = inputs.joystickPovs()[port].valueAt(now);
      controller.setPOV(0, povValues != null && povValues.length > 0 ? (int) povValues[0] : -1);

      controller.notifyNewData();
    }

    DriverStationSim.setEnabled(Boolean.TRUE.equals(inputs.enabled().valueAt(now, false)));
    DriverStationSim.setAutonomous(Boolean.TRUE.equals(inputs.autonomous().valueAt(now, false)));
    DriverStationSim.setDsAttached(true);
    DriverStationSim.notifyNewData();

    Logger.recordOutput("Replay/Elapsed Seconds", elapsedSeconds);
    Logger.recordOutput("Replay/Log Time Seconds", now);
  }
}
```

- [ ] **Step 2: Verify it compiles**

```bash
./gradlew compileJava --no-daemon
```

Expected: BUILD SUCCESSFUL

- [ ] **Step 3: Format and commit**

```bash
./gradlew spotlessApply --no-daemon
git add src/main/java/frc/robot/utility/replay/LogInputPlayer.java
git commit -m "Add frame-locked driver input player for log replay"
```

---

### Task 11: Wire replay into `Robot.java` and `build.gradle`

**Files:**
- Modify: `src/main/java/frc/robot/Robot.java`
- Modify: `src/main/java/frc/robot/RobotContainer.java:597-604`
- Modify: `build.gradle:220-238`

- [ ] **Step 1: Add the Gradle property wiring**

In `build.gradle`, inside the existing `tasks.withType(JavaExec).configureEach { ... }` block (after the `teleop.axes` block), add:

```groovy
    if (project.hasProperty('replay.inputs')) {
        jvmArgs "-Dai.replay.inputs=${project.getProperty('replay.inputs')}"
    }
    if (project.hasProperty('replay.anchor')) {
        jvmArgs "-Dai.replay.anchor=${project.getProperty('replay.anchor')}"
    }
    if (project.hasProperty('replay.battery')) {
        jvmArgs "-Dai.replay.battery=${project.getProperty('replay.battery')}"
    }
    if (project.hasProperty('replay.fast')) {
        jvmArgs '-Dai.replay.fast=true'
    }
    if (project.hasProperty('replay.teleopOnly')) {
        jvmArgs '-Dai.replay.teleopOnly=true'
    }
```

- [ ] **Step 2: Add the player field and construction to `Robot.java`**

Add these imports:

```java
import frc.robot.utility.replay.LogInputPlayer;
import frc.robot.utility.replay.MatchInputs;
import frc.robot.utility.replay.MatchLogReader;
```

Add this field beside the existing `aiShutdownInitiated` field:

```java
  private LogInputPlayer replayPlayer;
```

At the **end** of `robotInit()` — replacing the existing `startAiEnableThread` block, which must not run in replay mode because the player drives enable state itself:

```java
    String replayInputsPath = System.getProperty("ai.replay.inputs");
    if (Constants.getRobotMode() == Constants.Mode.SIM
        && replayInputsPath != null
        && !replayInputsPath.isBlank()) {
      try {
        MatchInputs matchInputs = MatchLogReader.read(replayInputsPath);
        replayPlayer = new LogInputPlayer(matchInputs);
        System.out.println(
            "[Replay] Loaded "
                + replayInputsPath
                + " — "
                + String.format("%.1f", matchInputs.durationSeconds())
                + "s match, auto="
                + matchInputs.autoName());
        if (Boolean.getBoolean("ai.replay.fast")) {
          // Free-run the loop instead of pacing to wall clock. The player is
          // frame-locked, so injection stays correct at any speed.
          setUseTiming(false);
        }
      } catch (Exception e) {
        throw new IllegalStateException("Failed to load replay log: " + replayInputsPath, e);
      }
    } else {
      String aiAutoName = System.getProperty("ai.auto.name");
      boolean aiLogging = Boolean.getBoolean("ai.logging");
      if (Constants.getRobotMode() == Constants.Mode.SIM && (aiAutoName != null || aiLogging)) {
        startAiEnableThread(aiAutoName != null);
      }
    }
```

- [ ] **Step 3: Tick the player from `robotPeriodic()`**

Change `robotPeriodic()` to step the player **before** the command scheduler runs, so the scheduler sees this loop's inputs:

```java
  @Override
  public void robotPeriodic() {
    if (replayPlayer != null) {
      replayPlayer.step(Constants.PERIODIC_LOOP_SEC);
      if (replayPlayer.isFinished() && !aiShutdownInitiated) {
        aiShutdownInitiated = true;
        endCompetition();
      }
    }

    /** TODO: Is this necessary? */
    Threads.setCurrentThreadPriority(true, 99);

    CommandScheduler.getInstance().run();

    Threads.setCurrentThreadPriority(false, 10);
  }
```

- [ ] **Step 4: Suppress the wall-clock teleop thread during replay**

In `teleopInit()`, change the existing guard so the old scripted-input thread does not fight the player:

```java
    if (Boolean.getBoolean("ai.logging") && replayPlayer == null) {
      startAiTeleopThread();
    }
```

- [ ] **Step 5: Make the auto selection follow the replay log**

In `RobotContainer.getAutoCommand()`, change the body to prefer an explicit `-Pauto.name`, then fall back to the replay log's recorded chooser value:

```java
  public Command getAutoCommand() {
    // When running headlessly for AI testing, bypass the dashboard chooser entirely.
    // Invoke: ./gradlew simulateJava -Pheadless -Pai.logging -Pauto.name=2x4TRight
    String aiAutoName = System.getProperty("ai.auto.name");
    if (aiAutoName == null || aiAutoName.isBlank()) {
      // In replay mode, run whatever auto the real match ran.
      aiAutoName = System.getProperty("ai.replay.auto.name");
    }
    if (aiAutoName != null && !aiAutoName.isBlank()) {
      return AutoBuilder.buildAuto(aiAutoName);
    }
    return autoChooser.get();
  }
```

And in `Robot.robotInit()`, immediately after constructing `replayPlayer`, publish the name so `RobotContainer` (constructed later) can read it:

```java
        if (replayPlayer.autoName() != null && !replayPlayer.autoName().isBlank()) {
          System.setProperty("ai.replay.auto.name", replayPlayer.autoName());
        }
```

**Ordering note:** `replayPlayer` must be constructed **before** `robotContainer = new RobotContainer();` for this property to be visible. Move the replay-loading block above the `robotContainer` construction line.

- [ ] **Step 6: Verify it compiles**

```bash
./gradlew compileJava --no-daemon
```

Expected: BUILD SUCCESSFUL

- [ ] **Step 7: Smoke-test the replay end to end**

```bash
./gradlew simulateJava --no-daemon -Pheadless -Pai.logging -Preplay.fast \
  "-Preplay.inputs=C:\\Users\\bruce\\Downloads\\LOGS-2026-main\\LOGS-2026-main\\Worlds\\akit_26-04-30_14-50-56_johnson_q54.wpilog"
```

Expected: `[Replay] Loaded ... 165.1s match, auto=...` printed, then the sim runs and exits. A `.wpilog` appears in `build/ai-logs/`.

Reminder — these stdout messages are **normal and not errors**: `The robot program quit unexpectedly` (that is how `endCompetition()` exits), `Joystick Button X on port Y not available`, `Device firmware could not be retrieved`.

- [ ] **Step 8: Format and commit**

```bash
./gradlew spotlessApply --no-daemon
git add build.gradle src/main/java/frc/robot/Robot.java src/main/java/frc/robot/RobotContainer.java
git commit -m "Wire log-driven input replay into robot startup"
```

---

### Task 12: Pose anchoring

**Files:**
- Create: `src/main/java/frc/robot/utility/replay/PoseAnchor.java`
- Modify: `src/main/java/frc/robot/RobotContainer.java` (`updateSimulation()`)

- [ ] **Step 1: Write the implementation**

Create `src/main/java/frc/robot/utility/replay/PoseAnchor.java`:

```java
package frc.robot.utility.replay;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

/**
 * Periodically snaps the simulated robot back to the pose the real robot held, and logs how far it
 * had drifted first.
 *
 * <p>Real matches involve contact the simulation does not model, so after autonomous the simulated
 * robot diverges. Left uncorrected it eventually wedges against a wall and draws current that never
 * happened, corrupting the power measurements this whole exercise exists to compare.
 *
 * <p>The interval defaults to 10 seconds rather than something tight. A short interval would keep
 * auto-aim inputs accurate but would also continuously erase drift — hiding a drivetrain model that
 * is genuinely wrong. At 10 seconds the drift accumulated before each correction is published as
 * {@code Replay/Anchor Error}, making drivetrain fidelity directly measurable.
 */
public final class PoseAnchor {

  private final SwerveDriveSimulation driveSimulation;
  private final double intervalSeconds;

  private double lastAnchorSeconds = Double.NEGATIVE_INFINITY;
  private boolean anchoredAtTeleopStart = false;

  /**
   * @param driveSimulation the maple-sim drivetrain to reposition
   * @param intervalSeconds seconds between corrections; zero or negative disables anchoring
   */
  public PoseAnchor(SwerveDriveSimulation driveSimulation, double intervalSeconds) {
    this.driveSimulation = driveSimulation;
    this.intervalSeconds = intervalSeconds;
  }

  /** True when anchoring is switched off entirely. */
  public boolean isDisabled() {
    return intervalSeconds <= 0.0;
  }

  /**
   * Consider anchoring at the current replay position.
   *
   * <p>Does nothing during autonomous — pose is trusted there and left free-running so the auto
   * segment stays a valid comparison.
   */
  public void update(LogInputPlayer player) {
    if (isDisabled() || player.isAutonomous()) {
      return;
    }

    double[] logged = player.loggedPose();
    if (logged == null) {
      return;
    }

    double now = player.elapsedSeconds();
    boolean teleopEntry = !anchoredAtTeleopStart;
    if (!teleopEntry && now - lastAnchorSeconds < intervalSeconds) {
      return;
    }

    Pose2d target = new Pose2d(logged[0], logged[1], new Rotation2d(logged[2]));
    Pose2d actual = driveSimulation.getSimulatedDriveTrainPose();

    // Publish the drift BEFORE correcting it — that gap is the measurement.
    if (!teleopEntry) {
      Logger.recordOutput(
          "Replay/Anchor Error/Translation Meters",
          actual.getTranslation().getDistance(target.getTranslation()));
      Logger.recordOutput(
          "Replay/Anchor Error/Rotation Degrees",
          Math.abs(actual.getRotation().minus(target.getRotation()).getDegrees()));
      Logger.recordOutput("Replay/Anchor Error/Window Seconds", now - lastAnchorSeconds);
    }

    // maple-sim's setSimulationWorldPose zeroes linear velocity:
    //     super.transform.set(...); super.linearVelocity.set(0, 0);
    // Teleporting without restoring velocity would stop the robot dead 14 times a
    // match and force the drive motors to re-accelerate from standstill each time,
    // injecting current spikes that never happened. Capture the speeds first and
    // put them back afterwards so only position is corrected.
    //
    // Field-relative is the correct frame here: setRobotSpeeds converts through
    // toDyn4jLinearVelocity into dyn4j world coordinates.
    ChassisSpeeds speeds = driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative();
    driveSimulation.setSimulationWorldPose(target);
    driveSimulation.setRobotSpeeds(speeds);

    lastAnchorSeconds = now;
    anchoredAtTeleopStart = true;
  }
}
```

- [ ] **Step 2: Wire it into `RobotContainer`**

Add the import and a field:

```java
import frc.robot.utility.replay.PoseAnchor;
```

```java
  private PoseAnchor poseAnchor;
```

Add a setter that `Robot` calls once the player exists:

```java
  /** Attach the replay pose anchor. SIM only; no-op if the drivetrain sim is absent. */
  public void attachPoseAnchor(double intervalSeconds) {
    if (Constants.getRobotMode() != Constants.Mode.SIM) {
      return;
    }
    poseAnchor =
        new PoseAnchor(RobotSimState.getInstance().getDriveSimulation(), intervalSeconds);
  }

  /** Run the replay pose anchor for this loop. */
  public void updatePoseAnchor(frc.robot.utility.replay.LogInputPlayer player) {
    if (poseAnchor != null && player != null) {
      poseAnchor.update(player);
    }
  }
```

- [ ] **Step 3: Call it from `Robot`**

In `robotInit()`, after `robotContainer = new RobotContainer();`:

```java
    if (replayPlayer != null) {
      robotContainer.attachPoseAnchor(
          Double.parseDouble(System.getProperty("ai.replay.anchor", "10.0")));
    }
```

In `robotPeriodic()`, after `replayPlayer.step(...)`:

```java
      robotContainer.updatePoseAnchor(replayPlayer);
```

- [ ] **Step 4: Implement `-Preplay.teleopOnly`**

The Gradle flag was wired in Task 11 but nothing consumes it yet. In `Robot.robotInit()`, after `attachPoseAnchor`, add:

```java
      if (Boolean.getBoolean("ai.replay.teleopOnly")) {
        double[] teleopPose = replayPlayer.seekToTeleop();
        if (teleopPose != null) {
          // Place the robot where the real one finished auto, otherwise teleop
          // would start from the drivetrain's initialisation pose instead.
          RobotSimState.getInstance()
              .getDriveSimulation()
              .setSimulationWorldPose(
                  new Pose2d(
                      teleopPose[0], teleopPose[1], new Rotation2d(teleopPose[2])));
          System.out.println(
              "[Replay] teleopOnly: skipped to t="
                  + String.format("%.1f", replayPlayer.elapsedSeconds())
                  + "s");
        } else {
          System.err.println("[Replay] teleopOnly requested but log has no teleop transition");
        }
      }
```

Add the imports `edu.wpi.first.math.geometry.Rotation2d` and `frc.robot.RobotSimState` if not already present (`Pose2d` is already imported in `Robot.java`).

- [ ] **Step 5: Verify it compiles**

```bash
./gradlew compileJava --no-daemon
```

Expected: BUILD SUCCESSFUL

- [ ] **Step 6: Run and confirm anchor error is logged**

```bash
./gradlew simulateJava --no-daemon -Pheadless -Pai.logging -Preplay.fast \
  "-Preplay.inputs=C:\\Users\\bruce\\Downloads\\LOGS-2026-main\\LOGS-2026-main\\Worlds\\akit_26-04-30_14-50-56_johnson_q54.wpilog"
python scripts/wpilog_to_csv.py build/ai-logs/<NEWEST>.wpilog --keys "RealOutputs/Replay/Anchor Error/Translation Meters"
```

Expected: roughly 14 records (one per 10 s of a ~140 s teleop), each a positive drift in metres.

- [ ] **Step 7: Format and commit**

```bash
./gradlew spotlessApply --no-daemon
git add src/main/java/frc/robot/utility/replay/PoseAnchor.java src/main/java/frc/robot/RobotContainer.java src/main/java/frc/robot/Robot.java
git commit -m "Anchor sim pose to logged pose every 10s and publish drift"
```

---

## Phase C — Battery Model

### Task 13: `SimBattery`

**Files:**
- Create: `src/main/java/frc/robot/utility/SimBattery.java`
- Test: `src/test/java/frc/robot/utility/SimBatteryTest.java`

- [ ] **Step 1: Write the failing test**

Create `src/test/java/frc/robot/utility/SimBatteryTest.java`:

```java
package frc.robot.utility;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class SimBatteryTest {

  @BeforeEach
  void reset() {
    SimBattery.getInstance().reset(12.8, 0.02);
  }

  @Test
  void noLoadSitsAtNominalVoltage() {
    assertEquals(12.8, SimBattery.getInstance().computeVoltage(), 1e-6);
  }

  @Test
  void loadSagsProportionalToCurrentAndResistance() {
    SimBattery.getInstance().register(() -> 100.0);
    // 12.8 V - (100 A * 0.02 ohm) = 10.8 V
    assertEquals(10.8, SimBattery.getInstance().computeVoltage(), 1e-6);
  }

  @Test
  void multipleSourcesSum() {
    SimBattery.getInstance().register(() -> 50.0);
    SimBattery.getInstance().register(() -> 50.0);
    assertEquals(10.8, SimBattery.getInstance().computeVoltage(), 1e-6);
  }

  @Test
  void voltageNeverGoesBelowFloor() {
    SimBattery.getInstance().register(() -> 10_000.0);
    assertTrue(SimBattery.getInstance().computeVoltage() >= SimBattery.MIN_VOLTAGE);
  }

  @Test
  void negativeCurrentFromRegenIsIgnored() {
    // A motor braking can report negative draw; it must not inflate pack voltage
    // above nominal, which would be unphysical for our purposes.
    SimBattery.getInstance().register(() -> -100.0);
    assertEquals(12.8, SimBattery.getInstance().computeVoltage(), 1e-6);
  }

  @Test
  void brownoutFlagsBelowThreshold() {
    SimBattery.getInstance().register(() -> 400.0); // 12.8 - 8.0 = 4.8 V
    SimBattery.getInstance().computeVoltage();
    assertTrue(SimBattery.getInstance().isBrownedOut());
  }

  @Test
  void noBrownoutUnderLightLoad() {
    SimBattery.getInstance().register(() -> 10.0);
    SimBattery.getInstance().computeVoltage();
    assertFalse(SimBattery.getInstance().isBrownedOut());
  }

  @Test
  void resetClearsRegisteredSources() {
    SimBattery.getInstance().register(() -> 100.0);
    SimBattery.getInstance().reset(12.8, 0.02);
    assertEquals(12.8, SimBattery.getInstance().computeVoltage(), 1e-6);
  }

  @Test
  void parsesConfigString() {
    SimBattery.getInstance().configureFromProperty("12.5:0.015");
    SimBattery.getInstance().register(() -> 100.0);
    assertEquals(11.0, SimBattery.getInstance().computeVoltage(), 1e-6);
  }

  @Test
  void ignoresMalformedConfigString() {
    SimBattery.getInstance().configureFromProperty("nonsense");
    assertEquals(12.8, SimBattery.getInstance().computeVoltage(), 1e-6);
  }
}
```

- [ ] **Step 2: Run test to verify it fails**

```bash
./gradlew test --no-daemon --tests "frc.robot.utility.SimBatteryTest"
```

Expected: FAIL — `cannot find symbol: class SimBattery`

- [ ] **Step 3: Write minimal implementation**

Create `src/main/java/frc/robot/utility/SimBattery.java`:

```java
package frc.robot.utility;

import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Simulated battery with load-dependent voltage sag.
 *
 * <p>Before this existed the simulation ran at a fixed 12 V, so brownouts were impossible and
 * motors never lost torque under load. Real match logs show the pack dropping to 5.95 V, and in the
 * one match where it stayed low the flywheel measurably failed to reach speed — so the sag has to
 * feed back into motor behaviour, not merely be reported.
 *
 * <p>That feedback comes for free: every {@code *IOSim} already calls {@code
 * setSupplyVoltage(RobotController.getBatteryVoltage())}, and {@link RoboRioSim#setVInVoltage} is
 * what that reads. Publishing the sagged voltage here reaches every motor on the robot.
 *
 * <p>Model: {@code V = nominal - I_total * R_internal}. Nominal voltage and internal resistance are
 * per-match tunable because battery condition varied between matches.
 */
public final class SimBattery {

  /** Absolute floor; a real pack under a dead short still holds some potential. */
  public static final double MIN_VOLTAGE = 4.0;

  /** roboRIO brownout threshold. */
  public static final double BROWNOUT_VOLTAGE = 6.8;

  public static final double DEFAULT_NOMINAL_VOLTS = 12.8;
  public static final double DEFAULT_RESISTANCE_OHMS = 0.02;

  private static SimBattery instance;

  private final List<DoubleSupplier> currentSources = new ArrayList<>();
  private double nominalVolts = DEFAULT_NOMINAL_VOLTS;
  private double resistanceOhms = DEFAULT_RESISTANCE_OHMS;
  private double lastVoltage = DEFAULT_NOMINAL_VOLTS;
  private double lastCurrentAmps = 0.0;
  private boolean brownedOut = false;

  private SimBattery() {}

  public static synchronized SimBattery getInstance() {
    if (instance == null) {
      instance = new SimBattery();
    }
    return instance;
  }

  /** Register a supply-current source, in amps. Called once per simulated motor at construction. */
  public void register(DoubleSupplier supplyCurrentAmps) {
    currentSources.add(supplyCurrentAmps);
  }

  /** Clear all sources and set new pack parameters. Intended for tests. */
  public void reset(double nominalVolts, double resistanceOhms) {
    currentSources.clear();
    this.nominalVolts = nominalVolts;
    this.resistanceOhms = resistanceOhms;
    this.lastVoltage = nominalVolts;
    this.lastCurrentAmps = 0.0;
    this.brownedOut = false;
  }

  /**
   * Apply a {@code "<nominalVolts>:<resistanceOhms>"} configuration string, as supplied by
   * {@code -Preplay.battery}. Malformed input is ignored so a typo degrades to defaults rather than
   * crashing a long simulation run.
   */
  public void configureFromProperty(String property) {
    if (property == null || property.isBlank()) {
      return;
    }
    String[] parts = property.split(":");
    if (parts.length != 2) {
      System.err.println("[SimBattery] Ignoring malformed battery config: " + property);
      return;
    }
    try {
      nominalVolts = Double.parseDouble(parts[0]);
      resistanceOhms = Double.parseDouble(parts[1]);
      System.out.println(
          "[SimBattery] nominal=" + nominalVolts + "V, R=" + resistanceOhms + " ohm");
    } catch (NumberFormatException e) {
      System.err.println("[SimBattery] Ignoring malformed battery config: " + property);
    }
  }

  public double lastVoltage() {
    return lastVoltage;
  }

  public boolean isBrownedOut() {
    return brownedOut;
  }

  /** Sum the registered sources and return the resulting pack voltage. */
  public double computeVoltage() {
    double total = 0.0;
    for (DoubleSupplier source : currentSources) {
      double amps = source.getAsDouble();
      // Braking motors can report negative draw. Treating that as a recharge
      // would push the pack above nominal, which is not a behaviour we want.
      if (amps > 0.0 && Double.isFinite(amps)) {
        total += amps;
      }
    }
    lastCurrentAmps = total;
    lastVoltage = Math.max(MIN_VOLTAGE, nominalVolts - total * resistanceOhms);
    brownedOut = lastVoltage < BROWNOUT_VOLTAGE;
    return lastVoltage;
  }

  /**
   * Compute the pack voltage, publish it to the simulated roboRIO so every motor sees it, and log
   * it. Call once per simulation loop.
   */
  public void update() {
    double voltage = computeVoltage();
    RoboRioSim.setVInVoltage(voltage);
    RoboRioSim.setBrownoutVoltage(BROWNOUT_VOLTAGE);
    Logger.recordOutput("SimBattery/Voltage", voltage);
    Logger.recordOutput("SimBattery/TotalCurrentAmps", lastCurrentAmps);
    Logger.recordOutput("SimBattery/BrownedOut", brownedOut);
    Logger.recordOutput("SimBattery/SourceCount", currentSources.size());
  }
}
```

- [ ] **Step 4: Run test to verify it passes**

```bash
./gradlew test --no-daemon --tests "frc.robot.utility.SimBatteryTest"
```

Expected: BUILD SUCCESSFUL, 10 tests passed

- [ ] **Step 5: Format and commit**

```bash
./gradlew spotlessApply --no-daemon
git add src/main/java/frc/robot/utility/SimBattery.java src/test/java/frc/robot/utility/SimBatteryTest.java
git commit -m "Add simulated battery with load-dependent voltage sag"
```

---

### Task 14: Register current sources and tick the battery

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/intake/intake_rack/IntakeRackIOSim.java`
- Modify: `src/main/java/frc/robot/subsystems/intake/intake_rollers/IntakeRollersIOSim.java`
- Modify: `src/main/java/frc/robot/subsystems/shooter/serializer/SerializerSim.java`
- Modify: `src/main/java/frc/robot/subsystems/shooter/shooter_accelerator/ShooterAcceleratorIOSim.java`
- Modify: `src/main/java/frc/robot/subsystems/shooter/shooter_flywheel/ShooterFlywheelIOSim.java`
- Modify: `src/main/java/frc/robot/subsystems/shooter/shooter_hood/ShooterHoodIOSim.java`
- Modify: `src/main/java/frc/robot/subsystems/shooter/shooter_omniwheel/ShooterOmniwheelIOSim.java`
- Modify: `src/main/java/frc/robot/subsystems/swerve/ModuleIOTalonFXSim.java`
- Modify: `src/main/java/frc/robot/RobotContainer.java` (`updateSimulation()`)

- [ ] **Step 1: Add a cached-current field to each mechanism sim IO**

In **each** of the seven mechanism `*IOSim` files, add this field:

```java
  /** Last computed supply current, published to SimBattery. */
  private double lastSupplyCurrentAmps = 0.0;
```

Add this line at the **end** of that file's constructor:

```java
    frc.robot.utility.SimBattery.getInstance().register(() -> lastSupplyCurrentAmps);
```

Add this line at the **end** of that file's `updateInputs(...)` method:

```java
    lastSupplyCurrentAmps = inputs.supplyCurrentAmps;
```

- [ ] **Step 2: Register the swerve modules**

In `ModuleIOTalonFXSim.java`, add to the end of the constructor:

```java
    // Phoenix models supply current for the maple-sim-driven modules directly.
    frc.robot.utility.SimBattery.getInstance()
        .register(() -> driveTalon.getSimState().getSupplyCurrent());
    frc.robot.utility.SimBattery.getInstance()
        .register(() -> steerTalon.getSimState().getSupplyCurrent());
```

If `driveTalon` and `steerTalon` are private in `ModuleIOTalonFX`, change them to `protected`.

- [ ] **Step 3: Tick the battery each simulation loop**

In `RobotContainer.updateSimulation()`, add as the **first** statement after the mode guard, so voltage is set before any IO reads it this loop:

```java
    SimBattery.getInstance().update();
```

Add the import:

```java
import frc.robot.utility.SimBattery;
```

- [ ] **Step 4: Apply the per-run battery configuration**

In `Robot.robotInit()`, inside the replay-loading block, before constructing the player:

```java
        SimBattery.getInstance()
            .configureFromProperty(System.getProperty("ai.replay.battery"));
```

with the import `import frc.robot.utility.SimBattery;`.

- [ ] **Step 5: Verify it compiles and tests still pass**

```bash
./gradlew compileJava --no-daemon && ./gradlew test --no-daemon
```

Expected: BUILD SUCCESSFUL

- [ ] **Step 6: Confirm the battery actually sags in a run**

```bash
./gradlew simulateJava --no-daemon -Pheadless -Pai.logging -Preplay.fast \
  "-Preplay.inputs=C:\\Users\\bruce\\Downloads\\LOGS-2026-main\\LOGS-2026-main\\Worlds\\akit_26-04-30_14-50-56_johnson_q54.wpilog"
python scripts/wpilog_to_csv.py build/ai-logs/<NEWEST>.wpilog \
  --keys "RealOutputs/SimBattery/Voltage,RealOutputs/SimBattery/TotalCurrentAmps,RealOutputs/SimBattery/SourceCount"
```

Expected: `SourceCount` is 15 (7 mechanisms + 8 swerve motors). Voltage varies rather than sitting flat at 12.8. If `SourceCount` is lower than 15, a registration was missed.

- [ ] **Step 7: Format and commit**

```bash
./gradlew spotlessApply --no-daemon
git add -A src/main/java
git commit -m "Feed simulated motor currents into battery voltage sag model"
```

---

### Task 15: Fix the sim IO current and voltage defects

These are the confirmed defects from the investigation table. Each one independently prevents power draw from matching.

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/intake/intake_rollers/IntakeRollersIOSim.java:69`
- Modify: `src/main/java/frc/robot/subsystems/shooter/shooter_accelerator/ShooterAcceleratorIOSim.java:70`
- Modify: `src/main/java/frc/robot/subsystems/shooter/shooter_flywheel/ShooterFlywheelIOSim.java:69`
- Modify: `src/main/java/frc/robot/subsystems/shooter/shooter_omniwheel/ShooterOmniwheelIOSim.java:70`
- Modify: `src/main/java/frc/robot/subsystems/intake/intake_rack/IntakeRackIOSim.java:76`
- Modify: `src/main/java/frc/robot/subsystems/shooter/serializer/SerializerSim.java:55`

- [ ] **Step 1: Clamp applied voltage to the actual battery, not a hardcoded 12 V**

In each of the four roller-style sim IOs, replace:

```java
    appliedVoltage = Math.max(-12, Math.min(12, appliedVoltage)); // Clamp to battery voltage
```

with:

```java
    // Clamp to the battery's ACTUAL voltage so sag reduces available torque.
    // The old hardcoded +/-12 made brownouts cosmetic: the pack could read 6 V
    // while the motor still behaved as though it had a full 12 V to work with.
    double availableVolts = RobotController.getBatteryVoltage();
    appliedVoltage = Math.max(-availableVolts, Math.min(availableVolts, appliedVoltage));
```

`RobotController` is already imported in all four files.

- [ ] **Step 2: Report supply current rather than stator current**

In the same four files, replace the single line

```java
    inputs.supplyCurrentAmps = Math.abs(SIM_FIELD.getCurrentDrawAmps());
```

with the block below, using each file's own sim field name:

| File | `SIM_FIELD` |
| --- | --- |
| `IntakeRollersIOSim.java` | `intakeRollersSim` |
| `ShooterAcceleratorIOSim.java` | `shooterAcceleratorSim` |
| `ShooterFlywheelIOSim.java` | `shooterFlywheelsSim` |
| `ShooterOmniwheelIOSim.java` | `shooterOmniwheelsSim` |

```java
    // getCurrentDrawAmps() is stator current. Supply current is lower by roughly
    // the duty cycle, since the motor controller is a buck converter. Reporting
    // stator as supply overstates pack draw and would make the battery model sag
    // far harder than the real robot does.
    double statorAmps = Math.abs(SIM_FIELD.getCurrentDrawAmps());
    double dutyCycle =
        availableVolts > 0.0 ? Math.abs(appliedVoltage) / availableVolts : 0.0;
    inputs.statorCurrentAmps = statorAmps;
    inputs.supplyCurrentAmps = statorAmps * dutyCycle;
```

`availableVolts` and `appliedVoltage` are both already in scope from Step 1.

Note `GenericRollersIOInputs` already declares a `statorCurrentAmps` field that no sim IO ever populated; this fills it, so the sim log gains the `StatorCurrentAmps` keys the real logs have.

- [ ] **Step 3: Simulate the intake rack's current instead of faking 1.0 A**

In `IntakeRackIOSim.updateInputs`, replace:

```java
    inputs.supplyCurrentAmps = 1.0; // Not simulated
```

with:

```java
    double availableVolts = RobotController.getBatteryVoltage();
    double statorAmps = Math.abs(intakeRackSim.getCurrentDrawAmps());
    double dutyCycle = availableVolts > 0.0 ? Math.abs(appliedVoltage) / availableVolts : 0.0;
    inputs.statorCurrent = statorAmps;
    inputs.supplyCurrentAmps = statorAmps * dutyCycle;
```

- [ ] **Step 4: Do the same for the serializer**

`SerializerSim` names its voltage variable `appliedVelocity` even though it holds `talon.getSimState().getMotorVoltage()` — a voltage. It also sets `inputs.appliedVelocity` and never sets `inputs.appliedVolts`, so the sim log is missing the `Serializer/AppliedVolts` key the real logs have, which makes that signal uncomparable.

In `SerializerSim.updateInputs`, replace:

```java
    inputs.connected = true;
    inputs.velocityRadsPerSec = velocityRPS;
    inputs.appliedVelocity = appliedVelocity;
    inputs.supplyCurrentAmps = 1.0; // Not simulated
```

with:

```java
    double availableVolts = RobotController.getBatteryVoltage();
    double statorAmps = Math.abs(serializerSim.getCurrentDrawAmps());
    double dutyCycle = availableVolts > 0.0 ? Math.abs(appliedVelocity) / availableVolts : 0.0;

    inputs.connected = true;
    inputs.velocityRadsPerSec = velocityRPS;
    inputs.appliedVelocity = appliedVelocity;
    // appliedVelocity actually holds a voltage (getMotorVoltage). Publish it as
    // appliedVolts too so the sim log carries the same key the real logs do.
    inputs.appliedVolts = appliedVelocity;
    inputs.statorCurrentAmps = statorAmps;
    inputs.supplyCurrentAmps = statorAmps * dutyCycle;
```

- [ ] **Step 5: Verify it compiles and tests pass**

```bash
./gradlew compileJava --no-daemon && ./gradlew test --no-daemon
```

Expected: BUILD SUCCESSFUL

- [ ] **Step 6: Format and commit**

```bash
./gradlew spotlessApply --no-daemon
git add -A src/main/java/frc/robot/subsystems
git commit -m "Fix hardcoded voltage clamps and fake supply currents in sim IOs"
```

---

## Phase D — Calibration Loop

### Task 16: Baseline comparison on the primary log

**Files:** none modified — this is a measurement task.

- [ ] **Step 1: Run the replay**

```bash
./gradlew simulateJava --no-daemon -Pheadless -Pai.logging -Preplay.fast \
  "-Preplay.inputs=C:\\Users\\bruce\\Downloads\\LOGS-2026-main\\LOGS-2026-main\\Worlds\\akit_26-04-30_14-50-56_johnson_q54.wpilog"
```

If `-Preplay.fast` produces unstable physics (NaN poses, absurd velocities), drop the flag and accept ~3 min per run. Note which was used.

- [ ] **Step 2: Compare against the real log**

```bash
python scripts/wpilog_to_csv.py --compare \
  build/ai-logs/<NEWEST>.wpilog \
  "C:\Users\bruce\Downloads\LOGS-2026-main\LOGS-2026-main\Worlds\akit_26-04-30_14-50-56_johnson_q54.wpilog" \
  --json build/ai-logs/fit-q54-baseline.json
```

- [ ] **Step 3: Read the drivetrain drift separately**

`Replay/Anchor Error` exists only in the sim log, so `--compare` cannot score it — it has nothing on the real side to compare against. Read it directly:

```bash
python scripts/wpilog_to_csv.py build/ai-logs/<NEWEST>.wpilog \
  --keys "RealOutputs/Replay/Anchor Error/Translation Meters,RealOutputs/Replay/Anchor Error/Rotation Degrees"
```

This is the 5th fidelity target from the spec. Record the mean and max drift alongside the category scores.

- [ ] **Step 4: Record the baseline**

Save the JSON as the reference every later iteration is measured against. Every subsequent tuning change must improve at least one category score without regressing the others.

- [ ] **Step 5: Commit the baseline**

`build/` is gitignored, so copy the JSON somewhere tracked before committing it:

```bash
mkdir -p docs/superpowers/fit-scores
cp build/ai-logs/fit-q54-baseline.json docs/superpowers/fit-scores/
git add docs/superpowers/fit-scores/fit-q54-baseline.json
git commit -m "Record baseline sim-vs-real fit scores for q54"
```

- [ ] **Step 6: CHECKPOINT — report to the user**

Report the category fit scores and the top diverging signals before proceeding. Per the spec, this is a checkpoint: do not begin tuning without showing the baseline first.

---

### Task 17: Dispatch analysis agents

- [ ] **Step 1: Dispatch 5 parallel agents**

Give each agent the comparison report, the baseline JSON, and one focus area:

1. Drivetrain — `mapleSimConfig` mass, MOI, wheel COF, module current, `Replay/Anchor Error` drift
2. Shooter — flywheel MOI, the reimplemented PID in `ShooterFlywheelIOSim`, spin-up curves
3. Intake — rack `ElevatorSim` mass and drum radius, roller MOI, deploy/retract timing
4. Battery and power — nominal voltage, internal resistance, aggregate amp-seconds
5. Cross-cutting — loop timing, update ordering, `Constants.PERIODIC_LOOP_SEC` mismatches, anything the other four would miss

Each agent reports specific files, constants, and evidence from the comparison. **Agents propose; they do not edit.**

- [ ] **Step 2: Compile findings and CHECKPOINT**

Deduplicate, rank by expected impact, and separate SIM-only changes from any that touch shared COMP constants. Per the spec, shared-constant changes require presenting evidence and getting approval **before** making them.

---

### Task 18: Iterate to convergence

- [ ] **Step 1: Apply one coherent group of changes**

Change one subsystem's parameters at a time. Changing several at once makes it impossible to attribute an improvement.

- [ ] **Step 2: Re-run and re-compare**

```bash
./gradlew simulateJava --no-daemon -Pheadless -Pai.logging -Preplay.fast "-Preplay.inputs=<PRIMARY_LOG>"
python scripts/wpilog_to_csv.py --compare build/ai-logs/<NEWEST>.wpilog "<PRIMARY_LOG>" --json build/ai-logs/fit-q54-iterN.json
```

- [ ] **Step 3: Keep or revert**

Compare `iterN` against the previous best. Keep the change only if a category improved and none regressed meaningfully. Revert otherwise. Commit each kept change separately with the score delta in the message.

- [ ] **Step 4: Repeat until scores plateau**

- [ ] **Step 5: CHECKPOINT — report convergence on q54**

---

### Task 19: Validate and generalize

- [ ] **Step 1: Run q14 (browning) and q93 (clean)**

```bash
./gradlew simulateJava --no-daemon -Pheadless -Pai.logging -Preplay.fast "-Preplay.inputs=<VALIDATION_LOG_1>"
python scripts/wpilog_to_csv.py --compare build/ai-logs/<NEWEST>.wpilog "<VALIDATION_LOG_1>" --json build/ai-logs/fit-q14.json
```

Repeat for q93.

- [ ] **Step 2: Fit battery parameters per log**

Sweep `-Preplay.battery` per match and record the best nominal/resistance for each. Report the spread — a wide spread means the battery model is absorbing error that belongs elsewhere.

- [ ] **Step 3: Re-tune only what generalizes**

Any change that improved q54 but hurts q14 or q93 was overfitting. Revert it.

- [ ] **Step 4: CHECKPOINT — report validation results**

---

### Task 20: Held-out check and documentation

- [ ] **Step 1: Run q103 once, with no tuning afterward**

```bash
./gradlew simulateJava --no-daemon -Pheadless -Pai.logging -Preplay.fast "-Preplay.inputs=<HELDOUT_LOG>"
python scripts/wpilog_to_csv.py --compare build/ai-logs/<NEWEST>.wpilog "<HELDOUT_LOG>" --json build/ai-logs/fit-q103-heldout.json
```

Whatever this shows is the honest result. Do not tune against it.

- [ ] **Step 2: Run the q64 brownout stress case**

Confirm the sim reproduces the flywheel degradation the real robot showed — up-to-speed time should fall well below the ~158 s typical of other matches.

- [ ] **Step 3: Update `.claude/commands/simulation-agent.md`**

Add a "Log-Driven Replay" section documenting: the `-Preplay.*` flags, that pose is only trustworthy during auto, that `Replay/Anchor Error` measures drivetrain fidelity, and the per-log battery parameters found in Task 19.

- [ ] **Step 4: Update `.claude/commands/log-analysis.md`**

Add a "Comparison Mode" section documenting `--compare`, `--json`, `--top`, and how to read the category fit scores.

- [ ] **Step 5: Commit**

```bash
./gradlew spotlessApply --no-daemon
git add -A
git commit -m "Document log-driven replay and comparison workflows"
```

- [ ] **Step 6: Final report**

Report per-log category fit scores, the battery parameter spread, every COMP-affecting constant changed, and an honest statement of where the sim still does not match.
