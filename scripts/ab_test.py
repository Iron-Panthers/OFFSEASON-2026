"""
A/B test a robot code change in simulation.

Runs a benchmark under two arms at matched random seeds, pairs the results, and reports whether
the difference is real or is just simulation noise.

The whole design exists because the simulation is not deterministic: shot spread and hub dispersal
draw from RNGs, so two runs of identical code score differently. Comparing one run against one run
measures the dice. This script instead runs each arm at the same seeds and compares pairs, which
removes the shared noise and leaves the effect of the change.

Usage
-----
    # Compare your uncommitted work against the last commit. The baseline is checked out into a
    # temporary worktree, so your working tree is never modified.
    python scripts/ab_test.py --bench shotgrid --baseline-ref HEAD

    # Compare against any revision.
    python scripts/ab_test.py --bench auto --auto "2xTBTB Right" --baseline-ref main

    # Noise floor check. Always do this for a new benchmark before trusting any result.
    python scripts/ab_test.py --bench drain --aa
"""

import argparse
import contextlib
import json
import math
import os
import statistics
import subprocess
import sys
import time
from bisect import bisect_right
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from wpilog_to_csv import read_log  # noqa: E402

REPO = Path(__file__).resolve().parent.parent
LOG_DIR = REPO / "build" / "ai-logs"
ARTIFACT_DIR = REPO / "build" / "artifacts"
NOISE_FLOOR_CACHE = REPO / "build" / "artifacts" / "noise-floors.json"

MIN_PAIRS = 3
MAX_PAIRS = 8

# Paired t threshold for a DECISIVE call. Deliberately stricter than the nominal 95% critical
# value at n=3 (2.92): with this few samples the variance estimate is itself noisy, and a false
# "this change helps" is more costly than another five minutes of simulation.
T_DECISIVE = 3.0
T_LEAN = 1.8

# An enabled window shorter than this means the auto never really ran. See run_trial.
MIN_AUTO_SECONDS = 3.0


# ----------------------------------------------------------------------


def _last(series, default=0.0):
    """Final value of a series, which for a monotonic counter is its total."""
    return series[-1][1] if series else default


def _series_stats(series):
    values = [v for _, v in series if isinstance(v, (int, float))]
    if not values:
        return {}
    return {
        "min": min(values),
        "max": max(values),
        "mean": statistics.fmean(values),
    }


def extract_metrics(log_path, bench):
    """
    Pull the metric set for `bench` out of one .wpilog.

    Returns {metric_name: float}. Every benchmark reports guardrail metrics alongside its primary
    one, because a change that lifts score while wrecking current draw or path tracking is not an
    improvement and the report must be able to say so.
    """
    data = read_log(str(log_path))

    def key(name):
        return data.get(f"RealOutputs/{name}", data.get(name, []))

    score = _last(key("Field Simulation/Score"))
    fired = _last(key("Field Simulation/Shots Fired"))
    duration = max((s[-1][0] for s in data.values() if s), default=0.0)
    enabled = key("DriverStation/Enabled")
    first_enable = next((ts for ts, v in enabled if v), 0.0)

    metrics = {
        "score": float(score),
        "shots_fired": float(fired),
        "accuracy": float(score) / float(fired) if fired else 0.0,
        "duration_s": duration - first_enable,
    }

    # Guardrails: shared across every benchmark.
    battery = _series_stats(key("SimBattery/Voltage"))
    current = _series_stats(key("SimBattery/TotalCurrentAmps"))
    if battery:
        metrics["min_voltage"] = battery["min"]
    if current:
        metrics["mean_current_a"] = current["mean"]

    if bench == "shotgrid":
        metrics["hit_rate"] = float(_last(key("AB/ShotGrid/HitRate")))
        metrics["hits"] = float(_last(key("AB/ShotGrid/Hits")))
        # Shots taken vs shots offered. A change that makes the robot refuse shots it used to
        # take shows up here and nowhere else -- hit_rate alone would just look like bad aim.
        metrics["shots_taken"] = float(_last(key("AB/ShotGrid/Fired")))
        metrics["precision"] = float(_last(key("AB/ShotGrid/Precision")))
        # Per-distance outcome, so the report can say *where* on the curve the change acted.
        for i in range(32):
            d = key(f"AB/ShotGrid/Shot{i}/Distance")
            hit = key(f"AB/ShotGrid/Shot{i}/Scored")
            if not d:
                break
            metrics[f"hit@{_last(d):.1f}m"] = 1.0 if _last(hit, False) else 0.0

    elif bench == "drain":
        empty_at = _last(key("AB/Drain/EmptyAt"))
        preloaded = _last(key("AB/Drain/Preloaded"))
        metrics["time_to_empty_s"] = float(empty_at)
        metrics["preloaded"] = float(preloaded)
        # NOT a throughput measurement, despite the name. RobotSimState fires on a fixed
        # 20 shots/sec timer (see setShooterRunning); the mechanisms only gate WHETHER it
        # fires, not how fast. This moves when a change drops a mechanism below the shooting
        # gate, and is flat otherwise -- useful as a guardrail, misleading as a headline.
        metrics["balls_per_sec"] = float(preloaded) / float(empty_at) if empty_at else 0.0
        if _last(key("AB/Drain/TimedOut"), False):
            metrics["timed_out"] = 1.0

    elif bench == "path":
        # PathPlanner logs the commanded and the measured pose under inconsistent key spellings
        # ("PathPlanner/TargetPose" vs "Path Planner/Current Pose"). The gap between them is the
        # tracking error, and it is what a drive gain change actually moves. Nothing logs the
        # error directly, so compute it here.
        target = key("PathPlanner/TargetPose")
        current = key("Path Planner/Current Pose")
        errors = _pose_errors(current, target)
        if errors:
            metrics["path_err_mean_m"] = statistics.fmean(errors)
            metrics["path_err_max_m"] = max(errors)

    elif bench == "step":
        vel = key("Shooter/Shooter Flywheels/Current Velocity")
        sp = key("Shooter/Shooter Flywheels/Target Velocity")
        rise = _rise_time(vel, sp)
        if rise is not None:
            metrics["rise_time_s"] = rise
        stats = _series_stats(vel)
        if stats:
            metrics["peak_velocity"] = stats["max"]

    return metrics


def _pose_errors(current, target):
    """
    Distance between the measured and commanded pose at each measured sample.

    Zero-order-holds the target onto the current series' timestamps, since the two callbacks fire
    independently and their samples do not line up.
    """
    if not current or not target:
        return []
    t_times = [t for t, _ in target]
    errors = []
    for ts, cur in current:
        if cur is None:
            continue
        idx = bisect_right(t_times, ts) - 1
        if idx < 0:
            continue
        tgt = target[idx][1]
        if tgt is None:
            continue
        errors.append(math.hypot(cur[0] - tgt[0], cur[1] - tgt[1]))
    return errors


def _rise_time(actual, setpoint):
    """
    Seconds from the setpoint going non-zero to the measurement first reaching 95% of it.

    Returns None when the mechanism never got there, which the report shows as a missing value
    rather than as a zero -- a mechanism that never reached setpoint is not a fast one.
    """
    if not actual or not setpoint:
        return None
    target = max((v for _, v in setpoint if isinstance(v, (int, float))), default=0.0)
    if target <= 0:
        return None
    start = next((ts for ts, v in setpoint if isinstance(v, (int, float)) and v > 0), None)
    if start is None:
        return None
    reached = next(
        (ts for ts, v in actual if ts >= start and isinstance(v, (int, float)) and v >= 0.95 * target),
        None,
    )
    return None if reached is None else reached - start


# ----------------------------------------------------------------------


def _newest_log(log_dir, before):
    """The .wpilog that appeared in `log_dir` since `before`, or None."""
    now = {p: p.stat().st_mtime for p in log_dir.glob("*.wpilog")}
    fresh = [p for p in now if p not in before]
    if not fresh:
        return None
    return max(fresh, key=lambda p: now[p])


def run_trial(bench, seed, args, label, tree=None):
    """
    Runs one simulation trial and returns (metrics, log_path).

    `tree` is the checkout to run in, which for git mode is a temporary worktree holding the
    baseline revision. Each tree has its own build/ai-logs, so log discovery is scoped to it.

    Raises RuntimeError when the sim produced no log, which means the run crashed. That must stop
    the sweep: silently dropping failed trials would bias the result toward whichever arm happens
    to survive more often.
    """
    tree = Path(tree) if tree else REPO
    log_dir = tree / "build" / "ai-logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    before = {p: p.stat().st_mtime for p in log_dir.glob("*.wpilog")}

    cmd = [
        str(tree / "gradlew.bat") if os.name == "nt" else str(tree / "gradlew"),
        "simulateJava",
        "--no-daemon",
        "-Pheadless",
        "-Pai.logging",
        f"-Psim.seed={seed}",
    ]
    if bench in ("drain", "shotgrid"):
        cmd.append(f"-Pbench={bench}")
        if args.count:
            cmd.append(f"-Pbench.count={args.count}")
        if args.distances:
            cmd.append(f"-Pbench.distances={args.distances}")
    elif bench in ("auto", "path"):
        cmd.append(f"-Pauto.name={args.auto}")
    elif bench == "step":
        cmd.append(f"-Pteleop.duration={args.duration}")
        cmd.append(f"-Pteleop.buttons={args.buttons}")

    started = time.time()
    proc = subprocess.run(
        cmd, cwd=str(tree), capture_output=True, text=True, timeout=args.trial_timeout
    )
    elapsed = time.time() - started

    log = _newest_log(log_dir, before)
    if log is None:
        tail = "\n".join((proc.stdout + proc.stderr).splitlines()[-25:])
        raise RuntimeError(f"{label} seed {seed} produced no log after {elapsed:.0f}s:\n{tail}")

    metrics = extract_metrics(log, bench)

    # A name PathPlanner does not recognise yields an empty command that "finishes" at once, so
    # the run ends after about a second having done nothing. Both arms would then score zero and
    # the sweep would confidently report "no difference" between two runs that never happened.
    if bench in ("auto", "path") and metrics.get("duration_s", 0.0) < MIN_AUTO_SECONDS:
        raise RuntimeError(
            f"{label} seed {seed}: auto '{args.auto}' ran only "
            f"{metrics.get('duration_s', 0.0):.1f}s. The name is probably wrong -- "
            f"PathPlanner builds an empty command for an unknown auto rather than failing. "
            f"Auto names in this repo contain spaces; check src/main/deploy/pathplanner/autos/."
        )

    print(f"    {label} seed={seed}  {elapsed:5.0f}s  " + _fmt_metrics(metrics, bench))
    return metrics, log


class BaselineWorktree:
    """
    A throwaway checkout of some git revision, used as arm A when the change is a real code diff.

    A separate worktree rather than stashing: the stash stack is shared across every worktree of
    the repository and other sessions may be using it, so stashing to swap arms risks popping
    someone else's work. This never touches the user's working tree at all.
    """

    def __init__(self, ref):
        self.ref = ref
        self.path = REPO / "build" / f"ab-baseline-{os.getpid()}"

    def __enter__(self):
        print(f"  Creating baseline worktree at {self.ref} ...")
        subprocess.run(
            ["git", "worktree", "add", "--detach", str(self.path), self.ref],
            cwd=str(REPO), check=True, capture_output=True, text=True,
        )
        return self.path

    def __exit__(self, *exc):
        print("  Removing baseline worktree ...")
        result = subprocess.run(
            ["git", "worktree", "remove", "--force", str(self.path)],
            cwd=str(REPO), capture_output=True, text=True,
        )
        if result.returncode != 0:
            # Leave the directory rather than deleting a tree git still tracks; say where it is.
            print(f"  Could not remove worktree automatically: {result.stderr.strip()}")
            print(f"  Remove it with: git worktree remove --force {self.path}")
        return False


def _fmt_metrics(m, bench):
    primary = PRIMARY[bench]
    bits = [f"{primary}={m.get(primary, float('nan')):.3f}"]
    if "score" in m and primary != "score":
        bits.append(f"score={m['score']:.0f}")
    return "  ".join(bits)


# ----------------------------------------------------------------------

PRIMARY = {
    "auto": "score",
    "shotgrid": "hit_rate",
    "drain": "accuracy",
    "step": "rise_time_s",
    "path": "path_err_mean_m",
}

# Metrics where a smaller number is the better outcome.
LOWER_IS_BETTER = {"rise_time_s", "path_err_mean_m", "path_err_max_m", "time_to_empty_s",
                   "mean_current_a", "duration_s"}

# Metrics worth warning about when they move the wrong way, even though they are not what the
# change was trying to improve. Deliberately excludes anything derived from the primary metric:
# flagging hits/precision/hit@3.0m alongside hit_rate would just restate the headline three more
# times and drown the warnings that actually carry new information.
GUARDRAILS = {
    "min_voltage",
    "mean_current_a",
    "duration_s",
    "shots_taken",
    "timed_out",
    "balls_per_sec",
    "path_err_max_m",
    "time_to_empty_s",
}


def paired_stats(a_values, b_values):
    """
    Paired comparison of two equal-length samples.

    Returns mean delta, the paired t statistic, and whether every pair moved the same way. Sign
    agreement matters independently of t: three pairs that all move the same direction is stronger
    evidence than the t value alone suggests at this sample size.
    """
    deltas = [b - a for a, b in zip(a_values, b_values)]
    n = len(deltas)
    mean = statistics.fmean(deltas)
    if n < 2:
        return {"n": n, "mean_delta": mean, "t": 0.0, "unanimous": True, "deltas": deltas}
    sd = statistics.stdev(deltas)
    t = 0.0 if sd == 0 else mean / (sd / (n ** 0.5))
    unanimous = all(d > 0 for d in deltas) or all(d < 0 for d in deltas)
    if sd == 0 and mean != 0:
        # Every pair moved by exactly the same non-zero amount. That is a real, perfectly
        # repeatable effect, not an undefined t.
        t = float("inf")
    return {
        "n": n,
        "mean_delta": mean,
        "sd_delta": sd,
        "t": t,
        "unanimous": unanimous,
        "deltas": deltas,
    }


def verdict(stats, metric, noise_floor):
    """
    Turns the statistics into a call, separating statistical clarity from practical size.

    A change can be perfectly repeatable and still too small to care about. Reporting only
    significance would present such a change as a win, so magnitude is judged against the
    benchmark's measured noise floor and reported alongside.
    """
    mean = stats["mean_delta"]
    t = abs(stats["t"])
    better_when_lower = metric in LOWER_IS_BETTER
    improved = (mean < 0) if better_when_lower else (mean > 0)

    if t >= T_DECISIVE and stats["unanimous"]:
        strength = "DECISIVE"
    elif t >= T_LEAN:
        strength = "LEAN"
    else:
        return "INCONCLUSIVE", "effect is not distinguishable from run-to-run noise"

    direction = "better" if improved else "worse"
    if noise_floor is not None and noise_floor > 0 and abs(mean) < noise_floor:
        return (
            f"{strength} {direction}",
            f"but the effect ({abs(mean):.3f}) is below this benchmark's measured "
            f"noise floor ({noise_floor:.3f}) -- statistically clear, practically negligible",
        )
    return f"{strength} {direction}", ""


# ----------------------------------------------------------------------


def load_noise_floors():
    if NOISE_FLOOR_CACHE.exists():
        return json.loads(NOISE_FLOOR_CACHE.read_text())
    return {}


def save_noise_floor(bench, metric, value):
    NOISE_FLOOR_CACHE.parent.mkdir(parents=True, exist_ok=True)
    floors = load_noise_floors()
    floors.setdefault(bench, {})[metric] = value
    NOISE_FLOOR_CACHE.write_text(json.dumps(floors, indent=2))


# ----------------------------------------------------------------------


def write_report(path, args, bench, pairs, results, call, note, primary, floors_used):
    lines = [
        f"# A/B test — {bench}",
        "",
        f"**Run:** {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}  ",
        f"**Benchmark:** `{bench}`  ",
        f"**Arm A:** `{args.baseline_ref or 'working tree'}`  ",
        f"**Arm B:** `working tree`  ",
        f"**Pairs:** {pairs}  ",
        f"**Primary metric:** `{primary}`",
        "",
        "## Verdict",
        "",
        f"### {call}",
        "",
    ]
    if note:
        lines += [note, ""]

    st = results[primary]
    lines += [
        f"- Mean delta (B − A): **{st['mean_delta']:+.4f}**",
        f"- Paired t: {st['t']:.2f}  ({'unanimous' if st['unanimous'] else 'mixed'} sign across pairs)",
        f"- Per-pair deltas: {', '.join(f'{d:+.3f}' for d in st['deltas'])}",
    ]
    if floors_used.get(primary) is not None:
        lines.append(f"- Measured noise floor for this benchmark: {floors_used[primary]:.4f}")
    lines += ["", "## All metrics", "", "| Metric | A mean | B mean | Delta | t | Unanimous |",
              "| --- | --- | --- | --- | --- | --- |"]

    for name, st in sorted(results.items()):
        marker = " **(primary)**" if name == primary else ""
        guard = ""
        if name in GUARDRAILS and name != primary and abs(st["t"]) >= T_LEAN:
            if (st["mean_delta"] > 0) == (name in LOWER_IS_BETTER):
                guard = " **regressed**"
        lines.append(
            f"| `{name}`{marker}{guard} | {st['a_mean']:.4f} | {st['b_mean']:.4f} | "
            f"{st['mean_delta']:+.4f} | {st['t']:.2f} | {'yes' if st['unanimous'] else 'no'} |"
        )

    lines += [
        "",
        "## How to read this",
        "",
        "Each pair ran arm A and arm B at the **same random seed**, so both saw identical shot",
        "spread and hub dispersal. The delta within a pair is therefore the effect of the change",
        "rather than the effect of the dice.",
        "",
        "Shot spread, hub dispersal and simulated camera noise are all seeded, so an A/A run of",
        "this benchmark reproduces exactly. That is what makes a measured noise floor of zero",
        "meaningful rather than suspicious.",
        "",
        "Pairing weakens as the two arms diverge behaviourally — once arm B fires at a different",
        "time or drives a different line, its noise draws diverge too. Treat a large measured",
        "effect as directionally right but not precisely quantified.",
        "",
        "A seed fixes one field realisation, not all of them. Agreement across several seeds is",
        "what shows a change helps generally rather than getting lucky on one.",
        "",
        "Raw logs for every trial are kept under `build/ai-logs/`.",
        "",
    ]
    path.write_text("\n".join(lines), encoding="utf-8")


# ----------------------------------------------------------------------


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--bench", required=True, choices=sorted(PRIMARY), help="benchmark to run")
    p.add_argument("--aa", action="store_true",
                   help="A/A noise-floor run: both arms identical. Do this before trusting a new benchmark.")
    p.add_argument("--auto", default="2xTBTB Right",
                   help="auto name for the auto/path benchmarks. Names contain spaces; see "
                        "src/main/deploy/pathplanner/autos/. Avoid the *Adaptive* autos -- they "
                        "do not terminate headless.")
    p.add_argument("--count", type=int, default=0, help="fuel to preload for drain")
    p.add_argument("--distances", default="", help="comma-separated shotgrid distances in metres")
    p.add_argument("--duration", default="12", help="teleop seconds for the step benchmark")
    p.add_argument("--buttons", default="1.0:0:6:true,10.0:0:6:false",
                   help="teleop button script for the step benchmark")
    p.add_argument("--min-pairs", type=int, default=MIN_PAIRS)
    p.add_argument("--max-pairs", type=int, default=MAX_PAIRS)
    p.add_argument("--trial-timeout", type=int, default=300, help="seconds before a trial is killed")
    p.add_argument("--clean", action="store_true", help="delete this run's .wpilog files when done")
    p.add_argument("--baseline-ref", default="",
                   help="git revision to compare against (e.g. HEAD). It becomes arm A; arm B is "
                        "your current working tree. The baseline is checked out into a temporary "
                        "worktree, so your working tree is never modified.")
    args = p.parse_args()

    if args.aa:
        # Both arms are the working tree: nothing differs, which is the point.
        args.baseline_ref = ""
    elif not args.baseline_ref:
        p.error("need --baseline-ref <git ref> to compare against, or --aa for a noise-floor run")

    bench = args.bench
    primary = PRIMARY[bench]
    floors = load_noise_floors().get(bench, {})

    print(f"\nA/B test - benchmark '{bench}', primary metric '{primary}'")
    print(f"  Arm A: {args.baseline_ref or 'working tree'}")
    print(f"  Arm B: working tree")
    if args.aa:
        print("  A/A mode: measuring the noise floor, both arms are identical.")
    if not args.aa and floors.get(primary) is None:
        print(f"  WARNING: no noise floor recorded for '{bench}'. Run with --aa first so the")
        print("      report can tell a real effect from this benchmark's own jitter.")
    print()

    with contextlib.ExitStack() as stack:
        a_tree = None
        if args.baseline_ref:
            a_tree = stack.enter_context(BaselineWorktree(args.baseline_ref))
        rc, stats_by_metric, pairs, logs = run_sweep(args, bench, primary, a_tree)
    if rc != 0:
        return rc

    st = stats_by_metric[primary]

    if args.aa:
        # The spread seen when nothing changed IS the noise floor: any future effect smaller
        # than this cannot be distinguished from the simulation talking to itself.
        floor = abs(st["mean_delta"]) + st.get("sd_delta", 0.0)
        save_noise_floor(bench, primary, floor)
        call = "A/A NOISE FLOOR"
        note = (f"Recorded noise floor for `{bench}` / `{primary}`: **{floor:.4f}**. "
                f"Effects smaller than this are not measurable with this benchmark.")
        if abs(st["t"]) >= T_DECISIVE and st["unanimous"]:
            call = "A/A FAILED"
            note = ("**The harness reported a difference between two identical arms.** Either the "
                    "seeding is not taking effect or the benchmark is not repeatable. Do not trust "
                    "any A/B result from this benchmark until this is fixed.")
    else:
        call, note = verdict(st, primary, floors.get(primary))

    ARTIFACT_DIR.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    report = ARTIFACT_DIR / f"ab-{bench}-{stamp}.md"
    write_report(report, args, bench, pairs, stats_by_metric, call, note, primary, floors)

    print("\n" + "=" * 72)
    print(f"  {call}")
    if note:
        print(f"  {note}")
    print("=" * 72)
    print(f"  {primary}: A={st['a_mean']:.4f}  B={st['b_mean']:.4f}  "
          f"delta={st['mean_delta']:+.4f}  t={st['t']:.2f}")
    print(f"  per-pair deltas: {', '.join(f'{d:+.3f}' for d in st['deltas'])}")

    regressions = [
        n for n, s in stats_by_metric.items()
        if n in GUARDRAILS and n != primary and abs(s["t"]) >= T_LEAN
        and ((s["mean_delta"] > 0) == (n in LOWER_IS_BETTER))
    ]
    if regressions:
        print(f"  guardrail regressions: {', '.join(sorted(regressions))}")

    print(f"\n  Report: {report.relative_to(REPO)}")

    if args.clean:
        for log in logs:
            try:
                Path(log).unlink()
            except OSError:
                pass
        print(f"  Deleted {len(logs)} trial logs (--clean).")
    else:
        print(f"  Raw logs: build/ai-logs/ ({len(logs)} files)")

    return 0


def run_sweep(args, bench, primary, a_tree):
    """
    Runs paired trials until the result is decisive or the pair cap is reached.

    Returns (exit_code, stats_by_metric, pairs_run, log_paths).
    """
    a_runs, b_runs, logs = [], [], []
    pairs = 0
    stats_by_metric = {}

    while pairs < args.max_pairs:
        seed = pairs + 1
        print(f"  pair {pairs + 1} (seed {seed}):")
        try:
            a_metrics, a_log = run_trial(bench, seed, args, "A", tree=a_tree)
            b_metrics, b_log = run_trial(bench, seed, args, "B")
        except RuntimeError as e:
            print(f"\n  TRIAL FAILED: {e}", file=sys.stderr)
            print("  Stopping. A failed trial cannot be silently dropped without biasing the result.",
                  file=sys.stderr)
            return 1
        except subprocess.TimeoutExpired:
            print(f"\n  TRIAL TIMED OUT after {args.trial_timeout}s. Stopping.", file=sys.stderr)
            return 1

        a_runs.append(a_metrics)
        b_runs.append(b_metrics)
        logs += [a_log, b_log]
        pairs += 1

        shared = set(a_runs[0]) & set(b_runs[0])
        stats_by_metric = {}
        for name in shared:
            av = [r.get(name, 0.0) for r in a_runs]
            bv = [r.get(name, 0.0) for r in b_runs]
            st = paired_stats(av, bv)
            st["a_mean"] = statistics.fmean(av)
            st["b_mean"] = statistics.fmean(bv)
            stats_by_metric[name] = st

        if pairs >= args.min_pairs:
            st = stats_by_metric[primary]
            if abs(st["t"]) >= T_DECISIVE and st["unanimous"]:
                print(f"\n  Decisive after {pairs} pairs; stopping early.")
                break

    return 0, stats_by_metric, pairs, logs


if __name__ == "__main__":
    sys.exit(main())
