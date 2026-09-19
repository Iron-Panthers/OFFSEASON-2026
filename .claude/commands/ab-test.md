---
name: ab-test
description: Use when the user wants to know whether a change actually helped — "is this better", "did that help", "compare before and after", "A/B test", "benchmark this change", "which tuning value is better". Runs the same benchmark with and without a change at matched random seeds and reports whether the difference is real or is simulation noise.
---

# A/B Testing — FRC-2026

Answer "did this change actually help?" with a measurement instead of an impression.

Runs a benchmark under two arms at **matched random seeds**, pairs the results, and reports whether
the difference survives the simulation's own noise.

---

## Why pairing, and why you cannot skip it

The simulation draws randomness in three places: shot spread, hub dispersal, and simulated camera
noise. Run the same code twice unseeded and it scores differently. **One run per arm measures the
dice, not the change.**

`-Psim.seed=<n>` pins all three. Run the baseline and the change at the same seed and they see the
same field, so the difference between them is the change. Run several seeds and you learn whether
the change helps generally or just got lucky on one.

Measured effect of seeding, on the `shotgrid` benchmark: noise floor **0.23 → 0.00** hit rate. An
A/A run now reproduces exactly, on both `shotgrid` and `auto`.

---

## Step 1 — Establish the noise floor (required for a new benchmark)

```bash
python scripts/ab_test.py --bench shotgrid --aa
```

`--aa` runs the benchmark against **itself**. Two identical arms must produce no difference.

| Result | Meaning |
| ------ | ------- |
| `A/A NOISE FLOOR` with a small number | Good. That number is the smallest effect this benchmark can resolve; it is cached and used as the practical-significance threshold. |
| `A/A FAILED` | The harness reported a difference between identical arms. Either seeding is not reaching something or the benchmark is not repeatable. **Every A/B result from this benchmark is worthless until fixed.** |

Floors are cached in `build/artifacts/noise-floors.json`. Re-measure after changing a benchmark's
parameters — a floor measured at 4 distances does not apply to a run at 8.

---

## Step 2 — Pick the benchmark

Choose from what changed, not from what is fastest.

| What changed | Benchmark | Why |
| ------------ | --------- | --- |
| Shooter LUT, hood angle, RPM, shot solver | `shotgrid` | Hit rate vs distance is the direct effect. Highest information per second. |
| Flywheel / accelerator / serializer gains | `drain` | Aim and electrical load under sustained rapid fire. Read the caveat below before using it. |
| Any PID or feedforward gain | `step` | ~10 s per trial, nearly noise-free. Screen here before spending time on a game benchmark. |
| Drive gains, slew rates, path constraints | `path` | Tracking error against the PathPlanner setpoint. |
| Auto path or auto sequence | `auto` | Nothing else exercises it. |
| Command or state-machine logic | `auto` | Needs full match context. |

**Screen cheap, confirm expensive.** For a shooter change, run `shotgrid` first; only if it shows an
effect is a full `auto` confirmation worth the wall time.

### What each benchmark does

- **`shotgrid`** — teleports to a series of distances from the hub, aims, fires exactly one ball at
  each. Reports hit rate, per-distance outcome, and `precision` (of the shots actually taken, how
  many scored) separately from `shots_taken` (how many it was willing to take at all).
- **`drain`** — preloads N fuel and holds the shoot button until empty. Primary metric is
  **accuracy under sustained fire**; also reports current draw and minimum battery voltage under
  continuous load.

  > **`drain` does not measure throughput.** `RobotSimState` fires on a fixed 20 shots/sec timer
  > (`setShooterRunning`); the mechanisms gate *whether* it fires, not how fast. So `balls_per_sec`
  > is pinned by that constant and only moves if a change drops a mechanism below the shooting
  > gate. It is kept as a guardrail — a sudden drop means the change broke the gate — but do not
  > read it as a throughput result. Making it real would mean changing the sim's shooting model,
  > which would alter behaviour every other sim consumer relies on.
- **`auto`** — runs a named auto. Reports score, shots fired, accuracy. Default `2xTBTB Right`
  (~22 s, scores ~7).
- **`path`** — runs an auto scored on PathPlanner tracking error (measured pose vs commanded pose)
  rather than on game outcome.

  > **Two traps with auto names.** They contain **spaces** in this repo (`2xTBTB Right`, not
  > `2xTBTBRight`) — quote them. And an unrecognised name does not fail: PathPlanner builds an
  > empty command that finishes instantly, so the run ends after ~1 s having done nothing and both
  > arms score zero. The runner rejects any auto trial shorter than 3 s for exactly this reason.
  >
  > **Avoid the `*Adaptive*` autos.** They do not terminate headless — one ran 485 s before being
  > killed. Use a fixed auto.
- **`step`** — scripted teleop run; mechanism velocity traces analysed for rise time and overshoot.

---

## Step 3 — Define the two arms

Arm A is a **git revision**. Arm B is your **current working tree**.

```bash
python scripts/ab_test.py --bench shotgrid --baseline-ref HEAD
```

Make the change, leave it uncommitted, and compare it against `HEAD`. Any ref works — `main`, a tag,
a SHA.

The baseline is checked out into a **temporary worktree** and built there. Your working tree is
never modified, and nothing is stashed: the stash stack is shared across every worktree of this repo
and other sessions may be using it.

Cost: one build per arm (~40 s), paid once rather than per trial.

---

## Step 4 — Run and read the verdict

```bash
python scripts/ab_test.py --bench shotgrid --baseline-ref HEAD
```

Runs 3 pairs, stops early if decisive, extends to 8 if not.

| Verdict | Meaning |
| ------- | ------- |
| `DECISIVE better/worse` | Every pair agreed in sign and the paired t cleared 3.0. Trust it. |
| `LEAN better/worse` | Directionally suggestive, not established. Re-run with `--max-pairs 8`, or use a different seed range. |
| `INCONCLUSIVE` | The effect is not distinguishable from noise. **This is a real answer**, not a failure — it means the change does not matter at the size you made it. |

### Reading it honestly

- **Statistical clarity and practical size are different questions.** A perfectly repeatable
  `+0.02 hit rate` is reported as clear-but-negligible. Do not ship it as a win.
- **Check the guardrails.** The report flags `min_voltage`, `mean_current_a`, `duration_s`,
  `shots_taken` and friends when they move the wrong way. A change that lifts score while spiking
  current draw is not an improvement.
- **A DECISIVE result covers the seeds you ran.** It says the change helps in those field
  realisations. Widen the seed range before treating it as universal.
- **Do not report a verdict you did not run.** If the sweep failed partway, say so.

---

## Wall-clock cost

Simulation runs at real time (`setUseTiming(false)` is REPLAY-only), so this is not free.

| Bench | Per trial | 3 pairs | 8 pairs |
| ----- | --------- | ------- | ------- |
| `step` | ~10 s | ~1 min | ~3 min |
| `shotgrid` (8 distances) | ~60 s | ~6 min | ~16 min |
| `shotgrid` (4 distances) | ~35 s | ~3.5 min | ~9 min |
| `auto` | ~45 s | ~4.5 min | ~12 min |
| `drain` | ~60 s | ~6 min | ~16 min |

Git mode adds one build per arm (~40 s), paid once, not per trial.

Tell the user the estimate before starting a long sweep.

---

## Output

- **Terminal** — verdict, per-pair deltas, guardrail warnings.
- **`build/artifacts/ab-<bench>-<timestamp>.md`** — full metric table and how-to-read notes.
- **`build/ai-logs/`** — every trial's raw `.wpilog`, kept so a follow-up question does not cost
  another sweep. `--clean` deletes them.

---

## Useful flags

| Flag | Purpose |
| ---- | ------- |
| `--aa` | Noise-floor run, both arms identical |
| `--baseline-ref <ref>` | The revision to compare your working tree against (required) |
| `--distances "2.0,3.0,4.0"` | Shotgrid poses. Fewer = faster but coarser |
| `--count 40` | Fuel to preload for `drain` |
| `--auto 2x4TRight` | Auto name for `auto` / `path` |
| `--min-pairs` / `--max-pairs` | Trial budget (default 3 / 8) |
| `--clean` | Delete this run's logs when done |

---

## Failure modes

| Symptom | Cause |
| ------- | ----- |
| `TRIAL FAILED: ... produced no log` | The sim crashed. The sweep stops — dropping failed trials would bias the result toward whichever arm survives more often. |
| Every pair delta is exactly `0.000` | The change had no effect on this benchmark, or it does not reach the code path this benchmark exercises. Check the change is actually uncommitted and in the working tree. |
| `need --baseline-ref ...` | Every A/B needs a revision to compare against. Use `--aa` if you meant a noise-floor run. |
| `A/A FAILED` | Seeding is not reaching some randomness source. Do not trust any result from this benchmark. |
| Both arms score 0 | The benchmark is mis-parameterised (e.g. all shotgrid distances out of range). Check a single run's log before blaming the change. |

---

## Related

- `/simulation-agent` — run a single sim and analyse it
- `/log-analysis` — investigate one `.wpilog` for a specific failure
- Design spec: `build/superpowers/specs/2026-09-18-ab-testing-skill-design.md`
