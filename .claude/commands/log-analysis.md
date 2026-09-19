---
name: log-analysis
description: Use when the user wants to investigate a .wpilog file, diagnose a robot failure from a match log, or answer questions about what the robot did during a run. Trigger on words like "wpilog", "log file", "why did", "what happened", "analyze the log", or any reference to a specific .wpilog path.
---

# Log Analysis — FRC-2026

Investigate a WPILib `.wpilog` file to understand robot behavior or diagnose a failure.

**Codebase:** `C:\Users\bruce\Documents\Coding\FRC-2026`

---

## Setup

Ask the user:

1. **Where is the log file?**
   - Sim logs: `build/ai-logs/*.wpilog` — list with `Get-ChildItem build/ai-logs/ -Filter *.wpilog | Sort-Object LastWriteTime -Descending`
   - Match logs: user provides the path
2. **What question are you trying to answer?** e.g. "why did we miss this shot?", "did the auto pick up all the fuel?", "why is the flywheel not reaching speed?"

---

## Tool Reference

All analysis uses `scripts/wpilog_to_csv.py`. Three tiers of granularity:

### Tier 1 — Summary (always run first)

```bash
python scripts/wpilog_to_csv.py <FILE>.wpilog --summary
```

Outputs: total duration, every key with record count and time range. Use this to understand what subsystems logged data and whether the log is valid.

### Tier 2 — Investigation preset

```bash
python scripts/wpilog_to_csv.py <FILE>.wpilog --investigate auto
python scripts/wpilog_to_csv.py <FILE>.wpilog --investigate shooter
python scripts/wpilog_to_csv.py <FILE>.wpilog --investigate intake
python scripts/wpilog_to_csv.py <FILE>.wpilog --investigate drive
python scripts/wpilog_to_csv.py <FILE>.wpilog --investigate vision

# Narrow to a time window (seconds)
python scripts/wpilog_to_csv.py <FILE>.wpilog --investigate shooter --from 10.0 --to 16.0
```

Outputs a structured report: match state, state machine transitions, continuous channel stats, anomaly flags.

### Tier 3 — Direct key query (for follow-up on specific anomalies)

```bash
python scripts/wpilog_to_csv.py <FILE>.wpilog \
  --keys "KEY1,KEY2,KEY3" \
  --from 11.0 --to 14.0
```

Outputs per key:

- **Numeric:** stats (min/max/mean/std) + sampled values every 0.5s, plateau runs collapsed
- **String/boolean:** all transitions with timestamps

Use this after a preset report flags a suspicious time window — zoom in with `--from`/`--to` and pull exactly the keys you need.

### Comparison Mode — sim vs real

```bash
python scripts/wpilog_to_csv.py --compare SIM.wpilog REAL.wpilog [--top N] [--json fit.json]
```

Aligns both logs on the first `DriverStation/Enabled -> True` and bounds the comparison by the **enabled window** (the last disable), so pre- and post-match idle never dilute the scores. Every numeric key present in both is resampled onto a 20 ms grid and scored.

Per signal:

- `nrmse` — RMSE normalized by the real signal's range. The ranking key.
- `mean_shift`, `p95_shift`, `peak_shift` — sim minus real.
- `corr` — Pearson correlation of shape. High `corr` with a large `mean_shift` means the sim has the right dynamics but the wrong magnitude — usually a constant (mass, MOI, gear ratio) rather than a modelling error.
- Three worst non-overlapping time windows per signal.

**Event-aligned state timing** pairs `*/Target` and `*/Target State` transitions positionally and reports per-event deltas, e.g. `INTAKE  real=9.34s  sim=9.53s  delta=+0.19s`. Pairing stops at the first state mismatch — past that point the runs took different branches and later timings are noise.

**Fit scores by category** (`--json`): `mechanisms`, `currents`, `voltage`, `aggregate`, `other`. Lower is better. Use these to prove a tuning change helped rather than eyeballing traces.

Keys that are metadata (NT client ports, epoch time, match number, CAN error counters, logger timings) are excluded, and keys constant on both sides are skipped — they have no dynamics to compare.

**Not covered by `--compare`:** `Replay/Anchor Error` exists only in the sim log, so read it directly with `--keys`. It is the drivetrain fidelity measure.

### Tier 4 — Raw CSV (last resort, high token cost)

```bash
python scripts/wpilog_to_csv.py <FILE>.wpilog --prefix RealOutputs/Shooter --out /tmp/out.csv
```

Only use when you need every 20ms tick for a narrow prefix. Avoid if possible.

---

## Log Key Reference

All subsystem outputs are under `RealOutputs/` in AdvantageKit logs. DriverStation inputs are at the root.

| What you're looking for      | Exact key                                                    |
| ---------------------------- | ------------------------------------------------------------ |
| Robot enabled / mode         | `DriverStation/Enabled`, `DriverStation/Autonomous`          |
| Robot estimated pose         | `RealOutputs/Robot State/Estimated Pose`                     |
| Swerve position              | `RealOutputs/Swerve/Current Position`                        |
| PathPlanner target pose      | `RealOutputs/Path Planner/Target Pose`                       |
| PathPlanner current pose     | `RealOutputs/Path Planner/Current Pose`                      |
| Active path waypoints        | `RealOutputs/Path Planner/Active Path`                       |
| Distance from path setpoint  | `RealOutputs/Swerve/Distance From Setpoint`                  |
| Shooter state machine        | `RealOutputs/Shooter/Target State`                           |
| Flywheel velocity (actual)   | `RealOutputs/Shooter/Shooter Flywheels/Current Velocity`     |
| Flywheel velocity (setpoint) | `RealOutputs/Shooter/Shooter Flywheels/Target Velocity`      |
| Flywheels up to speed        | `RealOutputs/Shooter/Flywheels Up To Speed`                  |
| Hood target position         | `RealOutputs/Shooter/Shooter Hood/Target Position`           |
| Accelerator target velocity  | `RealOutputs/Shooter/Shooter Accelerator/Target Velocity`    |
| Intake rack target           | `RealOutputs/Intake/Intake Rack/Target`                      |
| Intake rack reached target   | `RealOutputs/Intake/Intake Rack/Reached Target`              |
| Intake rollers target        | `RealOutputs/Intake/Intake Rollers/Target`                   |
| Fuel count (sim)             | `RealOutputs/Field Simulation/Fuel Count`                    |
| Vision accepted poses        | `RealOutputs/Vision/Camera0/Accepted Poses`                  |
| Shooting state predictor     | `RealOutputs/RobotState/Target Shooting State/Shooter Angle` |
| Loop cycle time              | `RealOutputs/LoggedRobot/FullCycleMS`                        |

---

## Investigation Templates

### Shooter not reaching speed

```bash
python scripts/wpilog_to_csv.py <FILE> --investigate shooter
```

Then zoom into the spin-up window:

```bash
python scripts/wpilog_to_csv.py <FILE> \
  --keys "RealOutputs/Shooter/Target State,RealOutputs/Shooter/Shooter Flywheels/Current Velocity,RealOutputs/Shooter/Shooter Flywheels/Target Velocity,RealOutputs/Shooter/Flywheels Up To Speed" \
  --from <T_start> --to <T_end>
```

**What to look for:**

- Velocity plateauing far below setpoint → likely a code bug (NPE or wrong motor output), not tuning
- Velocity oscillating around setpoint → PID kP too high
- Setpoint never logged (only 1 record) → `DEFAULT_SHOOT` target velocity may be null (known NPE)
- `Flywheels Up To Speed` never goes true → shoot command will wait forever

### Auto sequence wrong

```bash
python scripts/wpilog_to_csv.py <FILE> --investigate auto
```

Then for path tracking detail:

```bash
python scripts/wpilog_to_csv.py <FILE> \
  --keys "RealOutputs/Path Planner/Target Pose,RealOutputs/Swerve/Current Position,RealOutputs/Shooter/Target State,RealOutputs/Intake/Intake Rack/Target" \
  --from <T_anomaly - 1> --to <T_anomaly + 2>
```

**What to look for:**

- Path error > 0.25m at start of a segment → robot didn't finish previous segment cleanly
- State machine stuck in `INTAKE` when shooter should be spinning up → event trigger timing
- No `Path Planner/Target Pose` records → PathPlanner auto file not found or wrong name

### Intake not picking up

```bash
python scripts/wpilog_to_csv.py <FILE> --investigate intake
```

```bash
python scripts/wpilog_to_csv.py <FILE> \
  --keys "RealOutputs/Intake/Intake Rack/Target,RealOutputs/Intake/Intake Rack/Reached Target,RealOutputs/Intake/Intake Rollers/Target,RealOutputs/Field Simulation/Fuel Count" \
  --from <T_intake_window>
```

**What to look for:**

- `Intake Rack/Target` shows `INTAKE` but `Reached Target` is false → arm not reaching position (tuning or obstruction)
- `Fuel Count` not decreasing → robot isn't over the fuel, or rollers wrong direction
- `Intake Rollers/Target` shows `IDLE` during intake window → state machine bug

### Drive not following path

```bash
python scripts/wpilog_to_csv.py <FILE> --investigate drive
```

```bash
python scripts/wpilog_to_csv.py <FILE> \
  --keys "RealOutputs/Swerve/Distance From Setpoint,RealOutputs/Swerve/Drive Mode,RealOutputs/Robot State/Estimated Pose" \
  --from <T_segment>
```

**What to look for:**

- Distance From Setpoint stays high (>0.1m) → constraint violation or pose estimation drift
- Drive mode not `PATH_FOLLOWING` during auto → auto command not scheduled
- Pose jumps → vision fusing a bad estimate

---

## Step-by-Step Workflow

1. **Summary** — duration, key presence, record counts
2. **Preset report** — get the full picture for the relevant mode
3. **Identify anomaly window** — specific timestamp where behavior diverges
4. **Key query** — `--keys ... --from T1 --to T2` to zoom in on exactly that window
5. **Diagnose** — state transitions + continuous values together tell the story
6. **Report** — in this order, in the response and on the page alike:
   - **Conclusion** — the answer to the question in plain English, stated before
     any evidence. Say the mechanism, not just the symptom.
   - **Evidence** — the keys, timestamps and values that show it. Charts on the
     page; the same numbers inline in the response.
   - **Why that evidence supports the conclusion** — what specifically in each
     chart or value backs the claim, and what a healthy version would look like.
   - **Root cause** — specific file, line, or state.
   - **Confidence** — "clearly visible in logs" vs "inferred from Y".
7. **Chart it** — write the findings JSON and render the page (below)
8. **Propose the fix, last** — the smallest edit that moves the logged number (below)

**Be verbose in the explanations, terse in the claims.** Every conclusion should
carry enough explanation that a teammate who did not run the investigation can
follow it end to end — but every sentence in it should be one the log supports.
Use bullets whenever the explanation is a list of separate points; use a
paragraph when it is one continuous argument.

**Only report what the logs confirm.** Do not speculate about causes without log evidence.

---

## Charting the Findings

The analysis commands above are text-only and stay that way. Charting is a
separate step: write down what you concluded as a findings JSON, then render it.

```bash
python scripts/log_charts.py findings.json
```

Output is a single self-contained `build/artifacts/<run-id>/index.html` — no
network, no server, opens from a USB stick in the pit. `<run-id>` comes from the
log's match metadata (`2026cada-qm12`) or, for a sim log with none, its filename
and timestamp (`sim-20260918-143022`). Re-running for the same log replaces that
directory. The findings JSON is copied in beside the page.

Write the findings file to `build/` — it is scratch, like the page it produces.

### The shape of a finding — conclusion, evidence, reading

Every finding renders as three labelled parts, always in this order:

| Part           | Field      | What goes in it                                 |
| -------------- | ---------- | ----------------------------------------------- |
| **Conclusion** | `detail`   | What is wrong, stated in full, before any chart |
| **Evidence**   | `charts[]` | The graphs that show it                         |
| **Why**        | `reads_as` | What in _that_ chart backs the conclusion       |

This is not a suggestion — `detail` and `reads_as` are required, and the render
fails naming the finding if either is missing. A chart with no stated reading is
the exact failure this format exists to prevent: a picture dropped next to a
sentence, leaving the reader to guess which line in it was the point.

**Write for someone who was not there.** The reader is a teammate in the pit
between matches, or you in March. They did not run the investigation, they
cannot see your terminal, and they will not open the table view. Assume none of
the context is in their head.

- **Conclusion (`detail`)** — the mechanism, not a restatement of the title.
  What the robot did, what it should have done, and what that means. Name the
  keys, the timestamps and the numbers. Three or four sentences is normal; one
  clause is not.
- **Reading (`reads_as`)** — walk the reader's eye across the chart. Which line
  or lane, at which moment, reaching which value, and what a _healthy_ version
  of that chart would look like instead. "Flywheel is low" is not a reading.
  "The actual line flattens at 46.2 rot/s from 12.1s while the target holds at
  50.0, and never closes the gap before the state leaves SHOOT" is.
- **Number every claim.** A sentence in either field with no value, no timestamp
  and no key name is almost always filler — cut it or give it a number.
- **One reading per chart**, written about that chart. If two charts only mean
  something together, say so inside the second one's reading.

### Bullets

Any prose field takes **a string** (one paragraph) or **an array**: the first
entry is the lead paragraph and every entry after it becomes a bullet.

```json
"detail": [
  "The flywheel never recovered after the first feed, so every shot after 12.1s left at the wrong speed.",
  "Velocity fell to 46.2 rot/s against a 50.0 setpoint at 12.1s and stayed within 0.3 rot/s of that floor for 1.4s.",
  "Flywheels Up To Speed latched true at 11.9s, before the dip, so the shoot command never waited.",
  "The accelerator kept feeding through the whole window — the dip is load the controller did not reject, not a commanded change."
]
```

Reach for the array whenever the explanation has more than one independent
point: separate causes, several moments in time, or a claim plus the
alternatives you ruled out. Keep a single continuous argument as one paragraph —
bullets are for things that are genuinely a list, not a way to chop up a
sentence.

The top-level `verdict` takes the same form, and usually should be an array: a
lead sentence answering the question, then a bullet per supporting finding.

### Schema

```json
{
  "log": "build/ai-logs/akit_26-09-18_23-46-38.wpilog",
  "question": "Why did we miss the shot at 12s?",
  "verdict": [
    "The shot at 12s left at 46 rot/s against a 50 rot/s setpoint, because the up-to-speed latch fired before the flywheel dipped and nothing re-checked it.",
    "Flywheel velocity plateaued 7.6% below setpoint for 1.4s starting at 12.1s.",
    "Flywheels Up To Speed went true at 11.9s and never went false again, so nothing gated the feed.",
    "Hood and accelerator both held their targets through the window, which rules them out."
  ],
  "confidence": "clearly visible in logs",
  "bundles": ["shooter"],
  "stats": [
    {
      "label": "Worst velocity deficit",
      "value": "7.6",
      "unit": "%",
      "severity": "critical"
    }
  ],
  "findings": [
    {
      "title": "Flywheel never recovered after the first feed",
      "severity": "critical",
      "window": [11.0, 14.5],
      "detail": [
        "The flywheel dropped under feed load and the controller never pulled it back, so every shot after 12.1s left slow.",
        "Velocity fell from 50.1 to 46.2 rot/s between 11.9s and 12.1s and held that floor until the state left SHOOT at 13.5s.",
        "The setpoint is flat at 50.0 for the whole window, so this is a tracking failure, not a commanded change."
      ],
      "charts": [
        {
          "form": "timeseries",
          "title": "Flywheel velocity vs setpoint",
          "unit": "rot/s",
          "tolerance_pct": 0.05,
          "include_zero": true,
          "series": [
            {
              "key": "RealOutputs/Shooter/Shooter Flywheels/Current Velocity",
              "label": "Actual"
            },
            {
              "key": "RealOutputs/Shooter/Shooter Flywheels/Target Velocity",
              "label": "Target"
            }
          ],
          "reads_as": [
            "The actual line leaves the 5% band at 12.1s and stays outside it for the rest of the window — a healthy spin-up re-enters the band within about 0.2s.",
            "The target line is flat at 50.0 throughout, so the gap is the controller losing ground rather than the setpoint moving.",
            "The floor at 46.2 rot/s is flat rather than sagging, which is what a saturated output looks like, not a slow recovery."
          ]
        }
      ]
    }
  ]
}
```

**Top level:** `log` (required) · `question` · `verdict` · `confidence` ·
`label` (overrides the run-id) · `bundles` · `stats` · `findings`.
At least one of `findings` or `bundles` must be present.

**`bundles`** renders a baseline set of preset charts, collapsed under
"Baseline". One per investigate preset: `auto`, `shooter`, `intake`, `drive`,
`vision`. Bundle charts tolerate missing keys and carry no `reads_as` — they are
context nobody claimed anything about.

**`findings[]`:** `title` (required) · `detail` (**required**) · `severity` ·
`window` · `charts`. A finding's `window` becomes the default window for its own
charts. `severity` is one of `info`, `good`, `warning`, `serious`, `critical`.

**`charts[]`:** `form` (required) · `series` (required) · `reads_as`
(**required**) · `title` · `unit` · `window` · `include_zero` · `note`.

| `form`       | What it draws                                | Extra fields                              |
| ------------ | -------------------------------------------- | ----------------------------------------- |
| `timeseries` | Numeric signals over time, one shared y-axis | `tolerance_pct` (band around last series) |
| `timeline`   | One state-machine lane per series            | —                                         |
| `path`       | XY field path, equal aspect. 1–2 series      | `error_key` (adds an error chart below)   |

### Rules that keep the charts honest

- **One unit per `timeseries`.** Velocity and current go in two charts, never
  two y-scales on one. The script has no dual-axis mode on purpose.
- **A key that is not in the log is an error**, not an empty chart — it suggests
  the closest key it found. Bundle charts are the exception.
- **Every chart has a table view**, so no value is reachable only by hover.
- Charts plot raw logged keys. If a chart contradicts your prose, the chart is
  right.
- **Never write a reading the chart does not show.** A reading is a claim about
  pixels the reader can check. If the number you want to cite is not visible on
  that chart, either add the chart that shows it or drop the claim.

### Worked flow

```bash
# 1. investigate (text, as above)
python scripts/wpilog_to_csv.py <FILE> --investigate shooter
python scripts/wpilog_to_csv.py <FILE> --keys "..." --from 11 --to 15

# 2. write build/findings.json from what you concluded
#    conclusion per finding, reading per chart, bullets where there is a list

# 3. render
python scripts/log_charts.py build/findings.json
```

---


## Proposing the Fix — always last, always minimal

After the verdict and the charts, close with a concrete code change. This is the
final section of the response, never the first: a fix proposed before the
evidence is a guess with a diff attached.

**Find the real lines before writing anything.** Grep the repo for the logged
key, the state enum, or the constant named in the verdict, and read the
surrounding code. Never propose an edit to a line you have not read.

### Be greedy about not writing code

Walk this ladder top-down and stop at the first rung that fixes what the log
shows. Each rung down is roughly ten times the review cost for the team.

1. **Change a number** — a gain, tolerance, setpoint, timeout, gear ratio, LUT
   entry. Most log-visible failures end here.
2. **Rebind an existing enum** — point a state at a different child target in
   the state machine table.
3. **Flip or add one condition** — a single `&&`, an inverted boolean, a guard.
4. **A few lines inside an existing method.**
5. **A new method, field, or class** — last resort. If you land here, say
   explicitly in one sentence why rungs 1–4 cannot do it.

**Hard rules:**

- Propose the fewest changed lines that the log evidence justifies — and no
  speculative extras.
- If the proposal exceeds ~10 changed lines, state why nothing smaller works
  before showing it.
- **No drive-by work.** No refactors, renames, extra logging, new abstractions,
  reformatting, or fixing unrelated things you noticed while grepping. If you
  spot something else, mention it in one line under "Noticed, not fixing".
- One fix per root cause. Do not bundle.
- If the log narrows the cause to a subsystem but not to a line, say that
  plainly and name the single additional key or run that would pin it — do not
  invent a plausible-looking edit to fill the gap.
- Do not apply the change unless the user asks. If the real fix genuinely needs
  a new subsystem or a redesign, stop and offer `/spec-driven-dev` instead of
  sketching it.

### Format

```markdown
## Proposed fix

**`src/main/java/frc/robot/subsystems/shooter/ShooterConstants.java:47`** — 1 line

Flywheel settles 7.6% low against setpoint (chart 1); kP is too small to hold
load after the feed.

- `public static final double FLYWHEEL_KP = 0.08;`
+ `public static final double FLYWHEEL_KP = 0.14;`

Verify: re-run the sim and confirm `Shooter/Flywheels Up To Speed` stays true
through 11.0–14.5s.

Noticed, not fixing: hood tolerance is also loose, but no shot in this log missed
because of it.
```

Give the before/after lines exactly as they appear in the file, with the
`file:line` anchor, so the user can find them without a search. Always close the
proposal with the one log key or command that would confirm the fix worked.
