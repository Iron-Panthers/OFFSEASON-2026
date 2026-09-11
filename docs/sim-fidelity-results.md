# Simulation Fidelity vs Real Match Logs

What the simulation now reproduces from real Worlds match logs, what it still does not, and every
change made to get there.

**Method.** A real `.wpilog` is replayed through the simulation: the logged joystick axes, buttons,
POVs and driver-station state are injected frame-locked to the robot loop, the same auto is run, and
the resulting sim log is scored against the real one signal by signal.

```bash
./gradlew simulateJava --no-daemon -Pheadless -Pai.logging \
  "-Preplay.inputs=<real-match>.wpilog"

python scripts/wpilog_to_csv.py --compare <sim>.wpilog <real>.wpilog --json fit.json
```

Score is **nRMSE** — RMSE normalised by the real signal's range. 0.20 means typical error is 20% of
the range that signal covers during the match.

---

## Does it match?

### Fit scores, q54 (the log tuned against)

Baseline is the first replay run, re-scored with the current tooling so the comparison is fair.

| category | baseline | now | change |
| --- | --- | --- | --- |
| aggregate (4 signals) | 0.2926 | **0.1315** | **-55%** |
| currents (48 signals) | 0.4498 | **0.1966** | **-56%** |
| voltage | 0.4285 | **0.1364** | **-68%** |
| mechanisms (43 signals) | 0.3697 | **0.2333** | **-37%** |
| other (48 signals) | 0.2556 | **0.2111** | -17% |

### It generalises — five matches, one tuned against

Only `q54` was tuned against. `q93` and `q14` were validation, `q103` was held out and never looked
at until the end, `q64` is the brownout stress case.

| log | aggregate | currents | mechanisms | other | voltage |
| --- | --- | --- | --- | --- | --- |
| q54 (tuned) | 0.1315 | 0.1966 | 0.2333 | 0.2111 | 0.1364 |
| q93 | 0.1554 | 0.1836 | 0.2298 | 0.2120 | 0.1601 |
| q14 | 0.0872 | 0.1561 | 0.2160 | 0.1810 | 0.1231 |
| **q103 (held out)** | 0.1263 | 0.1751 | 0.2292 | 0.1796 | 0.1535 |
| q64 (brownout stress) | 0.1141 | 0.1853 | 0.2570 | 0.2818 | 0.1510 |

The held-out log is indistinguishable from the tuned one, and `q14` — never tuned against — is the
best-fitting match of the five. These are model fixes, not curve-fitting.

Battery parameters were fitted per match for q54, q93, q14 and q64; q103 ran on defaults, which is
part of why its numbers are a fair test.

#### What the mechanism load model changed

The load model (see *Mechanisms had no load at all*, below) was the single largest improvement made,
and it moved every match in the same direction:

| log | aggregate | currents | voltage | mechanisms |
| --- | --- | --- | --- | --- |
| q54 | 0.2480 → **0.1315** | 0.2410 → **0.1966** | 0.1927 → **0.1364** | 0.2241 → 0.2333 |
| q93 | 0.2366 → **0.1554** | 0.2178 → **0.1836** | 0.1909 → **0.1601** | 0.2238 → 0.2298 |
| q14 | 0.2647 → **0.0872** | 0.2250 → **0.1561** | 0.1851 → **0.1231** | 0.2126 → 0.2160 |
| q103 | 0.2412 → **0.1263** | 0.2368 → **0.1751** | 0.1833 → **0.1535** | 0.2156 → 0.2292 |
| q64 | 0.2695 → **0.1141** | 0.2301 → **0.1853** | 0.1792 → **0.1510** | 0.2228 → 0.2570 |

`mechanisms` got slightly worse on all five, by 0.003 to 0.034. That is a real cost and it is
tracked in *The rack trade-off*, below — it is not noise, because it is consistent in sign across
five independent matches.

### Brownout degrades the flywheel, as it does in reality — at a quarter the magnitude

The strongest available test of whether battery sag actually feeds back into torque, because it
predicts a *behavioural* consequence rather than a voltage number. In q64 the real robot's flywheel
spent markedly less time at speed than in a normal match:

| | q54 (normal) | q64 (brownout) | change |
| --- | --- | --- | --- |
| real flywheel up-to-speed | 153.0 s | 130.5 s | **-14.7%** |
| sim flywheel up-to-speed | 164.8 s | 159.1 s | **-3.5%** |
| real voltage minimum | 7.55 V | 5.95 V | |
| sim voltage minimum | 7.43 V | 7.00 V | |

The direction is right, so the sag-to-torque feedback is genuine rather than cosmetic. The
magnitude is about a quarter of the real effect, because the sim does not sag deep enough -- which
traces back to the mean-current deficit, the same root cause as everything else outstanding.

### The validation logs caught a real overfit

`q93` initially scored 0.3433 on currents against `q54`'s 0.2387 -- 44% worse, driven entirely by
one signal: `Intake Rack/Total Amp Seconds` at nRMSE **6.159**.

An earlier change had given the simulated rack a hard stop at 11.29 rotations, inferred from `q54`
where the real rack never exceeded 11.290 and stalled against whatever blocked it for 86.7% of the
match. That evidence was strong and the change improved `q54`.

`q93` disproved it: the real rack reaches **11.68** there -- past the 11.6 commanded target -- with
0.32 V mean applied and 1.87 A mean draw, no stall at all. `q54`'s ceiling was that match's
obstruction, not the mechanism's limit. Capping the sim left the controller with permanent error
against an unreachable target, drawing 2540 amp-seconds against a real 213.

With the bound raised above all observed travel, `q93` currents fell 0.3433 -> **0.2102**.

**The tuned log's score never revealed this.** It is the reason to hold logs back.

---

## What matches well

Measured on q54 against the committed build.

### Mechanisms had no load at all — the single biggest fix

Every roller and the intake rack drew essentially nothing between transients, because WPILib's
`FlywheelSim` and `ElevatorSim` are frictionless. Measured on q54, mean supply current per motor:

| mechanism | sim (before) | real |
| --- | --- | --- |
| shooter flywheel | 0.30 A | 9.62 A |
| serializer | 0.00 A | 9.47 A |
| intake rollers | 0.08 A | 8.02 A |
| intake rack | 0.18 A | 5.67 A |

Scaled by motor count that is roughly 74 A, which is essentially the whole of the pack-current gap.
The drivetrain was never the problem.

#### Why the previously documented fix could not have worked

The earlier version of this document proposed replacing `createFlywheelSystem` with
`identifyVelocitySystem(kV, kA)` characterised from the logs. **That cannot fix anything.**

`FlywheelSim` does not derive current from physics; it derives it from an inverse of its own plant.
Its constructor recovers the gearing from the plant matrices, `G = -Kv*A/B`, and then reports
`(V - omega*G/Kv)/R`. Any linear plant settles where `A*omega + B*V = 0`, which is exactly where
`omega*G = Kv*V` — so the two terms cancel identically. **Reported current is structurally zero at
every steady state, for every plant it can be given**, `identifyVelocitySystem` included.

#### What actually works — and it needs both halves

1. **Subtract a drag voltage from the plant input**, so the mechanism has to work to hold setpoint
   and coasts down at the real rate. `SimCurrentLimit.dragVolts`.
2. **Compute stator current explicitly**, `(V - backEmf)/R`, evaluated against the **commanded**
   voltage rather than the reduced one. `SimCurrentLimit.statorAmps`.

The difference between those two voltages *is* the drag, and it is what makes the steady-state
current come out at `coefficient * omega` instead of zero. An earlier attempt did only the first
half and moved mean pack current 113.84 → 113.50 A, which is why it was recorded as a failure.

Computing the current directly also removes a WPILib quirk: `getCurrentDrawAmps()` multiplies by
`signum(u)`, which flips the sign of a braking current and books regen as a large positive draw.
That is where the simulated omniwheel's 747 A spin-down came from.

The rack needed a different shape. It is deployed and stationary for two thirds to three quarters
of every match, so a speed-proportional model gives it nothing. It gets a constant directional
load instead — `SimCurrentLimit.applyConstantLoad`.

#### How the coefficients were measured

Not tuned against a fit score. For each mechanism, the median real mean stator current while
spinning, taken across all five matches, then solved for the coefficient that lands the simulation
on it given the transient current the plant already produces on its own:

| mechanism | coefficient (A per motor per mech rad/s) | sim stator | median real stator |
| --- | --- | --- | --- |
| shooter flywheel | 0.0341 | 7.92 A | 8.22 A |
| shooter accelerator | 0.0267 | 21.66 A | 21.64 A |
| intake rollers | 0.1068 | 15.85 A | 16.56 A |
| serializer | 0.2237 | 24.56 A | 25.47 A |
| shooter omniwheel | 0.0552 | 118.51 A | 41.13 A |
| intake rack | 0.61 V constant | — | — |

**These are lumped average match loads, not bearing friction.** Unloaded steady state measures 2-3x
lower on every mechanism; the rest is work done on game pieces, which nothing in the sim models.
That is a fitted parameter and is labelled as one in the source — but it is fitted to a five-log
median of a directly measured physical quantity, not to a fit score.

The omniwheel is the exception and is deliberately left at its unloaded value: its transient
current alone is 110 A mean against a real 41 A, so the solve returns a *negative* coefficient. No
drag value can fix it. See *The omniwheel slews too hard*, below.

#### A logging bug this exposed

The roller IOSims published the whole motor group's current, while `GenericRollersIOTalonFX` on the
real robot logs the leader Talon only. The flywheel signal was therefore 4x the real one. Fixed:
logged signals are now per motor, and the battery and `MotorOutputManager` see the group. This also
corrected the intake rollers and serializer, whose plants model one motor where the real mechanism
has two — the pack had been seeing half their draw.

#### The buck-converter model was worth checking, and holds

`supply = stator * dutyCycle` is assumed throughout. Measured against the real robot, binned by
duty cycle, measured-over-ideal comes out at 0.95-1.08 for every mechanism in every bin. The one
place it is not the limiting assumption is the rack — see below.

### Battery discharge through the match — now modelled

The simulated pack's no-load voltage used to be pinned at exactly 12.80 V in every window of the
match: it never discharged at all.

| window | real q54 peak | sim before | sim now |
| --- | --- | --- | --- |
| first 30 s | 13.34 V | 12.80 V | 12.45 V |
| 30-60 s | 12.74 V | 12.80 V | 12.30 V |
| 60-100 s | 12.25 V | 12.80 V | 12.04 V |
| 100-135 s | 12.72 V | 12.80 V | 11.77 V |
| last 30 s | 11.74 V | 12.80 V | 11.53 V |
| **fall over the match** | **-1.60 V** | **0.00 V** | **-0.92 V** |

On mean voltage the sim now falls 1.61 V against the real 2.02 V -- about 80% of the real
discharge. The residual is mostly the mean-current deficit below, which produces less I*R sag on
top of the droop.

Fitted per match by regressing `V = a + b*t + c*I` over the enabled window:

| log | OCV start | droop V/min | R mOhm |
| --- | --- | --- | --- |
| q54 | 12.451 | 0.439 | 11.96 |
| q93 | 12.074 | 0.345 | 11.59 |
| q14 | 12.162 | 0.652 | 10.51 |
| q64 | 12.270 | 1.019 | 10.90 |

Pass them with `-Preplay.battery=<volts>:<ohms>:<droopVoltsPerMinute>`. Resistance is consistent
across all four matches (10.5-12.0 mOhm), so it is a real constant and is now the default; the old
value was 20 mOhm, which sagged roughly twice as hard as the real pack. Open-circuit voltage and
droop vary with the battery's state of charge and health, so they are per-run. q64 -- the match
where the real robot browned out worst -- has by far the steepest droop, ending 2.8 V below where
it started.

### Brownout behaviour — matches

| | real | sim |
| --- | --- | --- |
| battery minimum | 7.55 V | **7.43 V** |
| samples below the 6.75 V brownout threshold | **0** | **0** |

Baseline sat at a pinned 4.00 V floor with 87 samples below brownout. Before the battery work a
brownout was *structurally impossible* in simulation -- maple-sim's own battery hard-clamps at the
threshold -- and the custom model was being overwritten every loop, so the number was fiction
either way.

### Intake deploy timing — within 16%, and moving the right way

| move | real | sim baseline | before load model | sim now |
| --- | --- | --- | --- | --- |
| STOW -> deployed (1 to 10 rot) | **0.500 s** | 0.219 s | 0.340 s | **0.420 s** |
| travel | 8.2 rot | (wrong units) | 8.6 rot | 8.6 rot |

The rack's constant load slowed deployment toward the real figure without being tuned for it — the
load was measured from holding voltage, and the timing improvement is a consequence. The remaining
19% suggests the simulated rack is still too light; its SIM mass is 0.1 kg, which is not a measured
value.

### Peak power draw — within 8%

| | real | sim baseline | sim now |
| --- | --- | --- | --- |
| peak total current | 318.7 A | 1172.6 A (3.7x) | **293.5 A** (0.92x) |

### Driver input reproduction — exact

Mean absolute axis error **0.0039** over 8254 samples against the logged values.

---

## Drivetrain and vision

Measured with `scripts/drive_vision_fidelity.py`, which was written for this and reports metrics
defined identically on both sides:

```bash
python scripts/drive_vision_fidelity.py <sim>.wpilog <real>.wpilog
```

### Vision was not being simulated in any meaningful sense

Four separate defects, all structural:

| defect | effect |
| --- | --- |
| `RobotContainer` passed **one** camera to `Vision` in SIM, at index 3 of the old five-entry SIM transform array -- a **rear-facing** camera the real robot does not have | the simulated robot localised off one backwards camera |
| a second camera was constructed on its own line and **thrown away** -- never passed to `Vision` | fed the simulated arena, never reached the pose estimator |
| SIM declared five camera transforms in entirely different places from COMP's three | nothing about simulated camera geometry matched the robot |
| `new SimCameraProperties()` is PhotonVision's `PERFECT_90DEG` -- zero calibration error, zero latency | vision was **exact at any range, instantly** |

Cameras 1 and 2 logged no observation for an entire match while the real robot's saw a tag in
98-100% of frames.

SIM now shares COMP's three cameras and transforms, and the camera model is a real one. The noise
is injected **in pixels, on the tag corners**, so accuracy falling off with range is a consequence
of the optics rather than a curve someone tuned -- a tag at 8 m spans a fraction of the pixels it
does at 2 m.

Per-camera target distance, q54:

| camera | sim before | sim now | real |
| --- | --- | --- | --- |
| 0 (in the rollers, pitched down) | 4.66 m | **2.45 m** | 2.46 m |
| 1 (side) | never saw a tag | **4.69 m** | 3.45 m |
| 2 (side) | never saw a tag | **4.84 m** | 4.33 m |

### Vision accuracy now degrades with distance, because it is measured

The honest measure of vision accuracy is what two cameras reporting on the **same loop** disagree
about. They saw the same robot at the same instant, so the pose estimator, its lag and its tuning
are all out of the picture, and what is left is vision error alone:

| target distance | sim at 0.25 px | sim at 0.75 px | real |
| --- | --- | --- | --- |
| 2-3 m | 0.030 m | **0.064 m** | 0.073 m |
| 3-4 m | 0.039 m | **0.048 m** | 0.084 m |
| 5-6 m | — | **0.108 m** | — |

The point is not only the magnitude but the shape: at 0.25 px simulated error was **flat** in
distance, which is not vision. It now grows.

**This was unmeasurable until this session.** The vision subsystem logs its per-camera pose
estimates as `struct:Pose3d[]`, and `scripts/wpilog_to_csv.py` decoded `Pose2d` but not `Pose3d` --
so every measurement of how accurate vision actually is had been silently unavailable on both real
and simulated logs. Adding 20 lines of struct decoding is what made the table above possible.

### Wheel slip: maple-sim only models the skid, not the slip

maple-sim does model skidding -- it caps module force at `mu * normalForce` and lets the wheel spin
past the ground when the controller asks for more. But while a module grips, it sets the wheel
speed to **exactly** the ground velocity projected onto the wheel. Simulated odometry is therefore
perfect except during a skid event, and a real robot's is never perfect.

Measured during **autonomous**, the closest thing to a contact-free sample, across five matches:

| metric | real (auto) | sim before | sim now |
| --- | --- | --- | --- |
| module disagreement, median | 0.028-0.038 | 0.012 | **0.021** |
| wheel distance / true distance | 1.077-1.111 | 1.006 | **1.088** |

Two effects needing two knobs, because the coefficient of friction only produces one of them.
Dropping it from 1.4 to maple-sim's lowest supported 0.65 moved wheel-over-true from 1.006 to 1.037
but left disagreement at 0.012 -- **four wheels skidding together still agree with each other**.
Real wheels disagree because they are not identical.

So the friction coefficient is set from published FRC tread-on-carpet figures (1.05) rather than
fitted, and the rest is a per-module scale on **reported** odometry only, never on the physics --
which is exactly what slip is. The simulated robot really does go where maple-sim says; it just no
longer knows precisely where that is, so the pose estimator has to lean on vision the way the real
one does.

### Validated on all five matches

The drivetrain and vision work was checked against every log, not just the one tuned against:

| log | currents | mechanisms | other | aggregate |
| --- | --- | --- | --- | --- |
| q54 | 0.1966 → **0.1805** | 0.2333 → **0.2146** | 0.2111 → **0.1986** | 0.1315 → 0.1354 |
| q93 | 0.1836 → **0.1740** | 0.2298 → 0.2415 | 0.2120 → **0.1981** | 0.1554 → 0.1587 |
| q14 | 0.1561 → **0.1525** | 0.2160 → **0.2138** | 0.1810 → **0.1734** | 0.0872 → **0.0845** |
| q103 | 0.1751 → **0.1743** | 0.2292 → 0.2315 | 0.1796 → 0.1858 | 0.1263 → **0.1199** |
| q64 | 0.1853 → **0.1702** | 0.2570 → **0.2296** | 0.2818 → **0.2722** | 0.1141 → 0.1153 |

The slip model, measured in autonomous on all five:

| log | sim wheel/true | real | sim disagreement | real |
| --- | --- | --- | --- | --- |
| q93 | 1.052 | 1.077 | 0.018 | 0.037 |
| q14 | 1.053 | 1.111 | 0.017 | 0.035 |
| q103 | 1.055 | 1.096 | 0.017 | 0.038 |
| q64 | 1.062 | 1.086 | 0.018 | 0.037 |

Simulated disagreement came out low by a factor of two in **every** match, which is what makes it a
model error rather than noise. Checked first that it was not an artifact of the two signals the
metric is built from being logged at different rates -- they are not, 37.3/43.5 Hz real against
37.3/39.0 Hz simulated -- and then widened the per-wheel scatter from ±3% to ±5.3%, which puts
simulated disagreement at 0.029 against a real 0.028-0.038.

### Realism and replay agreement pull in opposite directions

Widening that scatter made the **nRMSE fit slightly worse**, by about 0.01 per category on both
logs it was checked against:

| | q54 | q103 |
| --- | --- | --- |
| currents, ±3% → ±5.3% | 0.1805 → 0.1822 | 0.1743 → 0.1847 |
| mechanisms | 0.2146 → 0.2249 | 0.2315 → 0.2364 |
| autonomous disagreement | 0.0210 → **0.0292** | 0.0171 → **0.0298** |

This is not a defect in either number, it is the two goals disagreeing. nRMSE measures agreement
with **one particular run**, and noise is by definition not predictable: giving the simulation
realistic odometry error makes its trajectory diverge from the one real trajectory recorded, even
though its statistics are now right. A simulation with perfect odometry will always replay a
specific log more closely, and will always lie about how much the pose estimator has to work.

**±5.3% is the deliberate choice**, because the purpose here is a simulation that behaves like a
robot rather than one that reproduces a transcript. If replay agreement matters more for some piece
of work, `ODOMETRY_SCALE_SCATTER` is one constant.

### What is still not reproduced, and cannot be from a log

Real **teleop** module disagreement is 0.080-0.119 against the simulation's 0.036. The gap is not
grip: it is other robots. Teleop disagreement is 2.8x the same match's autonomous figure, and
nothing in a replay pushes the simulated robot. Reproducing it needs opponents on the field, not a
lower friction coefficient -- and pushing the coefficient down to chase it would be fitting contact
with a tyre model.

The simulation also has no **occlusion**: its side cameras see tags at 4.7 m mean against a real
3.5 m, and it gets multi-tag frames 66% of the time against a real 28%, because nothing ever blocks
its view. That makes simulated vision better-informed than real vision even with the right noise.

## What does not match

### Mean current draw — closed

| | real | sim (before) | sim (now) |
| --- | --- | --- | --- |
| mean pack current, q54 | 148.7 A | 85.4 A | **160.5 A** |

Across all five matches:

| log | sim mean | real mean | sim peak | real peak | sim V min | real V min |
| --- | --- | --- | --- | --- | --- | --- |
| q54 | 160.5 | 148.7 | 357.1 | 318.7 | 7.3 | 7.6 |
| q93 | 165.0 | 145.0 | 364.9 | 344.1 | 7.1 | 7.7 |
| q14 | 167.4 | 166.9 | 368.1 | 365.4 | 6.9 | 7.0 |
| q103 | 170.1 | 154.4 | 349.9 | 373.2 | 6.9 | 7.4 |
| q64 | 165.9 | 154.7 | 357.4 | 364.9 | 6.2 | 5.9 |

Now overshooting by about 8% rather than undershooting by 46%. The residual overshoot is traced,
not mysterious: the simulated serializer spins for 92% of the match against the real robot's 72%
(see *The serializer runs too much*, below), and it is one of the two heaviest mechanisms.

### The rack trade-off, and why `mechanisms` got slightly worse

The constant load fixed the rack's level and cost some of its timing. Both are real:

| | before | after | real |
| --- | --- | --- | --- |
| mean position error | +0.787 rot | **+0.157 rot** | — |
| peak position error | +1.074 rot | **+0.307 rot** | — |
| mean position | 11.325 rot | **10.623 rot** | 10.474 rot |
| fraction deployed | 0.96 | **0.89** | 0.88 |
| deploy time | 0.340 s | **0.420 s** | 0.500 s |
| mean supply current | 0.18 A | **2.52 A** | 5.67 A |
| position nRMSE | **0.118** | 0.215 | — |
| position correlation | **0.629** | 0.284 | — |

Every physical measure improved; the correlation halved. The cause is `SHOOTING_STOW`, which
targets 3 rotations at 6 rot/s: the simulated rack obeys it and retracts ~2.4 rotations in 0.44 s,
while the real rack, given the same command at the same moment, moved 0.15 rotations in 0.98 s. The
real rack often simply does not retract. That is a real-robot behaviour the simulation has no way
to know about, in the same category as being defended.

This is the whole of the `mechanisms` regression seen on all five matches, and it was kept because
the level, the current and the deploy timing all moved toward reality while one correlation moved
away. It is recorded here rather than buried so the decision can be revisited.

### The rack's load is bimodal, so one constant cannot capture it

`LOAD_VOLTS` is a single number, and the real rack does not behave like one. Applied voltage while
deployed and stationary:

| log | mean | p50 | p75 | p90 | fraction above 0.2 V |
| --- | --- | --- | --- | --- | --- |
| q54 | 0.730 | 0.00 | 2.37 | 2.63 | 0.28 |
| q93 | 0.282 | 0.32 | 0.37 | 0.46 | 0.76 |
| q14 | 0.605 | 0.69 | 0.76 | 0.85 | 0.97 |
| q103 | 0.483 | 0.40 | 0.83 | 1.10 | 0.96 |
| q64 | 0.777 | 0.00 | 2.29 | 2.42 | 0.33 |

In q93, q14 and q103 the rack holds continuously at a low voltage. In q54 and q64 it is idle two
thirds of the time and then pushes hard. Stator current is linear in voltage but supply current
goes as voltage squared, so the constant that reproduces mean *stator* current (the mean, 0.61 V)
is not the constant that reproduces mean *supply* current (the RMS, nearer 1.3 V). The mean was
kept, which is why simulated rack supply current sits at 2.52 A against a real 5.67 A. Fitting the
RMS instead would double its stator current error. Neither is right; the load is not constant.

### The serializer runs too much

Its stator current matches — 24.5 A simulated against 25.1 A real — but its supply current does
not, 17.3 A against 12.3 A, because the simulated serializer spins for 92% of the match where the
real one spins for 72%.

This is control flow, not plant. The targets are identical (`SPIN_UP` 10, `SHOOT` 100, `SLOW` 40
rot/s) and the replayed commands arrive at the same moments, but the real robot cycles
`SPIN_UP` ↔ `SHOOT` 36 times in q54 where the simulation cycles 20 — the real robot's readiness
gating takes longer to satisfy. The plant is right; it is being asked to run more.

This is the largest single contributor to the remaining 8% pack-current overshoot.

### The omniwheel slews too hard

Mean stator current while spinning is 118 A simulated against 41 A real, while its mean speed is
183 rad/s against a real 327. It is saturating its controller and taking too long to reach speed,
so it spends the match at high current and low speed.

Its mean *supply* current is nonetheless close (5.78 A against 5.24 A) because its duty cycle is
low, which is why this was invisible until the stator current became honest. No drag coefficient
can fix it — the solve returns a negative value — so it is left at its measured unloaded value and
recorded here as an open plant or gain problem.

### Regenerative current overshoots

Sim reaches -284 A against a real -102 A. Same root cause: with no load to brake against, the
mechanisms dump more energy back than the real ones do.

### Match-specific behaviour the sim cannot know

In q54 the real intake rack never passed 11.290 rotations and stalled against something for 86.7%
of the match, drawing 930 amp-seconds. In q93 the same mechanism reaches 11.68 freely, drawing 213.
The obstruction was specific to that match -- a game piece, most likely -- and nothing in the
driver inputs tells the simulation about it. The sim now models the unobstructed case, so it
under-draws on q54's rack and matches q93's.

### Post-auto robot pose

Not reproduced, and not expected to be: the real robot was defended and contacted. Cumulative wheel
odometry is excluded from scoring for this reason; instantaneous drive velocity is scored and does
match.

---

## A deliberate compensating approximation — retested, and it survives

`SimCurrentLimit.STATOR_TO_SUPPLY_RATIO` is set to **4.0**, which is *below* the intake rollers'
real measured ratio in every match (5.46, 6.02, 5.68). The physically honest value clears every
mechanism.

The previous version of this document said to revisit it once the plant had a real load. That
condition is now met, so it was retested at 6.5 on three matches:

| log | currents at 4.0 | currents at 6.5 | before the load model existed |
| --- | --- | --- | --- |
| q54 | 0.1966 | **0.1951** | 0.2387 -> 0.2536 |
| q93 | **0.1836** | 0.2027 | 0.2102 -> 0.2319 |
| q14 | **0.1561** | 0.1603 | not tested |

The load model shrank the penalty by roughly an order of magnitude but did not remove it. Two of
three matches are still worse at the honest value, so **4.0 stands**.

The reason has changed, though, and that is worth recording. It used to be that the mechanisms had
no load at all, so they overshot everywhere. Now they have the right steady-state load but still
**slew too hard** — the omniwheel produces 110 A mean stator on its own spin-ups against a real
41 A. The tight ceiling clips that. So the revisit condition is no longer "give the plant a load";
it is "make the mechanisms accelerate correctly", and a per-mechanism ceiling would be better than
any single number regardless.

It is labelled as such in the constant's javadoc and pinned by a test, so it is not "corrected" by
accident.

---

## Measurement caveat, and how it was fixed

Two runs of **identical code** originally produced fit scores differing by 0.016, with 52 of 155
signals moving by >0.01. AdvantageKit stamps records with wall clock, so runs finishing at slightly
different wall times put the same event on different grid points.

That noise floor was the same size as several effects being measured, and two changes were rejected
on differences inside it (a reported-current clamp and the viscous drag). The comparison now rebases
the sim on `Replay/Elapsed Seconds` -- exact match time -- which removes the jitter. Numbers quoted
above are from after that fix.

---

## Every change made

### New files

| file | purpose |
| --- | --- |
| `utility/replay/LogTimeline.java` | zero-order-hold time series with binary-search lookup |
| `utility/replay/MatchInputs.java` | parsed timelines from one match log |
| `utility/replay/MatchLogReader.java` | parses a real `.wpilog` into those timelines |
| `utility/replay/LogInputPlayer.java` | frame-locked injection into `DriverStationSim` |
| `utility/replay/PoseAnchor.java` | teleop pose re-anchoring, gyro kept in step |
| `utility/SimBattery.java` | pack voltage from summed motor current |
| `utility/SimCurrentLimit.java` | closed-form supply-limit voltage ceiling |
| `scripts/log_compare.py` | comparison engine |

### Harness bugs fixed

| what | effect |
| --- | --- |
| Match time never injected | DS reported -1 forever; `ShootCommandFactory` gates on `getTimeUntilOurHubShifts() <= 2`, and `-1 <= 2` is true, so the shooter ran enabled 100% of the sim match against roughly half for real |
| Start pose never seeded for auto replays | q54 began at (13.64, 4.27, 180deg) against the real (12.19, 7.45, 89.8deg) -- 3.5 m and 90 degrees apart before a wheel turned |
| `endCompetition()` called inline | did not stop the loop; must run off-thread |
| `-Preplay.fast` | free-running the loop makes AdvantageKit stamp wall clock, compressing a 165 s match into ~48 s of timestamps. Now warns; unusable for fidelity |

### Simulation model bugs fixed

| file | bug |
| --- | --- |
| `RobotContainer.updateSimulation` | `SimBattery.update()` ran *before* the maple-sim arena tick, which calls `RoboRioSim.setVInVoltage()` 5x per loop -- the battery model was dead code, overwritten every loop |
| `GenericSuperstructureIOSim` | never called `apply(config)` at all, so `SensorToMechanismRatio` was never set: the rack position loop ran in ROTOR units where the real robot runs in MECHANISM units (8/pi = 2.55x). Cause of the intake timing error |
| `IntakeRackIOSim` / `ShooterHoodIOSim` | applied gearing in **opposite directions** -- rack multiplied, hood divided -- so at most one could be right |
| `GenericRollersIOSim` | `stop()` never reached the physics; the accelerator spun at 322 rad/s for 130.5 s of a 165 s match while commanded to stop |
| `GenericRollersIOSim` / `GenericSuperstructureIOSim` | never registered with `MotorOutputManager`, so sim `TotalAmps` counted swerve only |
| 4 roller IOSims | sim PID regulated MECHANISM velocity where Phoenix `VelocityVoltage` regulates ROTOR velocity |
| 6 IOSims | `Math.abs()` on `getCurrentDrawAmps()` booked regenerative braking as consumption; real supply current reaches -69.94 A |
| 4 roller IOSims | clamped applied voltage to a hardcoded +/-12 V, so battery sag could not reduce torque |
| `IntakeRackIOSim`, `SerializerSim` | reported a hardcoded `1.0 A // Not simulated` |
| `SerializerSim` | rotor velocity was rad/s divided by the reduction -- two unit errors on one line -- leaving it running backwards all match; also never set `simState.Orientation` |
| `ShooterOmniwheelIOSim`, `SerializerSim` | never published `positionRads` (1 record of 0.0 for the whole match) |
| 5 roller/serializer IOSims | published the whole motor group's current where `GenericRollersIOTalonFX` logs the leader Talon only -- the flywheel signal was 4x the real one. The rollers and serializer had the reverse problem, with the pack seeing one motor where the mechanism has two |
| `GyroIOSim` | `degreesToRadians()` applied to a value already in rad/s, reporting yaw rate 57.3x too small |
| `DriveConstants` | `mapleSimConfig` never called `withBumperSize`, using maple-sim's 0.76 m default against real 33x37 in bumpers -- which also silently sized the intake, since `RobotSimState` derives it from those dimensions |

### Mechanism load model added

| file | change |
| --- | --- |
| `SimCurrentLimit.dragVolts` | rewritten and put to use. Viscous drag subtracted from the plant input, so a mechanism must work to hold setpoint |
| `SimCurrentLimit.statorAmps` | new. `(V - backEmf)/R` against the COMMANDED voltage, replacing `getCurrentDrawAmps()` — which is structurally zero at every steady state, and sign-flips regen |
| `SimCurrentLimit.applyConstantLoad` | new. A directional load for the rack, which is stationary two thirds of the match and so gets nothing from a speed-proportional model |
| `SimCurrentLimit.applyCoulombFriction` | written, measured, and removed. Symmetric friction let the rack dither about its target, giving a mean supply current of -0.01 A against a real 5.67 A. The real holding voltage never changes sign |
| 4 roller IOSims + `SerializerSim` | drag coefficient measured per mechanism from five matches; current reported per motor rather than per motor group |
| `IntakeRackIOSim` | constant load; explicit stator current, which also removes the 206 A artifact from `ElevatorSim` evaluating current after zeroing velocity at a hard stop |
| all of the above | `SimBattery` and `MotorOutputManager` now see the whole motor group where the logged signal is one motor |

### Constants aligned to the real robot (SIM arms only)

| constant | was | now |
| --- | --- | --- |
| `maxLinearVelocity` | 3.75 | 5 |
| `maxLinearAcceleration` | 6 | 8 |
| track width x length | 22.5 x 22.5 in (square) | 19.75 x 24.25 in |
| bumpers | 34 x 34 in | 33 x 37 in |
| wheel radius | 1.925 in | 1.97 in |
| steer gains | `(0.13, 0.79, 0.387, 2)` | `(0.16, 0.67, 0, 1.5)` |
| drive gains | `(0.25, 2.26, 0, 70)` | `(0.24, 2.4, 0.08, 70)` |
| flywheel reduction / gains | 0.71 / `(3,0,0,0,.1)` | 1.411 / `(0.5,0,0,0.2,0.35)` |
| accelerator reduction / gains | 0.67 / `(1,0,0,0,.1)` | 1.5 / `(.6,0,0,0.2,0.17746)` |
| omniwheel reduction / gains | 1 / `(1,0,0,0,.1)` | 1.25 / `(0.4,0,0,0.2,.137)` |
| roller reduction / gains | 2 / `(1,0,0,0,1)` | 2.4 / `(0.3,0,0,0.1,0.2739)` |
| flywheel current limit | 40 A | 20 A |
| omniwheel current limit | 30 A | 60 A |
| rack travel limits | +/-15 m (no stop) | real hard stop at 11.29 rot |
| flywheel / accelerator plant | 1 motor | 4 / 2 motors |
| serializer reduction / gains | 5 / `(1,0,0,0,1)` | 2.833333 / `(0.5,0,0,0.2,0.344827586)` |

The SIM reductions had been set to the **reciprocal** of the real ones (0.71 vs 1.411, 0.67 vs 1.5)
to compensate for the mechanism-vs-rotor PID bug. The accelerator carried the comment
`// changed in sim (otherwise 1)`. Once the error term was corrected the fudge became unnecessary.

The roller `kV` of 1 meant the feedforward alone demanded 50 V for a 50 rps target, so the simulated
rollers railed at battery voltage permanently.

The serializer was the last SIM reduction still at a made-up value; the other four were corrected
earlier and it was missed. It ran the simulated serializer at 80.2 rad/s against a real 58.7.

### One change affecting the real robot (approved)

`GenericSuperstructureIOTalonFX` registered the leader's supply current with `MotorOutputManager`
**twice** (identical statements, and a loop over a list nothing populates). This inflated the real
robot's logged `TotalAmps` by ~6 A mean and up to ~55 A at peaks -- and that series is the
calibration target. Robot behaviour is unaffected; the change is logging-only.

### Changes tried and reverted

| change | why reverted |
| --- | --- |
| Reported-current clamp | Measured worse at both ratios tried. Note: the deltas were inside the then-unknown noise floor, so this is **unresolved rather than disproven**. Helper retained, documented unused |
| Viscous drag, first attempt | Moved mean current 113.84 -> 113.50 A. **Later revived and it works** -- the failure was doing only half of it. Subtracting voltage from the plant does nothing on its own, because the sim kept reporting its structural zero; the current has to be computed against the commanded voltage as well |
| Symmetric Coulomb friction on the rack | Gave the rack a mean supply current of -0.01 A against a real 5.67 A. A controller dithering about its target has zero mean, and the real holding voltage never changes sign. Replaced with a directional constant load |
| `STATOR_TO_SUPPLY_RATIO` at its honest 6.5 | Retested after the load model landed, on three matches. Currents worse on two of three. See the compensating-approximation section |

---

## Conclusions that rest on a single match, and could still be wrong

Three constants derived from one log survived review, improved the tuning log, and were then
disproved. Two by a validation log, one by a direct physical argument.

| inference | evidence that looked strong | disproved by |
| --- | --- | --- |
| Intake rack has a hard stop at 11.29 rot | never exceeded 11.290; stalled 86.7% of the match; 930 amp-seconds | q93 reaches 11.68 freely with 0.32 V applied |
| Stator ceiling of 4.0x the supply limit is physical | rack 4.83x, omniwheel 2.66x, drive 3.75x | rollers are 5.46-6.02x in every match; it is a compensating error, retested and kept |
| `identifyVelocitySystem(kV, kA)` would fix the current gap | it is the textbook fix for a mis-characterised plant | `FlywheelSim` derives current from an inverse of its own plant, so it reports zero at every steady state regardless |
| The rack holds against symmetric friction | its holding current is large and its position barely moves | the holding voltage never changes sign; friction gave a mean supply current of -0.01 A |

Remaining single-log or single-source inferences, listed so they get checked rather than trusted:

- **The load coefficients are lumped average match loads, not friction.** They reproduce the median
  real stator current across five matches, which is the honest claim. They do not distinguish
  bearing drag from work done on game pieces, and they cannot -- every match runs each mechanism at
  essentially one speed, so Coulomb and viscous terms are indistinguishable from the data. A match
  with an unusual amount of game-piece contact will draw the wrong current.
- **The flywheel's q54 measurement is excluded as an outlier.** It measured 0.051 A per rad/s where
  the other four matches measured 0.0128-0.0161. Excluding it is a judgement call; if q54 was
  normal and the other four were the anomaly, the flywheel coefficient is 3x too low.
- **The rack's load is bimodal in two of five matches** and is modelled as a constant. Its mean
  stator current is right and its mean supply current is 2.5x low as a direct consequence.
- **`withRobotMass(54.4311 kg)`** is unchanged and unverified. 120 lb is light for a Worlds robot;
  the drivetrain analysis explicitly declined to change it without a scale reading, since no log
  evidence distinguishes it. It sets both the traction limit and the dyn4j inertia.
- **The rack's SIM mass of 0.1 kg is not a measured value**, and the rack still deploys 16% faster
  than the real one. That is the obvious next thing to try, and it was not tried here because it
  would have confounded the load-model measurement.
- **Battery parameters are fitted per match** for q54, q93, q14 and q64. q103 runs on defaults,
  which is what makes it a fair held-out test. Nothing validates the fitting procedure itself on a
  match it did not see.
