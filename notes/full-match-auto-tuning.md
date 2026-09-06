# Full Match Auto — tuning log

Metric: `RealOutputs/Field Simulation/Red Score` (max over run), 30 s headless run
(`-Pauto.name="Full Match Auto"`). Sim is stochastic — treat <10 pt deltas as noise, 3 reps/config.

| Date | Change | Before | After | Kept? |
|------|--------|--------|-------|-------|
| 2026-09-06 | Add `Field Simulation/Red Score` + `Blue Score` logging in `RobotContainer.updateSimulation()` | (unobservable) | 0 | kept — metric now readable |
| 2026-09-06 | **Root-cause fix for the pathfinding blocker:** hold `PathfindingCommand.warmupCommand()` in `Robot.java` and cancel it on enable. It shares the global `Pathfinding` singleton; `getCurrentPath()` clears the new-path flag, so while warmup was still running it swallowed the path `AlignToPoseCommand` requested into its own no-op output consumer. The real command never built a trajectory, so `output.accept` was never called and `Swerve/Magnitude` stayed 0. | 0 | 74 / 47 / 44 (mean 55) | **kept** |
| 2026-09-06 | `FULL_HOPPER_SHOOT_SEC` 2.0 → 4.0. Robot arrived at the hub holding ~59 fuel but 2.0 s only dumped ~36 of them (shoot rate ≈16.7 fuel/s), leaving ~24 unshot every cycle. | 74/47/44 (mean 55) | 68/68/68 (mean 68) | **kept** |
| 2026-09-06 | `PICKUP_BALL_GOAL` 40 → 25. Pickup #1 gathered 59 balls in 4.0 s, but chasing the goal to 40+ dragged pickup #2 out to 10.2 s for 42 balls — the marginal clusters are far away. Shorter legs → more score cycles. | 68 | 128/129/125 (mean 127) | **kept** |
| 2026-09-06 | `PICKUP_BALL_GOAL` 25 → 15 | 127 | 128/128/124 (mean 127) | reverted — indistinguishable from 25, no reason to gather less |
| 2026-09-06 | `PICKUP_PATH_CONSTRAINTS` 3.0/3.0 → 4.5/6.0 m/s² (drivetrain caps at 5/10, so there was headroom) | 127 | 127/126/125 (mean 126) | reverted — no effect; pickup legs are curvature/turn limited, not top-speed limited |
| 2026-09-06 | `SPIN_UP_SEC` 0.6 → 0.0 (shooter already holds TOTAL_SPIN_UP for the ~3.2 s drive, so the post-align wait looked redundant) | 127 | 123/128/126 (mean 126) | reverted — no gain, and 0.6 s is a useful guard when a score leg starts near the hub |

**Measurement change:** from here down, runs use the full 150 s match
(`AI_FULL_MATCH_AUTO_SEC` 30 → 150 in `RobotContainer`), per team request. Scores below are
not comparable to the 30 s numbers above.

## 150 s match results (metric: max `Field Simulation/Red Score`)

| Date | Change | Before | After | Kept? |
|------|--------|--------|-------|-------|
| 2026-09-06 | Baseline at 150 s with `FULL_HOPPER_SHOOT_SEC=4.0`, `PICKUP_BALL_GOAL=25` | — | 369/374 (mean 371) | baseline |
| 2026-09-06 | `PICKUP_BALL_GOAL` 25 → 40, re-tested at 150 s (pickup legs grow 4 s → 16 s as the field depletes, so bigger hauls per trip seemed worth it) | 371 | 324/329 (mean 327) | reverted — clearly worse, 25 holds up at 150 s too |
| 2026-09-06 | `PICKUP_BALL_GOAL` 25 → 15 at 150 s | 369.6 (n=5, σ16) | 351/443/403/442/302 (mean 388, σ57) | reverted — +19 against SE≈27 is not significant, and 15 is far noisier |
| 2026-09-06 | **Pool fix (algorithmic):** drop pooled cells within `SWEPT_RADIUS_M`=0.7 m of the robot in `ObjectDetection.updatePool`. The camera can never clear a cell it just drove over — it falls inside `MIN_RANGE_M` (0.4 m) and behind the lens (`relative.getX() > 0` fails), since camera and intake both face rearward — so stale high ball counts linger for `POOL_MEMORY_SEC` and bias the greedy tour toward empty ground. | 369.6 (n=5, σ16) | 402/457/374/373/222/393/394 (mean 374, σ70) | **reverted** — +4 is pure noise, and it quadruples variance incl. a 222 run: clearing by distance also deletes clusters the robot is still approaching, sometimes emptying the pool so `chooseStops` returns nothing and the pickup leg no-ops |

## Cycle anatomy at 150 s (from `Swerve/Drive Mode` + goal transitions)

9 SCORE legs per match. Score legs are stable at ~7 s; **pickup legs grow 4 s → 16 s** as the
field depletes. Per pickup leg: ~0.7 s pathfind to the observing pose, ~0.9 s precision PID
align there, then the pickup path (4 s early, 15 s late). Fuel carried into the hub falls from
59 early to **9** late, yet the shoot window is a fixed 4.0 s.

## Untested ideas, ranked by measured evidence

1. **Adaptive shoot window.** Late cycles carry ~9 fuel but still burn the full 4.0 s (9 balls
   need ~0.5 s at the measured 16.7 fuel/s). Ending the shoot on a hopper-empty signal would
   return ~3.5 s per late cycle. Needs a real fuel-count sensor on the robot, not `RobotSimState`.
2. **Drop the observing-pose precision align.** `AlignToPoseCommand(..., endOnAccurate=true)`
   demands 4 cm / 2.5° for a pose whose only job is aiming a camera — ~1.6 s/cycle, ~15 s/match,
   roughly one extra scoring cycle. The rear camera already faces the field while scoring, so the
   pool may already have what the planner needs.
3. **Replan the pickup tour mid-leg.** One path is committed for the whole leg (up to 15 s late).
   Chunking into 2–3 stops with a replan between would let it abandon clusters that turned out empty.
4. **Sweep lines instead of centroid-hopping.** The intake has width, so a straight fast pass
   through a dense band collects more per metre than a spline through cluster centroids — and
   straight paths are not curvature-limited, which is why raising `PICKUP_PATH_CONSTRAINTS` did
   nothing (see reverted row above).
5. **`CLUSTER_DISTANCE_BIAS_M` = 4.0 flattens distance discrimination** (a 0.5 m and a 4 m cluster
   cost 4.5 vs 8.0), so the tour crosses the field cheaply. Worth a sweep late-match.
