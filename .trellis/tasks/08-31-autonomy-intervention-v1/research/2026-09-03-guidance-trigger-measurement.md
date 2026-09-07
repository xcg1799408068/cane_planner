# GUIDANCE trigger measurement — 2026-09-03

Bag: `ours_pilot.bag` (31.4 s, 2 episodes, 296 intervention frames)
Config: `planner:=3 intervention_enable:=true intervention_method:=ours`,
`mpc/goal_closest_approach_enable=false` (legacy baseline), corridor forward
pruning active. Script: `analyze_intervention_bag.py`.

## Result: the GUIDANCE thresholds are not tunable from this signal yet

`GUIDANCE` fired in **0 of 296 frames**. Only `FREE`, `CAUTION`, `STOP_ADVICE`
were observed. This is structural, not a matter of picking better numbers.

### 1. `safe_count == 0` in 176 / 296 frames (59 %)

`GUIDANCE` requires `safe_count > 0`, so it is unreachable in 59 % of frames
before any threshold is consulted.

- 38 of those frames had **no corridor at all** (`source=empty`, `segments=0`)
- the other 138 had a corridor with `feasible=1, rejected_segments=0` and still
  zero safe candidates

With `intervention/corridor_tolerance = 0.02` (2 cm) a candidate is unsafe if
any CoM point leaves the footprint-eroded cell by more than 2 cm. That test
saturates: the autonomy metric spends most of the run pinned at zero.

### 2. Both gates are mis-scaled, over the 120 frames where GUIDANCE is possible

| gate | configured | observed range | frames satisfying |
|---|---|---|---|
| `T_autonomy < T_guide` | 0.25 s | 0.25 – 2.89 s (p10–p90 healthy 1.68 – 2.23) | 1 / 120 |
| `J_safe* > J_guide` | 0.15 | 0.001 – 0.179 (p10–p90 healthy 0.015 – 0.064) | 5 / 120 |

They are ANDed, and never co-occurred.

### 3. `T_autonomy` is mostly a corridor-geometry statistic

`T_autonomy == evaluation_horizon` in **84 / 120** frames with `safe_count > 0`
(70 %): when every candidate survives, the code sets autonomy time to the
horizon, and the horizon is `min(rollout, corridor.tEnd())`.

The corridor extent itself is unstable, so `T_autonomy` inherits that noise:

| corridor source | frames | `t_end` p10 / p50 / p90 |
|---|---|---|
| `previous_mppi` | 187 | 1.26 / 1.96 / 2.59 s |
| `astar_bootstrap` | 71 | 1.43 / 1.43 / 3.50 s |
| `empty` | 38 | 0 / 0 / 0 s |

Segments per frame ranged 0 – 9; the source changed between consecutive frames
28 times. So a threshold on absolute `T_autonomy` seconds partly thresholds
"how long the corridor happened to be this frame".

### 4. No candidate signal separates pre-collapse from healthy

Healthy = on-route (`drift <= 0.25 m`) with `safe_count > 0`;
pre-collapse = the 2 s before the first `safe_count == 0`.

| signal | healthy p10 / p50 / p90 | pre-collapse p10 / p50 / p90 | separable |
|---|---|---|---|
| `T_autonomy` | 1.68 / 1.96 / 2.23 | 0.47 / 1.96 / 2.47 | no |
| `J_safe*` | 0.015 / 0.039 / 0.064 | 0.023 / 0.041 / 0.052 | no |
| drift from `/astar/path` | 0.01 / 0.19 / 0.99 | 0.27 / 0.46 / 0.65 | no |

Drift is contaminated by replans (the route is republished from the robot's
current pose, resetting the metric), so it is not ruled out as a trigger — it is
just not measurable this way.

### 5. Terminal failures, for the record

| episode | duration | max drift | terminal rejection |
|---|---|---|---|
| 1 | 21.4 s | 2.22 m | `corridor_reject` 192 / 200 |
| 2 | 8.2 s | 2.35 m | `static_reject` 200 / 200 |

Both latch `STOP_ADVICE` with `valid_sample_ratio = 0.00`.

## Prerequisites before any threshold tuning

1. **Make "safe" achievable.** Re-derive `intervention/corridor_tolerance`
   (0.02 m) against the actual candidate spread, or the metric stays saturated
   at zero and there is nothing to gate on.
2. **Guarantee a corridor every frame** (13 % missing) and give it a
   deterministic time extent. Treat "corridor unavailable" as an explicit
   state instead of letting it collapse into `safe_count = 0`.
3. **Normalise autonomy time** — publish `T_autonomy / evaluation_horizon`
   so the gate stops proxying corridor length.
4. If `J_safe*` is kept as a gate, `J_guide` belongs near 0.05 – 0.08.

Then re-record and re-run this script before freezing thresholds.


---

# Follow-up, bags 2-5 (2026-09-03 evening)

## The margins were being clamped to zero by the caller

`MpcController::rolloutBatch()` kept a legacy line from when the evidence margin
was a one-sided penetration depth:

```cpp
signed_margin = std::min(signed_margin, -dyn_violation);
```

`-dyn_violation` is 0 whenever no pedestrian is penetrated, so once the margin
could be positive this capped every inside value at 0. Bags 3 and 4 therefore
still reported `margin=-0.000` everywhere. Both corridor helpers already fold
dynamic penetration in, so the clamp was removed (regression test
`MarginsStayPositiveWithNonPenetratedObstacles`). Bag 5 has real values.

## The tolerance question is settled: raising it does NOT help

On the 179 frames (bags 4+5) where the evaluator reports zero safe candidates,
the *best* candidate's 2-D margin is:

| p10 | p25 | p50 | p75 | p90 |
|---|---|---|---|---|
| -0.503 | -0.500 | -0.357 | -0.162 | -0.050 |

The best candidate is typically **0.36 m outside**. Raising
`intervention/corridor_tolerance` from 0.02 m rescues 11 % of those frames at
0.05 m, 23 % at 0.10 m, 31 % at 0.30 m. The tolerance is not the binding
constraint; the first recommendation in this document is withdrawn.

## The violation is longitudinal, not lateral

On those same frames the *lateral* margin is **positive** (p50 = +0.574): the
best candidate sits comfortably inside the corridor width. The 2-D margin's
minimum lands late in the horizon (argmin fraction p50 = 0.61, p90 = 0.94).

`buildTimedCorridorSegment()` fixes `t_start` / `t_end` from the pre-erosion
centreline, and only afterwards does `buildConvexCellForSegment()` call
`erodeHalfPlanesForHumanCaneFootprint()`, which pushes the front face back by the
footprint's forward support (`human_cane_cane_length` 0.65 +
`human_cane_front_radius` 0.12 = 0.77 m). Time coverage therefore extends past
the eroded geometry, and every candidate is judged unsafe for crossing the
corridor's own truncated front face.

That is why 54 % of frames report zero safe candidates while the planner reports
`valid_ratio = 1.00` with zero rejections, and why `eta` is bimodal.

## Trigger signals, for when the base rate is fixed

Frames bucketed by time until the next zero-safe-candidate frame (bag 5, n=87):

| time to collapse | n | lateral margin | 2-D margin | T_auto | J* | d(margin)/dt |
|---|---|---|---|---|---|---|
| 2-4 s | 27 | 0.347 | 0.302 | 1.96 | 0.032 | +0.17 |
| 1-2 s | 30 | 0.611 | 0.594 | 1.96 | 0.046 | +0.08 |
| 0.5-1 s | 15 | 0.731 | 0.380 | 1.68 | 0.064 | +0.05 |
| < 0.5 s | 15 | 0.391 | 0.288 | 1.05 | 0.104 | -0.29 |

- the **margin value** is not monotone in lead time and saturates at ~0.73 m
  (the +-1.0 m FIRI window minus the 0.28 m lateral erosion), so it is a poor gate
- **J\***, **T_auto** and the **margin trend** all move monotonically; the trend
  even changes sign in the last half second

Replaying gates over the 115 safe frames:

| gate | fires | median lead |
|---|---|---|
| current `T<0.25 and J>0.15` | 0 | n/a |
| `J>0.06` | 35 | 0.70 s |
| `J>0.06 and T<2.0` | 24 | 0.60 s |
| `J>0.06 and T<1.5` | 10 | 0.45 s |
| `J>0.08` | 12 | 0.40 s |

`J_guide` near 0.06 with the `T_auto` condition relaxed to ~2.0 (or dropped) is
the shape the data supports, but the distributions overlap heavily (>=2 s p90 =
0.069 vs <1 s p50 = 0.068) and **no frame in any bag is more than 4 s from a
collapse**. Thresholds fitted now would be fitted to a broken base rate.

## Next

Fix the evaluation-horizon / footprint-erosion mismatch, re-record, and only then
fit `J_guide`.
