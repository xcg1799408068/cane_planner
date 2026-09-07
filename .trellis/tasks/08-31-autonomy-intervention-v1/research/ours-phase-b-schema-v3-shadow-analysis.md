# Phase B schema-v3 shadow analysis

Bag: `/home/xcg/ws/ours_phase_b_schema_v3_shadow_seed7.bag`

## Integrity

- 258 intervention frames, all length 73.
- Schema version 3 on every frame.
- `use_dynamic_time_margin_trigger=0` on every frame, so the shadow did not control the plant.
- Effective parameters were constant: `t_sup=0.35`, reaction `0.35`, compliance `0.6`, cap `0.174533`, rate limit `0.1`, heading tolerance `0.02`, buffer `0.35`, max estimator steps `30`, `M_enter=0`, `M_release=0.35`.
- Frozen old trigger replay: 21/258 raw frames; old effective guidance: 31/258 frames.

## Dynamic shadow result

- Valid dynamic evidence: 66/258 frames.
- UNKNOWN: 192/258 frames.
- Target valid and turn recoverable: 93/258 frames; every valid target was estimated recoverable.
- Exact exit evidence: 5 frames.
- Adequately covered right-censored evidence: 61 frames.
- New raw entries: 4 frames.
- Latch events: 2 entries and 2 releases.
- New latch active: 6/258 frames, 2.10 walk-seconds total.
- Old/new raw disagreement: 17 frames.
- `pure_exit_time_insufficient`: 0 frames.

## Difficult sections

First latch event:

- entered at walk time 7.00 s;
- preceded `safe_count=0` by 1.75 walk-s;
- preceded policy STOP by 3.15 walk-s;
- but old-response applied heading had already begun 0.35 walk-s earlier;
- the dynamic latch lasted only one frame, then evidence became UNKNOWN, so with a 0.35 s reaction delay it would not mature into an applied heading command in active mode.

Second latch event:

- entered at walk time 1.75 s in its reset episode;
- remained active through 3.15 s and released at 3.50 s;
- preceded candidate collapse and was long enough to survive the response delay.

Several other collapses were preceded by short right-censored free horizons rather than an observed free exit. Those frames were correctly classified UNKNOWN and did not enter the latch. This prevents false claims from negative censored margins, but means the current trigger can miss difficult sections when timed-DWCG coverage is shorter than the dynamically required response time.

## Normal section behavior

Using adequately-covered right-censored frames with safe candidates and no policy STOP as the normal set:

- 61 normal frames;
- 0 new entry events;
- 2 latched frames were carry-over inside the `M_enter/M_release` hysteresis band, not new false entries;
- dynamic margin median 0.91 s, range approximately 0.00–2.71 s.

The shadow therefore did not generally pre-trigger in normal/wide-corridor evidence.

## Activation decision

**Do not activate the dynamic trigger yet.**

The instrumentation and normal-section behavior are correct, and the shadow demonstrates positive lead before some collapses. However, one of two latch events is shorter than the existing reaction delay and therefore would produce no applied correction, while 192/258 frames are UNKNOWN and several difficult sections have insufficient timed-corridor coverage.

This is not a reason to tune `M_enter`, `M_release`, the old T/J thresholds, guide hold, or STOP windows. The observed limitation is evidence coverage/recoverability, not threshold placement.

The specified escalation condition “`T_exit_free` remains large although the turn is already impossible” was not observed (`pure_exit_time_insufficient=0`). The immediate blocker is instead right-censored coverage shorter than `T_required_dynamic`. The next method step should establish delayed-state recoverability or sufficient future DWCG evidence, rather than activate or retune the current margin trigger.
