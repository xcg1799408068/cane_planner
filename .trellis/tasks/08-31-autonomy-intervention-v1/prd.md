# Profiling and recording follow-up

Current approved increment is profiling and ROS/steady transaction diagnostics against actual dirty primary124-test recovery. Preserve safety/route behavior. No cache or optimization is delivered: measured soft query bottleneck does not justify unsafe map reuse or broader snapshot architecture. One dedicated33-field topic records source identity, callback wait, transaction intervals/stages/skips; NaN resets prevent stale-stage data. Grid extraction timing nests within hard/soft query timing. Existing recorder gains raw source,clock,goals,odoms,rosout,paths/corridor/sample/debug/stop. Original2.1second all-topic gap cause remains unknown. No user recording/navigation launched. Detailed schema is in current-frame-pedestrian-corridor.md. Historical recovery requirements below remain preserved functionality, not new work.

# Obstacle-triggered A* route recovery

Approved scope, baseline `e2b7945` (2026-09-08). Prior single-pedestrian corridor and transaction fixes are integrated. Historical intervention Phase B remains abandoned.

## Requirements

- Retain the authoritative route while its local reference is dynamically feasible; update FIRI only. Trigger recovery only on valid same-hull `DYNAMIC_REFERENCE_BLOCKED`, not stale/invalid data or numerical failures.
- Reuse existing A* with temporary immutable full convex pedestrian geometry. Preserve selected static backend, continuous whole-cell certificates, physical-length/static comfort costs and empty-overlay behavior. No persistent map writes, dynamic comfort costs, second planner or timed prediction.
- Search from exact current LFPC CoM to actual final navigation goal. Certify all candidate/terminal/start/goal edges against complete hull at FIRI metric clearance. A partial horizon route is not success.
- Commit one finite exact-goal route atomically only with fresh original observation and certified rebuilt FIRI using the SAME hull. Replace authoritative path/projection/subgoal/visualization and reset warm start coherently. Never execute merely because A* succeeded. Failure retains old reference for safe subsequent evaluation and STOP heartbeat; never bypass dynamic checks.
- Preserve the continuous accepted-observation transaction, original stamp/receipt checks and disabled behavior. Recovery has a finite cooperative steady-clock deadline within remaining observation age, without increasing the 0.5 s timeout. No claim of hard realtime.
- Retry only when the current reference is blocked on an existing cycle. No A* on valid route or forced return to a shorter original route after the pedestrian leaves.
- Concise explicit recovery diagnostics; preserve metric positions and dynamic snapshot limitations. Planner !=3 unchanged. No scenario coordinate changes, installation, commit/push, main source/devel writes or simulation launch.

## Acceptance

- A* full polygon crossing/containment, tangency/clearance, exact connector, blocked endpoints, impossible passage, deadline and empty-overlay equivalence tests.
- Production manager recoverable broad passage -> exact-goal detour -> FIRI -> actual motion; fully blocked -> bounded STOP; clear -> recovery; valid detour retained without repeat searches; expired candidate cannot commit; existing disabled/goal/transaction/STOP tests maintained.
- Isolated catkin build and all 116 baseline regressions plus new cases, private master, measured search/cycle timings and negative control showing old manager cannot recover.
- Deliver only incremental e2b7945 patch plus baseline/result hashes, changed files and logs under isolated delivery/dynamic-astar-recovery. Independent main-session review precedes integration; user navigation is separate.

## Limits

Single current-frame pedestrian only. No physical braking, intent prediction, full human/cane footprint, optimal global route, future dynamic safety or hard-realtime guarantee. Existing local reference horizon may trigger conservative far-obstacle recovery; start inside hull legitimately STOPs.
