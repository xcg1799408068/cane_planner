# Phase A implementation report

## Scope

Implemented Phase A response semantics and measurement-only evidence. Phase B trigger work was not implemented. No frozen policy or STOP parameter was tuned.

## Path change

```text
Old: MPPI -> policy -> Ours-only PlannerManager response -> overwrite control(2)
New: MPPI + candidate evidence -> one method resolver -> one response adapter
     explicit human api + applied assistance -> final STOP arbiter -> LFPC plant
     copied LFPC + zero assistance -> independent free-rollout evidence only
```

## Assistance definitions

- B0: assistance 0| total api equals explicit human api; no intervention STOP.
- B1: time-zero lateral-margin Schmitt cue (enter 0.10 m, release 0.15 m), shared safe selector and response.
- B2: every usable cycle; candidate and heading both native-best.
- Ours: frozen T/J trigger and guide hold.
- B1/B2/Ours: policy walk-clock voting is the sole intervention STOP hysteresis.

## Metrics

`/mpc/debug_metrics` is unchanged. `mpc/intervention/metrics` indices 0-20 are unchanged: indices 21-39 append free-rollout, selection, control composition, final STOP evidence, and schema version 2.

## Freeze and rollback

The annotated local tag `phase-a-pre-response-2026-09-04` points to commit `891a618`. The verified bundle is `research/phase-a-freeze-2026-09-04/mpc-lfpc-prototype-pre-phase-a.bundle`; checksums, effective launch defaults, and the pre-Phase-A untracked response-module copies are in the same freeze directory.

## Verification

Passed:
- analyzer Python syntax
- active launch XML parsing
- static duplicate check: no PlannerManager inline response methods remain
- B0 policy-history isolation, B2 STOP priority, and immediate heading suppression once policy STOP is requested

Build: `/opt/ros/noetic/env.sh catkin build path_searching plan_manage -DCMAKE_BUILD_TYPE=Release --workspace /home/xcg/ws` passed on 2026-09-04: all 7 dependency/target packages succeeded, no failures. The final incremental build reported `Warnings: None`, `Abandoned: No packages were abandoned`, and `Failed: No packages failed`.

Focused controller test: `test_mpc_no_assistance_rollout` passed all 7 deterministic tests without a ROS master. The controller delegates to a ROS-free helper, so the tests exercise the same implementation with explicit horizon/tolerance inputs. Coverage includes nonzero human heading with zero assistance, source-LFPC immutability, safe right-censoring, exact substep/later-step exit time, shortened and absent coverage, independence from K/snapshot/J_guide, repeat-call/RNG independence, and dynamic-obstacle first-intersection timing. The endpoint fixture exposed and fixed one defect: a sample exactly at the half-open corridor `tEnd()` was incorrectly reported as an exit instead of right-censored coverage.

Regression tests: `/opt/ros/noetic/env.sh catkin run_tests path_searching --workspace /home/xcg/ws` passed. Package results were exactly `Summary: 150 tests, 0 errors, 0 failures, 0 skipped`; workspace `catkin_test_results /home/xcg/ws/build` reported `Summary: 152 tests, 0 errors, 0 failures, 0 skipped`. This includes 7 free-rollout, 5 `InterventionResponse`, 17 evaluator/policy, 18 dynamic-corridor, and 14 timed-corridor/feasibility tests.

## Runtime smoke

Not fabricated. The active simulator requires a user-provided RViz navigation goal, so a noninteractive fixed-seed four-method log could not be produced. After supplying the same initial goal, run planner 3 with intervention enabled, deterministic seed 7, and methods b0/b1/b2/ours while recording the intervention metrics topic and effective parameter dump.
