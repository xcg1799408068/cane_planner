# Dynamic walking corridor execution

Baseline: isolated clean checkout moved from stale `09abde7` to detached `96bc339` before edits. Main worktree listing reports `96bc339`. No primary git redirect, source write or build overwrite is permitted. Earlier static implementation narratives are historical and superseded by the baseline contracts, not pending work.

## Crossing starvation correction (current dirty-primary baseline)

Confirmed continuous 10 Hz observations invalidate every otherwise-actionable cycle in the geometry/MPPI lock gap. The approved correction supersedes the earlier requirement to reject every message arriving during MPPI: serialize accepted observation -> geometry -> MPPI -> fresh-snapshot authorization -> execution under one nonrecursive transaction lock, releasing before visualization sleep. Callbacks preserve their entry receipt time while waiting; queued observations are accepted only after commit and are not claimed checked by that commit. Keep immutable token and original stamp/receipt age checks, invalid/stale STOP, goal-reset locking, and disabled-source semantics. No timeout/cost/FIRI/A*/SFM/scenario tuning or retries.

Baseline is the actual dirty primary copied read-only into `/tmp/cane-crossing-transaction-baseline`, with SHA-256 per file; all eight relevant isolated files matched primary. Deliver only a new incremental bugfix against these bytes, not the historical full 96bc339 patch. Synchronization and queued production interleavings are implemented; no main source/devel writes or simulation launch. Independent review remains pending.

Isolated verification commands:

```sh
catkin build --workspace /tmp/cane-dwcg-a24e plan_manage --no-deps --no-status -j2 -p1
catkin build --workspace /tmp/cane-dwcg-a24e path_searching plan_manage --no-deps --no-status -j2 -p1 --make-args tests
python3 /tmp/cane-dwcg-a24e/run_tests.py
python3 /tmp/cane-dwcg-a24e/negative_old_gap.py
```

Production/test builds pass without compiler warnings; all 116 registered tests pass (102 path-searching + 14 manager; prior 114 with interleaving semantics updated plus two repeated-traffic regressions). Empty and nonblocking moving-pedestrian traffic each uses eight consecutive 0.1 s observation timestamps and callback entry during the actual MPPI transaction, with LFPC progress and STOP=false every cycle. Deterministic entry barriers verify callbacks cannot mutate cache until commit. Test-only clock driving permits unchanged visualization sleep; reported wall times include 20 ms contention barriers and ROS delivery waits, not a live scheduler/real-time benchmark. Empty mean/max 30.19/30.62 ms; moving 30.44/31.30 ms locally. No navigation success or hard real-time/fairness claim.

Accepted blocker -> frozen STOP -> fresh clear -> resumed actual integration passes. Queued blocker is explicitly accepted after the preceding fresh cycle, then blocks the next; original snapshot expiry during computation still freezes LFPC even with a queued clear frame. Queued receipt remains its callback-entry timestamp, not the later unlock time. Invalid, disabled, repeated STOP odometry and goal/reset concurrency tests pass. Original strict replacement tests are replaced by queued semantics, not bypassed via callback joins under a held lock.

Negative control compiled the SAME updated production manager/tests with only the old geometry/MPPI lock gap restored in a temporary source copy. Both repeated-traffic regressions failed with ordinary gtest exit 1 (not timeout/crash): every cycle STOPped and total displacement stayed zero, while fixed-source tests passed. No timeout, sample-count or scenario change was used to make this distinction.

Current logs: `/tmp/cane-dwcg-a24e/build-transaction.log`, `test-build-transaction.log`, `tests-transaction.log`, `test-logs/`; temporary negative-control build/logs under `old-gap-negative/`. TSAN remains unavailable from the recorded missing `libtsan_preinit.o`; no installation or sanitizer success claim. Deliver this fix in isolated `delivery/crossing-transaction/`, keeping historical full deliveries separate.

## Ordered implementation

- [x] Verify isolated baseline and read current task/spec/research contracts.
- [x] Replace obsolete task scope/manifests before source changes.
- [x] Implement pure bounded social-force/body hull module and focused tests.
- [x] Generalize one FIRI separator to convex polygons, protected-reference diagnostics and complete exclusion tests, preserving static numerics.
- [x] Integrate planner-3 source validity/TF/freshness, per-cycle geometry refresh and explicit snapshot non-capture; retain planner 4 and STOP serializer.
- [x] Add transient body/social markers, source-enable launch propagation and deterministic existing-simulator scenarios.
- [x] Register and execute pure/corridor/actual-manager transition regressions, static regressions and bounded timings.
- [x] Build path_searching and plan_manage with catkin_tools in a new isolated workspace extending `/home/xcg/ws/devel` read-only. Run tests under a bounded private ROS master; never launch user simulation.
- [x] Validate Python syntax/launch parameter propagation and inspect incremental diff. Delivery script generates/checks patch against `96bc339`, changed files and exact-command logs outside tracked source.
- [ ] Main-session independent review/integration; user-run navigation remains pending.

## Validation approach

Use a unique temporary catkin workspace with symlinks only to this isolated worktree, explicit `--workspace`, `--extend /home/xcg/ws/devel`, Release and `/usr/bin/python3`. Existing dependencies only; stop and report if unavailable. Build owning packages and registered tests. Private test master must be terminated in a finally/trap guard. Capture command stdout/stderr outside tracked sources. No test count, timing or completion claim before execution.

Rollback is the baseline-relative patch boundary: only this worktree is modified. Do not revert unrelated primary work, commit or push. If implementation cannot be completed, report exact partial status and unresolved verification directly rather than marking acceptance complete.

## P2 disabled-source correction

The callback now checks `pedestrians_enabled_` under the existing mutex before incrementing generation. Disabled messages are ignored without changing cache/generation; enabled invalid/same-stamp messages, actual enable transitions and goal resets retain the P1 invalidation mechanism. No final-authorization change or expanded runtime scope. Added one actual manager test injecting a blocker from a second thread after real MPPI while disabled, requiring a valid static plan, LFPC displacement/state advance, empty pedestrian cache and STOP=false. The exact runtime/test/contract delta against the reviewed P1 version is delivered as `p2-disabled-source.diff`; prior files are preserved under `/tmp/cane-dwcg-a24e/p2-before`.

Verification: isolated production manager build and path_searching/manager test build passed; all 114 tests passed (113 previous + 1, manager suite now 12). Latest logs are `/tmp/cane-dwcg-a24e/build-p2.log`, `test-build-p2.log`, `tests-p2.log` and `test-logs/`, copied to the same delivery directory. No compiler warnings reported in these builds. The same private-master runner terminates its master after tests. Patch/manifest refreshed against 96bc339; no main source/build writes or commit.

## P1 review corrections verified

Only the two reviewed concurrency defects were changed. Each production manager cycle now captures immutable generation/enable/stamp/receipt with corridor input; final authorization checks this token and holds `dynObsMutex_` through immediate LFPC integration and command/odom publication. Same-stamp/new/invalid messages and goal resets advance generation. Both goal callbacks use locking `resetCorridorForGoal`; already-locked geometry/clear/publication helpers do not recursively acquire the mutex. Source enable remains initialization-only in production; the friend test seam models future synchronized enable transitions with generation invalidation. No A*/MPPI algorithm, hull, launch or map changes in this correction.

Five deterministic manager regressions were added through compile-time test-only barriers (absent from kin_replan_node): same-stamp blocker replacement after an actual feasible MPPI evaluation, expired A masked by fresh empty B, unchanged-but-expired A, invalid/disable-reenable invalidation, and both actual goal callbacks racing real geometry generation/publication. The reset test uses a barrier to prove it waits on the generation lock, then repeats concurrent build/reset. Prior reviewed sources remain under `/tmp/cane-dwcg-a24e/p1-before`; `p1-corrections.diff` in delivery provides the exact four-source-file delta from that version.

Final verification: all 113 C++ tests pass (108 previous + 5 new; manager suite 11). Commands are the same isolated catkin builds/runner below; latest logs are `build-p1.log`, `test-build-p1-final.log`, `tests-p1-final.log`, and `test-logs/`. No new compiler warnings in final test build. TSAN was attempted via `g++ -std=c++14 -fsanitize=thread -g -pthread /tmp/cane-dwcg-a24e/tsan_probe.cpp -o /tmp/cane-dwcg-a24e/tsan_probe`; linking fails because `libtsan_preinit.o` is absent. No installation or TSAN success claim. `tsan-probe.log` records this limitation. Delivery patch/manifest/logs are refreshed at the existing path and rechecked against exact 96bc339; independent re-review remains pending.

## Executed validation (2026-09-08)

Workspace `/tmp/cane-dwcg-a24e`, isolated plan_env/path_searching/plan_manage source symlinks, existing main devel extended read-only. Commands:

```sh
catkin config --workspace /tmp/cane-dwcg-a24e --init --extend /home/xcg/ws/devel --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3
catkin build --workspace /tmp/cane-dwcg-a24e path_searching plan_manage --no-status -j2 -p1
catkin build --workspace /tmp/cane-dwcg-a24e path_searching plan_manage --no-deps --no-status -j2 -p1
catkin build --workspace /tmp/cane-dwcg-a24e path_searching plan_manage --no-deps --no-status -j2 -p1 --make-args tests
python3 /tmp/cane-dwcg-a24e/run_tests.py
/usr/bin/python3 /tmp/cane-dwcg-a24e/check_interfaces.py
```

All 108 registered C++ tests pass: A* 13, static geometry 13, snapshots 6, angular/pruning/MVIE 15, static MPPI/backend 11, legacy timed/dynamic/kinematic/smoother 34, new polygon/dynamic corridor 10, actual manager/STOP 6. The private master at 11519 is started/stopped by the test runner; no simulation launch. Manager tests distinguish fresh empty/missing/stale/future/malformed/untransformable frames, position/vector/full-box rotation, frozen STOP and actual blocked-to-clear integration. The successful branch's existing prepareNextStep zeroes instantaneous velocity; recovery is asserted by a valid plan plus changed gait state and displacement, not a fabricated nonzero serializer velocity.

Final owning-package build/test build had no compiler warnings. Private-master manager tests emit intermittent existing XmlRpc accept EAGAIN messages but complete successfully. Initial missing gtest main and recovery-fixture initialization/assertion errors were corrected; logs retain final passing evidence. Runtime human navigation is untested.

Ten-repeat small 100x100 empty-grid/five-waypoint timing: static corridor mean 0.241 ms, one pedestrian corridor mean 0.484 ms (local measurement, excludes SFM and ROS, no hard-real-time claim). Legacy/static regression tests pass with unchanged MPPI/A* sources. Source-enabled dynamic snapshots are explicitly suppressed by both manager and capture API; exact dynamic runtime replay remains unsupported.

Python AST, all launch XML, lightweight static/enabled parameter trees, Gazebo enabled/truth and localization-wrapper propagation pass. Existing FAST-LIO source path and generated onboard_detector messages were supplied read-only for resolution/import, not installed or rebuilt. Actual simulator class steps deterministically for approach/crossing/stationary scenarios. Hardware launch was not fully resolved because sensor includes were not exercised; its direct enable forwarding and shared consumer parameter were inspected. No RViz/GUI/navigation smoke claimed.

Final logs: `/tmp/cane-dwcg-a24e/build-delivery.log`, `test-build-delivery.log`, `tests-delivery.log`, `interfaces-delivery.log`, and `test-logs/`. Durable copies and reproduction scripts accompany the incremental patch under isolated `delivery/dynamic-walking-corridor/`; they are excluded from the patch. Main-session independent review and user navigation remain pending.
