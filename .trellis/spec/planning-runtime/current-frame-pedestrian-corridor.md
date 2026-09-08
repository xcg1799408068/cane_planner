# Planner-3 current-frame pedestrian corridor contract

## Scope

Planner 3 with private `use_pedestrians=true`. False preserves the existing static corridor. Planner 4 retains its legacy timed-corridor/obstacle flow. This is one ConvexCorridor/FIRI generator, not a new timed walking corridor implementation. The robust-speed timed spec is not activated by this feature.

## Source validity and transforms

- `/onboard_detector/dynamic_obstacles_info` uses its existing header and full position/velocity/size arrays. First version supports zero or one pedestrian; more than one is explicitly invalid, never silently truncated.
- Enabled missing, mismatched arrays, unsupported/nonfinite values, zero/future/stale stamps, empty frame, or unavailable observation-time TF produces `DYNAMIC_DATA_INVALID`. Invalid input replaces previous validity. Fresh empty is valid; timeout is never synthesized as empty.
- Default maximum source age is 0.5 s, configurable finite `(0,2]`. Check both observation and receipt age before geometry, after geometry, and after MPPI before integration. Future frames fail on receipt and cannot become valid merely by waiting. ROS clock rollback fails closed.
- Geometry captures a per-plan immutable `PedestrianFrameToken` (generation, enabled state, original stamp and receipt) under `dynObsMutex_`. Each accepted enabled frame increments generation, including same-stamp/invalid messages; disabled messages do not change cache/generation. Goal reset and any future mutable enable transition use this mutex and advance generation. Final authorization retains generation/enable and captured-age checks as invariants; expired snapshot gives `DYNAMIC_DATA_INVALID` / `PLANNED_FRAME_EXPIRED`. No retry, empty-frame exception or widened timeout.
- One continuous nonrecursive transaction holds `dynObsMutex_` from observation binding through geometry, MPPI, final authorization, immediate LFPC integration and command/odom output. `updateAndPublishConvexCorridorLocked` requires caller ownership. The lock is released before visualization's ROS-duration sleep. This supersedes the earlier geometry/MPPI lock gap and requirement to reject every concurrent frame, which caused systematic 10 Hz starvation. A callback waiting during the transaction is accepted after commit, including a blocker; only already-accepted observations are checked by that commit. Original snapshot expiry during computation still prevents LFPC advance. This is an explicit accepted-observation ordering, not a guarantee about unprocessed incoming data or future collision safety.
- Callback receipt is captured at callback entry BEFORE mutex wait, not when the cache is finally updated; header stamp is retained. Original receipt/header age validation prevents buffering from renewing old observations. Existing subscriber queue and cooperative FIRI/SFM limits remain; MPPI has finite configured sample/rollout counts, not a hard time bound. Long computation can yield stale STOP and callback delay; no unconditional latency/fairness or hard-real-time guarantee follows.
- Geometry clear/iteration and snapshot reset retain the same mutex. Goal callbacks own the mutex across goal state updates and call `resetCorridorForGoalLocked`; already-held paths call only Locked helpers, avoiding recursive lock deadlock. Regression-only barriers cover actual MPPI and callback entry before lock acquisition. Queued callbacks are joined only after transaction completion; repeated traffic tests must show LFPC progress as well as freshness/invalidity STOP. No runtime test flags.
- Planning frame is `world`, consistent with existing maps/LFPC/markers. Transform centres with full TF and velocity with rotation only, at the original header stamp. Conservatively enclose a rotated complete 3D source box using `abs(R)*full_size`; retain original header and source values separately. Unknown map/world identity is not allowed. The lightweight simulator explicitly emits configured world coordinates in planner-3 launches.
- Source-enable propagates through algorithm, lightweight, hardware and Gazebo entrypoints. Gazebo enable is the OR of lightweight pedestrians, truth bridge and world pedestrians; world-only without observation source therefore fails closed. Truth source takes precedence over lightweight source to avoid two contradictory publishers. Gazebo `ModelStates` is world data; bridge rejects arbitrary frame relabeling and retains callback receipt stamp on timer republishes. A source stall expires rather than becoming perpetually fresh. More than one Gazebo pedestrian intentionally fails the first-version cap.

## Pure polygon module

`PedestrianPolygon` uses observed velocity as desired velocity, hence zero initial drive. It omits target attraction, stochastic terms, other-pedestrian forces and intent inference. Force is an acceleration-like m/s^2 engineering approximation: bounded exponential nearest static-boundary repulsion plus LFPC centre-surrogate repulsion. System repulsion is amplified inside the pedestrian's +/-110-degree FOV. System heading is supplied/validated but the centre surrogate is isotropic; no full human/cane footprint is represented.

Nearest whole blocked-bin face/boundary and finite grid exterior come from the exact selected corridor grid; no raster-density force summation, raw-ESDF alternate classifier or pedestrian rasterization. Only nearest distance inside the finite influence range contributes. A cropped-domain boundary is conservatively treated as blocked, like FIRI, and may produce an artificial boundary force near its edge. Grid scan is bounded and has a cooperative per-observation budget.

Defaults and supported ranges (all finite):

| Parameter `pedestrian/…` | Default | Range / units |
|---|---:|---|
| safety_radius | 0.4 | [0.01,2] m; raised to enclosing half-size norm |
| wall_gain, system_gain | 1 | [0,20] m/s^2 |
| decay_length | 0.5 | [0.01,5] m |
| fov_amplification | 2 | [1,10] dimensionless |
| alpha | 0.3 | [0,5] s^2; force-to-distance, not prediction duration |
| max_force | 4 | (0,20] m/s^2 |
| max_extension | 1.5 | [0,5] m beyond safety radius |
| influence_distance | 3 | [0.01,10] m |
| max_seconds | 0.02 | (0,0.1] cooperative seconds |

Source full dimensions are [0.01,4] m, 3D speed <=10 m/s, absolute source coordinates <=10000 m; transformed planar values retain module bounds. Always include the physical rectangle and a **circumscribed**, not inscribed, octagonal safety disk. For moving observations add paper heading +/-110-degree keypoints and nonzero-force keypoint at `Ds+min(max_extension,alpha*|F|)`. Convex hull includes physical extent even for reverse force. Stationary speed <=1e-6 m/s keeps symmetric safety geometry; force <=1e-9 has no direction. This conservative engineering extension is not the paper's literal three-point triangle. The safety disk is additional pedestrian geometry, not a cane inflation or a future-motion certificate; existing metric FIRI clearance applies once afterward.

## FIRI and failure evidence

`buildStatic(path,grid,dynamic_polygons={})` shares the existing separator template with four-corner static arrays. Complete strictly convex CCW polygons (3–32 vertices, diameter <=20 m) use the same stationary/support-switch/endpoint-active directions, one common separating plane and unmodified metric/MVIE certificates. Empty dynamic input retains static operation order. Precheck all original local reference edges before inflation, then preserve endpoint-protected original edges, connectors/pruning and global budgets. Any reference obstruction with clearance gives `DYNAMIC_REFERENCE_BLOCKED`. Approved recovery reuses existing A* only on this result, with the same validated immutable hull as a temporary hard overlay, exact CoM start/final goal, and a recovery-only cooperative deadline. Static classification/comfort cost and persistent maps remain unchanged. Full exact-goal output is recertified, then same-hull FIRI rebuilt and original frame freshness checked before atomic authoritative route replacement. No partial horizon acceptance, dynamic comfort cost, seed repair or second planner. Failed recovery remains STOP; a valid detour is retained and does not cause every-frame search or automatic return to the old route.

`Result::has_dynamic_input` prevents `CorridorFailureCapture::saveOnce` from saving a static-only file even for an otherwise eligible numeric failure. It returns SKIPPED with `DYNAMIC_INPUT_NOT_CAPTURED`, without consuming the static capture latch. Manager also suppresses such capture and logs the limitation. Existing v1/v2 static snapshots and replay are unchanged; exact dynamic runtime replay is **not implemented**. Dynamic regressions preserve complete polygons as source fixtures instead.

## Obstacle-triggered route recovery

`Astar::searchRecovery(start,goal,polygon,steady_deadline)` reuses the existing search and static certificates. The immutable query polygon is validated as complete strictly convex CCW geometry (3–32 vertices, finite bounded coordinates); continuous segment/polygon clipping rejects closed intersections and segment-edge distances enforce the same FIRI clearance. Empty overlay does not affect the static objective/edge decisions. No overlay persists beyond the call, including timeout/failure. Dynamic geometry is hard-only, never a new comfort penalty. Recovery output must end at the exact actual final goal; partial horizon success is returned as `INCOMPLETE_PATH` with empty output, never committed.

Only actual `DYNAMIC_REFERENCE_BLOCKED` triggers recovery. Manager searches from current exact LFPC CoM to `end_pt_`, under the existing accepted-observation transaction. It allows at most min(0.2 s, half the remaining header/receipt freshness) of cooperative steady-clock search, checking expansion/static-cell/comfort loops and final full-route certificates. Normal search has no new wall deadline. Backend calls and reset/allocation work are finite but not hard-preemptible; timing is not hard real time. ROS-time age is independently checked after search, after FIRI rebuild and before LFPC advance; no 0.5 s timeout increase.

Candidate paths stay temporary until all edges/endpoint checks, same-hull FIRI rebuild and original-frame freshness pass. A new selected-grid crop is allowed, but SFM is not recomputed: the original physical-body-covering hull is reused. Commit updates `global_waypoints_`, index/projection/lookahead subgoal and warm start. Planner-3 `/astar/path` reads this accepted authority rather than uncommitted A* workspace. Goal callbacks, initial planner-3 search and path materialization serialize with recovery; Locked reset helpers never reacquire the mutex. Goal updates encompass final-goal state under that lock.

Failure leaves the old authoritative reference for later safe reevaluation, but motion still requires a valid current corridor and MPPI. No fallback through a blocked old route, no return-to-original-route rule after an obstacle leaves, no search while the current local reference remains valid. Still-blocked/no-path cases may retry on the next existing cycle, bounded as above; no timer/cooldown/voting/retry loop. Existing `max(8 m,2*lookahead)` reference may trigger a far crossing conservatively. Start/goal inside the hull or a fully closed passage legitimately STOP.

Existing debug text appends recovery status (`NOT_NEEDED`, `RECOVERY_UNAVAILABLE`, `ATTEMPTED`, `SUCCESS`, `NO_PATH`, `TIMEOUT`, `EXACT_GOAL_UNAVAILABLE`, `EXPIRED`, `REBUILD_FAILED`), attempt count and elapsed seconds. Existing metric indices and top-level dynamic/static STOP categories are retained. Dynamic replay limitations remain unchanged.

## Output and safety limits

Body/social outlines share `/mpc/convex_corridor` MarkerArray with stable namespaces `mpc_pedestrian_body` and `mpc_pedestrian_social_hull`, id 0, world frame and 0.3 s lifetime. DELETE on invalid/empty/reset and expired FSM data prevents persistent ghosts; corridor publication DELETEALL precedes current geometry. No tracking-ID persistence is assumed. Existing corridor/A*/MPPI visuals remain.

Metric positions 0–22 are unchanged, including legacy spatial/static-mode field 19. Append 23=source enabled, 24=current-frame corridor feasible, 25=FailureReason enum. Legacy MPPI validity/cost fields describe the evaluated rollout; if data expires after MPPI, field 24 is false and STOP reason is authoritative. Stop/debug text distinguishes `DYNAMIC_REFERENCE_BLOCKED`, `DYNAMIC_DATA_INVALID`, static invalidity and no feasible rollout.

No MPPI motion cost, foot/CoM static collision, union check, weighted rerollout/fallback, or LFPC STOP serializer change. Invalid or expired data withholds integration and preserves the lightweight STOP heartbeat. Current-frame corridor exclusion does not predict pedestrian motion, certify full human/cane footprint, or provide physical braking. Local reference retains `max(8 m,2*lookahead)` behavior, so farther reference crossings can conservatively stop the system early. Navigation is user-run, not established by geometry/unit tests.

## Initial planner-3 goal activation transaction

GEN_NEW_TRAJ and REPLAN_TRAJ call one locking activatePlanner3Route helper covering A* search, authoritative route materialization, subgoal/LFPC initialization and MPC_STEP activation. Inner callAstarPlan/generateGlobalWaypoints do not reacquire this nonrecursive mutex. Planner 4 retains its prior branch. Failed planner-3 search retries on the next existing FSM cycle, not a 0.5 s Gazebo sleep while locked.

A queued goal B is not accepted until A activation commits. Once accepted, goal reset invalidates initial_route_active_ and clears corridor/warm start; a stale MPC_STEP dispatch cannot advance LFPC. The real FSM must search/materialize B before reactivation. Both goal callbacks retain their full goal-state mutex ownership. The inactive guard defaults false; tests which directly invoke a step explicitly model an already activated route. No claim is made that received-but-unprocessed B was incorporated in A.

## Transaction diagnostic recording (planner3_transaction_v1)

One `/mpc/transaction_diagnostics` Float64MultiArray is published at each planner-3 step return, including inactive/invalid/STOP exits. Existing `/mpc/debug_metrics` indices are unchanged. Layout label includes `source_frame=<original header frame>` or DISABLED; transform output remains world. All time/duration values are seconds. Steady absolute time is local process/machine monotonic time, not ROS epoch or a cross-machine clock. NaN means skipped/unavailable; per-cycle storage is reset before use. No per-node log spam or eligibility changes.

| Index | Meaning |
|---|---|
| 0 | schema version 1 |
| 1,2 | transaction callback entry ROS epoch / steady seconds |
| 3,4 | prior transaction entry interval ROS / steady; first cycle NaN |
| 5,6 | pedestrian enabled / accepted token generation |
| 7,8 | original source header stamp / callback-entry receipt, before lock wait |
| 9,10 | source stamp / receipt age at transaction entry (negative diagnoses future/queued ordering) |
| 11 | accepted source callback lock wait, steady seconds |
| 12 | planner entry to transaction mutex acquisition, steady seconds |
| 13 | initial FIRI solve only |
| 14 | recovery A* search total (including final edge recertification) |
| 15 | recovery FIRI rebuild only |
| 16 | MPPI call |
| 17 | authorization/check and invalid-output cleanup |
| 18 | entry through critical transaction output, before unlock/visualization sleep |
| 19,20 | total step return elapsed steady / ROS delta, includes visualization sleep on success |
| 21 | no LFPC integration =1, integrated =0; early goal/inactive returns also 1, not a physical STOP guarantee |
| 22,23 | recovery inclusive hard-edge / soft-cost query seconds |
| 24 | recovery grid extraction seconds, NESTED within 22/23; never sum with them |
| 25,26,27,28 | hard calls / soft calls / grid calls / expanded nodes |
| 29,30 | accepted observation count / frame validation state before current computation |
| 31 | current corridor FailureReason enum, NaN if no corridor evaluated |
| 32 | actual LFPC integration/preparation and path collection duration |

Initial FIRI plus recovery rebuild is the FIRI solve total; geometry wrapper also includes map extraction/SFM and is not identical to FIRI. Indices 18/19 are inclusive totals, not sums of disjoint stages; 13–17/32 omit other callback work. Source receipt is never refreshed by waiting. Count/state 29/30 may describe malformed acceptance (state=0); use validity and failure reason, never count alone. Data age/expiry logic remains authoritative and unchanged. The step diagnostic does not cover initial global A* activation outside mpcSimStep; a long initial search can appear as an inter-step interval, not an identified cause.

Existing lightweight recorder includes raw DynamicObstacles, /clock even if absent in wall-time simulation, goals/initialpose, rosout, source/sim/hardware odoms, accepted A* path, corridor, trajectory/sample and debug/STOP topics. Command after reviewed integration: `rosrun plan_manage record_lightweight_mpc_eval.sh recovery_profile`. Recording is user-run; no recording was executed by implementation. A simultaneous all-topic two-second gap remains of unknown cause; ROS vs steady evidence helps investigate, not automatically attribute it to A*.

Profiling-only decision: no map query cache or optimization was added. Current adapter has no map revision/read-lock contract, so search-local caching would not guarantee unchanged live classification. Same returned-grid computation retains all samples, formulas and certificate checks. Search profiling is active only in recovery; normal static search does not start profiling clocks. Timer overhead is nonzero and nested totals must not be treated as zero-cost measurement.
