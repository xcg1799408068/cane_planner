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
- Geometry clear/iteration and snapshot reset retain the same mutex. Both goal callbacks call locking `resetCorridorForGoal`; already-held paths call only Locked helpers, avoiding recursive lock deadlock. Regression-only barriers cover actual MPPI and callback entry before lock acquisition. Queued callbacks are joined only after transaction completion; repeated traffic tests must show LFPC progress as well as freshness/invalidity STOP. No runtime test flags.
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

`buildStatic(path,grid,dynamic_polygons={})` shares the existing separator template with four-corner static arrays. Complete strictly convex CCW polygons (3–32 vertices, diameter <=20 m) use the same stationary/support-switch/endpoint-active directions, one common separating plane and unmodified metric/MVIE certificates. Empty dynamic input retains static operation order. Precheck all original local reference edges before inflation, then preserve endpoint-protected original edges, connectors/pruning and global budgets. Any reference obstruction with clearance gives `DYNAMIC_REFERENCE_BLOCKED`; no seed repair, dynamic A*, partial unsafe union or automatic replan is introduced. New valid geometry is evaluated on the next cycle.

`Result::has_dynamic_input` prevents `CorridorFailureCapture::saveOnce` from saving a static-only file even for an otherwise eligible numeric failure. It returns SKIPPED with `DYNAMIC_INPUT_NOT_CAPTURED`, without consuming the static capture latch. Manager also suppresses such capture and logs the limitation. Existing v1/v2 static snapshots and replay are unchanged; exact dynamic runtime replay is **not implemented**. Dynamic regressions preserve complete polygons as source fixtures instead.

## Output and safety limits

Body/social outlines share `/mpc/convex_corridor` MarkerArray with stable namespaces `mpc_pedestrian_body` and `mpc_pedestrian_social_hull`, id 0, world frame and 0.3 s lifetime. DELETE on invalid/empty/reset and expired FSM data prevents persistent ghosts; corridor publication DELETEALL precedes current geometry. No tracking-ID persistence is assumed. Existing corridor/A*/MPPI visuals remain.

Metric positions 0–22 are unchanged, including legacy spatial/static-mode field 19. Append 23=source enabled, 24=current-frame corridor feasible, 25=FailureReason enum. Legacy MPPI validity/cost fields describe the evaluated rollout; if data expires after MPPI, field 24 is false and STOP reason is authoritative. Stop/debug text distinguishes `DYNAMIC_REFERENCE_BLOCKED`, `DYNAMIC_DATA_INVALID`, static invalidity and no feasible rollout.

No MPPI motion cost, foot/CoM static collision, union check, weighted rerollout/fallback, or LFPC STOP serializer change. Invalid or expired data withholds integration and preserves the lightweight STOP heartbeat. Current-frame corridor exclusion does not predict pedestrian motion, certify full human/cane footprint, or provide physical braking. Local reference retains `max(8 m,2*lookahead)` behavior, so farther reference crossings can conservatively stop the system early. Navigation is user-run, not established by geometry/unit tests.
