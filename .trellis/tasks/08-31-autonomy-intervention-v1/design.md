# Diagnostic design follow-up

Current approved increment is profiling and ROS/steady transaction diagnostics against actual dirty primary124-test recovery. Preserve safety/route behavior. No cache or optimization is delivered: measured soft query bottleneck does not justify unsafe map reuse or broader snapshot architecture. One dedicated33-field topic records source identity, callback wait, transaction intervals/stages/skips; NaN resets prevent stale-stage data. Grid extraction timing nests within hard/soft query timing. Existing recorder gains raw source,clock,goals,odoms,rosout,paths/corridor/sample/debug/stop. Original2.1second all-topic gap cause remains unknown. No user recording/navigation launched. Detailed schema is in current-frame-pedestrian-corridor.md. Historical recovery requirements below remain preserved functionality, not new work.

# Dynamic A* recovery design

Baseline e2b7945. Existing current-frame polygon/FIRI certificates, A* selected static classification and comfort objective, MPPI costs and accepted-observation transaction remain authoritative.

## Temporary A* query

Extend the existing search with optional query-scoped immutable convex polygon overlay and steady-clock deadline, leaving default/static calls unchanged. Complete convex CCW polygons are validated. Hard edge checks use continuous segment/polygon clipping plus minimum segment/edge distance at the existing FIRI clearance; points/vertices alone are insufficient. Overlay enters hard checks only, not comfort costs. Recovery resets existing A* workspace, rejects partial horizon output and independently certifies the full returned exact-start/exact-goal path before use. No persistent obstacle state or second planner.

A recovery-only cooperative deadline is checked in expansion, static-bin certificate loops and comfort quadrature, plus final route validation. Budget is at most half the remaining ROS observation freshness (capped at 0.2 steady seconds), leaving time for FIRI/MPPI without changing the source timeout. Steady time bounds work even if ROS time pauses; original ROS header/receipt age is rechecked at commit and execution. Individual backend calls are not hard-preemptible. Node caps remain unchanged. Deadline/unsupported/no-path/exact-goal failure must clear candidate output.

## Manager transaction and route commit

Build current geometry normally under dynObsMutex. Only DYNAMIC_REFERENCE_BLOCKED with valid single hull attempts search. Use exact current CoM and end_pt_ (not lookahead mpc_sim_goal_). Capture the same immutable hull and frame; no unlock/generation-starvation gap during recovery. Existing cycle retries a still-blocked reference; valid detours persist after obstacle departure.

Candidate route is local temporary data. Obtain a static crop for candidate reference and rebuild FIRI with the original polygon, not a recomputed SFM hull at a changed crop. On successful exact route, finite whole-edge certificate, fresh frame and feasible rebuilt corridor, atomically update global_waypoints_, index, reanchor subgoal, warm start and /astar/path. Failed candidate/rebuild never authorizes motion or replaces the old route; existing dynamic STOP clears trajectory/heartbeat and next cycle re-evaluates. A* internal candidate cache must not become published authority before commit; planner-3 publication reads accepted global route.

Goal callbacks and accepted path changes must serialize with this transaction; avoid recursive helper locks. Recovery diagnostics are text appended to existing corridor debug (attempt count/status/elapsed), leaving metric indices untouched. Dynamic snapshots remain explicitly uncaptured.

## Validation

Pure overlay regressions use real A* and static map adapter. Manager regressions use actual FIRI/MPPI and LFPC integration, with compile-time-only existing friend seams for expiry injection, no runtime test switches. Preserve static no-overlay and 116 existing cases; extend tests for broad/narrow/blocked-start/clear/retained-detour and no unnecessary search. Negative control restores only disabled recovery in a temporary build. No actual ROS navigation launch; reuse historical headon/overtake/crossing parameters for future user checks.
