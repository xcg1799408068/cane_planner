# Static planner-3 FIRI port: existing map-model semantics

User explicitly approved preserving existing selected-backend traversability semantics and replacing only corridor generation. No observed-free volume requirement is imposed. Guarantees are relative to the existing map model.

User authorized the C++ port and static planner-3 replacement after isolated Python prototype review. Historical autonomy/Phase B and prototype-only gates are inactive. Work and validation remain isolated; the main session applies the baseline-relative patch to primary after review. Navigation retest is user-run.

## Requirements

- Approved A* follow-up: prefer global routes with additional clearance using path length plus a bounded soft integral over the same selected blocked-bin layer. Keep the existing continuous hard certificate and narrow-passage traversability. No additional hard radius, MPPI cost change, smoothing, runtime recovery/replan feature or initial map-fluctuation claim. Deliver only an incremental patch against freshly copied primary files.

- One authoritative ConvexCorridor generator: genuine attributed 2D separating-halfspace/MVIE inflation, arbitrary convex polygons, no rectangle width scan or generator switch.
- Preserve original A* edge topology and continuous coverage. Advance point seeds inside the currently covered prefix. Certify actual adjacent overlap area and an inscribed disk after generation; no fixed shared-area seed extension or fallback branches.
- Exclude entire occupied grid cells with metric clearance, not just centers. Outside the finite map domain is blocked; unknown classification follows the selected existing model, including optimistic local ESDF semantics. Missing or invalid backend data fail explicitly.
- Bounded local geometry, monotonic progress, explicit numerical envelope and fail-closed solver/output certificates. Translation-stable area calculations.
- Preserve planner-3 three costs, foot/CoM collision, union hard check, weighted rerollout/fallback and STOP. Do not alter genuine planner-4 dependencies or revive intervention code.
- Translucent filled polygons and thin outlines; keep A* and MPPI visualization, no dense labels by default.
- No global maximum-volume, hard-realtime, human-footprint, LFPC-navigability or dynamic-safety guarantee.
- Use existing Eigen/NLopt, no installation or additional external dependencies. Preserve pinned GCOPTER provenance and MIT attribution.

## Approved lightweight simulation STOP heartbeat follow-up

- On planner-3 invalid-plan STOP, lightweight simulation (`simulation_ && !gazebo_sim_`) must continue publishing fresh `/sim_odom` with the unchanged LFPC pose/yaw and zero linear/angular twist. Publication must not advance/reset gait time, LFPC state or control parameters.
- Reuse the normal serializer; retain moving output, STOP safety/trajectory clearing, other planners, hardware and Gazebo behavior. No timer, map-policy, solver or diagnostic changes.
- Regression must exercise the actual STOP branch and ROS serialization across repeated timestamps, verify frozen state/zero twist and mode guards, and preserve default moving serialization.
- This addresses the confirmed STOP odometry gap and stale simulated-pose override only; initial occupancy fluctuation and live navigation remain separate validation work.

## Acceptance

- [x] Native C++ geometry tests for broad/dense straight paths, 90-degree/diagonal/S corners, obstacle detours, near-wall/narrow, invalid/no-progress/budgets, whole-cell crossing, unknown/map edge, overlap and translation.
- [x] Static backend snapshot tests verify whole-cell mask, static known-policy flags, global ESDF precedence, missing-map failures, and real local SDF fusion/default-filter classification equivalence.
- [x] Existing static MPPI tests preserve collision, weighted rerollout/fallback and STOP.
- [x] Isolated catkin build path_searching/plan_manage succeeds; all 56 registered C++ tests pass.
- [ ] Main-session integration/review of primary incremental patch.
- [ ] User-run navigation retest.

## Approved original-edge segment revision (2026-09-07)

The runtime now walks original A* edges without shortcuts, subdividing at a configurable maximum segment length (default 1 m). Each restrictive solve protects both endpoints with the metric clearance norm included in its constraints; final normalized planes certify both endpoints and whole blocked cells. MVIE-only certified roundoff handling is unchanged.

The old min_progress, overlap_depth and overlap_area configuration is removed. Adjacent regions require a local-frame witness with at least 1e-6 m slack (100 times the 1e-8 m vertex feasibility tolerance), rather than a physical-sized disk or fixed area. A failed direct overlap inserts the same FIRI at the shared endpoint and checks both joins; failure clears all output. Connection regions count toward the global region/time budget. No footprint or LFPC guarantee follows.

Snapshot v2 stores max_segment_length. v1 readers explicitly consume and discard historical geometric-only progress/depth/area fields; the new segment length defaults to 1 m, never reinterpreting area as length. Historical failure diagnostics remain intact.

Final native verification: 13 geometry, 5 snapshot and 6 connector/MVIE-roundoff tests pass. All three exact historical fixtures require and achieve feasibility, each with 81 original edges and 81 regions. Separation initialization uses the closest segment/cell boundary pair when clearance-feasible, not the infeasible centroid guess; solver failures still reject. Diagnostics now include endpoint constraint violations. Dense original paths consume one region per edge at minimum: no coalescing or cap increase was added. A 500-edge synthetic path explicitly fails the default 100-region budget (and passes with a test-only 600 cap). Connector tests exercise point-touch repair with same-FIRI inflation, thin-join rejection, occupied connector and expired deadline. Full catkin integration and navigation retest remain pending.
