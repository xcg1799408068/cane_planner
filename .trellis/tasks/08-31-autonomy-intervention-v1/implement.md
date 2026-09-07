# Static FIRI port execution

Current clearance-preference follow-up is isolated in `/tmp/astar-clearance-ab9b`, with a freshly copied primary baseline and SHA-256 manifest. Reproduce with `python3 build.py`, `python3 build_manager.py`, `python3 build_routes.py`, `python3 run.py`, and `python3 check_launch.py` from that directory. Native -O2/-Wall compilation covers all current path-searching sources, collision adapter, manager node and actual STOP test; unchanged dependencies are linked read-only. All 94 C++ tests pass (13 A*, 13 geometry, 6 snapshot, 15 connector/pruning/angular/MVIE, 11 static MPC/backend, 2 actual STOP, 34 legacy). Reviewer corrections preflight unsupported preference/native-grid query budgets, retain a certified off-lattice goal at horizon termination, and restore omitted legacy sigma 0.5; the horizon test also fails with only that correction removed. No efficiency/cache changes were added. Logs and exact commands are in that directory. Private master port 11507 is stopped after tests. Full simulation/Gazebo parameter trees resolve; hardware sensor dependencies are absent, so only its planner include was resolved after removing sensor includes from temporary input. No full catkin rebuild or navigation claim.

Fixed-endpoint partial snapshot runs compare current-primary defaults against the new defaults, not full routes: SiatJ7 length 6.832889 -> 7.008625 m, whole-edge minimum clearance 0.036604 -> 0.294251 m; M6wfx3 8.015527 -> 8.304626 m and 0.018622 -> 0.454631 m. FIRI accepts all four (regions 4->4 and 4->3). Search increases from ~0.1-0.2 ms to ~48/75 ms locally; FIRI ~13-19 ms. Comfort quadrature is approximate, hard certificates unchanged. No map fluctuation or runtime recovery fix. Main-session patch review/integration and user navigation remain pending.

- [x] Verify worktree root/HEAD and primary hashes; preserve original copied baseline.
- [x] Implement attributed NLopt/Eigen C++ 2D FIRI with whole-cell separation and MVIE, point seeds, certificates, numerical envelope and bounds.
- [x] Replace rectangle generator and launch width parameters; preserve MPPI and planner 4.
- [x] Add aligned map-cell snapshot with mandatory known-space or explicit unsupported-backend failure.
- [x] Render arbitrary polygon fills and thin outlines; remove replaced rectangle helper and dense labels.
- [x] Build plan_env/path_searching/plan_manage in isolated catkin workspace.
- [x] Build all registered C++ test executables and run all 56 tests with bounded private roscore (port 11479), shut down on exit.
- [x] Preserve 17 passing Python reference regressions and regenerate baseline-relative patch including new files.
- [x] Replace extra evidence policy with exact existing map-model classification; global ESDF supported, static known flag respected, local SDF observed-FREE override removed.
- [x] Real default-filter SDF producer regression and all 56 C++ tests pass under bounded private master.
- [x] Attempt bounded headless default goal smoke (only RViz omitted): nodes initialize, but FSM reports no odom and corridor topic times out; no navigation success claimed. Existing simulator was not replaced.
- [ ] Main session reviews/applies patch; user performs live navigation retest.

## Reproducible isolated commands

    catkin config --workspace /tmp/cane-firi-catkin --init --extend /home/xcg/ws/devel --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3
    catkin build --workspace /tmp/cane-firi-catkin path_searching plan_manage --no-status -j2 -p1
    catkin build --workspace /tmp/cane-firi-catkin path_searching --no-deps --no-status --make-args tests -j2

Sources are symlinked from the verified isolated worktree into /tmp/cane-firi-catkin/src. The shell already exposes ROS Noetic; explicit source /opt/ros/noetic/setup.bash was refused by the isolation guard, so catkin's explicit existing workspace extension supplied dependencies without permission bypass.

Run each /tmp/cane-firi-catkin/devel/lib/path_searching/test_* executable with ROS_MASTER_URI=http://127.0.0.1:11479 and ROS_HOSTNAME=127.0.0.1 under a bounded private roscore. Logs: /tmp/firi-test_*.log; /tmp/cane-firi-roscore.log. Master-less initial static test timed out; rerun with private master passed. Build logs: /tmp/cane-firi-catkin/logs.

Results: 11 FIRI geometry, 11 static MPC/backend, 18 dynamic corridor legacy, 1 kinematic MPPI legacy, 2 path smoother, 3 timed trajectory, 10 timed corridor/feasibility = 56 passed. C++ obstacle-detour fixture ~0.44 ms, six regions (local measurement only). Initial dependency build emitted existing obj_predictor signedness, optional PCL I/O and deprecated gazebo_msgs warnings. Removed new unused visualization helper; an incremental package build had no warnings; final dependency-metadata reconfigure emitted existing CMake project-policy and optional PCL I/O warnings, with no compiler warnings. No ROS navigation/GUI or primary integration result claimed.

Latest logs: firi-model-build.log, firi-model-test-build.log, firi-test_*.log, firi-headless-smoke.log, firi-headless-goal.log, firi-headless-corridor.log. Durable copies accompany the patch under delivery/static-firi in this isolated worktree. No native run skill was available in this session; a bounded subprocess launch omitted only RViz.

## Approved original-edge segment revision (2026-09-07)

The runtime now walks original A* edges without shortcuts, subdividing at a configurable maximum segment length (default 1 m). Each restrictive solve protects both endpoints with the metric clearance norm included in its constraints; final normalized planes certify both endpoints and whole blocked cells. MVIE-only certified roundoff handling is unchanged.

The old min_progress, overlap_depth and overlap_area configuration is removed. Adjacent regions require a local-frame witness with at least 1e-6 m slack (100 times the 1e-8 m vertex feasibility tolerance), rather than a physical-sized disk or fixed area. A failed direct overlap inserts the same FIRI at the shared endpoint and checks both joins; failure clears all output. Connection regions count toward the global region/time budget. No footprint or LFPC guarantee follows.

Snapshot v2 stores max_segment_length. v1 readers explicitly consume and discard historical geometric-only progress/depth/area fields; the new segment length defaults to 1 m, never reinterpreting area as length. Historical failure diagnostics remain intact.

Pruning follow-up verified in /tmp/firi-prune-native: 13 geometry, 5 snapshot, 10 connector/pruning/MVIE-roundoff and 11 static MPC/backend tests pass. Commands: `python3 /tmp/firi-prune-native/checks.py`, `python3 /tmp/firi-prune-native/build_static.py`, `python3 /tmp/firi-prune-native/run_static.py`; logs: final-focused.log, static-build.log, static-test.log in that directory. Historical fixtures reduce from baseline 81 raw regions to 4/4/3 final regions (~23/24/21 ms); 500-edge straight reduces to 3 (~20 ms, test-only cap 600). The default cap still rejects that 500-edge input during raw generation. Point-touch/disjoint skips, uncovered diagonal despite covered vertices, pruning deadline, endpoint seed metadata and arbitrary raw polygons are covered. No solver/map/MPC changes, primary writes, full catkin rebuild or navigation claim. Native compile used -Wall with no warnings; the bounded private master on port 11491 was stopped.

## A* continuous-edge follow-up (2026-09-07)
Isolated incremental implementation checks candidate/start/terminal edges against selected native bins at corridor clearance. Exact reported diagonal rejects; small fixture reroutes and builds a corridor. Latest captured grid reroutes from 80 to 78 edges, all edge checks pass, but full corridor acceptance remains blocked by restrictive separation solver roundoff (-4). No solver tolerance or policy changed. Native verification: 28 geometry/snapshot/roundoff and 11 static MPC/backend tests pass; 2 of 3 new edge tests pass, full snapshot corridor assertion remains failing. Integration and navigation remain pending.

## Verified restrictive-separation numerical follow-up
The captured reroute edge72 starts on an active cell face and previously raises NLopt roundoff despite 1.165928 m geometric clearance. For nonpositive clearance-inclusive seed support, outward radial scaling cannot violate either seed constraint; initialize a sqrt(machine-epsilon) relative step inside the cell constraints in that unbounded feasible interval. Bounded intervals retain closest-face initialization. Tighten restrictive solver constraint tolerance from 1e-9 to 1e-12 so dimensionless feasibility also satisfies unchanged normalized metric endpoint certificates. Successful solver status remains mandatory; no roundoff acceptance, certificate relaxation, retries or alternate solver. Exact seed/cell regression fails baseline and passes corrected source. Final native results: 13 geometry, 5 snapshot, 11 connector/pruning/separation/MVIE, 11 static MPC/backend, 3 A* tests pass. Latest captured path reroutes 80 to 78 edges, produces 3 regions; old invalid diagonal still rejects. Search approximately 0.103 ms locally. Full catkin integration/navigation remain untested.
