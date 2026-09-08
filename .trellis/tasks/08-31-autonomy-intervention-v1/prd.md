# Planner-3 current-frame dynamic walking corridor

Approved implementation scope (2026-09-08), incremental against main HEAD `96bc339` (static baseline `88629bb`). Historical intervention Phase A/B, timed prediction corridors and previous experimental gates are abandoned. Existing static contracts in `static-firi-corridor.md` remain authoritative.

## Requirements

- Extend the single ConvexCorridor/FIRI pipeline: original static A* edges + selected static blocked-bin grid + current-frame convex pedestrian obstacles -> corridor union -> unchanged MPPI. No dynamic A*, seed movement, alternative generator, risk cost or timed slices.
- Preserve A* clearance preference, all three MPPI motion costs, foot/CoM hard checks, union check, weighted rerollout/fallback, STOP heartbeat and planner 4.
- Source disabled means static mode; enabled missing/stale/malformed/untransformable data fails closed as DYNAMIC_DATA_INVALID. A fresh empty frame is valid. Retain header, position, velocity and full dimensions; transform points and vectors rather than relabel frames.
- First-version pure bounded social-force polygon module uses observed velocity as desired velocity (zero initial drive), nearest selected-grid whole-cell boundary and human/cane centre surrogate repulsion, with 220-degree system FOV amplification. No stochastic, attraction or inter-pedestrian force. Explicit finite parameters and units. Zero speed uses symmetric physical safety geometry; zero force has no direction. Always convex-hull physical extent with social keypoints, including reverse force.
- Generalize the existing finite angular separator to a complete convex polygon with one common plane and unchanged certificates. No rasterization or independent vertex planes. Static empty-input behavior remains unchanged.
- Any protected reference segment intersecting a pedestrian polygon rejects as DYNAMIC_REFERENCE_BLOCKED and clears output. Re-evaluate next cycle; fresh clearance may resume. No automatic reference repair.
- Dynamic failures must not generate incomplete static-only replay evidence. Either compatible snapshots contain polygons or capture is explicitly suppressed with a reason.
- Existing lightweight pedestrian simulator supplies deterministic approach/crossing/stationary scenarios; no dependency installation or pedsim. Existing no-pedestrian defaults remain usable. Propagate the source-enable contract through all planner entrypoints, including Gazebo truth sources.
- Stable transient body/social-hull markers clear on empty/invalid/reset. Existing metric fields remain append-only. Explicit diagnostic reasons, no dense labels.

## Acceptance

- Pure force/hull tests cover physical extent, reverse/zero force, zero speed, invalid inputs and wall/system directions.
- Corridor tests cover whole-polygon exclusion, containing/crossing references, approach/depart and static equivalence.
- Actual manager tests cover missing/fresh-empty/stale/malformed/frame-transform transitions, blocked-to-clear recovery and unchanged frozen STOP odometry.
- Isolated catkin_tools Noetic builds and registered tests; static and single-pedestrian timings. No main source/devel writes, user simulation launch, installation, commits or publication.
- Deliver incremental patch, changed-file list and logs in isolated delivery directory for independent main-session review. Navigation remains user-run.

## Guarantee limits

Current-frame geometry only, not pedestrian trajectory prediction or future collision safety. STOP is no physical braking guarantee for an underactuated cane. Existing local reference horizon may cause conservative early STOP for a farther crossing. Centre repulsion is not a human/cane footprint certificate. Model units/parameters are an explicit engineering approximation, not validated pedestrian intent or exact paper reproduction.
