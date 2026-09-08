# Project Runtime Guidelines

> Guidance for the ROS1 Noetic navigation runtime in this repository.

## Scope

This layer covers the C++ ROS nodes and libraries that implement mapping, planning, trajectory generation, and motor control. The repository is a catkin workspace organized by runtime package, not an HTTP backend. Use the more specific guides below when changing these areas.

| Guide | Applies to |
|---|---|
| [Package and directory boundaries](./package-boundaries.md) | Any new ROS package, library, node, header, or runtime asset |
| [ROS interfaces and configuration](./ros-interfaces-and-config.md) | Topics, timers, TF, parameters, launch files, and message contracts |
| [Failure handling and diagnostics](./failure-logging.md) | Guard clauses, planner failures, TF/serial errors, and logging |
| [Current-frame pedestrian corridor](./current-frame-pedestrian-corridor.md) | Planner-3 source validity, pure social hulls, whole-polygon FIRI exclusion and STOP limits |
| [Robust underactuated corridor](./robust-underactuated-corridor.md) | Bounded user-speed intervals, conservative timed occupancy, and same-sequence MPPI checks |

## Runtime data flow

The planner stack follows the established pipeline:

```text
plan_env -> path_searching -> bspline / bspline_opt -> plan_ctrl -> plan_manage
```

`plan_manage` owns the FSM and composes the other packages; `plan_env` owns map/collision/prediction state; `path_searching` owns A*, kinodynamic A*, MPPI/LFPC, risk fields, and corridor/trajectory helpers. See `AGENTS.md` and `README.md` for the top-level architecture.

## Static whole-cell separation

Use one common plane for every vertex of an occupied convex cell, protecting both original-edge endpoints with metric clearance. In 2D the runtime enumerates ellipsoid-metric stationary directions, cell-support switches and clearance-active normals; it does not sample angles or solve a positive-offset separation QP. Signed support relative to the moving ellipse center is not a geometric infeasibility test. Preserve normalized endpoint/whole-cell certificates, explicit nonfinite/deadline failures, and MVIE interior checks. Solver status is unavailable for the geometric separation stage; historical snapshot solver diagnostics remain readable. Regression must distinguish independent segment/cell obstruction from numerical failure, including the frozen M6wfx3 same-map sweep.

## Evidence files

- `plan_manage/src/planner_manager.cpp`
- `plan_env/src/map_ros.cpp`
- `path_searching/src/astar.cpp`
- `path_searching/src/mpc_controller.cpp`
- `plan_ctrl/src/L1_controller_v2.cpp`
