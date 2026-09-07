# Failure Handling and Diagnostics

## Return-status contract

The dominant failure contract is boolean success/failure plus guard clauses and ROS diagnostics. Collision/geometry helpers return `false` for invalid or out-of-map data (`plan_env/include/plan_env/sdf_map.h`, `plan_env/src/collision_detection.cpp`). A* and kinodynamic A* return `false` for node-pool exhaustion, an empty open set, or no path (`path_searching/src/astar.cpp`, `kinodynamic_astar.cpp`). Callers such as `PlannerManager` check the result, warn, and stop or replan (`plan_manage/src/planner_manager.cpp`).

Make failure handling explicit at the call site. Preserve the caller's ability to distinguish “no path” from a successful empty result.

## Guard clauses and callbacks

ROS callbacks commonly validate sentinel inputs or runtime readiness, warn when useful, and return early. Examples include the negative-z goal guard in `PlannerManager::GoalCallback` and the out-of-map camera guard in `MapROS::depthPoseCallback`.

For new callbacks, validate message shape before indexing vectors or dereferencing optional data. Existing code has unsafe examples such as `PlannerManager::waypointCallback` dereferencing `msg->poses[0]`; do not reproduce that pattern.

## Exceptions

Use local `try/catch (tf::TransformException&)` around TF lookups, log the exception, and return from the current operation. `PlannerManager` and `L1_controller_v2` show this pattern. Serial initialization catches `serial::IOException`, reports the error, checks `isOpen()`, and disables serial use if opening failed.

The codebase has one narrower exceptional geometry path: `plan_env/src/raycast.cpp` reports overflow to `std::cerr` and throws `std::out_of_range`. Do not introduce a project-wide exception hierarchy or HTTP error envelope; none exists.

## Avoid

- Do not swallow a planner failure and continue as though a valid trajectory exists.
- Do not throw exceptions from ordinary ROS callbacks where the established code returns a status or early return.
- Do not assume a message contains a pose, obstacle, or point without checking its size and validity first.
