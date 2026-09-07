# Markers and Topic Contracts

## Standard message shapes

Prefer standard ROS visualization messages already used by the stack:

- planner output: `nav_msgs/Path` (`/astar/path`, `/mpc/path`, optionally `/kin_astar/path`)
- point clouds: `sensor_msgs/PointCloud2` for map and sensor displays
- goals: `geometry_msgs/PoseStamped` on `/move_base_simple/goal`
- obstacles and debug primitives: `visualization_msgs/Marker` or `MarkerArray`

Dynamic obstacles are represented as cube markers. `plan_env/src/map_ros.cpp` accepts `MarkerArray`, filters `ADD` + `CUBE` markers with positive dimensions, and converts pose/scale into obstacle boxes. `plan_env/src/obj_generator.cpp` publishes simulation cube markers on `/dynamic/obj`.

## Marker construction

Follow the existing controller pattern in `plan_ctrl/src/L1_controller_v2.cpp`: initialize marker frame and namespace once, set clear type/color/scale values, clear and repopulate transient point/line data each cycle, then publish. Use stable topic names and namespaces so RViz can retain display configuration.

Planner visualization is published by `plan_manage/src/planner_manager.cpp` for paths, samples, trajectories, and foot markers. If a marker topic changes, update its RViz display and any scripts that consume it.

## Topic/frame checklist

Before changing an operator-facing message:

1. Find the publisher and subscriber with `grep`.
2. Check the launch remap and parameter source.
3. Check `plan_manage/config/replan.rviz` and any scripts.
4. Verify the frame is connected to `world`/`map`/robot TF at runtime.
5. Exercise an actual launch scenario, not only static YAML syntax.

## Avoid

- Do not use a custom message where `Path`, `Marker(Array)`, `PoseStamped`, or `PointCloud2` already expresses the contract.
- Do not publish markers with zero dimensions, an invalid frame, or an unbounded lifetime when the consumer expects transient updates.
- Do not treat RViz markers as persistent application state; they are visualization output derived from runtime ROS state.
