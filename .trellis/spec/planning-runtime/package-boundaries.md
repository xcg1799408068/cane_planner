# Package and Directory Boundaries

## Package layout

This is a ROS1/catkin workspace. Keep code inside the package that owns its runtime responsibility; do not create web-style `services/`, `controllers/`, or `models/` trees.

```text
<package>/
├── include/<package>/   # public C++ headers
├── src/                 # implementations and node entry points
├── launch/              # ROS launch composition
├── config/              # YAML/JSON/RViz runtime configuration
├── script/ or scripts/  # Python ROS utilities and analysis tools
├── test/                # executable-style or C++ tests
├── msg/ / srv/          # custom ROS interfaces, when needed
├── maps/ models/ urdf/  # package-owned runtime assets
├── CMakeLists.txt
└── package.xml
```

Representative layouts are `plan_env/`, `path_searching/`, `plan_manage/`, and `plan_ctrl/`.

## Ownership boundaries

- `plan_env`: SDF/grid maps, collision queries, sensor integration, and object prediction.
- `path_searching`: reusable planning/model libraries such as `Astar`, `KinodynamicAstar`, `MpcController`, `LFPC`, risk fields, and corridor helpers.
- `bspline` / `bspline_opt`: B-spline representation and optional trajectory optimization.
- `plan_manage`: ROS node entry points and `PlannerManager` FSM orchestration.
- `plan_ctrl` / `plan_ctrl2` / `omniGKF_ctrl`: controller and hardware interfaces.
- `Utils`: standalone ROS utility packages and hardware/map helpers.

## Naming and placement

Package and directory names use lowercase snake_case (`plan_env`, `path_searching`). Public headers are below a package-named include directory, and implementation names mirror the role or class (`sdf_map.cpp`, `planner_manager.cpp`, `kinodynamic_astar.cpp`). Node entry points describe runtime roles (`kin_replan_node.cpp`, `sim_plan_node.cpp`).

Put shared algorithm code in a library and expose it through a package header. Put ROS wiring in the corresponding `*_ros.cpp`, manager, or node file. Follow `path_searching/CMakeLists.txt` for a library target and `plan_manage/CMakeLists.txt` for node targets.

## Avoid

- Do not invent HTTP route/service/controller layers for ROS functionality.
- Do not put package-owned launch/config assets at repository root.
- Do not include private implementation headers from another package's `src/`; expose a public header under `include/<package>/` instead.
