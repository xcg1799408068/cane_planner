# Operator Visualization Guidelines

> RViz and ROS-facing operator tools are the project's actual UI layer.

## Scope

There is no web frontend in this repository: no React/Vue/Angular/Svelte, TypeScript, HTML/CSS, `package.json`, roslibjs, rosbridge, Foxglove, Qt, or rqt application. Do not invent browser component, hook, or client-store conventions. Operator-facing work belongs in these guides.

| Guide | Applies to |
|---|---|
| [RViz and launch layout](./rviz-and-launch.md) | RViz configs, launch composition, frames, and displays |
| [Markers and topic contracts](./markers-and-topics.md) | Paths, point clouds, markers, obstacles, and goal topics |
| [Operator verification](./operator-verification.md) | Simulation, RViz inspection, and visualization quality |

## Operator interaction

RViz is the primary operator UI. `plan_manage/launch/rviz.launch` starts `plan_manage/config/replan.rviz`; operators set targets with RViz's `2D Nav Goal`, which publishes `/move_base_simple/goal`. `plan_manage/script/teleop_cmd.py` is a ROS/Python utility for converting goals into waypoints and marker output.

## Evidence files

- `plan_manage/config/replan.rviz`
- `plan_manage/launch/rviz.launch`
- `plan_manage/launch/sim_kin_replan.launch`
- `plan_manage/src/planner_manager.cpp`
- `plan_ctrl/src/L1_controller_v2.cpp`
