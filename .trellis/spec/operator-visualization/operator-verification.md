# Operator Verification

## Runtime smoke check

The project has no browser build, frontend lint, accessibility suite, or component test framework. Verify visualization changes through ROS launch and observation:

```bash
source devel/setup.bash
roslaunch plan_manage sim_kin_replan.launch planner:=3
```

Set a goal with RViz's `2D Nav Goal`. Confirm the fixed frame is valid, the goal is received, the relevant path/marker topic updates, and no TF or ROS error prevents the display from rendering.

## Visualization review

For a new or changed display, inspect:

- topic name and message type match the publisher;
- frame is connected and consistent with `world`/`map`/robot frames;
- alpha, scale, color, and lifetime make the display readable without hiding map or obstacle data;
- disabled-by-default displays remain intentional and documented;
- launch files actually start the publisher and RViz config.

Offline Python plots under `LFPC/`, `bspline_opt/script/`, and `plan_manage/scripts/plot_mpc_eval_trajectories.py` are analysis tools, not substitutes for RViz runtime verification.

## Current limitations

This repository does not provide a formal frontend test runner or screenshot automation. If a launch cannot run because ROS/dependencies are unavailable, report that limitation and still validate topic/config consistency statically.
