# Python Tools Guidelines

> Conventions for ROS Python utilities, simulation helpers, and offline analysis scripts.

## Scope and placement

Python is used for ROS utility nodes, simulation/bridge scripts, teleoperation, data recording, and offline plots. Keep package runtime scripts under the owning package's `script/` or `scripts/` directory, for example `plan_manage/scripts/`, `plan_manage/script/teleop_cmd.py`, and `omniGKF_ctrl/script/`. Keep standalone model prototypes under `LFPC/` and optimizer analysis under `bspline_opt/script/`.

These scripts are not a web frontend and there is no shared Python application framework.

## ROS script pattern

Use `rospy.init_node`, explicit publishers/subscribers, and callback functions that translate ROS messages into the topic contract. `plan_manage/script/teleop_cmd.py` subscribes to `/move_base_simple/goal`, publishes a `nav_msgs/Path` and a `visualization_msgs/Marker`, and uses a small `main()` loop. `plan_manage/scripts/pedestrian_sim.py` and `plan_manage/scripts/steering_bridge.py` are examples of runtime helper nodes.

Match the package's existing interpreter/import style and install runtime scripts through the package `CMakeLists.txt` when they are launch-time dependencies.

## State and analysis

Small scripts commonly keep callback state in module-level lists or simple objects. `omniGKF_ctrl/script/gkf_info_plot.py` collects message fields, writes CSV, and creates Matplotlib subplots after `rospy.spin()`. Offline prototypes such as `LFPC/LFPC_demo.py` use small classes around Matplotlib artists. Do not turn these scripts into a new framework without a separate design decision.

## Quality and failure handling

Check message fields before indexing, initialize publishers before publishing, and keep topic/frame names aligned with C++ nodes and RViz. For analysis scripts, make input/output paths and units obvious; avoid silently mixing simulation and hardware data. Validate syntax with the available Python interpreter and, for ROS scripts, prefer a launch smoke test when the environment is available.

## Avoid

- Do not add browser, TypeScript, or package-manager conventions to Python ROS tools.
- Do not duplicate a topic conversion already provided by an existing bridge/helper.
- Do not assume callbacks receive non-empty arrays; the existing C++ unchecked `poses[0]` pattern is a known bug class to avoid in Python.
