# Build, Tests, and Quality

## Build

Use ROS Noetic and `catkin_tools`, not `catkin_make`:

```bash
source /opt/ros/noetic/setup.bash
catkin build nano_gicp -DCMAKE_BUILD_TYPE=Release
catkin build quatro -DCMAKE_BUILD_TYPE=Release -DQUATRO_TBB=ON -DQUATRO_DEBUG=OFF
catkin build -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

Package CMake files generally select C++14, Release/O3, and `-Wall`; some targets also use `-g`. Match the package's existing CMake style and declare ROS/Eigen/PCL dependencies in `CMakeLists.txt` and `package.xml`.

## Tests and runtime checks

Tests are currently sparse and executable-oriented. `plan_env/test/esdf_test.cpp` is built as `esdf_test_node`; `path_searching/test/` contains focused C++ tests for corridor, trajectory, and MPPI helpers. Do not assume a registered gtest/rostest suite: inspect the package CMake target before choosing a test command.

For runtime behavior, build, source `devel/setup.bash`, launch a scenario, and inspect ROS output and RViz. The documented simulation smoke path is:

```bash
roslaunch plan_manage sim_kin_replan.launch planner:=3
```

Use RViz's `2D Nav Goal` to exercise the planner. Hardware paths and dynamic-detection paths are documented in `AGENTS.md`.

## Review checklist

- Does the change preserve the package ownership and ROS topic/frame/parameter contracts?
- Are failure statuses checked and diagnostics useful?
- Are new headers installed/exposed correctly and targets linked in CMake?
- Is there a focused C++ test or a launch/RViz scenario that exercises the changed path?
- Were only actual conventions documented? This repository has no clang-format, clang-tidy, CI, or `-Werror` policy; do not claim otherwise.

## Current-state caveats

The repository contains mixed indentation/naming styles, commented-out code, direct console output, and duplicated legacy controllers. Treat these as existing technical debt, not patterns to expand. Keep new code consistent with the surrounding package while avoiding new duplication and unchecked input.
