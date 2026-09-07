# RViz and Launch Layout

## RViz configuration

Keep operator display configuration under the owning package's `config/` directory. `plan_manage/config/replan.rviz` is the primary runtime view; `traj.rviz`, `plan_ctrl2/launch/check_urdf.launch`, and `Utils/map_generator/launch/test.rviz` are package-specific alternatives.

The primary config uses `world` as the fixed frame and includes controls for `SetInitialPose`, `SetGoal` (`/move_base_simple/goal`), and `PublishPoint`. Preserve frame names and display topic names when editing it. Group displays by their existing purpose: map/sensor clouds, planner paths/samples, dynamic obstacles, TF/robot model, and controller markers.

## Launch composition

Launch files compose runtime nodes and load parameters; they are not UI routes. `plan_manage/launch/rviz.launch` should remain a small RViz entry point. `sim_kin_replan.launch` composes algorithm, controller, waypoint generator, pedestrian simulation, and RViz for the documented simulation flow. Shared topic remaps and planner parameters belong in `plan_manage/launch/include/algorithm.launch` or the package config it loads.

When adding an operator display, update both the launch path that starts RViz and the config used by that launch. Confirm the launch works with the fixed frame and the topic's actual publisher.

## Avoid

- Do not create a web `components/` or `pages/` tree for RViz work.
- Do not hard-code a new fixed frame in one display while the config remains fixed to `world`.
- Do not add a display for a topic that no launch path publishes.
