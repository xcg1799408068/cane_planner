# ROS Interfaces and Configuration

## ROS boundary pattern

Runtime communication is expressed through ROS topics, publishers, subscribers, timers, TF lookups, and generated message types. For example, `PlannerManager::init` in `plan_manage/src/planner_manager.cpp` subscribes to goals, waypoints, odometry, and dynamic obstacles and creates the FSM timer; `MapROS::init` in `plan_env/src/map_ros.cpp` creates sensor subscribers, map publishers, and update/visualization timers.

Keep ROS transport at the node/ROS adapter boundary. Reusable planning code should accept C++ domain values and remain usable without creating publishers in every algorithm function.

## Node entry points

A node entry point should initialize ROS, construct the manager/controller, initialize it, and enter the ROS event loop. Existing examples are `plan_manage/src/kin_replan_node.cpp` and `plan_ctrl/src/L1_controller_node.cpp`.

## Parameters and launch files

Read parameters with `ros::NodeHandle::param` and an inline default, using hierarchical parameter keys. Existing examples include:

- `plan_env/src/sdf_map.cpp` for map dimensions and resolution.
- `path_searching/src/astar.cpp` and `kinodynamic_astar.cpp` for planner limits.
- `plan_manage/src/planner_manager.cpp` for FSM/planner selection.
- `plan_ctrl/src/L1_controller_v2.cpp` for controller settings.

Use launch files and `<rosparam>` to compose YAML/JSON configuration; see `plan_manage/launch/include/fasterlio.launch` and `plan_manage/launch/continuous_detection.launch`. Keep topic names, frame names, and parameter defaults synchronized with the consumer code and RViz config.

## Message and frame contracts

Use standard ROS messages when they fit: `nav_msgs/Path`, `geometry_msgs/PoseStamped`, `sensor_msgs/PointCloud2`, and `visualization_msgs/Marker`/`MarkerArray`. Custom messages live in package `msg/` directories, such as `omniGKF_ctrl/msg/`.

Trace the complete topic/frame contract when changing a message or topic. The default RViz configuration uses `world` as fixed frame and displays planner, map, TF, and obstacle topics; see `plan_manage/config/replan.rviz`.

## Lightweight planner-3 STOP odometry

`PlannerManager::mpcSimStep` must publish a `/sim_odom` heartbeat before returning from invalid-plan STOP when `simulation_ && !gazebo_sim_`. Use `publishSimOdom(true)` to serialize the current LFPC XY/yaw with fresh `ros::Time::now()`, existing `world` frame, zero Z position and zero twist; do not integrate/reset LFPC or alter its control/phase to obtain stopped velocities. Normal calls retain the default moving serialization. Planner 4 dispatches separately; hardware and Gazebo must not acquire this STOP heartbeat.

The existing MapROS simulated-pose override expires after 0.5 s: suppressing odometry during repeated STOP can switch the map ray origin to the stationary simulation generator. Heartbeats fix that publication gap, not initial occupancy fluctuation or arbitrary callback delays. Regressions should call the actual manager STOP branch under a private ROS master and check repeated timestamps, pose/yaw, all six twist components, unchanged LFPC time/control state, normal serialization and mode guards.

## Avoid

- Do not silently rename a topic, frame, or parameter without updating launch files, callbacks, and RViz.
- Do not add a second ad-hoc message shape when an existing ROS message already carries the data.
- Do not move parameter defaults into an undocumented global constant; the current code makes runtime defaults visible at the parameter boundary.
