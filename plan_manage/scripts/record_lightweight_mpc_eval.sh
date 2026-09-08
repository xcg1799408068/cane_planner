#!/usr/bin/env python3

import datetime
import os
import sys


TOPICS = [
    "/clock",
    "/mpc/transaction_diagnostics",
    "/move_base_simple/goal",
    "/initialpose",
    "/rosout",
    "/rosout_agg",
    "/astar/path",
    "/planning_vis/kinpath_sample",
    "/planning_vis/trajectory",
    "/mpc/convex_corridor",
    "/mpc/convex_corridor_debug",
    "/Odometry",
    "/odom_world",
    "/mpc/debug_metrics",
    "/mpc/stop_advice",
    "/mpc/stop_reason",
    "/mpc/path",
    "/mpc/best_traj",
    "/mpc/current_waypoint",
    "/mpc/waypoints",
    "/mpc/walking_corridors",
    "/simulation_generator/odom",
    "/sim_odom",
    "/onboard_detector/dynamic_obstacles_info",
    "/mpc/interaction_scene",
    "/mpc/interaction_mode",
    "/mpc/interaction_debug",
    "/pedestrian_sim/visualization",
    "/sdf_map/occupancy_local",
    "/sdf_map/occupancy_local_inflate",
]


def main():
    label = sys.argv[1] if len(sys.argv) > 1 else "lightweight"
    root_dir = os.environ.get("MPC_EVAL_RECORD_DIR", "/home/xcg/ws/records")
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = os.path.join(root_dir, "{}_{}".format(stamp, label))
    prefix = os.path.join(out_dir, "mpc_eval")

    os.makedirs(out_dir, exist_ok=True)

    metadata_path = os.path.join(out_dir, "topics.txt")
    with open(metadata_path, "w", encoding="utf-8") as f:
        f.write("label: {}\n".format(label))
        f.write("mode: lightweight\n")
        f.write("created_at: {}\n".format(stamp))
        f.write("bag_prefix: {}\n".format(prefix))
        f.write("odom_topic: /sim_odom\n")
        f.write("topics:\n")
        for topic in TOPICS:
            f.write("  {}\n".format(topic))

    print("[record_lightweight_mpc_eval] Writing bag to: {}_*.bag".format(prefix))
    print("[record_lightweight_mpc_eval] Metadata: {}".format(metadata_path))
    print("[record_lightweight_mpc_eval] Press Ctrl+C to stop recording.")
    sys.stdout.flush()

    argv = ["rosbag", "record", "-O", prefix] + TOPICS
    os.execvp("rosbag", argv)


if __name__ == "__main__":
    main()
