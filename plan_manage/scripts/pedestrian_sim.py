#!/usr/bin/env python3
"""
Simple pedestrian simulator for testing MPC dynamic obstacle avoidance.
Publishes onboard_detector::DynamicObstacles and visualization markers.
"""
import rospy
import numpy as np
import rospkg
import yaml
from geometry_msgs.msg import Vector3, Point, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from onboard_detector.msg import DynamicObstacles


class Pedestrian:
    def __init__(self, pid, start, vel, size, period, mode="trigger"):
        self.pid = pid
        self.start = np.array(start, dtype=float)
        self.vel_mag = np.array(vel, dtype=float)
        self.pos = self.start.copy()
        self.vel = self.vel_mag.copy()
        self.size = np.array(size, dtype=float)
        self.period = float(period)
        self.mode = mode           # "patrol", "loop", "trigger"
        self.elapsed = 0.0
        self.active = False        # trigger mode: waits for goal
        self.trail = []
        self.max_trail = 30

    def trigger(self):
        """Reset to start and begin walking."""
        self.elapsed = 0.0
        self.pos = self.start.copy()
        self.vel = self.vel_mag.copy()
        self.active = True

    def step(self, dt):
        if self.mode == "trigger":
            if not self.active:
                self.vel = np.zeros(3)
                return
            self.elapsed += dt
            if self.elapsed >= self.period:
                self.active = False
                self.vel = np.zeros(3)
                return
            self.vel = self.vel_mag.copy()
            self.pos = self.start + self.vel_mag * self.elapsed

        elif self.mode == "loop":
            self.elapsed += dt
            self.vel = self.vel_mag.copy()
            self.pos = self.start + self.vel_mag * self.elapsed
            if self.elapsed >= self.period:
                self.elapsed = 0.0
                self.pos = self.start.copy()

        else:  # patrol: back and forth, start = center
            self.elapsed += dt
            if self.elapsed >= self.period:
                self.elapsed -= self.period
            t = self.elapsed
            if t < self.period / 4.0:
                self.vel = self.vel_mag
            elif t < self.period * 3.0 / 4.0:
                self.vel = -self.vel_mag
            else:
                self.vel = self.vel_mag
            self.pos = self.pos + self.vel * dt

        self.trail.append(self.pos.copy())
        if len(self.trail) > self.max_trail:
            self.trail.pop(0)


class PedestrianSim:
    def __init__(self):
        rospy.init_node("pedestrian_sim")
        # These configured coordinates belong to this frame; no TF relabeling.
        self.frame_id = rospy.get_param("~frame_id", "map")
        if not isinstance(self.frame_id, str) or not self.frame_id:
            raise ValueError("frame_id must be a nonempty coordinate frame")

        # Publishers
        self.obs_pub = rospy.Publisher(
            "/onboard_detector/dynamic_obstacles_info", DynamicObstacles, queue_size=10)
        self.viz_pub = rospy.Publisher(
            "/pedestrian_sim/visualization", MarkerArray, queue_size=10)

        # Subscribe to goal topic to trigger pedestrians
        self.goal_sub = rospy.Subscriber(
            "/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=1)
        self._triggered = False

        default_pedestrians = [
            {"start": [-3.0, 3.0, 0.0], "vel": [0.8, 0.0, 0.0], "size": [0.5, 0.5, 1.7], "period": 5.0},
            {"start": [3.0, -2.0, 0.0], "vel": [-0.6, 0.3, 0.0], "size": [0.5, 0.5, 1.7], "period": 4.0},
        ]
        ped_configs = rospy.get_param("~pedestrians", None)
        if ped_configs is None:
            ped_configs = self.load_scenario(default_pedestrians)

        self.pedestrians = []
        for i, cfg in enumerate(ped_configs):
            period = cfg.get("period", 5.0)
            mode = cfg.get("mode", "trigger")
            self.pedestrians.append(Pedestrian(
                pid=i, start=cfg["start"], vel=cfg["vel"],
                size=cfg["size"], period=period, mode=mode))

        self.rate = rospy.Rate(10)  # 10 Hz
        rospy.loginfo("PedestrianSim: %d pedestrians, 10Hz", len(self.pedestrians))

    def load_scenario(self, default_pedestrians):
        scenario = rospy.get_param("~scenario", "mixed")
        config_path = rospy.get_param("~scenario_config", self.default_scenario_config())
        try:
            with open(config_path, "r") as f:
                config = yaml.safe_load(f) or {}
        except Exception as exc:
            rospy.logwarn(
                "PedestrianSim: failed to read scenario config %s: %s; using defaults",
                config_path,
                exc,
            )
            return default_pedestrians
        if scenario not in config:
            rospy.logwarn(
                "PedestrianSim: scenario '%s' not found in %s; available=%s; using defaults",
                scenario,
                config_path,
                sorted(config.keys()),
            )
            return default_pedestrians
        rospy.loginfo("PedestrianSim: scenario=%s config=%s", scenario, config_path)
        return config[scenario]

    def default_scenario_config(self):
        package_path = rospkg.RosPack().get_path("plan_manage")
        return package_path + "/config/pedestrian_scenarios.yaml"

    def goal_cb(self, msg):
        """Retrigger all pedestrians when a new goal is set."""
        rospy.loginfo("PedestrianSim: goal received, triggering pedestrians")
        for p in self.pedestrians:
            p.trigger()
        self._triggered = True

    def publish_obstacles(self):
        msg = DynamicObstacles()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        # Only publish active pedestrians (in trigger mode: not yet walked full period)
        active = [p for p in self.pedestrians
                  if p.mode != "trigger" or p.active]
        msg.num = len(active)
        for p in active:
            msg.position.append(Vector3(p.pos[0], p.pos[1], p.pos[2]))
            msg.velocity.append(Vector3(p.vel[0], p.vel[1], p.vel[2]))
            msg.size.append(Vector3(p.size[0], p.size[1], p.size[2]))
        self.obs_pub.publish(msg)

    def publish_viz(self):
        arr = MarkerArray()
        del_mk = Marker()
        del_mk.header.frame_id = self.frame_id
        del_mk.header.stamp = rospy.Time.now()
        del_mk.action = Marker.DELETEALL
        arr.markers.append(del_mk)

        for p in self.pedestrians:
            # Trigger mode: only show active pedestrians
            if p.mode == "trigger" and not p.active:
                continue

            # Bounding box
            mk = Marker()
            mk.header.frame_id = self.frame_id
            mk.header.stamp = rospy.Time.now()
            mk.ns = "ped_{}".format(p.pid)
            mk.id = 0
            mk.type = Marker.CUBE
            mk.action = Marker.ADD
            mk.pose.position.x = p.pos[0]
            mk.pose.position.y = p.pos[1]
            mk.pose.position.z = p.size[2] / 2.0
            mk.pose.orientation.w = 1.0
            mk.scale.x = p.size[0]
            mk.scale.y = p.size[1]
            mk.scale.z = p.size[2]
            if p.mode == "trigger":
                mk.color.r, mk.color.g, mk.color.b = 1.0, 0.2, 0.2  # red
            elif p.mode == "loop":
                mk.color.r, mk.color.g, mk.color.b = 0.2, 0.4, 1.0
            elif p.pid == 0:
                mk.color.r, mk.color.g, mk.color.b = 1.0, 0.3, 0.0
            else:
                mk.color.r, mk.color.g, mk.color.b = 1.0, 0.6, 0.0
            mk.color.a = 0.7
            arr.markers.append(mk)

            # Velocity arrow
            mk2 = Marker()
            mk2.header.frame_id = self.frame_id
            mk2.header.stamp = rospy.Time.now()
            mk2.ns = "ped_{}_vel".format(p.pid)
            mk2.id = 0
            mk2.type = Marker.ARROW
            mk2.action = Marker.ADD
            mk2.pose.position.x = p.pos[0]
            mk2.pose.position.y = p.pos[1]
            mk2.pose.position.z = p.size[2] / 2.0
            vel_norm = np.linalg.norm(p.vel[:2])
            if vel_norm > 0.01:
                yaw = np.arctan2(p.vel[1], p.vel[0])
                from tf.transformations import quaternion_from_euler
                q = quaternion_from_euler(0, 0, yaw)
                mk2.pose.orientation.x = q[0]
                mk2.pose.orientation.y = q[1]
                mk2.pose.orientation.z = q[2]
                mk2.pose.orientation.w = q[3]
            mk2.scale.x = vel_norm * 0.5 + 0.3
            mk2.scale.y = 0.08
            mk2.scale.z = 0.08
            mk2.color.a = 0.9
            mk2.color.r = 1.0
            mk2.color.g = 0.2
            mk2.color.b = 0.2
            arr.markers.append(mk2)

        self.viz_pub.publish(arr)

    def run(self):
        dt = 0.1
        while not rospy.is_shutdown():
            for p in self.pedestrians:
                p.step(dt)
            self.publish_obstacles()
            self.publish_viz()
            self.rate.sleep()


if __name__ == "__main__":
    try:
        PedestrianSim().run()
    except rospy.ROSInterruptException:
        pass
