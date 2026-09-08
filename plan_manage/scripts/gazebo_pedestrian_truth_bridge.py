#!/usr/bin/env python3
"""
Bridge Gazebo pedestrian model states to onboard_detector/DynamicObstacles.

This is intended for simulation validation: Gazebo provides the pedestrian
truth, while the planner consumes the same topic that LV-DOT would publish.
"""

import math
import re

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point, Vector3
from onboard_detector.msg import DynamicObstacles
from visualization_msgs.msg import Marker, MarkerArray


class GazeboPedestrianTruthBridge:
    def __init__(self):
        rospy.init_node("gazebo_pedestrian_truth_bridge")

        self.model_name_regex = re.compile(
            rospy.get_param("~model_name_regex", r"^person\d+(_.*)?$"))
        self.size_from_name_regex = re.compile(
            rospy.get_param(
                "~size_from_name_regex",
                r".*_(\d+(?:\.\d+)?)_(\d+(?:\.\d+)?)_(\d+(?:\.\d+)?)$"))
        self.default_size = self._vector_param("~default_size", [0.5, 0.5, 1.8])
        self.frame_id = rospy.get_param("~frame_id", "world")
        if self.frame_id != "world":
            raise ValueError("Gazebo ModelStates coordinates are world; arbitrary relabeling is invalid")
        self.publish_rate = float(rospy.get_param("~publish_rate", 15.0))
        self.center_z_from_size = bool(rospy.get_param("~center_z_from_size", True))
        self.publish_markers = bool(rospy.get_param("~publish_markers", True))
        self.dynamic_bbox_size = self._vector_param("~dynamic_bbox_size", [1.0, 1.0, 1.9])
        self.dynamic_bbox_past_time = float(rospy.get_param("~dynamic_bbox_past_time", 1.0))
        self.dynamic_bbox_future_time = float(rospy.get_param("~dynamic_bbox_future_time", 0.2))
        self.dynamic_bbox_min_sweep_speed = float(
            rospy.get_param("~dynamic_bbox_min_sweep_speed", 0.05))

        self.latest_msg = None
        self.prev_pos = {}
        self.prev_stamp = None

        self.obs_pub = rospy.Publisher(
            "/onboard_detector/dynamic_obstacles_info", DynamicObstacles, queue_size=10)
        self.dynamic_bbox_pub = rospy.Publisher(
            "/onboard_detector/dynamic_bboxes", MarkerArray, queue_size=10)
        self.marker_pub = rospy.Publisher(
            "/gazebo_pedestrian_truth/visualization", MarkerArray, queue_size=1)
        self.state_sub = rospy.Subscriber(
            "/gazebo/model_states", ModelStates, self.model_states_cb, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.publish)

        rospy.loginfo("GazeboPedestrianTruthBridge: publishing Gazebo pedestrians to "
                      "/onboard_detector/dynamic_obstacles_info")

    @staticmethod
    def _vector_param(name, default):
        value = rospy.get_param(name, default)
        if len(value) != 3:
            rospy.logwarn("%s must have 3 elements, using %s", name, default)
            value = default
        return [float(value[0]), float(value[1]), float(value[2])]

    def parse_size(self, model_name):
        match = self.size_from_name_regex.match(model_name)
        if not match:
            return self.default_size
        return [float(match.group(1)), float(match.group(2)), float(match.group(3))]

    def model_states_cb(self, msg):
        stamp = rospy.Time.now()
        if len(msg.name) != len(msg.pose) or len(msg.name) != len(msg.twist):
            # Stop emitting: consumer will expire the last real observation.
            self.latest_msg = None
            rospy.logerr_throttle(1.0, "Malformed Gazebo ModelStates arrays")
            return
        dt = None
        if self.prev_stamp is not None:
            dt = (stamp - self.prev_stamp).to_sec()

        out = DynamicObstacles()
        out.header.stamp = stamp
        out.header.frame_id = self.frame_id

        next_prev_pos = {}
        for i, name in enumerate(msg.name):
            if not self.model_name_regex.match(name):
                continue

            pose = msg.pose[i]
            twist = msg.twist[i]
            size = self.parse_size(name)

            x = pose.position.x
            y = pose.position.y
            z = pose.position.z + size[2] * 0.5 if self.center_z_from_size else pose.position.z

            vx = twist.linear.x
            vy = twist.linear.y
            vz = twist.linear.z
            if math.hypot(vx, vy) < 1e-4 and dt is not None and dt > 1e-4:
                prev = self.prev_pos.get(name)
                if prev is not None:
                    vx = (x - prev[0]) / dt
                    vy = (y - prev[1]) / dt
                    vz = (z - prev[2]) / dt

            out.position.append(Vector3(x, y, z))
            out.velocity.append(Vector3(vx, vy, vz))
            out.size.append(Vector3(size[0], size[1], size[2]))
            next_prev_pos[name] = (x, y, z)

        out.num = len(out.position)
        self.latest_msg = out
        self.prev_pos = next_prev_pos
        self.prev_stamp = stamp

    def publish(self, _event):
        if self.latest_msg is None:
            return
        # Never refresh cached observation stamps: source timeout is not empty.
        age = (rospy.Time.now() - self.latest_msg.header.stamp).to_sec()
        if age < 0.0 or age > 0.5:
            markers = MarkerArray()
            clear = Marker()
            clear.header.frame_id = self.frame_id
            clear.header.stamp = rospy.Time.now()
            clear.action = Marker.DELETEALL
            markers.markers.append(clear)
            self.dynamic_bbox_pub.publish(markers)
            if self.publish_markers:
                self.marker_pub.publish(markers)
            return
        self.obs_pub.publish(self.latest_msg)
        self.dynamic_bbox_pub.publish(self.make_dynamic_bboxes(self.latest_msg))
        if self.publish_markers:
            self.marker_pub.publish(self.make_markers(self.latest_msg))

    def make_dynamic_bboxes(self, msg):
        markers = MarkerArray()

        delete_all = Marker()
        delete_all.header = msg.header
        delete_all.action = Marker.DELETEALL
        delete_all.pose.orientation.w = 1.0
        markers.markers.append(delete_all)

        for i in range(msg.num):
            cx = msg.position[i].x
            cy = msg.position[i].y
            cz = msg.position[i].z
            sx = self.dynamic_bbox_size[0]
            sy = self.dynamic_bbox_size[1]
            sz = self.dynamic_bbox_size[2]

            vx = msg.velocity[i].x
            vy = msg.velocity[i].y
            speed = math.hypot(vx, vy)
            if speed >= self.dynamic_bbox_min_sweep_speed:
                start_x = cx - vx * self.dynamic_bbox_past_time
                start_y = cy - vy * self.dynamic_bbox_past_time
                end_x = cx + vx * self.dynamic_bbox_future_time
                end_y = cy + vy * self.dynamic_bbox_future_time
                lower_x = min(start_x, end_x) - sx * 0.5
                lower_y = min(start_y, end_y) - sy * 0.5
                upper_x = max(start_x, end_x) + sx * 0.5
                upper_y = max(start_y, end_y) + sy * 0.5
                cx = 0.5 * (lower_x + upper_x)
                cy = 0.5 * (lower_y + upper_y)
                sx = upper_x - lower_x
                sy = upper_y - lower_y

            box = Marker()
            box.header = msg.header
            box.ns = "gazebo_pedestrian_truth_dynamic_bboxes"
            box.id = i
            box.type = Marker.CUBE
            box.action = Marker.ADD
            box.pose.position.x = cx
            box.pose.position.y = cy
            box.pose.position.z = cz
            box.pose.orientation.w = 1.0
            box.scale.x = sx
            box.scale.y = sy
            box.scale.z = sz
            box.color.r = 0.0
            box.color.g = 0.9
            box.color.b = 1.0
            box.color.a = 0.35
            markers.markers.append(box)

        return markers

    def make_markers(self, msg):
        markers = MarkerArray()

        delete_all = Marker()
        delete_all.header = msg.header
        delete_all.action = Marker.DELETEALL
        delete_all.pose.orientation.w = 1.0
        markers.markers.append(delete_all)

        for i in range(msg.num):
            box = Marker()
            box.header = msg.header
            box.ns = "gazebo_pedestrian_truth"
            box.id = i
            box.type = Marker.CUBE
            box.action = Marker.ADD
            box.pose.position.x = msg.position[i].x
            box.pose.position.y = msg.position[i].y
            box.pose.position.z = msg.position[i].z
            box.pose.orientation.w = 1.0
            box.scale.x = msg.size[i].x
            box.scale.y = msg.size[i].y
            box.scale.z = msg.size[i].z
            box.color.r = 1.0
            box.color.g = 0.35
            box.color.b = 0.05
            box.color.a = 0.45
            markers.markers.append(box)

            speed = math.hypot(msg.velocity[i].x, msg.velocity[i].y)
            if speed < 1e-3:
                continue
            arrow = Marker()
            arrow.header = msg.header
            arrow.ns = "gazebo_pedestrian_truth_velocity"
            arrow.id = i
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.orientation.w = 1.0
            start = Point(x=msg.position[i].x, y=msg.position[i].y, z=msg.position[i].z)
            arrow.points.append(start)
            arrow.points.append(Point(
                x=msg.position[i].x + msg.velocity[i].x,
                y=msg.position[i].y + msg.velocity[i].y,
                z=msg.position[i].z))
            arrow.scale.x = 0.05
            arrow.scale.y = 0.12
            arrow.scale.z = 0.12
            arrow.color.r = 0.1
            arrow.color.g = 0.75
            arrow.color.b = 1.0
            arrow.color.a = 0.9
            markers.markers.append(arrow)

        return markers


if __name__ == "__main__":
    try:
        GazeboPedestrianTruthBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
