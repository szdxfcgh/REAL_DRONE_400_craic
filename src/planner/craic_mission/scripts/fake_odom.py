#!/usr/bin/env python3
import math
import threading
from typing import Tuple

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


Point = Tuple[float, float, float]


class FakeOdom:
    def __init__(self) -> None:
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.child_frame_id = rospy.get_param("~child_frame_id", "base_link")
        self.odom_topic = rospy.get_param("~odom_topic", "/Odom_high_freq")
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.publish_hz = float(rospy.get_param("~publish_hz", 30.0))
        self.speed_xy = float(rospy.get_param("~speed_xy", 1.2))
        self.speed_z = float(rospy.get_param("~speed_z", 0.8))
        self.snap_tolerance_xy = float(rospy.get_param("~snap_tolerance_xy", 0.02))
        self.snap_tolerance_z = float(rospy.get_param("~snap_tolerance_z", 0.02))

        initial = rospy.get_param("~initial_position", None)
        if initial is None:
            raw_points = rospy.get_param("~points", {})
            initial = raw_points.get("takeoff", [0.0, 0.0, 0.0])

        self.position: Point = (float(initial[0]), float(initial[1]), float(initial[2]))
        self.target: Point = self.position
        self.lock = threading.Lock()
        self.last_step_time = rospy.Time.now()

        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=1)
        rospy.Subscriber(self.goal_topic, PoseStamped, self._goal_cb, queue_size=1)

        period = 1.0 / max(self.publish_hz, 1.0)
        rospy.Timer(rospy.Duration(period), self._timer_cb)
        rospy.logwarn(
            "[fake odom] publishing %s from %s, following goals on %s",
            self.odom_topic,
            self.position,
            self.goal_topic,
        )

    def _goal_cb(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        with self.lock:
            self.target = (p.x, p.y, p.z)
        rospy.logwarn("[fake odom] new target: x=%.2f y=%.2f z=%.2f", p.x, p.y, p.z)

    def _timer_cb(self, _event) -> None:
        now = rospy.Time.now()
        dt = max((now - self.last_step_time).to_sec(), 0.0)
        self.last_step_time = now

        with self.lock:
            self.position = self._advance(self.position, self.target, dt)
            position = self.position

        msg = Odometry()
        msg.header.stamp = now
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = self.child_frame_id
        msg.pose.pose.position.x = position[0]
        msg.pose.pose.position.y = position[1]
        msg.pose.pose.position.z = position[2]
        msg.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(msg)

    def _advance(self, current: Point, target: Point, dt: float) -> Point:
        dx = target[0] - current[0]
        dy = target[1] - current[1]
        dz = target[2] - current[2]

        dist_xy = math.hypot(dx, dy)
        if dist_xy <= self.snap_tolerance_xy and abs(dz) <= self.snap_tolerance_z:
            return target

        next_x = current[0]
        next_y = current[1]
        next_z = current[2]

        max_xy_step = max(self.speed_xy, 0.0) * dt
        if dist_xy > 0.0 and max_xy_step > 0.0:
            ratio = min(1.0, max_xy_step / dist_xy)
            next_x += dx * ratio
            next_y += dy * ratio

        max_z_step = max(self.speed_z, 0.0) * dt
        if abs(dz) > 0.0 and max_z_step > 0.0:
            next_z += math.copysign(min(abs(dz), max_z_step), dz)

        return next_x, next_y, next_z


if __name__ == "__main__":
    rospy.init_node("craic_fake_odom")
    FakeOdom()
    rospy.spin()
