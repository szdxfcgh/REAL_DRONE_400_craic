#!/usr/bin/env python3
from typing import Optional

import rospy
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import TakeoffLand
from std_msgs.msg import String


class SafetyMonitor:
    def __init__(self) -> None:
        self.odom_topic = rospy.get_param("~odom_topic", "/Odom_high_freq")
        self.takeoff_land_topic = rospy.get_param("~takeoff_land_topic", "/px4ctrl/takeoff_land")
        self.enable_auto_land = bool(rospy.get_param("~enable_auto_land", False))
        self.odom_timeout = float(rospy.get_param("~odom_timeout", 0.8))

        self.min_x = float(rospy.get_param("~field/min_x", -0.8))
        self.max_x = float(rospy.get_param("~field/max_x", 9.2))
        self.min_y = float(rospy.get_param("~field/min_y", -3.2))
        self.max_y = float(rospy.get_param("~field/max_y", 3.2))
        self.min_z = float(rospy.get_param("~field/min_z", 0.0))
        self.max_z = float(rospy.get_param("~field/max_z", 2.8))

        self.last_odom_time: Optional[rospy.Time] = None
        self.status_pub = rospy.Publisher("~status", String, queue_size=1, latch=True)
        self.land_pub = rospy.Publisher(self.takeoff_land_topic, TakeoffLand, queue_size=1)

        rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=1)
        rospy.Timer(rospy.Duration(0.1), self._timer_cb)

    def _publish_status(self, status: str) -> None:
        rospy.logwarn("[safety] %s", status)
        self.status_pub.publish(String(data=status))

    def _request_land(self) -> None:
        if not self.enable_auto_land:
            return
        msg = TakeoffLand()
        msg.takeoff_land_cmd = 2
        self.land_pub.publish(msg)

    def _odom_cb(self, msg: Odometry) -> None:
        self.last_odom_time = rospy.Time.now()
        p = msg.pose.pose.position
        violations = []
        if p.x < self.min_x or p.x > self.max_x:
            violations.append("x=%.2f" % p.x)
        if p.y < self.min_y or p.y > self.max_y:
            violations.append("y=%.2f" % p.y)
        if p.z < self.min_z or p.z > self.max_z:
            violations.append("z=%.2f" % p.z)

        if violations:
            self._publish_status("OUT_OF_FIELD " + " ".join(violations))
            self._request_land()
        else:
            self.status_pub.publish(String(data="OK"))

    def _timer_cb(self, _event) -> None:
        if self.last_odom_time is None:
            return
        age = (rospy.Time.now() - self.last_odom_time).to_sec()
        if age > self.odom_timeout:
            self._publish_status("ODOM_TIMEOUT %.2fs" % age)
            self._request_land()


if __name__ == "__main__":
    rospy.init_node("craic_safety_monitor")
    SafetyMonitor()
    rospy.spin()
