#!/usr/bin/env python3
import math
import threading
from typing import Optional, Tuple

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import TakeoffLand
from std_msgs.msg import Empty, String


Point = Tuple[float, float, float]


class MinimalMissionFSM:
    IDLE = "IDLE"
    TAKEOFF = "TAKEOFF"
    GOTO_TEST_POINT = "GOTO_TEST_POINT"
    LAND = "LAND"
    END = "END"

    def __init__(self) -> None:
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.odom_topic = rospy.get_param("~topics/odom", "/Odom_high_freq")
        self.goal_topic = rospy.get_param("~topics/goal", "/move_base_simple/goal")
        self.takeoff_land_topic = rospy.get_param("~topics/takeoff_land", "/px4ctrl/takeoff_land")

        self.auto_start = bool(rospy.get_param("~mission/auto_start", False))
        self.mission_timeout_sec = float(rospy.get_param("~mission/mission_timeout_sec", 540.0))
        self.loop_hz = float(rospy.get_param("~mission/loop_hz", 20.0))
        self.command_republish_sec = float(rospy.get_param("~mission/command_republish_sec", 1.0))
        self.goal_republish_sec = float(rospy.get_param("~mission/goal_republish_sec", 1.0))

        self.takeoff_target_height = float(rospy.get_param("~takeoff/target_height", 0.5))
        self.takeoff_timeout_sec = float(rospy.get_param("~takeoff/timeout_sec", 25.0))
        self.takeoff_settle_sec = float(rospy.get_param("~takeoff/settle_sec", 2.0))

        raw_test_point = rospy.get_param("~goal/test_point", [0.5, 0.0, 0.6])
        self.test_point: Point = (
            float(raw_test_point[0]),
            float(raw_test_point[1]),
            float(raw_test_point[2]),
        )
        self.goal_tolerance_xy = float(rospy.get_param("~goal/tolerance_xy", 0.25))
        self.goal_tolerance_z = float(rospy.get_param("~goal/tolerance_z", 0.20))
        self.goal_timeout_sec = float(rospy.get_param("~goal/timeout_sec", 35.0))
        self.land_wait_sec = float(rospy.get_param("~land/wait_sec", 8.0))

        self.odom_timeout_sec = float(rospy.get_param("~safety/odom_timeout_sec", 0.8))
        self.min_x = float(rospy.get_param("~safety/geofence/min_x", -0.5))
        self.max_x = float(rospy.get_param("~safety/geofence/max_x", 9.5))
        self.min_y = float(rospy.get_param("~safety/geofence/min_y", -3.5))
        self.max_y = float(rospy.get_param("~safety/geofence/max_y", 3.5))
        self.min_z = float(rospy.get_param("~safety/geofence/min_z", 0.0))
        self.max_z = float(rospy.get_param("~safety/geofence/max_z", 2.2))

        self.current_odom: Optional[Odometry] = None
        self.last_odom_rx: Optional[rospy.Time] = None
        self.mission_start: Optional[rospy.Time] = None
        self.safety_reason: Optional[str] = None
        self.started = threading.Event()
        self.state = self.IDLE

        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1)
        self.takeoff_land_pub = rospy.Publisher(self.takeoff_land_topic, TakeoffLand, queue_size=1)
        self.state_pub = rospy.Publisher("~state", String, queue_size=1, latch=True)
        self.safety_pub = rospy.Publisher("~safety_reason", String, queue_size=1, latch=True)

        rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=1)
        rospy.Subscriber("~start", Empty, self._start_cb, queue_size=1)

    def _odom_cb(self, msg: Odometry) -> None:
        self.current_odom = msg
        self.last_odom_rx = rospy.Time.now()

    def _start_cb(self, _msg: Empty) -> None:
        self.started.set()

    def _set_state(self, state: str) -> None:
        if self.state != state:
            rospy.logwarn("[minimal mission] %s -> %s", self.state, state)
        self.state = state
        self.state_pub.publish(String(data=state))

    def _current_position(self) -> Optional[Point]:
        if self.current_odom is None:
            return None
        p = self.current_odom.pose.pose.position
        return p.x, p.y, p.z

    def _distance_to(self, point: Point) -> Tuple[float, float]:
        current = self._current_position()
        if current is None:
            return float("inf"), float("inf")
        dx = current[0] - point[0]
        dy = current[1] - point[1]
        dz = current[2] - point[2]
        return math.hypot(dx, dy), abs(dz)

    def _publish_goal(self, point: Point) -> None:
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = point[0]
        msg.pose.position.y = point[1]
        msg.pose.position.z = point[2]
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)

    def _publish_takeoff_land(self, cmd: int) -> None:
        msg = TakeoffLand()
        msg.takeoff_land_cmd = cmd
        self.takeoff_land_pub.publish(msg)

    def _trip_safety(self, reason: str) -> None:
        if self.safety_reason is None:
            self.safety_reason = reason
            rospy.logerr("[minimal mission] safety trigger: %s", reason)
            self.safety_pub.publish(String(data=reason))

    def _check_safety(self) -> bool:
        now = rospy.Time.now()

        if self.mission_start is None:
            return True

        elapsed = (now - self.mission_start).to_sec()
        if elapsed > self.mission_timeout_sec:
            self._trip_safety("MISSION_TIMEOUT %.1fs" % elapsed)
            return False

        if self.last_odom_rx is None:
            self._trip_safety("ODOM_TIMEOUT no_odom")
            return False

        odom_age = (now - self.last_odom_rx).to_sec()
        if odom_age > self.odom_timeout_sec:
            self._trip_safety("ODOM_TIMEOUT %.2fs" % odom_age)
            return False

        current = self._current_position()
        if current is None:
            return True

        x, y, z = current
        if x < self.min_x or x > self.max_x:
            self._trip_safety("GEOFENCE_X %.2f" % x)
            return False
        if y < self.min_y or y > self.max_y:
            self._trip_safety("GEOFENCE_Y %.2f" % y)
            return False
        if z < self.min_z or z > self.max_z:
            self._trip_safety("GEOFENCE_Z %.2f" % z)
            return False

        return True

    def _wait_for_odom_before_start(self) -> bool:
        rate = rospy.Rate(self.loop_hz)
        while not rospy.is_shutdown():
            if self.current_odom is not None:
                return True
            rospy.logwarn_throttle(2.0, "[minimal mission] waiting for odom on %s", self.odom_topic)
            rate.sleep()
        return False

    def _wait_for_start(self) -> bool:
        self._set_state(self.IDLE)
        if self.auto_start:
            self.started.set()

        rate = rospy.Rate(self.loop_hz)
        while not rospy.is_shutdown() and not self.started.is_set():
            self._check_safety()
            rate.sleep()
        return not rospy.is_shutdown()

    def _run_takeoff(self) -> bool:
        self._set_state(self.TAKEOFF)
        start = rospy.Time.now()
        last_cmd = rospy.Time(0)
        rate = rospy.Rate(self.loop_hz)

        while not rospy.is_shutdown():
            if not self._check_safety():
                return False

            now = rospy.Time.now()
            if last_cmd == rospy.Time(0) or (now - last_cmd).to_sec() >= self.command_republish_sec:
                self._publish_takeoff_land(TakeoffLand.TAKEOFF)
                last_cmd = now

            current = self._current_position()
            if current is not None and current[2] >= self.takeoff_target_height:
                rospy.sleep(self.takeoff_settle_sec)
                return self._check_safety()

            if (now - start).to_sec() > self.takeoff_timeout_sec:
                self._trip_safety("TAKEOFF_TIMEOUT")
                return False

            rate.sleep()
        return False

    def _run_goto_test_point(self) -> bool:
        self._set_state(self.GOTO_TEST_POINT)
        start = rospy.Time.now()
        last_goal = rospy.Time(0)
        rate = rospy.Rate(self.loop_hz)

        while not rospy.is_shutdown():
            if not self._check_safety():
                return False

            now = rospy.Time.now()
            if last_goal == rospy.Time(0) or (now - last_goal).to_sec() >= self.goal_republish_sec:
                self._publish_goal(self.test_point)
                last_goal = now

            dist_xy, dist_z = self._distance_to(self.test_point)
            if dist_xy <= self.goal_tolerance_xy and dist_z <= self.goal_tolerance_z:
                rospy.logwarn("[minimal mission] reached test point")
                return True

            if (now - start).to_sec() > self.goal_timeout_sec:
                self._trip_safety("GOTO_TEST_POINT_TIMEOUT")
                return False

            rate.sleep()
        return False

    def _run_land(self) -> None:
        self._set_state(self.LAND)
        start = rospy.Time.now()
        last_cmd = rospy.Time(0)
        rate = rospy.Rate(self.loop_hz)

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            if last_cmd == rospy.Time(0) or (now - last_cmd).to_sec() >= self.command_republish_sec:
                self._publish_takeoff_land(TakeoffLand.LAND)
                last_cmd = now

            if (now - start).to_sec() >= self.land_wait_sec:
                return

            rate.sleep()

    def run(self) -> None:
        if not self._wait_for_odom_before_start():
            return
        if not self._wait_for_start():
            return

        self.mission_start = rospy.Time.now()

        if self._run_takeoff():
            self._run_goto_test_point()

        self._run_land()
        self._set_state(self.END)


if __name__ == "__main__":
    rospy.init_node("craic_mission_minimal")
    MinimalMissionFSM().run()
