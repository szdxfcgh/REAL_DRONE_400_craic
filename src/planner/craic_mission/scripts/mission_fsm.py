#!/usr/bin/env python3
import math
import threading
from typing import Dict, List, Optional, Tuple

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import TakeoffLand
from std_msgs.msg import Empty, Int32, String


Point = Tuple[float, float, float]


class CraicMissionFSM:
    def __init__(self) -> None:
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.odom_topic = rospy.get_param("~odom_topic", "/Odom_high_freq")
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.takeoff_land_topic = rospy.get_param("~takeoff_land_topic", "/px4ctrl/takeoff_land")
        self.drop_cmd_topic = rospy.get_param("~drop_cmd_topic", "/craic/drop_cmd")
        self.qr_result_topic = rospy.get_param("~qr_result_topic", "/craic/qr_result")
        self.target_detected_topic = rospy.get_param("~target_detected_topic", "/craic/target_detected")
        self.ring_pose_topic = rospy.get_param("~ring_pose_topic", "/craic/ring_pose")

        self.cruise_height = float(rospy.get_param("~mission/cruise_height", 1.35))
        self.ring_height = float(rospy.get_param("~mission/ring_height", 1.60))
        self.drop_height = float(rospy.get_param("~mission/drop_height", 0.75))
        self.land_approach_height = float(rospy.get_param("~mission/land_approach_height", 1.20))
        self.goal_tolerance_xy = float(rospy.get_param("~mission/goal_tolerance_xy", 0.35))
        self.goal_tolerance_z = float(rospy.get_param("~mission/goal_tolerance_z", 0.25))
        self.goal_timeout = float(rospy.get_param("~mission/goal_timeout", 35.0))
        self.drop_settle_time = float(rospy.get_param("~mission/drop_settle_time", 1.5))
        self.target_confirm_timeout = float(rospy.get_param("~mission/target_confirm_timeout", 4.0))
        self.target_min_confidence = float(rospy.get_param("~mission/target_min_confidence", 0.50))
        self.takeoff_wait_timeout = float(rospy.get_param("~mission/takeoff_wait_timeout", 20.0))
        self.landing_wait_time = float(rospy.get_param("~mission/landing_wait_time", 8.0))
        self.obstacle_laps = int(rospy.get_param("~mission/obstacle_laps", 1))
        self.obstacle_radius = float(rospy.get_param("~mission/obstacle_radius", 1.05))
        self.obstacle_segments = int(rospy.get_param("~mission/obstacle_segments_per_lap", 8))
        self.dry_run = bool(rospy.get_param("~mission/dry_run", True))
        self.auto_start = bool(rospy.get_param("~mission/auto_start", False))
        self.mock_qr_text = str(rospy.get_param("~mission/mock_qr_text", "man,apple,left"))

        self.points = self._load_points()
        self.current_odom: Optional[Odometry] = None
        self.latest_qr: Optional[str] = None
        self.latest_target: Optional[Tuple[str, float, float, float, rospy.Time]] = None
        self.latest_ring_pose: Optional[PoseStamped] = None
        self.started = threading.Event()
        self.target_lock = threading.Lock()

        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1)
        self.takeoff_land_pub = rospy.Publisher(self.takeoff_land_topic, TakeoffLand, queue_size=1)
        self.drop_pub = rospy.Publisher(self.drop_cmd_topic, Int32, queue_size=1)
        self.state_pub = rospy.Publisher("~state", String, queue_size=1, latch=True)

        rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=1)
        rospy.Subscriber(self.qr_result_topic, String, self._qr_cb, queue_size=1)
        rospy.Subscriber(self.target_detected_topic, String, self._target_cb, queue_size=5)
        rospy.Subscriber(self.ring_pose_topic, PoseStamped, self._ring_cb, queue_size=1)
        rospy.Subscriber("~start", Empty, self._start_cb, queue_size=1)

    def _load_points(self) -> Dict[str, Point]:
        raw_points = rospy.get_param("~points", {})
        defaults = {
            "takeoff": [0.0, 0.0, 0.0],
            "qr_scan": [0.5, 0.0, self.cruise_height],
            "obstacle_center": [3.0, 0.0, self.cruise_height],
            "image_target_a": [5.3, 1.6, self.drop_height],
            "image_target_b": [5.3, -1.6, self.drop_height],
            "special_target": [6.4, 0.0, self.drop_height],
            "ring_pre": [6.2, 0.0, self.ring_height],
            "ring_center": [7.5, 0.0, self.ring_height],
            "ring_post": [8.6, 0.0, self.ring_height],
            "landing_left": [8.2, 1.4, 0.0],
            "landing_right": [8.2, -1.4, 0.0],
        }

        points: Dict[str, Point] = {}
        for key, fallback in defaults.items():
            value = raw_points.get(key, fallback)
            points[key] = (float(value[0]), float(value[1]), float(value[2]))
        return points

    def _odom_cb(self, msg: Odometry) -> None:
        self.current_odom = msg

    def _qr_cb(self, msg: String) -> None:
        self.latest_qr = msg.data.strip()

    def _target_cb(self, msg: String) -> None:
        parsed = self._parse_target_msg(msg.data)
        if parsed is None:
            rospy.logwarn("[CRAIC mission] ignore invalid target detection: %s", msg.data)
            return
        with self.target_lock:
            self.latest_target = parsed[0], parsed[1], parsed[2], parsed[3], rospy.Time.now()

    def _ring_cb(self, msg: PoseStamped) -> None:
        self.latest_ring_pose = msg

    def _start_cb(self, _msg: Empty) -> None:
        self.started.set()

    def _set_state(self, state: str) -> None:
        rospy.logwarn("[CRAIC mission] %s", state)
        self.state_pub.publish(String(data=state))

    def _publish_takeoff_land(self, cmd: int) -> None:
        if self.dry_run:
            rospy.logwarn("[CRAIC mission] dry_run: skip takeoff_land_cmd=%d", cmd)
            return
        msg = TakeoffLand()
        msg.takeoff_land_cmd = cmd
        self.takeoff_land_pub.publish(msg)

    def _publish_goal(self, point: Point) -> None:
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = point[0]
        msg.pose.position.y = point[1]
        msg.pose.position.z = point[2]
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)
        rospy.loginfo("[CRAIC mission] goal: x=%.2f y=%.2f z=%.2f", point[0], point[1], point[2])

    def _current_position(self) -> Optional[Point]:
        if self.current_odom is None:
            return None
        p = self.current_odom.pose.pose.position
        return (p.x, p.y, p.z)

    def _distance_to(self, point: Point) -> Tuple[float, float]:
        current = self._current_position()
        if current is None:
            return float("inf"), float("inf")
        dx = current[0] - point[0]
        dy = current[1] - point[1]
        dz = current[2] - point[2]
        return math.hypot(dx, dy), abs(dz)

    def _wait_for_odom(self, timeout: float = 10.0) -> bool:
        start = rospy.Time.now()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.current_odom is not None:
                return True
            if (rospy.Time.now() - start).to_sec() > timeout:
                return False
            rate.sleep()
        return False

    def _go_to(self, point: Point, label: str, timeout: Optional[float] = None) -> bool:
        timeout = self.goal_timeout if timeout is None else timeout
        self._set_state("GO_TO_" + label)
        start = rospy.Time.now()
        last_pub = rospy.Time(0)
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if last_pub == rospy.Time(0) or (rospy.Time.now() - last_pub).to_sec() > 2.0:
                self._publish_goal(point)
                last_pub = rospy.Time.now()
            dist_xy, dist_z = self._distance_to(point)
            if dist_xy <= self.goal_tolerance_xy and dist_z <= self.goal_tolerance_z:
                rospy.logwarn("[CRAIC mission] reached %s", label)
                return True
            if (rospy.Time.now() - start).to_sec() > timeout:
                rospy.logerr("[CRAIC mission] timeout at %s, dist_xy=%.2f dist_z=%.2f", label, dist_xy, dist_z)
                return False
            rate.sleep()
        return False

    def _parse_qr(self) -> Tuple[str, str, str]:
        raw = self.latest_qr or self.mock_qr_text
        if raw.upper().startswith("QR:"):
            raw = raw[3:]
        parts = [part.strip().lower() for part in raw.split(",")]
        if len(parts) != 3 or parts[2] not in ("left", "right"):
            rospy.logerr("[CRAIC mission] invalid QR text '%s', fallback to %s", raw, self.mock_qr_text)
            parts = [part.strip().lower() for part in self.mock_qr_text.split(",")]
        return parts[0], parts[1], parts[2]

    def _parse_target_msg(self, raw: str) -> Optional[Tuple[str, float, float, float]]:
        text = raw.strip()
        if text.upper().startswith("TARGET:"):
            text = text[7:]
        parts = [part.strip() for part in text.split(",")]
        if len(parts) < 4:
            return None
        try:
            label = parts[0].lower()
            confidence = float(parts[1])
            center_x = float(parts[2])
            center_y = float(parts[3])
        except ValueError:
            return None
        if not label:
            return None
        return label, confidence, center_x, center_y

    def _orbit_waypoints(self) -> List[Point]:
        center = self.points["obstacle_center"]
        total_segments = max(4, self.obstacle_segments) * max(1, self.obstacle_laps)
        waypoints: List[Point] = []

        for idx in range(total_segments + 1):
            theta = -2.0 * math.pi * idx / max(1, self.obstacle_segments)
            x = center[0] + self.obstacle_radius * math.cos(theta)
            y = center[1] + self.obstacle_radius * math.sin(theta)
            waypoints.append((x, y, self.cruise_height))
        return waypoints

    def _drop(self, drop_id: int, label: str) -> None:
        self._set_state("DROP_" + label)
        rospy.logwarn("[CRAIC mission] drop %d at %s", drop_id, label)
        self.drop_pub.publish(Int32(data=drop_id))
        rospy.sleep(self.drop_settle_time)

    def _wait_for_target_confirm(self, expected_class: str, label: str) -> bool:
        expected = expected_class.strip().lower()
        if not expected or self.target_confirm_timeout <= 0.0:
            return False

        self._set_state("WAIT_TARGET_" + label)
        start = rospy.Time.now()
        last_seen: Optional[Tuple[str, float, float, float, rospy.Time]] = None
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start).to_sec()
            if elapsed > self.target_confirm_timeout:
                break

            with self.target_lock:
                target = self.latest_target

            if target is not None and target[4] >= start:
                last_seen = target
                name, confidence, center_x, center_y, _stamp = target
                if name == expected and confidence >= self.target_min_confidence:
                    rospy.logwarn(
                        "[CRAIC mission] target confirmed: expected=%s confidence=%.2f center=(%.0f,%.0f)",
                        expected,
                        confidence,
                        center_x,
                        center_y,
                    )
                    return True

                rospy.loginfo_throttle(
                    1.0,
                    "[CRAIC mission] waiting target=%s, saw=%s confidence=%.2f",
                    expected,
                    name,
                    confidence,
                )

            rate.sleep()

        if last_seen is None:
            rospy.logwarn("[CRAIC mission] target confirm timeout at %s: no K230 target seen, continue default drop", label)
        else:
            rospy.logwarn(
                "[CRAIC mission] target confirm timeout at %s: expected=%s last=%s confidence=%.2f, continue default drop",
                label,
                expected,
                last_seen[0],
                last_seen[1],
            )
        return False

    def _drop_at(self, point_name: str, drop_id: int, label: str, expected_class: Optional[str] = None) -> bool:
        target = self.points[point_name]
        cruise = (target[0], target[1], self.cruise_height)
        drop = (target[0], target[1], self.drop_height)
        if not self._go_to(cruise, label + "_CRUISE"):
            return False
        if not self._go_to(drop, label + "_DROP_HEIGHT"):
            return False
        if expected_class is not None:
            self._wait_for_target_confirm(expected_class, label)
        self._drop(drop_id, label)
        return self._go_to(cruise, label + "_CLIMB")

    def run(self) -> None:
        if not self._wait_for_odom():
            rospy.logerr("[CRAIC mission] no odometry on %s", self.odom_topic)
            return

        if self.auto_start:
            self.started.set()

        self._set_state("WAIT_START")
        while not rospy.is_shutdown() and not self.started.is_set():
            rospy.sleep(0.1)

        if rospy.is_shutdown():
            return

        self._set_state("TAKEOFF")
        self._publish_takeoff_land(1)
        takeoff_target = (self.points["takeoff"][0], self.points["takeoff"][1], self.cruise_height)
        self._go_to(takeoff_target, "TAKEOFF_CLIMB", timeout=self.takeoff_wait_timeout)

        self._go_to(self.points["qr_scan"], "QR_SCAN")
        class_a, class_b, landing_side = self._parse_qr()
        rospy.logwarn("[CRAIC mission] QR result: class_a=%s class_b=%s landing=%s", class_a, class_b, landing_side)

        for index, point in enumerate(self._orbit_waypoints()):
            if not self._go_to(point, "ORBIT_%02d" % index):
                self._publish_takeoff_land(2)
                return

        if not self._drop_at("image_target_a", 1, "IMAGE_TARGET_A_" + class_a, expected_class=class_a):
            self._publish_takeoff_land(2)
            return
        if not self._drop_at("image_target_b", 2, "IMAGE_TARGET_B_" + class_b, expected_class=class_b):
            self._publish_takeoff_land(2)
            return
        if not self._drop_at("special_target", 3, "SPECIAL_TARGET"):
            self._publish_takeoff_land(2)
            return

        self._go_to(self.points["ring_pre"], "RING_PRE")
        ring_center = self.points["ring_center"]
        if self.latest_ring_pose is not None:
            p = self.latest_ring_pose.pose.position
            ring_center = (p.x, p.y, p.z if p.z > 0.2 else self.ring_height)
            rospy.logwarn("[CRAIC mission] using detected ring center: %.2f %.2f %.2f", ring_center[0], ring_center[1], ring_center[2])
        self._go_to(ring_center, "RING_CENTER")
        self._go_to(self.points["ring_post"], "RING_POST")

        landing_key = "landing_left" if landing_side == "left" else "landing_right"
        landing = self.points[landing_key]
        approach = (landing[0], landing[1], self.land_approach_height)
        self._go_to(approach, "LAND_APPROACH_" + landing_side.upper())

        self._set_state("LAND")
        self._publish_takeoff_land(2)
        rospy.sleep(self.landing_wait_time)
        self._set_state("DONE")


if __name__ == "__main__":
    rospy.init_node("craic_mission_fsm")
    node = CraicMissionFSM()
    node.run()
