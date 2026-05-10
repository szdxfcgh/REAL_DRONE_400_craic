#!/usr/bin/env python3
import time
from typing import Dict, Optional

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class K230StatusMonitor:
    def __init__(self) -> None:
        self.raw_timeout_sec = float(rospy.get_param("~raw_timeout_sec", 2.0))
        self.qr_timeout_sec = float(rospy.get_param("~qr_timeout_sec", 5.0))
        self.target_timeout_sec = float(rospy.get_param("~target_timeout_sec", 3.0))
        self.ring_timeout_sec = float(rospy.get_param("~ring_timeout_sec", 5.0))
        self.publish_hz = float(rospy.get_param("~publish_hz", 2.0))
        if self.publish_hz <= 0.0:
            self.publish_hz = 2.0

        self.last_seen: Dict[str, Optional[float]] = {
            "raw": None,
            "qr": None,
            "target": None,
            "ring": None,
        }

        self.status_pub = rospy.Publisher("/craic/k230/status", String, queue_size=1)

        rospy.Subscriber("/craic/k230/raw", String, self._raw_cb, queue_size=10)
        rospy.Subscriber("/craic/qr_result", String, self._qr_cb, queue_size=10)
        rospy.Subscriber("/craic/target_detected", String, self._target_cb, queue_size=10)
        rospy.Subscriber("/craic/ring_pose", PoseStamped, self._ring_cb, queue_size=10)

    def _mark(self, key: str) -> None:
        self.last_seen[key] = time.monotonic()

    def _raw_cb(self, _msg: String) -> None:
        self._mark("raw")

    def _qr_cb(self, _msg: String) -> None:
        self._mark("qr")

    def _target_cb(self, _msg: String) -> None:
        self._mark("target")

    def _ring_cb(self, _msg: PoseStamped) -> None:
        self._mark("ring")

    @staticmethod
    def _fresh(last_seen: Optional[float], now: float, timeout_sec: float) -> bool:
        return last_seen is not None and (now - last_seen) <= timeout_sec

    @staticmethod
    def _age_text(last_seen: Optional[float], now: float) -> str:
        if last_seen is None:
            return "never"
        return "%.2fs" % max(now - last_seen, 0.0)

    def _build_status(self) -> str:
        now = time.monotonic()
        parsed_ok = (
            self._fresh(self.last_seen["qr"], now, self.qr_timeout_sec)
            and self._fresh(self.last_seen["target"], now, self.target_timeout_sec)
            and self._fresh(self.last_seen["ring"], now, self.ring_timeout_sec)
        )

        codes = []

        raw_last = self.last_seen["raw"]
        if raw_last is None:
            if parsed_ok:
                codes.append("WARN_NO_RAW_BUT_PARSED_OK")
            else:
                codes.append("NO_RAW_DATA")
        elif (now - raw_last) > self.raw_timeout_sec:
            codes.append("STALE_RAW")

        checks = (
            ("qr", self.qr_timeout_sec, "NO_QR", "STALE_QR"),
            ("target", self.target_timeout_sec, "NO_TARGET", "STALE_TARGET"),
            ("ring", self.ring_timeout_sec, "NO_RING", "STALE_RING"),
        )
        for key, timeout_sec, no_code, stale_code in checks:
            last_seen = self.last_seen[key]
            if last_seen is None:
                codes.append(no_code)
            elif (now - last_seen) > timeout_sec:
                codes.append(stale_code)

        if not codes:
            codes.append("OK")
        elif codes == ["WARN_NO_RAW_BUT_PARSED_OK"]:
            codes.insert(0, "OK")

        ages = "raw_age=%s qr_age=%s target_age=%s ring_age=%s" % (
            self._age_text(self.last_seen["raw"], now),
            self._age_text(self.last_seen["qr"], now),
            self._age_text(self.last_seen["target"], now),
            self._age_text(self.last_seen["ring"], now),
        )
        return "%s %s" % (",".join(codes), ages)

    def run(self) -> None:
        rate = rospy.Rate(self.publish_hz)
        rospy.loginfo(
            "k230_status_monitor started: raw_timeout=%.2fs qr_timeout=%.2fs target_timeout=%.2fs ring_timeout=%.2fs",
            self.raw_timeout_sec,
            self.qr_timeout_sec,
            self.target_timeout_sec,
            self.ring_timeout_sec,
        )

        while not rospy.is_shutdown():
            self.status_pub.publish(String(data=self._build_status()))
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("k230_status_monitor")
    try:
        K230StatusMonitor().run()
    except rospy.ROSInterruptException:
        pass
