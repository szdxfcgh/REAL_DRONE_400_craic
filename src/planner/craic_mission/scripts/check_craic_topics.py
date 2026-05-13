#!/usr/bin/env python3
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import rosgraph
import rospy
import rostopic


TOPICS = [
    "/Odom_high_freq",
    "/move_base_simple/goal",
    "/drone_0_planning/bspline",
    "/position_cmd",
    "/craic/qr_result",
    "/craic/target_detected",
    "/craic/ring_pose",
    "/craic/special_target",
    "/craic/drop_cmd",
    "/craic/drop_status",
    "/cloud_registered",
    "/camera/depth/image_rect_raw",
    "/vins_fusion/extrinsic",
    "/px4ctrl/takeoff_land",
]

EXPECTED_TOPIC_TYPES = {
    "/craic/drop_status": "std_msgs/String",
    "/craic/special_target": "std_msgs/String",
}

WARN_IF_MISSING_TOPICS = {
    "/craic/drop_status",
}


@dataclass
class TopicSample:
    received: bool = False
    count: int = 0
    first_time: Optional[float] = None
    last_time: Optional[float] = None

    def mark(self) -> None:
        now = time.monotonic()
        if self.first_time is None:
            self.first_time = now
        self.last_time = now
        self.count += 1
        self.received = True

    def hz(self) -> Optional[float]:
        if self.count < 2 or self.first_time is None or self.last_time is None:
            return None
        span = self.last_time - self.first_time
        if span <= 0.0:
            return None
        return float(self.count - 1) / span


@dataclass
class TopicInfo:
    topic: str
    msg_type: str = "unknown"
    expected_msg_type: Optional[str] = None
    warn_if_missing: bool = False
    publishers: List[str] = field(default_factory=list)
    subscribers: List[str] = field(default_factory=list)
    sample: TopicSample = field(default_factory=TopicSample)

    def exists(self) -> bool:
        return bool(self.publishers or self.subscribers or self.msg_type != "unknown")

    def has_type_mismatch(self) -> bool:
        return self.expected_msg_type is not None and self.msg_type not in (
            "unknown",
            self.expected_msg_type,
        )

    def status(self) -> str:
        if self.has_type_mismatch():
            return "FAIL"
        if self.publishers or self.sample.received:
            return "OK"
        if self.exists():
            return "WARN"
        if self.warn_if_missing:
            return "WARN"
        return "FAIL"


class CraicTopicChecker:
    def __init__(self) -> None:
        self.timeout_sec = float(rospy.get_param("~timeout_sec", 2.0))
        self.check_hz = bool(rospy.get_param("~check_hz", False))
        self.verbose = bool(rospy.get_param("~verbose", False))
        self.master = rosgraph.Master(rospy.get_name())
        self.infos: Dict[str, TopicInfo] = {
            topic: TopicInfo(
                topic=topic,
                expected_msg_type=EXPECTED_TOPIC_TYPES.get(topic),
                warn_if_missing=topic in WARN_IF_MISSING_TOPICS,
            )
            for topic in TOPICS
        }
        self.subscribers = []

    def _refresh_master_state(self) -> None:
        publishers, subscribers, _services = self.master.getSystemState()
        topic_types = dict(self.master.getPublishedTopics(""))

        pub_map = {topic: nodes for topic, nodes in publishers}
        sub_map = {topic: nodes for topic, nodes in subscribers}

        for topic, info in self.infos.items():
            info.publishers = pub_map.get(topic, [])
            info.subscribers = sub_map.get(topic, [])
            info.msg_type = topic_types.get(topic, "unknown")

    def _message_cb(self, _msg, topic: str) -> None:
        self.infos[topic].sample.mark()

    def _subscribe_for_samples(self) -> None:
        for topic in TOPICS:
            try:
                msg_class, _real_topic, _eval_fn = rostopic.get_topic_class(topic, blocking=False)
            except Exception as exc:
                rospy.logwarn("Could not resolve topic class for %s: %s", topic, exc)
                continue

            if msg_class is None:
                continue

            self.subscribers.append(
                rospy.Subscriber(topic, msg_class, self._message_cb, callback_args=topic, queue_size=1)
            )

    def _wait_for_samples(self) -> None:
        end_time = time.monotonic() + max(self.timeout_sec, 0.0)
        while not rospy.is_shutdown() and time.monotonic() < end_time:
            rospy.sleep(0.05)

    def _format_line(self, info: TopicInfo) -> str:
        msg_seen = "yes" if info.sample.received else "no"
        parts = [
            "%-4s" % info.status(),
            info.topic,
            "type=%s" % info.msg_type,
            "pub=%d" % len(info.publishers),
            "sub=%d" % len(info.subscribers),
            "msg=%s" % msg_seen,
        ]

        if self.check_hz:
            hz = info.sample.hz()
            parts.append("hz=%s" % ("n/a" if hz is None else "%.2f" % hz))

        return " | ".join(parts)

    def run(self) -> int:
        self._refresh_master_state()
        self._subscribe_for_samples()
        self._wait_for_samples()
        self._refresh_master_state()

        counts = {"OK": 0, "WARN": 0, "FAIL": 0}
        print("CRAIC topic health check timeout=%.1fs" % self.timeout_sec)
        for topic in TOPICS:
            info = self.infos[topic]
            counts[info.status()] += 1
            print(self._format_line(info))
            if self.verbose:
                print("  publishers: %s" % (", ".join(info.publishers) if info.publishers else "-"))
                print("  subscribers: %s" % (", ".join(info.subscribers) if info.subscribers else "-"))

        print("summary: OK=%d WARN=%d FAIL=%d" % (counts["OK"], counts["WARN"], counts["FAIL"]))
        return 0


if __name__ == "__main__":
    rospy.init_node("check_craic_topics", anonymous=True)
    sys.exit(CraicTopicChecker().run())
