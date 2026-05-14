#!/usr/bin/env python3
import sys
import threading
import time
from typing import List

import rospy
from std_msgs.msg import Int32, String


class DropSequenceTester:
    def __init__(self) -> None:
        self.drop_ids = self._parse_drop_ids(rospy.get_param("~drop_ids", [1, 2, 3]))
        self.interval = max(0.0, float(rospy.get_param("~interval", 1.0)))
        self.timeout = max(0.0, float(rospy.get_param("~timeout", 3.0)))

        self._condition = threading.Condition()
        self._statuses = set()

        self.drop_pub = rospy.Publisher("/craic/drop_cmd", Int32, queue_size=10)
        rospy.Subscriber("/craic/drop_status", String, self._status_cb, queue_size=10)

    @staticmethod
    def _parse_drop_ids(raw_value) -> List[int]:
        if isinstance(raw_value, str):
            text = raw_value.strip()
            if text.startswith("[") and text.endswith("]"):
                text = text[1:-1]
            values = [item.strip() for item in text.split(",") if item.strip()]
        elif isinstance(raw_value, (list, tuple)):
            values = raw_value
        else:
            values = [raw_value]

        drop_ids = [int(value) for value in values]
        if not drop_ids:
            raise ValueError("~drop_ids must contain at least one drop id")
        return drop_ids

    def _status_cb(self, msg: String) -> None:
        status = str(msg.data).strip()
        with self._condition:
            self._statuses.add(status)
            self._condition.notify_all()

    def _wait_for_ack(self, expected_ack: str) -> bool:
        deadline = time.monotonic() + self.timeout
        with self._condition:
            while not rospy.is_shutdown():
                if expected_ack in self._statuses:
                    return True

                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return False

                self._condition.wait(timeout=remaining)

        return False

    def run(self) -> int:
        time.sleep(0.2)
        rospy.loginfo(
            "drop sequence test: drop_ids=%s interval=%.2fs timeout=%.2fs",
            self.drop_ids,
            self.interval,
            self.timeout,
        )

        for index, drop_id in enumerate(self.drop_ids):
            expected_ack = "ACK:DROP:%d" % drop_id

            with self._condition:
                self._statuses.clear()

            self.drop_pub.publish(Int32(data=drop_id))
            rospy.loginfo("sent DROP:%d", drop_id)

            if not self._wait_for_ack(expected_ack):
                rospy.logerr("timeout waiting for %s", expected_ack)
                return 1

            rospy.loginfo("got %s", expected_ack)

            if index < len(self.drop_ids) - 1 and self.interval > 0.0:
                time.sleep(self.interval)

        rospy.loginfo("drop sequence test complete")
        return 0


def main() -> int:
    rospy.init_node("test_drop_sequence", anonymous=True)
    try:
        tester = DropSequenceTester()
    except (TypeError, ValueError) as exc:
        rospy.logerr("invalid drop sequence parameter: %s", exc)
        return 2
    return tester.run()


if __name__ == "__main__":
    sys.exit(main())
