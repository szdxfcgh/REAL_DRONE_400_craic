#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32, String


class FakeDropController:
    VALID_DROP_IDS = (1, 2, 3)
    VALID_FAIL_MODES = ("busy", "timeout", "invalid")

    def __init__(self) -> None:
        self.ack_delay = max(0.0, float(rospy.get_param("~ack_delay", 0.3)))
        self.fail_drop_id = int(rospy.get_param("~fail_drop_id", 0))
        self.fail_mode = str(rospy.get_param("~fail_mode", "busy")).strip().lower()
        if self.fail_mode not in self.VALID_FAIL_MODES:
            rospy.logwarn(
                "[fake drop] unsupported fail_mode %r, falling back to busy",
                self.fail_mode,
            )
            self.fail_mode = "busy"

        self.status_pub = rospy.Publisher("/craic/drop_status", String, queue_size=10)
        rospy.Subscriber("/craic/drop_cmd", Int32, self._drop_cb, queue_size=10)

        rospy.loginfo(
            "[fake drop] ready: ack_delay=%.3f fail_drop_id=%d fail_mode=%s",
            self.ack_delay,
            self.fail_drop_id,
            self.fail_mode,
        )

    def _drop_cb(self, msg: Int32) -> None:
        drop_id = int(msg.data)
        rospy.loginfo("[fake drop] received drop command: %d", drop_id)

        if drop_id not in self.VALID_DROP_IDS:
            self._publish_status("ERR:INVALID")
            return

        if drop_id == self.fail_drop_id:
            self._handle_injected_failure(drop_id)
            return

        self._schedule_status("ACK:DROP:%d" % drop_id)

    def _handle_injected_failure(self, drop_id: int) -> None:
        if self.fail_mode == "timeout":
            rospy.logwarn("[fake drop] simulating timeout for DROP:%d", drop_id)
            return
        if self.fail_mode == "invalid":
            self._schedule_status("ERR:INVALID")
            return
        self._schedule_status("ERR:BUSY")

    def _schedule_status(self, status: str) -> None:
        if self.ack_delay <= 0.0:
            self._publish_status(status)
            return

        rospy.Timer(
            rospy.Duration(self.ack_delay),
            lambda _event: self._publish_status(status),
            oneshot=True,
        )

    def _publish_status(self, status: str) -> None:
        if rospy.is_shutdown():
            return
        rospy.loginfo("[fake drop] publishing status: %s", status)
        self.status_pub.publish(String(data=status))


if __name__ == "__main__":
    rospy.init_node("fake_drop_controller")
    FakeDropController()
    rospy.spin()
