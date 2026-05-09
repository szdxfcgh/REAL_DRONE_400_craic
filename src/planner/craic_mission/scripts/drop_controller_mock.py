#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32, String


class DropControllerMock:
    def __init__(self) -> None:
        self.drop_cmd_topic = rospy.get_param("~drop_cmd_topic", "/craic/drop_cmd")
        self.drop_done_topic = rospy.get_param("~drop_done_topic", "/craic/drop_done")
        self.actuation_time = float(rospy.get_param("~actuation_time", 0.5))
        self.done_pub = rospy.Publisher(self.drop_done_topic, String, queue_size=1)
        rospy.Subscriber(self.drop_cmd_topic, Int32, self._drop_cb, queue_size=5)

    def _drop_cb(self, msg: Int32) -> None:
        rospy.logwarn("[drop mock] received drop command: %d", msg.data)
        rospy.sleep(self.actuation_time)
        self.done_pub.publish(String(data="drop_%d_done" % msg.data))


if __name__ == "__main__":
    rospy.init_node("drop_controller_mock")
    DropControllerMock()
    rospy.spin()
