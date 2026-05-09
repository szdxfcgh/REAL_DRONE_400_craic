#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class FakeK230Publisher:
    def __init__(self) -> None:
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.qr_text = rospy.get_param("~qr_text", "man,apple,left")
        self.target_a = rospy.get_param("~target_a", "man")
        self.target_b = rospy.get_param("~target_b", "apple")
        self.landing_side = rospy.get_param("~landing_side", "left")
        self.ring_x = float(rospy.get_param("~ring_x", 7.5))
        self.ring_y = float(rospy.get_param("~ring_y", 0.0))
        self.ring_z = float(rospy.get_param("~ring_z", 1.6))
        self.publish_hz = float(rospy.get_param("~publish_hz", 2.0))
        if self.publish_hz <= 0.0:
            self.publish_hz = 2.0

        self.qr_pub = rospy.Publisher("/craic/qr_result", String, queue_size=1, latch=True)
        self.target_pub = rospy.Publisher("/craic/target_detected", String, queue_size=10)
        self.ring_pub = rospy.Publisher("/craic/ring_pose", PoseStamped, queue_size=1, latch=True)
        self.special_pub = rospy.Publisher("/craic/special_target", String, queue_size=1, latch=True)
        self.landing_pub = rospy.Publisher("/craic/landing_marker", String, queue_size=1, latch=True)
        self.target_index = 0

    def _ring_pose(self) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = self.ring_x
        msg.pose.position.y = self.ring_y
        msg.pose.position.z = self.ring_z
        msg.pose.orientation.w = 1.0
        return msg

    def _target_msg(self) -> str:
        if self.target_index % 2 == 0:
            data = "%s,0.870,320.0,240.0" % self.target_a
        else:
            data = "%s,0.820,300.0,250.0" % self.target_b
        self.target_index += 1
        return data

    def run(self) -> None:
        rate = rospy.Rate(self.publish_hz)
        rospy.logwarn(
            "[fake k230] qr=%s target_a=%s target_b=%s landing=%s ring=(%.2f, %.2f, %.2f)",
            self.qr_text,
            self.target_a,
            self.target_b,
            self.landing_side,
            self.ring_x,
            self.ring_y,
            self.ring_z,
        )

        while not rospy.is_shutdown():
            self.qr_pub.publish(String(data=self.qr_text))
            self.target_pub.publish(String(data=self._target_msg()))
            self.ring_pub.publish(self._ring_pose())
            self.special_pub.publish(String(data="0.90,320,240"))
            self.landing_pub.publish(String(data="%s,0.0,0.0,0.90" % self.landing_side))
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("fake_k230_publisher")
    FakeK230Publisher().run()
