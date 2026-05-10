#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


class FakePlannerInputs:
    def __init__(self) -> None:
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.publish_hz = float(rospy.get_param("~publish_hz", 10.0))
        self.cloud_topic = rospy.get_param("~cloud_topic", "/cloud_registered")
        self.extrinsic_topic = rospy.get_param("~extrinsic_topic", "/vins_fusion/extrinsic")
        if self.publish_hz <= 0.0:
            self.publish_hz = 10.0

        self.cloud_pub = rospy.Publisher(self.cloud_topic, PointCloud2, queue_size=1)
        self.extrinsic_pub = rospy.Publisher(self.extrinsic_topic, Odometry, queue_size=1)

    def _make_cloud(self) -> PointCloud2:
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.frame_id
        return point_cloud2.create_cloud_xyz32(header, [])

    def _make_extrinsic(self) -> Odometry:
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = "camera"
        msg.pose.pose.orientation.w = 1.0
        return msg

    def run(self) -> None:
        rate = rospy.Rate(self.publish_hz)
        rospy.logwarn(
            "[fake planner inputs] publishing %s and %s at %.1f Hz in frame %s",
            self.cloud_topic,
            self.extrinsic_topic,
            self.publish_hz,
            self.frame_id,
        )
        while not rospy.is_shutdown():
            self.cloud_pub.publish(self._make_cloud())
            self.extrinsic_pub.publish(self._make_extrinsic())
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("fake_planner_inputs")
    FakePlannerInputs().run()
