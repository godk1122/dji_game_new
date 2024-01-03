#! /usr/bin/python3
import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
import numpy as np


class TFPublisher:
    def __init__(self):
        rospy.init_node("tf_publisher")
        self.msg_cnt = 0
        self.msg_cnt_time0 = -1

    def run(self):
        rospy.loginfo("Start publishing tf")
        rospy.Subscriber(
            "pose", PoseStamped, self.pose_gt_callback
        )
        self.br = tf2_ros.TransformBroadcaster()

        rospy.spin()

    def pose_gt_callback(self, msg):
        if self.msg_cnt_time0 == -1:
            self.msg_cnt_time0 = rospy.Time.now().to_sec()

        self.msg_cnt += 1

        if self.msg_cnt == 100:
            rospy.loginfo(
                "TF Publisher Received %d messages, current frequency is %f Hz"
                % (
                    self.msg_cnt,
                    self.msg_cnt
                    / (0.01 + (rospy.Time.now().to_sec() - self.msg_cnt_time0)),
                )
            )
            self.msg_cnt = 0
            self.msg_cnt_time0 = rospy.Time.now().to_sec()

        t = TransformStamped()

        t.header.stamp = msg.header.stamp
        t.header.frame_id = "airsim_world"
        t.child_frame_id = "quadcopter"
        t.transform.translation = msg.pose.position
        
        t.transform.rotation = msg.pose.orientation

        self.br.sendTransform(
            t
        )  # Callback function to handle incoming PoseStamped messages
        # Publish the tf based on the received pose_gt message


if __name__ == "__main__":
    tf_publisher = TFPublisher()
    tf_publisher.run()
