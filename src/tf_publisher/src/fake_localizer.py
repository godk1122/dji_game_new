#! /usr/bin/python3
#  /airsim_node/drone_1/imu/imu  : sensor_msgs/Imu

# /airsim_node/drone_1/debug/pose_gt   : geometry_msgs/PoseStamped   
import rospy
import message_filters
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped

def callback(imu_msg:Imu, pose_msg:PoseStamped):
    # Store the position from the pose_gt message
    pose_msg.pose.orientation=imu_msg.orientation
    pose_msg.header.stamp=imu_msg.header.stamp

    # Publish the fake localization topic with the stored orientation and position
    fake_localization_pub.publish(pose_msg)

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('fake_localizer')

    # Subscribe to the imu topic
    imu_sub = message_filters.Subscriber('imu', Imu)

    # Subscribe to the pose_gt topic
    pose_sub = message_filters.Subscriber('gt', PoseStamped)

    # Synchronize the messages based on their timestamps
    ts = message_filters.ApproximateTimeSynchronizer([imu_sub, pose_sub], queue_size=100, slop=0.2)
    ts.registerCallback(callback)

    # Create a publisher for the fake_localization topic
    fake_localization_pub = rospy.Publisher('fake_localization', PoseStamped, queue_size=10)

    # Spin the ROS node
    rospy.spin()
