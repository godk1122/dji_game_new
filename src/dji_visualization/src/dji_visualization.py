#! /usr/bin/python3 

import rospy
from airsim_ros.msg import CirclePoses
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import tf_conversions
from numpy import pi
print("import finished")
def deg2rad(deg):
    return deg * pi / 180.0


class DJI_Visualization:
    def __init__(self) -> None:
        rospy.init_node('dji_visualization_node', anonymous=True)
        rospy.Subscriber('/airsim_node/drone_1/debug/circle_poses_gt', CirclePoses, self.circle_poses_callback)
        self.marker_pub = rospy.Publisher('/circle_markers', MarkerArray, queue_size=10)
        rospy.spin()

    def circle_poses_callback(self, data):
        # Print all the information
        # print("Circle Poses:")
        # for circle_pose in data.poses:
            # print(f"Pose:{circle_pose.yaw}")
            # print("")

        # Create marker array
        marker_array = MarkerArray()

        # Publish markers for each circle pose
        for i, circle_pose in enumerate(data.poses):
            marker = Marker()
            marker.header.frame_id = "airsim_world"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "circles"
            marker.id = i
            marker.type = Marker.MESH_RESOURCE
            marker.action = Marker.ADD
            marker.mesh_resource = "package://dji_visualization/meshes/ring.stl"
            marker.pose.position.x = circle_pose.position.x
            marker.pose.position.y = circle_pose.position.y
            marker.pose.position.z = circle_pose.position.z
            q=tf_conversions.transformations.quaternion_from_euler(0,0,deg2rad(circle_pose.yaw),axes='sxyz')
            marker.pose.orientation.x = q[0]
            marker.pose.orientation.y = q[1]
            marker.pose.orientation.z = q[2]
            marker.pose.orientation.w = q[3]
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)

        # Publish the marker array
        self.marker_pub.publish(marker_array)


if __name__ == '__main__':
    dji_visualization = DJI_Visualization()
    
    