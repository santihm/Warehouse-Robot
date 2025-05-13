#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def publish_markers():
    rospy.init_node("marker_publisher")
    pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)

    marker = Marker()
    marker.header.frame_id = "map"
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    marker.pose.position.x = 2.0
    marker.pose.position.y = 3.0
    marker.pose.position.z = 0.0
    marker.id = 1

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        marker.header.stamp = rospy.Time.now()
        pub.publish(marker)
        rate.sleep()

if __name__ == "__main__":
    publish_markers()
