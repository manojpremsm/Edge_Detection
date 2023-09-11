#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

class PointCloud_Marker_Publisher:
    
    def __init__(self, point_cloud_topic):
        # Ros node initializer
        rospy.init_node('marker_publisher')

        # creating publisher for the marker
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

        # creating a Marker message
        self.marker = Marker()
        self.marker.header.frame_id = "root_link"  
        self.marker.type = Marker.POINTS
        self.marker.action = Marker.ADD

        #transformation between root link and frame0
        self.marker.pose.position.x = 0.5352193017116099
        self.marker.pose.position.y = -0.30108668167000013
        self.marker.pose.position.z = 0.8035482978697652
        self.marker.pose.orientation.x = -0.5266434909206594
        self.marker.pose.orientation.y = 0.8163459839777715
        self.marker.pose.orientation.z = -0.2264453001786519
        self.marker.pose.orientation.w = 0.07033973601931494
        self.marker.scale.x = 0.01  # Marker point size
        self.marker.scale.y = 0.01
        self.marker.color.g = 1.0  # Marker color (green)
        self.marker.color.a = 1.0  # Alpha (transparency)

        # Initialize the point cloud data
        self.pc_data = []

        # Subscribe to the point cloud topic
        rospy.Subscriber(point_cloud_topic, PointCloud2, self.pointcloud_callback)

    def pointcloud_callback(self, msg):
        self.pc_data = []
        for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point
            self.pc_data.append((x, y, z))

    def publish_marker(self):
        rate = rospy.Rate(20)  

        while not rospy.is_shutdown():
            # header time stamp set to current time 
            self.marker.header.stamp = rospy.Time.now()

            # populating marker with points from point cloud data
            self.marker.points = [Point(x, y, z) for x, y, z in self.pc_data]

            # Publish the marker
            self.marker_pub.publish(self.marker)
            print("i am being published ")
            rate.sleep()

if __name__ == '__main__':
    try:
        
        

        marker_publisher = PointCloud_Marker_Publisher('/camera/depth_registered/points')
        marker_publisher.publish_marker()
    except rospy.ROSInterruptException:
        pass
