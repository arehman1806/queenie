#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Point, PointStamped
from queenie.msg import ExtremePoints
import tf2_ros
import tf2_geometry_msgs    
import time


class PointTransform:
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()  # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # self.pub = rospy.Publisher('/reward_signal', Float32, queue_size=10)
        self.extreme_points_seb = rospy.Subscriber("extreme_points_camera_frame", ExtremePoints, self.extreme_points_cb)
        self.handle_centroid_sub = rospy.Subscriber("/handle_centroid", PointStamped, self.handle_centroid_cb)

        self.extreme_points_transformed_pub = rospy.Publisher("extreme_points", ExtremePoints, queue_size=1)
        self.handle_centroid_pub = rospy.Publisher("handle_centroid_transformed", PointStamped, queue_size=1)

        self.count = 0
        self.rightmost_point = Point()
        self.leftmost_point = Point()
        self.rate = rospy.Rate(25)
    
    def handle_centroid_cb(self, data):
        x = 0
        while x < 5:
            try:
                trans = self.tf_buffer.lookup_transform("odom", "camera_optical", rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.rate.sleep()
                continue
            x += 1
            handle_centroid_transformed = tf2_geometry_msgs.do_transform_point(data, trans)
            self.handle_centroid_pub.publish(handle_centroid_transformed)

    
    def extreme_points_cb(self, data:ExtremePoints):
        x = 0
        while x < 5:
            try:
                trans = self.tf_buffer.lookup_transform("odom", "camera_optical", rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.rate.sleep()
                continue
            x += 1

            rightmost_transformed = tf2_geometry_msgs.do_transform_point(data.rightmost, trans)
            leftmost_transformed = tf2_geometry_msgs.do_transform_point(data.leftmost, trans)
            object_centroid_transformed = tf2_geometry_msgs.do_transform_point(data.point_centroid, trans)
            extreme_points_transformed = ExtremePoints()
            extreme_points_transformed.leftmost = leftmost_transformed
            extreme_points_transformed.rightmost = rightmost_transformed
            extreme_points_transformed.point_centroid = object_centroid_transformed
            self.extreme_points_transformed_pub.publish(extreme_points_transformed)


def main():
    rospy.init_node('point_transform', anonymous=True)
    point_transform = PointTransform()

    rospy.spin()

if __name__ == '__main__':
    main()
