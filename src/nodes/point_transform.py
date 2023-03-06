#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Point
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

        self.extreme_points_transformed_pub = rospy.Publisher("extreme_points", ExtremePoints, queue_size=1)

        self.count = 0
        self.rightmost_point = Point()
        self.leftmost_point = Point()
        self.rate = rospy.Rate(25)
    
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
            extreme_points_transformed = ExtremePoints()
            extreme_points_transformed.leftmost = leftmost_transformed
            extreme_points_transformed.rightmost = rightmost_transformed
            self.extreme_points_transformed_pub.publish(extreme_points_transformed)


def main():
    rospy.init_node('image_processor', anonymous=True)
    point_transform = PointTransform()

    rospy.spin()

if __name__ == '__main__':
    main()