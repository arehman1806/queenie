#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Int32
import struct

# Moving average filter class
class MovingAverageFilter:
    def __init__(self, window_size):
        self.window_size_ = window_size
        self.data_ = [0] * window_size
        self.sum_ = 0
        self.index_ = 0

    def addData(self, value):
        self.sum_ -= self.data_[self.index_]
        self.sum_ += value
        self.data_[self.index_] = value
        self.index_ = (self.index_ + 1) % self.window_size_

    def getAverage(self):
        return self.sum_ / self.window_size_

# Point cloud callback function with moving average filter and centroid computation
# Point cloud callback function with moving average filter and centroid computation
def pointcloud_callback(msg):
    global centroid_pub, num_clouds_pub

    filter_size = 5
    static_filter = MovingAverageFilter(filter_size)

    num_points = msg.width * msg.height

    if num_points > 0:
        # Calculate average point distance and add to filter
        avg_dist = 0
        x_sum = 0
        y_sum = 0
        z_sum = 0
        for i in range(0, len(msg.data), 12):
            if len(msg.data) < i + 12:
                rospy.logerr('Received incomplete point')
                return

            x, y, z = struct.unpack('fff', msg.data[i:i+12])
            x_sum += x
            y_sum += y
            z_sum += z
            avg_dist += (x*x + y*y + z*z) ** 0.5
        avg_dist /= num_points
        static_filter.addData(avg_dist)

        # Compute centroid and publish
        centroid_msg = PointStamped()
        centroid_msg.header.stamp = rospy.Time.now()
        centroid_msg.header.frame_id = msg.header.frame_id
        centroid_msg.point.x = x_sum / num_points
        centroid_msg.point.y = y_sum / num_points
        centroid_msg.point.z = z_sum / num_points
        centroid_pub.publish(centroid_msg)

        # Publish number of points
        num_clouds_pub.publish(num_points)

        # Print filtered result
        rospy.loginfo("Received point cloud with %d points, average distance: %.3f", num_points, static_filter.getAverage())
    else:
        # Publish -1 for centroid
        centroid_msg = PointStamped()
        centroid_msg.header.stamp = rospy.Time.now()
        centroid_msg.header.frame_id = msg.header.frame_id
        centroid_msg.point.x = -1
        centroid_msg.point.y = -1
        centroid_msg.point.z = -1
        centroid_pub.publish(centroid_msg)

        # Publish 0 for number of points
        num_clouds_pub.publish(0)

        # Print warning message
        rospy.logwarn("Received empty point cloud")
        # rospy.logwarn("Received empty point cloud")

if __name__ == '__main__':
    rospy.init_node('pointcloud_filter')
    centroid_pub = rospy.Publisher('/pointcloud_centroid', PointStamped, queue_size=1)
    num_clouds_pub = rospy.Publisher('/pointcloud_num_points', Int32, queue_size=1)

    topic_name = '/extract_cylinder_indices/output'
    if rospy.get_param('~topic', None) is not None:
        topic_name = rospy.get_param('~topic')

    rospy.Subscriber(topic_name, PointCloud2, pointcloud_callback)
    rospy.spin()
