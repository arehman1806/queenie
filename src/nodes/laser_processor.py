#!/usr/bin/env python3
import rospy
import numpy as np

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import LaserScan

rospy.init_node('laser_processor')



# define callback functions to handle the contact messages
def laserscan_callback(data):
    scan = np.array(data.ranges)
    scan = np.nan_to_num(scan)
    scan = np.clip(scan, data.range_min, data.range_max)
    scan = scan[14:len(scan)-14]
    # print(len(scan))
    # print(scan)
    
    processed = Float64MultiArray()
    processed.data = scan
    processed_pub.publish(processed)

# Publish processed laser ranges
processed_pub = rospy.Publisher("camera/processed_laser_scan", Float64MultiArray, queue_size=1)

# subscribe to the contact topics
rospy.Subscriber('camera/laser_scan', LaserScan, laserscan_callback)

# start the node
rospy.spin()
