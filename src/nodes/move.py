#!/usr/bin/env python3
import rospy
import math
import tf2_ros
import geometry_msgs.msg
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    print(rospy.get_caller_id() + "I heard %s", data.data)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('queenie_tf_listener', anonymous=True)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    queenie_driver = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)

    while not rospy.is_shutdown():
        try:
            # print(tfBuffer.lookup_transform("odom", "robot_footprint", rospy.Time()))
            2+2
        except:
            print("exception occurred")
    # spin() simply keeps python from exiting until this node is stopped
        msg = geometry_msgs.msg.Twist()

        msg.angular.z = 4 * math.atan2(4, 2)
        msg.linear.x = 0.5 * math.sqrt(4 ** 2 + 4 ** 2)

        # queenie_driver.publish(msg)
        print(msg)
        
    rospy.spin()

if __name__ == '__main__':
    print("node started")
    listener()