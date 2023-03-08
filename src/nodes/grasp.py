#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Pose
from queenie.msg import ExtremePoints
from std_msgs.msg import Float64, Float32, Header, Float64MultiArray, Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time
import math
import PyKDL
import copy
import numpy as np

class GraspObject(object):

    def __init__(self) -> None:

        self.distance_threshold_explore = 3
        self.go_around_distance_threshold = 3
        self.min_distance_to_handle_threshold = 5

        self.trajectory_publisher = rospy.Publisher('/queenie/head_controller/command', JointTrajectory, queue_size=1)
        self.gripper_controller = rospy.Publisher("/queenie/gripper_controller/command", Float64MultiArray, queue_size=1)

        self.grasp_topic = rospy.Subscriber("/grasp_object", Bool, self.grasp_topic_cb)
        self.rate = rospy.Rate(25)
        self.changing_position = False
        self.position_to_maintain = [0, 0]
    

    
    def grasp_topic_cb(self, data:Bool):
        if data.data:
            self.grasp()
        else:
            self.ungrasp()


    def maintain_position(self):
        while not rospy.is_shutdown():
            if self.changing_position:
                self.rate.sleep()
                continue
            msg = self.create_msg(self.position_to_maintain)
            self.trajectory_publisher.publish(msg)
            self.rate.sleep()
    
    def grasp(self):
        self.changing_position = True
        msg = Float64MultiArray()
        msg.data = [-50.00, -50.00]
        self.gripper_controller.publish(msg)
        rospy.sleep(3)
        self.trajectory_publisher.publish(self.create_msg([0.2,0.17, 0,0], 5))
        rospy.sleep(5)
        self.trajectory_publisher.publish(self.create_msg([-0.36,0.15, 0,0], 5))
        rospy.sleep(5)
        self.changing_position = False
        self.position_to_maintain = [-0.36,0.15, 0,0]
    
    def ungrasp(self):
        self.changing_position = True
        self.trajectory_publisher.publish(self.create_msg([0.2,0, 0,0], 5))
        rospy.sleep(5)
        msg = Float64MultiArray()
        msg.data = [5, 5]
        self.gripper_controller.publish(msg)
        rospy.sleep(3)
        self.trajectory_publisher.publish(self.create_msg([0,0,0,0], 5))
        rospy.sleep(5)
        self.changing_position = False
        self.position_to_maintain = [0,0, 0,0]



    def create_msg(self, positions, duration=0.1):
        msg = JointTrajectory()
        msg.joint_names = ["neck", "palm_riser", "palm_left_finger", "palm_right_finger"][0:2]
        msg.points=[JointTrajectoryPoint()]
        msg.header =Header()
        msg.points[0].positions = positions[0:2]
        msg.points[0].time_from_start = rospy.Duration.from_sec(duration)
        return msg
            
    
    

if __name__ == "__main__":
    rospy.init_node("explore")
    g = GraspObject()
    g.maintain_position()
    rospy.spin()